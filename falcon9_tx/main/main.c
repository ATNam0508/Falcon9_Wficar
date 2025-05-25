#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/gptimer.h"

#include "espnow.h"
#include "imu.h"

#include <stdio.h>
#include <string.h>

#define I2C_PORT        I2C_NUM_0
#define SDA_PIN         GPIO_NUM_21
#define SCL_PIN         GPIO_NUM_22

#define JOYSTICK_X_GPIO ADC_CHANNEL_6   // GPIO34
#define JOYSTICK_Y_GPIO ADC_CHANNEL_7   // GPIO35

#define BUTTON1_GPIO    GPIO_NUM_14
#define BUTTON2_GPIO    GPIO_NUM_27
#define BUTTON3_GPIO    GPIO_NUM_26
#define BUTTON4_GPIO    GPIO_NUM_25
#define BUTTON5_GPIO    GPIO_NUM_33

#define TIMER_INTERVAL_US 10000         // 10ms = 100Hz

adc_oneshot_unit_handle_t adc_handle;
int x_value = 0, y_value = 0;
static uint8_t dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast

typedef struct __attribute__((packed)) {
    uint16_t ADC1;
    uint16_t ADC2;
    float angle_tx;
    bool state_button_1;
    bool state_button_2;
    bool state_button_3;
    bool state_button_4;
    bool state_button_5;
} PacketData;

/**
 * @brief Khởi tạo ADC cho joystick
 */
void adc_init(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    adc_oneshot_config_channel(adc_handle, JOYSTICK_X_GPIO, &chan_cfg);
    adc_oneshot_config_channel(adc_handle, JOYSTICK_Y_GPIO, &chan_cfg);
}

/**
 * @brief Khởi tạo GPIO cho các nút nhấn
 */
void gpio_buttons_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON1_GPIO) | (1ULL << BUTTON2_GPIO) |
                        (1ULL << BUTTON3_GPIO) | (1ULL << BUTTON4_GPIO) |
                        (1ULL << BUTTON5_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_reset_pin(GPIO_NUM_2); 
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT); // dùng làm LED/debug
}

/**
 * @brief Callback ngắt timer (100Hz)
 */
static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    return true;
}

/**
 * @brief Khởi tạo timer định kỳ 10ms (100Hz)
 */
void init_timer_100Hz() {
    gptimer_handle_t gptimer = NULL;

    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 tick = 1µs
    };
    gptimer_new_timer(&config, &gptimer);

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = timer_callback,
    };
    gptimer_register_event_callbacks(gptimer, &callbacks, NULL);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_INTERVAL_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);
    gptimer_enable(gptimer);
    gptimer_start(gptimer);
}

/**
 * @brief Callback khi gửi dữ liệu qua ESP-NOW
 */
void on_send(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Bạn có thể log ở đây nếu muốn
}

/**
 * @brief Task cập nhật dữ liệu IMU liên tục
 */
void imu_task(void *pvParameters) {
    while (1) {
        imu_update();
        vTaskDelay(pdMS_TO_TICKS(1)); // 1000Hz
    }
}

/**
 * @brief Task in giá trị góc quay trục Z và joystick
 */
void print_task(void *pvParameters) {
    while (1) {
        printf("ADC1: %d, ADC2: %d, Angle: %.2f, Buttons: [%d %d %d %d %d]\n",
               x_value, y_value, imu_get_angle_z() / 10,
               gpio_get_level(BUTTON1_GPIO),
               gpio_get_level(BUTTON2_GPIO),
               gpio_get_level(BUTTON3_GPIO),
               gpio_get_level(BUTTON4_GPIO),    
               gpio_get_level(BUTTON5_GPIO));
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

/**
 * @brief Hàm main: khởi tạo và chạy chương trình
 */

 int _cnt;

void app_main(void) {
    // 1. Khởi tạo ESP-NOW
    espnow_init(1);
    espnow_register_send_cb(on_send);

    // 2. Khởi tạo phần cứng
    adc_init();
    gpio_buttons_init();
    init_timer_100Hz();

    // 3. Khởi tạo và hiệu chuẩn IMU
    if (imu_init(I2C_PORT, SDA_PIN, SCL_PIN, 400000) != ESP_OK) {
        printf("IMU init failed\n");
        return;
    }
    imu_calibrate();

    // 4. Tạo các task FreeRTOS
    xTaskCreate(imu_task, "imu_task", 2048, NULL, 10, NULL);
    // xTaskCreate(print_task, "print_task", 2048, NULL, 5, NULL);

    // 5. Vòng lặp chính: gửi dữ liệu qua ESP-NOW
    while (1) {
        // Đọc ADC
        adc_oneshot_read(adc_handle, JOYSTICK_X_GPIO, &x_value);
        adc_oneshot_read(adc_handle, JOYSTICK_Y_GPIO, &y_value);

        // Tạo và gửi gói dữ liệu
        PacketData data = {
            .ADC1 = x_value,
            .ADC2 = y_value,
            .angle_tx = (imu_get_angle_z() / 10),
            .state_button_1 = gpio_get_level(BUTTON1_GPIO),
            .state_button_2 = gpio_get_level(BUTTON2_GPIO),
            .state_button_3 = gpio_get_level(BUTTON3_GPIO),
            .state_button_4 = gpio_get_level(BUTTON4_GPIO),
            .state_button_5 = gpio_get_level(BUTTON5_GPIO),
        };

        espnow_send(dest_mac, (uint8_t *)&data, sizeof(data));
        vTaskDelay(pdMS_TO_TICKS(10)); // gửi 100Hz
    }
}
