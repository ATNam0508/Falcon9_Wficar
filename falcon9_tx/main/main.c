#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include <stdio.h>

#include "espnow.h"
#include "esp_log.h"
#include "string.h"


#define JOYSTICK_X_GPIO ADC_CHANNEL_6  // GPIO34
#define JOYSTICK_Y_GPIO ADC_CHANNEL_7  // GPIO35

#define BUTTON1_GPIO GPIO_NUM_14
#define BUTTON2_GPIO GPIO_NUM_27
#define BUTTON3_GPIO GPIO_NUM_26
#define BUTTON4_GPIO GPIO_NUM_25

adc_oneshot_unit_handle_t adc_handle;

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

void gpio_buttons_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON1_GPIO) | (1ULL << BUTTON2_GPIO) |
                        (1ULL << BUTTON3_GPIO) | (1ULL << BUTTON4_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}


static const char *TAG = "MAIN";

// Địa chỉ MAC broadcast (gửi tới tất cả)
static uint8_t dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/**
 * @brief Cấu trúc dữ liệu gửi đi
 */
typedef struct __attribute__((packed)) {
    uint16_t ADC1;  // ADC1
    uint16_t ADC2;  // ADC2
    float angle_tx;
    bool state_button_1;
    bool state_button_2;
    bool state_button_3;
    bool state_button_4;   
} PacketData;

/**
 * @brief Callback khi gửi xong
 */
void on_send(const uint8_t *mac_addr, esp_now_send_status_t status) {

}

/**
 * @brief Callback khi nhận dữ liệu
 */
void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len != sizeof(PacketData)) {
        ESP_LOGW(TAG, "Dữ liệu nhận sai kích thước (%d bytes)", len);
        return;
    }
    PacketData recv_data;
    memcpy(&recv_data, data, sizeof(recv_data));
}

/**
 * @brief Hàm chính
 */

 int x_value = 0, y_value = 0;

void app_main(void) {
    // Khởi tạo ESP-NOW với kênh 1
    espnow_init(1);

    // Khởi tạo ADC và GPIO
    adc_init();
    gpio_buttons_init();
    
    // Đăng ký callback gửi và nhận
    espnow_register_send_cb(on_send);
    espnow_register_recv_cb(on_recv);

    while (1) {
        adc_oneshot_read(adc_handle, JOYSTICK_X_GPIO, &x_value);
        adc_oneshot_read(adc_handle, JOYSTICK_Y_GPIO, &y_value);
        PacketData data = {
            .ADC1 = x_value,
            .ADC2 = y_value,
            .angle_tx = 45.6f,
            .state_button_1 = gpio_get_level(BUTTON1_GPIO),
            .state_button_2 = gpio_get_level(BUTTON2_GPIO),
            .state_button_3 = gpio_get_level(BUTTON3_GPIO),
            .state_button_4 = gpio_get_level(BUTTON4_GPIO),
        };

        espnow_send(dest_mac, (uint8_t *)&data, sizeof(data));
        vTaskDelay(pdMS_TO_TICKS(10));  // Gửi 10hz
    }
}