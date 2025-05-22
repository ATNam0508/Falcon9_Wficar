#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "motor.h"
#include "imu.h"
#include "pid.h"
#include "espnow.h"

#include "stdio.h"
#include "string.h"

static const char *TAG = "RX";

    /**
     * @brief Cấu trúc dữ liệu nhận
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
PacketData recv_data;

/**
 * @brief Callback khi nhận dữ liệu
 */
void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len != sizeof(PacketData)) {
        ESP_LOGW(TAG, "Sai kích thước gói nhận (%d bytes)", len);
        return;
    }
    memcpy(&recv_data, data, sizeof(recv_data));

}

/**
 * @brief Hàm chính
 */
void print_data(void) {
    printf("Nhận dữ liệu: ADC1 = %d, ADC2 = %d, angle_tx = %.2f, state_button_1 = %d, state_button_2 = %d, state_button_3 = %d, state_button_4 = %d\n",
           recv_data.ADC1, recv_data.ADC2, recv_data.angle_tx,
           recv_data.state_button_1, recv_data.state_button_2,
           recv_data.state_button_3, recv_data.state_button_4);
}

void app_main(void)
{   
    espnow_init(1);                         // Khởi tạo ESP-NOW với kênh 1
    espnow_register_recv_cb(on_recv);      // Đăng ký callback nhận
    motor_handler_t *motor = motor_new_handle(GPIO_NUM_18, GPIO_NUM_2, LEDC_CHANNEL_2, LEDC_TIMER_2); 

    while (1) {
        for (int duty = 0; duty < 4095; duty += 50) {
            motor_set_speed(motor, duty);
            print_data();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

           
        for (int duty = 4095; duty > 0; duty -= 50) {
            motor_set_speed(motor, duty);
            print_data();
            vTaskDelay(pdMS_TO_TICKS(10));
        }   

        
    }
}
