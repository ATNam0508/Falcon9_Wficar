#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "motor.h"
#include "imu.h"
#include "pid.h"
#include "espnow.h"

#include "stdio.h"
#include "string.h"

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
    bool state_button_5;
} PacketData;
PacketData recv_data;

/**
 * @brief Callback khi nhận dữ liệu
 */
void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    memcpy(&recv_data, data, sizeof(recv_data));
}

/**
 * @brief Hàm chính
 */
void print_data(void) {
    printf("Nhận dữ liệu: ADC1 = %d, ADC2 = %d, angle_tx = %.2f, state_button_1 = %d, state_button_2 = %d, state_button_3 = %d, state_button_4 = %d, state_button_5 = %d\n",
           recv_data.ADC1, recv_data.ADC2, recv_data.angle_tx,
           recv_data.state_button_1, recv_data.state_button_2,
           recv_data.state_button_3, recv_data.state_button_4,
           recv_data.state_button_5);
}




void app_main(void)
{   
    espnow_init(1);                         // Khởi tạo ESP-NOW với kênh 1
    espnow_register_recv_cb(on_recv);      // Đăng ký callback nhận
    motor_handler_t *motor2 = motor_new_handle(GPIO_NUM_26, GPIO_NUM_25, LEDC_CHANNEL_2, LEDC_TIMER_2); 
    motor_handler_t *motor1 = motor_new_handle(GPIO_NUM_33, GPIO_NUM_32, LEDC_CHANNEL_3, LEDC_TIMER_2); 

    while (1) {
        for (int duty = 3000; duty < 4095; duty += 25) {
            motor_set_speed(motor1, duty);
            motor_set_speed(motor2, duty);
            print_data();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

           
        for (int duty = 4095; duty > 3000; duty -= 25) {
            motor_set_speed(motor1, duty);
            motor_set_speed(motor2, duty);
            print_data();
            vTaskDelay(pdMS_TO_TICKS(10));
        }   

        
    }
}
