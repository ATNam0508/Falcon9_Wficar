#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "motor.h"
#include "imu.h"
#include "pid.h"
#include "espnow.h"

#include "stdio.h"
#include "string.h"

#define I2C_PORT    I2C_NUM_0
#define SDA_PIN     GPIO_NUM_21
#define SCL_PIN     GPIO_NUM_22

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

pid_controller_t *pid = NULL;

/**
 * @brief Callback khi nhận dữ liệu
 */
void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    memcpy(&recv_data, data, sizeof(recv_data));
}

int pwr;
int rotate;
extern float gyro_z_offset;

/**
 * @brief Hàm chính
 */
void print_data(void) {
    // printf("Nhận dữ liệu: ADC1 = %d, ADC2 = %d, angle_tx = %.2f, state_button_1 = %d, state_button_2 = %d, state_button_3 = %d, state_button_4 = %d, state_button_5 = %d, angle_z = %f\n",
    //        recv_data.ADC1, recv_data.ADC2, recv_data.angle_tx,
    //        recv_data.state_button_1, recv_data.state_button_2,
    //        recv_data.state_button_3, recv_data.state_button_4,
    //        recv_data.state_button_5, imu_get_angle_z());
    printf("pid_output = %d", pid_compute(pid, recv_data.angle_tx, imu_get_angle_z()));
    printf("ADC1 = %d, ADC2 = %d, angle_tx = %.2f, angle_z = %f\n",
           recv_data.ADC1, pwr, recv_data.angle_tx, imu_get_angle_z());
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void app_main(void)
{   
    espnow_init(1);                         // Khởi tạo ESP-NOW với kênh 1
    espnow_register_recv_cb(on_recv);      // Đăng ký callback nhận
    motor_handler_t *motor2 = motor_new_handle(GPIO_NUM_26, GPIO_NUM_25, LEDC_CHANNEL_2, LEDC_TIMER_2); 
    motor_handler_t *motor1 = motor_new_handle(GPIO_NUM_33, GPIO_NUM_32, LEDC_CHANNEL_3, LEDC_TIMER_2); 
    motor_set_enable(motor1, 0); // Bật động cơ 1
    motor_set_enable(motor2, 0); // Bật động cơ 2

    if (imu_init(I2C_PORT, SDA_PIN, SCL_PIN, 400000) != ESP_OK) {
        printf("IMU init failed\n");
        return;
    }
    imu_calibrate(); // Hiệu chuẩn IMU

    float kp = 0.5;
    float ki = 0.25;
    float kd = 0.0001;
    float dt = 0.001;  // thời gian lấy mẫu 10ms

    pid = pid_new_handle(kp, ki, kd, dt);
    if (!pid) {
        printf("PID init failed\n");
        return;
    }
    printf("ready\n");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Đợi 10ms
    motor_set_enable(motor1, 1); // Bật động cơ 1
    motor_set_enable(motor2, 1); // Bật động cơ 2

    while (1) {
        imu_update(); // Cập nhật dữ liệu IMU
        if (recv_data.state_button_3 == 0)  {                                                    
            pwr = 3500;
        } else {
            pwr = map(recv_data.ADC2, 1650, 0, 0, 2500);
        }

        if (recv_data.state_button_1 == 0) {
            rotate -= 5;
        }
        if (recv_data.state_button_4 == 0) {
            rotate += 5;
        }

        motor_control(motor1, -pwr + (-(pid_compute(pid, (recv_data.angle_tx) * 0.65 + rotate, imu_get_angle_z() / 10)))); // Điều khiển động cơ 1
        motor_control(motor2,  pwr + (-(pid_compute(pid, (recv_data.angle_tx) * 0.65 + rotate, imu_get_angle_z() / 10)))); // Điều khiển động cơ 2
        // print_data();
        vTaskDelay(pdMS_TO_TICKS(1)); // 1000hz 
    }
}
