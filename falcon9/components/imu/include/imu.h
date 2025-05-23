#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp_raw;
} mpu6050_data_t;

/**
 * @brief Khởi tạo I2C và MPU6050, tự động hiệu chuẩn trục Z
 */
esp_err_t imu_init(i2c_port_t i2c_port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed);

/**
 * @brief Cập nhật dữ liệu IMU (đọc và xử lý)
 */
void imu_update(void);

/**
 * @brief Trả về góc quay trục Z tính được (đơn vị độ)
 */
float imu_get_angle_z(void);

/**
 * @brief Reset lại góc quay Z về 0
 */
void imu_reset_angle_z(void);

/**
 * @brief calibrate IMU
 */
void imu_calibrate(void);

#ifdef __cplusplus
}
#endif
