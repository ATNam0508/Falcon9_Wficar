#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR         0x68
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43

/**
 * @brief Structure to hold raw data from the MPU6050 sensor
 */
typedef struct {
    int16_t accel_x;     ///< Acceleration on X-axis
    int16_t accel_y;     ///< Acceleration on Y-axis
    int16_t accel_z;     ///< Acceleration on Z-axis
    int16_t gyro_x;      ///< Gyroscope on X-axis
    int16_t gyro_y;      ///< Gyroscope on Y-axis
    int16_t gyro_z;      ///< Gyroscope on Z-axis
    int16_t temp_raw;    ///< Raw temperature data
} mpu6050_data_t;

/**
 * @brief Structure to configure and access the MPU6050
 */
typedef struct {
    i2c_port_t i2c_port; ///< I2C port used
    uint8_t addr;        ///< I2C address of the MPU6050
} mpu6050_t;

/**
 * @brief Initialize the MPU6050 sensor
 * 
 * @param dev Pointer to the MPU6050 device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6050_init(mpu6050_t* dev);

/**
 * @brief Read accelerometer, gyroscope, and temperature data from MPU6050
 * 
 * @param dev Pointer to the MPU6050 device structure
 * @param data Pointer to the structure to store sensor data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6050_read(mpu6050_t* dev, mpu6050_data_t* data);

/**
 * @brief User-defined execution function to process MPU6050 data
 * 
 * @param dev Pointer to the MPU6050 device structure
 * @param data Pointer to the data structure containing the latest sensor values
 */
void mpu6050_execute(mpu6050_t* dev, mpu6050_data_t* data);

#ifdef __cplusplus
}
#endif
