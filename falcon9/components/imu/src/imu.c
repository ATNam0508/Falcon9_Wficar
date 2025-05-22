#include "imu.h"
#include "esp_log.h"

#define TAG "MPU6050"

/**
 * @brief Read multiple bytes from a register
 * 
 * @param dev Pointer to the MPU6050 device structure
 * @param reg Register address to start reading from
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return esp_err_t ESP_OK on success
 */
static esp_err_t mpu6050_read_bytes(mpu6050_t* dev, uint8_t reg, uint8_t* data, size_t len) {
    return i2c_master_write_read_device(dev->i2c_port, dev->addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

/**
 * @brief Write a byte to a register
 * 
 * @param dev Pointer to the MPU6050 device structure
 * @param reg Register address to write to
 * @param value Value to write
 * @return esp_err_t ESP_OK on success
 */
static esp_err_t mpu6050_write_byte(mpu6050_t* dev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(dev->i2c_port, dev->addr, buf, 2, pdMS_TO_TICKS(100));
}

/**
 * @brief Initialize the MPU6050 sensor (check connection and wake up)
 * 
 * @param dev Pointer to the MPU6050 device structure
 * @return esp_err_t ESP_OK on success, ESP_FAIL if device not found
 */
esp_err_t mpu6050_init(mpu6050_t* dev) {
    dev->addr = MPU6050_ADDR;

    uint8_t who_am_i = 0;
    ESP_ERROR_CHECK(mpu6050_read_bytes(dev, MPU6050_WHO_AM_I_REG, &who_am_i, 1));
    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "MPU6050 not found! WHO_AM_I = 0x%02X", who_am_i);
        return ESP_FAIL;
    }

    // Wake up the MPU6050
    ESP_ERROR_CHECK(mpu6050_write_byte(dev, MPU6050_PWR_MGMT_1, 0x00));
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

/**
 * @brief Combine two bytes into a 16-bit signed integer
 * 
 * @param msb Most significant byte
 * @param lsb Least significant byte
 * @return int16_t Combined 16-bit signed integer
 */
static int16_t combine_bytes(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8) | lsb);
}

/**
 * @brief Read accelerometer, gyroscope, and temperature data
 * 
 * @param dev Pointer to the MPU6050 device structure
 * @param data Pointer to the structure to store read values
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6050_read(mpu6050_t* dev, mpu6050_data_t* data) {
    uint8_t buf[14];
    esp_err_t err = mpu6050_read_bytes(dev, MPU6050_ACCEL_XOUT_H, buf, 14);
    if (err != ESP_OK) return err;

    data->accel_x = combine_bytes(buf[0], buf[1]);
    data->accel_y = combine_bytes(buf[2], buf[3]);
    data->accel_z = combine_bytes(buf[4], buf[5]);
    data->temp_raw = combine_bytes(buf[6], buf[7]);
    data->gyro_x  = combine_bytes(buf[8], buf[9]);
    data->gyro_y  = combine_bytes(buf[10], buf[11]);
    data->gyro_z  = combine_bytes(buf[12], buf[13]);

    return ESP_OK;
}

/**
 * @brief User-defined function to process MPU6050 data
 * 
 * @param dev Pointer to the MPU6050 device structure
 * @param data Pointer to the structure containing the latest sensor data
 */
void mpu6050_execute(mpu6050_t* dev, mpu6050_data_t* data) {
    // TODO: Implement your custom processing here
    // Example: calculate orientation, filter data, send over UART, etc.
}

/**
 experimental code to read data MPU6050 using I2C
 * 
 * @param dev Pointer to the MPU6050 device structure
 * @param data Pointer to the structure to store read values
 * @return esp_err_t ESP_OK on success
 
#include "driver/i2c.h"
#include "mpu6050.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_SDA_IO 21
#define I2C_SCL_IO 22

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main() {
    i2c_master_init();

    mpu6050_t sensor = {
        .i2c_port = I2C_MASTER_NUM
    };

    if (mpu6050_init(&sensor) != ESP_OK) {
        printf("MPU6050 init failed\n");
        return;
    }

    mpu6050_data_t data;

    while (1) {
        if (mpu6050_read(&sensor, &data) == ESP_OK) {
            mpu6050_execute(&sensor, &data); // Gọi hàm xử lý riêng nếu có
            printf("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d | Temp Raw: %d\n",
                   data.accel_x, data.accel_y, data.accel_z,
                   data.gyro_x, data.gyro_y, data.gyro_z,
                   data.temp_raw);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/