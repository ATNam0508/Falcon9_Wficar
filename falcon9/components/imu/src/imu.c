#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MPU6050_ADDR         0x68
#define WHO_AM_I_REG         0x75
#define PWR_MGMT_1           0x6B
#define ACCEL_XOUT_H         0x3B

static const char *TAG = "IMU";

// Toàn cục dùng chung cho mọi thao tác
static i2c_port_t imu_i2c_port;
static float angle_z = 0;
static float gyro_z_offset = 0;
static uint8_t buf[14];  // Buffer dùng chung để đọc dữ liệu

static mpu6050_data_t raw_data;
static mpu6050_data_t data;

static esp_err_t read_bytes(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(imu_i2c_port, MPU6050_ADDR, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

static esp_err_t write_byte(uint8_t reg, uint8_t value) {
    uint8_t tmp[2] = {reg, value};
    return i2c_master_write_to_device(imu_i2c_port, MPU6050_ADDR, tmp, 2, pdMS_TO_TICKS(100));
}

static int16_t combine(uint8_t msb, uint8_t lsb) {
    return ((int16_t)msb << 8) | lsb;
}

esp_err_t imu_init(i2c_port_t i2c_port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed) {
    imu_i2c_port = i2c_port;

    // Cấu hình I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_speed
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));

    // Kiểm tra thiết bị
    uint8_t id = 0;
    ESP_ERROR_CHECK(read_bytes(WHO_AM_I_REG, &id, 1));
    if (id != 0x68) {
        ESP_LOGE(TAG, "MPU6050 not found (WHO_AM_I=0x%02X)", id);
        return ESP_FAIL;
    }

    // Đánh thức MPU6050
    ESP_ERROR_CHECK(write_byte(PWR_MGMT_1, 0x00));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "MPU6050 connected");
    return ESP_OK;
}

void imu_calibrate(void) {
    float sum = 0;
    for (int i = 0; i < 1000; i++) {
        if (read_bytes(ACCEL_XOUT_H, buf, 14) == ESP_OK) {
            int16_t gz = combine(buf[12], buf[13]);
            sum += gz;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    gyro_z_offset = sum / 1000.0f;
    angle_z = 0;
    ESP_LOGI(TAG, "IMU calibrated. Offset Z = %.2f", gyro_z_offset);
}

void imu_update(void) {
    if (read_bytes(ACCEL_XOUT_H, buf, 14) != ESP_OK) return;
    raw_data.gyro_z  = combine(buf[12], buf[13]);

    data = raw_data;

    float corrected = (float)data.gyro_z - gyro_z_offset;
    angle_z += corrected / 131.0f;
}

float imu_get_angle_z(void) {
    return angle_z;
}

void imu_reset_angle_z(void) {
    angle_z = 0;
}
