#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

#define I2C_PORT    I2C_NUM_0
#define SDA_PIN     GPIO_NUM_21
#define SCL_PIN     GPIO_NUM_22

void app_main(void) {
    if (imu_init(I2C_PORT, SDA_PIN, SCL_PIN, 400000) != ESP_OK) {
        printf("IMU init failed\n");
        return;
    }

    while (1) {
        imu_update();
        float angle = imu_get_angle_z();
        printf("Angle Z: %.2f deg\n", angle);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
