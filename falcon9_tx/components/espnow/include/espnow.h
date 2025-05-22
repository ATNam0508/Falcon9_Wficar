#pragma once

#include "esp_now.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo Wi-Fi (STA mode) và ESP-NOW
 * 
 * @param channel Kênh Wi-Fi sử dụng (1~13)
 * @return ESP_OK nếu thành công, mã lỗi nếu thất bại
 */
esp_err_t espnow_init(uint8_t channel);

/**
 * @brief Gửi dữ liệu qua ESP-NOW
 * 
 * @param dest_mac Địa chỉ MAC đích (6 byte)
 * @param data Dữ liệu gửi
 * @param len Độ dài dữ liệu
 * @return ESP_OK nếu thành công, mã lỗi nếu thất bại
 */
esp_err_t espnow_send(const uint8_t *dest_mac, const uint8_t *data, size_t len);

/**
 * @brief Đăng ký callback khi nhận dữ liệu
 * 
 * @param cb Hàm callback nhận ESP-NOW
 */
void espnow_register_recv_cb(esp_now_recv_cb_t cb);

/**
 * @brief Đăng ký callback khi gửi xong
 * 
 * @param cb Hàm callback gửi ESP-NOW
 */
void espnow_register_send_cb(esp_now_send_cb_t cb);

#ifdef __cplusplus
}
#endif
