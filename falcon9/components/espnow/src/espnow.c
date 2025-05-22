#include "espnow.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "espnow";

/**
 * @brief Khởi tạo Wi-Fi ở chế độ STA và khởi động ESP-NOW
 *
 * Thực hiện:
 * - Khởi động NVS
 * - Khởi tạo Wi-Fi và cấu hình channel
 * - Khởi tạo ESP-NOW
 * - Thêm broadcast peer để có thể gửi broadcast
 *
 * @param channel Kênh Wi-Fi sử dụng (1~13)
 * @return ESP_OK nếu thành công, mã lỗi nếu thất bại
 */
esp_err_t espnow_init(uint8_t channel) {
    ESP_LOGI(TAG, "Init Wi-Fi & ESP-NOW (channel %d)", channel);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());

    // Thêm broadcast peer để có thể gửi đến FF:FF:FF:FF:FF:FF
    esp_now_peer_info_t peer = {};
    peer.channel = channel;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    memset(peer.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW ready");
    return ESP_OK;
}

/**
 * @brief Gửi dữ liệu qua ESP-NOW
 *
 * @param dest_mac Địa chỉ MAC đích (6 byte)
 * @param data Dữ liệu muốn gửi
 * @param len Độ dài dữ liệu
 * @return ESP_OK nếu thành công, mã lỗi nếu thất bại
 */
esp_err_t espnow_send(const uint8_t *dest_mac, const uint8_t *data, size_t len) {
    return esp_now_send(dest_mac, data, len);
}

/**
 * @brief Đăng ký callback khi nhận dữ liệu
 *
 * @param cb Hàm callback nhận ESP-NOW
 */
void espnow_register_recv_cb(esp_now_recv_cb_t cb) {
    esp_now_register_recv_cb(cb);
}

/**
 * @brief Đăng ký callback khi gửi dữ liệu xong
 *
 * @param cb Hàm callback gửi ESP-NOW
 */
void espnow_register_send_cb(esp_now_send_cb_t cb) {
    esp_now_register_send_cb(cb);
}

/** example for espnow_send
    * @brief Gửi dữ liệu qua ESP-NOW
    
    * @param dest_mac Địa chỉ MAC đích (6 byte)
    * @param data Dữ liệu muốn gửi
    * @param len Độ dài dữ liệu
    * @return ESP_OK nếu thành công, mã lỗi nếu thất bại
    * @code
#include "espnow.h"
#include "esp_log.h"
#include <string.h"

static const char *TAG = "TX";

// Địa chỉ MAC đích - bạn thay bằng MAC thật nếu gửi unicast
static uint8_t dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast

void on_send(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Gửi đến "MACSTR": %s", MAC2STR(mac_addr),
             status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void app_main(void) {
    espnow_init(1);
    espnow_register_send_cb(on_send);

    while (1) {
        const char *msg = "ESP32 TX: Hello ESP-NOW!";
        espnow_send(dest_mac, (const uint8_t *)msg, strlen(msg));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
 */

/**
 * @brief Nhận dữ liệu qua ESP-NOW
 * @param info Thông tin về gói nhận
 * @param data Dữ liệu nhận
 * @param len Độ dài dữ liệu
 * @code
#include "espnow.h"
#include "esp_log.h"

static const char *TAG = "RX";

void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    ESP_LOGI(TAG, "Nhận từ "MACSTR": %.*s", MAC2STR(info->src_addr), len, data);
}

void app_main(void) {
    espnow_init(1);
    espnow_register_recv_cb(on_recv);

    // Không cần làm gì thêm, chỉ cần ngồi chờ data
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
 */

/**
 * @brief Gửi và nhận dữ liệu qua ESP-NOW
 * @param info Thông tin về gói nhận
 * @param data Dữ liệu nhận
 * @param len Độ dài dữ liệu
 * @code
void app_main(void) {
    espnow_init(1);
    espnow_register_recv_cb(on_recv);
    espnow_register_send_cb(on_send);

    while (1) {
        const char *msg = "Tôi vừa gửi vừa nhận!";
        espnow_send(dest_mac, (const uint8_t *)msg, strlen(msg));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
 */