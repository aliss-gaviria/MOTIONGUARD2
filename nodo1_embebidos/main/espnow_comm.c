#include "espnow_comm.h"
#include "board_config.h"

#include <string.h>
#include <stdio.h>
#include "pulse_oximeter.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "esp_log.h"

static const char *TAG = "ESP-NOW";

static uint8_t receiver_mac[] = RECEIVER_MAC;

/**
 * @brief Registra en log el resultado del ultimo envio ESP-NOW.
 *
 * @param info Informacion del destino entregada por ESP-IDF.
 * @param status Estado de entrega reportado por ESP-NOW.
 */
static void on_sent(const esp_now_send_info_t *info,
                    esp_now_send_status_t status)
{
    (void)info;

    if (status == ESP_NOW_SEND_SUCCESS)
        ESP_LOGI(TAG, "Entrega OK");
    else
        ESP_LOGW(TAG, "Entrega FAIL");
}

/**
 * @copydoc espnow_init
 */
void espnow_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());

    esp_now_register_send_cb(on_sent);

    esp_now_peer_info_t peer = {};

    memcpy(peer.peer_addr, receiver_mac, 6);

    peer.channel = WIFI_CHANNEL;

    peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW listo");
}

/**
 * @copydoc espnow_send_node1
 */
void espnow_send_node1(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       int spo2, int bpm)
{
    char frame[240];

    int len = snprintf(frame,
                       sizeof(frame),
                       "1,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d",
                       ax, ay, az,
                       gx, gy, gz,
                       spo2, bpm);

    ESP_LOGI(TAG, "%s", frame);

    esp_now_send(receiver_mac,
                 (uint8_t *)frame,
                 len);
}
