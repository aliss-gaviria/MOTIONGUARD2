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
 */
static void on_sent(const esp_now_send_info_t *info,
                    esp_now_send_status_t status)
{
    (void)info;

    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "Entrega OK");
    }
    else
    {
        ESP_LOGW(TAG, "Entrega FAIL");
    }
}

/**
 * @brief Inicializa ESP-NOW.
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

    
    wifi_country_t country = {
        .cc = "CO",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_MANUAL};

    ESP_ERROR_CHECK(esp_wifi_set_country(&country));

    ESP_ERROR_CHECK(
        esp_wifi_set_channel(
            WIFI_CHANNEL,
            WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());

    esp_now_register_send_cb(on_sent);

    esp_now_peer_info_t peer = {};

    memcpy(peer.peer_addr, receiver_mac, 6);

    peer.channel = WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    uint8_t mac[6];

    ESP_ERROR_CHECK(
        esp_wifi_get_mac(
            WIFI_IF_STA,
            mac));

    ESP_LOGI(TAG,
             "MAC NODO 1: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0],
             mac[1],
             mac[2],
             mac[3],
             mac[4],
             mac[5]);

    uint8_t primary;
    wifi_second_chan_t second;

    ESP_ERROR_CHECK(
        esp_wifi_get_channel(
            &primary,
            &second));

    ESP_LOGI(TAG, "CANAL NODO 1: %d", primary);

    ESP_LOGI(TAG,
             "RECEPTOR: %02X:%02X:%02X:%02X:%02X:%02X",
             receiver_mac[0],
             receiver_mac[1],
             receiver_mac[2],
             receiver_mac[3],
             receiver_mac[4],
             receiver_mac[5]);

    ESP_LOGI(TAG, "ESP-NOW listo");
}

/**
 * @brief Envia datos del nodo 1.
 */
void espnow_send_node1(float ax,
                       float ay,
                       float az,
                       float gx,
                       float gy,
                       float gz,
                       int spo2,
                       int bpm)
{
    char frame[240];

    int len = snprintf(frame,
                       sizeof(frame),
                       "1,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d",
                       ax,
                       ay,
                       az,
                       gx,
                       gy,
                       gz,
                       spo2,
                       bpm);

    if (len >= sizeof(frame))
    {
        ESP_LOGW(TAG, "Trama truncada");
    }

    ESP_LOGI(TAG, "Enviando: %s", frame);

    esp_err_t err =
        esp_now_send(
            receiver_mac,
            (uint8_t *)frame,
            len);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "esp_now_send error: %s",
                 esp_err_to_name(err));
    }
}