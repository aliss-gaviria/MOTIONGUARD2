/**
 * @file espnow_comm.c
 * @brief Implementación de la comunicación inalámbrica mediante ESP-NOW.
 *
 * Este módulo implementa la inicialización del sistema de comunicación
 * ESP-NOW y el envío de datos provenientes del sensor MPU6050 y el
 * módulo GPS NEO-6M/7M/8M.
 *
 * Los datos se transmiten en formato CSV con el siguiente esquema:
 *
 * node_id,ax,ay,az,gx,gy,gz,lat,lon,alt,spd
 *
 * Ejemplo de trama enviada:
 *
 * 2,0.0123,-0.0041,0.9982,0.1021,-0.0523,0.0102,2.441234,-76.612345,1750.5,0.0
 *
 * Este módulo utiliza el modo WiFi STA del ESP32 para habilitar el
 * protocolo ESP-NOW sin necesidad de conexión a un punto de acceso.
 */

#include "espnow_comm.h"
#include "config.h"

#include <string.h>
#include <stdio.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

/**
 * @brief Tag utilizado para los mensajes de log del módulo.
 */
static const char *TAG = "ESP-NOW";

/**
 * @brief Dirección MAC del nodo receptor.
 *
 * Esta dirección se define en el archivo `config.h` y corresponde
 * al dispositivo que recibirá las tramas enviadas mediante ESP-NOW.
 */
static uint8_t s_receiver_mac[] = RECEIVER_MAC;

/**
 * @brief Callback ejecutado tras cada intento de transmisión ESP-NOW.
 *
 * Esta función es invocada automáticamente por el stack ESP-NOW para
 * informar si la trama fue entregada correctamente al nodo receptor.
 *
 * @param tx_info Información del paquete transmitido.
 * @param status  Estado de la transmisión.
 */
static void on_sent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
        ESP_LOGI(TAG, "Entrega confirmada");
    else
        ESP_LOGW(TAG, "Entrega fallida");
}

/**
 * @brief Inicializa la comunicación ESP-NOW.
 *
 * Esta función realiza los siguientes pasos:
 *
 * 1. Inicializa la memoria NVS del sistema.
 * 2. Inicializa la interfaz de red del ESP32.
 * 3. Configura el módulo WiFi en modo estación (STA).
 * 4. Inicializa el protocolo ESP-NOW.
 * 5. Registra el callback de envío.
 * 6. Añade el nodo receptor como peer autorizado.
 *
 * Una vez ejecutada esta función, el dispositivo queda listo
 * para transmitir datos mediante ESP-NOW.
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
        .policy = WIFI_COUNTRY_POLICY_MANUAL
    };

    ESP_ERROR_CHECK(esp_wifi_set_country(&country));
    ESP_ERROR_CHECK(esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_send_cb(on_sent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, s_receiver_mac, 6);
    peer.channel = 11;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    uint8_t primary;
    wifi_second_chan_t second;
    ESP_ERROR_CHECK(esp_wifi_get_channel(&primary, &second));
    ESP_LOGI(TAG, "ESP-NOW listo. Canal actual: %d", primary);
}

/**
 * @brief Envía datos del MPU6050 y el GPS mediante ESP-NOW.
 *
 * Convierte los valores crudos del IMU a unidades físicas y construye
 * una trama CSV que incluye todos los campos del sensor y del GPS.
 *
 * Conversión de unidades:
 * - aceleración  → g
 * - velocidad angular → °/s
 * - latitud/longitud  → grados decimales (6 decimales mínimo)
 * - altitud       → metros
 * - velocidad GPS → km/h
 *
 * Formato de transmisión:
 *
 * node_id,ax,ay,az,gx,gy,gz,lat,lon,alt,spd
 *
 * @param imu Puntero a la estructura con las mediciones del MPU6050.
 * @param gps Puntero a la estructura con las mediciones del GPS.
 */
void send_data_espnow(const mpu_values_t *imu, const gps_data_t *gps)
{
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);

    /* Convertir IMU a unidades físicas */
    float ax = imu->AcX / ACCEL_SENSITIVITY;
    float ay = imu->AcY / ACCEL_SENSITIVITY;
    float az = imu->AcZ / ACCEL_SENSITIVITY;
    float gx = imu->GyX / GYRO_SENSITIVITY;
    float gy = imu->GyY / GYRO_SENSITIVITY;
    float gz = imu->GyZ / GYRO_SENSITIVITY;

    /**
     * Buffer donde se construye la trama a transmitir.
     * %.6f garantiza mínimo 6 decimales en latitud y longitud.
     */
    char frame[200];

    int len = snprintf(frame, sizeof(frame),
        "2,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.1f,%.1f",
        ax, ay, az, gx, gy, gz,
        gps->latitude,
        gps->longitude,
        gps->altitude,
        gps->speed
    );

    if (len >= (int)sizeof(frame))
        ESP_LOGW(TAG, "Trama truncada!");

    ESP_LOGI(TAG, "%s", frame);

    esp_err_t res = esp_now_send(s_receiver_mac, (uint8_t *)frame, (size_t)len);

    if (res != ESP_OK)
        ESP_LOGE(TAG, "Error enviando: %s", esp_err_to_name(res));
}