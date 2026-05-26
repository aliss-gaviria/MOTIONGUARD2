/**
 * @file gps.c
 * @brief Implementación del driver para el módulo GPS NEO-6M/7M/8M.
 *
 * Este archivo contiene las funciones necesarias para:
 * - Inicializar la comunicación UART2 con el módulo GPS
 * - Recibir y parsear tramas NMEA estándar
 * - Extraer latitud, longitud, altitud y velocidad
 *
 * Tramas NMEA utilizadas:
 * - GPRMC / GNRMC: latitud, longitud, velocidad, validez del fix
 * - GPGGA / GNGGA: altitud sobre el nivel del mar
 *
 * Formato GPRMC:
 * $GPRMC,HHMMSS.ss,A,LLLL.LLLL,a,YYYYY.YYYY,a,x.x,x.x,DDMMYY,...*hh
 *
 * Formato GPGGA:
 * $GPGGA,HHMMSS.ss,LLLL.LLLL,a,YYYYY.YYYY,a,x,xx,x.x,x.x,M,...*hh
 */

#include "gps.h"
#include "config.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "GPS";

// ════════════════════════════════════════════════
//  Funciones internas de parseo NMEA
// ════════════════════════════════════════════════

/**
 * @brief Convierte coordenada NMEA (DDDMM.MMMM) a grados decimales.
 *
 * El formato NMEA expresa coordenadas como DDDMM.MMMM donde DDD son
 * grados y MM.MMMM son minutos. La conversión a grados decimales es:
 * grados + (minutos / 60).
 *
 * @param nmea_coord  Valor en formato NMEA (ej: 0241.234567 para 2°41.234567').
 * @param hemisphere  Hemisferio: 'N', 'S', 'E' u 'O'/'W'.
 * @return Coordenada en grados decimales con signo según hemisferio.
 */
static double nmea_to_decimal(double nmea_coord, char hemisphere)
{
    int    degrees = (int)(nmea_coord / 100);
    double minutes = nmea_coord - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);

    /* Hemisferios sur y oeste son negativos */
    if (hemisphere == 'S' || hemisphere == 'W' || hemisphere == 'O')
        decimal = -decimal;

    return decimal;
}

/**
 * @brief Extrae el campo N-ésimo de una trama NMEA separada por comas.
 *
 * @param sentence  Cadena con la trama NMEA completa.
 * @param field     Índice del campo a extraer (0 = identificador).
 * @param out       Buffer donde se copia el campo extraído.
 * @param out_len   Tamaño máximo del buffer de salida.
 * @return true si el campo existe y fue copiado, false en caso contrario.
 */
static bool nmea_get_field(const char *sentence, int field,
                           char *out, size_t out_len)
{
    int current = 0;
    const char *p = sentence;

    while (*p && current < field) {
        if (*p == ',') current++;
        p++;
    }

    if (!*p) return false;

    size_t i = 0;
    while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n') {
        if (i < out_len - 1) out[i++] = *p;
        p++;
    }
    out[i] = '\0';
    return (i > 0);
}

/**
 * @brief Parsea una trama GPRMC o GNRMC.
 *
 * Extrae latitud, longitud, velocidad y estado del fix.
 *
 * @param sentence  Trama NMEA completa.
 * @param data      Estructura donde se almacenan los valores.
 * @return true si la trama es válida y tiene fix activo (campo estado = 'A').
 */
static bool parse_gprmc(const char *sentence, gps_data_t *data)
{
    char field[20];

    /* Campo 2: estado ('A' = activo, 'V' = inválido) */
    if (!nmea_get_field(sentence, 2, field, sizeof(field))) return false;
    if (field[0] != 'A') return false;

    /* Campo 3: latitud en formato NMEA */
    char lat_str[15], lat_hem[3], lon_str[15], lon_hem[3], spd_str[10];

    if (!nmea_get_field(sentence, 3, lat_str,  sizeof(lat_str)))  return false;
    if (!nmea_get_field(sentence, 4, lat_hem,  sizeof(lat_hem)))  return false;
    if (!nmea_get_field(sentence, 5, lon_str,  sizeof(lon_str)))  return false;
    if (!nmea_get_field(sentence, 6, lon_hem,  sizeof(lon_hem)))  return false;
    if (!nmea_get_field(sentence, 7, spd_str,  sizeof(spd_str)))  return false;

    data->latitude  = nmea_to_decimal(atof(lat_str), lat_hem[0]);
    data->longitude = nmea_to_decimal(atof(lon_str), lon_hem[0]);

    /* Velocidad en nudos → km/h (1 nudo = 1.852 km/h) */
    data->speed = (float)(atof(spd_str) * 1.852);
    data->valid = true;

    return true;
}

/**
 * @brief Parsea una trama GPGGA o GNGGA.
 *
 * Extrae la altitud sobre el nivel del mar.
 *
 * @param sentence  Trama NMEA completa.
 * @param data      Estructura donde se almacena la altitud.
 * @return true si la trama contiene un fix válido (campo calidad > 0).
 */
static bool parse_gpgga(const char *sentence, gps_data_t *data)
{
    char field[20];

    /* Campo 6: calidad del fix (0 = sin fix) */
    if (!nmea_get_field(sentence, 6, field, sizeof(field))) return false;
    if (atoi(field) == 0) return false;

    /* Campo 9: altitud en metros */
    char alt_str[12];
    if (!nmea_get_field(sentence, 9, alt_str, sizeof(alt_str))) return false;

    data->altitude = (float)atof(alt_str);
    return true;
}

// ════════════════════════════════════════════════
//  API pública
// ════════════════════════════════════════════════

void gps_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate  = GPS_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM,
                                 GPS_TX_PIN,     /* TX del ESP32 → RX del GPS */
                                 GPS_RX_PIN,     /* RX del ESP32 ← TX del GPS */
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM,
                                        GPS_UART_BUF_SIZE * 2, 0,
                                        0, NULL, 0));

    ESP_LOGI(TAG, "GPS listo (UART2 RX=GPIO%d TX=GPIO%d @ %d baud)",
             GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);
}

bool gps_read(gps_data_t *data)
{
    /* Inicializar estructura con valores en cero */
    data->latitude  = 0.0;
    data->longitude = 0.0;
    data->altitude  = 0.0f;
    data->speed     = 0.0f;
    data->valid     = false;

    bool got_rmc = false;
    bool got_gga = false;

    char line[128];
    int  idx = 0;
    uint8_t byte;

    int64_t deadline = (int64_t)GPS_READ_TIMEOUT_MS * 1000; // en µs
    int64_t elapsed  = 0;
    int64_t t_start  = esp_timer_get_time();

    /* Leer bytes hasta obtener GPRMC y GPGGA o agotar el timeout */
    while (elapsed < deadline) {

        int n = uart_read_bytes(GPS_UART_NUM, &byte, 1, pdMS_TO_TICKS(10));
        elapsed = esp_timer_get_time() - t_start;

        if (n <= 0) continue;

        if (byte == '\n') {
    
            /* Identificar y parsear la trama */
            if (strncmp(line, "$GPRMC", 6) == 0 ||
                strncmp(line, "$GNRMC", 6) == 0) {
                got_rmc = parse_gprmc(line, data);
            }
            else if (strncmp(line, "$GPGGA", 6) == 0 ||
                     strncmp(line, "$GNGGA", 6) == 0) {
                got_gga = parse_gpgga(line, data);
            }

            /* Salir cuando se tienen ambas tramas */
            if (got_rmc && got_gga) break;

        } else if (byte != '\r') {
            if (idx < (int)sizeof(line) - 1)
                line[idx++] = (char)byte;
        }
    }

    if (data->valid) {
        ESP_LOGI(TAG, "Fix: lat=%.6f lon=%.6f alt=%.1fm spd=%.1fkm/h",
                 data->latitude, data->longitude,
                 data->altitude, data->speed);
    } else {
        ESP_LOGW(TAG, "Sin fix GPS (timeout o trama inválida)");
    }

    return data->valid;
}