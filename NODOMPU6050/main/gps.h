/**
 * @file gps.h
 * @brief API pública del driver para el módulo GPS NEO-6M/7M/8M.
 *
 * Este archivo define:
 * - La estructura de datos para almacenar las lecturas del GPS
 * - Las funciones públicas para inicializar y leer el módulo GPS
 *
 * La comunicación se realiza mediante UART2, con RX en GPIO25
 * y TX en GPIO26, leyendo y parseando tramas NMEA GPRMC y GPGGA.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Estructura que almacena una lectura completa del módulo GPS.
 *
 * Los campos de latitud y longitud se expresan en grados decimales
 * con al menos 6 decimales de precisión.
 */
typedef struct {
    double  latitude;   /**< Latitud en grados decimales  (ej: 2.441234) */
    double  longitude;  /**< Longitud en grados decimales (ej: -76.612345) */
    float   altitude;   /**< Altitud sobre el nivel del mar [m] */
    float   speed;      /**< Velocidad sobre el suelo [km/h] */
    bool    valid;      /**< true si la trama GPS contiene un fix válido */
} gps_data_t;


/**
 * @brief Inicializa el módulo GPS configurando el puerto UART2.
 *
 * Configura la comunicación UART con los parámetros definidos en
 * config.h (pines, baudios, buffer). Debe llamarse una sola vez
 * durante la inicialización del sistema.
 */
void gps_init(void);


/**
 * @brief Lee y parsea una trama NMEA del módulo GPS.
 *
 * Espera hasta GPS_READ_TIMEOUT_MS milisegundos a recibir tramas
 * GPRMC (latitud, longitud, velocidad) y GPGGA (altitud).
 * Si el GPS no tiene fix satelital, el campo `valid` se establece
 * en false y los demás valores son cero.
 *
 * @param[out] data  Puntero a la estructura donde se almacenan los datos.
 * @return true  si se obtuvo al menos una trama válida con fix.
 * @return false si no se recibió trama válida en el tiempo de espera.
 */
bool gps_read(gps_data_t *data);