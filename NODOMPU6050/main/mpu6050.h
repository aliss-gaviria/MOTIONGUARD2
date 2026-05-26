/**
 * @file mpu6050.h
 * @brief Definiciones y prototipos del driver para el sensor MPU6050.
 *
 * Este archivo define:
 * - La estructura de datos utilizada para almacenar las lecturas del sensor
 * - Las funciones públicas para inicializar, calibrar y leer el sensor
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Estructura que almacena las lecturas del sensor MPU6050.
 *
 * @note El campo de temperatura fue eliminado intencionalmente.
 *       El sensor sigue leyendo los 14 bytes del registro ACCEL_XOUT
 *       pero los bytes correspondientes a temperatura (raw[6] y raw[7])
 *       se descartan sin procesarse.
 */
typedef struct {

    /* Valores del acelerómetro */
    int16_t AcX;
    int16_t AcY;
    int16_t AcZ;

    /* Valores del giroscopio */
    int16_t GyX;
    int16_t GyY;
    int16_t GyZ;

} mpu_values_t;


/**
 * @brief Inicializa el sensor MPU6050.
 */
void mpu6050_init(void);


/**
 * @brief Ejecuta la calibración del sensor.
 */
void mpu6050_calibrate(void);


/**
 * @brief Obtiene los valores actuales del sensor.
 *
 * @param v puntero a la estructura donde se almacenarán los datos
 * @return esp_err_t estado de la operación
 */
esp_err_t mpu_get_values(mpu_values_t *v);