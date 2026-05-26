#ifndef IMU_H
#define IMU_H

/**
 * @file imu.h
 * @brief Lectura calibrada del sensor inercial QMI8658.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/**
 * @brief Muestra filtrada de acelerometro y giroscopio.
 */
typedef struct {
    /** Aceleracion en el eje X. */
    float ax;
    /** Aceleracion en el eje Y. */
    float ay;
    /** Aceleracion en el eje Z. */
    float az;

    /** Velocidad angular en el eje X. */
    float gx;
    /** Velocidad angular en el eje Y. */
    float gy;
    /** Velocidad angular en el eje Z. */
    float gz;
} imu_data_t;

/**
 * @brief Inicializa el bus I2C, configura el QMI8658 y calcula offsets.
 *
 * @return ESP_OK si el sensor queda listo; ESP_FAIL ante error de inicializacion.
 */
esp_err_t imu_init(void);

/**
 * @brief Lee una muestra promediada y filtrada de la IMU.
 *
 * @param data Estructura de salida donde se escriben los datos.
 * @return ESP_OK si la lectura es valida; ESP_FAIL si el sensor no esta listo.
 */
esp_err_t imu_read(imu_data_t *data);

#ifdef __cplusplus
}
#endif

#endif
