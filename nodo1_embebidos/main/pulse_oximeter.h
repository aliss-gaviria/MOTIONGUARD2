#ifndef PULSE_OXIMETER_H
#define PULSE_OXIMETER_H

/**
 * @file pulse_oximeter.h
 * @brief Interfaz publica para el sensor MAX30102.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Lecturas procesadas del pulsioximetro.
 */
typedef struct {
    /** Frecuencia cardiaca calculada en latidos por minuto. */
    int bpm;
    /** Saturacion de oxigeno calculada en porcentaje. */
    int spo2;
    /** Indica si la lectura de BPM es confiable. */
    bool bpm_valid;
    /** Indica si la lectura de SpO2 es confiable. */
    bool spo2_valid;
    /** Indica si el sensor detecta contacto con el dedo. */
    bool finger_detected;
    /** Valor infrarrojo crudo mas reciente. */
    uint32_t ir_raw;
    /** Valor rojo crudo mas reciente. */
    uint32_t red_raw;
    /** Amplitud AC infrarroja de la ventana de senal. */
    uint32_t ir_ac;
    /** Amplitud AC roja de la ventana de senal. */
    uint32_t red_ac;
} pulse_oximeter_data_t;

/**
 * @brief Detecta, configura e inicializa el sensor MAX30102.
 *
 * @return ESP_OK si el sensor esta listo; un codigo de error ESP-IDF si falla.
 */
esp_err_t pulse_oximeter_init(void);

/**
 * @brief Actualiza el procesamiento PPG y entrega la ultima medicion publica.
 *
 * @param data Estructura de salida con BPM, SpO2, validez y senales crudas.
 * @return ESP_OK si la lectura se completo; ESP_FAIL si el sensor no esta listo.
 */
esp_err_t pulse_oximeter_read(pulse_oximeter_data_t *data);

#ifdef __cplusplus
}
#endif

#endif
