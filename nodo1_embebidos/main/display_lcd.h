#pragma once

/**
 * @file display_lcd.h
 * @brief Interfaz de visualizacion para la pantalla LCD GC9A01.
 */

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa la pantalla, el bus SPI y el panel LCD.
 *
 * @return ESP_OK si la pantalla queda lista para dibujar.
 */
esp_err_t lcd_init_display(void);

/**
 * @brief Muestra los datos actuales de movimiento y pulsioximetria.
 *
 * @param ax Aceleracion en X.
 * @param ay Aceleracion en Y.
 * @param az Aceleracion en Z.
 * @param gx Velocidad angular en X.
 * @param gy Velocidad angular en Y.
 * @param gz Velocidad angular en Z.
 * @param bpm Frecuencia cardiaca calculada.
 * @param spo2 Saturacion de oxigeno calculada.
 * @param bpm_valid Validez de la lectura BPM.
 * @param spo2_valid Validez de la lectura SpO2.
 * @param finger_detected Estado de contacto con el sensor.
 */
void lcd_show_imu_data(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       int bpm, int spo2,
                       bool bpm_valid, bool spo2_valid,
                       bool finger_detected);

#ifdef __cplusplus
}
#endif
