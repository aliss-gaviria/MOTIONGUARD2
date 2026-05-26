#pragma once

/**
 * @file oled.h
 * @brief API pública para el manejo de la pantalla OLED SSD1306 128x64.
 *
 * Utiliza el bus I2C ya inicializado por el módulo mpu6050,
 * comunicándose con la dirección 0x3C del SSD1306.
 */

#include <stdint.h>

/**
 * @brief Inicializa la pantalla OLED SSD1306.
 *
 * Envía la secuencia de comandos de configuración al controlador.
 * Debe llamarse después de mpu6050_init(), ya que el bus I2C
 * debe estar activo antes de usar esta función.
 */
void oled_init(void);

/**
 * @brief Borra completamente el contenido de la pantalla (pone todo en negro).
 */
void oled_clear(void);

/**
 * @brief Muestra los datos del sensor MPU6050 en la pantalla OLED.
 *
 * Presenta en pantalla los seis ejes (acelerómetro y giroscopio)
 * con sus valores convertidos a unidades físicas (g y °/s).
 *
 * @param ax  Aceleración en X [g]
 * @param ay  Aceleración en Y [g]
 * @param az  Aceleración en Z [g]
 * @param gx  Velocidad angular en X [°/s]
 * @param gy  Velocidad angular en Y [°/s]
 * @param gz  Velocidad angular en Z [°/s]
 */
void oled_show_data(float ax, float ay, float az,
                    float gx, float gy, float gz);

/**
 * @brief Muestra los datos del módulo GPS en la pantalla OLED.
 *
 * Presenta latitud, longitud, altitud y velocidad en pantalla.
 * La latitud y longitud se muestran con 6 decimales de precisión.
 *
 * @param lat  Latitud en grados decimales
 * @param lon  Longitud en grados decimales
 * @param alt  Altitud sobre el nivel del mar [m]
 * @param spd  Velocidad sobre el suelo [km/h]
 */
void oled_show_gps(double lat, double lon, float alt, float spd);