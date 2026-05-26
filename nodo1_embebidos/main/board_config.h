#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/**
 * @file board_config.h
 * @brief Definiciones de pines, dimensiones y parametros de comunicacion del nodo.
 */

#include "driver/gpio.h"

#define LCD_H_RES      240
#define LCD_V_RES      240

#define LCD_BL         GPIO_NUM_2
#define LCD_DC         GPIO_NUM_8
#define LCD_CS         GPIO_NUM_9
#define LCD_CLK        GPIO_NUM_10
#define LCD_MOSI       GPIO_NUM_11
#define LCD_MISO       GPIO_NUM_12
#define LCD_RST        GPIO_NUM_14

#define I2C_SDA        GPIO_NUM_6
#define I2C_SCL        GPIO_NUM_7

#define MAX30102_I2C_PORT I2C_NUM_1
#define MAX30102_SDA      GPIO_NUM_15
#define MAX30102_SCL      GPIO_NUM_16

#define WIFI_CHANNEL   11
#define ESPNOW_WIFI_SSID "Galaxy S20 FEE8DE"
#define RECEIVER_MAC {0x2C,0xBC,0xBB,0x06,0xB5,0x3C}
#endif
