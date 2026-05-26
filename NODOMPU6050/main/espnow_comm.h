/**
 * @file espnow_comm.h
 * @brief Interfaz del módulo de comunicación ESP-NOW.
 *
 * Este archivo define las funciones públicas utilizadas para inicializar
 * la comunicación inalámbrica y transmitir datos provenientes del
 * sensor MPU6050 y el módulo GPS.
 */

#pragma once
#include "mpu6050.h"
#include "gps.h"

/**
 * @brief Inicializa el sistema de comunicación ESP-NOW.
 *
 * Configura el módulo WiFi del ESP32 en modo estación (STA) e
 * inicializa el protocolo ESP-NOW. Además, registra el nodo
 * receptor como peer autorizado utilizando la dirección MAC
 * definida en `config.h`.
 */
void espnow_init(void);

/**
 * @brief Envía los datos del sensor MPU6050 y el GPS mediante ESP-NOW.
 *
 * Esta función serializa las mediciones en formato CSV y las transmite
 * al nodo receptor. El esquema de la trama es:
 *
 * node_id,ax,ay,az,gx,gy,gz,lat,lon,alt,spd
 *
 * @param imu Puntero a la estructura con las mediciones del MPU6050.
 * @param gps Puntero a la estructura con las mediciones del GPS.
 */
void send_data_espnow(const mpu_values_t *imu, const gps_data_t *gps);