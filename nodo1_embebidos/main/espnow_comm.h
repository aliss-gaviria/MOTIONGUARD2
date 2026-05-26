#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include "imu.h"

/**
 * @file espnow_comm.h
 * @brief Interfaz de comunicacion ESP-NOW para el nodo de sensores.
 */

/**
 * @brief Inicializa Wi-Fi en modo estacion y registra el receptor ESP-NOW.
 */
void espnow_init(void);

/**
 * @brief Envia la trama completa del nodo 1 por ESP-NOW.
 *
 * La trama contiene identificador de nodo, acelerometro, giroscopio,
 * saturacion de oxigeno y frecuencia cardiaca.
 *
 * @param ax Aceleracion en el eje X.
 * @param ay Aceleracion en el eje Y.
 * @param az Aceleracion en el eje Z.
 * @param gx Velocidad angular en el eje X.
 * @param gy Velocidad angular en el eje Y.
 * @param gz Velocidad angular en el eje Z.
 * @param spo2 Saturacion de oxigeno en porcentaje.
 * @param bpm Frecuencia cardiaca en latidos por minuto.
 */
void espnow_send_node1(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       int spo2, int bpm);

#endif
