/**
 * @file config.h
 * @brief Parámetros de configuración del nodo sensor basado en ESP32.
 *
 * Este archivo centraliza todos los parámetros configurables del sistema,
 * incluyendo:
 *
 * - Número de muestras a adquirir
 * - Intervalo de muestreo
 * - Parámetros de calibración del sensor MPU6050
 * - Configuración del bus I2C
 * - Sensibilidad del acelerómetro y giroscopio
 * - Configuración del LED indicador
 * - Tiempo de Deep Sleep
 * - Dirección MAC del nodo receptor
 * - Configuración del módulo GPS NEO-6M/7M/8M
 *
 * Separar estos parámetros en un archivo dedicado facilita la
 * modificación y mantenimiento del sistema sin necesidad de
 * alterar la lógica del programa.
 */

#pragma once

/**
 * @brief Intervalo entre muestras en milisegundos.
 */
#define SAMPLE_INTERVAL_MS    10000


/**
 * @brief Número de muestras utilizadas para calibrar el sensor MPU6050.
 */
#define CALIB_SAMPLES         200

/**
 * @brief Retardo entre muestras durante el proceso de calibración (ms).
 */
#define CALIB_DELAY_MS        5


/**
 * @brief Pin GPIO utilizado como línea SCL del bus I2C.
 */
#define I2C_SCL_IO            4

/**
 * @brief Pin GPIO utilizado como línea SDA del bus I2C.
 */
#define I2C_SDA_IO            5

/**
 * @brief Frecuencia de operación del bus I2C.
 */
#define I2C_FREQ_HZ           100000

/**
 * @brief Tiempo máximo de espera en transacciones I2C (ms).
 */
#define I2C_TIMEOUT_MS        100


/**
 * @brief Dirección I2C del sensor MPU6050.
 */
#define MPU6050_ADDR          0x68


/**
 * @brief Sensibilidad del acelerómetro en configuración ±2g.
 *
 * Valor tomado del datasheet del MPU6050.
 * Permite convertir los valores crudos del sensor a unidades de g.
 */
#define ACCEL_SENSITIVITY     16384.0f


/**
 * @brief Sensibilidad del giroscopio en configuración ±250°/s.
 *
 * Permite convertir los valores crudos del sensor a grados por segundo.
 */
#define GYRO_SENSITIVITY      131.0f


/**
 * @brief Pin GPIO utilizado para el LED indicador del nodo.
 */
#define LED_PIN               2


/**
 * @brief Dirección MAC del nodo receptor para comunicación ESP-NOW.
 *
 * Esta dirección identifica el dispositivo que recibirá los datos
 * transmitidos por el nodo sensor.
 */
#define RECEIVER_MAC          { 0x2C, 0xBC, 0xBB, 0x06, 0xB5, 0x3C }


// ── GPS NEO-6M/7M/8M ────────────────────────────

/**
 * @brief Puerto UART utilizado para comunicación con el módulo GPS.
 */
#define GPS_UART_NUM          UART_NUM_2

/**
 * @brief Pin GPIO de recepción de datos desde el GPS (RX del ESP32).
 */
#define GPS_RX_PIN            25

/**
 * @brief Pin GPIO de transmisión de datos hacia el GPS (TX del ESP32).
 */
#define GPS_TX_PIN            26

/**
 * @brief Velocidad de comunicación del módulo GPS en baudios.
 *
 * El NEO-6M/7M/8M usa 9600 baudios por defecto.
 */
#define GPS_BAUD_RATE         9600

/**
 * @brief Tamaño del buffer de recepción UART para el GPS.
 */
#define GPS_UART_BUF_SIZE     512

/**
 * @brief Tiempo máximo de espera para obtener una trama NMEA válida (ms).
 */
#define GPS_READ_TIMEOUT_MS   2000