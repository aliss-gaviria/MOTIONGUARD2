/**
 * @file main.c
 * @brief Nodo de adquisición de datos usando ESP32, MPU6050 y GPS NEO-6M/7M/8M.
 *
 * Este archivo implementa la lógica principal del nodo sensor. El sistema
 * utiliza un microcontrolador ESP32 conectado a un sensor MPU6050 para
 * adquirir datos de aceleración y velocidad angular, y un módulo GPS para
 * obtener posición geográfica, altitud y velocidad.
 *
 * Los datos se procesan y posteriormente se envían de forma inalámbrica
 * utilizando el protocolo ESP-NOW. Además, el sistema incluye un LED
 * indicador de estado y una pantalla OLED para visualización local.
 *
 * Arquitectura del sistema:
 * - Sensor MPU6050 (acelerómetro + giroscopio) — I2C pines 4/5
 * - Módulo GPS NEO-6M/7M/8M (NMEA) — UART2 pines 25(RX)/26(TX)
 * - Pantalla OLED SSD1306 128x64 — I2C compartido con MPU6050
 * - Comunicación inalámbrica ESP-NOW
 * - FreeRTOS para manejo de tareas
 * - LED indicador de actividad
 *
 * Flujo de funcionamiento:
 * 1. Inicialización de periféricos
 * 2. Calibración del sensor MPU6050
 * 3. Creación de tareas FreeRTOS
 * 4. task_imu  — lee MPU6050 cada SAMPLE_INTERVAL_MS de forma continua
 * 5. task_gps  — lee GPS de forma independiente (no bloquea el IMU)
 * 6. task_oled — alterna pantalla: 5s IMU / 5s GPS cada 10 segundos
 * 7. task_send — envía ambos datos por ESP-NOW de forma continua
 *
 * @note El modo Deep Sleep fue eliminado. El nodo opera de forma
 *       continua e indefinida.
 */

#include "config.h"
#include "mpu6050.h"
#include "espnow_comm.h"
#include "oled.h"
#include "gps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

/**
 * @brief Tag utilizado para los mensajes de log del sistema.
 */
static const char *TAG = "MAIN";

/**
 * @brief Estructura compartida con los datos más recientes del IMU.
 *
 * Accedida por task_imu, task_oled y task_send.
 * Protegida por s_imu_mutex.
 */
static mpu_values_t      s_shared_imu;
static SemaphoreHandle_t s_imu_mutex;

/**
 * @brief Estructura compartida con los datos más recientes del GPS.
 *
 * Accedida por task_gps, task_oled y task_send.
 * Protegida por s_gps_mutex.
 */
static gps_data_t        s_shared_gps;
static SemaphoreHandle_t s_gps_mutex;

/**
 * @brief Inicializa el LED de estado.
 *
 * Configura el pin definido en `LED_PIN` como salida digital para
 * permitir indicar el estado del nodo mediante encendido o apagado.
 */
static void led_init(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

// ════════════════════════════════════════════════
//  Tarea: lectura del IMU (MPU6050)
// ════════════════════════════════════════════════

/**
 * @brief Tarea FreeRTOS que lee el MPU6050 de forma continua e independiente.
 *
 * Lee el sensor cada SAMPLE_INTERVAL_MS y actualiza el dato compartido.
 * La visualización en OLED es responsabilidad exclusiva de task_oled,
 * por lo que esta tarea no llama a ninguna función de pantalla.
 *
 * @param pvParameters Parámetros de la tarea (no utilizados).
 */
static void task_imu(void *pvParameters)
{
    ESP_LOGI(TAG, "[IMU] Tarea iniciada");

    while (1) {
        mpu_values_t temp;

        if (mpu_get_values(&temp) == ESP_OK) {

            float ax = temp.AcX / ACCEL_SENSITIVITY;
            float ay = temp.AcY / ACCEL_SENSITIVITY;
            float az = temp.AcZ / ACCEL_SENSITIVITY;
            float gx = temp.GyX / GYRO_SENSITIVITY;
            float gy = temp.GyY / GYRO_SENSITIVITY;
            float gz = temp.GyZ / GYRO_SENSITIVITY;

            ESP_LOGI(TAG, "[IMU] Accel → X:%.3fg Y:%.3fg Z:%.3fg", ax, ay, az);
            ESP_LOGI(TAG, "[IMU] Gyro  → X:%.1f  Y:%.1f  Z:%.1f °/s", gx, gy, gz);

            xSemaphoreTake(s_imu_mutex, portMAX_DELAY);
            s_shared_imu = temp;
            xSemaphoreGive(s_imu_mutex);

        } else {
            ESP_LOGE(TAG, "[IMU] Error de lectura");
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

// ════════════════════════════════════════════════
//  Tarea: lectura del GPS
// ════════════════════════════════════════════════

/**
 * @brief Tarea FreeRTOS que lee el GPS de forma continua e independiente.
 *
 * Corre en su propia tarea para que el timeout de espera de tramas NMEA
 * no interfiera con la lectura del IMU ni con la OLED.
 * Actualiza s_shared_gps con el último fix disponible.
 *
 * @param pvParameters Parámetros de la tarea (no utilizados).
 */
static void task_gps(void *pvParameters)
{
    ESP_LOGI(TAG, "[GPS] Tarea iniciada");

    while (1) {
        gps_data_t temp;

        if (gps_read(&temp)) {
            ESP_LOGI(TAG, "[GPS] Lat:%.6f Lon:%.6f Alt:%.1fm Vel:%.1fkm/h",
                     temp.latitude, temp.longitude,
                     temp.altitude, temp.speed);

            xSemaphoreTake(s_gps_mutex, portMAX_DELAY);
            s_shared_gps = temp;
            xSemaphoreGive(s_gps_mutex);

        } else {
            ESP_LOGW(TAG, "[GPS] Sin fix satelital");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// ════════════════════════════════════════════════
//  Tarea: control de la pantalla OLED
// ════════════════════════════════════════════════

/**
 * @brief Tarea FreeRTOS que alterna la pantalla OLED cada 5 segundos.
 *
 * Ciclo de 10 segundos total:
 *  - Primeros 5s: muestra datos del MPU6050 (acelerómetro y giroscopio)
 *  - Últimos  5s: muestra datos del GPS (lat, lon, alt, velocidad)
 *
 * Al separar el control de pantalla en su propia tarea se evita que
 * los delays de visualización bloqueen la adquisición de datos.
 *
 * @param pvParameters Parámetros de la tarea (no utilizados).
 */
static void task_oled(void *pvParameters)
{
    ESP_LOGI(TAG, "[OLED] Tarea iniciada");

    while (1) {

        /* ── 5 segundos mostrando datos del IMU ── */
        {
            mpu_values_t imu;
            xSemaphoreTake(s_imu_mutex, portMAX_DELAY);
            imu = s_shared_imu;
            xSemaphoreGive(s_imu_mutex);

            float ax = imu.AcX / ACCEL_SENSITIVITY;
            float ay = imu.AcY / ACCEL_SENSITIVITY;
            float az = imu.AcZ / ACCEL_SENSITIVITY;
            float gx = imu.GyX / GYRO_SENSITIVITY;
            float gy = imu.GyY / GYRO_SENSITIVITY;
            float gz = imu.GyZ / GYRO_SENSITIVITY;

            oled_show_data(ax, ay, az, gx, gy, gz);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));

        /* ── 5 segundos mostrando datos del GPS ── */
        {
            gps_data_t gps;
            xSemaphoreTake(s_gps_mutex, portMAX_DELAY);
            gps = s_shared_gps;
            xSemaphoreGive(s_gps_mutex);

            oled_show_gps(gps.latitude, gps.longitude,
                          gps.altitude, gps.speed);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ════════════════════════════════════════════════
//  Tarea: envío ESP-NOW
// ════════════════════════════════════════════════

/**
 * @brief Tarea encargada del envío continuo de datos por ESP-NOW.
 *
 * Toma los últimos datos disponibles de IMU y GPS y los transmite
 * al nodo receptor cada SAMPLE_INTERVAL_MS. Opera de forma indefinida
 * sin entrar en Deep Sleep.
 *
 * @param pvParameters Parámetros de la tarea (no utilizados).
 */
static void task_send(void *pvParameters)
{
    ESP_LOGI(TAG, "[SEND] Tarea iniciada");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));

        mpu_values_t imu;
        gps_data_t   gps;

        xSemaphoreTake(s_imu_mutex, portMAX_DELAY);
        imu = s_shared_imu;
        xSemaphoreGive(s_imu_mutex);

        xSemaphoreTake(s_gps_mutex, portMAX_DELAY);
        gps = s_shared_gps;
        xSemaphoreGive(s_gps_mutex);

        ESP_LOGI(TAG, "[SEND] Enviando datos...");
        send_data_espnow(&imu, &gps);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ════════════════════════════════════════════════
//  app_main
// ════════════════════════════════════════════════

/**
 * @brief Función principal del sistema.
 *
 * Esta función representa el punto de entrada del firmware en ESP-IDF.
 * En ella se realizan las siguientes operaciones:
 *
 * - Inicialización del LED indicador
 * - Inicialización del sensor MPU6050 (activa el bus I2C)
 * - Inicialización de la pantalla OLED (reutiliza el bus I2C)
 * - Calibración del sensor MPU6050
 * - Inicialización del módulo GPS (UART2)
 * - Inicialización de la comunicación ESP-NOW
 * - Creación de mutex de sincronización
 * - Creación de tareas FreeRTOS
 *
 * El sistema opera de forma continua e indefinida.
 * El sistema queda gestionado por las tareas:
 * `task_imu`, `task_gps`, `task_oled` y `task_send`.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "*** INICIO NODO 2 ***");

    led_init();
    gpio_set_level(LED_PIN, 1);

    mpu6050_init();      /* Inicializa I2C — debe ir antes que oled_init() */
    oled_init();         /* Reutiliza el bus I2C ya activo                  */
    mpu6050_calibrate();
    gps_init();          /* Inicializa UART2 para el módulo GPS             */
    espnow_init();

    /* Crear mutex separados para IMU y GPS */
    s_imu_mutex = xSemaphoreCreateMutex();
    s_gps_mutex = xSemaphoreCreateMutex();

    if (s_imu_mutex == NULL || s_gps_mutex == NULL) {
        ESP_LOGE(TAG, "Error creando mutex");
        return;
    }

    /* Inicializar GPS compartido en cero */
    s_shared_gps.latitude  = 0.0;
    s_shared_gps.longitude = 0.0;
    s_shared_gps.altitude  = 0.0f;
    s_shared_gps.speed     = 0.0f;
    s_shared_gps.valid     = false;

    /* Inicializar IMU compartido en cero */
    s_shared_imu.AcX = 0; s_shared_imu.AcY = 0; s_shared_imu.AcZ = 0;
    s_shared_imu.GyX = 0; s_shared_imu.GyY = 0; s_shared_imu.GyZ = 0;

    ESP_LOGI(TAG, "Creando tareas...");

    xTaskCreate(task_imu,  "task_imu",  4096, NULL, 5, NULL);
    xTaskCreate(task_gps,  "task_gps",  8192, NULL, 4, NULL);
    xTaskCreate(task_oled, "task_oled", 4096, NULL, 3, NULL);
    xTaskCreate(task_send, "task_send", 4096, NULL, 2, NULL);
}