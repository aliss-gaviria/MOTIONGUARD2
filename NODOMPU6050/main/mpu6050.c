/**
 * @file mpu6050.c
 * @brief Implementación del driver para el sensor MPU6050.
 *
 * Este archivo contiene las funciones necesarias para:
 * - Inicializar el sensor MPU6050 mediante I2C
 * - Leer los datos crudos del acelerómetro y giroscopio
 * - Realizar calibración del sensor
 * - Obtener valores corregidos utilizando los offsets calculados
 */

#include "mpu6050.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "MPU6050";

/* Direcciones de registros internos del MPU6050 */
#define REG_PWR_MGMT_1    0x6B
#define REG_ACCEL_CONFIG  0x1C
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_XOUT    0x3B

/* Configuración del bus I2C */
#define I2C_PORT          I2C_NUM_0
#define ACK_CHECK_EN      true
#define ACK_VAL           0x00
#define NACK_VAL          0x01

/**
 * @brief Estructura para almacenar los offsets obtenidos durante la calibración.
 */
typedef struct {
    float ax_off, ay_off, az_off; // offsets del acelerómetro
    float gx_off, gy_off, gz_off; // offsets del giroscopio
    bool  done;                   // indica si la calibración fue realizada
} calib_t;

/* Variable global que almacena los offsets de calibración */
static calib_t s_calib = {0};


/**
 * @brief Escribe un byte en un registro del MPU6050 mediante I2C.
 *
 * @param reg Registro del sensor
 * @param data Dato a escribir
 * @return esp_err_t Estado de la operación
 */
static esp_err_t mpu_write(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg,  ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return ret;
}


/**
 * @brief Lee uno o más bytes desde un registro del MPU6050.
 *
 * @param reg Registro inicial de lectura
 * @param out Buffer donde se almacenarán los datos
 * @param len Número de bytes a leer
 * @return esp_err_t Estado de la operación
 */
static esp_err_t mpu_read(uint8_t reg, uint8_t *out, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    /* Se indica al sensor el registro desde donde se empezará a leer */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return ret;

    /* Lectura de los datos solicitados */
    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    if (len > 1)
        i2c_master_read(cmd, out, len - 1, ACK_VAL);

    i2c_master_read_byte(cmd, out + len - 1, NACK_VAL);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return ret;
}


/**
 * @brief Convierte dos bytes (MSB y LSB) en un entero de 16 bits.
 */
static int16_t bytes_to_int(uint8_t msb, uint8_t lsb)
{
    return (int16_t)((msb << 8) | lsb);
}


/**
 * @brief Lee los valores crudos del sensor MPU6050.
 *
 * Se leen los 14 bytes del bloque de registros ACCEL_XOUT.
 * Los bytes raw[6] y raw[7] corresponden a temperatura y se
 * descartan intencionalmente sin procesarse.
 */
static esp_err_t mpu_read_raw(mpu_values_t *v)
{
    uint8_t raw[14];

    esp_err_t ret = mpu_read(REG_ACCEL_XOUT, raw, 14);
    if (ret != ESP_OK) return ret;

    v->AcX = bytes_to_int(raw[0],  raw[1]);
    v->AcY = bytes_to_int(raw[2],  raw[3]);
    v->AcZ = bytes_to_int(raw[4],  raw[5]);

    /* raw[6] y raw[7] = temperatura — descartados intencionalmente */

    v->GyX = bytes_to_int(raw[8],  raw[9]);
    v->GyY = bytes_to_int(raw[10], raw[11]);
    v->GyZ = bytes_to_int(raw[12], raw[13]);

    return ESP_OK;
}


/**
 * @brief Inicializa el sensor MPU6050 y el bus I2C.
 *
 * @note El bus I2C inicializado aquí es compartido con la pantalla OLED.
 *       Por ello, oled_init() debe llamarse después de esta función.
 */
void mpu6050_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_IO,
        .scl_io_num       = I2C_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    /* Despertar el sensor */
    ESP_ERROR_CHECK(mpu_write(REG_PWR_MGMT_1,  0x00));
    vTaskDelay(pdMS_TO_TICKS(150));

    /* Configuración de rangos del acelerómetro y giroscopio */
    ESP_ERROR_CHECK(mpu_write(REG_ACCEL_CONFIG, 0x00));
    ESP_ERROR_CHECK(mpu_write(REG_GYRO_CONFIG,  0x00));

    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "MPU6050 listo");
}


/**
 * @brief Realiza la calibración del sensor.
 *
 * Calcula los offsets del acelerómetro y giroscopio
 * promediando múltiples muestras con el sensor en reposo.
 */
void mpu6050_calibrate(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, " CALIBRACIÓN (%d muestras)", CALIB_SAMPLES);
    ESP_LOGI(TAG, " Mantenga el sensor en reposo y horizontal...");
    ESP_LOGI(TAG, "==============================================");

    vTaskDelay(pdMS_TO_TICKS(2000));

    int64_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int valid = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {

        mpu_values_t v;

        if (mpu_read_raw(&v) == ESP_OK) {
            sum_ax += v.AcX;
            sum_ay += v.AcY;
            sum_az += v.AcZ;

            sum_gx += v.GyX;
            sum_gy += v.GyY;
            sum_gz += v.GyZ;

            valid++;
        }

        vTaskDelay(pdMS_TO_TICKS(CALIB_DELAY_MS));
    }

    if (valid == 0) {
        ESP_LOGE(TAG, "Calibración fallida");
        return;
    }

    /* Cálculo de offsets */
    s_calib.ax_off = (float)sum_ax / valid;
    s_calib.ay_off = (float)sum_ay / valid;
    s_calib.az_off = (float)sum_az / valid - ACCEL_SENSITIVITY;

    s_calib.gx_off = (float)sum_gx / valid;
    s_calib.gy_off = (float)sum_gy / valid;
    s_calib.gz_off = (float)sum_gz / valid;

    s_calib.done   = true;

    ESP_LOGI(TAG, " Calibración completa (%d muestras válidas)", valid);
}


/**
 * @brief Obtiene los valores del sensor aplicando los offsets de calibración.
 */
esp_err_t mpu_get_values(mpu_values_t *v)
{
    esp_err_t ret = mpu_read_raw(v);
    if (ret != ESP_OK) return ret;

    /* Se corrigen los valores si ya se realizó la calibración */
    if (s_calib.done) {
        v->AcX = (int16_t)(v->AcX - s_calib.ax_off);
        v->AcY = (int16_t)(v->AcY - s_calib.ay_off);
        v->AcZ = (int16_t)(v->AcZ - s_calib.az_off);

        v->GyX = (int16_t)(v->GyX - s_calib.gx_off);
        v->GyY = (int16_t)(v->GyY - s_calib.gy_off);
        v->GyZ = (int16_t)(v->GyZ - s_calib.gz_off);
    }

    return ESP_OK;
}