#include "imu.h"

#include <memory>
#include <system_error>

#include "board_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "qmi8658.hpp"

static const char *TAG = "IMU";

#define CALIBRATION_SAMPLES 120
#define READ_AVG_SAMPLES 20
#define FILTER_ALPHA 0.80f

static i2c_master_bus_handle_t bus_handle = NULL;
static std::shared_ptr<espp::Qmi8658<>> imu;
static int64_t last_update_us = 0;
static bool calibration_ready = false;
static imu_data_t offsets = {};
static imu_data_t filtered = {};

/**
 * @brief Aplica un filtro pasa bajos de primer orden a una lectura.
 *
 * @param previous Valor filtrado anterior.
 * @param current Nueva muestra sin filtrar.
 * @return Valor filtrado actualizado.
 */
static float apply_low_pass(float previous, float current)
{
    return previous + FILTER_ALPHA * (current - previous);
}

/**
 * @brief Actualiza el driver QMI8658 respetando el intervalo real entre muestras.
 *
 * @return ESP_OK si el driver entrego una muestra valida; ESP_FAIL si falla.
 */
static esp_err_t imu_update_driver(void)
{
    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - last_update_us) / 1000000.0f;
    if (dt <= 0.0f) {
        dt = 0.01f;
    }
    last_update_us = now_us;

    std::error_code ec;
    if (!imu->update(dt, ec)) {
        ESP_LOGW(TAG, "No se pudo leer QMI8658: %s", ec.message().c_str());
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Calcula offsets de acelerometro y giroscopio con la placa en reposo.
 *
 * @return ESP_OK si obtuvo muestras validas; ESP_FAIL si no hubo lecturas.
 */
static esp_err_t calibrate_imu(void)
{
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    int valid_samples = 0;

    ESP_LOGI(TAG, "Calibrando IMU: mantenga la placa quieta...");

    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        if (imu_update_driver() == ESP_OK) {
            auto accel = imu->get_accelerometer();
            auto gyro = imu->get_gyroscope();

            ax += accel.x;
            ay += accel.y;
            az += accel.z;
            gx += gyro.x;
            gy += gyro.y;
            gz += gyro.z;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (valid_samples == 0) {
        return ESP_FAIL;
    }

    ax /= valid_samples;
    ay /= valid_samples;
    az /= valid_samples;
    gx /= valid_samples;
    gy /= valid_samples;
    gz /= valid_samples;

    offsets.ax = ax;
    offsets.ay = ay;
    offsets.az = (az >= 0.0f) ? (az - 1.0f) : (az + 1.0f);
    offsets.gx = gx;
    offsets.gy = gy;
    offsets.gz = gz;

    filtered.ax = ax - offsets.ax;
    filtered.ay = ay - offsets.ay;
    filtered.az = az - offsets.az;
    filtered.gx = 0.0f;
    filtered.gy = 0.0f;
    filtered.gz = 0.0f;
    calibration_ready = true;

    ESP_LOGI(TAG,
             "Offsets IMU | ACC %.3f %.3f %.3f | GYR %.3f %.3f %.3f",
             offsets.ax, offsets.ay, offsets.az,
             offsets.gx, offsets.gy, offsets.gz);

    return ESP_OK;
}

/**
 * @copydoc imu_init
 */
extern "C" esp_err_t imu_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)I2C_SDA,
        .scl_io_num = (gpio_num_t)I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = espp::Qmi8658<>::DEFAULT_ADDRESS,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags = {},
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    espp::Qmi8658<>::Config config = {
        .device_address = espp::Qmi8658<>::DEFAULT_ADDRESS,
        .write = [dev_handle](uint8_t, const uint8_t *data, size_t len) -> bool {
            return i2c_master_transmit(dev_handle, data, len, 100) == ESP_OK;
        },
        .read = [dev_handle](uint8_t, uint8_t *data, size_t len) -> bool {
            return i2c_master_receive(dev_handle, data, len, 100) == ESP_OK;
        },
        .imu_config = {
            .accelerometer_range = espp::Qmi8658<>::AccelerometerRange::RANGE_8G,
            .accelerometer_odr = espp::Qmi8658<>::ODR::ODR_250_HZ,
            .gyroscope_range = espp::Qmi8658<>::GyroscopeRange::RANGE_512_DPS,
            .gyroscope_odr = espp::Qmi8658<>::ODR::ODR_250_HZ,
        },
        .orientation_filter = nullptr,
        .auto_init = false,
        .log_level = espp::Logger::Verbosity::WARN,
    };

    imu = std::make_shared<espp::Qmi8658<>>(config);

    std::error_code ec;
    if (!imu->init(ec)) {
        ESP_LOGE(TAG, "No se pudo inicializar QMI8658: %s", ec.message().c_str());
        return ESP_FAIL;
    }

    last_update_us = esp_timer_get_time();
    ESP_ERROR_CHECK(calibrate_imu());
    ESP_LOGI(TAG, "QMI8658 inicializado correctamente");
    return ESP_OK;
}

/**
 * @copydoc imu_read
 */
extern "C" esp_err_t imu_read(imu_data_t *data)
{
    if (!imu || !data) {
        return ESP_FAIL;
    }

    if (!calibration_ready) {
        return ESP_FAIL;
    }

    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    int valid_samples = 0;

    for (int i = 0; i < READ_AVG_SAMPLES; ++i) {
        if (imu_update_driver() == ESP_OK) {
            auto accel = imu->get_accelerometer();
            auto gyro = imu->get_gyroscope();

            ax += accel.x - offsets.ax;
            ay += accel.y - offsets.ay;
            az += accel.z - offsets.az;
            gx += gyro.x - offsets.gx;
            gy += gyro.y - offsets.gy;
            gz += gyro.z - offsets.gz;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (valid_samples == 0) {
        return ESP_FAIL;
    }

    ax /= valid_samples;
    ay /= valid_samples;
    az /= valid_samples;
    gx /= valid_samples;
    gy /= valid_samples;
    gz /= valid_samples;

    filtered.ax = apply_low_pass(filtered.ax, ax);
    filtered.ay = apply_low_pass(filtered.ay, ay);
    filtered.az = apply_low_pass(filtered.az, az);
    filtered.gx = apply_low_pass(filtered.gx, gx);
    filtered.gy = apply_low_pass(filtered.gy, gy);
    filtered.gz = apply_low_pass(filtered.gz, gz);

    data->ax = filtered.ax;
    data->ay = filtered.ay;
    data->az = filtered.az;
    data->gx = filtered.gx;
    data->gy = filtered.gy;
    data->gz = filtered.gz;

    return ESP_OK;
}
