#include "pulse_oximeter.h"

#include <string.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

static const char *TAG = "MAX30102";

#define MAX30102_ADDRESS 0x57
#define CONTACT_IR_THRESHOLD 2000U
#define CONTACT_LOST_TIMEOUT_MS 1800
#define BPM_HOLD_TIMEOUT_MS 10000
#define SPO2_HOLD_TIMEOUT_MS 12000
#define DEBUG_LOG_INTERVAL_MS 1000
#define PPG_SAMPLE_RATE_HZ 25U
#define MIN_VALID_BPM 40.0f
#define MAX_VALID_BPM 180.0f
#define MIN_VALID_SPO2 90
#define SATURATION_LIMIT 250000U
#define MIN_SPO2_IR_AC 250U
#define MAX_SPO2_IR_AC 20000U
#define RATE_SIZE 8
#define SPO2_BUFFER_SIZE 100
#define MAX_SAMPLES_PER_READ 32
#define SOFT_I2C_DELAY_US 5

#define FIFO_AVG_4 0x40
#define FIFO_ROLLOVER_ENABLE 0x10
#define SPO2_ADC_RANGE_16384NA 0x60
#define SPO2_SAMPLE_RATE_100HZ 0x04
#define SPO2_PULSE_WIDTH_411US 0x03
#define LED_CURRENT_WRIST 0x5F

static const uint8_t REG_INT_STATUS_1 = 0x00;
static const uint8_t REG_FIFO_WR_PTR = 0x04;
static const uint8_t REG_OVF_COUNTER = 0x05;
static const uint8_t REG_FIFO_RD_PTR = 0x06;
static const uint8_t REG_FIFO_DATA = 0x07;
static const uint8_t REG_FIFO_CONFIG = 0x08;
static const uint8_t REG_MODE_CONFIG = 0x09;
static const uint8_t REG_SPO2_CONFIG = 0x0A;
static const uint8_t REG_LED1_PA = 0x0C;
static const uint8_t REG_LED2_PA = 0x0D;
static const uint8_t REG_LED3_PA = 0x0E;
static const uint8_t REG_MULTI_LED_CTRL_1 = 0x11;
static const uint8_t REG_PART_ID = 0xFF;

static bool sensor_ready = false;
static gpio_num_t active_sda = MAX30102_SDA;
static gpio_num_t active_scl = MAX30102_SCL;

static uint8_t rates[RATE_SIZE] = {};
static uint8_t rate_spot = 0;
static uint8_t rate_count = 0;
static uint32_t ppg_sample_index = 0;
static uint32_t last_beat_sample = 0;
static int64_t last_contact_ms = 0;
static int64_t last_good_signal_ms = 0;
static int64_t last_valid_bpm_ms = 0;
static int64_t last_valid_spo2_ms = 0;
static int64_t last_debug_log_ms = 0;
static int beat_avg = 0;
static bool bpm_valid = false;

static uint32_t ir_buffer[SPO2_BUFFER_SIZE] = {};
static uint32_t red_buffer[SPO2_BUFFER_SIZE] = {};
static int buffer_index = 0;
static int32_t spo2 = 0;
static int8_t valid_spo2 = 0;
static int32_t algorithm_spo2 = 0;
static int32_t algorithm_heart_rate = 0;
static int8_t algorithm_spo2_valid = 0;
static int8_t algorithm_heart_rate_valid = 0;

static pulse_oximeter_data_t last_data = {};

static uint32_t ir_window_min = UINT32_MAX;
static uint32_t ir_window_max = 0;
static uint32_t red_window_min = UINT32_MAX;
static uint32_t red_window_max = 0;
static uint16_t signal_window_samples = 0;

/**
 * @brief Genera el retardo base del bus I2C por software.
 */
static inline void soft_i2c_delay(void)
{
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
}

/**
 * @brief Libera la linea SDA dejando que el pull-up la lleve a nivel alto.
 */
static inline void sda_release(void)
{
    gpio_set_level(active_sda, 1);
}

/**
 * @brief Fuerza la linea SDA a nivel bajo.
 */
static inline void sda_low(void)
{
    gpio_set_level(active_sda, 0);
}

/**
 * @brief Libera la linea SCL dejando que el pull-up la lleve a nivel alto.
 */
static inline void scl_release(void)
{
    gpio_set_level(active_scl, 1);
}

/**
 * @brief Fuerza la linea SCL a nivel bajo.
 */
static inline void scl_low(void)
{
    gpio_set_level(active_scl, 0);
}

/**
 * @brief Espera hasta que SCL alcance nivel alto.
 *
 * @return true si SCL sube antes del timeout; false en caso contrario.
 */
static bool scl_wait_high(void)
{
    scl_release();
    for (int i = 0; i < 100; ++i) {
        if (gpio_get_level(active_scl)) {
            return true;
        }
        esp_rom_delay_us(1);
    }
    return false;
}

/**
 * @brief Configura los pines usados por el bus I2C por software.
 */
static void soft_i2c_configure(gpio_num_t sda, gpio_num_t scl)
{
    active_sda = sda;
    active_scl = scl;

    gpio_config_t conf = {};
    conf.pin_bit_mask = (1ULL << sda) | (1ULL << scl);
    conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);

    sda_release();
    scl_release();
    soft_i2c_delay();
}

/**
 * @brief Emite una condicion START en el bus I2C por software.
 */
static void soft_i2c_start(void)
{
    sda_release();
    scl_wait_high();
    soft_i2c_delay();
    sda_low();
    soft_i2c_delay();
    scl_low();
}

/**
 * @brief Emite una condicion STOP en el bus I2C por software.
 */
static void soft_i2c_stop(void)
{
    sda_low();
    soft_i2c_delay();
    scl_wait_high();
    soft_i2c_delay();
    sda_release();
    soft_i2c_delay();
}

/**
 * @brief Escribe un byte en el bus I2C por software.
 *
 * @return true si el dispositivo responde con ACK.
 */
static bool soft_i2c_write_byte(uint8_t value)
{
    for (int bit = 7; bit >= 0; --bit) {
        if (value & (1 << bit)) {
            sda_release();
        } else {
            sda_low();
        }
        soft_i2c_delay();
        scl_wait_high();
        soft_i2c_delay();
        scl_low();
    }

    sda_release();
    soft_i2c_delay();
    scl_wait_high();
    bool ack = (gpio_get_level(active_sda) == 0);
    soft_i2c_delay();
    scl_low();
    return ack;
}

/**
 * @brief Lee un byte desde el bus I2C por software.
 *
 * @param ack Indica si se debe responder ACK despues de la lectura.
 * @return Byte recibido.
 */
static uint8_t soft_i2c_read_byte(bool ack)
{
    uint8_t value = 0;
    sda_release();

    for (int bit = 7; bit >= 0; --bit) {
        soft_i2c_delay();
        scl_wait_high();
        if (gpio_get_level(active_sda)) {
            value |= (1 << bit);
        }
        soft_i2c_delay();
        scl_low();
    }

    if (ack) {
        sda_low();
    } else {
        sda_release();
    }
    soft_i2c_delay();
    scl_wait_high();
    soft_i2c_delay();
    scl_low();
    sda_release();
    return value;
}

/**
 * @brief Verifica si existe un dispositivo I2C en la direccion indicada.
 */
static bool probe_address(uint8_t address)
{
    soft_i2c_start();
    bool ack = soft_i2c_write_byte((address << 1) | 0);
    soft_i2c_stop();
    return ack;
}

/**
 * @brief Escribe un registro del MAX30102.
 */
static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    soft_i2c_start();
    bool ok = soft_i2c_write_byte((MAX30102_ADDRESS << 1) | 0) &&
              soft_i2c_write_byte(reg) &&
              soft_i2c_write_byte(value);
    soft_i2c_stop();
    return ok ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Lee un registro del MAX30102.
 */
static esp_err_t read_reg(uint8_t reg, uint8_t *value)
{
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }

    soft_i2c_start();
    bool ok = soft_i2c_write_byte((MAX30102_ADDRESS << 1) | 0) &&
              soft_i2c_write_byte(reg);
    if (!ok) {
        soft_i2c_stop();
        return ESP_FAIL;
    }

    soft_i2c_start();
    ok = soft_i2c_write_byte((MAX30102_ADDRESS << 1) | 1);
    if (!ok) {
        soft_i2c_stop();
        return ESP_FAIL;
    }

    *value = soft_i2c_read_byte(false);
    soft_i2c_stop();
    return ESP_OK;
}

/**
 * @brief Lee una secuencia de bytes desde un registro del MAX30102.
 */
static esp_err_t read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    soft_i2c_start();
    bool ok = soft_i2c_write_byte((MAX30102_ADDRESS << 1) | 0) &&
              soft_i2c_write_byte(reg);
    if (!ok) {
        soft_i2c_stop();
        return ESP_FAIL;
    }

    soft_i2c_start();
    ok = soft_i2c_write_byte((MAX30102_ADDRESS << 1) | 1);
    if (!ok) {
        soft_i2c_stop();
        return ESP_FAIL;
    }

    for (size_t i = 0; i < len; ++i) {
        data[i] = soft_i2c_read_byte(i + 1 < len);
    }

    soft_i2c_stop();
    return ESP_OK;
}

/**
 * @brief Reinicia los punteros FIFO del MAX30102.
 */
static void clear_fifo(void)
{
    write_reg(REG_FIFO_WR_PTR, 0);
    write_reg(REG_OVF_COUNTER, 0);
    write_reg(REG_FIFO_RD_PTR, 0);
}

/**
 * @brief Configura modo SpO2, FIFO y corrientes LED del MAX30102.
 */
static esp_err_t setup_sensor(void)
{
    write_reg(REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100));

    clear_fifo();
    write_reg(REG_FIFO_CONFIG, FIFO_AVG_4 | FIFO_ROLLOVER_ENABLE);
    write_reg(REG_MODE_CONFIG, 0x03);
    write_reg(REG_SPO2_CONFIG,
              SPO2_ADC_RANGE_16384NA | SPO2_SAMPLE_RATE_100HZ | SPO2_PULSE_WIDTH_411US);
    write_reg(REG_LED1_PA, LED_CURRENT_WRIST);
    write_reg(REG_LED2_PA, LED_CURRENT_WRIST);
    write_reg(REG_LED3_PA, 0x00);
    write_reg(REG_MULTI_LED_CTRL_1, 0x21);

    uint8_t ignored = 0;
    read_reg(REG_INT_STATUS_1, &ignored);
    clear_fifo();
    return ESP_OK;
}

/**
 * @brief Calcula cuantas muestras hay disponibles en FIFO.
 */
static uint8_t fifo_samples_available(void)
{
    uint8_t read_ptr = 0;
    uint8_t write_ptr = 0;
    if (read_reg(REG_FIFO_RD_PTR, &read_ptr) != ESP_OK ||
        read_reg(REG_FIFO_WR_PTR, &write_ptr) != ESP_OK) {
        return 0;
    }

    int samples = (int)write_ptr - (int)read_ptr;
    if (samples < 0) {
        samples += 32;
    }
    return (uint8_t)samples;
}

/**
 * @brief Lee una muestra RED/IR desde el FIFO del MAX30102.
 */
static bool read_fifo_sample(uint32_t *red, uint32_t *ir)
{
    uint8_t buffer[6] = {};
    if (read_bytes(REG_FIFO_DATA, buffer, sizeof(buffer)) != ESP_OK) {
        return false;
    }

    *red = (((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2]) & 0x3FFFF;
    *ir = (((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5]) & 0x3FFFF;
    return true;
}

/**
 * @brief Escanea el bus I2C por software para diagnostico.
 */
static bool scan_i2c_bus(void)
{
    bool found_any = false;
    ESP_LOGI(TAG, "Escaneando I2C por software en SDA GPIO%d / SCL GPIO%d...",
             active_sda, active_scl);

    for (uint8_t address = 0x08; address < 0x78; ++address) {
        if (probe_address(address)) {
            ESP_LOGI(TAG, "Dispositivo I2C encontrado en 0x%02X", address);
            found_any = true;
        }
    }

    if (!found_any) {
        ESP_LOGW(TAG, "No se detecto ningun dispositivo I2C en GPIO%d/GPIO%d",
                 active_sda, active_scl);
    }

    return found_any;
}

/**
 * @brief Busca el MAX30102 en los pares de pines candidatos.
 */
static esp_err_t find_max30102_pins(void)
{
    typedef struct {
        gpio_num_t sda;
        gpio_num_t scl;
    } pin_pair_t;

    const pin_pair_t candidates[] = {
        {GPIO_NUM_15, GPIO_NUM_16},
        {GPIO_NUM_16, GPIO_NUM_15},
        {GPIO_NUM_17, GPIO_NUM_18},
        {GPIO_NUM_18, GPIO_NUM_17},
        {GPIO_NUM_21, GPIO_NUM_33},
        {GPIO_NUM_33, GPIO_NUM_21},
    };

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        soft_i2c_configure(candidates[i].sda, candidates[i].scl);
        ESP_LOGI(TAG, "Probando MAX30102 con I2C por software en SDA GPIO%d / SCL GPIO%d",
                 candidates[i].sda, candidates[i].scl);

        if (!probe_address(MAX30102_ADDRESS)) {
            continue;
        }

        uint8_t part_id = 0;
        if (read_reg(REG_PART_ID, &part_id) == ESP_OK && part_id == 0x15) {
            ESP_LOGI(TAG, "MAX30102 detectado en SDA GPIO%d / SCL GPIO%d",
                     candidates[i].sda, candidates[i].scl);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Hay ACK en 0x57 pero PART_ID=0x%02X en SDA GPIO%d / SCL GPIO%d",
                 part_id, candidates[i].sda, candidates[i].scl);
    }

    return ESP_FAIL;
}

/**
 * @brief Limpia el estado interno de calculo de BPM y SpO2.
 */
static void reset_measurement_state(void)
{
    rate_spot = 0;
    rate_count = 0;
    ppg_sample_index = 0;
    last_beat_sample = 0;
    last_contact_ms = 0;
    last_good_signal_ms = 0;
    last_valid_bpm_ms = 0;
    last_valid_spo2_ms = 0;
    beat_avg = 0;
    bpm_valid = false;
    resetHeartRateDetector();
    buffer_index = 0;
    spo2 = 0;
    valid_spo2 = 0;
    algorithm_spo2 = 0;
    algorithm_heart_rate = 0;
    algorithm_spo2_valid = 0;
    algorithm_heart_rate_valid = 0;
    for (uint8_t i = 0; i < RATE_SIZE; ++i) {
        rates[i] = 0;
    }
}

/**
 * @brief Limpia tambien la medicion publica expuesta al resto del firmware.
 */
static void reset_public_measurement(void)
{
    reset_measurement_state();
    last_data.bpm = 0;
    last_data.spo2 = 0;
    last_data.bpm_valid = false;
    last_data.spo2_valid = false;
    last_data.finger_detected = false;
}

/**
 * @brief Actualiza minimos, maximos y amplitudes AC de la ventana PPG.
 */
static void update_signal_window(uint32_t ir_value, uint32_t red_value)
{
    if (signal_window_samples == 0) {
        ir_window_min = ir_value;
        ir_window_max = ir_value;
        red_window_min = red_value;
        red_window_max = red_value;
    } else {
        if (ir_value < ir_window_min) {
            ir_window_min = ir_value;
        }
        if (ir_value > ir_window_max) {
            ir_window_max = ir_value;
        }
        if (red_value < red_window_min) {
            red_window_min = red_value;
        }
        if (red_value > red_window_max) {
            red_window_max = red_value;
        }
    }

    signal_window_samples++;
    last_data.ir_ac = ir_window_max - ir_window_min;
    last_data.red_ac = red_window_max - red_window_min;
}

/**
 * @brief Reinicia la ventana usada para estimar amplitudes AC.
 */
static void clear_signal_window(void)
{
    signal_window_samples = 0;
    ir_window_min = UINT32_MAX;
    ir_window_max = 0;
    red_window_min = UINT32_MAX;
    red_window_max = 0;
}

/**
 * @brief Evalua si la senal actual cumple condiciones minimas para SpO2.
 */
static bool signal_quality_ok_for_spo2(void)
{
    if (!last_data.finger_detected) {
        return false;
    }
    if (last_data.ir_raw >= SATURATION_LIMIT || last_data.red_raw >= SATURATION_LIMIT) {
        return false;
    }
    if (last_data.ir_ac < MIN_SPO2_IR_AC || last_data.ir_ac > MAX_SPO2_IR_AC) {
        return false;
    }
    return true;
}

/**
 * @brief Verifica si un valor de BPM esta dentro del rango fisiologico esperado.
 */
static bool bpm_in_range(float bpm)
{
    return bpm >= MIN_VALID_BPM && bpm <= MAX_VALID_BPM;
}

/**
 * @brief Acepta una medicion BPM y actualiza el promedio movil.
 */
static void accept_bpm(float bpm, int64_t now_ms)
{
    if (!bpm_in_range(bpm)) {
        return;
    }

    rates[rate_spot++] = (uint8_t)bpm;
    rate_spot %= RATE_SIZE;
    if (rate_count < RATE_SIZE) {
        rate_count++;
    }

    int sum = 0;
    for (uint8_t i = 0; i < rate_count; ++i) {
        sum += rates[i];
    }
    beat_avg = sum / rate_count;
    bpm_valid = rate_count >= 2;
    last_valid_bpm_ms = now_ms;
}

/**
 * @brief Detecta pulsos sobre la senal IR y estima BPM por periodo.
 */
static void update_bpm(uint32_t ir_value, uint32_t sample_index)
{
    if (!checkForBeat((int32_t)ir_value)) {
        return;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    if (last_beat_sample == 0) {
        last_beat_sample = sample_index;
        return;
    }

    uint32_t delta_samples = sample_index - last_beat_sample;
    last_beat_sample = sample_index;
    if (delta_samples == 0) {
        return;
    }

    float bpm = (60.0f * (float)PPG_SAMPLE_RATE_HZ) / (float)delta_samples;
    accept_bpm(bpm, now_ms);
}

/**
 * @brief Alimenta el buffer SpO2 y ejecuta el algoritmo cuando la ventana se llena.
 */
static void update_spo2(uint32_t ir_value, uint32_t red_value)
{
    ir_buffer[buffer_index] = ir_value;
    red_buffer[buffer_index] = red_value;
    buffer_index++;

    if (buffer_index < SPO2_BUFFER_SIZE) {
        return;
    }

    int32_t heart_rate = 0;
    int8_t valid_heart_rate = 0;
    int32_t computed_spo2 = 0;
    int8_t computed_valid_spo2 = 0;

    maxim_heart_rate_and_oxygen_saturation(ir_buffer,
                                           SPO2_BUFFER_SIZE,
                                           red_buffer,
                                           &computed_spo2,
                                           &computed_valid_spo2,
                                           &heart_rate,
                                           &valid_heart_rate);
    algorithm_spo2 = computed_spo2;
    algorithm_heart_rate = heart_rate;
    algorithm_spo2_valid = computed_valid_spo2;
    algorithm_heart_rate_valid = valid_heart_rate;

    int64_t now_ms = esp_timer_get_time() / 1000;
    if (valid_heart_rate && signal_quality_ok_for_spo2()) {
        accept_bpm((float)heart_rate, now_ms);
    }

    static int32_t last_spo2 = 95;
    if (computed_valid_spo2 &&
        computed_spo2 >= MIN_VALID_SPO2 &&
        computed_spo2 <= 100 &&
        signal_quality_ok_for_spo2()) {
        spo2 = (computed_spo2 + last_spo2) / 2;
        last_spo2 = spo2;
        valid_spo2 = 1;
        last_valid_spo2_ms = now_ms;
    }

    buffer_index = 0;
}

/**
 * @copydoc pulse_oximeter_init
 */
extern "C" esp_err_t pulse_oximeter_init(void)
{
    esp_err_t ret = find_max30102_pins();
    if (ret != ESP_OK) {
        scan_i2c_bus();
        ESP_LOGE(TAG, "MAX30102 no detectado con I2C por software en los pares probados");
        return ret;
    }

    uint8_t part_id = 0;
    if (read_reg(REG_PART_ID, &part_id) != ESP_OK || part_id != 0x15) {
        ESP_LOGE(TAG, "MAX30102 no encontrado en SDA GPIO%d / SCL GPIO%d. PART_ID leido: 0x%02X",
                 active_sda, active_scl, part_id);
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(setup_sensor());

    reset_measurement_state();
    memset(&last_data, 0, sizeof(last_data));
    sensor_ready = true;
    ESP_LOGI(TAG, "MAX30102 listo en SDA GPIO%d / SCL GPIO%d", active_sda, active_scl);
    return ESP_OK;
}

/**
 * @copydoc pulse_oximeter_read
 */
extern "C" esp_err_t pulse_oximeter_read(pulse_oximeter_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensor_ready) {
        memset(data, 0, sizeof(*data));
        return ESP_FAIL;
    }

    uint16_t samples = fifo_samples_available();
    if (samples > MAX_SAMPLES_PER_READ) {
        samples = MAX_SAMPLES_PER_READ;
    }

    for (uint16_t i = 0; i < samples; ++i) {
        int64_t sample_now_ms = esp_timer_get_time() / 1000;
        uint32_t red_value = 0;
        uint32_t ir_value = 0;
        if (!read_fifo_sample(&red_value, &ir_value)) {
            break;
        }

        ppg_sample_index++;
        update_signal_window(ir_value, red_value);

        bool contact = ir_value > CONTACT_IR_THRESHOLD;
        last_data.ir_raw = ir_value;
        last_data.red_raw = red_value;

        if (!contact) {
            bool contact_lost = last_contact_ms == 0 ||
                                (sample_now_ms - last_contact_ms) > CONTACT_LOST_TIMEOUT_MS;
            if (contact_lost) {
                reset_public_measurement();
            }
            continue;
        }

        last_contact_ms = sample_now_ms;
        last_data.finger_detected = true;
        if (signal_quality_ok_for_spo2()) {
            last_good_signal_ms = sample_now_ms;
        }

        update_bpm(ir_value, ppg_sample_index);
        update_spo2(ir_value, red_value);
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    bool contact_recent = last_contact_ms > 0 &&
                          (now_ms - last_contact_ms) <= CONTACT_LOST_TIMEOUT_MS;
    bool good_signal_recent = last_good_signal_ms > 0 &&
                              (now_ms - last_good_signal_ms) <= SPO2_HOLD_TIMEOUT_MS;
    bool bpm_is_recent = last_valid_bpm_ms > 0 &&
                         (now_ms - last_valid_bpm_ms) <= BPM_HOLD_TIMEOUT_MS;
    bool spo2_is_recent = last_valid_spo2_ms > 0 &&
                          (now_ms - last_valid_spo2_ms) <= SPO2_HOLD_TIMEOUT_MS;

    if (!contact_recent || !bpm_is_recent) {
        bpm_valid = false;
        beat_avg = 0;
    }
    if (!contact_recent || !spo2_is_recent) {
        valid_spo2 = 0;
        spo2 = 0;
    }

    last_data.finger_detected = contact_recent;
    last_data.bpm_valid = bpm_valid && bpm_is_recent && contact_recent;
    last_data.spo2_valid = valid_spo2 &&
                            spo2_is_recent &&
                            contact_recent &&
                            last_data.bpm_valid &&
                            good_signal_recent;
    last_data.bpm = last_data.bpm_valid ? beat_avg : 0;
    last_data.spo2 = last_data.spo2_valid ? (int)spo2 : 0;

    if ((now_ms - last_debug_log_ms) >= DEBUG_LOG_INTERVAL_MS) {
        ESP_LOGI(TAG,
                 "PPG raw IR=%lu RED=%lu AC_IR=%lu AC_RED=%lu"
                 " contacto=%d BPM=%d valid=%d SpO2=%d valid=%d algHR=%ld algHRv=%d algSpO2=%ld algSpO2v=%d q=%d",
                 (unsigned long)last_data.ir_raw,
                 (unsigned long)last_data.red_raw,
                 (unsigned long)last_data.ir_ac,
                 (unsigned long)last_data.red_ac,
                 last_data.finger_detected,
                 last_data.bpm,
                 last_data.bpm_valid,
                 last_data.spo2,
                 last_data.spo2_valid,
                 (long)algorithm_heart_rate,
                 algorithm_heart_rate_valid,
                 (long)algorithm_spo2,
                 algorithm_spo2_valid,
                 signal_quality_ok_for_spo2());
        last_debug_log_ms = now_ms;
        clear_signal_window();
    }

    *data = last_data;
    return ESP_OK;
}
