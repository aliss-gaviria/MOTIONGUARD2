#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display_lcd.h"
#include "espnow_comm.h"
#include "imu.h"
#include "pulse_oximeter.h"

/**
 * @brief Punto de entrada principal de la aplicacion.
 *
 * Inicializa pantalla, comunicacion ESP-NOW, IMU y pulsioximetro. En el ciclo
 * principal actualiza las lecturas, muestra los datos en LCD y transmite la
 * trama consolidada del nodo.
 */
void app_main(void)
{
    lcd_init_display();
    espnow_init();

    if (imu_init() != ESP_OK)
    {
        printf("No se pudo inicializar la IMU\n");
    }

    if (pulse_oximeter_init() != ESP_OK)
    {
        printf("No se pudo inicializar el MAX30102\n");
    }

    imu_data_t data;
    pulse_oximeter_data_t pulse;

    while (1)
    {
        for (int i = 0; i < 100; ++i)
        {
            pulse_oximeter_read(&pulse);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (imu_read(&data) == ESP_OK)
        {
            printf("\n=== IMU ===\n");

            printf("ACC X: %.2f\n", data.ax);
            printf("ACC Y: %.2f\n", data.ay);
            printf("ACC Z: %.2f\n", data.az);

            printf("GYR X: %.2f\n", data.gx);
            printf("GYR Y: %.2f\n", data.gy);
            printf("GYR Z: %.2f\n", data.gz);
            printf("BPM: %d | SpO2: %d | contacto: %d | IR: %lu | RED: %lu | AC_IR: %lu | AC_RED: %lu\n",
                   pulse.bpm,
                   pulse.spo2,
                   pulse.finger_detected,
                   (unsigned long)pulse.ir_raw,
                   (unsigned long)pulse.red_raw,
                   (unsigned long)pulse.ir_ac,
                   (unsigned long)pulse.red_ac);

            lcd_show_imu_data(data.ax, data.ay, data.az,
                              data.gx, data.gy, data.gz,
                              pulse.bpm, pulse.spo2,
                              pulse.bpm_valid, pulse.spo2_valid,
                              pulse.finger_detected);
            espnow_send_node1(data.ax, data.ay, data.az,
                              data.gx, data.gy, data.gz,
                              pulse.spo2, pulse.bpm);
        }
    }
}
