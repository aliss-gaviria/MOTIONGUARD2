idf_component_register(
    SRCS
        "main.c"
        "imu.cpp"
        "pulse_oximeter.cpp"
        "../managed_components/heartRate/heartRate.cpp"
        "../managed_components/spo2/spo2_algorithm.cpp"
        "espnow_comm.c"
        "display_lcd.cpp"

    INCLUDE_DIRS
        "."
        "../managed_components/heartRate"
        "../managed_components/spo2"

    REQUIRES
        driver
        esp_lcd
        esp_timer
        esp_wifi
        nvs_flash
        esp_event
        espressif__esp_lcd_gc9a01
        espp__qmi8658
)
