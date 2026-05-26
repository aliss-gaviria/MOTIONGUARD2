# Registro del componente principal del proyecto
idf_component_register(
    SRCS
        "main.c"          # Punto de entrada y tareas FreeRTOS
        "mpu6050.c"       # Driver MPU6050 (I2C + calibración)
        "espnow_comm.c"   # Comunicación ESP-NOW
        "oled.c"          # Driver pantalla OLED SSD1306 128x64
        "gps.c"           # Driver GPS NEO-6M/7M/8M (UART2, NMEA)
    INCLUDE_DIRS
        "."               # Permite incluir headers del propio directorio main/
    REQUIRES
        driver            # API I2C legacy (driver/i2c.h)
        esp_driver_gpio   # GPIO (driver/gpio.h)
        esp_driver_uart   # UART para el módulo GPS
        esp_wifi          # Wi-Fi + ESP-NOW
        esp_event         # Sistema de eventos
        esp_netif         # Abstracción de red
        nvs_flash         # Almacenamiento no volátil (requerido por Wi-Fi)
        esp_hw_support    # Deep sleep y funciones de hardware
        esp_timer         # Timer de alta resolución (usado en gps_read)
)