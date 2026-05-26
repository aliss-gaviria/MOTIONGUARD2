# Nodo 1 Embebidos

Firmware para un nodo de sensores basado en ESP-IDF. El sistema lee datos de
movimiento desde una IMU QMI8658, calcula frecuencia cardiaca y SpO2 con un
MAX30102, muestra las mediciones en una pantalla LCD GC9A01 y envia una trama
consolidada mediante ESP-NOW.

## Funcionamiento general

Al iniciar, `app_main()` configura la pantalla, ESP-NOW, la IMU y el
pulsioximetro. Despues entra en un ciclo continuo donde actualiza el sensor
MAX30102, lee una muestra filtrada de la IMU, imprime datos de diagnostico por
consola, actualiza la pantalla LCD y transmite la informacion del nodo.

La trama ESP-NOW enviada por `espnow_send_node1()` tiene el formato:

```text
1,ax,ay,az,gx,gy,gz,spo2,bpm
```

El primer campo identifica el nodo. Los siguientes seis campos corresponden a
aceleracion y giroscopio, y los dos ultimos a saturacion de oxigeno y frecuencia
cardiaca.

## Archivos del proyecto

### `main/main.c`

Contiene el punto de entrada del firmware. Coordina la inicializacion de los
modulos y ejecuta el ciclo principal de adquisicion, visualizacion y transmision.

### `main/board_config.h`

Centraliza la configuracion de hardware: pines de la pantalla LCD, pines I2C,
canal Wi-Fi y direccion MAC del receptor ESP-NOW.

### `main/imu.h` y `main/imu.cpp`

Implementan la interfaz de la IMU QMI8658. El modulo inicializa el bus I2C,
configura acelerometro y giroscopio, calcula offsets de calibracion y entrega
lecturas promediadas con filtro pasa bajos.

### `main/pulse_oximeter.h` y `main/pulse_oximeter.cpp`

Implementan el manejo del MAX30102 mediante I2C por software. El modulo detecta
los pines activos, configura el sensor, lee muestras FIFO, estima BPM, calcula
SpO2 y reporta banderas de validez para evitar mostrar o transmitir valores no
confiables como mediciones validas.

### `main/display_lcd.h` y `main/display_lcd.cpp`

Inicializan la pantalla GC9A01 por SPI y dibujan una interfaz compacta con los
valores de aceleracion, giroscopio, BPM y SpO2. El renderizado se hace con
primitivas simples de rectangulos y una fuente bitmap interna.

### `main/espnow_comm.h` y `main/espnow_comm.c`

Configuran Wi-Fi en modo estacion, inicializan ESP-NOW, registran el receptor y
envian la trama completa del nodo 1. La funcion antigua que enviaba solo datos
de IMU fue eliminada porque el firmware actual transmite la trama consolidada
con IMU, BPM y SpO2.

### `main/CMakeLists.txt`

Declara los archivos fuente del componente principal, rutas de inclusion y
dependencias ESP-IDF necesarias para compilar el firmware.

### `partitions.csv`, `sdkconfig` y `dependencies.lock`

Contienen la configuracion del proyecto ESP-IDF, el esquema de particiones y las
versiones bloqueadas de los componentes administrados.

### `managed_components/`

Directorio generado por el administrador de componentes de ESP-IDF. Incluye
dependencias externas como LVGL, el driver GC9A01, la libreria QMI8658 y los
algoritmos auxiliares de frecuencia cardiaca y SpO2. Normalmente no se edita
directamente.

## Documentacion Doxygen

Las interfaces publicas en `main/*.h` documentan el contrato de cada modulo. En
los archivos `main/*.c` y `main/*.cpp` tambien se documentan las funciones
internas relevantes, especialmente callbacks, procesamiento de senal, dibujo en
LCD y comunicacion de bajo nivel. Esto permite generar documentacion completa
sin llenar las implementaciones con comentarios de bajo valor.

## Compilacion

Desde la raiz del proyecto, con ESP-IDF configurado:

```bash
idf.py build
```

Para cargar el firmware en la placa:

```bash
idf.py flash monitor
```
