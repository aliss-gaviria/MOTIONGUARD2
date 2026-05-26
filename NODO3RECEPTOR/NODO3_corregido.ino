#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>

// =========================
// Pines y configuracion
// =========================

#define LED_PULSO 25
#define LED_SPO2 26
#define LED_MOV 16

const char *ssid = "Galaxy S20 FEE8DE";
const char *password = "vdbf8047";

const char *serverHost = "10.119.68.39";
const char *serverURL = "http://10.119.68.39/sistema/insertar.php";
const char *umbralURL = "http://10.119.68.39/sistema/obtener_umbrales.php";

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0,
    U8X8_PIN_NONE,
    4, // SCL
    5  // SDA
);

// =========================
// Datos del sistema
// =========================

typedef struct
{
    float bpm = 0;
    float spo2 = 0;

    float ax1 = 0;
    float ay1 = 0;
    float az1 = 0;
    float gx1 = 0;
    float gy1 = 0;
    float gz1 = 0;

    float ax2 = 0;
    float ay2 = 0;
    float az2 = 0;
    float gx2 = 0;
    float gy2 = 0;
    float gz2 = 0;

    double latitud = 0;
    double longitud = 0;
    float altitud = 0;
    float velocidad = 0;
} DatosSistema;

typedef struct
{
    char texto[512];
} MensajeEspNow;

DatosSistema datos;

SemaphoreHandle_t xMutex;
QueueHandle_t xColaMensajes;

bool nodo1Pendiente = false;
bool nodo2Pendiente = false;
bool nodo1Valido = false;
bool nodo2Valido = false;

float pulsoMax = 120;
float pulsoMin = 60;
float spo2Min = 90;
float spo2Max = 100;
float movimientoAccelMax = 0.5;
float movimientoGyroMax = 10;

bool alertaPulso = false;
bool alertaSpo2 = false;
bool alertaMovimiento = false;

const unsigned long DURACION_MENSAJE_EVENTO_MS = 3000;
const int MAX_EVENTOS_OLED = 8;
String eventoColaLinea1[MAX_EVENTOS_OLED];
String eventoColaLinea2[MAX_EVENTOS_OLED];
String eventoColaLinea3[MAX_EVENTOS_OLED];
String eventoColaLinea4[MAX_EVENTOS_OLED];
int eventoColaInicio = 0;
int eventoColaCantidad = 0;
bool eventoActivo = false;
String eventoLinea1 = "";
String eventoLinea2 = "";
String eventoLinea3 = "";
String eventoLinea4 = "";
unsigned long eventoHasta = 0;

unsigned long inicioPulso = 0;
unsigned long inicioSpo2 = 0;
unsigned long inicioMovimiento = 0;

int contadorPulsoNormal = 0;
int contadorSpo2Normal = 0;
int contadorMovimientoBajo = 0;

// =========================
// OLED
// =========================

void mostrarOLED(String linea1, String linea2 = "", String linea3 = "", String linea4 = "")
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, linea1.c_str());
    u8g2.drawStr(0, 28, linea2.c_str());
    u8g2.drawStr(0, 44, linea3.c_str());
    u8g2.drawStr(0, 60, linea4.c_str());
    u8g2.sendBuffer();
}

bool hayAlertasActivas()
{
    bool hayAlertas;

    xSemaphoreTake(xMutex, portMAX_DELAY);
    hayAlertas = alertaPulso || alertaSpo2 || alertaMovimiento;
    xSemaphoreGive(xMutex);

    return hayAlertas;
}

void mostrarEventoSiNoHayAlertas(String linea1, String linea2 = "", String linea3 = "", String linea4 = "")
{
    xSemaphoreTake(xMutex, portMAX_DELAY);

    bool hayAlertas = alertaPulso || alertaSpo2 || alertaMovimiento;

    if (!hayAlertas)
    {
        if (eventoColaCantidad >= MAX_EVENTOS_OLED)
        {
            eventoColaInicio = (eventoColaInicio + 1) % MAX_EVENTOS_OLED;
            eventoColaCantidad--;
        }

        int posicion = (eventoColaInicio + eventoColaCantidad) % MAX_EVENTOS_OLED;
        eventoColaLinea1[posicion] = linea1;
        eventoColaLinea2[posicion] = linea2;
        eventoColaLinea3[posicion] = linea3;
        eventoColaLinea4[posicion] = linea4;
        eventoColaCantidad++;
    }

    xSemaphoreGive(xMutex);
}

// =========================
// Alertas
// =========================

void actualizarAlertaPulso(float bpm)
{
    if (bpm > pulsoMax)
    {
        if (inicioPulso == 0)
        {
            inicioPulso = millis();
        }

        if (millis() - inicioPulso >= 60000)
        {
            alertaPulso = true;
        }

        contadorPulsoNormal = 0;
        return;
    }

    inicioPulso = 0;

    if (alertaPulso && bpm < pulsoMax)
    {
        contadorPulsoNormal++;

        if (contadorPulsoNormal >= 3)
        {
            alertaPulso = false;
            contadorPulsoNormal = 0;
        }
    }
    else if (alertaPulso)
    {
        contadorPulsoNormal = 0;
    }
}

void actualizarAlertaSpo2(float spo2)
{
    if (spo2 > spo2Max)
    {
        if (inicioSpo2 == 0)
        {
            inicioSpo2 = millis();
        }

        if (millis() - inicioSpo2 >= 120000)
        {
            alertaSpo2 = true;
        }

        contadorSpo2Normal = 0;
        return;
    }

    inicioSpo2 = 0;

    if (alertaSpo2 && spo2 < spo2Max)
    {
        contadorSpo2Normal++;

        if (contadorSpo2Normal >= 5)
        {
            alertaSpo2 = false;
            contadorSpo2Normal = 0;
        }
    }
    else if (alertaSpo2)
    {
        contadorSpo2Normal = 0;
    }
}

void actualizarAlertaMovimiento()
{
    if (!nodo1Valido || !nodo2Valido)
    {
        return;
    }

    float accel1 = sqrt(datos.ax1 * datos.ax1 + datos.ay1 * datos.ay1 + datos.az1 * datos.az1);
    float accel2 = sqrt(datos.ax2 * datos.ax2 + datos.ay2 * datos.ay2 + datos.az2 * datos.az2);

    bool movimientoAlto =
        accel1 > movimientoAccelMax ||
        accel2 > movimientoAccelMax ;


    if (movimientoAlto)
    {
        if (inicioMovimiento == 0)
        {
            inicioMovimiento = millis();
        }

        if (millis() - inicioMovimiento >= 30000)
        {
            alertaMovimiento = true;
        }

        contadorMovimientoBajo = 0;
        return;
    }

    inicioMovimiento = 0;

    if (alertaMovimiento)
    {
        contadorMovimientoBajo++;

        if (contadorMovimientoBajo >= 2)
        {
            alertaMovimiento = false;
            contadorMovimientoBajo = 0;
        }
    }
}

// =========================
// Servidor
// =========================

void enviarServidor()
{
    DatosSistema copia;

    xSemaphoreTake(xMutex, portMAX_DELAY);
    copia = datos;
    xSemaphoreGive(xMutex);

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi desconectado. No se envio al servidor.");
        return;
    }

    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

    JsonDocument doc;
    doc["bpm"] = copia.bpm;
    doc["spo2"] = copia.spo2;
    doc["ax1"] = copia.ax1;
    doc["ay1"] = copia.ay1;
    doc["az1"] = copia.az1;
    doc["gx1"] = copia.gx1;
    doc["gy1"] = copia.gy1;
    doc["gz1"] = copia.gz1;
    doc["ax2"] = copia.ax2;
    doc["ay2"] = copia.ay2;
    doc["az2"] = copia.az2;
    doc["gx2"] = copia.gx2;
    doc["gy2"] = copia.gy2;
    doc["gz2"] = copia.gz2;
    doc["latitud"] = copia.latitud;
    doc["longitud"] = copia.longitud;
    doc["altitud"] = copia.altitud;
    doc["velocidad"] = copia.velocidad;

    String json;
    serializeJson(doc, json);

    Serial.println("\nJSON ENVIADO:");
    Serial.println(json);

    int httpCode = http.POST(json);
    String respuesta = http.getString();

    Serial.print("HTTP CODE: ");
    Serial.println(httpCode);
    Serial.println("RESPUESTA:");
    Serial.println(respuesta);

    http.end();

    if (httpCode > 0)
    {
        mostrarEventoSiNoHayAlertas("DATOS ENVIADOS", "Servidor OK");
    }
    else
    {
        mostrarEventoSiNoHayAlertas("ERROR ENVIO", "Servidor");
    }
}

void consultarUmbrales()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return;
    }

    HTTPClient http;
    http.begin(umbralURL);

    int code = http.GET();

    if (code == 200)
    {
        String payload = http.getString();
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload);

        if (error)
        {
            Serial.print("Error JSON umbrales: ");
            Serial.println(error.c_str());
            http.end();
            return;
        }

        xSemaphoreTake(xMutex, portMAX_DELAY);

        if (!doc["pulso_max"].isNull())
        {
            pulsoMax = doc["pulso_max"].as<float>();
        }

        if (!doc["pulso_min"].isNull())
        {
            pulsoMin = doc["pulso_min"].as<float>();
        }

        if (!doc["spo2_max"].isNull())
        {
            spo2Max = doc["spo2_max"].as<float>();
        }

        if (!doc["spo2_min"].isNull())
        {
            spo2Min = doc["spo2_min"].as<float>();
        }

        if (!doc["movimiento_accel_max"].isNull())
        {
            movimientoAccelMax = doc["movimiento_accel_max"].as<float>();
        }

        if (!doc["movimiento_gyro_max"].isNull())
        {
            movimientoGyroMax = doc["movimiento_gyro_max"].as<float>();
        }

        xSemaphoreGive(xMutex);

        Serial.println("Umbrales actualizados");
        Serial.println("spo2Min:");
        Serial.println(spo2Min);
        Serial.println("spo2Max:");
        Serial.println(spo2Max);
        Serial.println("pulsoMin:");
        Serial.println(pulsoMin);
        Serial.println("pulsoMax:");
        Serial.println(pulsoMax);
        Serial.println("movimientoAccelMax:");
        Serial.println(movimientoAccelMax);
        Serial.println("movimientoGyroMax:");
        Serial.println(movimientoGyroMax);

    }
    else
    {
        Serial.print("Error consultando umbrales. HTTP: ");
        Serial.println(code);
    }

    http.end();
}

// =========================
// Recepcion y procesamiento
// =========================

void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
{
    MensajeEspNow mensaje;

    if (len >= (int)sizeof(mensaje.texto))
    {
        len = sizeof(mensaje.texto) - 1;
    }

    memcpy(mensaje.texto, incomingData, len);
    mensaje.texto[len] = '\0';

    if (xColaMensajes != NULL)
    {
        xQueueSend(xColaMensajes, &mensaje, 0);
    }

    Serial.printf("RECIBIDO DE: %02X:%02X:%02X:%02X:%02X:%02X\n",
              recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
              recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
}

void procesarNodo1(char *msg)
{
    // Formato real del Nodo 1:
    // 1,ax,ay,az,gx,gy,gz,spo2,bpm
    int nodo = 0;
    float ax = 0;
    float ay = 0;
    float az = 0;
    float gx = 0;
    float gy = 0;
    float gz = 0;
    float spo2 = 0;
    float bpm = 0;

    int result = sscanf(
        msg,
        "%d,%f,%f,%f,%f,%f,%f,%f,%f",
        &nodo,
        &ax,
        &ay,
        &az,
        &gx,
        &gy,
        &gz,
        &spo2,
        &bpm);

    if (result != 9)
    {
        Serial.println("ERROR PARSE NODO 1");
        return;
    }

    xSemaphoreTake(xMutex, portMAX_DELAY);

    datos.bpm = bpm;
    datos.spo2 = spo2;
    datos.ax1 = ax;
    datos.ay1 = ay;
    datos.az1 = az;
    datos.gx1 = gx;
    datos.gy1 = gy;
    datos.gz1 = gz;

    nodo1Pendiente = true;
    nodo1Valido = true;

    actualizarAlertaPulso(bpm);
    actualizarAlertaSpo2(spo2);
    actualizarAlertaMovimiento();

    xSemaphoreGive(xMutex);

    Serial.println("===== DATOS NODO 1 =====");
    Serial.print("BPM: ");
    Serial.println(bpm);
    Serial.print("SPO2: ");
    Serial.println(spo2);

    mostrarEventoSiNoHayAlertas(
        "NODO 1",
        "BPM: " + String(bpm, 1),
        "SPO2: " + String(spo2, 1),
        "Datos OK");
}

void procesarNodo2(char *msg)
{
    int nodo = 0;
    float ax = 0;
    float ay = 0;
    float az = 0;
    float gx = 0;
    float gy = 0;
    float gz = 0;
    double lat = 0;
    double lon = 0;
    float alt = 0;
    float vel = 0;

    int result = sscanf(
        msg,
        "%d,%f,%f,%f,%f,%f,%f,%lf,%lf,%f,%f",
        &nodo,
        &ax,
        &ay,
        &az,
        &gx,
        &gy,
        &gz,
        &lat,
        &lon,
        &alt,
        &vel);

    if (result != 11)
    {
        Serial.println("ERROR PARSE NODO 2");
        return;
    }

    xSemaphoreTake(xMutex, portMAX_DELAY);

    datos.ax2 = ax;
    datos.ay2 = ay;
    datos.az2 = az;
    datos.gx2 = gx;
    datos.gy2 = gy;
    datos.gz2 = gz;
    datos.latitud = lat;
    datos.longitud = lon;
    datos.altitud = alt;
    datos.velocidad = vel;

    nodo2Pendiente = true;
    nodo2Valido = true;

    actualizarAlertaMovimiento();

    xSemaphoreGive(xMutex);

    Serial.println("===== DATOS NODO 2 =====");
    Serial.print("LATITUD: ");
    Serial.println(lat, 6);
    Serial.print("LONGITUD: ");
    Serial.println(lon, 6);
    Serial.print("VELOCIDAD: ");
    Serial.println(vel, 2);

    mostrarEventoSiNoHayAlertas(
        "NODO 2",
        "LAT:" + String(lat, 6),
        "LON:" + String(lon, 6),
        "Datos OK");
}

void TaskRecepcion(void *pvParameters)
{
    MensajeEspNow mensaje;

    while (true)
    {
        if (xQueueReceive(xColaMensajes, &mensaje, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("\n======================");
            Serial.println("MENSAJE RECIBIDO:");
            Serial.println(mensaje.texto);
            Serial.println("======================");

            if (strlen(mensaje.texto) == 0)
            {
                Serial.println("Mensaje vacio");
                continue;
            }

            int nodo = 0;
            sscanf(mensaje.texto, "%d,", &nodo);

            if (nodo == 1)
            {
                procesarNodo1(mensaje.texto);
            }
            else if (nodo == 2)
            {
                procesarNodo2(mensaje.texto);
            }
            else
            {
                Serial.println("Nodo desconocido");
                continue;
            }

            bool enviarRegistro = false;

            xSemaphoreTake(xMutex, portMAX_DELAY);

            if (nodo1Pendiente && nodo2Pendiente)
            {
                nodo1Pendiente = false;
                nodo2Pendiente = false;
                enviarRegistro = true;
            }

            xSemaphoreGive(xMutex);

            if (enviarRegistro)
            {
                Serial.println("Ambos nodos recibidos. Enviando registro al servidor...");
                enviarServidor();
            }
        }
    }
}

// =========================
// Tareas FreeRTOS
// =========================

void TaskAlertas(void *pvParameters)
{
    while (true)
    {
        bool pulso;
        bool spo2;
        bool mov;

        xSemaphoreTake(xMutex, portMAX_DELAY);
        pulso = alertaPulso;
        spo2 = alertaSpo2;
        mov = alertaMovimiento;
        xSemaphoreGive(xMutex);

        digitalWrite(LED_PULSO, pulso ? HIGH : LOW);
        digitalWrite(LED_SPO2, spo2 ? HIGH : LOW);
        digitalWrite(LED_MOV, mov ? HIGH : LOW);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void TaskOLED(void *pvParameters)
{
    while (true)
    {
        bool pulso;
        bool spo2Alerta;
        bool mov;
        float bpm;
        float spo2;
        bool mostrarEvento = false;
        String ev1 = "";
        String ev2 = "";
        String ev3 = "";
        String ev4 = "";

        xSemaphoreTake(xMutex, portMAX_DELAY);
        pulso = alertaPulso;
        spo2Alerta = alertaSpo2;
        mov = alertaMovimiento;
        bpm = datos.bpm;
        spo2 = datos.spo2;

        if (!pulso && !spo2Alerta && !mov && eventoActivo && millis() < eventoHasta)
        {
            mostrarEvento = true;
            ev1 = eventoLinea1;
            ev2 = eventoLinea2;
            ev3 = eventoLinea3;
            ev4 = eventoLinea4;
        }
        else if (!pulso && !spo2Alerta && !mov && eventoColaCantidad > 0)
        {
            eventoLinea1 = eventoColaLinea1[eventoColaInicio];
            eventoLinea2 = eventoColaLinea2[eventoColaInicio];
            eventoLinea3 = eventoColaLinea3[eventoColaInicio];
            eventoLinea4 = eventoColaLinea4[eventoColaInicio];
            eventoColaInicio = (eventoColaInicio + 1) % MAX_EVENTOS_OLED;
            eventoColaCantidad--;
            eventoActivo = true;
            eventoHasta = millis() + DURACION_MENSAJE_EVENTO_MS;

            mostrarEvento = true;
            ev1 = eventoLinea1;
            ev2 = eventoLinea2;
            ev3 = eventoLinea3;
            ev4 = eventoLinea4;
        }
        else
        {
            eventoActivo = false;
            eventoHasta = 0;
        }

        xSemaphoreGive(xMutex);

        if (pulso || spo2Alerta || mov)
        {
            String l1 = "ALERTAS ACTIVAS";
            String l2 = pulso ? "PULSO " + String(bpm, 0) + " BPM" : "";
            String l3 = spo2Alerta ? "OXIG " + String(spo2, 0) + " %" : "";
            String l4 = mov ? "ALTO NIVEL MOVIM" : "";

            if (!pulso && spo2Alerta && mov)
            {
                l2 = "ALERTA OXIGENACION";
                l3 = String(spo2, 0) + " %";
                l4 = "ALTO NIVEL MOVIM";
            }
            else if (pulso && !spo2Alerta && mov)
            {
                l2 = "ALERTA PULSO";
                l3 = String(bpm, 0) + " BPM";
                l4 = "ALTO NIVEL MOVIM";
            }
            else if (pulso && spo2Alerta && !mov)
            {
                l2 = "PULSO " + String(bpm, 0) + " BPM";
                l3 = "OXIG " + String(spo2, 0) + " %";
                l4 = "";
            }
            else if (pulso && !spo2Alerta && !mov)
            {
                l1 = "ALERTA PULSO";
                l2 = String(bpm, 0) + " BPM";
            }
            else if (!pulso && spo2Alerta && !mov)
            {
                l1 = "ALERTA OXIGENACION";
                l2 = String(spo2, 0) + " %";
            }
            else if (!pulso && !spo2Alerta && mov)
            {
                l1 = "ALTO NIVEL MOVIM";
            }

            mostrarOLED(l1, l2, l3, l4);
        }
        else if (mostrarEvento)
        {
            mostrarOLED(ev1, ev2, ev3, ev4);
        }
        else
        {
            mostrarOLED(
                "SISTEMA OK",
                "BPM: " + String(bpm, 0),
                "SPO2: " + String(spo2, 0) + " %",
                "Esperando datos");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TaskUmbrales(void *pvParameters)
{
    while (true)
    {
        consultarUmbrales();
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

// =========================
// Setup
// =========================

void setup()
{
    Serial.begin(115200);

    pinMode(LED_PULSO, OUTPUT);
    pinMode(LED_SPO2, OUTPUT);
    pinMode(LED_MOV, OUTPUT);

    u8g2.begin();
    mostrarOLED("NODO 3", "Iniciando...");

    xMutex = xSemaphoreCreateMutex();
    xColaMensajes = xQueueCreate(10, sizeof(MensajeEspNow));

    if (xMutex == NULL || xColaMensajes == NULL)
    {
        Serial.println("ERROR creando mutex o cola");
        mostrarOLED("ERROR", "FreeRTOS");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print("Conectando WiFi");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WIFI CONECTADO");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC NODO 3: ");
    Serial.println(WiFi.macAddress());
    Serial.print("CANAL WIFI NODO 3: ");
    Serial.println(WiFi.channel());

    WiFiClient testClient;
    Serial.println("Probando servidor...");

    if (testClient.connect(serverHost, 80))
    {
        Serial.println("SERVIDOR ALCANZABLE");
        testClient.stop();
    }
    else
    {
        Serial.println("NO SE PUEDE LLEGAR AL SERVIDOR");
    }

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ERROR ESPNOW");
        mostrarOLED("ERROR", "ESP NOW");
        return;
    }


    esp_now_register_recv_cb(onDataRecv);

    Serial.println("ESP-NOW OK");
    consultarUmbrales();
    mostrarOLED("NODO 3", "Esperando", "datos...");

    xTaskCreatePinnedToCore(TaskRecepcion, "Recepcion", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskAlertas, "Alertas", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TaskOLED, "OLED", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TaskUmbrales, "Umbrales", 4096, NULL, 1, NULL, 0);
}

void loop()
{
    if (Serial.available())
    {
        String linea = Serial.readStringUntil('\n');
        linea.trim();

        if (linea.length() > 0)
        {
            MensajeEspNow mensaje;
            linea.toCharArray(mensaje.texto, sizeof(mensaje.texto));

            if (xColaMensajes != NULL)
            {
                xQueueSend(xColaMensajes, &mensaje, pdMS_TO_TICKS(100));
                Serial.println("Mensaje de prueba agregado a la cola:");
                Serial.println(mensaje.texto);
            }
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}
