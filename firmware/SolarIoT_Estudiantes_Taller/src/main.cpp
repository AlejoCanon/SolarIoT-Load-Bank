/**
 * ============================================================
 *  Taller IEE 1 — Módulo Solar Fotovoltaico IoT
 *  MINIPROYECTO — Código base del estudiante
 * ============================================================
 *
 *  Descripción:
 *    La ESP32 lee un sensor analógico (LDR o potenciómetro)
 *    que simula la irradiancia solar o la demanda energética.
 *    En función de esa lectura, decide qué configuración de
 *    carga activar y publica el comando vía MQTT al Web Master.
 *
 *    Simultáneamente, se suscribe al tópico de estado del
 *    Web Master y refleja en sus 6 LEDs la configuración
 *    confirmada por el sistema. Así se verifica el ciclo
 *    completo sin necesitar la carga real de la planta solar.
 *
 *  Tópicos MQTT:
 *    Publicación : lab/solar/G{NUMERO_GRUPO}/cmd  <- comandos
 *    Suscripción : lab/solar/estado               <- confirmación
 *
 *  Payload publicado (cmd):
 *    {"ch":3,"val":80}   ch = canal (1-6), val = porcentaje (0-100)
 *
 *  Payload recibido (estado del Web Master):
 *    {"v":[0,50,100,25,0,75],"g":3}
 *
 *  Conexiones de hardware:
 *    Sensor analógico (LDR o potenciómetro) -> GPIO 34
 *    Canal 1 LED -> GPIO 13 -> R 220ohm -> GND
 *    Canal 2 LED -> GPIO 12 -> R 220ohm -> GND
 *    Canal 3 LED -> GPIO 14 -> R 220ohm -> GND
 *    Canal 4 LED -> GPIO 27 -> R 220ohm -> GND
 *    Canal 5 LED -> GPIO 26 -> R 220ohm -> GND
 *    Canal 6 LED -> GPIO 25 -> R 220ohm -> GND
 *
 *  Librerías (platformio.ini):
 *    lib_deps =
 *        knolleary/PubSubClient @ ^2.8
 *        bblanchon/ArduinoJson @ ^6.21
 *        adafruit/DHT sensor library @ ^1.4
 *        adafruit/Adafruit Unified Sensor @ ^1.1
 *
 *  Universidad de los Andes — Departamento IEE
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ------------------------------------------------------------
//  CONFIGURACIÓN — editar antes de cargar
// ------------------------------------------------------------

// *** CAMBIAR AL NÚMERO DE SU GRUPO (1 al 10) ***
#define NUMERO_GRUPO    7

// Credenciales — ver secrets.h (copiar desde secrets.h.example y rellenar)
#include "secrets.h"

// Intervalo entre lecturas del sensor y envío de comandos (ms)
// No enviar más frecuente de 5 s para no saturar el broker
#define INTERVALO_SENSOR_MS  10000

// Intervalo de reconexión MQTT (ms)
#define INTERVALO_RECONEXION_MS  5000

// ------------------------------------------------------------
//  HARDWARE
// ------------------------------------------------------------
#define PIN_SENSOR      34    // GPIO analógico — LDR o potenciómetro
#define ADC_MAX         4095  // Resolución ADC del ESP32 (12 bits)

const int NUM_CANALES            = 6;
const int LED_PINES[NUM_CANALES] = {13, 12, 14, 27, 26, 25};
const int PWM_FRECUENCIA         = 5000;
const int PWM_RESOLUCION         = 8;

// Estado confirmado por el Web Master (lo que realmente está activo)
int estadoConfirmado[NUM_CANALES] = {0};

// ------------------------------------------------------------
//  TÓPICOS MQTT (se construyen en setup)
// ------------------------------------------------------------
char TOPICO_CMD[40];
const char TOPICO_ESTADO[] = "lab/solar/estado";
char CLIENT_ID[32];

// ------------------------------------------------------------
//  OBJETOS DE RED
// ------------------------------------------------------------
WiFiClientSecure espClient;
PubSubClient     mqtt(espClient);

uint32_t ultimoSensor    = 0;
uint32_t ultimoIntento   = 0;

// ============================================================
//  PWM — CONTROL DE LEDs
// ============================================================
void inicializarPWM() {
    for (int i = 0; i < NUM_CANALES; i++) {
        ledcSetup(i, PWM_FRECUENCIA, PWM_RESOLUCION);
        ledcAttachPin(LED_PINES[i], i);
        ledcWrite(i, 0);
    }
}

/** Aplica un porcentaje (0-100) al LED del canal dado (índice 0-5). */
void aplicarLED(int canal, int porcent) {
    estadoConfirmado[canal] = constrain(porcent, 0, 100);
    ledcWrite(canal, map(estadoConfirmado[canal], 0, 100, 0, 255));
}


// ============================================================
//  LÓGICA DE SENSOR → CONFIGURACIÓN DE CARGA
// ============================================================

/**
 * Estructura que representa un comando de canal.
 */
struct Comando {
    int canal;  // 1-6
    int valor;  // 0-100
};

/**
 * Lee el sensor y decide cuántos canales activar y a qué nivel.
 *
 * LÓGICA BASE (los estudiantes deben mejorarla):
 *   El sensor representa la irradiancia solar (0 = oscuridad, 4095 = sol pleno).
 *   Cuando hay más irradiancia, se activan más cargas para aprovechar el
 *   excedente de generación — principio de gestión activa de demanda.
 *
 *   Niveles:
 *     0-20%   irradiancia  -> 0 canales activos   (sistema inactivo)
 *     21-40%  irradiancia  -> 2 canales al 50%
 *     41-60%  irradiancia  -> 4 canales al 75%
 *     61-80%  irradiancia  -> 6 canales al 75%
 *     81-100% irradiancia  -> 6 canales al 100%
 *
 * @param lectura  Valor ADC bruto (0-4095)
 * @param cmds     Array de comandos a rellenar
 * @return         Número de comandos generados
 */
int decidirConfiguracion(int lectura, Comando cmds[]) {
    float pct = (float)lectura / ADC_MAX * 100.0f;  // Porcentaje 0-100
    int n = 0;

    // Apagar todos primero
    for (int i = 1; i <= NUM_CANALES; i++) {
        cmds[n++] = {i, 0};
    }
    n = 0;  // Reiniciar y construir solo los comandos necesarios

    if (pct <= 20.0f) {
        // Sin generacion suficiente: apagar todo
        for (int i = 1; i <= NUM_CANALES; i++) cmds[n++] = {i, 0};

    } else if (pct <= 40.0f) {
        cmds[n++] = {1, 50};
        cmds[n++] = {2, 50};
        // Canales 3-6 apagados
        for (int i = 3; i <= NUM_CANALES; i++) cmds[n++] = {i, 0};

    } else if (pct <= 60.0f) {
        for (int i = 1; i <= 4; i++) cmds[n++] = {i, 75};
        cmds[n++] = {5, 0};
        cmds[n++] = {6, 0};

    } else if (pct <= 80.0f) {
        for (int i = 1; i <= NUM_CANALES; i++) cmds[n++] = {i, 75};

    } else {
        for (int i = 1; i <= NUM_CANALES; i++) cmds[n++] = {i, 100};
    }

    return n;
}


// ============================================================
//  MQTT
// ============================================================

/**
 * Callback: llega un mensaje de estado del Web Master.
 * Payload: {"v":[0,50,100,25,0,75],"g":3}
 * Actualiza los LEDs con el estado CONFIRMADO por el maestro.
 */
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, payload, length)) return;

    // Verificar que el mensaje sea del grupo correcto
    int grupoMsg = doc["g"] | 0;
    if (grupoMsg != NUMERO_GRUPO) return;  // Ignorar si no es nuestro grupo

    JsonArray arr = doc["v"].as<JsonArray>();
    if (arr.isNull() || arr.size() != NUM_CANALES) return;

    Serial.println("Estado confirmado por Web Master:");
    for (int i = 0; i < NUM_CANALES; i++) {
        int v = arr[i] | 0;
        aplicarLED(i, v);
        Serial.printf("  Canal %d: %d%%\n", i + 1, v);
    }
}

/**
 * Publica un comando al tópico del grupo.
 * Payload: {"ch":canal,"val":valor}
 */
void publicarComando(int canal, int valor) {
    if (!mqtt.connected()) {
        Serial.println("MQTT desconectado — comando no enviado.");
        return;
    }

    StaticJsonDocument<64> doc;
    doc["ch"]  = canal;
    doc["val"] = valor;

    char buf[64];
    serializeJson(doc, buf);
    bool ok = mqtt.publish(TOPICO_CMD, buf);
    Serial.printf("MQTT pub -> Canal %d = %d%% [%s]\n",
                  canal, valor, ok ? "OK" : "FALLO");
}

/** (Re)conecta al broker y suscribe al tópico de estado. */
void reconectarMQTT() {
    if (millis() - ultimoIntento < INTERVALO_RECONEXION_MS) return;
    ultimoIntento = millis();

    Serial.print("Conectando a HiveMQ... ");
    if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS)) {
        Serial.println("conectado.");
        mqtt.subscribe(TOPICO_ESTADO, 1);
        Serial.printf("Suscrito a: %s\n", TOPICO_ESTADO);
    } else {
        Serial.printf("fallo rc=%d\n", mqtt.state());
    }
}


// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.printf("\n=== Miniproyecto Solar — Grupo %d ===\n", NUMERO_GRUPO);

    // Construir tópico y client ID únicos por grupo
    snprintf(TOPICO_CMD, sizeof(TOPICO_CMD),
             "lab/solar/G%d/cmd", NUMERO_GRUPO);
    snprintf(CLIENT_ID, sizeof(CLIENT_ID),
             "SolarG%d_%04X", NUMERO_GRUPO, (uint16_t)ESP.getEfuseMac());

    Serial.printf("Topico de publicacion : %s\n", TOPICO_CMD);
    Serial.printf("Topico de suscripcion : %s\n", TOPICO_ESTADO);

    // LEDs
    inicializarPWM();
    analogSetAttenuation(ADC_11db);   // Rango completo del ADC (0-3.3 V)

    // WiFi
    Serial.printf("Conectando a WiFi: %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.printf("\nWiFi OK — IP: %s\n", WiFi.localIP().toString().c_str());

    // MQTT
    espClient.setInsecure();   // TLS sin verificacion de certificado
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(callbackMQTT);
    mqtt.setBufferSize(256);
}


// ============================================================
//  LOOP
// ============================================================
void loop() {
    // Mantener conexion MQTT
    if (!mqtt.connected()) reconectarMQTT();
    mqtt.loop();

    // Cada INTERVALO_SENSOR_MS: leer sensor y publicar comandos
    if (millis() - ultimoSensor >= INTERVALO_SENSOR_MS) {
        ultimoSensor = millis();

        int lectura = analogRead(PIN_SENSOR);
        float pct   = (float)lectura / ADC_MAX * 100.0f;

        Serial.printf("\n--- Lectura sensor: %d (%.1f%%) ---\n",
                      lectura, pct);

        // Decidir configuracion
        Comando cmds[6];
        int n = decidirConfiguracion(lectura, cmds);

        // Publicar cada comando con una pequeña pausa entre envios
        for (int i = 0; i < n; i++) {
            publicarComando(cmds[i].canal, cmds[i].valor);
            delay(100);   // Evitar saturar el broker
        }
    }
}
