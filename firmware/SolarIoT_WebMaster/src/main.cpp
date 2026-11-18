/**
 * ============================================================
 *  SolarIoT_WebMaster v3.4 — MQTT + I2C + sensores + modos + arriendo
 * ============================================================
 *
 *  Base: firmware Diego Cañón (v1). CONSERVA la arquitectura original:
 *  mqttTask fijada al núcleo 0, HTTP y sensores en el núcleo 1, colas y un
 *  mutex de estado.
 *
 *  Todo lo que sigue son cambios frente a v1, el firmware en operación hoy.
 *  Cada uno está marcado [Mn] en el punto del código donde ocurre.
 *
 *  CAMBIOS FRENTE A v1 (marcados [Mn] en el código):
 *
 *   [M1] MODOS AUTOMÁTICO / AULA, persistentes y REALES.
 *
 *        Advertencia importante: el botón "Control Manual / Modo MQTT" de la
 *        página v1 NO era un modo. Vivía entero en el navegador
 *        (`let modoManual=false`), y lo único que hacía era dejar de pisar la
 *        posición de los deslizadores con los valores consultados. El ESP32
 *        seguía escuchando MQTT todo el tiempo. Es decir que "Control Manual"
 *        no le quitaba el mando a nadie: daba la impresión de exclusividad sin
 *        implementarla.
 *
 *        Aquí el modo vive en el firmware, se guarda en NVS y decide a qué se
 *        suscribe el equipo:
 *          MODO_AUTO (por defecto): escucha SIEMPRE lab/solar/auto/cmd, sin
 *              depender del selector de grupo. Es el modo programático, el que
 *              usa el SCADA. El control manual por web queda DESHABILITADO.
 *          MODO_AULA: escucha el grupo seleccionado, como hasta ahora, y el
 *              control manual por web está habilitado.
 *        Se cambia por web o publicando en lab/solar/modo/cmd.
 *
 *   [M2] PERSISTENCIA en NVS de modo, grupo y tope de potencia. En v1
 *        grupoActivo arrancaba en 0 y con grupo 0 no había suscripción a
 *        ningún tópico: cada reinicio dejaba el banco sordo. Esa es la causa
 *        de que "el grupo se pierda solo".
 *
 *   [M3] LAST WILL. El tópico de estado se publica retenido, y sin Last Will
 *        un equipo muerto deja para siempre un mensaje que dice
 *        {"v":[0,0,0,0,0,0],"g":0}, indistinguible de un banco sano en reposo.
 *
 *   [M4] AUTENTICACIÓN por token en los puntos de control. En v1 cualquiera
 *        en la red podía llamar /fans?on=0 y apagar la refrigeración.
 *
 *   [M5] PERROS GUARDIANES Y BROWNOUT REACTIVADOS. v1 desactivaba los tres,
 *        que son justamente los mecanismos que permiten recuperarse solo.
 *
 *   [M6] SUPERVISIÓN DE WIFI. v1 conectaba una vez en setup() y no volvía a
 *        mirar; y si la conexión inicial fallaba, pasaba a modo AP de forma
 *        permanente y nunca reintentaba la red real.
 *
 *   [M7] TEMPERATURA HACIA EL ESCLAVO. El termopar está aquí pero la
 *        protección térmica vive en el esclavo, para que sobreviva a la caída
 *        de este equipo. Se envía cada segundo.
 *
 *   [M8] LECTURA DE VUELTA POR I2C. El esclavo ya publicaba sus potencias en
 *        onI2CRequest() y v1 nunca las pedía: si el esclavo se reiniciaba,
 *        este equipo seguía reportando valores viejos mientras la carga estaba
 *        apagada. Ahora se compara contra lo ESPERADO y se reporta divergencia.
 *
 *   ---- CAMBIOS QUE ACOMPAÑAN A LAS PROTECCIONES NUEVAS DEL ESCLAVO ----
 *
 *   [M9]  VALIDEZ DEL TERMOPAR. En v1 el MAX6675 se lee cada segundo, se
 *         muestra y se publica, pero no actúa sobre nada. Al hacerlo actuar
 *         (protección térmica en el esclavo, [C5] y [C11]) hay que resolver un
 *         problema que en v1 no importaba: `if (!isnan(c)) resistorTemp = c;`
 *         conserva la última lectura buena para siempre. Eso está bien para
 *         una pantalla, y es inaceptable para una protección: tras UNA lectura
 *         buena, un termopar desconectado quedaría oculto indefinidamente.
 *         Aquí una lectura inválida sostenida se propaga como INVÁLIDA
 *         explícita, y el esclavo deja de conducir.
 *
 *   [M10] ENCLAVAMIENTO DE VENTILADORES. En v1 los ventiladores son una
 *         perilla independiente y se puede energizar la carga y olvidarlos.
 *         Aquí son una función DERIVADA de la carga. Encender siempre se
 *         obedece; apagar es una petición que solo se honra cuando es seguro.
 *
 *   [M11] SIN RESURRECCIÓN DE SETPOINTS. Un esclavo o un maestro que se
 *         reinicia no debe volver solo a la potencia que había antes. v1 no
 *         tenía este problema porque no leía de vuelta al esclavo; al añadir la
 *         lectura de vuelta [M8] hay que decidir explícitamente qué hacer ante
 *         un reinicio, y la respuesta correcta es exigir un comando nuevo.
 *
 *   [M12] DIVERGENCIA CONTRA LO ESPERADO, no solo "aplica de más". La
 *         expectativa se construye con todo lo que legítimamente reduce la
 *         potencia (tope, térmico, fase muerta, hombre muerto), de modo que un
 *         comando I2C perdido se detecte en vez de declararse sano.
 *
 *   [M13] Cola I2C FIFO en vez de la de profundidad 1 con xQueueOverwrite de
 *         v1, que descarta comandos cuando HTTP y MQTT llegan juntos. Además:
 *         NVS serializada y solo ante cambio real, `modo` bajo mutex, y timeout
 *         de socket por debajo del perro guardián.
 *
 *   ---- v3 (Cañón, sep 2026) ----
 *
 *   [M14] ARRIENDO DEL MANDO SUPERIOR. CORRIGE EL FALLO QUE IMPEDIA OPERAR.
 *         En v2 el hombre muerto del esclavo solo se renovaba con opcodes de
 *         canal, y este equipo solo los envia cuando llega una orden. Sin
 *         reenvio periodico, a los 30 s de cada comando el arriendo vencia, se
 *         REVOCABA la potencia y no volvia. El banco moria solo tras cada
 *         comando. Ahora se releva la frescura del mando superior con
 *         CMD_LEASE cada segundo, sin reenviar setpoints (reenviarlos habria
 *         vuelto decorativa la proteccion, que es lo que v2 queria evitar).
 *         POLITICA: el banco permanece activo mientras la pagina siga abierta;
 *         el sondeo de /values cuenta como presencia. Tope de sesion de 4 h
 *         desde la ultima ORDEN real, para que una pestana olvidada no
 *         sostenga la carga indefinidamente.
 *
 *   [M15] Opcodes para el modo de conmutacion del esclavo (angulo de fase /
 *         ciclos integrales) y para la fuente de sincronismo de una fase.
 *
 *   [M16] MAPEO FISICO FASE->LINEA declarado y publicado. Las etiquetas R/S/T
 *         del firmware son del cableado del banco y no coinciden con L1/L2/L3
 *         del inversor: por eso el mismo fallo se reporto como "canales 1 y 2"
 *         y como "L3". La conexion es fija, asi que la correspondencia se
 *         declara una vez y viaja en /values y en MQTT.
 *
 *   [M17] Modo por defecto AULA en vez de AUTO, con migracion de NVS. En AUTO
 *         el equipo solo escucha lab/solar/auto/cmd y deshabilita el control
 *         manual, de modo que con la infraestructura actual quedaba sordo y
 *         sin control manual a la vez.
 *
 *  ---- v3.1 (González Mancera, 2026-09-09) ----
 *
 *   [M18] EL ARRIENDO SE RENOVABA SOLO. Es el cambio importante de esta
 *         version y corrige un fallo de seguridad introducido en v3.
 *
 *         La intencion de [M14] esta bien y el encabezado del esclavo la
 *         enuncia con precision: «si el SCADA muere, el maestro deja de
 *         afirmar el arriendo y el banco cae». El maestro no hacia eso.
 *         `marcarPresencia()` se llamaba desde DOS sitios que no son el mando
 *         superior:
 *           - `manejarValues()`, es decir CUALQUIER lectura de /values. El
 *             colector del SCADA la sondea cada 10 s, las 24 h, solo para
 *             registrar la temperatura: la vigilancia pasiva sostenia la
 *             actuacion.
 *           - `publicarEstado()`, el latido propio del equipo, cada 3 s,
 *             incondicionalmente. Con eso el ESP32 se renovaba el arriendo a
 *             si mismo y `arriendoActivo()` se reducia a «hubo una orden en
 *             las ultimas 4 h». El hombre muerto no existia.
 *
 *         La politica de [M14] se CONSERVA —mientras la pagina siga abierta el
 *         banco permanece activo— pero ahora hay que declararla: POST
 *         /presencia, que la pagina llama cada 10 s. La diferencia es que es
 *         una afirmacion deliberada de quien quiere la carga encendida, y no
 *         un efecto colateral de leer. Un sistema en el que MIRAR mantiene la
 *         carga encendida no tiene hombre muerto.
 *
 *   [M19] El equipo publica QUE es y DONDE esta: `fw` e `ip` en /values y en
 *         el estado MQTT. El 2026-09-08 hubo que deducir la version mirando
 *         que campos traia el payload, y el 2026-09-09 se barrio un /24 entero
 *         buscando este equipo, sin encontrarlo, mientras publicaba su estado
 *         sin interrupcion.
 *
 *   [M20] API_TOKEN: faltaba en secrets.h.example, de modo que copiar el
 *         ejemplo daba un fichero con el que el maestro NO COMPILA. Ahora esta,
 *         y si se deja el valor de relleno se avisa por consola al arrancar y
 *         se publica `token_default` en el estado. El valor de ejemplo esta en
 *         el repositorio: dejarlo puesto es no tener token.
 *
 *  NOTA SOBRE LAS ETIQUETAS: [M14]-[M17] de v3 (arriendo, opcodes del esclavo,
 *  mapeo fase->linea, modo AULA por defecto) son de Cañon. Un borrador paralelo
 *  de González Mancera uso [M14]-[M17] para otras cosas; ese borrador queda
 *  descartado y aqui manda esta numeracion. Lo suyo continua en [M18]-[M20].
 *
 *  ---- v3.2 (González Mancera, 2026-09-10) ----
 *
 *   [M21] EL ARRIENDO SE ENVIABA A CIEGAS. Es la correccion que motiva esta
 *         version. El 2026-09-10 el banco se apago solo tres veces con TODAS
 *         las inhibiciones en claro; el registro del SCADA descarta el disparo
 *         termico (es enclavado y valia 0 en los tres episodios) y solo deja
 *         una explicacion posible: el HOMBRE MUERTO del esclavo disparando de
 *         forma transitoria y REVOCANDO los setpoints, que es permanente.
 *
 *         Y el arriendo se enviaba asi:
 *             Wire.beginTransmission(...); Wire.write(CMD_LEASE);
 *             Wire.write(ok); Wire.endTransmission();
 *         sin mirar el valor de retorno. Era la UNICA escritura I2C sin
 *         verificar: setDimmer() reintenta tres veces desde v2. De modo que el
 *         maestro publicaba `lease: true` —que es lo que EL cree— mientras las
 *         tramas podian no estar llegando. Ahora todas las escrituras pasan por
 *         `escribirI2C()`, que reintenta y CUENTA los fallos, el arriendo se
 *         renueva cada 500 ms en vez de cada 1000 (perder tramas ya no acerca
 *         a los 30 s tan rapido), y el equipo publica cuantas ha enviado para
 *         contrastarlas con las que el esclavo dice haber recibido.
 *
 *   [M22] UMBRALES TERMICOS COMO VARIABLE DE OPERACION, no de compilacion.
 *         El maestro es el dueño del valor, lo guarda en NVS, lo empuja al
 *         esclavo cada segundo con CMD_SET_TLIM y comprueba el eco. Se fija por
 *         POST /tlimit o por MQTT. Con esto desaparece TEMP_WARN_C_SLV, que era
 *         una COPIA del umbral del esclavo: en v3.1 se quedo en 90 mientras el
 *         esclavo pasaba a 200, y como esa constante gobierna la deteccion de
 *         divergencia, el maestro habria dejado de ver comandos perdidos justo
 *         en la banda de trabajo. Un umbral con dos dueños no es un umbral.
 *
 *   [M23] EL TOPE NO LLEGABA AL ESCLAVO. /maxpower guardaba en NVS y respondia
 *         OK sin enviar CMD_SET_CAP, asi que el clamp del esclavo —el que
 *         sobrevive a la caida de este equipo— se quedaba en el valor con el
 *         que arranco. Ahora se envia.
 *
 *   [M24] `v` DEJA DE VENIR RECORTADO POR EL TOPE. `intensidad[]` guardaba
 *         min(pedido, tope), y eso es lo que se publica; con el tope en 80 y
 *         alguien pidiendo 100, el archivo del SCADA registraba «se pidio 80».
 *         La orden real no quedaba en ningun sitio. Ahora `v` es lo PEDIDO, el
 *         tope viaja aparte en `cap`, y el recorte se aplica al enviar.
 *
 *  ---- v3.4 (González Mancera, 2026-09-25) ----
 *
 *   [M29] ARRIENDO LIGADO A LA RUTA DE MANDO. Una orden MQTT solo puede ser
 *         sostenida por presencia MQTT, y una orden HTTP solo por presencia
 *         HTTP. Si MQTT cae mientras gobierna, se revoca inmediatamente y se
 *         borran los setpoints deseados. La presencia posterior no resucita
 *         una orden vieja: hace falta ordenar de nuevo.
 *
 *   [M30] ESTADO DEL ESCLAVO REPUESTO POR UNA SOLA RUTA, umbrales inseguros
 *         rechazados e instrumentacion de cola y reautorizacion.
 *
 *   [M31] MQTT/TLS AUTENTICA A KIVA con la CA privada de la planta.
 *
 *   [M32] PREPARACION PARA SENECA Y TRAZABILIDAD DE ARRANQUE (2026-10-06):
 *         WiFi abierto con autenticacion por MAC, MAC STA/SSID/build/boot_id
 *         publicados, y NTP acotado obligatorio antes de abrir MQTT/TLS.
 *
 *  ---- v3.4 review3 (González Mancera, 2026-10-23) ----
 *
 *   [M33] LO QUE DEJO LA SEMANA DEL 20 DE OCTUBRE. Cuatro reinicios por corte
 *         del AC-out, una caida de internet de 19 h y AULA abierto dias:
 *         (a) VENTILADORES AUTOMATICOS POR DEFECTO. Antes cada arranque los
 *             dejaba encendidos hasta que alguien pidiera apagarlos (~73 W,
 *             ~40 % de la perdida nocturna de la planta). Ahora siguen al
 *             enclavamiento [M10]; "encender" fuerza, "apagar" vuelve a
 *             automatico. El reposo del rele (NC) sigue siendo ventilar.
 *         (b) MOTIVO DE REINICIO publicado (`reset_reason`): separa un corte
 *             de alimentacion (POWERON/BROWNOUT) de un fallo (PANIC/WDT).
 *         (c) PROCEDENCIA DEL MODO (`modo_src`, `modo_ip`, `modo_since_s`,
 *             `modo_epoch`): quien cambio AULA/AUTO, por que ruta y cuando.
 *         (d) NTP LOCAL PRIMERO (kiva, luego solarblivinglabx, luego
 *             internet). Con MQTT/TLS bloqueado sin reloj y el
 *             NTP en internet, un reinicio durante una caida de internet
 *             dejaba al equipo fuera del broker LOCAL hasta que volviera.
 *
 *  Compatibilidad: el mensaje de estado conserva "v" y "g" con el mismo
 *  significado, y los puntos HTTP existentes siguen respondiendo igual.
 *  NO ES COMPATIBLE con esclavos v3.0/v3.1: el bloque de estado I2C paso de
 *  20 a 32 bytes. Los dos equipos se flashean juntos.
 * ============================================================
 */

#include <Arduino.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <time.h>
#include <Wire.h>
#include <max6675.h>
#include <DHT.h>

#include <ESPmDNS.h>
#include "secrets.h"   // + define API_TOKEN, ver secrets.h.example

// Compatibilidad con un secrets.h anterior. Son nombres públicos, no secretos.
// [M33] El primero es KIVA: el broker es local y el reloj tambien debe serlo,
// o un reinicio sin internet deja el equipo sin MQTT/TLS (2026-10-22). El
// segundo es solarblivinglabx, en el campus y con RTC respaldado por bateria;
// el tercero, internet.
#ifndef NTP_SERVER_1
#define NTP_SERVER_1 "pool.ntp.org"
#endif
#ifndef NTP_SERVER_2
#define NTP_SERVER_2 "time.google.com"
#endif
#ifndef NTP_SERVER_3
#define NTP_SERVER_3 "pool.ntp.org"
#endif

// [M20] Un token de relleno es lo mismo que no tener token: el valor de
// ejemplo esta publicado en este repositorio y en los informes. Un comentario
// no basta —ya se demostro que se pasa por alto—, asi que esto grita en cada
// compilacion. No se convierte en #error a proposito: bloquear la compilacion
// dejaria al estudiante sin poder probar nada, y el objetivo es que lo sepa,
// no que se atasque.
#ifndef API_TOKEN
#error "Falta API_TOKEN en secrets.h. Copia secrets.h.example y genera uno: openssl rand -hex 24"
#endif
// En tiempo de ejecucion: se avisa por consola al arrancar y se publica en el
// estado, para que el SCADA pueda marcarlo en el tablero. Un aviso que solo
// existe en la consola de compilacion se pierde en cuanto alguien flashea
// desde otra maquina.
static inline bool tokenEsRelleno() {
    return strcmp(API_TOKEN, "CAMBIAR_ESTE_TOKEN") == 0;
}

// ------------------------------------------------------------
//  CONFIGURACIÓN
// ------------------------------------------------------------
// [M19] Version del firmware, publicada por el propio equipo. Existe por algo
// concreto: el 2026-09-08 hubo que deducir que firmware corria mirando si el
// payload traia el campo `modo`, y el 2026-09-09 hubo que barrer un /24 entero
// buscando este equipo —sin encontrarlo— mientras publicaba su estado sin
// interrupcion. Un aparato debe poder decir que es y donde esta, por el canal
// que sobrevive.
#define FW_VERSION               "3.4"
#define FW_BUILD                 "2026-10-23-review3"

#define TOPICO_ESTADO            "lab/solar/estado"
#define TOPICO_AUTO_CMD          "lab/solar/auto/cmd"    // [M1]
#define TOPICO_MODO_CMD          "lab/solar/modo/cmd"    // [M1]
#define INTERVALO_ESTADO_MS      3000
#define INTERVALO_RECONEXION_MS  5000
#define INTERVALO_I2C_SYNC_MS    1000    // [M7][M10] temperatura + ventiladores + lectura
#define WDT_TIMEOUT_S            10
#define SOCKET_TIMEOUT_S         5       // [M13] por debajo del perro guardián
#define NTP_SYNC_TIMEOUT_MS      15000UL // [M32] espera acotada al arrancar
#define NTP_RETRY_MS             60000UL // reintento sin bloquear la operación HTTP/I2C
#define CLOCK_MIN_EPOCH          1704067200UL // 2024-01-01 UTC: TLS no arranca en 1970

#define I2C_SLAVE_ADDR  0x08
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22

#define MAX6675_SCK   19
#define MAX6675_CS    32
#define MAX6675_SO    33
#define DHT_PIN       18
#define RELAY_PIN     23

// Protocolo I2C v2 — debe coincidir con SolarIoT_Slave_v2
#define CMD_SET_TEMP    0x10
#define CMD_HEARTBEAT   0x11
#define CMD_CLEAR_TRIP  0x12
#define CMD_SET_CAP     0x13
#define CMD_SET_FANS    0x14
#define CMD_LEASE       0x15    // [M14] arriendo del mando superior
#define CMD_SET_MODE    0x16    // [M15] modo de conmutacion del esclavo
#define CMD_SET_SRC     0x17    // [M15] fuente de sincronismo de una fase
#define CMD_SET_TLIM    0x18    // [M22] umbrales termicos del esclavo
#define I2C_STATUS_LEN  32      // v3.2 [M21]: +revocacion +arriendo +umbrales

#define TEMP_INVALID_RAW ((int16_t)0x8000)   // [M9] centinela, ver esclavo [C11]

// Banderas del byte 6 que devuelve el esclavo
#define SL_TRIP        0x01
#define SL_DEADMAN     0x02
#define SL_FASE_R      0x04
#define SL_FASE_S      0x08
#define SL_FASE_T      0x10
#define SL_TEMP_MALA   0x20
#define SL_MUDO        0x40
#define SL_FANS_OK     0x80

// [M9] Si el MAX6675 no entrega una lectura buena durante este tiempo,
// dejamos de fingir que la última sigue siendo válida. Tres intentos.
#define TEMP_VALIDA_MS   3000UL

// [M10] Enclavamiento de ventiladores.
//   - Post-enfriamiento: la resistencia sigue caliente después de apagar.
//   - Umbral de temperatura: ventilamos mientras siga por encima.
//   - Debe coincidir con TEMP_WARN_C del esclavo para explicar el recorte.
#define FAN_POSTRUN_MS   120000UL   // 2 min tras la última carga activa
#define FAN_TEMP_ON_C    45.0f      // ventila mientras la resistencia esté tibia

// [M22] UMBRALES TERMICOS. Este equipo es el DUEÑO del valor y el esclavo lo
// obedece; aqui no hay copia de nada. Los topes duros viven en el esclavo
// (200/215) y son de compilacion alli, que es donde tienen que estar: la
// proteccion no puede depender de que este equipo siga vivo.
//
// El arranque es conservador y coincide con el del esclavo a proposito: si los
// dos se reinician a la vez, coinciden sin necesidad de negociar.
#define TEMP_WARN_BOOT   120
#define TEMP_TRIP_BOOT   140
#define TEMP_WARN_TOPE   200        // igual que TEMP_WARN_MAX_C del esclavo
#define TEMP_TRIP_TOPE   215        // igual que TEMP_TRIP_MAX_C del esclavo
#define TEMP_MARGEN      10
#define TLIMIT_TEMP_MARGIN_C 5.0f  // [M30] no armar un trip ya violado

// ============================================================
//  [M14] ARRIENDO DEL MANDO SUPERIOR
//
//  El esclavo revoca la potencia si no le renuevan el arriendo. En v2 nadie lo
//  renovaba: setDimmer() solo se llama cuando llega una orden, asi que a los
//  30 s de cada comando la carga caia y no volvia. Ese era el fallo.
//
//  No se arregla reenviando setpoints: eso volveria la proteccion decorativa,
//  que es justo lo que v2 queria evitar. Se arregla RELEVANDO la frescura del
//  mando superior, que es informacion que solo este equipo tiene.
//
//  Cuenta como presencia del mando superior:
//    - una orden real por MQTT o HTTP
//    - una consulta a /values (la pagina la hace cada 3 s mientras este
//      abierta, y el SCADA tambien la sondea)
//
//  POLITICA ELEGIDA: el banco permanece activo mientras la pagina siga
//  abierta. El riesgo residual es una pestana olvidada toda la noche, por eso
//  existe LEASE_MAX_SESSION_MS: pasado ese tiempo desde la ultima ORDEN real,
//  el arriendo cae aunque siga habiendo sondeo, y hace falta un comando nuevo.
//  Poner en 0 para desactivar el tope de sesion.
// ============================================================
#define LEASE_MS              25000UL     // < DEADMAN_MS (30 s) del esclavo
// [M21] Cada 500 ms, no cada 1000. El esclavo revoca a los 30 s sin renovar;
// con un envio por segundo hacen falta 30 tramas perdidas seguidas para
// revocar, y con uno cada 500 ms hacen falta 60. Duplicar el margen cuesta una
// transaccion I2C de 2 bytes cada medio segundo.
#define LEASE_SEND_MS         500UL
// [M25] 1 h, no 4 h. El informe v3.2 propuso bajarlo sin fijar cifra. Con el
// banco en 5,7 kW, 4 h son ~20 kWh que una pestana olvidada puede sostener
// contra una bateria que no tiene nada parecido. Una hora cubre una clase o una
// tanda de medidas, y rearmarlo es tocar cualquier control. Es una decision de
// operacion, no tecnica: cambiar aqui si la campana de armonicos lo exige.
#define LEASE_MAX_SESSION_MS  3600000UL   // 1 h desde la ultima ORDEN real

// [M16] MAPEO FISICO FASE->LINEA DEL INVERSOR.
//  Las etiquetas R/S/T del firmware son del cableado del banco, no de la
//  acometida. El canal 1-2 (firmware "R") esta fisicamente en otra linea del
//  inversor, y por eso el SCADA y el firmware nombran distinto a la misma
//  fase. La conexion es fija, asi que la correspondencia se declara aqui una
//  sola vez y se publica, para que nadie vuelva a cruzar las etiquetas.
//  Verificado contra el testigo independiente del Cerbo el 2026-09-08 y
//  confirmado por el estado v3.3: R↔L3, S↔L1, T↔L2. El candidato anterior
//  decia R↔L2 y dejaba S/T abiertas; era una regresion documental, no evidencia.
static const char* LINEA_FISICA[3] = { "L3", "L1", "L2" };

// [M1] Modos de operación
enum Modo { MODO_AULA = 0, MODO_AUTO = 1 };

const int NUM_CANALES = 6;
int intensidad[NUM_CANALES] = {0, 0, 0, 0, 0, 0};

// Qué fase alimenta cada canal (1-2 R, 3-4 S, 5-6 T)
const uint8_t FASE_DE_CANAL[NUM_CANALES] = {0, 0, 1, 1, 2, 2};

MAX6675 termopar(MAX6675_SCK, MAX6675_CS, MAX6675_SO);
DHT     dht(DHT_PIN, DHT22);

volatile float resistorTemp = NAN;
volatile float ambientTemp  = NAN;
volatile float ambientHum   = NAN;
volatile bool  fansOn       = true;
// [M33] Encendido forzado por orden explicita. Falso = AUTOMATICO: los
// ventiladores siguen al enclavamiento [M10] y se apagan solos en frio.
volatile bool  fansForzados = false;
volatile uint8_t maxPowerLimit = 80;
// [M22] Umbrales termicos vigentes, persistidos en NVS.
volatile uint8_t tempWarnC = TEMP_WARN_BOOT;
volatile uint8_t tempTripC = TEMP_TRIP_BOOT;
const char* tlimitSrc = "boot";  // [M30] trazabilidad del ultimo cambio

uint32_t ultimaTempValida  = 0;         // [M9]
uint32_t ultimoCargaActiva = 0;         // [M10]

// [M8] Estado leído del esclavo
volatile uint8_t  slaveApplied[NUM_CANALES] = {0,0,0,0,0,0};
volatile uint8_t  slaveFlags     = 0;
volatile uint8_t  slaveTempC     = 255;
volatile uint16_t slaveUptime    = 0;
volatile bool     slaveOnline    = false;
volatile bool     slaveDiverged  = false;

uint32_t ultimoMax6675 = 0, ultimoDHT = 0;

int  grupoActivo = 0;
// [M14] Por defecto AULA, no AUTO. En AUTO el equipo solo escucha
// lab/solar/auto/cmd y deshabilita el control manual por web, de modo que un
// despliegue con la infraestructura actual (lab/solar/G{N}/cmd) queda sordo y
// sin control manual: parece un equipo averiado. AUTO se elige a proposito.
Modo modo        = MODO_AULA;
char topicoCmd[48] = "";

volatile bool mqttIsConnected = false;

// [M14] Relojes del arriendo. ultimoMandoSuperior lo refresca cualquier
// presencia (orden o sondeo de /values); ultimaOrdenReal solo una orden.
volatile uint32_t ultimoMandoSuperior = 0;
volatile uint32_t ultimaOrdenReal     = 0;
volatile bool     arriendoVigente     = false;
// [M29] La presencia solo renueva una orden nacida por la misma ruta.
enum OrigenMando : uint8_t { ORIGEN_NINGUNO = 0, ORIGEN_HTTP = 1, ORIGEN_MQTT = 2 };
volatile OrigenMando origenArriendo = ORIGEN_NINGUNO;

// [M19] Ancho de la ventana de cruce por cero de cada fase, en us, tal como lo
// mide el esclavo. Viaja por I2C porque el puerto serie del esclavo es
// inaccesible con el equipo conectado. 0 = el ZCD ya no conmuta (saturado).
volatile uint8_t  slaveWindow[3]      = {0, 0, 0};

// [M20] Flancos crudos por segundo en cada pin SYNC, sin filtrar. Separa un
// detector limpio (120/s) de uno que rebota al cruzar el umbral (cientos/s).
volatile uint16_t slaveEdges[3]       = {0, 0, 0};

// [M26] Fuente de sincronismo de cada fase, persistida aqui. El esclavo no la
// guarda: tras cualquier reinicio suyo (perro guardian, parpadeo de la
// alimentacion) la fase reconstruida volvia a quedar muerta hasta que alguien
// repitiera /sync a mano. Era la duda nº4 del informe. Codificacion por fase:
// -1 = detector propio, 0..2 = indice de la fase de referencia.
volatile int8_t   phaseSrc[3]         = { -1, -1, -1 };

// [M21][M22] Diagnostico que el esclavo v3.2 devuelve en su bloque de estado.
// Existe porque el 2026-09-10 la pregunta «por que dejo de conducir» no se
// podia contestar desde fuera.
volatile uint8_t  slaveRevocs      = 0;   // revocaciones desde su arranque
volatile uint8_t  slaveRevocReason = 0;   // 0 ninguna 1 hombre muerto 2 trip 3 dif. 4 explicita
volatile uint16_t slaveLeaseAgeDs  = 0;   // edad del arriendo, decimas de s
volatile uint16_t slaveLeaseGapDs  = 0;   // PEOR hueco entre renovaciones
volatile uint8_t  slaveWarnC       = 0;   // eco del umbral de recorte
volatile uint8_t  slaveTripC       = 0;   // eco del umbral de disparo
volatile uint16_t slaveBadFrames   = 0;   // tramas mal formadas que descarto
volatile uint16_t slaveLeaseRx     = 0;   // CMD_LEASE que dice haber recibido

// [M21] Lo que ESTE equipo cree haber enviado. Contrastar los dos numeros es
// lo que distingue «el bus pierde tramas» de «el esclavo las ignora», y es
// justo la distincion que no se pudo hacer el 2026-09-10.
volatile uint32_t leaseTxCount     = 0;   // [M30] no satura tras 9,1 h
volatile uint16_t i2cWriteErrors   = 0;
volatile uint32_t i2cQueueDrops    = 0;   // [M30] xQueueSend rechazados
volatile uint8_t  i2cQueuePeak     = 0;
volatile uint32_t reauthCount      = 0;   // [M30] reenvios de [M28]
uint32_t ultimoEstado = 0, ultimoIntento = 0, ultimoI2CSync = 0;
uint32_t ultimoArriendo = 0;   // [M21] reloj propio del arriendo
uint32_t ultimoWifiCheck = 0;
uint32_t ultimoNtpIntento = 0;
char bootId[9] = "00000000";  // [M32] identidad de este arranque, no una version
const char* resetReason = "UNKNOWN";   // [M33] por que arranco este boot

// [M33] Procedencia del modo vigente. `modo_src`: boot | http | mqtt.
enum OrigenModo : uint8_t { MODO_SRC_BOOT = 0, MODO_SRC_HTTP = 1, MODO_SRC_MQTT = 2 };
static const char* const MODO_SRC_TXT[3] = { "boot", "http", "mqtt" };
volatile uint8_t modoSrc      = MODO_SRC_BOOT;
volatile uint32_t modoDesdeMs = 0;
volatile uint32_t modoEpoch   = 0;
char modoIp[16]          = "";   // cliente HTTP que lo cambio
char modoIpPendiente[16] = "";   // escrito por el handler, adoptado en aplicarModo

static bool relojValido();

WebServer        server(8080);      // el equipo desplegado usa 8080
WiFiClientSecure espClient;
PubSubClient     mqtt(espClient);
Preferences      prefs;             // [M2]

struct I2CCmd { uint8_t ch; uint8_t val; };
// [M13] FIFO real: con profundidad 1 y xQueueOverwrite, un comando HTTP y uno
// MQTT que llegaran en la misma ventana de 10 ms se pisaban en silencio.
static QueueHandle_t     grupoQueue = nullptr;
static QueueHandle_t     i2cQueue   = nullptr;
// PubSubClient no admite llamar subscribe()/unsubscribe() desde dentro de su
// propio callback: corrompe el buffer de recepción. El cambio de modo se
// difiere por cola y se ejecuta en mqttTask, igual que el cambio de grupo.
static QueueHandle_t     modoQueue  = nullptr;
static QueueHandle_t     tripQueue  = nullptr;
static SemaphoreHandle_t stateMutex = nullptr;
static SemaphoreHandle_t cfgMutex   = nullptr;   // [M13] NVS desde dos núcleos

// ============================================================
//  COMPATIBILIDAD DEL PERRO GUARDIÁN
//  Arduino-ESP32 3.x cambió la firma de esp_task_wdt_init a una estructura.
// ============================================================
static void wdt_init_compat(uint32_t timeout_s) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t cfg = {
        .timeout_ms = timeout_s * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_reconfigure(&cfg);
#else
    esp_task_wdt_init(timeout_s, true);
#endif
}

// ============================================================
//  [M2][M13] PERSISTENCIA
//  Serializada, y escribe SOLO si el valor cambió. La persistencia es nueva
//  ([M2]); sin el chequeo de cambio, cada asignación de grupo escribiría NVS,
//  incluida la repetida, y un cliente MQTT podría desgastar la flash a
//  voluntad. guardarConfig() se llama desde los dos núcleos, de ahí el mutex.
// ============================================================
static void guardarConfig() {
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    prefs.begin("solariot", false);
    if (prefs.getUChar("modo", 255)  != (uint8_t)modo)   prefs.putUChar("modo",  (uint8_t)modo);
    if (prefs.getInt  ("grupo", -1)  != grupoActivo)     prefs.putInt  ("grupo", grupoActivo);
    if (prefs.getUChar("cap", 255)   != maxPowerLimit)   prefs.putUChar("cap",   maxPowerLimit);
    if (prefs.getUChar("twarn", 255) != tempWarnC)      prefs.putUChar("twarn", tempWarnC);   // [M22]
    if (prefs.getUChar("ttrip", 255) != tempTripC)      prefs.putUChar("ttrip", tempTripC);   // [M22]
    uint8_t src = (uint8_t)(((phaseSrc[0] + 1) & 3) | (((phaseSrc[1] + 1) & 3) << 2)
                                                    | (((phaseSrc[2] + 1) & 3) << 4));
    if (prefs.getUChar("psrc", 255) != src)             prefs.putUChar("psrc",  src);          // [M26]
    prefs.end();
    xSemaphoreGive(cfgMutex);
}

static void cargarConfig() {
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    prefs.begin("solariot", true);
    // [M14] Version de configuracion: un equipo que ya corrio v2 tiene "modo"
    // guardado en AUTO en NVS, y sin esto v3 arrancaria igual de sordo.
    uint8_t cfgver = prefs.getUChar("cfgver", 0);
    prefs.end();
    if (cfgver < 3) {
        prefs.begin("solariot", false);
        prefs.putUChar("cfgver", 3);
        prefs.putUChar("modo", (uint8_t)MODO_AULA);
        prefs.end();
        Serial.println("Config migrada a v3: modo AULA por defecto.");
    }
    prefs.begin("solariot", true);
    modo          = (Modo)prefs.getUChar("modo", (uint8_t)MODO_AULA);
    grupoActivo   = prefs.getInt("grupo", 1);
    maxPowerLimit = prefs.getUChar("cap", 80);
    uint8_t src   = prefs.getUChar("psrc", 0);                 // [M26]
    for (int i = 0; i < 3; i++) phaseSrc[i] = (int8_t)(((src >> (2 * i)) & 3) - 1);
    tempWarnC     = prefs.getUChar("twarn", TEMP_WARN_BOOT);   // [M22]
    tempTripC     = prefs.getUChar("ttrip", TEMP_TRIP_BOOT);
    prefs.end();
    xSemaphoreGive(cfgMutex);
    Serial.printf("Config: modo=%s grupo=%d cap=%d%% umbrales=%d/%d C\n",
                  modo == MODO_AUTO ? "AUTO" : "AULA", grupoActivo, maxPowerLimit,
                  tempWarnC, tempTripC);
}

// ============================================================
//  [M10] ENCLAVAMIENTO DE VENTILADORES
//
//  Polaridad del relé (heredada de v1, NO cambiar sin medir):
//    relé NC con disparo por nivel bajo.
//    HIGH = relé desenergizado = contacto NC cerrado = VENTILADORES ON
//    LOW  = relé energizado    = contacto abierto    = VENTILADORES OFF
//  Es decir que el estado seguro es el de reposo, y un ESP32 reiniciado o sin
//  firmware deja los ventiladores ENCENDIDOS. Esa propiedad hay que conservarla.
//
//  Los ventiladores dejan de ser una perilla y pasan a ser una función de la
//  carga. Con esto desaparece el modo de fallo de "encender la carga y olvidar
//  el ventilador", que ya no depende de que nadie se acuerde de nada.
//
//  [M33] Y tampoco el inverso: hasta la review2 el arranque dejaba
//  `fansManualOff=false`, de modo que tras cada reinicio quedaban encendidos
//  con el banco frio hasta que alguien pidiera apagarlos. Ahora el modo por
//  defecto es AUTOMATICO: se apagan solos cuando no se requieren. Una orden de
//  encender los FUERZA (pruebas); una de apagar vuelve a automatico.
//
//  Límite conocido: no hay tacómetro ni sensor de flujo. Este enclavamiento
//  garantiza que el relé está CERRADO, no que el aire se mueva. Un ventilador
//  atascado con el relé cerrado es indetectable aquí; el respaldo para ese caso
//  es el disparo térmico del esclavo, y por eso el termopar debe estar sobre la
//  resistencia y no sobre el aire.
// ============================================================
static bool cargaActiva() {
    for (int i = 0; i < NUM_CANALES; i++)
        if (intensidad[i] > 0 || slaveApplied[i] > 0) return true;
    return false;
}

static void actualizarVentiladores() {
    uint32_t now = millis();
    if (cargaActiva()) ultimoCargaActiva = now;

    float rt = resistorTemp;
    bool requeridos =
           cargaActiva()                                        // hay carga ahora
        || (now - ultimoCargaActiva) < FAN_POSTRUN_MS           // post-enfriamiento
        || isnan(rt)                                            // sin termómetro: ventilar
        || rt >= FAN_TEMP_ON_C;                                 // sigue tibia

    bool nuevo = requeridos || fansForzados;
    if (nuevo != fansOn) {
        fansOn = nuevo;
        digitalWrite(RELAY_PIN, nuevo ? HIGH : LOW);
    }
}

// ============================================================
//  I2C — solo desde el núcleo 0 (mqttTask)
// ============================================================
// [M21] TODA escritura I2C pasa por aqui, y toda escritura se verifica.
//
// Hasta v3.1 solo setDimmer() comprobaba endTransmission(); la temperatura, los
// ventiladores y —lo que importa— el ARRIENDO se enviaban a ciegas. El equipo
// publicaba entonces `lease: true` porque es lo que el creia, no lo que el
// esclavo habia recibido. Un mensaje que no se comprueba no es un mensaje: es
// una intencion.
static bool escribirI2C(const uint8_t* datos, uint8_t n, const char* que) {
    bool ok = false;
    for (int r = 0; r < 3 && !ok; r++) {
        Wire.beginTransmission(I2C_SLAVE_ADDR);
        Wire.write(datos, n);
        ok = (Wire.endTransmission() == 0);
        if (!ok && r < 2) delay(5);       // 5 ms, no 30: esto corre cada 500 ms
    }
    if (!ok) {
        if (i2cWriteErrors < 65535) i2cWriteErrors++;
        Serial.printf("I2C: fallo al escribir %s\n", que);
    }
    return ok;
}

bool setDimmer(uint8_t dimmer, uint8_t power) {
    uint8_t d[2] = { dimmer, power };
    return escribirI2C(d, 2, "setpoint de canal");
}

// ============================================================
//  [M14] PRESENCIA Y ARRIENDO
// ============================================================
static inline uint32_t ms_no_cero() { uint32_t m = millis(); return m ? m : 1; }

// Una ORDEN real fija tambien la ruta que posee el arriendo.
static void marcarOrdenReal(OrigenMando origen) {
    uint32_t m = ms_no_cero();
    origenArriendo      = origen;
    ultimaOrdenReal     = m;
    ultimoMandoSuperior = m;
}

// Presencia sin orden: solo vale si llega por la ruta que emitio la orden.
static bool marcarPresencia(OrigenMando origen) {
    if (origenArriendo != origen) return false;
    ultimoMandoSuperior = ms_no_cero();
    return true;
}

// [M29] Revocacion irreversible: borra la intencion local y manda ceros. Debe
// ejecutarse desde mqttTask, unico propietario normal del bus I2C.
static void revocarSetpoints(const char* motivo) {
    Serial.printf("Arriendo revocado: %s. Se exige una orden nueva.\n", motivo);
    xQueueReset(i2cQueue);          // [M34] ninguna orden vieja sale detras de los ceros
    ultimoMandoSuperior = 0;
    ultimaOrdenReal = 0;
    origenArriendo = ORIGEN_NINGUNO;
    for (int i = 0; i < NUM_CANALES; i++) {
        intensidad[i] = 0;
        setDimmer(i + 1, 0);
    }
}

// El arriendo se afirma si hay presencia fresca y la sesion no se ha pasado.
static bool arriendoActivo() {
    uint32_t now = millis();
    if (ultimoMandoSuperior == 0) return false;
    if (now - ultimoMandoSuperior > LEASE_MS) return false;
#if LEASE_MAX_SESSION_MS > 0
    if (ultimaOrdenReal == 0) return false;
    if (now - ultimaOrdenReal > LEASE_MAX_SESSION_MS) return false;   // sesion vencida
#endif
    return true;
}

// [M14] Esto es lo que faltaba en v2. Se envia cada segundo junto con la
// temperatura y los ventiladores.
static void enviarArriendo() {
    static bool antesAutorizado = false;
    bool ok = arriendoActivo();
    if (antesAutorizado && !ok) revocarSetpoints("presencia o sesion vencida");
    antesAutorizado = ok;
    uint8_t d[2] = { CMD_LEASE, (uint8_t)(ok ? 1 : 0) };
    bool entregado = escribirI2C(d, 2, "arriendo");
    leaseTxCount++;  // uint32 modular, delta especificado en el informe
    // [M21] `arriendoVigente` es lo que se PUBLICA, y ahora significa las dos
    // cosas que tienen que ser ciertas: que este equipo afirma el arriendo y
    // que la trama salio del bus. Antes solo significaba la primera, y por eso
    // el estado decia `lease: true` durante un fallo del arriendo.
    arriendoVigente = ok && entregado;
}

// [M22] Empujar los umbrales termicos al esclavo. Se manda cada segundo, sin
// condicion: es idempotente, cuesta tres bytes, y significa que un esclavo que
// se reinicio recupera el umbral de operacion en <1 s en vez de quedarse en su
// arranque conservador esperando a que alguien se acuerde.
static void enviarUmbrales() {
    uint8_t d[3] = { CMD_SET_TLIM, (uint8_t)tempWarnC, (uint8_t)tempTripC };
    escribirI2C(d, 3, "umbrales termicos");
}

// ============================================================
//  [M18] REPORTE DEL ESCLAVO POR CONSOLA
//
//  v2 solo publicaba el estado del esclavo por HTTP y MQTT, asi que durante la
//  puesta en marcha la consola se queda muda aunque el enlace I2C este caido:
//  no se distingue "todo bien" de "el esclavo no contesta". Se imprime solo
//  cuando algo CAMBIA, para no ensuciar el log en regimen permanente.
// ============================================================
// ============================================================
//  [M28] REENVIO POR DIVERGENCIA SOSTENIDA
//
//  El 2026-09-10 el equipo publico `div: true` durante minutos —sabia que el
//  esclavo no estaba aplicando lo pedido— y no hizo nada. Aqui lo hace.
//
//  Esto NO contradice la regla de no resucitar setpoints: aquella prohibe
//  reenergizar con una orden vieja DESPUES de que el arriendo venciera. Este
//  reenvio solo ocurre con el arriendo VIGENTE ahora mismo, es decir, con el
//  mando superior presente, y sin ninguna proteccion activa. Es reafirmar una
//  orden viva, no revivir una muerta.
//
//  Hacen falta 3 ciclos seguidos de divergencia (~3 s) para no dispararlo por
//  el desfase normal entre enviar y leer.
// ============================================================
static void reautorizarSiDiverge() {
    static uint8_t seguidos = 0;

    bool bloqueado = (slaveFlags & SL_TRIP) || (slaveFlags & SL_DEADMAN) ||
                     (slaveFlags & SL_TEMP_MALA) || !(slaveFlags & SL_FANS_OK);
    if (!slaveOnline || !slaveDiverged || !arriendoVigente || bloqueado) {
        seguidos = 0;
        return;
    }
    if (++seguidos < 3) return;
    seguidos = 0;

    Serial.println("  Divergencia sostenida con arriendo vigente: reafirmando setpoints.");
    reauthCount++;
    for (int i = 0; i < NUM_CANALES; i++) {
        uint8_t v = intensidad[i];
        if (v > maxPowerLimit) v = maxPowerLimit;
        setDimmer(i + 1, v);
    }
}

static void reportarEsclavo() {
    // [M27] Consola de operacion, no de depuracion. Los cambios de firmware ya
    // estan confirmados en el equipo, asi que lo que hace falta ahora es ver
    // rapido si todo va bien y que salte lo que no. Se imprime:
    //   - una linea de ESTADO cuando cambia algo que importa, o cada 60 s;
    //   - una linea de ALERTA, aparte, solo si hay algo que mirar.
    // Las ventanas y los flancos crudos dejan de imprimirse: fluctuan solos,
    // llenaban el log y estan en /values de forma continua para quien los
    // necesite. Vuelven a la consola si una fase se declara muerta.
    static bool     primera    = true;
    static bool     ult_online = false;
    static uint8_t  ult_flags  = 0xFF;
    static bool     ult_lease  = false;
    static uint8_t  ult_revocs = 0;
    static uint32_t ult_print  = 0;

    bool    on    = slaveOnline;
    uint8_t flags = slaveFlags;
    bool    lease = arriendoVigente;
    uint32_t now  = millis();

    bool cambio = primera || on != ult_online || flags != ult_flags ||
                  lease != ult_lease || slaveRevocs != ult_revocs;
    if (!cambio && (now - ult_print) < 60000UL) return;
    primera = false; ult_online = on; ult_flags = flags;
    ult_lease = lease; ult_revocs = slaveRevocs; ult_print = now;

    if (!on) { Serial.println("ESCLAVO: sin respuesta por I2C (0x08)"); return; }

    static const uint8_t FASE_BIT[3] = { SL_FASE_R, SL_FASE_S, SL_FASE_T };
    int vivas = 0;
    for (int p = 0; p < 3; p++) if (flags & FASE_BIT[p]) vivas++;

    // --- linea de estado ---
    Serial.printf("ESCLAVO  fases %d/3  arriendo %s (hueco max %.1fs)  "
                  "temp %u C (umbral %u/%u)  aplicado %u/%u/%u/%u/%u/%u %%\n",
        vivas, lease ? "OK" : "NO", slaveLeaseGapDs / 10.0f,
        slaveTempC, slaveWarnC, slaveTripC,
        slaveApplied[0], slaveApplied[1], slaveApplied[2],
        slaveApplied[3], slaveApplied[4], slaveApplied[5]);

    // --- linea de alerta, solo si hay de que ---
    static const char* MOTIVO[] = { "ninguna", "HOMBRE MUERTO",
                                    "DISPARO TERMICO", "peticion diferida",
                                    "REVOCACION EXPLICITA" };       // [C22]
    char al[240]; int n = 0;
    for (int p = 0; p < 3; p++)
        if (!(flags & FASE_BIT[p]))
            n += snprintf(al + n, sizeof(al) - n, " fase %s MUERTA;", LINEA_FISICA[p]);
    if (flags & SL_TRIP)      n += snprintf(al + n, sizeof(al) - n, " DISPARO TERMICO;");
    if (flags & SL_TEMP_MALA) n += snprintf(al + n, sizeof(al) - n, " temperatura no fiable;");
    if (!(flags & SL_FANS_OK))n += snprintf(al + n, sizeof(al) - n, " ventiladores sin confirmar;");
    if (slaveRevocs)          n += snprintf(al + n, sizeof(al) - n,
                                   " %u revocaciones (ultima: %s);",
                                   slaveRevocs, MOTIVO[slaveRevocReason < 5 ? slaveRevocReason : 0]);
    if (i2cWriteErrors)       n += snprintf(al + n, sizeof(al) - n,
                                   " %u fallos de escritura I2C;", i2cWriteErrors);
    if (slaveBadFrames)       n += snprintf(al + n, sizeof(al) - n,
                                   " %u tramas malformadas;", slaveBadFrames);
    if (slaveWarnC != tempWarnC || slaveTripC != tempTripC)
        n += snprintf(al + n, sizeof(al) - n, " umbrales NO coinciden;");
    if (n > 0) Serial.printf("  AVISO:%s\n", al);
}

// [M10] El esclavo no conduce si no le confirmamos la refrigeración.
static void enviarEstadoVentiladores() {
    uint8_t d[2] = { CMD_SET_FANS, (uint8_t)(fansOn ? 1 : 0) };
    escribirI2C(d, 2, "estado de ventiladores");
}

// [M30] Una sola lista para arranque y reinicio del esclavo. Evita que una
// propiedad persistida, como phaseSrc, se reponga en un camino y se olvide en otro.
static void reponerEstadoDelEsclavo(const char* motivo) {
    Serial.printf("Reponiendo estado seguro del esclavo: %s.\n", motivo);
    uint8_t cap[2] = { CMD_SET_CAP, (uint8_t)maxPowerLimit };
    escribirI2C(cap, 2, "tope de potencia");
    enviarEstadoVentiladores();
    enviarUmbrales();
    for (int f = 0; f < 3; f++) {
        if (phaseSrc[f] < 0) continue;
        uint8_t src[2] = { CMD_SET_SRC,
                           (uint8_t)((f << 4) | ((phaseSrc[f] + 1) & 0x0F)) };
        escribirI2C(src, 2, "fuente de sincronismo");
    }
}

// [M7][M9] Enviar temperatura al esclavo. La protección térmica vive allá para
// que sobreviva a la caída de este equipo, de la red y del broker. Una lectura
// ausente se envía como INVÁLIDA explícita, no se calla.
static void enviarTemperatura() {
    float t = resistorTemp;
    int16_t raw = (isnan(t)) ? TEMP_INVALID_RAW : (int16_t)(t * 10.0f);
    uint8_t d[3] = { CMD_SET_TEMP,
                     (uint8_t)((uint16_t)raw >> 8),
                     (uint8_t)((uint16_t)raw & 0xFF) };
    escribirI2C(d, 3, "temperatura");
}

// [M8][M11][M12] Leer el estado real del esclavo, comparar con lo ESPERADO y
// detectar reinicio. Un reinicio NO reinyecta setpoints: los da por perdidos.
static void leerEstadoEsclavo() {
    uint8_t buf[I2C_STATUS_LEN];
    uint8_t n = Wire.requestFrom((uint8_t)I2C_SLAVE_ADDR, (uint8_t)I2C_STATUS_LEN);
    if (n != I2C_STATUS_LEN) { slaveOnline = false; return; }
    for (int i = 0; i < I2C_STATUS_LEN; i++) buf[i] = Wire.read();
    slaveOnline = true;

    uint16_t up = ((uint16_t)buf[8] << 8) | buf[9];
    bool reinicio = (up < slaveUptime);      // el contador solo puede crecer
    slaveUptime = up;

    for (int i = 0; i < NUM_CANALES; i++) slaveApplied[i] = buf[i];
    uint8_t flags = buf[6];
    slaveFlags = flags;
    slaveTempC = buf[7];
    for (int i = 0; i < 3; i++) {
        slaveWindow[i] = buf[11 + i];                          // [M19]
        slaveEdges[i]  = ((uint16_t)buf[14 + 2*i] << 8) | buf[15 + 2*i];  // [M20]
    }
    // [M21][M22] Diagnostico de la revocacion y del arriendo (esclavo v3.2).
    slaveRevocs      = buf[20];
    slaveRevocReason = buf[21];
    slaveLeaseAgeDs  = ((uint16_t)buf[22] << 8) | buf[23];
    slaveLeaseGapDs  = ((uint16_t)buf[24] << 8) | buf[25];
    slaveWarnC       = buf[26];
    slaveTripC       = buf[27];
    slaveBadFrames   = ((uint16_t)buf[28] << 8) | buf[29];
    slaveLeaseRx     = ((uint16_t)buf[30] << 8) | buf[31];

    // [M12] Construimos la expectativa con TODO lo que legítimamente reduce la
    // potencia. Lo que quede fuera de esa expectativa es divergencia de verdad.
    bool faseViva[3] = { (flags & SL_FASE_R) != 0,
                         (flags & SL_FASE_S) != 0,
                         (flags & SL_FASE_T) != 0 };
    bool inhibido = (flags & SL_TRIP) || (flags & SL_DEADMAN) ||
                    (flags & SL_MUDO) || !(flags & SL_FANS_OK);
    // El recorte térmico progresivo puede dejar el aplicado en cualquier valor
    // entre 0 y lo esperado, así que solo por debajo de WARN podemos exigir
    // igualdad. Por encima, únicamente que no se pase.
    // [M22] El umbral es el NUESTRO, que es el que el esclavo obedece, y no una
    // copia en un #define. En v3.1 esa copia se quedo en 90 mientras el esclavo
    // pasaba a 200, y como gobierna esta linea, el maestro habria dejado de
    // detectar comandos perdidos justo en la banda normal de trabajo.
    bool recorteTermico = (flags & SL_TEMP_MALA) ||
                          (slaveTempC != 255 && slaveTempC >= tempWarnC);

    bool div = reinicio;
    for (int i = 0; i < NUM_CANALES; i++) {
        uint8_t esperado = 0;
        if (!inhibido && faseViva[FASE_DE_CANAL[i]]) {
            int e = intensidad[i];
            if (e > (int)maxPowerLimit) e = (int)maxPowerLimit;
            esperado = (uint8_t)e;
        }
        if (slaveApplied[i] > esperado) div = true;              // aplica de más
        else if (!recorteTermico && slaveApplied[i] < esperado)  // comando perdido
            div = true;
    }
    slaveDiverged = div;

    if (reinicio) {
        // [M11] El esclavo arrancó de cero. Reenviamos el TOPE (es un límite,
        // no una autorización) y damos los setpoints por perdidos. Volver a
        // energizar exige un comando nuevo del nivel superior. La alternativa
        // (reinyectar aquí los seis setpoints guardados) haría que un esclavo
        // reiniciado por perro guardián regresara solo a plena potencia.
        Serial.println("Esclavo reiniciado: tope reenviado, setpoints a 0 "
                       "(se exige comando nuevo).");
        reponerEstadoDelEsclavo("reinicio detectado");  // [M30] incluye phaseSrc
        for (int i = 0; i < NUM_CANALES; i++) intensidad[i] = 0;
    }
}

// ============================================================
//  CONTROL DE CANALES
// ============================================================
void aplicarIntensidad(int canal, int porcent, OrigenMando origen) {
    marcarOrdenReal(origen);                          // [M14][M29]
    // [M24] `intensidad[]` guarda lo PEDIDO, sin recortar. Antes guardaba
    // min(pedido, tope) y eso es lo que se publicaba, de modo que con el tope
    // en 80 y alguien pidiendo 100 el archivo del SCADA registraba «se pidio
    // 80»: la orden real no quedaba en ningun sitio. El tope viaja aparte en
    // `cap` y se aplica aqui abajo, al enviar.
    int solicitado = constrain(porcent, 0, 100);
    intensidad[canal] = solicitado;
    uint8_t aEnviar = (uint8_t)min(solicitado, (int)maxPowerLimit);
    // [M10] Los ventiladores se ajustan ANTES de encolar el comando, para que
    // el relé ya esté cerrado cuando la potencia llegue al esclavo.
    actualizarVentiladores();
    I2CCmd cmd = {(uint8_t)(canal + 1), aEnviar};
    if (xQueueSend(i2cQueue, &cmd, 0) != pdTRUE) i2cQueueDrops++;
    UBaseType_t ocup = uxQueueMessagesWaiting(i2cQueue);
    if (ocup > i2cQueuePeak) i2cQueuePeak = (uint8_t)ocup;
}

// ============================================================
//  [M1] MODO Y SUSCRIPCIONES
//
//  En MODO_AUTO el equipo escucha SIEMPRE el tópico reservado, sin importar el
//  selector de grupo. En MODO_AULA escucha el grupo seleccionado. El tópico de
//  modo se escucha en ambos, para poder volver desde el computador.
// ============================================================
static void suscribirSegunModo() {
    if (!mqtt.connected()) return;
    mqtt.subscribe(TOPICO_MODO_CMD, 1);
    if (modo == MODO_AUTO) {
        mqtt.subscribe(TOPICO_AUTO_CMD, 1);
        Serial.println("Modo AUTO: suscrito a " TOPICO_AUTO_CMD);
    } else if (grupoActivo > 0) {
        snprintf(topicoCmd, sizeof(topicoCmd), "lab/solar/G%d/cmd", grupoActivo);
        mqtt.subscribe(topicoCmd, 1);
        Serial.printf("Modo AULA: suscrito a %s\n", topicoCmd);
    } else {
        Serial.println("Modo AULA sin grupo seleccionado: no se escucha nada.");
    }
}

static void desuscribirTodo() {
    if (!mqtt.connected()) return;
    mqtt.unsubscribe(TOPICO_AUTO_CMD);
    if (topicoCmd[0]) mqtt.unsubscribe(topicoCmd);
}

// Ejecutado SOLO desde mqttTask, nunca desde el callback MQTT.
static void aplicarModo(Modo m, uint8_t src) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    Modo actual = modo;
    xSemaphoreGive(stateMutex);
    if (m == actual) return;

    // [M30][V8] Cambiar de autoridad no hereda ni reafirma un vector viejo.
    revocarSetpoints("cambio de modo");
    desuscribirTodo();
    xSemaphoreTake(stateMutex, portMAX_DELAY);         // [M13] modo bajo mutex
    modo = m;
    modoSrc = src <= MODO_SRC_MQTT ? src : MODO_SRC_BOOT;   // [M33]
    modoDesdeMs = millis();
    modoEpoch = relojValido() ? (uint32_t)time(nullptr) : 0;
    if (src == MODO_SRC_HTTP) strlcpy(modoIp, modoIpPendiente, sizeof(modoIp));
    else modoIp[0] = '\0';
    xSemaphoreGive(stateMutex);
    guardarConfig();
    suscribirSegunModo();
    Serial.printf("Modo -> %s (%s %s)\n", m == MODO_AUTO ? "AUTO" : "AULA",
                  MODO_SRC_TXT[modoSrc], modoIp);
}

// Seguros de llamar desde cualquier contexto: solo encolan.
// [M33] El origen viaja con el modo en la misma cola: bits 0-7 modo, 8-15 origen.
void cambiarModo(Modo m, uint8_t src) {
    int v = (int)m | ((int)src << 8);
    xQueueOverwrite(modoQueue, &v);
}
void cambiarGrupo(int g)   { xQueueOverwrite(grupoQueue, &g); }
void pedirClearTrip()      { int v = 1;      xQueueOverwrite(tripQueue, &v); }

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<96> doc;
    if (deserializeJson(doc, payload, length)) return;

    if (strcmp(topic, TOPICO_MODO_CMD) == 0) {          // [M1]
        const char* m = doc["modo"] | "";
        // "scada" se acepta como sinónimo de "auto": el modo programático se
        // llamó así en un borrador interno y algún cliente puede usarlo.
        if (!strcmp(m, "auto") || !strcmp(m, "scada")) cambiarModo(MODO_AUTO, MODO_SRC_MQTT);
        else if (!strcmp(m, "aula")) cambiarModo(MODO_AULA, MODO_SRC_MQTT);
        if (doc.containsKey("grupo")) {
            int g = doc["grupo"] | 0;
            if (g >= 0 && g <= 10) cambiarGrupo(g);
        }
        // [M21] PRESENCIA POR MQTT. [M18] la puso solo en HTTP, y el HTTP de
        // este equipo es justo la ruta que ha fallado (2026-09-08 13:31:
        // inalcanzable por HTTP durante horas mientras MQTT seguia). Dejar el
        // hombre muerto colgando de la ruta fragil es construir el fallo.
        if (doc.containsKey("presencia") && (doc["presencia"] | false))
            marcarPresencia(ORIGEN_MQTT);
        // [M21] Ventiladores y tope por MQTT. Se anuncian en `caps`, asi que
        // tienen que EXISTIR: anunciar una capacidad que no se implementa es
        // el fallo silencioso que `caps` existe para eliminar. Encender se
        // obedece; apagar sigue siendo una peticion sujeta al enclavamiento
        // [M10], y como por MQTT no hay 409 con el que contestar, el resultado
        // se lee en `fans` y `fans_held` del estado publicado.
        // [M33] true = forzar encendido; false = volver a automatico.
        if (doc.containsKey("fans")) {
            fansForzados = (doc["fans"] | false);
            actualizarVentiladores();
        }
        if (doc.containsKey("cap")) {
            int c = doc["cap"] | -1;
            if (c >= 0 && c <= 100) {
                xSemaphoreTake(stateMutex, portMAX_DELAY);
                maxPowerLimit = (uint8_t)c;
                xSemaphoreGive(stateMutex);
                guardarConfig();
                uint8_t d[2] = { CMD_SET_CAP, (uint8_t)c };   // [M23]
                escribirI2C(d, 2, "tope de potencia (MQTT)");
            }
        }
        // [M22] Umbrales termicos, tambien por la ruta que sobrevive.
        if (doc.containsKey("twarn")) {
            int w = doc["twarn"] | (int)TEMP_WARN_BOOT;
            int t = doc["ttrip"] | (w + TEMP_MARGEN);
            if (w < 40)  w = 40;
            if (w > TEMP_WARN_TOPE) w = TEMP_WARN_TOPE;
            if (t < w + TEMP_MARGEN) t = w + TEMP_MARGEN;
            if (t > TEMP_TRIP_TOPE) { t = TEMP_TRIP_TOPE; w = t - TEMP_MARGEN; }
            float actual = resistorTemp;
            if (!isnan(actual) && t <= actual + TLIMIT_TEMP_MARGIN_C) {
                Serial.printf("MQTT: tlimit rechazado, trip %d C con resistencia %.1f C.\n",
                              t, actual);
            } else {
                xSemaphoreTake(stateMutex, portMAX_DELAY);
                tempWarnC = (uint8_t)w; tempTripC = (uint8_t)t;
                tlimitSrc = "mqtt";
                xSemaphoreGive(stateMutex);
                guardarConfig();
            }
        }
        return;
    }

    if (doc.containsKey("clear_trip")) {                // liberar disparo térmico
        pedirClearTrip();
        return;
    }

    int canal = doc["ch"]  | 0;
    int valor = doc["val"] | 0;
    if (canal < 1 || canal > NUM_CANALES) return;
    aplicarIntensidad(canal - 1, valor, ORIGEN_MQTT);
    Serial.printf("MQTT[%s] canal %d = %d%%\n", topic, canal, valor);
}

void publicarEstado() {
    if (!mqtt.connected()) return;
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    int  ga  = grupoActivo;
    Modo md  = modo;
    xSemaphoreGive(stateMutex);

    // [M18] AQUI HABIA UN marcarPresencia(). Renovaba el arriendo cada 3 s
    // desde el propio latido del equipo, de modo que arriendoActivo() se
    // reducia a "hubo una orden en las ultimas 4 h" y el hombre muerto
    // desaparecia: el ESP32 se sostenia la carga hablando consigo mismo.
    // La presencia tiene que venir de ARRIBA, nunca de aqui.

    // Copias no volátiles: ArduinoJson no acepta operandos volatile.

    uint8_t flags = slaveFlags;
    float   rt    = resistorTemp,  at = ambientTemp,  ah = ambientHum;

    // 1536 y no 896: el estado paso de ~330 a ~470 caracteres al anadir fw,
    // ip, rssi, lineas y lease, y ArduinoJson no avisa cuando se queda corto
    // —simplemente deja campos fuera—. Con RAM al 15 % el margen es gratis.
    StaticJsonDocument<2048> doc;
    JsonArray arr = doc.createNestedArray("v");
    for (int i = 0; i < NUM_CANALES; i++) arr.add(intensidad[i]);
    doc["g"] = ga;                       // compatibilidad con v1
    doc["t"] = at;
    doc["h"] = ah;
    // Campos nuevos, aditivos
    doc["online"] = true;                                     // [M3]
    doc["modo"]   = (md == MODO_AUTO) ? "auto" : "aula";      // [M1]
    doc["rt"]     = rt;
    doc["cap"]    = (uint8_t)maxPowerLimit;
    doc["fans"]   = (bool)fansOn;
    doc["fans_held"] = (bool)(fansOn && !fansForzados);        // [M21] los requiere [M10]
    doc["fans_mode"] = fansForzados ? "forced" : "auto";        // [M33]
    JsonArray ap = doc.createNestedArray("applied");          // [M8] lo real
    for (int i = 0; i < NUM_CANALES; i++) ap.add((uint8_t)slaveApplied[i]);
    doc["sl_ok"]    = (bool)slaveOnline;
    doc["sl_flags"] = flags;
    doc["trip"]     = (flags & SL_TRIP)    != 0;
    doc["deadman"]  = (flags & SL_DEADMAN) != 0;
    doc["temp_ok"]  = (flags & SL_TEMP_MALA) == 0;            // [M9]
    doc["fans_ok"]  = (flags & SL_FANS_OK) != 0;              // [M10]
    doc["div"]      = (bool)slaveDiverged;                    // [M12]
    doc["fw"]       = FW_VERSION;                             // [M19]
    doc["build"]    = FW_BUILD;                               // [M32]
    doc["boot_id"]  = bootId;
    doc["reset_reason"] = resetReason;                        // [M33]
    doc["uptime_s"] = millis() / 1000;
    doc["clock_ok"] = (time(nullptr) >= (time_t)CLOCK_MIN_EPOCH);
    // [M22] Umbrales vigentes y si el esclavo los confirma. `tw_ok` falso
    // significa que el esclavo NO esta usando el umbral que creemos haberle
    // dado, que es exactamente el defecto de v3.1 hecho visible.
    doc["twarn"]    = (uint8_t)tempWarnC;
    doc["ttrip"]    = (uint8_t)tempTripC;
    doc["tw_ok"]    = (slaveWarnC == tempWarnC && slaveTripC == tempTripC);
    // [M21] Por que dejo de conducir, y salud del arriendo.
    doc["revocs"]     = (uint8_t)slaveRevocs;
    doc["revoc_why"]  = (uint8_t)slaveRevocReason;   // 1 deadman 2 trip 3 dif.
    doc["lease_age"]  = (uint16_t)slaveLeaseAgeDs;   // decimas de segundo
    doc["lease_gap"]  = (uint16_t)slaveLeaseGapDs;   // PEOR hueco, decimas
    doc["lease_tx"]   = (uint32_t)leaseTxCount;
    doc["lease_rx"]   = (uint16_t)slaveLeaseRx;
    doc["queue_drop"] = (uint32_t)i2cQueueDrops;
    doc["queue_peak"] = (uint8_t)i2cQueuePeak;
    doc["reauth_count"] = (uint32_t)reauthCount;
    doc["tlimit_src"] = tlimitSrc;
    doc["modo_src"]     = MODO_SRC_TXT[modoSrc];              // [M33]
    doc["modo_ip"]      = modoIp;
    doc["modo_since_s"] = (millis() - modoDesdeMs) / 1000;
    doc["modo_epoch"]   = (uint32_t)modoEpoch;
    doc["i2c_err"]    = (uint16_t)i2cWriteErrors;
    doc["bad_frames"] = (uint16_t)slaveBadFrames;
    doc["token_default"] = tokenEsRelleno();                  // [M20]
    // [M21] El equipo ANUNCIA lo que sabe hacer. El SCADA no debe deducirlo de
    // un numero de version: `supports_mqtt_supervisory()` lo hacia asi y dio
    // verdadero para el v3.1, que no lo soportaba. Una version no es una
    // capacidad; esto si lo es.
    JsonArray caps = doc.createNestedArray("caps");
    caps.add("supervisory");     // presencia, umbrales y grupo por MQTT
    caps.add("tlimit");          // umbrales termicos ajustables
    caps.add("leasediag");       // diagnostico del arriendo
    caps.add("lease_source_bound"); // [M29] mando y presencia comparten ruta
    caps.add("identity_v2");      // MAC STA, SSID, build, boot_id y reloj
    caps.add("tls_ca");           // certificado del broker validado
    caps.add("fans_auto");        // [M33] apagado automatico en frio por defecto
    caps.add("boot_reason");      // [M33] reset_reason y procedencia del modo
    doc["lease_owner"] = origenArriendo == ORIGEN_MQTT ? "mqtt" :
                         (origenArriendo == ORIGEN_HTTP ? "http" : "none");
    doc["ip"]       = WiFi.localIP().toString();              // [M19]
    doc["mac"]      = WiFi.macAddress();                      // [M32] MAC STA
    doc["ssid"]     = WiFi.SSID();
    doc["rssi"]     = WiFi.RSSI();                            // [M19]
    // [M19] El mapeo fisico viaja tambien por MQTT. Es una constante del
    // cableado, no un dato vivo, pero es justo lo que hay que tener a mano
    // cuando se compara `fases` con cons_l1..l3 del Cerbo, y quien lee el
    // estado por MQTT no tiene por que poder alcanzar /values.
    JsonArray ln = doc.createNestedArray("lineas");
    for (int i = 0; i < 3; i++) ln.add(LINEA_FISICA[i]);
    doc["lease"]    = (bool)arriendoVigente;                  // [M14]
    JsonArray ph = doc.createNestedArray("fases");            // R, S, T vivas
    ph.add((flags & SL_FASE_R) != 0);
    ph.add((flags & SL_FASE_S) != 0);
    ph.add((flags & SL_FASE_T) != 0);

    char buf[1536];
    // Si el documento se hubiera quedado corto, esto lo dice en vez de
    // publicar un estado incompleto que parece sano.
    size_t n = serializeJson(doc, buf, sizeof(buf));
    if (doc.overflowed() || n == 0) {
        Serial.println("ERROR: el estado no cupo en el documento JSON");
        return;
    }
    mqtt.publish(TOPICO_ESTADO, buf, true);
}

// ============================================================
//  [M4] AUTENTICACIÓN
// ============================================================
static bool autorizado(bool permitirQuery = true) {
    if (server.hasHeader("X-API-Key") && server.header("X-API-Key") == API_TOKEN)
        return true;
    if (permitirQuery && server.hasArg("key") && server.arg("key") == API_TOKEN)
        return true;
    server.send(401, "text/plain", "No autorizado");
    return false;
}

// ============================================================
//  HTTP
// ============================================================
extern const char PAGINA_HTML[];

void manejarRaiz() {
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "text/html", PAGINA_HTML);
}

void manejarSet() {
    if (!autorizado()) return;
    // [M1] Una sola fuente gobierna el actuador. En AUTO manda el SCADA; el
    // deslizador de la web no compite con él en silencio.
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    Modo md = modo;
    xSemaphoreGive(stateMutex);
    if (md == MODO_AUTO) {
        server.send(409, "text/plain",
                    "En modo AUTO manda el SCADA. Pase a modo AULA para el "
                    "control manual.");
        return;
    }
    if (!server.hasArg("c") || !server.hasArg("v")) {
        server.send(400, "text/plain", "Faltan c y v"); return; }
    int c = server.arg("c").toInt(), v = server.arg("v").toInt();
    if (c < 1 || c > NUM_CANALES || v < 0 || v > 100) {
        server.send(400, "text/plain", "Rango invalido"); return; }
    aplicarIntensidad(c - 1, v, ORIGEN_HTTP);
    server.send(200, "text/plain", "OK");
}

void manejarValues() {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    int ga = grupoActivo; Modo md = modo; uint8_t cap = maxPowerLimit;
    xSemaphoreGive(stateMutex);

    // [M18] AQUI HABIA OTRO marcarPresencia(). /values es una lectura, y el
    // SCADA la sondea cada 10 s las 24 h del dia para registrar temperatura:
    // con esto, la vigilancia pasiva sostenia la actuacion. Un sistema en el
    // que MIRAR mantiene la carga encendida no tiene hombre muerto.
    // La intencion de [M14] (pagina abierta = presencia) se conserva, pero
    // ahora hay que DECLARARLA: POST /presencia, mas abajo.

    uint8_t flags = slaveFlags;
    float   rt    = resistorTemp,  at = ambientTemp,  ah = ambientHum;

    StaticJsonDocument<2048> doc;
    JsonArray arr = doc.createNestedArray("valores");
    for (int i = 0; i < NUM_CANALES; i++) arr.add(intensidad[i]);
    JsonArray ap = doc.createNestedArray("aplicados");
    for (int i = 0; i < NUM_CANALES; i++) ap.add((uint8_t)slaveApplied[i]);
    doc["grupo"]        = ga;
    doc["modo"]         = (md == MODO_AUTO) ? "auto" : "aula";
    doc["mqtt"]         = mqttIsConnected ? "Conectado" : "Desconectado";
    doc["resistorTemp"] = rt;
    doc["ambientTemp"]  = at;
    doc["ambientHum"]   = ah;
    doc["fansOn"]       = (bool)fansOn;
    doc["fansRetenidos"]= (bool)(fansOn && !fansForzados);    // [M10]
    doc["fans_mode"]    = fansForzados ? "forced" : "auto";   // [M33]
    doc["maxLimit"]     = cap;
    doc["esclavo"]      = (bool)slaveOnline;
    doc["trip"]         = (flags & SL_TRIP)    != 0;
    doc["deadman"]      = (flags & SL_DEADMAN) != 0;
    doc["tempOk"]       = (flags & SL_TEMP_MALA) == 0;
    doc["fansOk"]       = (flags & SL_FANS_OK) != 0;
    doc["divergencia"]  = (bool)slaveDiverged;
    JsonArray ph = doc.createNestedArray("fases");
    ph.add((flags & SL_FASE_R) != 0);
    ph.add((flags & SL_FASE_S) != 0);
    ph.add((flags & SL_FASE_T) != 0);
    doc["arriendo"]     = (bool)arriendoVigente;              // [M14]
    doc["lease_owner"]  = origenArriendo == ORIGEN_MQTT ? "mqtt" :
                           (origenArriendo == ORIGEN_HTTP ? "http" : "none");
    doc["sesion_s"]     = ultimaOrdenReal ? (millis() - ultimaOrdenReal) / 1000 : 0;
    JsonArray ln = doc.createNestedArray("lineas");           // [M16]
    for (int i = 0; i < 3; i++) ln.add(LINEA_FISICA[i]);
    JsonArray wv = doc.createNestedArray("ventanas_us");      // [M19]
    for (int i = 0; i < 3; i++) wv.add((uint8_t)slaveWindow[i]);
    JsonArray ev = doc.createNestedArray("flancos_s");        // [M20]
    for (int i = 0; i < 3; i++) ev.add((uint16_t)slaveEdges[i]);
    doc["twarn"]        = (uint8_t)tempWarnC;             // [M22]
    doc["ttrip"]        = (uint8_t)tempTripC;
    doc["tw_ok"]        = (slaveWarnC == tempWarnC && slaveTripC == tempTripC);
    doc["revocs"]       = (uint8_t)slaveRevocs;           // [M21]
    doc["revoc_why"]    = (uint8_t)slaveRevocReason;
    doc["lease_age"]    = (uint16_t)slaveLeaseAgeDs;
    doc["lease_gap"]    = (uint16_t)slaveLeaseGapDs;
    doc["lease_tx"]     = (uint32_t)leaseTxCount;
    doc["lease_rx"]     = (uint16_t)slaveLeaseRx;
    doc["i2c_err"]      = (uint16_t)i2cWriteErrors;
    doc["bad_frames"]   = (uint16_t)slaveBadFrames;
    doc["queue_drop"]   = (uint32_t)i2cQueueDrops;
    doc["queue_peak"]   = (uint8_t)i2cQueuePeak;
    doc["reauth_count"] = (uint32_t)reauthCount;
    doc["tlimit_src"]   = tlimitSrc;
    doc["modo_src"]     = MODO_SRC_TXT[modoSrc];          // [M33]
    doc["modo_ip"]      = modoIp;
    doc["modo_since_s"] = (millis() - modoDesdeMs) / 1000;
    doc["modo_epoch"]   = (uint32_t)modoEpoch;
    doc["fw"]           = FW_VERSION;                     // [M19]
    doc["build"]        = FW_BUILD;                       // [M32]
    doc["boot_id"]      = bootId;
    doc["reset_reason"] = resetReason;                   // [M33]
    doc["token_default"]= tokenEsRelleno();              // [M20]
    doc["ip"]           = WiFi.localIP().toString();      // [M19]
    doc["mac"]          = WiFi.macAddress();              // MAC de estacion
    doc["ssid"]         = WiFi.SSID();
    doc["rssi"]         = WiFi.RSSI();
    doc["uptime_s"]     = millis() / 1000;
    doc["clock_ok"]     = (time(nullptr) >= (time_t)CLOCK_MIN_EPOCH);
    doc["epoch"]        = (uint32_t)time(nullptr);

    char buf[2048];
    size_t nb = serializeJson(doc, buf, sizeof(buf));
    if (doc.overflowed() || nb == 0) {
        server.send(500, "text/plain", "El estado no cupo en el documento JSON");
        return;
    }
    server.send(200, "application/json", buf);
}

// ============================================================
//  [M23] RECONSTRUCCION DE FASE
//
//  Con dos fases enganchadas, el desfase de la tercera queda determinado:
//      offset(p desde ref) = half - delta(ref -> la tercera)
//  Permite recuperar los canales de una fase cuyo detector de cruce esta
//  averiado, sin tocar el hardware.
//
//  Uso: POST /sync?f=0&ref=1 con X-API-Key  fase 0 (R) desde 1 (S)
//       POST /sync?f=0&ref=-1 con X-API-Key vuelve al detector propio
//  El token nunca viaja en la URL.
//
//  ES UN PARCHE, NO UNA ARQUITECTURA. Introduce un punto unico de fallo
//  silencioso: si el detector de la referencia se degrada, arrastra a la fase
//  reconstruida sin avisar. Verificar con pinza que las corrientes de los
//  canales recuperados sean equilibradas antes de dejarlo operando.
// ============================================================
void manejarSync() {
    if (!autorizado(false)) return;  // [M30] nunca filtrar token en URL
    if (!server.hasArg("f") || !server.hasArg("ref")) {
        server.send(400, "text/plain", "Faltan f y ref"); return;
    }
    int f   = server.arg("f").toInt();
    int ref = server.arg("ref").toInt();
    if (f < 0 || f > 2 || ref < -1 || ref > 2 || ref == f) {
        server.send(400, "text/plain", "Parametros invalidos"); return;
    }
    // Byte1 = fase<<4 | (ref+1);  ref+1 == 0 significa detector propio.
    uint8_t b = (uint8_t)((f << 4) | ((ref + 1) & 0x0F));
    uint8_t d[2] = { CMD_SET_SRC, b };
    if (!escribirI2C(d, 2, "fuente de sincronismo")) {      // [M21]
        server.send(502, "text/plain", "I2C fallo tras reintentos");
        return;
    }
    phaseSrc[f] = (int8_t)ref;                              // [M26] persistir
    guardarConfig();

    char msg[160];
    // El esclavo puede rechazarlo si las otras dos fases no estan enganchadas
    // o si el desfase medido no cae cerca de 60 o 120 grados. Se comprueba
    // mirando si la fase pasa a VIVA en el siguiente ciclo de estado.
    snprintf(msg, sizeof(msg),
             "Enviado. Revisa si %s pasa a VIVA en 1-2 s; si sigue MUERTA, "
             "el esclavo rechazo el desfase.", LINEA_FISICA[f]);
    server.send(200, "text/plain", msg);
}

void manejarGrupo() {
    if (!autorizado()) return;
    if (!server.hasArg("g")) { server.send(400, "text/plain", "Falta g"); return; }
    int g = server.arg("g").toInt();
    if (g < 0 || g > 10) { server.send(400, "text/plain", "Grupo invalido"); return; }
    cambiarGrupo(g);
    server.send(200, "text/plain", "OK");
}

// [M1] POST /modo?m=aula|auto   ("scada" se acepta como sinónimo de "auto")
void manejarModo() {
    if (!autorizado()) return;
    if (!server.hasArg("m")) { server.send(400, "text/plain", "Falta m"); return; }
    String m = server.arg("m");
    if (m != "aula" && m != "auto" && m != "scada") {
        server.send(400, "text/plain", "Modo invalido"); return;
    }
    // [M33] Quien lo pidio. Se deja antes de encolar; aplicarModo lo adopta.
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    strlcpy(modoIpPendiente, server.client().remoteIP().toString().c_str(),
            sizeof(modoIpPendiente));
    xSemaphoreGive(stateMutex);
    cambiarModo(m == "aula" ? MODO_AULA : MODO_AUTO, MODO_SRC_HTTP);
    server.send(200, "text/plain", "OK");
}

void manejarMaxPower() {
    if (!autorizado()) return;
    if (!server.hasArg("v")) { server.send(400, "text/plain", "Falta v"); return; }
    int v = server.arg("v").toInt();
    if (v < 0 || v > 100) { server.send(400, "text/plain", "Rango invalido"); return; }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    maxPowerLimit = (uint8_t)v;
    xSemaphoreGive(stateMutex);
    guardarConfig();                                     // [M2]
    // [M23] Hasta v3.1 esto guardaba en NVS, respondia OK y no le decia nada al
    // esclavo, de modo que SU clamp —el que sobrevive si este equipo cae— se
    // quedaba en el valor con el que hubiera arrancado. Bajar un tope nunca
    // energiza nada, asi que es seguro enviarlo desde aqui.
    uint8_t d[2] = { CMD_SET_CAP, (uint8_t)v };
    escribirI2C(d, 2, "tope de potencia");
    server.send(200, "text/plain", "OK");
}

// [M22] POST /tlimit?warn=<C>&trip=<C>
//
// Los umbrales termicos dejan de exigir reflasheo. Este equipo es el dueño del
// valor: lo guarda en NVS y lo empuja al esclavo cada segundo. Los TOPES DUROS
// (200/215) viven en el esclavo y son de compilacion alli, que es donde tienen
// que estar: la proteccion no puede depender de que este equipo siga vivo. Lo
// que se pida por encima se recorta, aqui y otra vez alla.
void manejarTLimit() {
    if (!autorizado()) return;
    if (!server.hasArg("warn")) {
        server.send(400, "text/plain", "Falta warn"); return; }
    int w = server.arg("warn").toInt();
    int t = server.hasArg("trip") ? server.arg("trip").toInt() : w + TEMP_MARGEN;
    if (w < 40)  w = 40;
    if (w > TEMP_WARN_TOPE) w = TEMP_WARN_TOPE;
    if (t < w + TEMP_MARGEN) t = w + TEMP_MARGEN;
    if (t > TEMP_TRIP_TOPE) { t = TEMP_TRIP_TOPE; w = t - TEMP_MARGEN; }
    float actual = resistorTemp;
    if (!isnan(actual) && t <= actual + TLIMIT_TEMP_MARGIN_C) {
        char err[160];
        snprintf(err, sizeof(err),
                 "No puedo poner el disparo en %d C con la resistencia a %.1f C; "
                 "se exigen %.0f C de margen.", t, actual, TLIMIT_TEMP_MARGIN_C);
        server.send(409, "text/plain", err);
        return;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    tempWarnC = (uint8_t)w; tempTripC = (uint8_t)t;
    tlimitSrc = "http";
    xSemaphoreGive(stateMutex);
    guardarConfig();
    enviarUmbrales();
    char msg[160];
    snprintf(msg, sizeof(msg),
             "Umbrales: recorte %d C, disparo %d C. Confirmalo en tw_ok del "
             "estado, que compara con el eco del esclavo.", w, t);
    server.send(200, "text/plain", msg);
}

// [M10] Encender siempre se obedece (y desde [M33] queda FORZADO). Apagar
// devuelve el control al enclavamiento; si este aun los requiere, se responde
// 409 y se dice por qué.
void manejarFans() {
    if (!autorizado()) return;
    if (!server.hasArg("on")) { server.send(400, "text/plain", "Falta on"); return; }
    bool on = (server.arg("on").toInt() != 0);
    fansForzados = on;                                  // [M33]
    actualizarVentiladores();
    if (!on && fansOn) {
        server.send(409, "text/plain",
                    "Ventiladores retenidos por enclavamiento: hay carga activa, "
                    "post-enfriamiento en curso, o la resistencia sigue caliente. "
                    "Se apagaran solos cuando sea seguro.");
        return;
    }
    server.send(200, "text/plain", "OK");
}

// [M18] PRESENCIA DECLARADA, no inferida.
//
// Sostiene el arriendo sin ordenar nada. La pagina la llama en su temporizador,
// de modo que "mientras la pagina siga abierta el banco sigue activo" —la
// politica de [M14]— se cumple igual que antes. La diferencia es que ahora es
// una AFIRMACION deliberada de un cliente que quiere la carga encendida, y no
// un efecto colateral de leer.
//
// No puede energizar nada por si sola: arriendoActivo() sigue exigiendo que
// haya habido una ORDEN real, y el tope de sesion de 4 h se mide desde esa
// orden y no se renueva aqui. Pide token porque es una accion de control.
void manejarPresencia() {
    if (!autorizado()) return;
    if (!marcarPresencia(ORIGEN_HTTP)) {
        server.send(409, "text/plain",
                    "La orden vigente no pertenece a HTTP; presencia rechazada");
        return;
    }
    server.send(200, "text/plain", "OK");
}

void manejarClearTrip() {
    if (!autorizado()) return;
    // Encolado: el bus I2C se usa exclusivamente desde el núcleo 0.
    pedirClearTrip();
    server.send(200, "text/plain", "OK");
}

// ============================================================
//  TAREA MQTT — núcleo 0
// ============================================================
void mqttTask(void* pv) {
    esp_task_wdt_add(NULL);                              // [M5]
    for (;;) {
        esp_task_wdt_reset();

        // [M13] Drenar la FIFO, no un solo elemento. El tope evita que una
        // ráfaga monopolice el lazo y dispare el perro guardián.
        I2CCmd icmd;
        int guard = 0;
        while (guard++ < NUM_CANALES &&
               xQueueReceive(i2cQueue, &icmd, 0) == pdTRUE) {
            // [M10] El esclavo exige confirmación de ventiladores para conducir.
            // Mandarla junto al comando elimina la espera de hasta 1 s que
            // habría si dependiera solo del ciclo de sincronización.
            enviarEstadoVentiladores();
            setDimmer(icmd.ch, icmd.val);
        }

        int pm;
        if (xQueueReceive(modoQueue, &pm, 0) == pdTRUE)  // [M1] diferido
            aplicarModo((Modo)(pm & 0xFF), (uint8_t)((pm >> 8) & 0xFF));

        int pg;
        if (xQueueReceive(grupoQueue, &pg, 0) == pdTRUE) {
            desuscribirTodo();
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            grupoActivo = pg;
            xSemaphoreGive(stateMutex);
            guardarConfig();                             // [M2]
            suscribirSegunModo();
        }

        int pt;
        if (xQueueReceive(tripQueue, &pt, 0) == pdTRUE) {
            Wire.beginTransmission(I2C_SLAVE_ADDR);
            Wire.write(CMD_CLEAR_TRIP);
            Wire.endTransmission();
        }

        // [M21] EL ARRIENDO VA EN SU PROPIO RELOJ, cada 500 ms, separado del
        // ciclo de sincronizacion. Dos razones. Una: duplica el numero de
        // tramas que hay que perder seguidas para que el esclavo revoque (60 en
        // vez de 30). Dos: lo saca de la rafaga de cuatro transacciones
        // consecutivas del bloque de abajo, que es justo donde una trama tiene
        // mas probabilidad de perderse si el esclavo esta ocupado.
        if (millis() - ultimoArriendo >= LEASE_SEND_MS) {
            ultimoArriendo = millis();
            enviarArriendo();
        }

        // [M7][M9][M10] Temperatura, ventiladores y lectura de vuelta.
        if (millis() - ultimoI2CSync >= INTERVALO_I2C_SYNC_MS) {
            ultimoI2CSync = millis();
            actualizarVentiladores();
            enviarTemperatura();
            enviarEstadoVentiladores();
            enviarUmbrales();                 // [M22] idempotente
            leerEstadoEsclavo();
            reautorizarSiDiverge();           // [M28]
            reportarEsclavo();                // [M18] visible en la consola
        }

        if (!mqtt.connected()) {
            mqttIsConnected = false;
            // [M29] No permitir que otra ruta sostenga una orden MQTT vieja.
            if (origenArriendo == ORIGEN_MQTT)
                revocarSetpoints("conexion MQTT perdida");
            if (millis() - ultimoIntento >= INTERVALO_RECONEXION_MS) {
                ultimoIntento = millis();
                if (WiFi.status() == WL_CONNECTED && relojValido()) {
                    Serial.print("Conectando al broker MQTT/TLS... ");
                    // [M3] Last Will retenido: si este equipo muere, el
                    // suscriptor lo sabe en lugar de leer para siempre un
                    // estado que parece un banco sano en reposo.
                    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                                     TOPICO_ESTADO, 1, true,
                                     "{\"online\":false}")) {
                        mqttIsConnected = true;
                        Serial.println("conectado.");
                        suscribirSegunModo();
                        // Identidad retenida inmediatamente tras cada conexion,
                        // sin esperar al periodo normal de tres segundos.
                        publicarEstado();
                    } else {
                        Serial.printf("fallo rc=%d\n", mqtt.state());
                    }
                } else if (WiFi.status() == WL_CONNECTED) {
                    Serial.println("MQTT/TLS bloqueado: reloj NTP no valido.");
                }
            }
        } else {
            mqttIsConnected = true;
            mqtt.loop();
            if (millis() - ultimoEstado > INTERVALO_ESTADO_MS) {
                ultimoEstado = millis();
                publicarEstado();
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ============================================================
//  [M6] WIFI
//
//  v1 conectaba una vez y no volvía a mirar; si fallaba, pasaba a modo AP de
//  forma permanente. Aquí el AP es un recurso temporal y siempre se vuelve a
//  intentar la red real.
// ============================================================
static void iniciarConexionWifi() {
    // `seneca` autentica por MAC y no tiene clave WiFi. Un password vacio debe
    // abrir la variante correcta de WiFi.begin(); pasarlo como WPA vacio no es
    // equivalente en todas las versiones de Arduino-ESP32.
    if (strlen(WIFI_PASSWORD) == 0) WiFi.begin(WIFI_SSID);
    else                            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

static bool relojValido() {
    return time(nullptr) >= (time_t)CLOCK_MIN_EPOCH;
}

static void solicitarNtp() {
    ultimoNtpIntento = millis();
    // UTC basta para validar certificados. La hora de operacion la conserva el
    // SCADA en America/Bogota; no se duplica aqui una zona horaria de negocio.
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);   // [M33]
}

static bool sincronizarReloj(uint32_t timeout_ms) {
    if (WiFi.status() != WL_CONNECTED) return false;
    solicitarNtp();
    uint32_t t0 = millis();
    while (!relojValido() && millis() - t0 < timeout_ms) {
        delay(250);
        esp_task_wdt_reset();
    }
    if (relojValido()) {
        Serial.printf("NTP: reloj valido, epoch=%lu. TLS habilitado.\n",
                      (unsigned long)time(nullptr));
        return true;
    }
    Serial.printf("NTP: sin hora valida tras %lu ms. MQTT/TLS queda BLOQUEADO; "
                  "HTTP e I2C siguen disponibles.\n", (unsigned long)timeout_ms);
    return false;
}

static void conectarWifi(uint32_t timeout_ms) {
    WiFi.mode(WIFI_STA);
    // [M22] Sin esto el ESP32 duerme entre balizas y la latencia salta de 3 ms
    // a 100 ms de forma erratica (se ve en el ping). Ademas puede descartar el
    // primer SYN de una conexion TCP, lo que hace que la pagina "a veces" no
    // cargue. Cuesta unos 50 mA extra, irrelevante en un equipo alimentado.
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    iniciarConexionWifi();
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) {
        delay(250);
        esp_task_wdt_reset();
    }
    if (WiFi.status() == WL_CONNECTED) {
        // [M32] La MAC es la de ESTACION que debe registrar Redes; no es la del
        // AP de respaldo ni la de Bluetooth. Tambien viaja retenida por MQTT.
        Serial.printf("WiFi: SSID=\"%s\"  MAC_STA=%s  IP=%s  GW=%s  "
                      "mascara=%s  RSSI=%d dBm\n",
                      WiFi.SSID().c_str(), WiFi.macAddress().c_str(),
                      WiFi.localIP().toString().c_str(),
                      WiFi.gatewayIP().toString().c_str(),
                      WiFi.subnetMask().toString().c_str(), WiFi.RSSI());
        Serial.printf("Pagina: http://%s:8080  (el servidor NO usa el puerto 80)\n",
                      WiFi.localIP().toString().c_str());
        if (MDNS.begin("solarmaster")) {
            MDNS.addService("http", "tcp", 8080);
            Serial.println("mDNS: http://solarmaster.local:8080");
        }
    } else {
        Serial.printf("WiFi: NO se pudo conectar a \"%s\". Se reintenta en "
                      "segundo plano.\n", WIFI_SSID);
    }
}

static void supervisarWifi() {
    if (millis() - ultimoWifiCheck < 10000) return;
    ultimoWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("WiFi caido (SSID objetivo \"%s\"): reconectando...\n", WIFI_SSID);
        WiFi.disconnect();
        iniciarConexionWifi();
        return;
    }
    // setCACert() exige fecha valida. El reintento es no bloqueante para que
    // una restriccion de UDP/123 en seneca no congele HTTP ni las protecciones.
    if (!relojValido() && millis() - ultimoNtpIntento >= NTP_RETRY_MS)
        solicitarNtp();
}

// [M33] Un corte del AC-out que alimenta el equipo aparece como POWERON (o
// BROWNOUT si la tension cae despacio); un fallo propio como PANIC o *_WDT.
// Esta semana hubo cuatro reinicios y solo se pudieron clasificar cruzando
// con el Cerbo.
static const char* textoMotivoReinicio(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    // [M5] NO se desactiva el brownout ni los perros guardianes. Si el equipo
    // se reinicia con ellos activos, eso es información sobre la alimentación,
    // no un problema a ocultar.
    Serial.begin(115200);
    delay(1000);
    snprintf(bootId, sizeof(bootId), "%08lx", (unsigned long)esp_random());
    resetReason = textoMotivoReinicio(esp_reset_reason());   // [M33]
    Serial.printf("\n=== SolarIoT WebMaster %s (%s), boot_id=%s, reset=%s ===\n",
                  FW_VERSION, FW_BUILD, bootId, resetReason);

    wdt_init_compat(WDT_TIMEOUT_S);
    esp_task_wdt_add(NULL);

    grupoQueue = xQueueCreate(1, sizeof(int));
    modoQueue  = xQueueCreate(1, sizeof(int));
    tripQueue  = xQueueCreate(1, sizeof(int));
    i2cQueue   = xQueueCreate(16, sizeof(I2CCmd));       // [M13] FIFO
    stateMutex = xSemaphoreCreateMutex();
    cfgMutex   = xSemaphoreCreateMutex();

    cargarConfig();                                      // [M2] (usa cfgMutex)

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeout(10);

    // [M10] Relé NC con disparo por nivel bajo: HIGH = desenergizado = NC
    // cerrado = VENTILADORES ON. El estado de reposo es el seguro.
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);
    fansOn = true;
    fansForzados = false;   // [M33] automatico: se apagan solos tras el post-enfriamiento
    ultimoCargaActiva = millis();

    dht.begin();

    conectarWifi(30000);                                 // [M6]
    sincronizarReloj(NTP_SYNC_TIMEOUT_MS);                // [M32] antes de TLS

    // [M13] Con el perro guardián en 10 s, un connect() TLS que se cuelgue
    // contra un broker inalcanzable provocaría un ciclo de reinicios. El
    // timeout de socket tiene que quedar por debajo del perro guardián.
    // [M31] CA privada de la planta: cifra y autentica al broker. MQTT_HOST
    // debe coincidir con un SAN del certificado (IP o DNS del broker).
    espClient.setCACert(MQTT_CA_CERT);
    espClient.setTimeout(SOCKET_TIMEOUT_S);
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(callbackMQTT);
    // [M21] 1280, no 768. El estado paso de ~470 a ~700 caracteres al añadir
    // el diagnostico del arriendo, y PubSubClient NO publica —en silencio— un
    // payload que no cabe en su buffer. Es el mismo modo de fallo que
    // ArduinoJson y hay que cerrarlo en los dos sitios.
    mqtt.setBufferSize(1536);
    mqtt.setSocketTimeout(SOCKET_TIMEOUT_S);

    const char* headers[] = { "X-API-Key" };
    server.collectHeaders(headers, 1);                   // [M4]
    server.on("/",          HTTP_GET,  manejarRaiz);
    server.on("/set",       HTTP_POST, manejarSet);
    server.on("/values",    HTTP_GET,  manejarValues);
    server.on("/grupo",     HTTP_POST, manejarGrupo);
    server.on("/modo",      HTTP_POST, manejarModo);
    server.on("/maxpower",  HTTP_POST, manejarMaxPower);
    server.on("/fans",      HTTP_POST, manejarFans);
    server.on("/cleartrip", HTTP_POST, manejarClearTrip);
    server.on("/presencia", HTTP_POST, manejarPresencia);   // [M18]
    server.on("/tlimit",    HTTP_POST, manejarTLimit);      // [M22]
    if (tokenEsRelleno())                                  // [M20]
        Serial.println("\n*** AVISO: API_TOKEN es el valor de ejemplo. Los "
                       "puntos de control estan efectivamente abiertos. "
                       "Genera uno: openssl rand -hex 24 ***\n");
    server.on("/sync",      manejarSync);                // [M23] cualquier metodo
    // [M23] Sin esto, cada favicon.ico de Chrome imprime "request handler not
    // found" y ensucia el log durante la puesta en marcha.
    server.onNotFound([]() { server.send(404, "text/plain", "No encontrado"); });
    server.begin();

    // [M11] Revocar ANTES de reponer configuracion. El esclavo puede no haberse
    // reiniciado y conservar carga; ninguna tarea de arranque debe prolongarla.
    for (int i = 0; i < NUM_CANALES; i++) {
        intensidad[i] = 0;
        setDimmer(i + 1, 0);
    }

    // [M30] Tope, ventiladores, umbrales y fuentes se reponen como unidad.
    reponerEstadoDelEsclavo("arranque del maestro");    // [M30][V1b]

    xTaskCreatePinnedToCore(mqttTask, "mqttTask", 12288, NULL, 1, NULL, 0);
    Serial.println("Listo.");
}

// ============================================================
//  LOOP — HTTP y sensores, núcleo 1
// ============================================================
void loop() {
    esp_task_wdt_reset();
    server.handleClient();
    supervisarWifi();                                    // [M6]

    uint32_t now = millis();

    if (now - ultimoMax6675 >= 1000) {
        ultimoMax6675 = now;
        float c = termopar.readCelsius();
        // [M9] Una lectura buena refresca la validez. Una mala NO conserva la
        // anterior indefinidamente: pasados TEMP_VALIDA_MS la damos por
        // perdida y el esclavo se entera. v1 hace `if (!isnan(c)) ...` y nada
        // más, que basta para una pantalla pero deja un termopar desconectado
        // invisible para siempre tras la primera lectura buena.
        if (!isnan(c)) {
            resistorTemp    = c;
            ultimaTempValida = now;
        } else if (now - ultimaTempValida > TEMP_VALIDA_MS) {
            resistorTemp = NAN;
        }
    }
    if (now - ultimoDHT >= 2000) {
        ultimoDHT = now;
        float t = dht.readTemperature(), h = dht.readHumidity();
        if (!isnan(t)) ambientTemp = t;
        if (!isnan(h)) ambientHum  = h;
    }

    actualizarVentiladores();                            // [M10]
}

// ============================================================
//  PÁGINA WEB
//  Igual que en v1, más el selector de modo real, el estado del esclavo y la
//  salud por fase. Las llamadas de control adjuntan el token.
//  NOTA: aquí ya NO existe el botón "Control Manual / Modo MQTT" de v1. Aquel
//  botón no cambiaba nada en el equipo (ver [M1]); su lugar lo ocupa el
//  selector de modo, que sí gobierna a qué escucha el firmware.
// ============================================================
const char PAGINA_HTML[] = R"rawhtml(
<!DOCTYPE html><html lang="es"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Web Master Solar v2</title><style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',Arial,sans-serif;background:#0d0d1a;color:#e0e0e0;padding:20px}
h1{text-align:center;color:#7eb8f7;font-size:1.5em;margin-bottom:4px}
.sub{text-align:center;color:#555;font-size:.78em;margin-bottom:20px}
.contenedor{max-width:560px;margin:0 auto}
.grupo-box{background:#0e1a2a;border:1px solid #1a3a5c;border-radius:12px;padding:14px 18px;margin-bottom:16px}
.grupo-box h2{font-size:.9em;color:#7eb8f7;margin-bottom:10px}
.grupo-fila{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
select,.num-in{background:#15152a;color:#e0e0e0;border:1px solid #252545;border-radius:8px;padding:6px 12px;font-size:.9em}
.num-in{width:90px}
.btn{background:#7eb8f7;color:#0d0d1a;border:none;border-radius:8px;padding:7px 18px;font-weight:700;cursor:pointer}
.btn-sec{background:#1e2e45;color:#7eb8f7;border:1px solid #2a4a6a;border-radius:8px;padding:7px 16px;font-weight:600;cursor:pointer}
.tarjeta{background:#15152a;border:1px solid #252545;border-radius:12px;padding:14px 18px;margin-bottom:10px}
.tarjeta.bloq{opacity:.45}
.fila{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:10px}
.valor{font-size:1.1em;font-weight:700;color:#7eb8f7}
.real{font-size:.75em;color:#8a8;margin-left:8px}
input[type=range]{width:100%;height:5px;-webkit-appearance:none;background:#252545;border-radius:3px}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;background:#7eb8f7}
input[type=range]:disabled::-webkit-slider-thumb{background:#555}
.estado-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;font-size:.82em;color:#bcd}
.estado-grid b{color:#7eb8f7}
.alarma{background:#3a1020;border-color:#e94560;color:#ff9db0}
.modo-auto{color:#4caf50;font-weight:700}
.modo-aula{color:#f0a500;font-weight:700}
.nota{font-size:.75em;color:#8a8;margin-top:6px}
.pie{text-align:center;margin-top:14px;font-size:.75em;color:#333}
</style></head><body>
<h1>Web Master Solar v2</h1>
<p class="sub">Control de carga resistiva &middot; modos AUTOMATICO / AULA</p>
<div class="contenedor">

<div class="grupo-box">
  <h2>Modo de operacion: <span id="modoTxt">--</span></h2>
  <div class="grupo-fila">
    <button class="btn" onclick="setModo('auto')">Modo AUTOMATICO</button>
    <button class="btn-sec" onclick="setModo('aula')">Modo AULA</button>
  </div>
  <p class="sub" style="text-align:left;margin:8px 0 0">
    AUTOMATICO: manda el SCADA por lab/solar/auto/cmd y los deslizadores de
    esta pagina quedan bloqueados. AULA: manda el grupo elegido y el control
    manual esta habilitado. El modo se conserva tras un reinicio.</p>
</div>

<div class="grupo-box">
  <h2>Grupo activo (solo en modo AULA)</h2>
  <div class="grupo-fila">
    <select id="sel"><option value="0">-- Ninguno --</option></select>
    <button class="btn" onclick="cambiarGrupo()">Aplicar</button>
  </div>
  <div class="sub" style="text-align:left;margin-top:6px" id="ga">Cargando...</div>
</div>

<div id="contenedor"></div>

<div class="grupo-box" id="boxEstado">
  <h2>Estado del sistema</h2>
  <div class="estado-grid">
    <div>Temp. resistencia: <b id="rtemp">--</b> &deg;C</div>
    <div>Temp. ambiente: <b id="atemp">--</b> &deg;C</div>
    <div>Humedad: <b id="ahum">--</b> %</div>
    <div>Ventiladores: <b id="fanst">--</b></div>
    <div>Esclavo I2C: <b id="escl">--</b></div>
    <div>Fases R/S/T: <b id="fases">--</b></div>
    <div>Disparo termico: <b id="trip">--</b></div>
    <div>Hombre muerto: <b id="dead">--</b></div>
    <div>Termopar: <b id="tempok">--</b></div>
    <div>Divergencia: <b id="div">--</b></div>
    <div>Revocaciones: <b id="revoc">--</b></div>
    <div>Arriendo: <b id="lease">--</b></div>
  </div>
  <div class="grupo-fila" style="margin-top:10px">
    <button class="btn-sec" onclick="clearTrip()">Liberar disparo</button>
  </div>
</div>

<div class="grupo-box">
  <h2>Seguridad y refrigeracion</h2>
  <div class="grupo-fila">
    <span>Token:</span><input class="num-in" id="tok" type="password" placeholder="API key">
    <button class="btn-sec" onclick="guardarTok()">Guardar</button>
  </div>
  <div class="grupo-fila" style="margin-top:8px">
    <span>Tope potencia (%):</span>
    <input type="number" class="num-in" id="maxp" min="0" max="100" value="80">
    <button class="btn" onclick="aplicarMaxPower()">Aplicar</button>
  </div>
  <div class="grupo-fila" style="margin-top:8px">
    <span>Ventiladores:</span>
    <button class="btn-sec" onclick="setFans(1)">ON</button>
    <button class="btn-sec" onclick="setFans(0)">OFF</button>
  </div>
  <div class="grupo-fila" style="margin-top:8px">
    <span>Temperatura (recorte / disparo, &deg;C):</span>
    <input type="number" class="num-in" id="twarn" min="40" max="200" value="120">
    <input type="number" class="num-in" id="ttrip" min="50" max="215" value="140">
    <button class="btn" onclick="aplicarTLimit()">Aplicar</button>
  </div>
  <p class="nota">El recorte empieza en el primero y el disparo enclavado en el
    segundo. Topes duros del esclavo: 200 y 215 &deg;C; lo que se pida por
    encima se recorta. El banco arranca en 120/140 tras un reinicio, a
    proposito: nadie deberia volver a 195 &deg;C sin pedirlo.</p>
  <p class="nota">Los ventiladores son autom&aacute;ticos: se encienden con
    carga, post-enfriamiento o resistencia caliente, y se apagan solos en
    fr&iacute;o. ON los fuerza encendidos (pruebas); OFF vuelve a autom&aacute;tico.</p>
</div>

<div class="grupo-box"><span id="mqtt-st">Iniciando...</span></div>
</div>
<p class="pie">Universidad de los Andes &middot; Facultad de Ingenier&iacute;a</p>
<script>
const N=6, cont=document.getElementById('contenedor'), timers={};
let tok=localStorage.getItem('wm_token')||'';
let modoActual='auto';
document.getElementById('tok').value=tok;
function guardarTok(){tok=document.getElementById('tok').value;localStorage.setItem('wm_token',tok);}
const sel=document.getElementById('sel');
for(let g=1;g<=10;g++){const o=document.createElement('option');o.value=g;o.textContent='Grupo '+g;sel.appendChild(o);}
for(let i=0;i<N;i++){
  const d=document.createElement('div');d.className='tarjeta';d.id='card'+i;
  d.innerHTML=`<div class="fila"><div><b>Canal ${i+1}</b><div class="sub" style="text-align:left">Fase ${'RRSSTT'[i]}</div></div>
    <div><span class="valor" id="val${i}">0%</span><span class="real" id="ap${i}"></span></div></div>
    <input type="range" id="sl${i}" min="0" max="100" value="0">`;
  cont.appendChild(d);
  document.getElementById('sl'+i).addEventListener('input',function(){
    const v=+this.value;document.getElementById('val'+i).textContent=v+'%';
    clearTimeout(timers[i]);timers[i]=setTimeout(()=>post('/set?c='+(i+1)+'&v='+v),150);});
}
async function post(p){try{const r=await fetch(p,{method:'POST',headers:{'X-API-Key':tok}});
  if(!r.ok){alert(await r.text());}}catch(e){}}
async function setModo(m){await post('/modo?m='+m);consultar();}
async function cambiarGrupo(){await post('/grupo?g='+sel.value);consultar();}
async function aplicarMaxPower(){await post('/maxpower?v='+document.getElementById('maxp').value);consultar();}
async function setFans(o){await post('/fans?on='+o);consultar();}
async function clearTrip(){await post('/cleartrip');consultar();}
async function aplicarTLimit(){
  await post('/tlimit?warn='+document.getElementById('twarn').value
             +'&trip='+document.getElementById('ttrip').value);
  consultar();}
function f(x){return (x===null||x===undefined)?'--':Number(x).toFixed(1);}
async function consultar(){
 try{
  const d=await (await fetch('/values')).json();
  modoActual=d.modo;
  const auto=(d.modo==='auto');
  document.getElementById('modoTxt').innerHTML=auto
    ?'<span class="modo-auto">AUTOMATICO (SCADA)</span>'
    :'<span class="modo-aula">AULA (clase)</span>';
  d.valores.forEach((v,i)=>{const s=document.getElementById('sl'+i);
    s.disabled=auto;
    document.getElementById('card'+i).className='tarjeta'+(auto?' bloq':'');
    if(!s.matches(':active')){s.value=v;document.getElementById('val'+i).textContent=v+'%';}
    document.getElementById('ap'+i).textContent='real '+d.aplicados[i]+'%';});
  sel.value=d.grupo;
  document.getElementById('ga').textContent=auto
    ?'Irrelevante en modo AUTOMATICO':(d.grupo===0?'Sin grupo activo':'Escuchando grupo '+d.grupo);
  document.getElementById('rtemp').textContent=f(d.resistorTemp);
  document.getElementById('atemp').textContent=f(d.ambientTemp);
  document.getElementById('ahum').textContent=f(d.ambientHum);
  document.getElementById('fanst').textContent=(d.fansOn?'ON':'OFF')+(d.fans_mode==='forced'?' (forzados)':' (auto)');
  document.getElementById('escl').textContent=d.esclavo?'OK':'SIN RESPUESTA';
  document.getElementById('fases').textContent=d.fases.map((x,i)=>('RST'[i])+(x?'+':'-')).join(' ');
  document.getElementById('trip').textContent=d.trip?'ACTIVO':'no';
  document.getElementById('dead').textContent=d.deadman?'ACTIVO':'no';
  document.getElementById('tempok').textContent=d.tempOk?'OK':'NO CONFIABLE';
  document.getElementById('div').textContent=d.divergencia?'SI':'no';
  /* [M21] Por que dejo de conducir. `deadman` de arriba se limpia solo; esto
     no, y por eso es lo que hay que mirar cuando el banco aparece apagado con
     todo en verde. */
  const WHY=['-','HOMBRE MUERTO','DISPARO TERMICO','peticion diferida'];
  document.getElementById('revoc').textContent=
    (d.revocs||0)+(d.revocs?' ('+(WHY[d.revoc_why]||'?')+')':'');
  document.getElementById('lease').textContent=
    (d.arriendo?'vigente':'SIN AFIRMAR')+' · edad '+((d.lease_age||0)/10).toFixed(1)
    +'s · peor hueco '+((d.lease_gap||0)/10).toFixed(1)+'s'
    +' · tx '+(d.lease_tx||0)+'/rx '+(d.lease_rx||0);
  const tw=document.getElementById('twarn'), tt=document.getElementById('ttrip');
  if(tw!==document.activeElement && d.twarn) tw.value=d.twarn;
  if(tt!==document.activeElement && d.ttrip) tt.value=d.ttrip;
  const mal=(d.trip||d.deadman||!d.esclavo||!d.tempOk||d.divergencia
             ||d.fases.includes(false)||d.tw_ok===false||d.revocs>0);
  document.getElementById('boxEstado').className='grupo-box'+(mal?' alarma':'');
  const mp=document.getElementById('maxp'); if(mp!==document.activeElement)mp.value=d.maxLimit;
  document.getElementById('mqtt-st').textContent='MQTT: '+d.mqtt+' | RSSI '+d.rssi
    +' dBm | up '+d.uptime_s+' s | fw '+(d.fw||'?')+' | '+(d.ip||'?')
    +(tok?'':'  |  SIN TOKEN: no se manda presencia, la carga caera en 30 s');
 }catch(e){document.getElementById('mqtt-st').textContent='Sin respuesta';}
}
consultar();setInterval(consultar,3000);

/* [M18] PRESENCIA DECLARADA.
   La politica de [M14] no cambia: mientras esta pagina siga abierta el banco
   permanece activo, y al cerrarla el maestro deja de afirmar el arriendo y el
   esclavo lleva la carga a cero en ~30 s.
   Lo que cambia es COMO se dice. Antes bastaba con leer /values, y como el
   SCADA sondea /values cada 10 s para registrar la temperatura, la vigilancia
   pasiva sostenia la actuacion y el hombre muerto no vencia nunca. Ahora la
   presencia se afirma explicitamente y solo la afirma quien quiere la carga
   encendida.
   Cada 10 s: cuatro veces por ventana de arriendo (25 s), asi que se pueden
   perder tres seguidas sin que el banco caiga. */
async function presencia(){ if(tok && modoActual==='aula') await post('/presencia'); }
presencia(); setInterval(presencia,10000);
</script></body></html>
)rawhtml";