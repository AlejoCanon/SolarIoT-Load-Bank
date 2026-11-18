/**
 * ============================================================
 *  SolarIoT_Slave v3.4 — fusión
 * ============================================================
 *
 *  Autor: Diego Cañón Borbón (d.canonb@uniandes.edu.co)
 *  Departamento de Ingeniería Eléctrica y Electrónica, Universidad de los Andes
 *
 *  Fusiona dos líneas de trabajo:
 *
 *  (A) Base de tiempo y estrategia de conmutación (Cañón, jul 2026):
 *      base enganchada por fase (PLL), clasificación de los dos flancos de la
 *      ventana de cruce, reconstrucción de una fase por desfase de 120°, tres
 *      modos de conmutación conmutables en caliente, linealización en potencia.
 *
 *  (B) Modelo de protección (González Mancera, ago 2026, firmware v2):
 *      salud de fase, protección térmica con validez explícita, hombre muerto,
 *      enclavamiento de ventiladores, protocolo I2C ampliado, perro guardián,
 *      y sobre todo la distinción REVOCACIÓN vs INHIBICIÓN, que es la idea
 *      más valiosa del aporte y se conserva íntegra.
 *
 *  ============================================================
 *  DEFECTO CORREGIDO DE v2: EL ARRIENDO NO TENÍA QUIEN LO RENOVARA
 *  ============================================================
 *  En v2 el hombre muerto se renueva SOLO con opcodes de canal (1..6), y el
 *  maestro los envía únicamente cuando alguien mueve un comando. Al no existir
 *  reenvío periódico, a los 30 s de cada comando el arriendo vence, se REVOCA
 *  (nivel 1), y la carga cae a cero sin volver. Ese es el motivo de que el
 *  banco no funcione con v2 flasheado.
 *
 *  La intención de v2 era correcta: el arriendo debe medir la frescura del
 *  MANDO SUPERIOR (SCADA/MQTT), no la salud del cable I2C. Por eso NO se
 *  resuelve reenviando setpoints, que volvería la protección decorativa.
 *
 *  Se resuelve con un opcode nuevo, CMD_LEASE: el maestro sabe cuándo recibió
 *  la última orden real del nivel superior y RELEVA esa frescura al esclavo
 *  cada segundo (1 = mando fresco, 0 = mando vencido). El arriendo se renueva
 *  con un comando de canal o con CMD_LEASE(1). Si el SCADA muere, el maestro
 *  deja de afirmar el arriendo y el banco cae igual que en el diseño de v2.
 *
 *  ============================================================
 *  v3.1 (González Mancera, 2026-09-09): TEMPERATURAS REALES
 *  ============================================================
 *  [C14] 90/110 C -> 200/215 C. Los umbrales viejos los puse yo SIN hoja de
 *        datos y estaban marcados como provisionales. El laboratorio ha
 *        llevado la resistencia a 200 C y el fabricante recomienda no superar
 *        esa cifra en regimen permanente. El recorte empieza ahora
 *        exactamente en ese techo, y el disparo enclavado 15 C mas arriba.
 *        Con 90/110, la operacion normal del 2026-09-09 (5,7 kW, resistencia
 *        a 115 C) habria enclavado el disparo y el banco habria parecido
 *        averiado estando sano.
 *
 *  [C15] LIMITE DE PENDIENTE ASIMETRICO. El recorte proporcional puro entro en
 *        ciclo limite en planta: con los seis canales pedidos al 100 %, lo
 *        aplicado oscilo entre 43 % y 90 % con periodo de minutos. Ganancia
 *        alta, retardo del sensor y una resistencia que a 5,7 kW sube de 21 a
 *        100 C en menos de tres minutos. Ahora el factor BAJA de golpe y SUBE
 *        con pendiente limitada (20 s de 0 a 100 %): un lazo que solo puede
 *        subir despacio no puede oscilar rapido, y la proteccion no pierde ni
 *        un milisegundo en el sentido que importa.
 *
 *        El regimen permanente de 195 C que quiere el laboratorio lo sostiene
 *        el SCADA, que puede anticipar; este recorte vuelve a ser lo que debia
 *        ser, el ultimo recurso que en operacion normal no se dispara nunca.
 *
 *  ============================================================
 *  v3.2 (González Mancera, 2026-09-10): EL HOMBRE MUERTO ERA EL QUE APAGABA
 *  ============================================================
 *  DIAGNOSTICO. El 2026-09-10 el banco se apago solo tres veces (10:07, 10:45
 *  y 11:23) tras 21, 35 y 5 minutos de operacion, y solo volvia con un comando
 *  nuevo. El registro del SCADA descarta todas las explicaciones comodas:
 *  `trip`=0 en los tres episodios (y el disparo termico es ENCLAVADO, asi que
 *  si hubiera actuado una sola vez seguiria en 1), `deadman`=0, `temp_ok`=1,
 *  `fans_ok`=1, `sl_ok`=1, las tres fases vivas, y la resistencia por debajo
 *  de 104 C contra un umbral de 110.
 *
 *  Si lo aplicado es 0 con el factor termico positivo y ninguna inhibicion
 *  puesta, entonces `power_pct` vale 0, y `power_pct` solo se borra por una
 *  REVOCACION. Hay exactamente dos, y el disparo termico esta descartado por
 *  no estar enclavado. Es el HOMBRE MUERTO, disparando de forma transitoria:
 *  no es enclavado, asi que a los 10 s —cuando el colector muestrea— la
 *  bandera ya se limpio, pero la revocacion que causo es permanente.
 *
 *  [C16] LIMITES TERMICOS AJUSTABLES EN CALIENTE, con tope duro. El umbral
 *        deja de ser una constante de compilacion: el maestro lo empuja con
 *        CMD_SET_TLIM y el esclavo lo recorta a 200/215 pase lo que pase. El
 *        arranque es CONSERVADOR (120/140) a proposito: un ESP32 que se
 *        reinicia solo no debe volver a 195 C sin que nadie lo haya pedido.
 *        Efecto lateral que importa: el maestro ya no necesita una COPIA del
 *        umbral para construir su expectativa, y esa copia era un defecto.
 *
 *  [C17] INSTRUMENTACION DE LA REVOCACION. La pregunta «por que se apago» no
 *        se podia contestar desde fuera, y esa es la razon de que este fallo
 *        sobreviviera un mes. Ahora el esclavo publica QUIEN revoco
 *        (`revoc_reason`, latcheado hasta el siguiente comando de canal),
 *        CUANTAS veces (`revoc_count`), la EDAD del arriendo y —el numero que
 *        de verdad caza este fallo— el HUECO MAXIMO entre renovaciones del
 *        arriendo desde el arranque. Si el arriendo se esta perdiendo a
 *        rachas, `lease_gap_max` sube mucho antes de que llegue a 30 s.
 *        Mas: cuenta las tramas mal formadas que antes se descartaban en
 *        silencio, y las de arriendo recibidas, para contrastarlas con las
 *        que el maestro dice haber enviado.
 *
 *  [C18] DEBUG_SERIAL a 0. printDebug() escribe ~570 bytes cada 2 s y a 115200
 *        baudios eso BLOQUEA loop() entre 25 y 50 ms. loop() es quien dispara
 *        los triacs y quien atiende el bus: son 3 a 6 semiciclos perdidos en
 *        los seis canales a la vez, cada 2 s, y es el principal sospechoso de
 *        que se pierdan tramas I2C. Un diagnostico que altera lo que mide no
 *        puede quedarse encendido en produccion.
 *
 *  ============================================================
 *  MODOS DE CONTROL (conmutables en caliente por Serial o I2C)
 *  ============================================================
 *   0 = MODE_PHASE         Ángulo de disparo, linealizado en potencia.
 *   1 = MODE_ICC_FULL      Ciclos integrales por CICLO COMPLETO. DC nula.
 *   2 = MODE_ICC_BALANCED  Ciclos integrales por semiciclo, |DC| <= 1.
 *
 *  NUNCA decidir por semiciclo sin restricción de balance: para la mayoría de
 *  los setpoints el acumulador elige siempre semiciclos de la misma polaridad
 *  y el resultado es rectificación de media onda (amperios de DC hacia el
 *  Quattro, que tiene protección de inyección DC).
 *
 *  ============================================================
 *  NOTA DE HARDWARE — POR QUÉ AQUÍ NO SE USA INPUT_PULLUP
 *  ============================================================
 *  v2 pone INPUT_PULLUP en los pines SYNC suponiendo detectores en colector
 *  abierto. Los módulos de este banco NO son colector abierto: son seguidores
 *  de emisor (colector a 5 V, resistencia a GND, salida en el emisor), que
 *  gobiernan activamente los dos niveles. El pull-up interno (~45 kΩ) forma un
 *  divisor con la resistencia de emisor y levanta el nivel BAJO: con 10 kΩ el
 *  cero se va a ~0,6 V contra un V_IL de 0,83 V, dejando 0,2 V de margen, y
 *  además frena el flanco de bajada. Es decir, ataca justo el mecanismo que
 *  está fallando. Aquí se deja INPUT.
 * ============================================================
 */

#include <Arduino.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <Wire.h>

// ============================================================
// MODOS
// ============================================================
#define MODE_PHASE          0
#define MODE_ICC_FULL       1
#define MODE_ICC_BALANCED   2
#define MODE_COUNT          3
#define DEFAULT_MODE        MODE_ICC_FULL

// ============================================================
// TIEMPOS DE DISPARO
// ============================================================
#define PULSE_US            300UL
#define MIN_FIRE_DELAY_US   900UL     // modo fase: tensión de enclavamiento
#define ICC_FIRE_DELAY_US   600UL     // modo ICC: lo más cerca del cruce posible
#define TAIL_GUARD_US       300UL
#define HALF_MIN_US         4000UL
#define HALF_MAX_US         12000UL
#define MAX_CATCHUP         8

// ============================================================
// VENTANA DE CRUCE POR CERO (los módulos entregan tren de pulsos a 120 Hz)
//
// MEDIDO EN EL EQUIPO (sep 2026). v3.3 reescribe este bloque: el texto anterior
// seguia defendiendo el flanco de CIERRE mientras el #define de abajo ya estaba
// en MIDPOINT. Se corrige el texto, que es el que estaba equivocado.
//
// El umbral de deteccion NO es la Vf del LED (1,2 V), sino la tension de red
// necesaria para que el fototransistor lleve el emisor por encima del V_IH del
// ESP32 (2,475 V). Con Rpd=10k, Rin=68k y CTR=100 % hacen falta 0,25 mA, o sea
// 18 V de red; a 0,064 V/us son 281 us por lado: VENTANA ~560 us, no 37. A CTR
// 300-400 % baja a 120-180 us, que es lo medido en S y T.
//
// De ahi que la referencia sea el PUNTO MEDIO y no el cierre: el cierre llega
// 120-280 us tarde (2,6 a 6,1 grados) segun el CTR de CADA modulo, o sea un
// error distinto por fase. El punto medio solo se desvia la mitad del storage
// time, 0,2 a 1,1 grados, y es el mismo criterio en las tres.
//
// Rebote: 300-730 flancos/s frente a los 240 de una senal limpia. Es la rampa
// analogica cruzando despacio el umbral CMOS, inherente a esta topologia; NO es
// precursor de saturacion. El modo de falla real es el de la fase R: CERO
// flancos, la salida no conmuta nunca. ZC_DEBOUNCE_US = 60 us colapsa cada
// rafaga en un flanco (los rebotes medidos van de 5 a 34 us) sin tocar la
// ventana, que es dos a diez veces mayor.

#define ZC_DEBOUNCE_US      60UL
#define ZC_WINDOW_MAX_US    2000UL
#define ZC_EDGE_LEADING     0
#define ZC_EDGE_TRAILING    1
#define ZC_EDGE_MIDPOINT    2
#define ZC_EDGE_MODE        ZC_EDGE_MIDPOINT

// ============================================================
// PLL
// ============================================================
#define PLL_LOCK_COUNT      8
#define PLL_PHASE_SHIFT     2
#define PLL_PERIOD_SHIFT    5
#define PLL_TIMEOUT_US      300000ULL
#define PLL_OUTLIER_DIV     3
#define SRC_HW              (-1)
#define PHASE_R_SRC         SRC_HW
#define PHASE_S_SRC         SRC_HW
#define PHASE_T_SRC         SRC_HW

// ============================================================
// PROTECCIONES (modelo de González Mancera)
// ============================================================
#define PHASE_TIMEOUT_MS    200UL     // fase sin ciclos válidos -> muerta
// [C14] TEMPERATURAS REALES DE LA RESISTENCIA (2026-09-09).
// Los 90/110 anteriores los puso González Mancera SIN hoja de datos, y estaban
// declarados como provisionales. Medidas del laboratorio: la resistencia se ha
// llevado hasta 200 C, el fabricante recomienda NO superar 200 C en regimen
// permanente, y 195 C es el punto en el que el laboratorio esta comodo.
//
// De donde sale cada numero:
//   WARN = 200  el recorte empieza EXACTAMENTE en el techo de regimen
//               permanente del fabricante. Por debajo, el firmware no
//               interviene; por encima, garantiza que no se sostenga ahi.
//   TRIP = 215  disparo enclavado. 15 C por encima del techo: ya no es
//               operacion, es una fuga termica.
//
// Con los umbrales viejos, la operacion normal del 2026-09-09 (5,7 kW, la
// resistencia a 115 C) habria enclavado el disparo y el banco habria parecido
// averiado estando sano. Ese es el fallo que esto corrige.
// [C16] Los umbrales son AJUSTABLES EN CALIENTE (CMD_SET_TLIM). Lo que sigue
// son los TOPES DUROS, que si son de compilacion: el esclavo recorta contra
// ellos cualquier valor que le manden, de modo que ningun error del nivel
// superior puede pedirle a la resistencia mas de lo que aguanta.
//   200 C  techo de regimen permanente que recomienda el fabricante
//   215 C  disparo enclavado: por encima ya no es operacion, es fuga termica
#define TEMP_WARN_MAX_C     200.0f
#define TEMP_TRIP_MAX_C     215.0f
// Arranque CONSERVADOR. Un equipo recien flasheado, o uno que se reinicio solo
// por perro guardian, no debe volver a 195 C porque alguien lo pidio hace una
// hora. El valor de operacion lo empuja el maestro cada segundo.
#define TEMP_WARN_BOOT_C    120.0f
#define TEMP_TRIP_BOOT_C    140.0f
#define TEMP_TRIP_MARGIN_C   10.0f    // trip siempre >= warn + esto

// [C15] LIMITE DE PENDIENTE ASIMETRICO PARA EL RECORTE.
//
// El recorte proporcional puro entra en CICLO LIMITE, y se observo en planta el
// 2026-09-09: con los seis canales pedidos al 100 %, lo aplicado oscilo entre
// 43 % y 90 % con periodo de minutos mientras la resistencia iba y venia
// alrededor del umbral. La causa es de manual: ganancia alta (todo el rango de
// potencia dentro de la banda), retardo del sensor, y una resistencia que a
// 5,7 kW pasa de 21 a 100 C en menos de tres minutos.
//
// La cura NO es bajar la ganancia sin mas, porque eso tambien enlentece la
// proteccion. Es hacerla ASIMETRICA: BAJAR de golpe (proteger es urgente),
// SUBIR despacio (recuperar no lo es). Un lazo que solo puede subir despacio
// no puede oscilar rapido, y la proteccion no pierde ni un milisegundo de
// reaccion en el sentido que importa.
//
// 0,5 % por cada 100 ms => del 0 al 100 % en 20 s. Suficientemente lento
// frente al periodo del ciclo limite observado, suficientemente rapido para
// que un enfriamiento real se aproveche en menos de medio minuto.
#define THERMAL_RISE_PER_100MS  0.005f
#define TEMP_STALE_MS       15000UL
#define DEADMAN_MS          30000UL   // arriendo del mando superior
#define MASTER_STALE_MS     5000UL    // silencio del maestro
#define WDT_TIMEOUT_S       5

// [C18] LA CONSOLA SE QUEDA; EL VOLCADO PERIODICO SE APAGA.
//
// En v3 esto valia 1 y su propio comentario decia «0 en produccion». No es un
// detalle cosmetico: printDebug() escribe ~570 bytes cada 2 s y a 115200
// baudios eso BLOQUEA loop() entre 25 y 50 ms. loop() es quien dispara los
// triacs y quien atiende el bus I2C, asi que cada 2 s se pierden 3 a 6
// semiciclos en los seis canales a la vez y se abre una ventana en la que el
// esclavo no puede responder al maestro. Es el principal sospechoso de las
// tramas de arriendo perdidas del 2026-09-10.
//
// Pero apagarlo del todo se lleva por delante la consola, que es LA
// herramienta de puesta en marcha del esclavo. Asi que se separan: el codigo
// de consola queda compilado (leer una tecla no cuesta nada) y el VOLCADO
// PERIODICO pasa a ser una peticion explicita en tiempo de ejecucion, apagada
// al arrancar. Se enciende con 'd' por el puerto serie mientras se depura, y
// entonces uno sabe que esta midiendo con el instrumento dentro del circuito.
#define DEBUG_SERIAL        1
static bool debug_stream = false;      // volcado periodico: apagado por defecto

#define ENABLE_I2C          1
#define I2C_SLAVE_ADDR      0x08

// ============================================================
// PROTOCOLO I2C (superconjunto del de v2; el legado de 2 bytes es idéntico)
//
// Maestro -> esclavo:
//   [1..6][0..100]   Fijar canal. Renueva el arriendo.
//   [0x10][hi][lo]   Temperatura, int16 en décimas de °C. 0x8000 = INVÁLIDA.
//   [0x11]           Latido del maestro (NO es actuación).
//   [0x12]           Liberar disparo térmico enclavado.
//   [0x13][0..100]   Tope de potencia.
//   [0x14][0|1]      Estado real de los ventiladores.
//   [0x15][0|1]      NUEVO. Arriendo del mando superior: 1 = el maestro tiene
//                    una orden fresca del SCADA/MQTT/web. Renueva el arriendo
//                    sin necesidad de reenviar setpoints.
//   [0x16][0..2]     NUEVO. Modo de conmutación.
//   [0x17][f<<4|r+1] Fuente de sincronismo de la fase f (r=0 -> ZCD propio;
//                    r=1..3 -> sintetizada desde la fase r-1).
//   [0x18][warn][trip]  NUEVO v3.2 [C16]. Umbrales termicos en C enteros. Se
//                    recortan contra TEMP_WARN_MAX_C / TEMP_TRIP_MAX_C y se
//                    fuerza trip >= warn + TEMP_TRIP_MARGIN_C. Un valor fuera
//                    de rango NO se rechaza: se recorta, porque rechazar en
//                    silencio dejaria al equipo con el umbral viejo creyendo
//                    el nivel superior que lo cambio.
//
// Esclavo -> maestro (32 bytes en v3.2; eran 20 en v3.0/v3.1):
//   [0..5]   potencia aplicada por canal
//   [6]      banderas: b0 trip térmico, b1 hombre muerto, b2/b3/b4 fase R/S/T
//            viva, b5 temperatura no confiable, b6 maestro mudo, b7 ventiladores
//   [7]      temperatura en °C (255 = desconocida)
//   [8..9]   segundos desde el arranque (si DISMINUYE, el esclavo se reinició)
//   [10]     modo de conmutación vigente
//   [11..13] ancho de la ventana de cruce por fase, en us (0 = ZCD muerto)
//   [14..19] flancos crudos por segundo por fase (uint16 x3)
//   --- v3.2 [C17]: por que dejo de conducir -------------------------------
//   [20]     revocaciones desde el arranque (satura en 255)
//   [21]     motivo de la ULTIMA revocacion, latcheado hasta el siguiente
//            comando de canal: 0 ninguna, 1 hombre muerto, 2 disparo termico,
//            3 peticion diferida
//   [22..23] edad del arriendo en decimas de segundo (uint16)
//   [24..25] HUECO MAXIMO entre renovaciones del arriendo desde el arranque,
//            en decimas de segundo. Este es el numero que caza el fallo del
//            2026-09-10: si sube acercandose a 300 (30 s), el arriendo se esta
//            perdiendo a rachas mucho antes de que llegue a revocar.
//   [26]     umbral de recorte vigente en C  (eco de CMD_SET_TLIM)
//   [27]     umbral de disparo vigente en C  (eco de CMD_SET_TLIM)
//   [28..29] tramas I2C mal formadas descartadas (uint16). Antes se tiraban en
//            silencio, que es como se pierde un arriendo sin dejar rastro.
//   [30..31] tramas CMD_LEASE recibidas (uint16). Se contrasta con las que el
//            maestro dice haber enviado: si no coinciden, el bus pierde tramas.
// ============================================================
#define CMD_SET_TEMP     0x10
#define CMD_HEARTBEAT    0x11
#define CMD_CLEAR_TRIP   0x12
#define CMD_SET_CAP      0x13
#define CMD_SET_FANS     0x14
#define CMD_LEASE        0x15
#define CMD_SET_MODE     0x16
#define CMD_SET_SRC      0x17
#define CMD_SET_TLIM     0x18    // [C16] v3.2

#define TEMP_INVALID_RAW ((int16_t)0x8000)
#define I2C_STATUS_LEN   32      // v3.2 [C17]: 20 -> 32

// ============================================================
// TIPOS
// ============================================================
typedef enum { PHASE_R = 0, PHASE_S = 1, PHASE_T = 2 } Phase;
const char* PHASE_NAMES[] = { "R", "S", "T" };
const char* MODE_NAMES[]  = { "ANGULO DE FASE", "CICLOS INTEGRALES (ciclo completo)",
                              "CICLOS INTEGRALES (semiciclo balanceado)" };

struct PhaseState {
    // --- escrito por el ISR ---
    volatile uint64_t hw_last_us;
    volatile uint32_t hw_seq;
    volatile uint64_t edge_last_us;
    volatile uint32_t window_us;
    volatile int8_t   window_level;
    volatile uint32_t raw_edges;      // flancos crudos, sin filtrar (diagnostico)
    uint32_t raw_prev;
    uint32_t raw_rate;
    // --- propiedad de loop() ---
    uint64_t next_edge_us;
    uint64_t last_edge_us;
    uint32_t half_us;
    uint32_t seq;
    uint32_t hw_seq_seen;
    uint64_t hw_prev_us;
    uint64_t last_ok_us;
    uint32_t last_valid_ms;
    uint16_t lock_count;
    bool     locked;
    bool     alive;
    int8_t   src;
    uint16_t offset_permil;
    const uint8_t sync_pin;
};

struct Dimmer {
    const uint8_t  ch_pin;
    const Phase    phase;
    uint8_t        power_pct;      // el ARRIENDO: lo último que pidió el mando
    uint8_t        applied_pct;    // lo realmente aplicado tras límites
    uint16_t       accumulator;
    const uint16_t acc_seed;
    int8_t         dc_balance;
    bool           active_now;
    bool           fired;
    uint64_t       fire_time_us;
    const bool     high_bank;
    const uint32_t pin_mask;
};

#define GPIO_HIGH(d) do { \
    if ((d).high_bank) GPIO.out1_w1ts.val = (d).pin_mask; \
    else               GPIO.out_w1ts      = (d).pin_mask; \
} while(0)

#define GPIO_LOW(d) do { \
    if ((d).high_bank) GPIO.out1_w1tc.val = (d).pin_mask; \
    else               GPIO.out_w1tc      = (d).pin_mask; \
} while(0)

static portMUX_TYPE phase_mux[3] = {
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
};

// hw_last, hw_seq, edge_last, window, wlevel, next, last, half, seq, seen,
// prev, ok, valid_ms, lockc, locked, alive, src, offset, pin
PhaseState phases[3] = {
    { 0,0,0,0,-1, 0,0,0, 0,0,0,0,0, 0,0,0,0, false,false, PHASE_R_SRC, 0,  4 },
    { 0,0,0,0,-1, 0,0,0, 0,0,0,0,0, 0,0,0,0, false,false, PHASE_S_SRC, 0, 13 },
    { 0,0,0,0,-1, 0,0,0, 0,0,0,0,0, 0,0,0,0, false,false, PHASE_T_SRC, 0, 14 },
};

// ch_pin, phase, pwr, applied, acc, seed, dcbal, active, fired, fire_t, high, mask
Dimmer dimmers[6] = {
    { 32, PHASE_R, 0, 0,  0,  0, 0, false, false, 0, true,  1UL << (32 - 32) },
    { 33, PHASE_R, 0, 0, 50, 50, 0, false, false, 0, true,  1UL << (33 - 32) },
    { 25, PHASE_S, 0, 0,  0,  0, 0, false, false, 0, false, 1UL << 25        },
    { 26, PHASE_S, 0, 0, 50, 50, 0, false, false, 0, false, 1UL << 26        },
    { 27, PHASE_T, 0, 0,  0,  0, 0, false, false, 0, false, 1UL << 27        },
    { 18, PHASE_T, 0, 0, 50, 50, 0, false, false, 0, false, 1UL << 18        },
};

// ============================================================
// ESTADO DE PROTECCIÓN
// ============================================================
volatile float    resistor_temp_c   = NAN;
volatile bool     temp_valid        = false;
volatile uint32_t last_temp_ms      = 0;
volatile uint32_t last_actuation_ms = 0;   // arriendo del mando superior
volatile uint32_t last_master_ms    = 0;   // cualquier tráfico del maestro
volatile bool     fans_on           = false;
volatile bool     thermal_trip      = false;
volatile bool     deadman_active    = true;
volatile bool     master_silent     = true;
volatile bool     fans_ok           = false;
volatile uint8_t  power_cap         = 80;
volatile uint8_t  control_mode      = DEFAULT_MODE;

// [C16] Umbrales termicos vigentes. Variables, no constantes: el maestro los
// empuja con CMD_SET_TLIM y aqui se recortan contra los topes duros.
// [C20] Peticion diferida de cambio de sincronismo. Hasta v3.2 setPhaseSource()
// se ejecutaba dentro del callback de I2C, reconfigurando next_edge_us, half_us
// y locked de una fase en paralelo con loop(), sin seccion critica. Ahora el
// callback solo deja la peticion y loop() la aplica en su propio contexto.
volatile bool     src_req_pending   = false;
volatile uint8_t  src_req_byte      = 0;

volatile float    temp_warn_c       = TEMP_WARN_BOOT_C;
volatile float    temp_trip_c       = TEMP_TRIP_BOOT_C;

// Petición diferida de revocación: el callback I2C no toca power_pct
// directamente cuando hay carrera con recompute_applied().
// [C19] pending_zero ELIMINADO en v3.3. Se declaraba y se leia pero nunca se
// escribia en todo el fichero: el mecanismo no estaba conectado, tal como
// senalaba la duda nº2. La carrera que pretendia cubrir (un comando que llega
// justo mientras recompute_applied() revoca) se resuelve aguas arriba: el
// maestro reenvia los setpoints al detectar divergencia con arriendo vigente.

// ============================================================
// [C17] INSTRUMENTACION DE LA REVOCACION
//
// El fallo del 2026-09-10 sobrevivio un mes porque el sistema no podia
// contestar «por que dejaste de conducir». Las banderas dicen el estado AHORA;
// una revocacion es un EVENTO, y su efecto (power_pct borrado) sobrevive a la
// condicion que lo causo. Por eso el motivo se LATCHEA hasta el siguiente
// comando de canal, en vez de limpiarse con la bandera.
// ============================================================
#define REVOC_NINGUNA    0
#define REVOC_DEADMAN    1
#define REVOC_TRIP       2
#define REVOC_PENDING    3
#define REVOC_EXPLICITA  4   // [C22] CMD_LEASE(0) del maestro

volatile uint8_t  revoc_count       = 0;   // satura en 255
volatile uint8_t  revoc_reason      = REVOC_NINGUNA;
volatile uint16_t frame_bad_count   = 0;   // tramas descartadas por mal formadas
volatile uint16_t cmd_lease_rx      = 0;   // CMD_LEASE recibidos
// Hueco maximo entre renovaciones del arriendo, en ms. Es el instrumento que
// caza un arriendo que se pierde a rachas ANTES de que llegue a los 30 s.
volatile uint32_t lease_gap_max_ms  = 0;
volatile bool     lease_session_open = false;  // v3.4: maxima por sesion activa
volatile bool     revoke_explicit    = false;  // [C22] ultimo cierre fue CMD_LEASE(0)
volatile bool     lease_dirty        = false;  // [C23] hubo hueco >= DEADMAN en la sesion

static uint16_t alpha_permil[101];

// ============================================================
// LINEALIZACIÓN EN POTENCIA
//   P(a)/Pmax = 1 - a/pi + sin(2a)/(2pi)   monótona decreciente en a
// El clamp de MIN_FIRE_DELAY_US limita el extremo real a ~99.19 %.
// ============================================================
static float power_fraction(float a) {
    return 1.0f - a / (float)M_PI + sinf(2.0f * a) / (2.0f * (float)M_PI);
}

static void build_linearization_table() {
    for (int pct = 0; pct <= 100; pct++) {
        if (pct == 0) { alpha_permil[pct] = 1000; continue; }
        float target = pct / 100.0f;
        float lo = 0.0f, hi = (float)M_PI;
        for (int it = 0; it < 40; it++) {
            float mid = 0.5f * (lo + hi);
            if (power_fraction(mid) > target) lo = mid; else hi = mid;
        }
        float a = 0.5f * (lo + hi);
        alpha_permil[pct] = (uint16_t)(1000.0f * a / (float)M_PI + 0.5f);
    }
}

// ============================================================
// ISRs — clasifican los dos flancos de la ventana de cruce.
// El nivel de la ventana se aprende solo, de modo que el mismo código sirve
// para pull-down, pull-up y módulos de onda cuadrada.
// ============================================================
static inline int IRAM_ATTR fastRead(uint8_t pin) {
    if (pin < 32) return (int)((GPIO.in >> pin) & 1u);
    return (int)((GPIO.in1.val >> (pin - 32)) & 1u);
}

static void IRAM_ATTR isr_common(int p, uint64_t now) {
    PhaseState &ph = phases[p];
    int level_now = fastRead(ph.sync_pin);

    portENTER_CRITICAL_ISR(&phase_mux[p]);
    ph.raw_edges++;                 // TODO flanco, antes del antirrebote
    uint32_t gap = (uint32_t)(now - ph.edge_last_us);
    if (gap < ZC_DEBOUNCE_US) { portEXIT_CRITICAL_ISR(&phase_mux[p]); return; }
    ph.edge_last_us = now;

    int level_before = level_now ? 0 : 1;
    uint64_t zc = 0;
    bool emit = false;

    if (gap <= ZC_WINDOW_MAX_US) {
        ph.window_level = (int8_t)level_before;      // cierre de la ventana
        ph.window_us    = gap;
#if   ZC_EDGE_MODE == ZC_EDGE_TRAILING
        zc = now;
#elif ZC_EDGE_MODE == ZC_EDGE_LEADING
        zc = now - gap;
#else
        zc = now - gap / 2;
#endif
        emit = true;
    } else if (gap >= HALF_MIN_US && gap <= HALF_MAX_US) {
        if (ph.window_level < 0) { zc = now; emit = true; }   // onda cuadrada
        // si ya conocemos la ventana, esto es su apertura: se ignora
    }

    if (emit && (zc - ph.hw_last_us) >= HALF_MIN_US) {
        ph.hw_last_us = zc;
        ph.hw_seq++;
    }
    portEXIT_CRITICAL_ISR(&phase_mux[p]);
}

void IRAM_ATTR isr_phase_r() { isr_common(PHASE_R, esp_timer_get_time()); }
void IRAM_ATTR isr_phase_s() { isr_common(PHASE_S, esp_timer_get_time()); }
void IRAM_ATTR isr_phase_t() { isr_common(PHASE_T, esp_timer_get_time()); }

static void read_hw(int p, uint64_t &t, uint32_t &s, uint32_t &w) {
    portENTER_CRITICAL(&phase_mux[p]);
    t = phases[p].hw_last_us;
    s = phases[p].hw_seq;
    w = phases[p].window_us;
    portEXIT_CRITICAL(&phase_mux[p]);
}

// ============================================================
// PASOS BRESENHAM
// ============================================================
static inline void iccStepFull(Dimmer &d, uint8_t pct) {
    if (pct == 0)   { d.accumulator = d.acc_seed; d.active_now = false; return; }
    if (pct >= 100) { d.accumulator = 0;          d.active_now = true;  return; }
    d.accumulator += pct;
    if (d.accumulator >= 100) { d.accumulator -= 100; d.active_now = true; }
    else                      { d.active_now = false; }
}

static inline void iccStepBalanced(Dimmer &d, uint8_t pct, int8_t pol) {
    if (pct == 0)   { d.accumulator = d.acc_seed; d.dc_balance = 0; d.active_now = false; return; }
    if (pct >= 100) { d.accumulator = 0; d.dc_balance = 0; d.active_now = true; return; }
    d.accumulator += pct;
    if (d.accumulator >= 100) {
        int8_t next = (int8_t)(d.dc_balance + pol);
        if (next >= -1 && next <= 1) {
            d.accumulator -= 100; d.dc_balance = next; d.active_now = true;
        } else { d.active_now = false; }          // postergado un semiciclo
    } else { d.active_now = false; }
}

static void emitHalfCycle(int p, uint32_t seq) {
    for (int i = 0; i < 6; i++) {
        if ((int)dimmers[i].phase != p) continue;
        GPIO_LOW(dimmers[i]);
        dimmers[i].fired = false;
    }
    if (control_mode == MODE_ICC_FULL) {
        if ((seq & 1u) == 0)
            for (int i = 0; i < 6; i++)
                if ((int)dimmers[i].phase == p)
                    iccStepFull(dimmers[i], dimmers[i].applied_pct);
    } else if (control_mode == MODE_ICC_BALANCED) {
        int8_t pol = ((seq & 1u) == 0) ? (int8_t)+1 : (int8_t)-1;
        for (int i = 0; i < 6; i++)
            if ((int)dimmers[i].phase == p)
                iccStepBalanced(dimmers[i], dimmers[i].applied_pct, pol);
    }
}

// ============================================================
// PLL
// ============================================================
static void pllUpdate(int p, uint64_t hw_now, uint64_t now) {
    PhaseState &ph = phases[p];
    uint64_t prev = ph.hw_prev_us;
    ph.hw_prev_us = hw_now;

    if (!ph.locked) {
        if (prev == 0) return;
        uint32_t measured = (uint32_t)(hw_now - prev);
        // Fuera de rango solo se descarta; NO reinicia el contador, porque con
        // un ZCD intermitente exigir intervalos consecutivos impediría enganchar.
        if (measured < HALF_MIN_US || measured > HALF_MAX_US) return;
        ph.half_us      = measured;
        ph.last_edge_us = hw_now;
        ph.next_edge_us = hw_now + measured;
        ph.last_ok_us   = now;
        if (++ph.lock_count >= PLL_LOCK_COUNT) ph.locked = true;
        return;
    }

    // Enganchado: el intervalo bruto ya no es criterio de validez. Un flanco
    // perdido produce un hueco de 2*half que el plegado a +-half/2 absorbe.
    // Por eso este firmware NO puede sufrir el enclavamiento permanente que
    // González Mancera identificó en v1: la referencia siempre avanza.
    int64_t h   = (int64_t)ph.half_us;
    int64_t err = (int64_t)hw_now - (int64_t)ph.last_edge_us;
    while (err >  h / 2) err -= h;
    while (err < -h / 2) err += h;
    if (err > h / PLL_OUTLIER_DIV || err < -h / PLL_OUTLIER_DIV) return;

    ph.last_ok_us   = now;
    ph.next_edge_us = (uint64_t)((int64_t)ph.next_edge_us + (err >> PLL_PHASE_SHIFT));
    int32_t nh = (int32_t)ph.half_us + (int32_t)(err >> PLL_PERIOD_SHIFT);
    if (nh >= (int32_t)HALF_MIN_US && nh <= (int32_t)HALF_MAX_US) ph.half_us = (uint32_t)nh;
}

// ============================================================
// FUENTE DE SINCRONISMO
//   offset(p desde ref) = half - delta(ref -> la tercera fase)
// ============================================================
bool setPhaseSource(uint8_t p, int8_t ref) {
    if (p > 2) return false;
    if (ref < 0) {
        phases[p].src = SRC_HW; phases[p].locked = false;
        phases[p].lock_count = 0; phases[p].hw_prev_us = 0;
        return true;
    }
    if (ref > 2 || ref == (int8_t)p) return false;
    if (!phases[ref].locked) return false;
    int other = 3 - p - ref;
    if (!phases[other].locked) return false;

    uint32_t half = phases[ref].half_us;
    if (!half) return false;
    int64_t d = (int64_t)phases[other].last_edge_us - (int64_t)phases[ref].last_edge_us;
    d %= (int64_t)half;
    if (d < 0) d += half;
    int64_t off = (int64_t)half - d;
    if (off >= (int64_t)half) off -= half;

    uint32_t permil = (uint32_t)(off * 1000 / half);
    // Cordura: debe caer cerca de 60° (1/3) o 120° (2/3) del semiciclo.
    if (!((permil > 283 && permil < 383) || (permil > 617 && permil < 717))) return false;

    phases[p].src           = ref;
    phases[p].offset_permil = (uint16_t)permil;
    phases[p].locked        = false;
    return true;
}

// ============================================================
// TÉRMICO
// Los tres estados no sanos dan 0: inválida, vencida, o nunca hubo.
// El sensor que protege no puede ser el que se calle.
// ============================================================
static bool temp_no_confiable() {
    uint32_t now = millis();
    return (!temp_valid) || (last_temp_ms == 0) ||
           (now - last_temp_ms > TEMP_STALE_MS) || isnan(resistor_temp_c);
}

// Objetivo instantaneo del recorte, sin suavizar. Es la ley que ya estaba.
// [C21] El objetivo devuelve ademas POR QUE vale 0. La rampa lenta de
// recuperacion tiene sentido para la histeresis termica, pero no para una
// lectura perdida: una trama MAX6675 mala dejaba el banco 20 s derateado sin
// que la resistencia estuviera caliente, y en el registro del equipo esos
// "temp=MALA" transitorios aparecen con regularidad.
static float thermal_target(bool &por_invalidez) {
    por_invalidez = false;
    if (thermal_trip) return 0.0f;
    if (temp_no_confiable()) { por_invalidez = true; return 0.0f; }
    float t = resistor_temp_c;
    float warn = temp_warn_c, trip = temp_trip_c;   // [C16] copias no volatiles
    if (t >= trip) { thermal_trip = true; return 0.0f; }
    if (t <= warn) return 1.0f;
    return (trip - t) / (trip - warn);
}

// [C15] El factor que se aplica de verdad: sigue al objetivo INMEDIATAMENTE
// hacia abajo y con pendiente limitada hacia arriba.
static float thermal_factor() {
    static float    applied       = 1.0f;
    static uint32_t last_ms       = 0;
    static bool     prev_invalida = false;

    uint32_t now = millis();
    bool  invalida = false;
    float target   = thermal_target(invalida);

    if (prev_invalida && !invalida) {
        applied = target;                 // [C21] volvio una lectura buena: sin rampa
    } else if (target <= applied) {
        applied = target;                 // bajar: sin freno, es la proteccion
    } else if (last_ms != 0) {
        uint32_t dt = now - last_ms;
        applied += THERMAL_RISE_PER_100MS * (float)dt / 100.0f;
        if (applied > target) applied = target;
    }

    prev_invalida = invalida;
    last_ms       = now;
    if (applied < 0.0f) applied = 0.0f;
    if (applied > 1.0f) applied = 1.0f;
    return applied;
}

// ============================================================
// LÍMITES — REVOCACIÓN vs INHIBICIÓN (modelo de González Mancera)
//
// NIVEL 1 — REVOCACIÓN (se borra power_pct): hombre muerto vencido, disparo
//   térmico enclavado. Al desaparecer la condición la potencia NO vuelve sola:
//   hace falta un comando nuevo. Si el optimizador lleva media hora muerto,
//   que la I2C se recupere no autoriza a reenergizar con un setpoint viejo.
//
// NIVEL 2 — INHIBICIÓN (power_pct se conserva, applied_pct va a 0): fase
//   muerta, maestro mudo, ventiladores sin confirmar, recorte térmico. Son
//   transitorias; exigir comando nuevo tras un hipo de 200 ms sería inoperable.
//
// Los ventiladores son INHIBICIÓN y no REVOCACIÓN a propósito: el maestro los
// enciende PORQUE hay carga pedida, así que revocar apagaría la carga, que
// apagaría los ventiladores, que impediría arrancar. Abrazo mortal evitado.
// ============================================================
static void recompute_applied() {
    uint32_t now_ms = millis();

    deadman_active = (last_actuation_ms == 0) ||
                     (now_ms - last_actuation_ms > DEADMAN_MS);
    if (deadman_active && lease_session_open && last_actuation_ms != 0)
        lease_dirty = true;                       // [C23]
    master_silent  = (last_master_ms == 0) ||
                     (now_ms - last_master_ms > MASTER_STALE_MS);
    fans_ok        = fans_on && !master_silent;

    float tf = thermal_factor();

    if (deadman_active || thermal_trip) {
        // [C17] Solo es una REVOCACION si habia algo que revocar. Sin esta
        // comprobacion un banco en reposo contaria cientos por segundo y el
        // contador no diria nada.
        bool habia_arriendo = false;
        for (int i = 0; i < 6; i++)
            if (dimmers[i].power_pct) { habia_arriendo = true; break; }
        if (habia_arriendo) {
            // [C22] el hombre muerto por revocacion explicita del maestro se
            // distingue del de un maestro mudo.
            revoc_reason = deadman_active ? (revoke_explicit ? REVOC_EXPLICITA : REVOC_DEADMAN)
                         : (thermal_trip  ? REVOC_TRIP : REVOC_PENDING);
            if (revoc_count < 255) revoc_count++;
        }
        for (int i = 0; i < 6; i++) dimmers[i].power_pct = 0;
    }

    for (int i = 0; i < 6; i++) {
        uint8_t p = dimmers[i].power_pct;
        if (p > power_cap) p = power_cap;
        p = (uint8_t)(p * tf + 0.5f);
        if (!phases[dimmers[i].phase].alive) p = 0;
        if (master_silent)                   p = 0;
        if (!fans_ok)                        p = 0;
        dimmers[i].applied_pct = p;
    }
}

// ============================================================
// I2C
// ============================================================
#if ENABLE_I2C

// [C17] Renovar el arriendo, midiendo el hueco que se acaba de cerrar.
//
// El fallo del 2026-09-10 fue un arriendo que vencio sin que nadie lo viera.
// `deadman_active` dice si esta vencido AHORA, y como no es enclavado, para
// cuando alguien mira ya se limpio. `lease_gap_max_ms` no se limpia: guarda el
// peor hueco desde el arranque, de modo que un arriendo que se pierde a rachas
// se delata mucho antes de llegar a DEADMAN_MS.
static inline void renovar_arriendo(uint32_t now) {
    // v3.4: un almuerzo no es un hueco de una sesion. La primera afirmacion
    // abre una ventana nueva y reinicia su maximo.
    if (!lease_session_open || last_actuation_ms == 0) {
        lease_session_open = true;
        if (!lease_dirty) lease_gap_max_ms = 0;       // [C23] la evidencia sobrevive
        lease_dirty = false;
    } else {
        uint32_t gap = now - last_actuation_ms;      // seguro ante el vuelco
        if (gap > lease_gap_max_ms && gap < 3600000UL) lease_gap_max_ms = gap;
    }
    revoke_explicit   = false;                        // [C22]
    last_actuation_ms = now;
}

void onI2CReceive(int bytes) {
    if (bytes < 1) return;
    uint8_t op = Wire.read();
    uint32_t now = millis();
    if (now == 0) now = 1;                    // 0 es el centinela de "nunca"

    if (op >= 1 && op <= 6) {
        if (bytes < 2) {                      // [C17] ya no en silencio
            if (frame_bad_count < 65535) frame_bad_count++;
            while (Wire.available()) Wire.read();
            return;
        }
        uint8_t val = Wire.read();
        if (val > 100) val = 100;
        // v3.4: el ultimo motivo queda persistente hasta reinicio. Un comando
        // nuevo recupera la operacion, pero no borra la evidencia del evento.
        renovar_arriendo(now);              // [C24] primero el arriendo...
        dimmers[op - 1].power_pct = val;    // ...y luego el valor
        last_master_ms    = now;
    } else if (op == CMD_SET_TEMP && bytes >= 3) {
        uint8_t hi = Wire.read();             // lecturas separadas: el orden de
        uint8_t lo = Wire.read();             // evaluación de "|" no está definido
        int16_t raw = (int16_t)(((uint16_t)hi << 8) | lo);
        if (raw == TEMP_INVALID_RAW) { temp_valid = false; resistor_temp_c = NAN; }
        else                         { resistor_temp_c = raw / 10.0f; temp_valid = true; }
        last_temp_ms   = now;
        last_master_ms = now;
    } else if (op == CMD_HEARTBEAT) {
        last_master_ms = now;                 // NO es actuación
    } else if (op == CMD_CLEAR_TRIP) {
        if (temp_valid && !isnan(resistor_temp_c) && resistor_temp_c < temp_warn_c)
            thermal_trip = false;
        last_master_ms = now;
    } else if (op == CMD_SET_CAP && bytes >= 2) {
        uint8_t c = Wire.read();
        power_cap = (c > 100) ? 100 : c;
        last_master_ms = now;
    } else if (op == CMD_SET_FANS && bytes >= 2) {
        fans_on = (Wire.read() != 0);
        last_master_ms = now;
    } else if (op == CMD_LEASE && bytes >= 2) {
        // El maestro RELEVA la frescura del mando superior. Esto es lo que
        // hacía falta en v2: sin este opcode el arriendo no tenía quién lo
        // renovara y la carga caía a los 30 s de cada comando, sin volver.
        if (cmd_lease_rx < 65535) cmd_lease_rx++;      // limite del marco v3.2
        bool afirmado = (Wire.read() != 0);
        if (afirmado) {
            renovar_arriendo(now);
        } else {
            // v3.4: revocacion explicita, no esperar 30 s. recompute_applied()
            // borrara atomicamente los seis setpoints en el siguiente ciclo.
            // [C23] el hueco que cierra la sesion (solo aritmetica).
            if (last_actuation_ms != 0) {
                uint32_t gap = now - last_actuation_ms;
                if (gap > lease_gap_max_ms && gap < 3600000UL) lease_gap_max_ms = gap;
                if (gap > DEADMAN_MS) lease_dirty = true;
            }
            revoke_explicit    = true;                 // [C22] antes de last_actuation_ms
            last_actuation_ms  = 0;
            lease_session_open = false;
        }
        last_master_ms = now;
    } else if (op == CMD_SET_TLIM && bytes >= 3) {
        // [C16] Umbrales termicos. Se RECORTAN, no se rechazan: rechazar en
        // silencio dejaria al equipo con el umbral viejo mientras el nivel
        // superior cree que lo cambio, que es la clase de fallo que este
        // firmware existe para eliminar.
        float w = (float)Wire.read();
        float t = (float)Wire.read();
        if (w > TEMP_WARN_MAX_C) w = TEMP_WARN_MAX_C;
        if (w < 40.0f)           w = 40.0f;
        if (t > TEMP_TRIP_MAX_C) t = TEMP_TRIP_MAX_C;
        if (t < w + TEMP_TRIP_MARGIN_C) t = w + TEMP_TRIP_MARGIN_C;
        if (t > TEMP_TRIP_MAX_C) { t = TEMP_TRIP_MAX_C; w = t - TEMP_TRIP_MARGIN_C; }
        temp_warn_c = w;
        temp_trip_c = t;
        last_master_ms = now;
    } else if (op == CMD_SET_MODE && bytes >= 2) {
        uint8_t m = Wire.read();
        if (m < MODE_COUNT) {
            control_mode = m;
            for (int i = 0; i < 6; i++) {
                dimmers[i].accumulator = dimmers[i].acc_seed;
                dimmers[i].dc_balance  = 0;
                dimmers[i].active_now  = false;
            }
        }
        last_master_ms = now;
    } else if (op == CMD_SET_SRC && bytes >= 2) {
        src_req_byte    = Wire.read();      // [C20] diferido a loop()
        src_req_pending = true;
        last_master_ms  = now;
    } else {
        // [C17] Opcode desconocido, o conocido pero con la trama corta. Antes
        // caia aqui y desaparecia. Un CMD_LEASE truncado es exactamente asi
        // como se pierde un arriendo sin dejar rastro.
        if (frame_bad_count < 65535) frame_bad_count++;
    }
    while (Wire.available()) Wire.read();
}

void onI2CRequest() {
    uint8_t resp[I2C_STATUS_LEN];
    for (int i = 0; i < 6; i++) resp[i] = dimmers[i].applied_pct;

    uint8_t flags = 0;
    if (thermal_trip)          flags |= 0x01;
    if (deadman_active)        flags |= 0x02;
    if (phases[PHASE_R].alive) flags |= 0x04;
    if (phases[PHASE_S].alive) flags |= 0x08;
    if (phases[PHASE_T].alive) flags |= 0x10;
    if (temp_no_confiable())   flags |= 0x20;
    if (master_silent)         flags |= 0x40;
    if (fans_ok)               flags |= 0x80;
    resp[6] = flags;

    float t = resistor_temp_c;
    resp[7] = (isnan(t) || t < 0 || t > 254) ? 255 : (uint8_t)t;

    uint32_t up = millis() / 1000;
    if (up > 65535) up = 65535;
    resp[8]  = (uint8_t)(up >> 8);
    resp[9]  = (uint8_t)(up & 0xFF);
    resp[10] = control_mode;

    for (int p = 0; p < 3; p++) {             // ventana por fase
        uint32_t w = phases[p].window_us;
        resp[11 + p] = (w > 255) ? 255 : (uint8_t)w;
        uint32_t r = phases[p].raw_rate;      // flancos crudos por segundo
        if (r > 65535) r = 65535;
        resp[14 + 2 * p]     = (uint8_t)(r >> 8);
        resp[15 + 2 * p]     = (uint8_t)(r & 0xFF);
    }

    // --- [C17] por que dejo de conducir ---------------------------------
    resp[20] = revoc_count;
    resp[21] = revoc_reason;

    // Edad del arriendo y peor hueco, en decimas de segundo.
    uint32_t age_ds = (last_actuation_ms == 0)
                    ? 65535U : (millis() - last_actuation_ms) / 100U;
    if (age_ds > 65535U) age_ds = 65535U;
    resp[22] = (uint8_t)(age_ds >> 8);
    resp[23] = (uint8_t)(age_ds & 0xFF);

    uint32_t gap_ds = lease_gap_max_ms / 100U;
    if (gap_ds > 65535U) gap_ds = 65535U;
    resp[24] = (uint8_t)(gap_ds >> 8);
    resp[25] = (uint8_t)(gap_ds & 0xFF);

    // [C16] Eco de los umbrales vigentes: el maestro comprueba que su
    // CMD_SET_TLIM tomo efecto en vez de suponerlo.
    float tw = temp_warn_c, tt = temp_trip_c;
    resp[26] = (uint8_t)(tw < 0 ? 0 : (tw > 254 ? 254 : tw));
    resp[27] = (uint8_t)(tt < 0 ? 0 : (tt > 254 ? 254 : tt));

    uint16_t bad = frame_bad_count, lrx = cmd_lease_rx;
    resp[28] = (uint8_t)(bad >> 8);  resp[29] = (uint8_t)(bad & 0xFF);
    resp[30] = (uint8_t)(lrx >> 8);  resp[31] = (uint8_t)(lrx & 0xFF);

    Wire.write(resp, I2C_STATUS_LEN);
}
#endif

// ============================================================
// CONSOLA
// ============================================================
#if DEBUG_SERIAL
void printDebug() {
    Serial.printf("--- modo %d (%s) ---\n", control_mode, MODE_NAMES[control_mode]);
    for (int p = 0; p < 3; p++) {
        Serial.printf("Fase %s: half=%luus f=%luHz viva=%d enganche=%d ventana=%luus%s",
            PHASE_NAMES[p], (unsigned long)phases[p].half_us,
            phases[p].half_us ? 1000000UL / (2 * phases[p].half_us) : 0,
            phases[p].alive, phases[p].locked, (unsigned long)phases[p].window_us,
            phases[p].window_us == 0 ? "  [SIN VENTANA: ZCD saturado]" : "");
        if (phases[p].src != SRC_HW)
            Serial.printf("  [sintetizada desde %s]", PHASE_NAMES[phases[p].src]);
        Serial.println();
    }
    for (int a = 0; a < 3; a++) {
        int b = (a + 1) % 3;
        if (!phases[a].locked || !phases[b].locked || !phases[a].half_us) continue;
        uint32_t h = phases[a].half_us;
        int64_t d = (int64_t)phases[b].last_edge_us - (int64_t)phases[a].last_edge_us;
        d %= (int64_t)h; if (d < 0) d += h;
        Serial.printf("delta %s->%s = %ldus = %ld grados\n",
                      PHASE_NAMES[a], PHASE_NAMES[b], (long)d, (long)(d * 180 / h));
    }
    for (int i = 0; i < 6; i++)
        Serial.printf("D%d pedido=%3d%% aplicado=%3d%%\n",
                      i + 1, dimmers[i].power_pct, dimmers[i].applied_pct);
    Serial.printf("trip=%d deadman=%d mudo=%d vent=%d temp=%.1fC cap=%d%%\n",
                  thermal_trip, deadman_active, master_silent, fans_ok,
                  resistor_temp_c, power_cap);
    static const char* MOTIVO[] = { "ninguna", "HOMBRE MUERTO",
                                    "DISPARO TERMICO", "peticion diferida",
                                    "REVOCACION EXPLICITA" };       // [C22]
    Serial.printf("umbrales=%.0f/%.0fC  revocaciones=%u (ultima: %s)  "
                  "arriendo: edad=%lums peor_hueco=%lums  "
                  "lease_rx=%u tramas_malas=%u\n",
                  temp_warn_c, temp_trip_c, revoc_count,
                  MOTIVO[revoc_reason < 5 ? revoc_reason : 0],
                  (unsigned long)(last_actuation_ms ? millis() - last_actuation_ms : 0),
                  (unsigned long)lease_gap_max_ms,
                  cmd_lease_rx, frame_bad_count);
}

void handleSerial() {
    String in = Serial.readStringUntil('\n');
    in.trim();
    int sp = in.indexOf(' ');
    if (sp < 1) {
        // Comandos de una sola letra, sin argumento.
        if (in == "d") {
            debug_stream = !debug_stream;             // [C18]
            Serial.printf("Volcado periodico %s. Con el encendido, loop() se "
                          "bloquea ~30 ms cada 2 s y el banco pierde algunos "
                          "semiciclos: usalo para depurar, no para operar.\n",
                          debug_stream ? "ENCENDIDO" : "apagado");
            return;
        }
        if (in == "?" || in == "p") { printDebug(); return; }
        Serial.println(F("Uso: <1-6> <0-100> | m <0-2> | src <fase> <ref|-1> | "
                         "tlim <warn> <trip> | p (estado) | d (volcado on/off)"));
        return;
    }
    String head = in.substring(0, sp);
    String tail = in.substring(sp + 1); tail.trim();

    if (head == "m") {
        int v = tail.toInt();
        if (v >= 0 && v < MODE_COUNT) { control_mode = (uint8_t)v; printDebug(); }
        return;
    }
    if (head == "tlim") {                                   // [C16]
        int s2 = tail.indexOf(' ');
        if (s2 < 1) { Serial.println(F("Uso: tlim <warn> <trip>")); return; }
        float w = tail.substring(0, s2).toFloat();
        float t = tail.substring(s2 + 1).toFloat();
        if (w > TEMP_WARN_MAX_C) w = TEMP_WARN_MAX_C;
        if (w < 40.0f)           w = 40.0f;
        if (t < w + TEMP_TRIP_MARGIN_C) t = w + TEMP_TRIP_MARGIN_C;
        if (t > TEMP_TRIP_MAX_C) { t = TEMP_TRIP_MAX_C; w = t - TEMP_TRIP_MARGIN_C; }
        temp_warn_c = w; temp_trip_c = t;
        Serial.printf("Umbrales -> %.0f / %.0f C (topes duros %.0f / %.0f). "
                      "El maestro los reescribe cada segundo.\n",
                      w, t, TEMP_WARN_MAX_C, TEMP_TRIP_MAX_C);
        return;
    }
    if (head == "src") {
        int s2 = tail.indexOf(' ');
        if (s2 < 1) return;
        if (setPhaseSource((uint8_t)tail.substring(0, s2).toInt(),
                           (int8_t)tail.substring(s2 + 1).toInt())) printDebug();
        else Serial.println(F("ERROR: hacen falta dos fases enganchadas y desfase valido"));
        return;
    }
    int n = head.toInt(), v = tail.toInt();
    if (n < 1 || n > 6 || v < 0 || v > 100) return;
    dimmers[n - 1].power_pct = (uint8_t)v;
    revoc_reason = REVOC_NINGUNA;
    renovar_arriendo(millis());
    printDebug();
}
#endif

// ============================================================
// PERRO GUARDIÁN — compatible con Arduino-ESP32 2.x y 3.x
// ============================================================
static void wdt_init_compat(uint32_t timeout_s) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t cfg = { timeout_s * 1000, 0, true };
    esp_task_wdt_reconfigure(&cfg);
#else
    esp_task_wdt_init(timeout_s, true);
#endif
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    build_linearization_table();

    for (int i = 0; i < 6; i++) {
        pinMode(dimmers[i].ch_pin, OUTPUT);
        GPIO_LOW(dimmers[i]);
    }
    // INPUT, no INPUT_PULLUP: ver la nota de hardware del encabezado.
    for (int p = 0; p < 3; p++) pinMode(phases[p].sync_pin, INPUT);

    attachInterrupt(digitalPinToInterrupt( 4), isr_phase_r, CHANGE);
    attachInterrupt(digitalPinToInterrupt(13), isr_phase_s, CHANGE);
    attachInterrupt(digitalPinToInterrupt(14), isr_phase_t, CHANGE);

#if ENABLE_I2C
    Wire.begin((uint8_t)I2C_SLAVE_ADDR);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
#endif

    wdt_init_compat(WDT_TIMEOUT_S);
    esp_task_wdt_add(NULL);

    Serial.printf("SolarIoT_Slave v3.4 listo. Umbrales de arranque %.0f/%.0f C "
                  "(topes duros %.0f/%.0f). El maestro los ajusta.\n",
                  temp_warn_c, temp_trip_c, TEMP_WARN_MAX_C, TEMP_TRIP_MAX_C);
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    esp_task_wdt_reset();

    uint64_t now    = esp_timer_get_time();
    uint32_t now_ms = (uint32_t)(now / 1000ULL);

    // --- A) Base de tiempo por fase ------------------------------------
    for (int p = 0; p < 3; p++) {
        PhaseState &ph = phases[p];

        if (ph.src == SRC_HW) {
            uint64_t hw_t; uint32_t hw_s, hw_w;
            read_hw(p, hw_t, hw_s, hw_w);
            if (hw_s != ph.hw_seq_seen) {
                ph.hw_seq_seen = hw_s;
                pllUpdate(p, hw_t, now);
                ph.last_valid_ms = now_ms;
            }
            if (ph.locked && (now - ph.last_ok_us) > PLL_TIMEOUT_US) {
                ph.locked = false; ph.lock_count = 0; ph.hw_prev_us = 0;
            }
        } else {
            PhaseState &ref = phases[ph.src];
            ph.locked  = ref.locked;
            ph.half_us = ref.half_us;
            ph.last_valid_ms = ref.last_valid_ms;
            if (ref.locked && ref.half_us) {
                uint64_t anchor = ref.last_edge_us +
                    (uint32_t)((uint64_t)ref.half_us * ph.offset_permil / 1000ULL);
                if (anchor > ph.last_edge_us) ph.next_edge_us = anchor;
            }
        }

        // Salud de fase: sin ciclos válidos, muerta y canales forzados a 0.
        bool viva = ph.locked && (now_ms - ph.last_valid_ms) <= PHASE_TIMEOUT_MS;
        if (ph.alive && !viva) {
            for (int i = 0; i < 6; i++)
                if ((int)dimmers[i].phase == p) GPIO_LOW(dimmers[i]);
        }
        ph.alive = viva;

        if (!ph.locked || ph.half_us == 0) continue;

        int guard = 0;
        while (now >= ph.next_edge_us && guard < MAX_CATCHUP) {
            ph.last_edge_us  = ph.next_edge_us;
            ph.next_edge_us += ph.half_us;
            emitHalfCycle(p, ph.seq);
            ph.seq++;
            guard++;
        }
        if (guard >= MAX_CATCHUP) {                 // el lazo se atascó
            ph.last_edge_us = now;
            ph.next_edge_us = now + ph.half_us;
        }
    }

    // Tasa de flancos crudos, una vez por segundo (diagnostico del ZCD).
    static uint32_t last_rate_ms = 0;
    if (now_ms - last_rate_ms >= 1000) {
        last_rate_ms = now_ms;
        for (int p = 0; p < 3; p++) {
            uint32_t c = phases[p].raw_edges;
            phases[p].raw_rate = c - phases[p].raw_prev;
            phases[p].raw_prev = c;
        }
    }

    // --- Límites vigentes ----------------------------------------------
    if (src_req_pending) {                  // [C20] fuera del contexto del callback
        src_req_pending = false;
        uint8_t b = src_req_byte;
        setPhaseSource((uint8_t)(b >> 4), (int8_t)((b & 0x0F) - 1));
    }

    recompute_applied();

    // --- B) Disparo ----------------------------------------------------
    for (int i = 0; i < 6; i++) {
        Dimmer &d = dimmers[i];
        if (d.fired) continue;
        if (d.applied_pct == 0) continue;

        PhaseState &ph = phases[d.phase];
        if (!ph.alive || !ph.locked || ph.half_us == 0) continue;

        uint32_t half = ph.half_us;
        uint32_t delay_us;

        if (control_mode == MODE_PHASE) {
            delay_us = (uint32_t)((uint64_t)half * alpha_permil[d.applied_pct] / 1000ULL);
            if (delay_us < MIN_FIRE_DELAY_US) delay_us = MIN_FIRE_DELAY_US;
        } else {
            if (!d.active_now) continue;
            delay_us = ICC_FIRE_DELAY_US;
        }

        if (half > TAIL_GUARD_US && delay_us >= half - TAIL_GUARD_US) continue;

        if (now >= ph.last_edge_us + (uint64_t)delay_us) {
            GPIO_HIGH(d);
            d.fire_time_us = now;
            d.fired        = true;
        }
    }

    // --- C) Fin del pulso ----------------------------------------------
    for (int i = 0; i < 6; i++)
        if (dimmers[i].fired && (now - dimmers[i].fire_time_us) >= PULSE_US)
            GPIO_LOW(dimmers[i]);

    // --- D) Consola ----------------------------------------------------
#if DEBUG_SERIAL
    if (Serial.available()) handleSerial();
    // [C18] Solo si alguien lo pidio con 'd'. Ver la nota de DEBUG_SERIAL.
    static uint64_t last_dbg = 0;
    if (debug_stream && now - last_dbg >= 2000000ULL) {
        last_dbg = now; printDebug();
    }
#endif
}