/*
  Sensor de fuerza de 2 ejes (cruz de malta, 8 galgas) - VERSION V10.1
  ADS131M02 (2000 SPS, PGA 128) + XIAO ESP32-C3

  Uso (monitor serie a 115200, fin de línea "Nueva línea"):
    1. Pon la ESP en el socket, deja el sensor SIN CARGA y envía  i
       -> inicia el ADC y comienza la calibración de cero sin bloquear el loop.
    2. Comandos:
         z  -> vuelve a tomar el cero (sin carga; proceso no bloqueante)
         p  -> pausa / reanuda la salida
         v  -> cambia entre CSV (para guardar) y texto legible con nombres y unidades
         l  -> explica qué significa cada dato
         k  -> muestra la calibración
         h  -> ayuda

  Salida CSV (OUT_HZ veces por segundo):
    t_s,Fy_N,Fx_N,F_N,ang_deg,dY_uV,dX_uV,Z_pattern,perdidas,pos_out_deg,moving
      F_N     = sqrt(Fy² + Fx²)
      ang_deg = dirección de la fuerza en el plano (0° = +Y, 90° = +X)
      dY/dX   = señal de cada puente respecto al cero, en el marco del mecanismo
      Z       = 1 si reaccionan los dos puentes a la vez (patrón de carga en Z):
                Fy/Fx de esa lectura NO son confiables.

  Calibración (pendientes medidas, uV/N, marco del mecanismo):
      carga en Y: canal Y +23.97, canal X -0.83
      carga en X: canal Y  +1.76, canal X -25.87
    [Fy; Fx] = C * [dY; dX]   con C = inversa de esa matriz (N/mV)

  Conexiones: D8=SCLK, D9=DOUT, D10=DIN, D5=CS, D0=DRDY
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#include "secrets.h"
#include "thingspeak_ca.h"

// ----------------------------- Pines ---------------------------------
#define PIN_SCLK  D8
#define PIN_MISO  D9    // DOUT del ADC
#define PIN_MOSI  D10   // DIN del ADC
#define PIN_CS    D5
#define PIN_DRDY  D0

// ----------------------- Diagnóstico de rendimiento ------------------
// 0: desactivado (recomendado para ensayos normales).
// 1: informa cada segundo tiempos de SPI, intervalo entre muestras y tasa DRDY.
#ifndef DEBUG_PERFORMANCE
#define DEBUG_PERFORMANCE 0
#endif
#define PERF_REPORT_MS    1000

// ----------------------- Parámetros del sistema -----------------------
#define SPI_HZ        4000000UL
#define FS_HZ         2000        // 4.096 MHz / 2 / OSR 1024
#define PGA_GAIN      128         // ±1.2 V / 128 = ±9.375 mV
#define ADC_VREF      1.2f
#define AMP_GAIN      1.0f
#define SAT_MV        8.5f         // aviso de saturación

// ------------- Canales del ADC -> ejes del mecanismo -------------------
// Igual que en la calibración: Y = -AIN1, X = +AIN0.
// Si al empujar en +Y el Fy sale negativo, cambia SIGN_Y (igual para X).
#define ADC_CH_Y      1
#define SIGN_Y        (-1.0f)
#define ADC_CH_X      0
#define SIGN_X        (+1.0f)

// ----------------------- Calibración (N/mV) ---------------------------
#define C_YY   41.8173f    // Fy por mV de dY
#define C_YX    2.8449f    // Fy por mV de dX
#define C_XY   -1.3416f    // Fx por mV de dY
#define C_XX  -38.7461f    // Fx por mV de dX

// ----------------------- Detección de carga en Z ----------------------
// Carga en Y o X: el otro canal responde 3-7 % (δ <= 4°).
// Carga en Z: los dos canales responden parecido (~50 %, δ mediana 15°).
#define Z_SENAL_MIN_UV  15.0f      // por debajo no se decide (ruido / carga muy baja)
#define Z_DELTA_DEG     7.0f       // δ = atan(menor/mayor) por encima de esto -> patrón Z
#define Z_CONFIRMA      3         // lecturas seguidas para activar/desactivar la marca

// ----------------------------- Salida ---------------------------------
#define OUT_HZ          10        // 2000 / 10 = 200 muestras por lectura (6 ciclos de 60 Hz)
#define OUT_AVG         (FS_HZ / OUT_HZ)
#define ZERO_SECONDS    5
#define ZERO_COUNTDOWN  3

// ----------------------- Constantes del ADS131M02 ---------------------
#define CMD_RESET     0x0011
#define CMD_RREG      0xA000
#define CMD_WREG      0x6000
#define REG_ID          0x00
#define REG_MODE        0x02
#define REG_CLOCK       0x03
#define REG_GAIN        0x04
#define REG_THRSHLD_LSB 0x08
#define CLK_CH_EN     0x0300
#define OSR_1024      (0x3 << 2)
#define PWR_HR        0x2
#define MODE_VALUE    0x0110
#define NUM_WORDS     4
#define WORD_BYTES    3
#define FRAME_BYTES   (NUM_WORDS * WORD_BYTES)

// ----------------------------- Estado ---------------------------------
volatile uint32_t drdyPending = 0;
void IRAM_ATTR onDRDY() { __atomic_fetch_add(&drdyPending, 1, __ATOMIC_RELAXED); }

float    zero_mV[2] = {0.0f, 0.0f};   // cero en el marco del mecanismo: [0]=Y, [1]=X
uint32_t lostSamples = 0;
uint32_t totalDrdy   = 0;
bool     adcReady    = false;
bool     output      = true;

#if DEBUG_PERFORMANCE
static uint32_t perfLastSampleUs = 0;
static uint32_t perfMaxSampleGapUs = 0;
static uint32_t perfMaxSpiUs = 0;
static uint32_t perfMaxReadUs = 0;
static uint32_t perfMaxLoopUs = 0;
static uint32_t perfReportStartMs = 0;
static uint32_t perfLastDrdy = 0;
static uint32_t perfLastLost = 0;
#endif

// ----------------------------- SPI de bajo nivel ----------------------
static void spiFrame(const uint8_t *tx, uint8_t *rx) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);
  SPI.transferBytes(tx, rx, FRAME_BYTES);
  delayMicroseconds(1);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

static void putWord(uint8_t *buf, uint8_t idx, uint16_t w) {
  buf[idx * WORD_BYTES + 0] = w >> 8;
  buf[idx * WORD_BYTES + 1] = w & 0xFF;
  buf[idx * WORD_BYTES + 2] = 0;
}

static uint16_t getWord16(const uint8_t *buf, uint8_t idx) {
  return ((uint16_t)buf[idx * WORD_BYTES] << 8) | buf[idx * WORD_BYTES + 1];
}

static void nullFrame(uint8_t *rx) {
  uint8_t tx[FRAME_BYTES] = {0};
  spiFrame(tx, rx);
}

static bool adcRegisterWrite(uint8_t addr, uint16_t data) {
  uint8_t tx[FRAME_BYTES] = {0}, rx[FRAME_BYTES];
  putWord(tx, 0, CMD_WREG | ((uint16_t)addr << 7));
  putWord(tx, 1, data);
  spiFrame(tx, rx);
  nullFrame(rx);
  return getWord16(rx, 0) == (0x4000 | ((uint16_t)addr << 7));
}

static uint16_t adcRegisterRead(uint8_t addr) {
  uint8_t tx[FRAME_BYTES] = {0}, rx[FRAME_BYTES];
  putWord(tx, 0, CMD_RREG | ((uint16_t)addr << 7));
  spiFrame(tx, rx);
  nullFrame(rx);
  return getWord16(rx, 0);
}

// ----------------------------- Datos ----------------------------------
static int32_t signExtend24(const uint8_t *p) {
  int32_t v = ((int32_t)p[0] << 16) | ((int32_t)p[1] << 8) | p[2];
  if (v & 0x800000) v |= 0xFF000000;
  return v;
}

// Escala constante: mV por código ADC. Se calcula una sola vez.
static constexpr float MV_PER_CODE =
    (ADC_VREF / (float)PGA_GAIN) / 8388608.0f / AMP_GAIN * 1000.0f;

static inline float codeToMV(int32_t code) {
  return (float)code * MV_PER_CODE;
}

// Devuelve una muestra ya en el marco del mecanismo: mv[0]=Y, mv[1]=X. raw = canales crudos.
static bool getSample(float mv[2], float raw[2],
                      bool countMeasurementStats,
                      uint32_t *skippedSamples = nullptr,
                      int32_t codes[2] = nullptr) {
  uint32_t p = __atomic_exchange_n(&drdyPending, 0, __ATOMIC_SEQ_CST);
  if (p == 0) return false;

  // Una lectura SPI entrega la conversión más reciente, no una cola histórica.
  // Si llegaron varios DRDY, las conversiones intermedias no se recuperan.
  if (p > 1) {
    uint32_t skipped = p - 1;
    if (skippedSamples != nullptr) *skippedSamples += skipped;
    if (countMeasurementStats) lostSamples += skipped;
  }
  if (countMeasurementStats) totalDrdy += p;

  uint8_t tx[FRAME_BYTES] = {0}, rx[FRAME_BYTES];
#if DEBUG_PERFORMANCE
  uint32_t readStartUs = micros();
  uint32_t spiStartUs = readStartUs;
#endif
  spiFrame(tx, rx);
#if DEBUG_PERFORMANCE
  uint32_t sampleEndUs = micros();
  uint32_t spiDurationUs = sampleEndUs - spiStartUs;
  if (spiDurationUs > perfMaxSpiUs) perfMaxSpiUs = spiDurationUs;

  if (perfLastSampleUs != 0) {
    uint32_t sampleGapUs = sampleEndUs - perfLastSampleUs;
    if (sampleGapUs > perfMaxSampleGapUs) perfMaxSampleGapUs = sampleGapUs;
  }
  perfLastSampleUs = sampleEndUs;
#endif

  int32_t rawCode[2];
  rawCode[0] = signExtend24(&rx[1 * WORD_BYTES]);  // AIN0
  rawCode[1] = signExtend24(&rx[2 * WORD_BYTES]);  // AIN1

  raw[0] = codeToMV(rawCode[0]);
  raw[1] = codeToMV(rawCode[1]);
  mv[0] = SIGN_Y * raw[ADC_CH_Y];
  mv[1] = SIGN_X * raw[ADC_CH_X];

  if (codes != nullptr) {
    codes[0] = (int32_t)SIGN_Y * rawCode[ADC_CH_Y]; // Y en marco mecanismo
    codes[1] = (int32_t)SIGN_X * rawCode[ADC_CH_X]; // X en marco mecanismo
  }
#if DEBUG_PERFORMANCE
  uint32_t readDurationUs = micros() - readStartUs;
  if (readDurationUs > perfMaxReadUs) perfMaxReadUs = readDurationUs;
#endif
  return true;
}

static inline void toForce(float dy_mV, float dx_mV, float &fy, float &fx) {
  fy = C_YY * dy_mV + C_YX * dx_mV;
  fx = C_XY * dy_mV + C_XX * dx_mV;
}

// ----------------------------- Inicialización -------------------------
static uint8_t pgaCode(uint8_t g) {
  switch (g) {
    case 1: return 0; case 2: return 1; case 4: return 2; case 8: return 3;
    case 16: return 4; case 32: return 5; case 64: return 6; default: return 7;
  }
}

static bool adcInit() {
  uint8_t tx[FRAME_BYTES] = {0}, rx[FRAME_BYTES];
  putWord(tx, 0, CMD_RESET);
  spiFrame(tx, rx);
  delay(20);
  nullFrame(rx);

  uint16_t id = adcRegisterRead(REG_ID);
  Serial.printf("# ID: 0x%04X (esperado 0x22xx)\n", id);
  if ((id >> 8) != 0x22) return false;

  bool ok = true;
  ok &= adcRegisterWrite(REG_MODE, MODE_VALUE);
  ok &= adcRegisterWrite(REG_THRSHLD_LSB, 0x0000);
  uint8_t g = pgaCode(PGA_GAIN);
  ok &= adcRegisterWrite(REG_GAIN, ((uint16_t)g << 4) | g);
  ok &= adcRegisterWrite(REG_CLOCK, CLK_CH_EN | OSR_1024 | PWR_HR);
  return ok;
}

static uint32_t flushAdc() {
  uint8_t rx[FRAME_BYTES];
  for (uint8_t i = 0; i < 3; i++) nullFrame(rx);

  // Los DRDY generados durante el vaciado también corresponden a
  // conversiones que no se entregarán al procesamiento.
  uint32_t discarded =
      __atomic_exchange_n(&drdyPending, 0, __ATOMIC_SEQ_CST);

  return discarded;
}

// ----------------------------- Cero no bloqueante ----------------------
static void printHeader();

enum class ZeroState : uint8_t {
  IDLE,
  COUNTDOWN,
  ACQUIRING
};

static ZeroState zeroState = ZeroState::IDLE;
static bool zeroWasReady = false;
static float previousZero[2] = {0.0f, 0.0f};
static uint32_t zeroLastDrdyMs = 0;
static uint32_t zeroNextCountdownMs = 0;
static uint8_t zeroCountdownValue = 0;
static uint32_t zeroTargetSamples = 0;
static uint32_t zeroValidSamples = 0;
static uint32_t zeroLostSamples = 0;
static uint32_t zeroDiscardedBefore = 0;
static int64_t zeroSumCode[2] = {0, 0};

static bool zeroIsActive() {
  return zeroState != ZeroState::IDLE;
}

// Inicia la calibración sin bloquear loop(). Durante la cuenta regresiva
// el sistema sigue atendiendo el puerto serie; al adquirir el cero, cada
// vuelta de loop() procesa como máximo una muestra disponible.
static void beginZero() {
  if (zeroIsActive()) {
    Serial.println("# Ya hay una calibracion de cero en curso.");
    return;
  }

  zeroWasReady = adcReady;
  previousZero[0] = zero_mV[0];
  previousZero[1] = zero_mV[1];

  zeroTargetSamples = (uint32_t)FS_HZ * ZERO_SECONDS;
  zeroValidSamples = 0;
  zeroLostSamples = 0;
  zeroSumCode[0] = 0;
  zeroSumCode[1] = 0;

  zeroCountdownValue = ZERO_COUNTDOWN;
  zeroNextCountdownMs = millis();
  zeroState = ZeroState::COUNTDOWN;

  Serial.println("# CALIBRACION DE CERO: deja el sensor SIN CARGA y sin tocar.");
}

static void finishZero(bool success) {
  if (success) {
    zero_mV[0] = ((float)zeroSumCode[0] / (float)zeroValidSamples) * MV_PER_CODE;
    zero_mV[1] = ((float)zeroSumCode[1] / (float)zeroValidSamples) * MV_PER_CODE;

    Serial.printf("# CERO OK: Y0=%.5f mV, X0=%.5f mV\n",
                  zero_mV[0], zero_mV[1]);
    Serial.printf("# Muestras validas=%lu, conversiones no leidas=%lu, descartadas al preparar=%lu\n",
                  (unsigned long)zeroValidSamples,
                  (unsigned long)zeroLostSamples,
                  (unsigned long)zeroDiscardedBefore);

    // La base de tiempo de medición comienza/reinicia al terminar el cero.
    totalDrdy = 0;
    lostSamples = 0;
#if DEBUG_PERFORMANCE
    perfLastSampleUs = 0;
    perfMaxSampleGapUs = 0;
    perfMaxSpiUs = 0;
    perfMaxReadUs = 0;
    perfMaxLoopUs = 0;
    perfReportStartMs = 0;
    perfLastDrdy = 0;
    perfLastLost = 0;
#endif
  } else {
    zero_mV[0] = previousZero[0];
    zero_mV[1] = previousZero[1];
    Serial.println("# ERROR en calibracion: no se recibieron muestras DRDY durante 1 s.");
    Serial.println("# Se conserva el cero anterior.");
  }

  zeroState = ZeroState::IDLE;
  adcReady = zeroWasReady || success;
  if (adcReady) {
    // Evita que una ventana parcial anterior a la calibración contamine
    // la primera medición posterior.
    printHeader();
  }
}

static void serviceZero() {
  const uint32_t now = millis();

  if (zeroState == ZeroState::COUNTDOWN) {
    if ((int32_t)(now - zeroNextCountdownMs) >= 0) {
      if (zeroCountdownValue > 0) {
        Serial.printf("# Inicio de cero en %u s...\n", zeroCountdownValue);
        zeroCountdownValue--;
        zeroNextCountdownMs = now + 1000;
      } else {
        zeroDiscardedBefore = flushAdc();
        zeroLastDrdyMs = now;
        zeroState = ZeroState::ACQUIRING;
        Serial.printf("# Adquiriendo cero: %lu muestras...\n",
                      (unsigned long)zeroTargetSamples);
      }
    }
    return;
  }

  if (zeroState == ZeroState::ACQUIRING) {
    float mv[2], raw[2];
    int32_t codes[2];

    if (getSample(mv, raw, false, &zeroLostSamples, codes)) {
      zeroSumCode[0] += codes[0];
      zeroSumCode[1] += codes[1];
      zeroValidSamples++;
      zeroLastDrdyMs = now;

      if (zeroValidSamples >= zeroTargetSamples) {
        finishZero(true);
      }
    } else if (now - zeroLastDrdyMs > 1000) {
      finishZero(false);
    }
  }
}

// ----------------------------- Comandos -------------------------------
bool legible = false;   // false = CSV (para guardar/graficar) | true = texto con nombres y unidades

static void printHeader() {
  if (!legible) Serial.println("t_s,Fy_N,Fx_N,F_N,ang_deg,dY_uV,dX_uV,Z_pattern,perdidas,pos_out_deg,moving");
}

// Explica qué significa cada dato de la salida. Se imprime al iniciar y con el comando 'l'.
static void printLegend() {
  Serial.println("# ================== QUÉ SIGNIFICA CADA DATO ==================");
  Serial.println("# t_s      Tiempo desde que se inició la medición [s].");
  Serial.println("# Fy_N     Fuerza en el eje Y del sensor [N]. Positiva hacia +Y, negativa hacia -Y.");
  Serial.println("# Fx_N     Fuerza en el eje X del sensor [N]. Positiva hacia +X, negativa hacia -X.");
  Serial.println("# F_N      Magnitud de la fuerza en el plano X-Y [N] = raiz(Fy^2 + Fx^2). Siempre positiva.");
  Serial.println("# ang_deg  Hacia dónde apunta la fuerza [grados]: 0 = +Y, 90 = +X, 180 = -Y, -90 = -X.");
  Serial.println("#          Si F_N es casi 0, este ángulo no significa nada.");
  Serial.println("# dY_uV    Señal del puente Y respecto al cero [microvoltios]. Es el dato crudo antes de");
  Serial.println("#          convertir a fuerza (~24 uV por cada N en Y).");
  Serial.println("# dX_uV    Señal del puente X respecto al cero [microvoltios] (~26 uV por cada N en X).");
  Serial.println("# Z_pattern 0 = patron Z no detectado; 1 = magnitudes similares y signos opuestos.");
  Serial.println("#          Es un indicador de patron compatible con carga perpendicular; NO mide Fz.");
  Serial.println("#          Ante Z_pattern=1, no interpretar Fy/Fx como fuerzas tangenciales confiables.");
  Serial.println("# pos_out_deg Posicion de salida estimada del mecanismo [grados], no medida por encoder.");
  Serial.println("# moving      0 = detenido; 1 = movimiento en curso.");
  Serial.println("# perdidas Muestras de medición no leídas desde el último cero. Debe quedarse en 0;");
  Serial.println("#          si sube, el loop no alcanzó a leer todas las conversiones.");
  Serial.println("# Líneas que empiezan con '#': mensajes, no son datos.");
  Serial.println("# =============================================================");
}

static void printHelp() {
  Serial.println("# Movimiento: a (posicion A) | b (posicion B) | s (STOP)");
  Serial.println("# Sensado: i (iniciar ADC) | z (cero SIN CARGA) | p (pausa) | v (CSV/texto)");
  Serial.println("#           l (leyenda) | k (calibracion) | h (ayuda)");
  Serial.println("# MQTT remoto: field1=0 STOP, 1 A, 2 B; field2=secuencia; field3=expiracion Unix UTC.");
#if DEBUG_PERFORMANCE
  Serial.println("# Diagnostico de rendimiento activo (DEBUG_PERFORMANCE=1).");
#endif
}

static void printCal() {
  Serial.printf("# C = [[%.4f, %.4f], [%.4f, %.4f]] N/mV | Y = %+.0f*AIN%d, X = %+.0f*AIN%d\n",
                C_YY, C_YX, C_XY, C_XX, SIGN_Y, ADC_CH_Y, SIGN_X, ADC_CH_X);
  Serial.printf("# Z: senal > %.0f uV, delta > %.0f deg, signos opuestos, %d lecturas | salida %d Hz\n",
                Z_SENAL_MIN_UV, Z_DELTA_DEG, Z_CONFIRMA, OUT_HZ);
}

static void startAdc() {
  static bool irqAttached = false;
  adcReady = false;
  if (irqAttached) { detachInterrupt(digitalPinToInterrupt(PIN_DRDY)); irqAttached = false; }
  if (!adcInit()) {
    Serial.println("# El ADC no respondió. Revisa que la ESP esté bien asentada y envía 'i' de nuevo.");
    return;
  }
  Serial.println("# ADC OK");
  flushAdc();
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), onDRDY, FALLING);
  irqAttached = true;
  adcReady = false;
  beginZero();
  printCal();
  printLegend();
  printHelp();
}

// ============================================================
// 1. CONFIGURACION CENTRALIZADA
// ============================================================

struct MotionConfig {
  int servoPin;
  int pulseStop;
  int pulseForward;
  int pulseReverse;

  float forwardSpeed;       // Grados equivalentes del servo / segundo
  float reverseSpeed;       // Grados equivalentes del servo / segundo

  float transmissionRatio;  // Entrada / salida
  float minPosition;        // Limite de entrada
  float maxPosition;        // Limite de entrada

  float forwardTravel;      // Recorrido equivalente calibrado hacia adelante
  float reverseTravel;      // Recorrido equivalente calibrado hacia atras

  float positionA;          // Entrada correspondiente a 0 grados de salida
  float positionB;          // Entrada correspondiente a 30 grados de salida

  float positionTolerance;
  int servoMinPulse;
  int servoMaxPulse;
};

// Parametros validados experimentalmente.
const MotionConfig MOTION_CONFIG = {
  D1,       // servoPin
  1500,     // pulseStop
  1589,     // pulseForward
  1411,     // pulseReverse
  90.0,     // forwardSpeed
  93.8,     // reverseSpeed
  7.0,      // transmissionRatio
  0.0,      // minPosition
  210.0,    // maxPosition
  305.0,    // forwardTravel
  250.0,    // reverseTravel
  0.0,      // positionA
  210.0,    // positionB
  0.5,      // positionTolerance
  500,      // servoMinPulse
  2500      // servoMaxPulse
};

// ============================================================
// 2. TIPOS Y ESTADOS DEL CONTROLADOR
// ============================================================

enum class MotionState : uint8_t {
  IDLE,
  MOVING
};

enum class MotionDirection : int8_t {
  REVERSE = -1,
  NONE = 0,
  FORWARD = 1
};

enum class Target : uint8_t {
  POSITION_A,
  POSITION_B
};

// ============================================================
// 3. CONTROLADOR DE MOVIMIENTO
// ============================================================

class MotionController {
public:
  explicit MotionController(const MotionConfig& config)
    : cfg(config) {}

  void begin() {
    servo.setPeriodHertz(50);
    servo.attach(cfg.servoPin, cfg.servoMinPulse, cfg.servoMaxPulse);
    stopServo();
    state = MotionState::IDLE;

    // El mecanismo debe iniciar fisicamente en A.
    currentPosition = cfg.positionA;
  }

  bool moveTo(Target target) {
    if (state != MotionState::IDLE) {
      return false;
    }

    const float requestedPosition =
      (target == Target::POSITION_A) ? cfg.positionA : cfg.positionB;

    // Saturacion automatica dentro del rango mecanico.
    const float destination = clampPosition(requestedPosition);
    const float delta = destination - currentPosition;

    if (fabsf(delta) < cfg.positionTolerance) {
      Serial.println("Posicion solicitada ya alcanzada.");
      printPosition();
      return true;
    }

    direction = (delta > 0.0f)
      ? MotionDirection::FORWARD
      : MotionDirection::REVERSE;

    const bool movingForward = (direction == MotionDirection::FORWARD);
    const float calibratedTravel = movingForward
      ? cfg.forwardTravel : cfg.reverseTravel;
    const float servoSpeed = movingForward
      ? cfg.forwardSpeed : cfg.reverseSpeed;
    const float fullRange = cfg.maxPosition - cfg.minPosition;

    if (fullRange <= 0.0f || calibratedTravel <= 0.0f || servoSpeed <= 0.0f) {
      Serial.println("ERROR: parametros de movimiento invalidos.");
      return false;
    }

    const float conversionFactor = calibratedTravel / fullRange;
    const float requiredServoTravel = fabsf(delta) * conversionFactor;
    const float durationSeconds = requiredServoTravel / servoSpeed;

    startPosition = currentPosition;
    targetPosition = destination;
    inputSpeed = servoSpeed / conversionFactor;
    durationMs = (unsigned long)(durationSeconds * 1000.0f);
    startTimeMs = millis();

    // Mostrar datos antes de activar el motor para evitar demoras seriales
    // mientras el mecanismo ya esta en movimiento.
    Serial.println();
    Serial.println("=== MOVIMIENTO INICIADO ===");
    Serial.print("Destino: ");
    Serial.println(target == Target::POSITION_A ? "A (0 grados de salida)" :
                                                   "B (30 grados de salida)");
    Serial.print("Posicion inicial estimada: ");
    Serial.print(startPosition, 2);
    Serial.println(" grados de entrada");
    Serial.print("Objetivo: ");
    Serial.print(targetPosition, 2);
    Serial.println(" grados de entrada");
    Serial.print("Recorrido: ");
    Serial.print(fabsf(delta), 2);
    Serial.println(" grados de entrada");
    Serial.print("Tiempo estimado: ");
    Serial.print(durationMs);
    Serial.println(" ms");
    Serial.println("Envia 's' para detener.");

    servo.writeMicroseconds(movingForward ? cfg.pulseForward : cfg.pulseReverse);
    state = MotionState::MOVING;
    return true;
  }

  void update() {
    if (state != MotionState::MOVING) {
      return;
    }

    const unsigned long elapsedMs = millis() - startTimeMs;
    if (elapsedMs >= durationMs) {
      stopServo();
      currentPosition = targetPosition;
      state = MotionState::IDLE;
      direction = MotionDirection::NONE;

      Serial.println();
      Serial.println("=== MOVIMIENTO COMPLETADO ===");
      printPosition();
      Serial.println("-----------------------------");
    }
  }

  void stop() {
    if (state != MotionState::MOVING) {
      stopServo();
      Serial.println("Motor detenido. No habia movimiento activo.");
      return;
    }

    // Capturar el instante y detener antes de imprimir mensajes.
    const unsigned long stopTimeMs = millis();
    stopServo();
    const unsigned long elapsedMs = stopTimeMs - startTimeMs;

    if (elapsedMs >= durationMs) {
      currentPosition = targetPosition;
    } else {
      const float elapsedSeconds = elapsedMs / 1000.0f;
      const float estimatedTravel = elapsedSeconds * inputSpeed;
      currentPosition = startPosition +
        (static_cast<int8_t>(direction) * estimatedTravel);
      currentPosition = clampPosition(currentPosition);
    }

    state = MotionState::IDLE;
    direction = MotionDirection::NONE;

    Serial.println();
    Serial.println("=== MOVIMIENTO INTERRUMPIDO ===");
    Serial.print("Tiempo transcurrido: ");
    Serial.print(elapsedMs);
    Serial.println(" ms");
    printPosition();
    Serial.println("Sistema disponible para A o B.");
    Serial.println("-----------------------------");
  }

  bool isMoving() const {
    return state == MotionState::MOVING;
  }

  float getInputPosition() const {
    return currentPosition;
  }

  float getOutputPosition() const {
    return currentPosition / cfg.transmissionRatio;
  }

private:
  const MotionConfig& cfg;
  Servo servo;

  MotionState state = MotionState::IDLE;
  MotionDirection direction = MotionDirection::NONE;

  float currentPosition = 0.0f;
  float startPosition = 0.0f;
  float targetPosition = 0.0f;
  float inputSpeed = 0.0f;

  unsigned long startTimeMs = 0;
  unsigned long durationMs = 0;

  float clampPosition(float position) const {
    return constrain(position, cfg.minPosition, cfg.maxPosition);
  }

  void stopServo() {
    servo.writeMicroseconds(cfg.pulseStop);
  }

  void printPosition() const {
    Serial.print("Posicion estimada de entrada: ");
    Serial.print(currentPosition, 2);
    Serial.println(" grados");
    Serial.print("Posicion estimada de salida: ");
    Serial.print(getOutputPosition(), 2);
    Serial.println(" grados");
  }
};

// Instancia global del controlador de movimiento.
// Debe declararse despues de la clase MotionController y antes de usarla.
MotionController motion(MOTION_CONFIG);

static void handleLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  size_t n = strlen(line);
  while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\t')) line[--n] = '\0';
  for (size_t i = 0; line[i] != '\0'; ++i) {
    if (line[i] >= 'A' && line[i] <= 'Z') line[i] = line[i] - 'A' + 'a';
  }
  if (line[0] == '\0') return;

  // Los comandos de movimiento siempre se atienden, incluso durante el cero.
  if (strcmp(line, "a") == 0) {
    if (zeroIsActive()) { Serial.println("# Movimiento A bloqueado durante calibracion de cero."); return; }
    if (!motion.moveTo(Target::POSITION_A)) Serial.println("# Comando A no aceptado: hay un movimiento activo.");
    return;
  }
  if (strcmp(line, "b") == 0) {
    if (zeroIsActive()) { Serial.println("# Movimiento B bloqueado durante calibracion de cero."); return; }
    if (!motion.moveTo(Target::POSITION_B)) Serial.println("# Comando B no aceptado: hay un movimiento activo.");
    return;
  }
  if (strcmp(line, "s") == 0) {
    motion.stop();
    return;
  }

  char c = line[0];
  if (c == 'h') { printHelp(); return; }
  if (zeroIsActive()) { Serial.println("# Calibracion en curso; espera a que termine. A, B y s siguen disponibles."); return; }
  if (c == 'i') { startAdc(); return; }
  if (!adcReady) { Serial.println("# Primero envia 'i' (sensor sin carga). A, B y s controlan el movimiento."); return; }
  if (c == 'z') { beginZero(); }
  else if (c == 'p') { output = !output; Serial.printf("# Salida %s\n", output ? "ON" : "PAUSADA"); if (output) printHeader(); }
  else if (c == 'k') printCal();
  else if (c == 'l') printLegend();
  else if (c == 'v') {
    legible = !legible;
    Serial.printf("# Formato: %s\n", legible ? "texto legible" : "CSV");
    printHeader();
  }
  else printHelp();
}

// ============================================================
// V10.1 - COMUNICACION WIFI / MQTT THINGSPEAK
// ============================================================
#define THINGSPEAK_MQTT_HOST       "mqtt3.thingspeak.com"
#define THINGSPEAK_MQTT_PORT       8883
#define THINGSPEAK_TELEMETRY_ID    3572966UL
#define THINGSPEAK_COMMAND_ID      3572972UL
#define TELEMETRY_INTERVAL_MS      15000UL
#define WIFI_RETRY_INTERVAL_MS     15000UL
#define MQTT_RETRY_INTERVAL_MS     7000UL
#define COMMAND_MAX_FUTURE_S       120UL

static const char TOPIC_TELEMETRY[] = "channels/3572966/publish";
static const char TOPIC_COMMANDS[]  = "channels/3572972/subscribe";

static WiFiClientSecure secureClient;
static PubSubClient mqttClient(secureClient);

struct TelemetrySnapshot {
  float fy;
  float fx;
  float magnitude;
  float angle;
  float outputPosition;
  uint32_t lost;
  uint8_t moving;
  uint8_t zPattern;
  bool valid;
};

static TelemetrySnapshot telemetrySnapshot = {};
static char commandStatus[48] = "BOOT";
static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

// El callback MQTT no acciona directamente el servo. Solo deposita la orden;
// el loop principal la ejecuta para mantener un único propietario del movimiento.
static volatile bool remoteCommandPending = false;
static volatile uint8_t remoteCommandCode = 255;
static volatile uint32_t remoteCommandSequence = 0;
static portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t lastReceivedCommandSequence = 0;

static void setCommandStatus(const char *status) {
  portENTER_CRITICAL(&dataMux);
  snprintf(commandStatus, sizeof(commandStatus), "%s", status);
  portEXIT_CRITICAL(&dataMux);
}

static void updateTelemetrySnapshot(float fy, float fx, float magnitude,
                                    float angle, uint8_t zPattern) {
  portENTER_CRITICAL(&dataMux);
  telemetrySnapshot.fy = fy;
  telemetrySnapshot.fx = fx;
  telemetrySnapshot.magnitude = magnitude;
  telemetrySnapshot.angle = angle;
  telemetrySnapshot.outputPosition = motion.getOutputPosition();
  telemetrySnapshot.lost = lostSamples;
  telemetrySnapshot.moving = motion.isMoving() ? 1 : 0;
  telemetrySnapshot.zPattern = zPattern;
  telemetrySnapshot.valid = true;
  portEXIT_CRITICAL(&dataMux);
}

static bool jsonReadUnsigned(const char *json, const char *key, uint32_t &value) {
  char token[24];
  snprintf(token, sizeof(token), "\"%s\"", key);
  const char *p = strstr(json, token);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  ++p;
  while (*p == ' ' || *p == '\t') ++p;
  if (*p == '"') ++p;
  if (*p < '0' || *p > '9') return false;
  char *end = nullptr;
  unsigned long parsed = strtoul(p, &end, 10);
  if (end == p) return false;
  value = (uint32_t)parsed;
  return true;
}

static void mqttCallback(char *topic, byte *payload, unsigned int length) {
  (void)topic;
  // Mensaje de ThingSpeak: JSON con field1=orden, field2=secuencia,
  // field3=fecha de expiracion Unix (UTC).
  char message[384];
  if (length == 0 || length >= sizeof(message)) return;
  memcpy(message, payload, length);
  message[length] = '\0';

  uint32_t command = 255, sequence = 0, expiresAt = 0;
  if (!jsonReadUnsigned(message, "field1", command) ||
      !jsonReadUnsigned(message, "field2", sequence) ||
      !jsonReadUnsigned(message, "field3", expiresAt)) {
    setCommandStatus("CMD_BAD_FORMAT");
    return;
  }
  if (command > 2 || sequence == 0) {
    setCommandStatus("CMD_INVALID");
    return;
  }

  time_t now = time(nullptr);
  // No ejecutar órdenes si el reloj aún no está sincronizado por NTP.
  if (now < 1700000000) {
    setCommandStatus("CMD_REJECT_NO_TIME");
    return;
  }
  if (expiresAt <= (uint32_t)now ||
      (expiresAt - (uint32_t)now) > COMMAND_MAX_FUTURE_S) {
    setCommandStatus("CMD_EXPIRED");
    return;
  }
  if (lastReceivedCommandSequence != 0 &&
      (int32_t)(sequence - lastReceivedCommandSequence) <= 0) {
    setCommandStatus("CMD_DUPLICATE");
    return;
  }

  lastReceivedCommandSequence = sequence;
  portENTER_CRITICAL(&commandMux);
  remoteCommandCode = (uint8_t)command;
  remoteCommandSequence = sequence;
  remoteCommandPending = true;
  portEXIT_CRITICAL(&commandMux);
}

static void processRemoteCommand() {
  bool pending = false;
  uint8_t command = 255;
  uint32_t sequence = 0;

  portENTER_CRITICAL(&commandMux);
  if (remoteCommandPending) {
    pending = true;
    command = remoteCommandCode;
    sequence = remoteCommandSequence;
    remoteCommandPending = false;
  }
  portEXIT_CRITICAL(&commandMux);
  if (!pending) return;

  char status[48];
  if (command == 0) {
    motion.stop();
    snprintf(status, sizeof(status), "CMD_STOP_%lu", (unsigned long)sequence);
    setCommandStatus(status);
    return;
  }
  if (zeroIsActive()) {
    snprintf(status, sizeof(status), "CMD_ZERO_BUSY_%lu", (unsigned long)sequence);
    setCommandStatus(status);
    return;
  }
  if (motion.isMoving()) {
    snprintf(status, sizeof(status), "CMD_MOTION_BUSY_%lu", (unsigned long)sequence);
    setCommandStatus(status);
    return;
  }

  bool accepted = motion.moveTo(command == 1 ? Target::POSITION_A : Target::POSITION_B);
  snprintf(status, sizeof(status), "%s_%lu",
           accepted ? "CMD_ACCEPTED" : "CMD_REJECTED", (unsigned long)sequence);
  setCommandStatus(status);
}

static void publishTelemetry() {
  TelemetrySnapshot snapshot;
  char status[48];
  portENTER_CRITICAL(&dataMux);
  snapshot = telemetrySnapshot;
  snprintf(status, sizeof(status), "%s", commandStatus);
  portEXIT_CRITICAL(&dataMux);
  if (!snapshot.valid) return; // Espera a que el ADC termine su calibracion de cero.

  char payload[256];
  int n = snprintf(payload, sizeof(payload),
    "field1=%.3f&field2=%.3f&field3=%.3f&field4=%.2f&field5=%.2f&field6=%u&field7=%u&field8=%lu&status=%s",
    snapshot.fy, snapshot.fx, snapshot.magnitude, snapshot.angle,
    snapshot.outputPosition, snapshot.moving, snapshot.zPattern,
    (unsigned long)snapshot.lost, status);
  if (n <= 0 || n >= (int)sizeof(payload)) return;

  bool ok = mqttClient.publish(TOPIC_TELEMETRY, payload, false);
  if (!ok) Serial.println("# MQTT: fallo al publicar telemetria.");
  else Serial.println("# MQTT: telemetria publicada en ThingSpeak.");
}

static void mqttTask(void *parameter) {
  (void)parameter;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t lastWifiAttempt = millis();
  uint32_t lastMqttAttempt = 0;
  uint32_t lastTelemetry = millis() - TELEMETRY_INTERVAL_MS;
  bool ntpStarted = false;

  for (;;) {
    uint32_t nowMs = millis();

    if (WiFi.status() != WL_CONNECTED) {
      if (mqttClient.connected()) mqttClient.disconnect();
      if (nowMs - lastWifiAttempt >= WIFI_RETRY_INTERVAL_MS) {
        lastWifiAttempt = nowMs;
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      }
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    if (!ntpStarted) {
      configTime(0, 0, "pool.ntp.org", "time.google.com");
      ntpStarted = true;
    }

    // La validacion TLS comprueba la vigencia del certificado; espera a que NTP
    // establezca una fecha UTC razonable antes de iniciar el handshake.
    if (time(nullptr) < 1700000000) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (!mqttClient.connected()) {
      if (nowMs - lastMqttAttempt >= MQTT_RETRY_INTERVAL_MS) {
        lastMqttAttempt = nowMs;
        Serial.println("# MQTT: intentando conexion TLS...");
        bool connected = mqttClient.connect(MQTT_DEVICE_CLIENT_ID,
                                            MQTT_DEVICE_USERNAME,
                                            MQTT_DEVICE_PASSWORD);
        if (connected) {
          bool subscribed = mqttClient.subscribe(TOPIC_COMMANDS, 0);
          if (subscribed) {
            Serial.println("# MQTT: conectado y suscrito al canal de comandos.");
            setCommandStatus("MQTT_ONLINE");
            lastTelemetry = millis() - TELEMETRY_INTERVAL_MS;
          } else {
            Serial.println("# MQTT: conexion establecida, pero fallo la suscripcion.");
            mqttClient.disconnect();
          }
        } else {
          Serial.printf("# MQTT: conexion fallida, estado=%d\n", mqttClient.state());
        }
      }
    } else {
      mqttClient.loop();
      if (millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
        lastTelemetry = millis();
        publishTelemetry();
      }
    }

    // Tarea de prioridad baja: deja tiempo de CPU al loop de adquisicion.
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ----------------------------- Setup / loop ---------------------------
static char serialLine[40];
static uint8_t serialLength = 0;
static bool serialLineOverflow = false;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}

  // Inicializar movimiento y adquisicion.
  motion.begin();
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DRDY, INPUT_PULLUP);
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);

  Serial.println();
  Serial.println("===============================================");
  Serial.println(" SISTEMA INTEGRADO V10.1 - FIJACION MAGNETICA");
  Serial.println("===============================================");
  Serial.println("Movimiento: A / B / s (STOP)");
  Serial.println("Sensado: i (iniciar) / z (cero) / p (pausar salida)");
  Serial.println("         v (CSV/texto) / k (calibracion) / l (leyenda) / h (ayuda)");
  Serial.println("Inicio de posicion asumido: A. Verifica posicion fisica antes de energizar.");
  Serial.println("Envie 'i' con el sensor sin carga para iniciar el ADC y calibrar cero.");
  Serial.println("V10.1: Wi-Fi + MQTT TLS ThingSpeak (telemetria 15 s).");

  secureClient.setCACert(THINGSPEAK_ROOT_CA);
  secureClient.setTimeout(2000);
  mqttClient.setServer(THINGSPEAK_MQTT_HOST, THINGSPEAK_MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(2);

  if (strlen(WIFI_SSID) == 0 || strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0 ||
      strlen(WIFI_PASSWORD) == 0 || strcmp(MQTT_DEVICE_CLIENT_ID, "YOUR_MQTT_CLIENT_ID") == 0 ||
      strcmp(MQTT_DEVICE_USERNAME, "YOUR_MQTT_USERNAME") == 0 ||
      strcmp(MQTT_DEVICE_PASSWORD, "YOUR_MQTT_PASSWORD") == 0) {
    Serial.println("# MQTT deshabilitado: completa las credenciales en secrets.h.");
  } else {
    BaseType_t taskCreated = xTaskCreate(mqttTask, "mqttTask", 12288, nullptr, 1, nullptr);
    if (taskCreated == pdPASS) Serial.println("# Tarea de comunicacion Wi-Fi/MQTT iniciada.");
    else Serial.println("# ERROR: no se pudo crear mqttTask.");
  }
}

void loop() {
#if DEBUG_PERFORMANCE
  uint32_t loopStartUs = micros();
#endif
  static float acc[2] = {0.0f, 0.0f};
  static float rawMax = 0.0f;
  static uint16_t cnt = 0;
  static uint8_t zCount = 0;
  static bool zFlag = false;

  float mv[2], raw[2];

  // Actualizacion del servo en cada iteracion: nunca bloquea la adquisicion.
  motion.update();
  processRemoteCommand();

  // Servicio de calibracion no bloqueante del ADC.
  if (zeroIsActive()) {
    serviceZero();
  } else if (adcReady && getSample(mv, raw, true)) {
    acc[0] += mv[0];
    acc[1] += mv[1];
    rawMax = fmaxf(rawMax, fmaxf(fabsf(raw[0]), fabsf(raw[1])));

    if (++cnt >= OUT_AVG) {
      float dy = acc[0] / cnt - zero_mV[0]; // mV
      float dx = acc[1] / cnt - zero_mV[1]; // mV
      bool sat = rawMax > SAT_MV;
      acc[0] = acc[1] = 0.0f;
      cnt = 0;
      rawMax = 0.0f;

      // Detector de patron Z: magnitudes similares + signos opuestos.
      // Reconoce compresion/traccion si el patron conserva la oposicion de signos.
      float ay = fabsf(dy) * 1000.0f;
      float ax = fabsf(dx) * 1000.0f;
      float menor = fminf(ay, ax);
      float mayor = fmaxf(ay, ax);
      constexpr float TAN_Z_DELTA = 0.12278456f; // tan(7 grados)
      constexpr float Z_MIN_SQ = Z_SENAL_MIN_UV * Z_SENAL_MIN_UV;
      bool oppositeSigns = (dy * dx) < 0.0f;
      bool zNow = ((ay * ay + ax * ax) > Z_MIN_SQ) &&
                  (mayor > 0.0f) &&
                  (menor > TAN_Z_DELTA * mayor) &&
                  oppositeSigns;

      if (zNow != zFlag) {
        if (++zCount >= Z_CONFIRMA) {
          zFlag = zNow;
          zCount = 0;
        }
      } else {
        zCount = 0;
      }

      float fy, fx;
      toForce(dy, dx, fy, fx);
      float f = hypotf(fy, fx);
      float ang = atan2f(fx, fy) * 57.29578f;
      float t_s = (float)totalDrdy / (float)FS_HZ;
      updateTelemetrySnapshot(fy, fx, f, ang, zFlag ? 1 : 0);

      if (output) {
        if (sat) Serial.println("# ADVERTENCIA: cerca de saturacion del PGA (senal > 8.5 mV).");

        if (legible) {
          Serial.printf("t=%7.2f s | Fy=%+8.3f N | Fx=%+8.3f N | |F|=%7.3f N | dir=%+6.1f deg | dY=%+7.1f uV | dX=%+7.1f uV | Z_patron=%s | perdidas=%lu | pos=%6.2f deg | mov=%s\n",
                        t_s, fy, fx, f, ang, dy * 1000.0f, dx * 1000.0f,
                        zFlag ? "SI" : "NO", (unsigned long)lostSamples,
                        motion.getOutputPosition(), motion.isMoving() ? "MOVING" : "IDLE");
        } else {
          // CSV V10: incluye posicion estimada y estado de movimiento.
          Serial.printf("%.2f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%d,%lu,%.2f,%d\n",
                        t_s, fy, fx, f, ang,
                        dy * 1000.0f, dx * 1000.0f,
                        zFlag ? 1 : 0, (unsigned long)lostSamples,
                        motion.getOutputPosition(), motion.isMoving() ? 1 : 0);
        }
      }
    }
  }

  // Parser serial unico para movimiento y sensado; limita carga por iteracion.
  uint8_t bytesProcessed = 0;
  while (Serial.available() && bytesProcessed < 16) {
    bytesProcessed++;
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (!serialLineOverflow && serialLength > 0) {
        serialLine[serialLength] = '\0';
        handleLine(serialLine);
        // Descarta la ventana parcial al cambiar de modo/cero.
        acc[0] = acc[1] = 0.0f;
        cnt = 0;
        rawMax = 0.0f;
      } else if (serialLineOverflow) {
        Serial.println("# Comando descartado: linea demasiado larga.");
      }
      serialLength = 0;
      serialLineOverflow = false;
    } else if (!serialLineOverflow) {
      if (serialLength < sizeof(serialLine) - 1) {
        serialLine[serialLength++] = c;
      } else {
        serialLineOverflow = true;
      }
    }
  }

#if DEBUG_PERFORMANCE
  uint32_t nowMs = millis();
  if (perfReportStartMs == 0) {
    perfReportStartMs = nowMs;
    perfLastDrdy = totalDrdy;
    perfLastLost = lostSamples;
  } else if (nowMs - perfReportStartMs >= PERF_REPORT_MS) {
    uint32_t elapsedMs = nowMs - perfReportStartMs;
    uint32_t drdyDelta = totalDrdy - perfLastDrdy;
    uint32_t lostDelta = lostSamples - perfLastLost;
    float effectiveHz = (elapsedMs > 0) ? ((float)drdyDelta * 1000.0f / (float)elapsedMs) : 0.0f;

    Serial.printf("# PERF: DRDY=%.1f Hz, perdidas=%lu, max_gap=%lu us, max_SPI=%lu us, max_read=%lu us, max_loop=%lu us\n",
                  effectiveHz, (unsigned long)lostDelta,
                  (unsigned long)perfMaxSampleGapUs, (unsigned long)perfMaxSpiUs,
                  (unsigned long)perfMaxReadUs, (unsigned long)perfMaxLoopUs);

    perfReportStartMs = nowMs;
    perfLastDrdy = totalDrdy;
    perfLastLost = lostSamples;
    perfMaxSampleGapUs = 0;
    perfMaxSpiUs = 0;
    perfMaxReadUs = 0;
    perfMaxLoopUs = 0;
  }
  uint32_t loopDurationUs = micros() - loopStartUs;
  if (loopDurationUs > perfMaxLoopUs) perfMaxLoopUs = loopDurationUs;
#endif
}
