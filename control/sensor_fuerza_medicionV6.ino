/*
  Sensor de fuerza de 2 ejes (cruz de malta, 8 galgas) - MODO MEDICIÓN CONTINUA
  ADS131M02 (2000 SPS, PGA 128) + XIAO ESP32-C3

  Uso (monitor serie a 115200, fin de línea "Nueva línea"):
    1. Pon la ESP en el socket, deja el sensor SIN CARGA y envía  i
       -> inicia el ADC, toma el cero y empieza a medir sola, todo el tiempo.
    2. Comandos:
         z  -> vuelve a tomar el cero (sensor SIN CARGA)
         p  -> pausa / reanuda la salida
         v  -> cambia entre CSV (para guardar) y texto legible con nombres y unidades
         l  -> explica qué significa cada dato
         k  -> muestra la calibración
         h  -> ayuda

  Salida CSV (OUT_HZ veces por segundo):
    t_s,Fy_N,Fx_N,F_N,ang_deg,dY_uV,dX_uV,Z,perdidas
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

// ----------------------------- Pines ---------------------------------
#define PIN_SCLK  D8
#define PIN_MISO  D9    // DOUT del ADC
#define PIN_MOSI  D10   // DIN del ADC
#define PIN_CS    D5
#define PIN_DRDY  D0

// ----------------------- Parámetros del sistema -----------------------
#define SPI_HZ        4000000UL
#define FS_HZ         2000        // 4.096 MHz / 2 / OSR 1024
#define PGA_GAIN      128         // ±1.2 V / 128 = ±9.375 mV
#define ADC_VREF      1.2
#define AMP_GAIN      1.0
#define SAT_MV        8.5         // aviso de saturación

// ------------- Canales del ADC -> ejes del mecanismo -------------------
// Igual que en la calibración: Y = -AIN1, X = +AIN0.
// Si al empujar en +Y el Fy sale negativo, cambia SIGN_Y (igual para X).
#define ADC_CH_Y      1
#define SIGN_Y        (-1.0)
#define ADC_CH_X      0
#define SIGN_X        (+1.0)

// ----------------------- Calibración (N/mV) ---------------------------
#define C_YY   41.8173    // Fy por mV de dY
#define C_YX    2.8449    // Fy por mV de dX
#define C_XY   -1.3416    // Fx por mV de dY
#define C_XX  -38.7461    // Fx por mV de dX

// ----------------------- Detección de carga en Z ----------------------
// Carga en Y o X: el otro canal responde 3-7 % (δ <= 4°).
// Carga en Z: los dos canales responden parecido (~50 %, δ mediana 15°).
#define Z_SENAL_MIN_UV  15.0      // por debajo no se decide (ruido / carga muy baja)
#define Z_DELTA_DEG     7.0       // δ = atan(menor/mayor) por encima de esto -> patrón Z
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

double   zero_mV[2] = {0.0, 0.0};   // cero en el marco del mecanismo: [0]=Y, [1]=X
uint32_t lostSamples = 0;
uint32_t totalDrdy   = 0;
bool     adcReady    = false;
bool     output      = true;

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

static inline double codeToMV(int32_t code) {
  return (double)code * (ADC_VREF / PGA_GAIN) / 8388608.0 / AMP_GAIN * 1000.0;
}

// Devuelve una muestra ya en el marco del mecanismo: mv[0]=Y, mv[1]=X. raw = canales crudos.
static bool getSample(double mv[2], double raw[2]) {
  uint32_t p = __atomic_exchange_n(&drdyPending, 0, __ATOMIC_SEQ_CST);
  if (p == 0) return false;
  if (p > 1) lostSamples += p - 1;
  totalDrdy += p;

  uint8_t tx[FRAME_BYTES] = {0}, rx[FRAME_BYTES];
  spiFrame(tx, rx);
  raw[0] = codeToMV(signExtend24(&rx[1 * WORD_BYTES]));   // AIN0
  raw[1] = codeToMV(signExtend24(&rx[2 * WORD_BYTES]));   // AIN1
  mv[0] = SIGN_Y * raw[ADC_CH_Y];
  mv[1] = SIGN_X * raw[ADC_CH_X];
  return true;
}

static inline void toForce(double dy_mV, double dx_mV, double &fy, double &fx) {
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

static void flushAdc() {
  uint8_t rx[FRAME_BYTES];
  for (int i = 0; i < 3; i++) nullFrame(rx);
  __atomic_exchange_n(&drdyPending, 0, __ATOMIC_SEQ_CST);
}

// ----------------------------- Cero -----------------------------------
static bool takeZero() {
  for (int s = ZERO_COUNTDOWN; s > 0; s--) {
    Serial.printf("# Cero (sin carga, sin tocar) en %d s...\n", s);
    delay(1000);
  }
  flushAdc();
  const uint32_t N = (uint32_t)FS_HZ * ZERO_SECONDS;
  double sum[2] = {0, 0}, mv[2], raw[2];
  uint32_t n = 0, tLast = millis();
  while (n < N) {
    if (getSample(mv, raw)) { sum[0] += mv[0]; sum[1] += mv[1]; n++; tLast = millis(); }
    else if (millis() - tLast > 1000) { Serial.println("# ERROR: no llegan pulsos de DRDY."); return false; }
  }
  zero_mV[0] = sum[0] / n;
  zero_mV[1] = sum[1] / n;
  Serial.printf("# CERO: Y0=%.5f mV  X0=%.5f mV\n", zero_mV[0], zero_mV[1]);
  return true;
}

// ----------------------------- Comandos -------------------------------
bool legible = false;   // false = CSV (para guardar/graficar) | true = texto con nombres y unidades

static void printHeader() {
  if (!legible) Serial.println("t_s,Fy_N,Fx_N,F_N,ang_deg,dY_uV,dX_uV,Z,perdidas");
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
  Serial.println("# Z        0 = carga normal en X/Y: Fy y Fx son confiables.");
  Serial.println("#          1 = reaccionan los dos puentes a la vez (patrón de carga en Z o carga en");
  Serial.println("#              diagonal): Fy y Fx de esa lectura NO son confiables.");
  Serial.println("# perdidas Muestras del ADC que no se alcanzaron a leer desde el inicio. Debe quedarse en 0;");
  Serial.println("#          si sube, la ESP está ocupada (por ejemplo, el monitor serie no da abasto).");
  Serial.println("# Líneas que empiezan con '#': mensajes, no son datos.");
  Serial.println("# =============================================================");
}

static void printHelp() {
  Serial.println("# Comandos: i (iniciar) | z (cero, SIN CARGA) | p (pausa) | v (CSV <-> texto legible)");
  Serial.println("#           l (qué significa cada dato) | k (calibración) | h (ayuda)");
}

static void printCal() {
  Serial.printf("# C = [[%.4f, %.4f], [%.4f, %.4f]] N/mV | Y = %+.0f*AIN%d, X = %+.0f*AIN%d\n",
                C_YY, C_YX, C_XY, C_XX, SIGN_Y, ADC_CH_Y, SIGN_X, ADC_CH_X);
  Serial.printf("# Z: señal > %.0f uV y delta > %.0f deg durante %d lecturas | salida %d Hz\n",
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
  if (!takeZero()) {
    Serial.println("# Sin DRDY. Revisa DRDY y el oscilador, y envía 'i' de nuevo.");
    return;
  }
  totalDrdy = 0;
  lostSamples = 0;
  adcReady = true;
  printCal();
  printLegend();
  printHelp();
  printHeader();
}

static void handleLine(char *line) {
  while (*line == ' ') line++;
  char c = tolower(line[0]);
  if (c == 'i') { startAdc(); return; }
  if (c == 'h' || c == 0) { printHelp(); return; }
  if (!adcReady) { Serial.println("# Primero envía 'i' (ESP en el socket, sensor sin carga)."); return; }
  if (c == 'z') { if (takeZero()) printHeader(); }
  else if (c == 'p') { output = !output; Serial.printf("# Salida %s\n", output ? "ON" : "PAUSADA"); if (output) printHeader(); }
  else if (c == 'k') printCal();
  else if (c == 'l') printLegend();
  else if (c == 'v') {
    legible = !legible;
    Serial.printf("# Formato: %s\n", legible ? "texto legible" : "CSV");
    printHeader();
  }
  else printHelp();
  flushAdc();
}

// ----------------------------- Setup / loop ---------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DRDY, INPUT_PULLUP);
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  Serial.println("# MEDICIÓN. Pon la ESP en el socket, deja el sensor SIN CARGA y envía 'i'.");
}

void loop() {
  static double   acc[2] = {0, 0}, rawMax = 0;
  static uint16_t cnt = 0;
  static uint8_t  zCount = 0;
  static bool     zFlag = false;
  static char     line[32];
  static uint8_t  len = 0;
  double mv[2], raw[2];

  if (adcReady && getSample(mv, raw)) {
    acc[0] += mv[0]; acc[1] += mv[1];
    rawMax = fmax(rawMax, fmax(fabs(raw[0]), fabs(raw[1])));
    if (++cnt >= OUT_AVG) {
      double dy = acc[0] / cnt - zero_mV[0];          // mV
      double dx = acc[1] / cnt - zero_mV[1];
      bool sat = rawMax > SAT_MV;
      acc[0] = acc[1] = 0; cnt = 0; rawMax = 0;

      double fy, fx;
      toForce(dy, dx, fy, fx);
      double f   = hypot(fy, fx);
      double ang = atan2(fx, fy) * 57.29578;

      // Patrón Z: los dos puentes reaccionan a la vez
      double ay = fabs(dy) * 1000.0, ax = fabs(dx) * 1000.0;      // uV
      double M = hypot(ay, ax);
      double delta = (M > 0) ? atan2(fmin(ay, ax), fmax(ay, ax)) * 57.29578 : 0;
      bool zNow = (M > Z_SENAL_MIN_UV) && (delta > Z_DELTA_DEG);
      if (zNow != zFlag) { if (++zCount >= Z_CONFIRMA) { zFlag = zNow; zCount = 0; } }
      else zCount = 0;

      if (output) {
        double t_s = totalDrdy / (double)FS_HZ;
        if (sat) Serial.println("# ADVERTENCIA: cerca de saturación del PGA (señal > 8.5 mV): fuerza fuera de rango");
        if (legible) {
          // Una línea con nombre y unidad de cada dato
          Serial.printf("t=%7.2f s | Fy=%+8.3f N | Fx=%+8.3f N | |F|=%7.3f N | dir=%+6.1f deg | "
                        "dY=%+7.1f uV | dX=%+7.1f uV | %s | perdidas=%lu\n",
                        t_s, fy, fx, f, ang, dy * 1000.0, dx * 1000.0,
                        zFlag ? "Z: SI (no confiar en Fy/Fx)" : "Z: no", (unsigned long)lostSamples);
        } else {
          // CSV, en este orden (ver printLegend):
          Serial.printf("%.2f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%d,%lu\n",
                        t_s,               // t_s      tiempo [s]
                        fy,                // Fy_N     fuerza en Y [N]
                        fx,                // Fx_N     fuerza en X [N]
                        f,                 // F_N      magnitud en el plano [N]
                        ang,               // ang_deg  dirección: 0=+Y, 90=+X [°]
                        dy * 1000.0,       // dY_uV    señal del puente Y [uV]
                        dx * 1000.0,       // dX_uV    señal del puente X [uV]
                        zFlag ? 1 : 0,     // Z        1 = patrón de carga en Z (Fy/Fx no confiables)
                        (unsigned long)lostSamples);  // perdidas  muestras no leídas
        }
      }
    }
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { line[len] = 0; handleLine(line); len = 0; acc[0] = acc[1] = 0; cnt = 0; rawMax = 0; }
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}
