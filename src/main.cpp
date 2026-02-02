#include <imu.h>
#define PACKET_SIZE 31
#define DEBUG true

HardwareSerial DebugSerial(PA10, PA9);
TwoWire SensorBus(PB4, PA7);
TwoWire SlaveBus(PB7, PB6);

Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345, &SensorBus);
Adafruit_ICM20948 icm;

// --- Double buffer ---
static volatile uint8_t txA[PACKET_SIZE];
static volatile uint8_t txB[PACKET_SIZE];
// Mindig erre a bufferre küld az onRequest()
static volatile uint8_t* txActive = txA;
// Ezt tölti a loop(), majd swap
static volatile uint8_t* txBuild  = txB;

void requestEvent() {
  // ISR-ben: csak kiküldjük az aktuális, kész packetet
  SlaveBus.write((const uint8_t*)txActive, PACKET_SIZE);
}

static inline void appendInt16(int16_t value, uint8_t* buffer, int &idx) {
  buffer[idx++] = (uint8_t)(value & 0xFF);
  buffer[idx++] = (uint8_t)((value >> 8) & 0xFF);
}

static inline void appendUint24(uint32_t value, uint8_t* buffer, int &idx) {
  buffer[idx++] = (uint8_t)(value & 0xFF);
  buffer[idx++] = (uint8_t)((value >> 8) & 0xFF);
  buffer[idx++] = (uint8_t)((value >> 16) & 0xFF);
}

static int16_t packTempCenti(float tempC) {
  if (isnan(tempC)) return INT16_MIN;
  long v = lroundf(tempC * 100.0f);
  if (v > INT16_MAX) v = INT16_MAX;
  if (v < (long)INT16_MIN + 1) v = (long)INT16_MIN + 1;
  return (int16_t)v;
}

static float readStm32TempC() {
#if defined(ARDUINO_ARCH_STM32) && defined(ATEMP)
  const uint32_t raw = analogRead(ATEMP);
  if (raw <= 1 || raw >= 4094) return NAN;

  const float vref = 3.3f;
  const float vsense = (raw * vref) / 4095.0f;
  const float v25 = 0.76f;
  const float avgSlope = 0.0025f; // V/C
  return ((vsense - v25) / avgSlope) + 25.0f;
#else
  return NAN; // fontos: mindig legyen return
#endif
}

void setup() {
  delay(1000);

#if DEBUG
  DebugSerial.begin(115200);
  DebugSerial.println("[IMU] (INIT) -=-=-=-=-=-=-=-=-=-=-=-=-=-=-");
  DebugSerial.println("[IMU] (INIT) OnionSAT - PROJECT NEPTUNE");
  DebugSerial.println("[IMU] (INIT) Inertium Measurement Unit");
  DebugSerial.println("[IMU] (INIT)");
  DebugSerial.print("[IMU] (INIT) Build Date: "); DebugSerial.print(__DATE__); DebugSerial.print(" "); DebugSerial.println(__TIME__);
  DebugSerial.print("[IMU] (INIT) Build Version: "); DebugSerial.println("v0.1");
  DebugSerial.print("[IMU] (INIT) Compiler: "); DebugSerial.println(__VERSION__);
  DebugSerial.println("[IMU] (INIT) -=-=-=-=-=-=-=-=-=-=-=-=-=-=-");
#endif

  SensorBus.begin();

#if defined(ARDUINO_ARCH_STM32)
  analogReadResolution(12);
#endif

  DebugSerial.println("[IMU] (INIT) ADXL375 - INIT");
  if (adxl.begin()) DebugSerial.println("[IMU] (INIT) ADXL375 - OK");
  else             DebugSerial.println("[IMU] (INIT) ADXL375 - !!!ERROR!!!");

  DebugSerial.println("[IMU] (INIT) ICM20948 - INIT");
  if (icm.begin_I2C(0x68, &SensorBus)) DebugSerial.println("[IMU] (INIT) ICM20948 - OK");
  else                                DebugSerial.println("[IMU] (INIT) ICM20948 - !!!ERROR!!!");

  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);

  DebugSerial.println("[IMU] (INIT) I2C SLAVE (0x51) - INIT");
  SlaveBus.begin(0x51);
  SlaveBus.onRequest(requestEvent);
  DebugSerial.println("[IMU] (INIT) I2C SLAVE (0x51) - OK");
  DebugSerial.println("[IMU] (INIT) END OF INITIALIZATION!");

  // opcionális: inicializáld a buffereket nullára
  for (int i = 0; i < PACKET_SIZE; i++) {
    ((uint8_t*)txA)[i] = 0;
    ((uint8_t*)txB)[i] = 0;
  }
}

void loop() {
  sensors_event_t ev_adxl, a_icm, g_icm, m_icm, t_icm;
  adxl.getEvent(&ev_adxl);
  icm.getEvent(&a_icm, &g_icm, &t_icm, &m_icm);

  const float temp_icm_c  = t_icm.temperature;
  const float temp_stm32_c = readStm32TempC();
  const uint32_t obc_seconds = (uint32_t)(millis() / 1000UL);

  // 1) Packet építése NEM kritikus szakaszban, a "build" bufferbe
  uint8_t local[PACKET_SIZE];
  int idx = 0;

  const float adxl_factor = 20.48f / 9.80665f;
  appendInt16((int16_t)lroundf(ev_adxl.acceleration.x * adxl_factor), local, idx);
  appendInt16((int16_t)lroundf(ev_adxl.acceleration.y * adxl_factor), local, idx);
  appendInt16((int16_t)lroundf(ev_adxl.acceleration.z * adxl_factor), local, idx);

  const float icm_a_factor = 2048.0f / 9.80665f;
  appendInt16((int16_t)lroundf(a_icm.acceleration.x * icm_a_factor), local, idx);
  appendInt16((int16_t)lroundf(a_icm.acceleration.y * icm_a_factor), local, idx);
  appendInt16((int16_t)lroundf(a_icm.acceleration.z * icm_a_factor), local, idx);

  const float icm_g_factor = 57.2958f * 16.4f; // rad/s -> dps, majd LSB/dps
  appendInt16((int16_t)lroundf(g_icm.gyro.x * icm_g_factor), local, idx);
  appendInt16((int16_t)lroundf(g_icm.gyro.y * icm_g_factor), local, idx);
  appendInt16((int16_t)lroundf(g_icm.gyro.z * icm_g_factor), local, idx);

  const float icm_m_factor = 1.0f / 0.15f; // uT -> LSB
  appendInt16((int16_t)lroundf(m_icm.magnetic.x * icm_m_factor), local, idx);
  appendInt16((int16_t)lroundf(m_icm.magnetic.y * icm_m_factor), local, idx);
  appendInt16((int16_t)lroundf(m_icm.magnetic.z * icm_m_factor), local, idx);

  appendInt16(packTempCenti(temp_icm_c), local, idx);
  appendInt16(packTempCenti(temp_stm32_c), local, idx);
  appendUint24(obc_seconds, local, idx);

  // Biztonsági ellenőrzés (DEBUG-nál hasznos)
#if DEBUG
  if (idx != PACKET_SIZE) {
    DebugSerial.print("[IMU] (ERR) Packet size mismatch, idx=");
    DebugSerial.println(idx);
  }
#endif

  // 2) Build bufferbe másolás (még mindig nem kritikus)
  for (int i = 0; i < PACKET_SIZE; i++) {
    ((uint8_t*)txBuild)[i] = local[i];
  }

  // 3) Atomikus swap: nagyon rövid kritikus szakasz
  noInterrupts();
  volatile uint8_t* oldActive = txActive;
  txActive = txBuild;
  txBuild  = oldActive;
  interrupts();

#if DEBUG
  static uint32_t lastT = 0;
  if (millis() - lastT > 500) {
    DebugSerial.print("[IMU] (DATA) - ADXL - a[m/s^2]: ");
    DebugSerial.print(ev_adxl.acceleration.x, 3); DebugSerial.print(" ");
    DebugSerial.print(ev_adxl.acceleration.y, 3); DebugSerial.print(" ");
    DebugSerial.println(ev_adxl.acceleration.z, 3);

    DebugSerial.print("[IMU] (DATA) - ICM  - a[m/s^2]: ");
    DebugSerial.print(a_icm.acceleration.x, 3); DebugSerial.print(" ");
    DebugSerial.print(a_icm.acceleration.y, 3); DebugSerial.print(" ");
    DebugSerial.println(a_icm.acceleration.z, 3);

    DebugSerial.print("[IMU] (DATA) - ICM  - g[rad/s]: ");
    DebugSerial.print(g_icm.gyro.x, 3); DebugSerial.print(" ");
    DebugSerial.print(g_icm.gyro.y, 3); DebugSerial.print(" ");
    DebugSerial.println(g_icm.gyro.z, 3);

    DebugSerial.print("[IMU] (DATA) - ICM  - m[uT]: ");
    DebugSerial.print(m_icm.magnetic.x, 3); DebugSerial.print(" ");
    DebugSerial.print(m_icm.magnetic.y, 3); DebugSerial.print(" ");
    DebugSerial.println(m_icm.magnetic.z, 3);

    DebugSerial.print("[IMU] (DATA) - ICM  -  T[C]: "); DebugSerial.println(temp_icm_c);
    DebugSerial.print("[IMU] (DATA) - MCU  -  T[C]: "); DebugSerial.println(temp_stm32_c);
    DebugSerial.print("[IMU] (DATA) - OBC  -  t[s]: "); DebugSerial.println(obc_seconds);

    DebugSerial.print("[IMU] (I2C) PACKET: ");
    for (int i = 0; i < PACKET_SIZE; i++) {
      uint8_t b = ((const uint8_t*)txActive)[i];
      if (b < 0x10) DebugSerial.print("0");
      DebugSerial.print(b, HEX);
    }
    DebugSerial.println();
    lastT = millis();
  }
#endif
}
