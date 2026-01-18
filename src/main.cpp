#include <imu.h>
#define PACKET_SIZE 31 
#define DEBUG false

volatile uint8_t txBuffer[PACKET_SIZE];

HardwareSerial DebugSerial(PA10, PA9); 
TwoWire SensorBus(PB4, PA7);
TwoWire SlaveBus(PB7, PB6);

Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345, &SensorBus);
Adafruit_ICM20948 icm;

// --- SKÁLÁZÁSI FAKTOROK (Konstansok) ---
// Ezekkel konvertáljuk vissza a floatot 16 bites int-re
// ADXL375: 49 mg/LSB -> 1 G = ~20.48 LSB. 1 G = 9.81 m/s2.
// Tehát: Val = (Accel_m_s2 / 9.81) * 20.48
// ICM Accel (16G range): 2048 LSB / G
// ICM Gyro (2000 dps range): 16.4 LSB / dps
// ICM Mag: ~0.15 uT / LSB -> Val = uT / 0.15

void requestEvent() {
  SlaveBus.write((uint8_t*)txBuffer, PACKET_SIZE);
}

void appendInt16(int16_t value, volatile uint8_t* buffer, int &idx) {
  buffer[idx++] = value & 0xFF;
  buffer[idx++] = (value >> 8) & 0xFF;
}

void appendUint8(uint8_t value, volatile uint8_t* buffer, int &idx) {
  buffer[idx++] = value;
}

void appendUint24(uint32_t value, volatile uint8_t* buffer, int &idx) {
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

static int8_t packTempInt8C(float tempC) {
  if (isnan(tempC)) return INT8_MIN;
  long v = lroundf(tempC);
  if (v > INT8_MAX) v = INT8_MAX;
  if (v < INT8_MIN + 1) v = INT8_MIN + 1;
  return (int8_t)v;
}

static float readStm32TempC() {
  #if defined(ARDUINO_ARCH_STM32) && defined(ATEMP)
    const uint32_t raw = analogRead(ATEMP);
    if (raw <= 1 || raw >= 4094) {
      return NAN;
    }
    const float vref = 3.3f;
    const float vsense = (raw * vref) / 4095.0f;
    const float v25 = 0.76f;
    const float avgSlope = 0.0025f; // V/C
    return ((vsense - v25) / avgSlope) + 25.0f;
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
  if(adxl.begin()){
    DebugSerial.println("[IMU] (INIT) ADXL375 - OK");
  } else {
    DebugSerial.println("[IMU] (INIT) ADXL375 - !!!ERROR!!!");
  } 


  DebugSerial.println("[IMU] (INIT) ICM20948 - INIT");

  if(icm.begin_I2C(0x68, &SensorBus)) {
    DebugSerial.println("[IMU] (INIT) ICM20948 - OK");
  } else {
    DebugSerial.println("[IMU] (INIT) ICM20948 - !!!ERROR!!!");
  } 
  
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  
  DebugSerial.println("[IMU] (INIT) I2C SLAVE (0x51) - INIT");

  SlaveBus.begin(0x51);
  SlaveBus.onRequest(requestEvent);
  
  DebugSerial.println("[IMU] (INIT) I2C SLAVE (0x51) -  OK");
  DebugSerial.println("[IMU] (INIT) END OF INITIALIZATION!");
}

void loop() {
  sensors_event_t ev_adxl, a_icm, g_icm, m_icm, t_icm;
  adxl.getEvent(&ev_adxl);
  icm.getEvent(&a_icm, &g_icm, &t_icm, &m_icm);

  const float temp_icm_c = t_icm.temperature;
  const float temp_stm32_c = readStm32TempC();
  const uint32_t obc_seconds = (uint32_t)(millis() / 1000UL);

  noInterrupts();
  
  int idx = 0;

  float adxl_factor = 20.48 / 9.80665; 
  appendInt16((int16_t)(ev_adxl.acceleration.x * adxl_factor), txBuffer, idx);
  appendInt16((int16_t)(ev_adxl.acceleration.y * adxl_factor), txBuffer, idx);
  appendInt16((int16_t)(ev_adxl.acceleration.z * adxl_factor), txBuffer, idx);

  float icm_a_factor = 2048.0 / 9.80665;
  appendInt16((int16_t)(a_icm.acceleration.x * icm_a_factor), txBuffer, idx);
  appendInt16((int16_t)(a_icm.acceleration.y * icm_a_factor), txBuffer, idx);
  appendInt16((int16_t)(a_icm.acceleration.z * icm_a_factor), txBuffer, idx);

  float icm_g_factor = 57.2958 * 16.4;
  appendInt16((int16_t)(g_icm.gyro.x * icm_g_factor), txBuffer, idx);
  appendInt16((int16_t)(g_icm.gyro.y * icm_g_factor), txBuffer, idx);
  appendInt16((int16_t)(g_icm.gyro.z * icm_g_factor), txBuffer, idx);

  float icm_m_factor = 1.0 / 0.15;
  appendInt16((int16_t)(m_icm.magnetic.x * icm_m_factor), txBuffer, idx);
  appendInt16((int16_t)(m_icm.magnetic.y * icm_m_factor), txBuffer, idx);
  appendInt16((int16_t)(m_icm.magnetic.z * icm_m_factor), txBuffer, idx);

  appendInt16(packTempCenti(temp_icm_c), txBuffer, idx);
  appendInt16(packTempCenti(temp_stm32_c), txBuffer, idx);
  appendUint24(obc_seconds, txBuffer, idx);

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
      for(int i=0; i<PACKET_SIZE; i++) {
          if(txBuffer[i]<0x10) DebugSerial.print("0");
          DebugSerial.print(txBuffer[i], HEX);
      }
      DebugSerial.println();
      lastT = millis();
    }
  #endif
}