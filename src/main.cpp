/* PID_GPT_V.1.2 – KamilBol (17.11.2025)
   ESP32-S3 DevKitC-1 N16R8
   Pełny regulator PID z OTA, WiFiManager, Nextion, SD, Modbus

*/

#include <Arduino.h>
#include <ModbusMaster.h>
#include <PZEM004Tv30.h>
#include <SdFat.h>
#include <DHT.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>        // <-- dodane

// ===== Sekcja nr. 1 "DEFINICJE I KONFIGURACJA HARDWARE" =====
#define PZEM_RX 16
#define PZEM_TX 17
#define PZEM_DE 4

#define VFD_RX 9
#define VFD_TX 10
#define VFD_DE 5

#define NEXTION_RX 43   // ESP TX → Nextion RX
#define NEXTION_TX 44   // ESP RX ← Nextion TX

#define DHT_PIN 11
#define SD_CS_PIN 13

#define PZEM_ADDR 0x01
#define VFD_ADDR 1

const long PZEM_BAUD = 9600;
const long VFD_BAUD = 38400;
const unsigned long PID_TS_MS = 200;
const unsigned long MODBUS_WATCHDOG_MS = 5000;
const float I_MAX_DEFAULT = 38.0;
const float I_WARNING_MARGIN = 0.5;
const float ANALOG_MAX_V = 17.0;
float FREQ_MAX_HZ = 40.0;
const float RATE_LIMIT_PERCENT_PER_S = 2.0;
const unsigned long PRED_DELAY_MS = 4500;
// ===== KONIEC SEKCJI NR 1 =====

// ===== Sekcja nr. 2 "OBIEKTY I BIBLIOTEKI" =====
HardwareSerial PZEMSerial(2);
HardwareSerial VFDSerial(1);
HardwareSerial NextionSerial(0);

ModbusMaster node;
PZEM004Tv30 pzem(&PZEMSerial);           // <-- poprawiony konstruktor
SdFat SD;
DHT dht(DHT_PIN, DHT11);
WiFiManager wifiManager;
// ===== KONIEC SEKCJI NR 2 =====

// ===== Sekcja nr. 3 "ZMIENNE GLOBALNE" =====
volatile unsigned long lastModbusOK = 0;
float I_meas = 0.0;
float I_max = I_MAX_DEFAULT;
float I_min = 0.0;
float Kp = 0.6, Ki = 0.4, Kd = 0.02;
float pid_integral = 0.0;
float last_error = 0.0;
float last_output_hz10 = 0.0;
float i_slope = 0.0;
unsigned long lastIread_ms = 0;
unsigned long programStart_ms = 0;

const uint32_t MAX_FILE_SIZE = 10UL * 1024UL * 1024UL;
const uint16_t MAX_FILES = 100;
uint32_t file_counter = 0;
char currentLogName[32];
File logFile;
// ===== KONIEC SEKCJI 3 =====

// ===== Sekcja nr. 4 "FUNKCJE POMOCNICZE - RS485 / DE" =====
void preTransmissionVFD()  { digitalWrite(VFD_DE, HIGH); delayMicroseconds(50); }
void postTransmissionVFD() { delayMicroseconds(50); digitalWrite(VFD_DE, LOW); }
// ===== KONIEC SEKCJI 4 =====

// ===== Sekcja nr. 5 "MODBUS - ZAPIS/ODCZYT" =====
bool writeSetpointModbus(uint16_t regValue) {
  uint8_t result = node.writeSingleRegister(2, regValue);
  if (result == node.ku8MBSuccess) {
    lastModbusOK = millis();
    return true;
  }
  return false;
}
bool readFalownikRegs(uint16_t &freq_out_hz10, uint16_t &fault) {
  uint8_t res = node.readHoldingRegisters(7, 2);
  if (res == node.ku8MBSuccess) {
    freq_out_hz10 = node.getResponseBuffer(0);
    fault = node.getResponseBuffer(1);
    lastModbusOK = millis();
    return true;
  }
  return false;
}
// ===== KONIEC SEKCJI 5 =====

// ===== Sekcja nr. 6 "PZEM - ODCZYT" =====
void readPZEM() {
  float i = pzem.current();
  if (!isnan(i)) {
    unsigned long now = millis();
    if (lastIread_ms != 0) {
      float dt = (now - lastIread_ms) / 1000.0f;
      float slope = (i - I_meas) / max(dt, 0.001f);
      i_slope = i_slope * 0.8f + slope * 0.2f;
    }
    I_meas = i;
    lastIread_ms = now;
  }
}
// ===== KONIEC SEKCJI 6 =====

// ===== Sekcja nr. 7 "PID + PREDYKCJA + RATE LIMIT" =====
float predictFutureI() {
  return I_meas + i_slope * (PRED_DELAY_MS / 1000.0f);
}
uint16_t computeSetpointValue() {
  float I_pred = predictFutureI();
  float I_target = min(I_max, I_pred);
  float error = I_target - I_meas;
  float dt = PID_TS_MS / 1000.0f;

  pid_integral += error * dt;
  float integral_limit = 1000.0f;
  if (pid_integral > integral_limit) pid_integral = integral_limit;
  if (pid_integral < -integral_limit) pid_integral = -integral_limit;

  float derivative = (error - last_error) / max(dt, 1e-6f);
  float raw = Kp * error + Ki * pid_integral + Kd * derivative;
  float out_percent = constrain(raw, -100.0f, 100.0f);

  float baseHz = last_output_hz10 / 10.0f;
  float desiredHz = baseHz + (out_percent / 100.0f) * FREQ_MAX_HZ;

  float maxDeltaPerSecHz = (RATE_LIMIT_PERCENT_PER_S / 100.0f) * FREQ_MAX_HZ;
  float maxDeltaPerLoopHz = maxDeltaPerSecHz * dt;
  float deltaHz = constrain(desiredHz - baseHz, -maxDeltaPerLoopHz, maxDeltaPerLoopHz);
  float finalHz = constrain(baseHz + deltaHz, 0.0f, FREQ_MAX_HZ);

  last_error = error;
  last_output_hz10 = round(finalHz * 10.0f);
  return (uint16_t)last_output_hz10;
}
// ===== KONIEC SEKCJI 7 =====

// ===== Sekcja nr. 8 "WATCHDOG I SOFT FALLBACK" =====
enum CommState { COMM_OK, COMM_LOST };
CommState commState = COMM_OK;

void checkComm() {
  unsigned long now = millis();
  if (now - lastModbusOK > MODBUS_WATCHDOG_MS) {
    if (commState != COMM_LOST) {
      commState = COMM_LOST;
      Ki *= 0.3f;
    }
  } else {
    if (commState == COMM_LOST) {
      commState = COMM_OK;
    }
  }
}
// ===== KONIEC SEKCJI 8 =====

// ===== Sekcja nr. 9 "SD LOGGING + ROTACJA PLIKÓW" =====
void newLogFile() {
  file_counter++;
  sprintf(currentLogName, "/log_%06lu.csv", file_counter);
  if (logFile) logFile.close();
  logFile = SD.open(currentLogName, FILE_WRITE);
  if (logFile) {
    logFile.println("idx,ms_since_start,I_meas,I_set,modbus_freq_hz10,freq_out_hz10,error,temp,hum,commState");
    logFile.flush();
  }
  if (file_counter > MAX_FILES) {
    char oldname[32];
    sprintf(oldname, "/log_%06lu.csv", file_counter - MAX_FILES);
    SD.remove(oldname);
  }
}
void logToSD() {
  if (!logFile || logFile.size() > MAX_FILE_SIZE) { newLogFile(); if (!logFile) return; }
  float T = dht.readTemperature();
  float H = dht.readHumidity();
  unsigned long ms = millis() - programStart_ms;
  uint16_t freq_out=0, fault=0;
  readFalownikRegs(freq_out, fault);
  logFile.printf("%lu,%lu,%.3f,%.3f,%d,%d,%.3f,%.1f,%.1f,%d\n",
    file_counter, ms, I_meas, I_max-I_meas, (int)last_output_hz10, freq_out,
    I_max-I_meas, T, H, (int)commState);
  logFile.flush();
}
// ===== KONIEC SEKCJI 9 =====

// ===== Sekcja nr. 10 "NEXTION - WYSYŁANIE" =====
void nextionSend(const char *cmd) {
  NextionSerial.print(cmd);
  NextionSerial.write(0xFF); NextionSerial.write(0xFF); NextionSerial.write(0xFF);
}
void updateNextion() {
  char b[64];
  snprintf(b, sizeof(b), "tI.txt=\"%0.2fA\"", I_meas); nextionSend(b);
  float T = dht.readTemperature();
  float H = dht.readHumidity();
  snprintf(b, sizeof(b), "tTemp.txt=\"%0.1fC\"", T); nextionSend(b);
  snprintf(b, sizeof(b), "tHum.txt=\"%0.1f%%\"", H); nextionSend(b);
}
// ===== KONIEC SEKCJI 10 =====

// ===== Sekcja nr. 11 "SETUP i LOOP" =====
void setup() {
  Serial.begin(115200);
  Serial.println("=== Regulator_PID v1.2 – KamilBol ===");

  pinMode(PZEM_DE, OUTPUT); digitalWrite(PZEM_DE, LOW);
  pinMode(VFD_DE, OUTPUT); digitalWrite(VFD_DE, LOW);
  pinMode(SD_CS_PIN, OUTPUT);

  // === WIFI + OTA ===
  wifiManager.autoConnect("PID-Setup");
  ArduinoOTA.setHostname("Regulator-PID");
  ArduinoOTA.setPassword("pid123");
  ArduinoOTA.begin();
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // UART-y
  PZEMSerial.begin(PZEM_BAUD, SERIAL_8N1, PZEM_RX, PZEM_TX);
  VFDSerial.begin(VFD_BAUD, SERIAL_8N1, VFD_RX, VFD_TX);
  NextionSerial.begin(115200, SERIAL_8N1, NEXTION_TX, NEXTION_RX);

  node.begin(VFD_ADDR, VFDSerial);
  node.preTransmission(preTransmissionVFD);
  node.postTransmission(postTransmissionVFD);

  pzem.setAddress(PZEM_ADDR);  // <-- ważne!
  dht.begin();
  if (SD.begin(SD_CS_PIN)) newLogFile();

  programStart_ms = millis();
  lastModbusOK = millis();
}

unsigned long lastPid = 0, lastLog = 0;
void loop() {
  ArduinoOTA.handle();  // <-- OTA zawsze aktywne

  unsigned long now = millis();
  if (now - lastIread_ms >= 200) readPZEM();
  checkComm();

  if (now - lastPid >= PID_TS_MS) {
    lastPid = now;
    uint16_t val = computeSetpointValue();
    if (commState == COMM_OK) writeSetpointModbus(val);
    updateNextion();
  }

  if (now - lastLog >= 1000) {
    lastLog = now;
    logToSD();
  }

  delay(1);
}
// ===== KONIEC SEKCJI NR 11 =====
