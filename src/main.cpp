/* PID_GPT_V.1.1 - ESP32-S3 (DevKitC-1 N16R8) - Poprawiona wersja z OTA
   Cel: PID sterujący Optidrive E3 przez RS485, odczyt PZEM-004T, Nextion, DHT11, SD logging.
   Sekcje oznaczone zgodnie z wymaganiem użytkownika.
   Biblioteki wymagane: ModbusMaster, PZEM004Tv30, SdFat, DHT, WiFi, ArduinoOTA, WiFiManager.
*/

#include <Arduino.h>
#include <ModbusMaster.h>
#include <PZEM004Tv30.h>
#include <SdFat.h>
#include <DHT.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>  // Dodane: Łatwy config WiFi bez hardkodowania

// ===== Sekcja nr. 1 "DEFINICJE I KONFIGURACJA HARDWARE" =====
/* PINY (potwierdzone przez użytkownika) */
#define PZEM_RX 16        // ESP RX <- PZEM TX
#define PZEM_TX 17        // ESP TX -> PZEM RX
#define PZEM_DE 4         // DE/RE sterowanie (PZEM RS485 DE/RE połączone)

#define VFD_RX 9          // ESP RX <- RS485 falownik RO
#define VFD_TX 10         // ESP TX -> RS485 falownik DI
#define VFD_DE 5          // DE/RE sterowanie dla falownika (połączone)

#define NEXTION_RX 43     // ESP TX -> Nextion RX (dla S3 - swapped)
#define NEXTION_TX 44     // ESP RX <- Nextion TX

#define DHT_PIN 11        // DHT11 data pin
#define SD_CS_PIN 13      // SD CS pin (SPI CS)

#define PZEM_ADDR 0x01    // adres PZEM (domyślny)
#define VFD_ADDR 1        // adres falownika (potwierdzone)

// Systemowe parametry
const long PZEM_BAUD = 9600;
const long VFD_BAUD = 38400; // potwierdzone przez ciebie
const unsigned long PID_TS_MS = 200;
const unsigned long MODBUS_WATCHDOG_MS = 5000;
const float I_MAX_DEFAULT = 38.0;
const float I_WARNING_MARGIN = 0.5;
const float ANALOG_MAX_V = 17.0;
float FREQ_MAX_HZ = 40.0; // potwierdziłeś 40 Hz
const float RATE_LIMIT_PERCENT_PER_S = 2.0;
const unsigned long PRED_DELAY_MS = 4500; // 4.5s

// ===== KONIEC SEKCJI NR 1 "DEFINICJE I KONFIGURACJA HARDWARE" =====

// ===== Sekcja nr. 2 "OBIEKTY I BIBLIOTEKI" =====
HardwareSerial PZEMSerial(2);    // UART2 dla PZEM
HardwareSerial VFDSerial(1);     // UART1 dla falownika
HardwareSerial NextionSerial(0); // UART0 dla Nextion (custom pins)

ModbusMaster node;              // Modbus dla falownika
PZEM004Tv30 pzem(&PZEMSerial);  // PZEM (fix: bez ADDR w konstruktorze, użyj setAddress)
SdFat SD;
DHT dht(DHT_PIN, DHT11);
WiFiManager wifiManager;        // Dodane: Łatwy setup WiFi
// ===== KONIEC SEKCJI NR 2 "OBIEKTY I BIBLIOTEKI" =====

// ===== Sekcja nr. 3 "ZMIENNE GLOBALNE" =====
volatile unsigned long lastModbusOK = 0;
float I_meas = 0.0;
float I_max = I_MAX_DEFAULT;
float I_min = 0.0;
float Kp = 0.6, Ki = 0.4, Kd = 0.02;
float pid_integral = 0.0;
float last_error = 0.0;
float last_output_hz10 = 0.0; // Hz * 10
float i_slope = 0.0;
unsigned long lastIread_ms = 0;
unsigned long programStart_ms = 0;

// Logging
const uint32_t MAX_FILE_SIZE = 10UL * 1024UL * 1024UL;
const uint16_t MAX_FILES = 100;
uint32_t file_counter = 0;
char currentLogName[32];
File logFile;
// ===== KONIEC SEKCJI NR 3 "ZMIENNE GLOBALNE" =====


// ===== Sekcja nr. 4 "FUNKCJE POMOCNICZE - RS485 / DE" =====
void preTransmissionVFD() {
  digitalWrite(VFD_DE, HIGH); // DE HIGH - TX mode
  delayMicroseconds(50);
}
void postTransmissionVFD() {
  delayMicroseconds(50);
  digitalWrite(VFD_DE, LOW);  // DE LOW - RX mode
}
// ===== KONIEC SEKCJI NR 4 "FUNKCJE POMOCNICZE - RS485 / DE" =====


// ===== Sekcja nr. 5 "MODBUS - ZAPIS/ODCZYT" =====
bool writeSetpointModbus(uint16_t regValue) {
  // writeSingleRegister(rejestr, value) - rejestr 2 wg instrukcji
  uint8_t result = node.writeSingleRegister(2, regValue);
  if (result == node.ku8MBSuccess) {
    lastModbusOK = millis();
    return true;
  }
  return false;
}

bool readFalownikRegs(uint16_t &freq_out_hz10, uint16_t &fault) {
  uint8_t res = node.readHoldingRegisters(7, 2); // rejestr 7: freq_out, 8: fault (przykład z instr.)
  if (res == node.ku8MBSuccess) {
    freq_out_hz10 = node.getResponseBuffer(0);
    fault = node.getResponseBuffer(1);
    lastModbusOK = millis();
    return true;
  }
  return false;
}
// ===== KONIEC SEKCJI NR 5 "MODBUS - ZAPIS/ODCZYT" =====


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
// ===== KONIEC SEKCJI NR 6 "PZEM - ODCZYT" =====


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
  // anti-windup
  float integral_limit = 1000.0f;
  if (pid_integral > integral_limit) pid_integral = integral_limit;
  if (pid_integral < -integral_limit) pid_integral = -integral_limit;

  float derivative = (error - last_error) / max(dt, 1e-6f);
  float raw = Kp * error + Ki * pid_integral + Kd * derivative;

  // map raw to Hz change percent (tune later)
  float out_percent = raw;
  out_percent = constrain(out_percent, -100.0f, 100.0f);

  float baseHz = last_output_hz10 / 10.0f;
  float desiredHz = baseHz + (out_percent / 100.0f) * FREQ_MAX_HZ;

  // rate limiter
  float maxDeltaPerSecHz = (RATE_LIMIT_PERCENT_PER_S / 100.0f) * FREQ_MAX_HZ;
  float maxDeltaPerLoopHz = maxDeltaPerSecHz * (PID_TS_MS / 1000.0f);
  float deltaHz = desiredHz - baseHz;
  deltaHz = constrain(deltaHz, -maxDeltaPerLoopHz, maxDeltaPerLoopHz);
  float finalHz = baseHz + deltaHz;
  finalHz = constrain(finalHz, 0.0f, FREQ_MAX_HZ);

  last_error = error;
  last_output_hz10 = round(finalHz * 10.0f);
  return (uint16_t)last_output_hz10;
}
// ===== KONIEC SEKCJI NR 7 "PID + PREDYKCJA + RATE LIMIT" =====


// ===== Sekcja nr. 8 "WATCHDOG I SOFT FALLBACK" =====
enum CommState { COMM_OK, COMM_LOST };
CommState commState = COMM_OK;

void checkComm() {
  unsigned long now = millis();
  if (now - lastModbusOK > MODBUS_WATCHDOG_MS) {
    if (commState != COMM_LOST) {
      commState = COMM_LOST;
      // soft fallback: zmniejsz całkowanie, nie pisz agresywnie
      Ki *= 0.3f;
    }
  } else {
    if (commState != COMM_OK) {
      commState = COMM_OK;
      // restore Ki (można przechowywać oryginalne Ki jeśli chcesz)
      Ki = max(Ki, 0.01f);
    }
  }
}
// ===== KONIEC SEKCJI NR 8 "WATCHDOG I SOFT FALLBACK" =====


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
    unsigned long oldest = file_counter - MAX_FILES;
    char oldname[32];
    sprintf(oldname, "/log_%06lu.csv", oldest);
    SD.remove(oldname);
  }
}

void logToSD() {
  if (!logFile) return;
  if (logFile.size() > MAX_FILE_SIZE) { newLogFile(); if (!logFile) return; }
  float T = dht.readTemperature();
  float H = dht.readHumidity();
  unsigned long ms = millis() - programStart_ms;
  uint16_t freq_out=0, fault=0;
  readFalownikRegs(freq_out, fault); // nie krytyczne jeśli false
  logFile.print(file_counter); logFile.print(',');
  logFile.print(ms); logFile.print(',');
  logFile.print(I_meas,3); logFile.print(',');
  logFile.print((I_max - I_meas),3); logFile.print(','); // I_set approximated
  logFile.print((int)last_output_hz10); logFile.print(',');
  logFile.print(freq_out); logFile.print(',');
  logFile.print((I_max - I_meas),3); logFile.print(',');
  logFile.print(T,2); logFile.print(',');
  logFile.print(H,2); logFile.print(',');
  logFile.println((int)commState);
  logFile.flush();
}
// ===== KONIEC SEKCJI NR 9 "SD LOGGING + ROTACJA PLIKÓW" =====


// ===== Sekcja nr. 10 "NEXTION - WYSYŁANIE" =====
void nextionSend(const char *cmd) {
  NextionSerial.print(cmd);
  NextionSerial.write(0xFF); NextionSerial.write(0xFF); NextionSerial.write(0xFF);
}

void updateNextion() {
  char b[64];
  snprintf(b, sizeof(b), "tI.txt=\"%0.2fA\"", I_meas);
  nextionSend(b);
  float T = dht.readTemperature();
  float H = dht.readHumidity();
  snprintf(b, sizeof(b), "tTemp.txt=\"%0.1f\"", T); nextionSend(b);
  snprintf(b, sizeof(b), "tHum.txt=\"%0.1f\"", H); nextionSend(b);
}
// ===== KONIEC SEKCJI NR 10 "NEXTION - WYSYŁANIE" =====


// ===== Sekcja nr. 11 "SETUP i LOOP" =====
void setup() {
  // piny
  pinMode(PZEM_DE, OUTPUT); digitalWrite(PZEM_DE, LOW);
  pinMode(VFD_DE, OUTPUT); digitalWrite(VFD_DE, LOW);
  pinMode(SD_CS_PIN, OUTPUT);
  // UART init
  PZEMSerial.begin(PZEM_BAUD, SERIAL_8N1, PZEM_RX, PZEM_TX);
  VFDSerial.begin(VFD_BAUD, SERIAL_8N1, VFD_RX, VFD_TX);
  NextionSerial.begin(115200, SERIAL_8N1, NEXTION_TX, NEXTION_RX); // swap pins if needed
  // Modbus setup
  node.begin(VFD_ADDR, VFDSerial);
  node.preTransmission(preTransmissionVFD);
  node.postTransmission(postTransmissionVFD);

  // SD init
  if (!SD.begin(SD_CS_PIN)) {
    // SD nie podłączona - logi nie będą zapisywane
  } else {
    newLogFile();
  }
  dht.begin();
  programStart_ms = millis();
  lastModbusOK = millis();
}

unsigned long lastPid = 0;
unsigned long lastLog = 0;
void loop() {
  unsigned long now = millis();
  if (now - lastIread_ms >= 200) readPZEM();
  checkComm();

  // PID loop
  if (now - lastPid >= PID_TS_MS) {
    lastPid = now;
    uint16_t setVal = computeSetpointValue();
    // write only if comm ok
    if (commState == COMM_OK) {
      writeSetpointModbus(setVal);
    } else {
      // soft fallback: nie piszemy, trzymajemy ostatni setpoint
    }
    updateNextion();
  }

  // logging
  if (now - lastLog >= 1000) {
    lastLog = now;
    logToSD();
  }

  delay(1);
}
// ===== KONIEC SEKCJI NR 11 "SETUP i LOOP" =====

void setupOTA() {
  ArduinoOTA.setHostname("Regulator-PID");
  ArduinoOTA.setPassword("pid123");
  ArduinoOTA.begin();
  Serial.println("OTA gotowe – upload po WiFi!");
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== Regulator PID v1.1 – KamilBol (z OTA) ===");

  // WiFiManager + OTA (nowe!)
  wifiManager.autoConnect("PID-Setup");  // Tworzy AP jeśli brak WiFi
  setupOTA();

  // [Reszta setup z Twojego kodu – piny, UART, Modbus, SD, DHT]
  pinMode(PZEM_DE, OUTPUT); digitalWrite(PZEM_DE, LOW);
  pinMode(VFD_DE, OUTPUT); digitalWrite(VFD_DE, LOW);
  pinMode(SD_CS_PIN, OUTPUT);

  PZEMSerial.begin(PZEM_BAUD, SERIAL_8N1, PZEM_RX, PZEM_TX);
  VFDSerial.begin(VFD_BAUD, SERIAL_8N1, VFD_RX, VFD_TX);
  NextionSerial.begin(115200, SERIAL_8N1, NEXTION_TX, NEXTION_RX);  // Swapped dla S3

  node.begin(VFD_ADDR, VFDSerial);
  node.preTransmission([]() { digitalWrite(VFD_DE, HIGH); delayMicroseconds(50); });
  node.postTransmission([]() { delayMicroseconds(50); digitalWrite(VFD_DE, LOW); });

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
  } else {
    newLogFile();
  }
  dht.begin();
  pzem.setAddress(PZEM_ADDR);  // Fix PZEM
  programStart_ms = millis();
  lastModbusOK = millis();
}

void loop() {
  ArduinoOTA.handle();  // OTA zawsze!

  // [Reszta loop z Twojego kodu – odczyty, PID, logi]
  // ... (wklej sekcje 4-11 bez zmian)
}
