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

// [Reszta Twojego kodu bez zmian – sekcje 3-11, z dodatkiem OTA w setup()]

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
