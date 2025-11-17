#include <Arduino.h>
#include <ModbusMaster.h>
#include <PZEM004Tv30.h>
#include <DHT.h>
#include <SdFat.h>
#include <Nextion.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ===== Sekcja nr. 1 "DEFINICJE PINÓW I STAŁYCH" =====
// (tutaj wstawisz swoje piny po przetestowaniu)
#define PIN_PZEM_RX     16
#define PIN_PZEM_TX     17
#define PIN_RS485_DE    18
#define PIN_RS485_RE    19
#define PIN_DHT         4
#define PIN_SD_CS       5
#define PIN_NEXTION_RX  20
#define PIN_NEXTION_TX  21

#define DHT_TYPE        DHT11
#define MODBUS_BAUD     19200
#define PZEM_BAUD       9600
#define NEXTION_BAUD    115200

// PID parametry (będą edytowalne z Nextion)
float Kp = 0.8, Ki = 0.5, Kd = 0.02;
float I_min = 36.0, I_max = 37.8;  // Twoje widełki

// ===== KONIEC SEKCJI NR 1 =====

// ===== Sekcja nr. 2 "OBIEKTY GLOBALNE" =====
PZEM004Tv30 pzem(Serial2, PIN_PZEM_RX, PIN_PZEM_TX);
ModbusMaster modbus;
DHT dht(PIN_DHT, DHT_TYPE);
SdFat SD;
HardwareSerial NextionSerial(1);

// ===== KONIEC SEKCJI NR 2 =====

// ===== Sekcja nr. 3 "OTA I WIFI SETUP" =====
void setupOTA() {
  ArduinoOTA.setHostname("PID-Granulator");
  ArduinoOTA.setPassword("admin123");
  ArduinoOTA.begin();
  Serial.println("OTA gotowe – IP: ");
  Serial.println(WiFi.localIP());
}
// ===== KONIEC SEKCJI NR 3 =====

void setup() {
  Serial.begin(115200);
  Serial.println("=== PID Regulator v0.1 – KamilBol ===");

  // WiFi + OTA
  WiFi.begin("TWOJA_SIEC", "TWOJE_HASLO");  // ← zmień!
  while (WiFi.status() != WL_CONNECTED) delay(500);
  setupOTA();

  // Inicjalizacja reszty – będzie w kolejnych sekcjach
}

void loop() {
  ArduinoOTA.handle();  // <-- to umożliwia OTA zawsze
  delay(10);
}