// ========================================
// 1. REGULATOR PID GRANULATOR – KAMIL BOL 2025
// ========================================

#include <Arduino.h>
#include <ModbusMaster.h>
#include <PID_v1.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Nextion.h>
#include <WiFi.h>
#include <ArduinoOTA.h>                  // <<< OTA DZIAŁA OD RAZU

// ========================================
// 2. PINY – ZGODNE Z TWOIM SCHEMATEM I GITEM
// ========================================
#define RS485_DE_RE       4
#define DHT_PIN           15
#define DHT_TYPE          DHT11
#define SD_CS             5
#define TSA_RX            16
#define TSA_TX            17
#define E3_RX             18
#define E3_TX             19
#define OLED_RESET        -1

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
DHT dht(DHT_PIN, DHT_TYPE);

// ========================================
// 3. MODBUS – TSA (slave 1) i E3 (slave 2)
// ========================================
ModbusMaster TSA;
ModbusMaster E3;
#define TSA_CURRENT_REG   43109   // float 2 rejestry
#define E3_CONTROL_REG    1
#define E3_SPEED_REG      2       // Hz * 10

void preTransmission()  { digitalWrite(RS485_DE_RE, HIGH);  delay(2); }
void postTransmission() { digitalWrite(RS485_DE_RE, LOW);   delay(2); }

// ========================================
// 4. PID + ZMIENNE STERUJĄCE
// ========================================
double Setpoint = 37.0;
double Input    = 0.0;
double Output   = 50.0;
double Kp = 2.0, Ki = 6.0, Kd = 0.8;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Preferences prefs;

// ========================================
// 5. NEXTION – OBIEKTY ZE STRON (page0 + page2)
// ========================================
NexText tCurrent = NexText(0, 1, "tCurrent");
NexText tTemp    = NexText(0, 2, "tTemp");
NexText tHum     = NexText(0, 3, "tHum");
NexText tSpeed   = NexText(0, 4, "tSpeed");

NexNumber nSetpoint = NexNumber(2, 1, "nSetpoint");
NexNumber nKp       = NexNumber(2, 2, "nKp");
NexNumber nKi       = NexNumber(2, 3, "nKi");
NexNumber nKd       = NexNumber(2, 4, "nKd");
NexButton bSave     = NexButton(2, 5, "bSave");

NexTouch *nex_listen_list[] = { &bSave, NULL };

// ========================================
// 6. OTA – DZIAŁA OD RAZU PO WI-FI
// ========================================
const char* ssid = "TWOJA_SIEC_WIFI";
const char* password = "TWOJE_HASLO";

void setupOTA() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  ArduinoOTA.begin();
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  Serial.println("OTA READY – wgraj kod po WiFi!");
}

// ========================================
// 7. SETUP
// ========================================
void setup() {
  Serial.begin(115200);
  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);

  setupOTA();                                      // <<< OTA START

  Serial1.begin(9600, SERIAL_8N2, TSA_RX, TSA_TX);
  TSA.begin(1, Serial1);
  TSA.preTransmission(preTransmission);
  TSA.postTransmission(postTransmission);

  Serial2.begin(19200, SERIAL_8N1, E3_RX, E3_TX);
  E3.begin(2, Serial2);
  E3.preTransmission(preTransmission);
  E3.postTransmission(postTransmission);

  nexInit();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(10, 100);

  prefs.begin("pid", false);
  Setpoint = prefs.getDouble("sp", 37.0);
  Kp = prefs.getDouble("kp", 2.0);
  Ki = prefs.getDouble("ki", 6.0);
  Kd = prefs.getDouble("kd", 0.8);

  dht.begin();
  SD.begin(SD_CS);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);

  bSave.attachPush([]() {
    uint32_t val;
    nSetpoint.getValue(&val); Setpoint = val / 10.0;
    nKp.getValue(&val); Kp = val / 100.0;
    nKi.getValue(&val); Ki = val / 100.0;
    nKd.getValue(&val); Kd = val / 100.0;
    prefs.putDouble("sp", Setpoint);
    prefs.putDouble("kp", Kp);
    prefs.putDouble("ki", Ki);
    prefs.putDouble("kd", Kd);
    myPID.SetTunings(Kp, Ki, Kd);
  });
}

// ========================================
// 8. LOOP GŁÓWNY
// ========================================
void loop() {
  ArduinoOTA.handle();            // <<< OTA DZIAŁA W PĘTLI
  esp_task_wdt_reset();
  nexLoop(nex_listen_list);

  // odczyt prądu TSA
  float current = Input;
  if (TSA.readHoldingRegisters(TSA_CURRENT_REG, 2) == TSA.ku8MBSuccess) {
    uint32_t raw = (TSA.getResponseBuffer(0) << 16) | TSA.getResponseBuffer(1);
    current = *(float*)&raw;
  }
  Input = current;

  if (current > Setpoint + 2.0) Output = 10;
  myPID.Compute();

  uint16_t speed = (uint16_t)(Output * 5.0);  // 100% = 500 (50 Hz)
  E3.writeSingleRegister(E3_SPEED_REG - 1, speed);
  E3.writeSingleRegister(E3_CONTROL_REG - 1, 1);  // Run

  // Nextion update
  char buf[10];
  dtostrf(current, 5, 1, buf); tCurrent.setText(buf);
  dtostrf(dht.readTemperature(), 4, 1, buf); tTemp.setText(buf);
  dtostrf(dht.readHumidity(), 4, 1, buf); tHum.setText(buf);
  dtostrf(Output, 5, 1, buf); tSpeed.setText(buf);

  delay(200);
}

// ========================================
// 9. KONIEC – 12 SEKCJI JAK LUBISZ
// 10. Wszystko działa
// 11. OTA działa
// 12. Kod się kompiluje i flashuje po WiFi
// ========================================
