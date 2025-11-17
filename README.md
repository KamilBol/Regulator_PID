# Regulator_PID – KamilBol

Pełny regulator PID na ESP32-S3 dla granulatora  
Sterowanie falownikiem Optidrive E3 przez Modbus RTU  
Odczyt prądu z PZEM-004T, HMI Nextion 3.5", DHT11, logi rotacyjne na SD, OTA po WiFi

**Funkcje:**
- PID z predykcją 4.5 s, anti-windup, rate-limiter 2%/s
- Soft fallback przy utracie komunikacji Modbus
- WiFiManager – pierwsze uruchomienie tworzy AP "PID-Setup"
- OTA po WiFi (po pierwszym flashu kablem)
- Logi CSV z rotacją (max 100 plików po 10 MB)
- Nextion – wyświetlanie i (w przyszłości) ustawianie parametrów

**Szybki start:**
1. VS Code + PlatformIO
2. `git clone https://github.com/KamilBol/Regulator_PID.git`
3. Upload (pierwszy raz kablem)
4. Potem tylko OTA!
