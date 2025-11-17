# Regulator PID Granulator – KamilBol

Zaawansowany regulator PID na ESP32-S3 DevKitC-1 do sterowania falownikiem Optidrive E3 przez Modbus RTU (RS485).
- Odczyt prądu z PZEM-004T V3 (z 1 fazy granulatora).
- HMI: Nextion NX4832F035 (strony: monitor A/Hz/Temp/Hum, ustawienia min/max/Kp/Ki/Kd, logi).
- Logowanie: Rotacyjne CSV na SD (ms_since_start, A, V, Power, T, H – bez daty, łatwe do Excela).
- PID: Z anti-windup, rate limiter, hysteresis, predykcją (4-5s lookahead), fallback soft (bez relay).
- OTA: Upload po WiFi/Bluetooth.
- Fallback: Płynne zwalnianie w widełkach (np. 36-38A), bez blokowania starego analogu (10-17V).

## Szybki Start
1. Zainstaluj VS Code + PlatformIO (code.visualstudio.com + extension PlatformIO).
2. git clone https://github.com/KamilBol/Regulator_PID.git
3. Otwórz folder w VS Code → Build (Ctrl+Alt+B).
4. Pierwszy upload: USB (esptool).
5. Potem: OTA po WiFi (upload_port = IP ESP).

## Struktura
- `src/main.cpp`: Kod sekcjami (PID, Modbus, Nextion, SD).
- `lib/`: Custom libs.
- `docs/`: Instrukcje falownika, schematy (Fritzing).
- `hmi/`: Pliki Nextion .HMI.

## Etapy
- Etap 1: Hardware pinout.
- Etap 2: Odczyty PZEM/DHT.
- Etap 3: PID + Modbus.
- ...
