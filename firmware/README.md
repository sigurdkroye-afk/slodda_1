# ESP32 Firmware — TPU Arm

## Fil
`servo_tid_test66.ino` — tidsbasert servokontroll med Pi UART-kommunikasjon.

## Flash-instruksjon
1. Koble fra Pi TX/RX-kabler fra ESP32
2. Åpne `servo_tid_test66.ino` i Arduino IDE
3. Board: **ESP32 Dev Module** — Port: den som dukker opp (USB)
4. Last opp → `Done uploading`
5. Koble Pi-kabler tilbake

## Pin-mapping

| Funksjon | ESP32 GPIO |
|----------|-----------|
| Servo 1 (venstre vaier) | GPIO 27 |
| Servo 2 (topp vaier) | GPIO 26 |
| Servo 3 (høyre vaier) | GPIO 25 |
| IR-sensor venstre | GPIO 35 |
| UART2 RX (fra Pi TX) | GPIO 13 |
| UART2 TX (til Pi RX) | GPIO 17 |

Serial Monitor: 115200 baud (output-only, kommandoer kun via Pi).
