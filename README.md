# Digital Hourglass - Arduino UNO R4 WiFi

## Wiring

| Module | Module pin | UNO R4 pin |
|---|---|---|
| SSD1306 OLED | SDA / SCL | SDA / SCL header |
| SSD1306 OLED | VCC / GND | 5V / GND |
| GY-87 IMU | SDA / SCL | A4 / A5 |
| GY-87 IMU | VCC_IN / GND | 5V / GND |
| Rotary encoder | CLK / DT / SW | D2 / D3 / D4 |
| Rotary encoder | + / GND | 5V / GND |
| Start button | Leg 1 / Leg 2 | D5 / GND |
| Buzzer driver | 1k resistor to NPN base | D6 |
| Buzzer driver | NPN emitter | GND |
| Buzzer driver | Buzzer + / Buzzer - | 5V / NPN collector |
| MAX7219 #1 | DIN / CS / CLK | D11 / D10 / D13 |
| MAX7219 #1 | VCC / GND | 5V / GND |
| MAX7219 #2 | Input header | MAX7219 #1 output header |

Put a 1N4148 across the buzzer, cathode to 5V.

## Build steps

1. Wire everything as in the table above.
2. Open this folder in VS Code with PlatformIO.
3. Build and upload to the UNO R4 WiFi.
4. Open the serial monitor at 115200 baud.
5. Check for "OLED detected", "MPU6050 detected" and "Digital Hourglass ready".

## Controls

1. Press START to start, pause or resume.
2. Hold START for 5 s (when ready or finished) to enter time setting.
3. Rotate the encoder to change the time in 10 s steps (10 s to 10 min).
4. Press the encoder to save.
5. Shake the device to reset the timer.
