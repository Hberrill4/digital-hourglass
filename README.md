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

## Bill of Materials

| # | Component | Part / Model | Qty | Purpose | Connects to |
|---|---|---|---|---|---|
| 1 | Microcontroller board | Arduino UNO R4 WiFi (ABX00087) | 1 | Runs the firmware | USB-C to PC / 5 V supply |
| 2 | LED matrix module | MAX7219 8×8 dot matrix, FC16 type | 2 | The two hourglass chambers (128 LEDs of "sand") | DIN D11, CLK D13, CS D10, 5V, GND; second module daisy-chained from the first |
| 3 | LCD display | 16×2 LCD 1602 with I2C backpack (PCF8574, address 0x27 / 0x3F) | 1 | Shows mode, time, instructions and error messages | SDA A4, SCL A5, 5V, GND |
| 4 | Motion sensor | GY-87 IMU module (MPU6050 accelerometer, address 0x68) | 1 | Detects a shake to reset the timer | SDA A4, SCL A5, VCC_IN 5V, GND |
| 5 | Rotary encoder | KY-040 module (20 detents, with push switch) | 1 | Sets the time (turn) and saves it (press) | CLK D2, DT D3, SW D4, + 5V, GND |
| 6 | Push button | 6×6 mm tactile switch (e.g. Omron B3F-1000) | 1 | START / pause / resume; hold 5 s to set the time | D5 and GND |
| 7 | Buzzer | TMB12A05 active magnetic buzzer, 5 V, 30 mA (equivalent: PUI Audio AI-1223-TWT-5V-R) | 1 | Audio feedback and alarms | + to 5V, − to Q1 collector |


### Software libraries

| Library | Author | Used for |
|---|---|---|
| MD_MAX72XX | majicDesigns | Driving the MAX7219 LED matrices |
| LiquidCrystal I2C | Frank de Brabander (PlatformIO: `marcoschwartz/LiquidCrystal_I2C`) | Driving the I2C LCD 1602 |
| Adafruit MPU6050 | Adafruit | Configuring the motion sensor |
| Adafruit Unified Sensor | Adafruit | Dependency of Adafruit MPU6050 |
| Adafruit BusIO | Adafruit | Dependency of Adafruit MPU6050 |
| Wire, SPI | Arduino (built in) | I2C and SPI communication |

### Notes

- All modules share a **common ground** with the Arduino.
- The I2C bus runs at **100 kHz** because the LCD backpack's PCF8574 chip is rated for that speed.
- Turn the **blue contrast knob** on the back of the LCD if the screen is blank or shows solid blocks.
- PN2222A pinout (flat face towards you, legs down): **E – B – C**.

## Controls

1. Press START to start, pause or resume.
2. Hold START for 5 s (when ready or finished) to enter time setting.
3. Rotate the encoder to change the time in 10 s steps (10 s to 10 min).
4. Press the encoder to save.
5. Shake the device to reset the timer.
## Display guide

### OLED layout

The 128×64 OLED is split into three areas:

| Area | Position | Shows |
|---|---|---|
| Mode bar | Top strip (white) | `MODE: <current mode>`, plus `FAULT` on the right if a hardware fault is active |
| Time | Middle (large) | Time remaining while running/paused, otherwise the selected duration (`MM:SS`) |
| Instructions | Bottom three lines | What to do next in the current mode, or a temporary message |

### Modes and on-screen instructions

| Mode | Line 1 | Line 2 | Line 3 |
|---|---|---|---|
| `READY` | START: begin timer | Hold START 5s: set | Shake: reset |
| `RUNNING` | START: pause | Shake: reset | Total: 01:00 |
| `PAUSED` | START: resume | Shake: reset | Total: 01:00 |
| `SET TIME` | Turn knob: +/-10s | Press knob: save | Range 00:10 - 10:00 |
| `FINISHED` | TIME'S UP! | START: run again | Hold START 5s: set |

While START is being held towards a 5-second action, the hint changes to a live countdown: **Keep holding: 4s… 3s… 2s… 1s**.

If no IMU is detected, the shake hints change to **Shake: off (no IMU)** and **Hold START 5s: reset**. Holding START for 5 seconds then resets a running or paused timer instead.

### Sounds

| Sound | Meaning |
|---|---|
| 2 short beeps | Timer started or resumed |
| 1 medium beep | Timer paused |
| 1 short beep | 10-second mark reached (50, 40, 30, 20, 10) |
| 3 long beeps | Time's up |
| 2 medium beeps | Entered SET TIME mode |
| 1 long beep (300 ms) | Time saved, left SET TIME |
| N short beeps | New time selected: one beep per 10 seconds (e.g. 6 beeps = 01:00) |
| 2 slow beeps | Timer reset by shaking |
| **3 fast chirps** | **Action not allowed, or a fault was detected** |
| **1 very long beep (800 ms)** | **OLED missing or disconnected** |

### User messages

These replace the bottom
