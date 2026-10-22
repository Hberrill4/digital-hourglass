# Digital Hourglass – Arduino UNO R4 WiFi

A digital hourglass: two 8×8 LED matrices act as the glass, with "sand" falling from the top chamber to the bottom as the time runs down. Set the time with a potentiometer or an IR remote, pick one of six food timers, start it with a button or the remote, and shake it to reset.

**New here?** Read the [instruction manual](MANUAL.md) first.

## Features

- Two MAX7219 8×8 LED matrices show the sand falling (128 LEDs)
- 0.96" OLED shows the mode, the time and what to do next
- Potentiometer sets the time from 0:10 to 10:00 in 10 s steps
- IR remote: start / pause, type an exact time, ±10 s, ±1 min, food timers, reset
- Six food timers: the food's name is beeped in Morse code, and the food itself becomes the sand on the matrices
- Shake to reset (GY-87 motion sensor)
- Buzzer feedback for every action, the 10-second marks and the alarm
- Power-up self-test, on-screen error messages, automatic reconnection of the OLED and motion sensor

---

## Pinout

| UNO R4 pin | Connected to | Signal |
|---|---|---|
| **5V** | 5 V breadboard rail | Power for every module |
| **GND** | GND breadboard rail | Common ground for every module |
| **A0** | Potentiometer middle leg (wiper) | Analog 0–5 V, time setting |
| **SDA / SCL header** | OLED SDA / SCL | I2C bus, 400 kHz |
| **A4 / A5** | GY-87 SDA / SCL | Same I2C bus as the SDA / SCL header |
| **D5** | START button (other leg to GND) | Digital input, internal pull-up |
| **D6** | 1 kΩ resistor → NPN transistor base | Buzzer on/off |
| **D8** | IR receiver OUT | Remote control signal |
| **D10** | MAX7219 #1 CS | SPI chip select |
| **D11** | MAX7219 #1 DIN | SPI data (MOSI) |
| **D13** | MAX7219 #1 CLK | SPI clock |

Unused: D0–D4, D7, D9, D12, A1–A3.

### Module pinouts

| Module | Pin | Connects to |
|---|---|---|
| SSD1306 OLED | GND / VCC / SCL / SDA | GND / 5V / SCL header / SDA header |
| GY-87 IMU | VCC_IN / GND / SCL / SDA | 5V / GND / A5 / A4 (3V3, FSYNC, INTA, DRDY unused) |
| Potentiometer | Left leg / middle leg / right leg | GND / A0 / 5V |
| IR receiver (VS1838B) | OUT / GND / VCC | D8 / GND / 5V |
| MAX7219 #1 (IN header) | VCC / GND / DIN / CS / CLK | 5V / GND / D11 / D10 / D13 |
| MAX7219 #2 (IN header) | VCC / GND / DIN / CS / CLK | MAX7219 #1 OUT header, pin for pin |
| START button | Leg 1 / Leg 2 | D5 / GND |
| Buzzer driver | See construction step 9 | D6, 5V, GND |

---

## Bill of Materials

| # | Component | Part / Model | Qty | Purpose |
|---|---|---|---|---|
| 1 | Microcontroller board | Arduino UNO R4 WiFi (ABX00087) | 1 | Runs the firmware |
| 2 | LED matrix module | MAX7219 8×8 dot matrix, FC16 type | 2 | The two hourglass chambers |
| 3 | OLED display | 0.96" SSD1306 128×64 I2C (address 0x3C / 0x3D) | 1 | Mode, time, instructions, messages |
| 4 | Motion sensor | GY-87 IMU module (MPU6050, address 0x68) | 1 | Shake to reset |
| 5 | Potentiometer | 10 kΩ linear rotary (B10K) with knob | 1 | Sets the time |
| 6 | IR receiver | VS1838B receiver module (38 kHz) | 1 | Receives the remote |
| 7 | IR remote | 21-key "Car MP3" NEC remote (CR2025 cell) | 1 | Full remote control |
| 8 | Push button | 6×6 mm tactile switch (e.g. Omron B3F-1000) | 1 | START / pause / resume / hold to reset |
| 9 | Buzzer | TMB12A05 active magnetic buzzer, 5 V, 30 mA | 1 | Audio feedback and Morse code |
| 10 | NPN transistor | PN2222A / 2N2222A, TO-92 | 1 | Switches the buzzer |
| 11 | Resistor | 1 kΩ, ¼ W | 1 | Transistor base resistor |
| 12 | Diode | 1N4148 | 1 | Flyback protection for the buzzer |
| 13 | Capacitor | 100 µF electrolytic, ≥ 10 V | 1 | Smooths the 5 V supply |
| 14 | Breadboard | Full size, 830 tie-points | 1 | Prototyping and power rails |
| 15 | Jumper wires | Male-male and male-female Dupont | ~30 | Connections |
| 16 | USB cable | USB-C data cable | 1 | Power, programming, Serial Monitor |

### Software libraries

| Library | PlatformIO name | Used for |
|---|---|---|
| MD_MAX72XX | `majicdesigns/MD_MAX72XX` | LED matrices |
| Adafruit SSD1306 + GFX | `adafruit/Adafruit SSD1306`, `adafruit/Adafruit GFX Library` | OLED |
| Adafruit MPU6050 | `adafruit/Adafruit MPU6050` (+ Unified Sensor, BusIO) | Motion sensor set-up |
| IRremote 4.7.1 | `z3t0/IRremote@4.7.1` | IR remote decoding |
| Wire, SPI | built in | I2C and SPI |

---

## Construction

Unplug the USB cable while wiring. Use the labels printed on each module for its pins.

1. **Power rails.** Run a wire from the Arduino **5V** pin to the breadboard's red (+) rail, and from **GND** to the blue (−) rail. Every module below takes its power from these rails.
2. **Supply capacitor.** Plug the 100 µF capacitor across the two rails near where the matrices will connect. The longer leg (+) goes to 5V; the leg next to the stripe on the case goes to GND.
3. **LED matrix #1.** Connect its **IN** header (the end marked DIN): VCC → 5V rail, GND → GND rail, DIN → **D11**, CS → **D10**, CLK → **D13**.
4. **LED matrix #2.** Connect its IN header to matrix #1's **OUT** header pin for pin: VCC→VCC, GND→GND, DOUT→DIN, CS→CS, CLK→CLK. Place it so matrix #2 sits above matrix #1, as the upper chamber.
5. **OLED.** GND → GND rail, VCC → 5V rail, SCL → **SCL** header pin, SDA → **SDA** header pin (the two pins next to AREF).
6. **GY-87 motion sensor.** VCC_IN → 5V rail, GND → GND rail, SCL → **A5**, SDA → **A4**. Leave 3V3, FSYNC, INTA and DRDY unconnected.
7. **Potentiometer.** With the knob facing you and the legs pointing down: left leg → GND rail, middle leg → **A0**, right leg → 5V rail.
8. **IR receiver.** OUT (sometimes marked S) → **D8**, VCC (+) → 5V rail, GND (−) → GND rail. Mount it where the remote can see it.
9. **Buzzer driver.**
   1. Place the PN2222A with its flat face towards you and the legs pointing down. From left to right the legs are **E, B, C**.
   2. Emitter (E) → GND rail.
   3. Base (B) → one end of the 1 kΩ resistor; the other end of the resistor → **D6**.
   4. Collector (C) → buzzer **−** leg.
   5. Buzzer **+** leg (the longer leg, marked + on top) → 5V rail.
   6. 1N4148 across the buzzer's two legs, with the **striped end on the + / 5V side**.
10. **START button.** Place it across the breadboard's centre gap. Connect one leg to **D5** and the diagonally opposite leg to the GND rail.
11. **Check.** Before plugging in, check every 5V and GND connection again. A 5V-to-GND short is the only wiring mistake that can damage the board.
12. **Upload.** Plug in the USB cable, open the project in VS Code with PlatformIO, click **✓ Build**, then **→ Upload**.
13. **Self-test.** Open the Serial Monitor at 115200 baud. All 128 LEDs light, the OLED shows `SELF TEST: PASS`, and the buzzer gives one short beep.
14. **Function test.**
    1. Turn the pot: the time changes in 10 s steps with a click.
    2. Press START: the sand starts falling.
    3. Press each remote key: the Serial Monitor prints a line such as `[IR] address 0x0  command 0x43`.
    4. Press CH: a food picture appears and its name is beeped in Morse.
    5. Shake the device: the timer resets.

If turning the pot clockwise makes the time go **down**, swap the two outer pot wires. If the food pictures appear mirrored or upside down, the matrices are mounted the other way round from the firmware's assumption.

---

## Controls

| Action | Button / pot | Remote (21-key) |
|---|---|---|
| Set the time | Turn the pot (READY or FINISHED) | Digits then ▶‖, or + / − / ⏮ / ⏭ |
| Pick a food timer | – | CH |
| Start / pause / resume | Press START | ▶‖ |
| Reset | Hold START 5 s, or shake | EQ |
| Cancel typing | – | 100+ |

### Remote key map (21-key "Car MP3" remote)

| Key | Action |
|---|---|
| ▶‖ | Start / pause / resume, or confirm a typed time |
| 0–9 | Type a time: digits are read as MM:SS (1, 3, 0 then ▶‖ = 1:30) |
| + / − | ±10 s (hold to scroll) |
| ⏭ / ⏮ | ±1 minute |
| CH | Next food timer |
| EQ | Reset |
| 100+ | Cancel typing |

A 17-key arrow remote is also supported. Set `REMOTE_LAYOUT` to `RemoteLayout::Arrows_17Key` in `main.cpp`: OK = start, ↑/↓ = ±10 s, ←/→ = ±1 min, `#` = reset, `*` = cancel. Any other NEC remote can be mapped using the codes printed in the Serial Monitor.

### How the remote is received

The remote flashes its IR LED at 38 kHz in the NEC pattern for the key pressed. The VS1838B filters out room light and outputs clean pulses on D8. The IRremote library samples D8 in a background timer interrupt and decodes the key. Holding a key sends repeat codes, which only + and − act on (every 200 ms).

### How the pot and remote share the time

- At power-up the pot position sets the time.
- After the remote sets a time, the pot is ignored until it is turned. Then it takes over again, so the time never jumps by itself.
- While the timer is running or paused, turning the pot shows `! Pot locked`.

---

## Food timers

Press **CH** on the remote to step through six food timers. The times are typical cooking times and all fit within the 10-minute limit.

| CH press | Food | Time | Morse word | Matrix picture |
|---|---|---|---|---|
| 1st | Soft-boiled egg (from boiling) | 6:00 | EGG `. --. --.` | Egg |
| 2nd | Spaghetti (dried) | 10:00 | PASTA `.--. .- ... - .-` | Spaghetti nest |
| 3rd | Instant noodles | 3:00 | NOODLES | Noodle cup with chopsticks |
| 4th | Microwave popcorn | 2:30 | POPCORN | Popcorn bucket |
| 5th | Steamed broccoli | 4:00 | BROCCOLI | Broccoli floret |
| 6th | Corn on the cob (boiled) | 5:00 | CORN `-.-. --- .-. -.` | Corn cob |

When a food is picked:

1. **Morse code.** The buzzer spells the food's name in Morse at about 20 words per minute (EGG 1.5 s, BROCCOLI 5.6 s). It plays in the background, so the controls keep working.
2. **Splash screen.** For 2.5 s the upper matrix shows the food and the lower matrix shows an hourglass.
3. **Food as sand.** While the timer runs, the food drains from the top of the upper chamber and rebuilds from the bottom of the lower chamber. At 0:00 the whole food is in the bottom chamber.
4. **Finish.** The OLED shows `<food> is ready!`, and the three long alarm beeps are followed by the food's name in Morse.

Turning the pot or setting a time with the digits or + / − switches back to normal sand. The foods, times and 8×8 pictures are defined in the `FOODS[]` table in `main.cpp`, so they're easy to change.

---

## Display guide

### OLED layout

| Area | Position | Shows |
|---|---|---|
| Mode bar | Top strip (white) | `MODE: <mode>`, plus `FAULT` if a hardware fault is active |
| Time | Middle (large) | Time remaining while running/paused, otherwise the selected time (`MM:SS`) |
| Instructions | Bottom three lines | What to do next, or a temporary message |

### Modes and on-screen instructions

| Mode | Line 1 | Line 2 | Line 3 |
|---|---|---|---|
| `READY` | START/OK: begin | Turn pot: set time | Remote: 0-9 then OK |
| `RUNNING` | START/OK: pause | Shake / EQ: reset | Total 01:00 |
| `PAUSED` | START/OK: resume | Shake / EQ: reset | Total 01:00 |
| `FINISHED` | TIME'S UP! | START/OK: run again | Pot/remote: new time |
| `TYPE TIME` | OK: set this time | Keep typing digits | 100+: cancel |

With a food timer selected:

- READY line 3 shows the food, e.g. **Food: Soft egg**.
- RUNNING and PAUSED line 3 shows the food and its time, e.g. **Soft egg 06:00**.
- FINISHED line 1 reads **Soft egg is ready!**

Other variations:

- After the remote sets a time, line 2 in READY reads **Turn pot to adjust**.
- While START is held during RUNNING, PAUSED or FINISHED, the lines show **Resetting...** and a live **Keep holding: 4s… 3s…** countdown.
- With no motion sensor, the reset hint reads **Hold START/EQ: reset**.

### Sounds

| Sound | Meaning |
|---|---|
| Tiny click | Pot step, remote key accepted, or time set |
| Morse code | Name of the food timer just picked, or that it's ready |
| 2 short beeps | Timer started or resumed |
| 1 medium beep | Timer paused |
| 1 short beep | 10-second mark reached (50, 40, 30, 20, 10) |
| 3 long beeps | Time's up (followed by Morse if a food timer was used) |
| 2 medium beeps | Timer reset (START hold or remote) |
| 2 slow beeps | Timer reset by shaking |
| **3 fast chirps** | **Action not allowed, or a fault was detected** |
| **1 very long beep** | **OLED missing or disconnected** |

### Messages

| Message | Cause | What to do |
|---|---|---|
| `Food: <name>` / `Morse: <WORD>` | CH pressed | Press CH again for the next food |
| `! Can't change now` | Remote time keys pressed while running or paused | Pause and reset first |
| `! Pot locked` | Pot turned while running or paused | Reset the timer, then turn the pot |
| `! Invalid time` | Typed seconds above 59 (e.g. 1:75) | Type the time again |
| `! Out of range` | Time below 0:10 or above 10:00 | Choose a time in range |
| `Time set: MM:SS` | Remote set a new time | – |
| `Typing cancelled` | 100+ pressed, or no key for 6 s | – |
| `IR key 0xNN is not mapped` | A remote key with no action | See the code in the Serial Monitor |
| `Reset by remote` / `Timer reset` / `Timer reset (shake)` | Timer was reset | START/OK to begin |
| `! START button stuck` | START held 20 s or at power-up | Release it, or check D5 wiring |
| `! IMU disconnected` | Motion sensor stopped responding | Check GY-87 wiring; reconnects automatically |
| `! IMU wrong chip` | Clone chip at 0x68 that is not an MPU6050 | Replace the GY-87 |
| `IMU connected` | Motion sensor reconnected | – |

### Power-up self-test

For 2 s (pass) or 5 s (fault), all 128 LEDs light and the OLED shows:

```
SELF TEST: PASS
OLED   OK  (0x3C)
IMU    OK  (0x68)
START  OK
POT    01:00
IR D8 ready. LEDs on?
```

The same report, plus every message and remote code, is printed to the Serial Monitor at **115200 baud**.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Matrices dark, only the small "L" LED flickers | Matrices have no power or data | Check 5V/GND and that wires go to the **IN** header |
| Only one matrix lights | Broken chain | Check matrix #1 OUT → matrix #2 IN |
| Food pictures mirrored or upside down | Matrices mounted differently | Rotate the modules, or adjust `setChamberPixel()` in `main.cpp` |
| Time jumps around when the pot is still | Loose wiper wire | Re-seat the A0 wire; add 100 nF from A0 to GND |
| Pot works backwards | Outer legs swapped | Swap the 5V and GND pot wires |
| Remote does nothing | Receiver wiring, flat battery, or out of sight | Check D8/5V/GND, replace the CR2025, point at the receiver |
| Remote keys do the wrong thing | Different remote codes | Read the codes in the Serial Monitor and update the key table |
| Morse sounds like one long beep | Buzzer is a passive type | Use an active buzzer such as the TMB12A05 |
| Frequent `! IMU disconnected` | Long or loose I2C wires | Keep I2C wires short and firmly seated |
| No beeps | Transistor reversed or missing | Check the E-B-C order and the buzzer polarity |
| Board resets when the alarm sounds | USB supply sagging | Check the 100 µF capacitor is fitted |
