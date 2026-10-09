#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <MD_MAX72xx.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

// ============================================================
// DIGITAL HOURGLASS - ARDUINO UNO R4 WIFI
// Hardware:
// - 2 daisy-chained MAX7219 8x8 matrices (FC16 modules)
// - Rotary encoder with push switch (KY-040 style)
// - Separate start/pause button
// - Active buzzer driven through an NPN transistor
// - GY-87 IMU (MPU6050 accelerometer/gyroscope)
// - Optional SSD1306 128x64 I2C OLED
//
// Changes from v1:
// - Encoder counted by interrupt (no missed detents during I2C)
// - MAX7219 auto-update disabled; one SPI flush per frame
// - I2C at 400 kHz (held there for the OLED and the IMU)
// - OLED text positions fit inside 64 px
// - 10-second beeps fire at 50, 40, 30 ... instead of 59, 49 ...
// - Pause/resume keeps the partial second
// ============================================================

// ---------------- PIN ASSIGNMENTS ----------------

constexpr uint8_t MATRIX_CS_PIN = 10;   // DIN = D11 (MOSI), CLK = D13 (SCK)

constexpr uint8_t ENCODER_A_PIN = 2;    // KY-040 CLK (interrupt-capable)
constexpr uint8_t ENCODER_B_PIN = 3;    // KY-040 DT
constexpr uint8_t ENCODER_SW_PIN = 4;   // KY-040 SW

constexpr uint8_t START_BUTTON_PIN = 5; // button to GND
constexpr uint8_t BUZZER_PIN = 6;       // to NPN base via 1k

// I2C: SDA = A4 / SDA header, SCL = A5 / SCL header

// ---------------- MATRIX ----------------

constexpr uint8_t MATRIX_COUNT = 2;
constexpr uint8_t MATRIX_BRIGHTNESS = 3;

MD_MAX72XX matrix(MD_MAX72XX::FC16_HW, MATRIX_CS_PIN, MATRIX_COUNT);

// Global matrix coordinates: row 0-7, column 0-15.
// Columns 8-15 = upper chamber, columns 0-7 = lower chamber.

// ---------------- OLED ----------------

constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr int8_t OLED_RESET_PIN = -1;
constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr uint32_t OLED_REFRESH_MS = 150;

constexpr uint32_t I2C_CLOCK_HZ = 400000UL;

// The last two arguments stop the library dropping the bus back to
// 100 kHz after every display update.
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN,
                      I2C_CLOCK_HZ, I2C_CLOCK_HZ);
bool oledAvailable = false;

// ---------------- GY-87 IMU ----------------

Adafruit_MPU6050 imu;
bool imuAvailable = false;

// ---------------- TIMER SETTINGS ----------------

constexpr uint32_t MIN_TIME_SECONDS = 10;
constexpr uint32_t MAX_TIME_SECONDS = 600;
constexpr uint32_t TIME_STEP_SECONDS = 10;
constexpr uint32_t DEFAULT_TIME_SECONDS = 60;

constexpr uint32_t SETTING_HOLD_MS = 5000;
constexpr uint32_t DEBOUNCE_MS = 35;
constexpr uint32_t ENCODER_DEBOUNCE_US = 1500;

// ---------------- SHAKE DETECTION ----------------

constexpr float SHAKE_THRESHOLD = 7.0f;       // m/s^2 change between reads
constexpr uint32_t SHAKE_CONFIRM_MS = 450;
constexpr uint32_t SHAKE_COOLDOWN_MS = 1800;

float previousAccelerationMagnitude = 9.81f;
uint32_t shakeCandidateAt = 0;
uint32_t lastShakeAt = 0;

// ---------------- TIMER STATE ----------------

enum class TimerState : uint8_t { Ready, Running, Paused, Setting, Finished };

TimerState timerState = TimerState::Ready;

uint32_t selectedSeconds = DEFAULT_TIME_SECONDS;
uint32_t remainingSeconds = DEFAULT_TIME_SECONDS;

uint32_t lastTimerTickMs = 0;
uint32_t pausedRemainderMs = 0;
uint32_t lastDisplayUpdateMs = 0;

// ---------------- BUTTON STATE ----------------

bool startButtonDown = false;
bool startHoldHandled = false;
uint32_t startPressedAt = 0;
uint32_t lastStartEdge = 0;

bool encoderSwitchDown = false;
uint32_t lastEncoderSwitchEdge = 0;

// ---------------- ENCODER STATE ----------------

volatile int32_t encoderPosition = 0;
volatile uint32_t lastEncoderEdgeUs = 0;
int32_t handledEncoderPosition = 0;

// ============================================================
// ENCODER INTERRUPT
// ============================================================

void encoderISR() {
    uint32_t now = micros();

    if (now - lastEncoderEdgeUs < ENCODER_DEBOUNCE_US) {
        return;
    }

    lastEncoderEdgeUs = now;

    // Called on the falling edge of A. B high = clockwise.
    if (digitalRead(ENCODER_B_PIN) == HIGH) {
        encoderPosition++;
    } else {
        encoderPosition--;
    }
}

int32_t readEncoderPosition() {
    noInterrupts();
    int32_t position = encoderPosition;
    interrupts();
    return position;
}

// ============================================================
// NON-BLOCKING BUZZER
// ============================================================

struct Beeper {
    uint16_t totalBeeps = 0;
    uint16_t completedBeeps = 0;
    uint16_t onTimeMs = 65;
    uint16_t offTimeMs = 100;
    uint32_t nextChangeMs = 0;
    bool active = false;
    bool sounding = false;
};

Beeper beeper;

void stopBeeper() {
    beeper.active = false;
    beeper.sounding = false;
    digitalWrite(BUZZER_PIN, LOW);
}

void beep(uint16_t count, uint16_t onMs, uint16_t offMs = 100) {
    stopBeeper();

    if (count == 0) {
        return;
    }

    beeper.totalBeeps = count;
    beeper.completedBeeps = 0;
    beeper.onTimeMs = onMs;
    beeper.offTimeMs = offMs;
    beeper.active = true;
    beeper.sounding = true;

    digitalWrite(BUZZER_PIN, HIGH);
    beeper.nextChangeMs = millis() + onMs;
}

void updateBeeper() {
    if (!beeper.active) {
        return;
    }

    if (static_cast<int32_t>(millis() - beeper.nextChangeMs) < 0) {
        return;
    }

    if (beeper.sounding) {
        digitalWrite(BUZZER_PIN, LOW);
        beeper.sounding = false;
        beeper.completedBeeps++;

        if (beeper.completedBeeps >= beeper.totalBeeps) {
            beeper.active = false;
            return;
        }

        beeper.nextChangeMs = millis() + beeper.offTimeMs;
    } else {
        digitalWrite(BUZZER_PIN, HIGH);
        beeper.sounding = true;
        beeper.nextChangeMs = millis() + beeper.onTimeMs;
    }
}

// ============================================================
// MAX7219 DISPLAY
// ============================================================

void setPixel(uint8_t row, uint16_t column, bool on) {
    if (row >= 8 || column >= 16) {
        return;
    }

    matrix.setPoint(row, column, on);
}

void drawHourglass(uint32_t secondsLeft) {
    matrix.clear();

    if (selectedSeconds == 0) {
        matrix.update();
        return;
    }

    if (secondsLeft > selectedSeconds) {
        secondsLeft = selectedSeconds;
    }

    uint32_t elapsed = selectedSeconds - secondsLeft;

    uint8_t sandMoved = static_cast<uint8_t>((elapsed * 64UL) / selectedSeconds);
    uint8_t topSand = 64 - sandMoved;
    uint8_t bottomSand = sandMoved;

    for (uint8_t row = 0; row < 8; row++) {
        for (uint8_t col = 0; col < 8; col++) {
            // Rank 0 = bottom-left pixel, rank 63 = top-right pixel.
            uint8_t rank = (7 - row) * 8 + col;

            setPixel(row, col + 8, rank < topSand);    // upper chamber
            setPixel(row, col, rank < bottomSand);     // lower chamber
        }
    }

    // One SPI flush for the whole frame.
    matrix.update();
}

// ============================================================
// OLED DISPLAY
// ============================================================

const char* stateName() {
    switch (timerState) {
        case TimerState::Ready:    return "READY";
        case TimerState::Running:  return "RUNNING";
        case TimerState::Paused:   return "PAUSED";
        case TimerState::Setting:  return "SET TIME";
        case TimerState::Finished: return "FINISHED";
    }

    return "";
}

void formatTime(uint32_t seconds, char* buffer, size_t length) {
    snprintf(buffer, length, "%02lu:%02lu",
             static_cast<unsigned long>(seconds / 60),
             static_cast<unsigned long>(seconds % 60));
}

void updateOLED(bool force = false) {
    if (!oledAvailable) {
        return;
    }

    uint32_t now = millis();

    if (!force && now - lastDisplayUpdateMs < OLED_REFRESH_MS) {
        return;
    }

    lastDisplayUpdateMs = now;

    uint32_t displaySeconds = selectedSeconds;

    if (timerState == TimerState::Running || timerState == TimerState::Paused) {
        displaySeconds = remainingSeconds;
    }

    char timeText[8];
    formatTime(displaySeconds, timeText, sizeof(timeText));

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextWrap(false);

    // Header: y 0-7
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(F("DIGITAL HOURGLASS"));

    // Time: y 18-41
    oled.setTextSize(3);
    oled.setCursor(14, 18);
    oled.print(timeText);

    // State: y 45-52
    oled.setTextSize(1);
    oled.setCursor(0, 45);
    oled.print(stateName());

    // Hint: y 55-62
    oled.setCursor(0, 55);

    switch (timerState) {
        case TimerState::Setting:
            oled.print(F("Rotate, press to save"));
            break;
        case TimerState::Paused:
            oled.print(F("START: resume"));
            break;
        case TimerState::Running:
            oled.print(F("START: pause"));
            break;
        default:
            oled.print(F("Hold START 5s: set"));
            break;
    }

    oled.display();
}

// ============================================================
// TIMER CONTROLS
// ============================================================

void resetTimer(bool playSound) {
    timerState = TimerState::Ready;
    remainingSeconds = selectedSeconds;
    lastTimerTickMs = millis();
    pausedRemainderMs = 0;

    stopBeeper();
    drawHourglass(remainingSeconds);

    if (playSound) {
        beep(2, 100, 100);
    }

    updateOLED(true);
}

void startTimer() {
    remainingSeconds = selectedSeconds;
    timerState = TimerState::Running;
    lastTimerTickMs = millis();
    pausedRemainderMs = 0;

    beep(2, 65, 100);

    drawHourglass(remainingSeconds);
    updateOLED(true);
}

void pauseTimer() {
    timerState = TimerState::Paused;

    // Keep the part of the current second already elapsed.
    pausedRemainderMs = millis() - lastTimerTickMs;

    beep(1, 180);
    updateOLED(true);
}

void resumeTimer() {
    timerState = TimerState::Running;
    lastTimerTickMs = millis() - pausedRemainderMs;
    pausedRemainderMs = 0;

    beep(2, 65, 100);
    updateOLED(true);
}

void finishTimer() {
    timerState = TimerState::Finished;
    remainingSeconds = 0;

    drawHourglass(0);
    beep(3, 450, 180);
    updateOLED(true);
}

void handleStartPress() {
    switch (timerState) {
        case TimerState::Ready:
        case TimerState::Finished:
            startTimer();
            break;
        case TimerState::Running:
            pauseTimer();
            break;
        case TimerState::Paused:
            resumeTimer();
            break;
        case TimerState::Setting:
            break;
    }
}

void updateTimer() {
    if (timerState != TimerState::Running) {
        return;
    }

    uint32_t elapsedSeconds = (millis() - lastTimerTickMs) / 1000UL;

    if (elapsedSeconds == 0) {
        return;
    }

    lastTimerTickMs += elapsedSeconds * 1000UL;

    if (elapsedSeconds >= remainingSeconds) {
        finishTimer();
        return;
    }

    uint32_t oldRemaining = remainingSeconds;
    remainingSeconds -= elapsedSeconds;

    // Beep on reaching 50, 40, 30, 20, 10 (any multiple of 10).
    // Also catches boundaries skipped by a slow loop.
    if ((oldRemaining + 9) / 10 > (remainingSeconds + 9) / 10) {
        beep(1, 65);
    }

    drawHourglass(remainingSeconds);
    updateOLED(true);
}

// ============================================================
// SETTING MODE
// ============================================================

void enterSettingMode() {
    if (timerState == TimerState::Running || timerState == TimerState::Paused) {
        return;
    }

    timerState = TimerState::Setting;
    handledEncoderPosition = readEncoderPosition();

    stopBeeper();
    beep(2, 120, 100);

    remainingSeconds = selectedSeconds;
    drawHourglass(remainingSeconds);
    updateOLED(true);
}

void exitSettingMode() {
    timerState = TimerState::Ready;
    remainingSeconds = selectedSeconds;

    drawHourglass(remainingSeconds);
    beep(1, 300);
    updateOLED(true);
}

// ============================================================
// START BUTTON
// ============================================================

void updateStartButton() {
    uint32_t now = millis();
    bool pressed = digitalRead(START_BUTTON_PIN) == LOW;

    if (pressed != startButtonDown && now - lastStartEdge >= DEBOUNCE_MS) {
        lastStartEdge = now;
        startButtonDown = pressed;

        if (pressed) {
            startPressedAt = now;
            startHoldHandled = false;
        } else if (!startHoldHandled) {
            handleStartPress();
        }
    }

    if (pressed && !startHoldHandled &&
        (timerState == TimerState::Ready || timerState == TimerState::Finished) &&
        now - startPressedAt >= SETTING_HOLD_MS) {

        startHoldHandled = true;
        enterSettingMode();
    }
}

// ============================================================
// ROTARY ENCODER
// ============================================================

void updateEncoderRotation() {
    int32_t position = readEncoderPosition();

    if (timerState != TimerState::Setting) {
        handledEncoderPosition = position;
        return;
    }

    if (position == handledEncoderPosition) {
        return;
    }

    int32_t delta = position - handledEncoderPosition;
    handledEncoderPosition = position;

    int32_t newTime = static_cast<int32_t>(selectedSeconds) +
                      delta * static_cast<int32_t>(TIME_STEP_SECONDS);

    newTime = constrain(newTime,
                        static_cast<int32_t>(MIN_TIME_SECONDS),
                        static_cast<int32_t>(MAX_TIME_SECONDS));

    if (static_cast<uint32_t>(newTime) == selectedSeconds) {
        return;
    }

    selectedSeconds = static_cast<uint32_t>(newTime);
    remainingSeconds = selectedSeconds;

    // One short beep per 10 seconds selected (60 s = 6 beeps).
    beep(static_cast<uint16_t>(selectedSeconds / 10), 65, 45);

    drawHourglass(remainingSeconds);
    updateOLED(true);
}

void updateEncoderSwitch() {
    uint32_t now = millis();
    bool pressed = digitalRead(ENCODER_SW_PIN) == LOW;

    if (pressed != encoderSwitchDown && now - lastEncoderSwitchEdge >= DEBOUNCE_MS) {
        lastEncoderSwitchEdge = now;
        encoderSwitchDown = pressed;

        if (pressed && timerState == TimerState::Setting) {
            exitSettingMode();
        }
    }
}

// ============================================================
// GY-87 SHAKE DETECTION
// ============================================================

bool detectShake() {
    if (!imuAvailable) {
        return false;
    }

    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    imu.getEvent(&acceleration, &gyro, &temperature);

    float ax = acceleration.acceleration.x;
    float ay = acceleration.acceleration.y;
    float az = acceleration.acceleration.z;

    float magnitude = sqrtf(ax * ax + ay * ay + az * az);
    float change = fabsf(magnitude - previousAccelerationMagnitude);
    previousAccelerationMagnitude = magnitude;

    uint32_t now = millis();

    // A shake = two large jolts within SHAKE_CONFIRM_MS.
    if (change > SHAKE_THRESHOLD) {
        if (shakeCandidateAt != 0 &&
            now - shakeCandidateAt <= SHAKE_CONFIRM_MS &&
            now - lastShakeAt >= SHAKE_COOLDOWN_MS) {

            shakeCandidateAt = 0;
            lastShakeAt = now;
            return true;
        }

        shakeCandidateAt = now;
    }

    if (shakeCandidateAt != 0 && now - shakeCandidateAt > SHAKE_CONFIRM_MS) {
        shakeCandidateAt = 0;
    }

    return false;
}

void updateShakeDetection() {
    if (timerState == TimerState::Setting) {
        return;
    }

    if (detectShake()) {
        resetTimer(false);
        beep(2, 180, 150);
    }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);

    // MAX7219 matrices: manual update mode, one flush per frame.
    matrix.begin();
    matrix.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
    matrix.control(MD_MAX72XX::INTENSITY, MATRIX_BRIGHTNESS);
    matrix.clear();
    matrix.update();

    oledAvailable = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    Serial.println(oledAvailable ? F("OLED detected at 0x3C.")
                                 : F("OLED not detected at 0x3C."));

    // GY-87 MPU6050 is normally 0x68; some boards use 0x69.
    imuAvailable = imu.begin(0x68, &Wire);

    if (!imuAvailable) {
        imuAvailable = imu.begin(0x69, &Wire);
    }

    if (imuAvailable) {
        imu.setAccelerometerRange(MPU6050_RANGE_8_G);
        imu.setGyroRange(MPU6050_RANGE_500_DEG);
        imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println(F("MPU6050 detected."));
    } else {
        Serial.println(F("MPU6050 not detected. Shake reset disabled."));
    }

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, FALLING);

    selectedSeconds = DEFAULT_TIME_SECONDS;
    resetTimer(false);

    Serial.println(F("Digital Hourglass ready."));
    Serial.println(F("Hold START 5 s to set time, rotate to adjust, press encoder to save."));
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
    updateStartButton();

    updateEncoderRotation();
    updateEncoderSwitch();

    updateTimer();
    updateShakeDetection();

    updateBeeper();
    updateOLED();
}