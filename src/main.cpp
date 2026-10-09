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
// Features:
// - Encoder counted by interrupt (no missed detents during I2C)
// - MAX7219 manual update mode; one SPI flush per frame
// - I2C at 400 kHz (held there for the OLED and the IMU)
// - OLED shows the current mode and what to do next
// - Self-test at power-up (OLED, IMU, buttons, LED matrix)
// - On-screen messages for user mistakes and hardware faults
// - IMU and OLED are re-detected automatically if unplugged
// - Every message is also printed to Serial at 115200 baud
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

// ---------------- I2C ----------------

constexpr uint32_t I2C_CLOCK_HZ = 400000UL;

// ---------------- OLED ----------------

constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr int8_t OLED_RESET_PIN = -1;
constexpr uint8_t OLED_ADDRESSES[2] = {0x3C, 0x3D};
constexpr uint32_t OLED_REFRESH_MS = 150;

// The last two arguments stop the library dropping the bus back to
// 100 kHz after every display update.
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN,
                      I2C_CLOCK_HZ, I2C_CLOCK_HZ);

bool oledAvailable = false;
uint8_t oledAddress = 0;

// ---------------- GY-87 IMU ----------------

constexpr uint8_t IMU_ADDRESSES[2] = {0x68, 0x69};
constexpr uint8_t IMU_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t IMU_REG_WHO_AM_I = 0x75;
constexpr uint8_t IMU_EXPECTED_ID = 0x68;

// Accelerometer is set to +/-8 g: 4096 counts per g.
constexpr float IMU_COUNTS_PER_G = 4096.0f;
constexpr float STANDARD_GRAVITY = 9.80665f;

// Consecutive bad reads before the IMU is treated as lost.
constexpr uint8_t IMU_FAIL_LIMIT = 3;
constexpr uint8_t IMU_ZERO_LIMIT = 5;

enum class ImuStatus : uint8_t { Missing, WrongChip, Ok };

Adafruit_MPU6050 imu;
bool imuAvailable = false;
ImuStatus imuStatus = ImuStatus::Missing;
uint8_t imuAddress = 0;
uint8_t imuChipId = 0;
uint8_t imuFailCount = 0;
uint8_t imuZeroCount = 0;

// ---------------- TIMER SETTINGS ----------------

constexpr uint32_t MIN_TIME_SECONDS = 10;
constexpr uint32_t MAX_TIME_SECONDS = 600;
constexpr uint32_t TIME_STEP_SECONDS = 10;
constexpr uint32_t DEFAULT_TIME_SECONDS = 60;

constexpr uint32_t SETTING_HOLD_MS = 5000;
constexpr uint32_t SETTING_TIMEOUT_MS = 30000;
constexpr uint32_t DEBOUNCE_MS = 35;
constexpr uint32_t ENCODER_DEBOUNCE_US = 1500;

// ---------------- FAULT HANDLING SETTINGS ----------------

constexpr uint32_t BUTTON_STUCK_MS = 20000;
constexpr uint32_t HEALTH_CHECK_MS = 1000;
constexpr uint32_t RECONNECT_MS = 3000;
constexpr uint32_t NOTICE_MS = 3000;
constexpr uint32_t SELF_TEST_OK_MS = 2000;
constexpr uint32_t SELF_TEST_FAULT_MS = 5000;

// ---------------- SHAKE DETECTION ----------------

constexpr float SHAKE_THRESHOLD = 7.0f;       // m/s^2 change between reads
constexpr uint32_t SHAKE_CONFIRM_MS = 450;
constexpr uint32_t SHAKE_COOLDOWN_MS = 1800;

float previousAccelerationMagnitude = STANDARD_GRAVITY;
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
uint32_t settingActivityMs = 0;

// ---------------- BUTTON STATE ----------------

bool startButtonDown = false;
bool startHoldHandled = false;
bool startStuck = false;
uint32_t startPressedAt = 0;
uint32_t lastStartEdge = 0;

bool encoderSwitchDown = false;
bool knobStuck = false;
uint32_t encoderSwitchPressedAt = 0;
uint32_t lastEncoderSwitchEdge = 0;

// ---------------- HEALTH CHECK STATE ----------------

uint32_t lastHealthCheckMs = 0;
uint32_t lastImuRetryMs = 0;
uint32_t lastOledRetryMs = 0;

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

// Three fast chirps: "that didn't work" or "something is wrong".
void errorBeep() {
    beep(3, 40, 40);
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
// ON-SCREEN NOTICES
// ============================================================
//
// A notice temporarily replaces the three instruction lines on
// the OLED. Each one is also printed to Serial, so messages are
// still visible when no OLED is fitted.

enum class NoticeId : uint8_t {
    None,
    KnobLocked,
    StartNotUsed,
    CantSetNow,
    LimitReached,
    TimerReset,
    SettingTimedOut,
    ImuConnected,
    ImuLost,
    ImuWrongChip
};

struct Notice {
    NoticeId id = NoticeId::None;
    const char* lines[3] = {nullptr, nullptr, nullptr};
    uint32_t shownAt = 0;
    uint32_t durationMs = 0;
};

Notice notice;
char imuIdText[22];

void updateOLED(bool force = false);

bool noticeActive(uint32_t now) {
    return notice.id != NoticeId::None && now - notice.shownAt < notice.durationMs;
}

// Returns true if this notice was not already on screen, so the
// caller only beeps once per message rather than on every repeat.
bool showNotice(NoticeId id, const char* line0, const char* line1,
                const char* line2, uint32_t durationMs = NOTICE_MS) {
    uint32_t now = millis();
    bool isNew = !(noticeActive(now) && notice.id == id);

    notice.id = id;
    notice.lines[0] = line0;
    notice.lines[1] = line1;
    notice.lines[2] = line2;
    notice.shownAt = now;
    notice.durationMs = durationMs;

    if (isNew) {
        Serial.print(F("[NOTICE] "));
        for (uint8_t i = 0; i < 3; i++) {
            if (notice.lines[i] != nullptr) {
                Serial.print(notice.lines[i]);
                Serial.print(i < 2 && notice.lines[i + 1] != nullptr ? F(" | ") : F(""));
            }
        }
        Serial.println();
    }

    updateOLED(true);
    return isNew;
}

void clearNotice() {
    notice.id = NoticeId::None;
}

bool hardwareFaultActive() {
    return startStuck || knobStuck || imuStatus != ImuStatus::Ok;
}

// ============================================================
// I2C HELPERS
// ============================================================

bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

// Returns 0xFF if the register could not be read.
uint8_t i2cReadRegister(uint8_t address, uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return 0xFF;
    }

    if (Wire.requestFrom(address, static_cast<size_t>(1)) != 1) {
        return 0xFF;
    }

    return static_cast<uint8_t>(Wire.read());
}

// ============================================================
// OLED CONNECTION
// ============================================================

bool connectOled() {
    for (uint8_t address : OLED_ADDRESSES) {
        if (!i2cDevicePresent(address)) {
            continue;
        }

        // periphBegin = false: Wire is already running.
        if (oled.begin(SSD1306_SWITCHCAPVCC, address, true, false)) {
            oledAddress = address;
            oledAvailable = true;

            Serial.print(F("OLED found at 0x"));
            Serial.println(address, HEX);
            return true;
        }

        Serial.println(F("OLED answered but failed to start (out of RAM?)."));
    }

    oledAvailable = false;
    oledAddress = 0;
    return false;
}

// ============================================================
// IMU CONNECTION
// ============================================================

void resetShakeState() {
    previousAccelerationMagnitude = STANDARD_GRAVITY;
    shakeCandidateAt = 0;
    imuFailCount = 0;
    imuZeroCount = 0;
}

ImuStatus connectImu() {
    for (uint8_t address : IMU_ADDRESSES) {
        if (!i2cDevicePresent(address)) {
            continue;
        }

        imuAddress = address;

        if (imu.begin(address, &Wire)) {
            imu.setAccelerometerRange(MPU6050_RANGE_8_G);
            imu.setGyroRange(MPU6050_RANGE_500_DEG);
            imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

            resetShakeState();
            return ImuStatus::Ok;
        }

        // Something answered, but it is not an MPU6050.
        imuChipId = i2cReadRegister(address, IMU_REG_WHO_AM_I);
        return ImuStatus::WrongChip;
    }

    imuAddress = 0;
    return ImuStatus::Missing;
}

void setImuStatus(ImuStatus status, bool announce) {
    imuAvailable = (status == ImuStatus::Ok);

    if (status == imuStatus) {
        return;
    }

    imuStatus = status;

    if (!announce) {
        return;
    }

    switch (status) {
        case ImuStatus::Ok:
            showNotice(NoticeId::ImuConnected,
                       "IMU connected", "Shake reset is on", nullptr);
            beep(1, 65);
            break;

        case ImuStatus::Missing:
            showNotice(NoticeId::ImuLost,
                       "! IMU disconnected", "Check GY-87 wiring",
                       "Shake reset is off", NOTICE_MS * 2);
            errorBeep();
            break;

        case ImuStatus::WrongChip:
            snprintf(imuIdText, sizeof(imuIdText), "Got ID 0x%02X, not 0x%02X",
                     imuChipId, IMU_EXPECTED_ID);
            showNotice(NoticeId::ImuWrongChip,
                       "! IMU wrong chip", imuIdText,
                       "Shake reset is off", NOTICE_MS * 2);
            errorBeep();
            break;
    }
}

// ============================================================
// HEALTH CHECKS (run once a second)
// ============================================================

void updateHealth() {
    uint32_t now = millis();

    if (now - lastHealthCheckMs < HEALTH_CHECK_MS) {
        return;
    }

    lastHealthCheckMs = now;

    // IMU: losses are caught on every read in detectShake().
    // Here we only try to bring a missing IMU back.
    if (imuStatus != ImuStatus::Ok && now - lastImuRetryMs >= RECONNECT_MS) {
        lastImuRetryMs = now;
        setImuStatus(connectImu(), true);
    }

    // OLED: the library cannot report write failures, so poll it.
    if (oledAvailable) {
        if (!i2cDevicePresent(oledAddress)) {
            oledAvailable = false;
            Serial.println(F("[ERROR] OLED disconnected. Check OLED wiring."));
            beep(1, 800);
        }
    } else if (now - lastOledRetryMs >= RECONNECT_MS) {
        lastOledRetryMs = now;

        if (connectOled()) {
            Serial.println(F("OLED reconnected."));
            updateOLED(true);
        }
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

// Lights every LED at the normal brightness. The MAX7219's own
// TEST mode is not used because it forces full brightness, which
// can draw more current than USB can supply.
void drawAllLeds() {
    for (uint8_t row = 0; row < 8; row++) {
        for (uint8_t col = 0; col < 16; col++) {
            setPixel(row, col, true);
        }
    }

    matrix.update();
}

// ============================================================
// OLED DISPLAY
// ============================================================
//
// Layout (128 x 64):
//   y  0-9   highlighted bar: current mode, and FAULT if any
//   y 12-35  time, large
//   y 38-63  three lines telling the user what to do next

constexpr uint8_t OLED_LINE_Y[3] = {38, 47, 56};
constexpr uint32_t HOLD_HINT_AFTER_MS = 400;

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

void printLine(uint8_t line, const __FlashStringHelper* text) {
    oled.setCursor(0, OLED_LINE_Y[line]);
    oled.print(text);
}

void printLine(uint8_t line, const char* text) {
    if (text == nullptr) {
        return;
    }

    oled.setCursor(0, OLED_LINE_Y[line]);
    oled.print(text);
}

// Shake resets the timer; without an IMU, holding START does it.
void printResetHint(uint8_t line) {
    if (imuAvailable) {
        printLine(line, F("Shake: reset"));
    } else {
        printLine(line, F("Hold START 5s: reset"));
    }
}

void printTotalLine(uint8_t line) {
    char timeText[8];
    char text[16];

    formatTime(selectedSeconds, timeText, sizeof(timeText));
    snprintf(text, sizeof(text), "Total: %s", timeText);
    printLine(line, text);
}

// True while START is being held towards a 5-second hold action.
bool startHoldInProgress(uint32_t now) {
    return startButtonDown && !startHoldHandled && !startStuck &&
           timerState != TimerState::Setting &&
           now - startPressedAt >= HOLD_HINT_AFTER_MS;
}

void printHoldCountdown(uint8_t line, uint32_t now) {
    uint32_t held = now - startPressedAt;
    uint32_t left = 0;

    if (held < SETTING_HOLD_MS) {
        left = (SETTING_HOLD_MS - held + 999UL) / 1000UL;
    }

    char text[22];
    snprintf(text, sizeof(text), "Keep holding: %lus",
             static_cast<unsigned long>(left));
    printLine(line, text);
}

void printInstructions(uint32_t now) {
    switch (timerState) {
        case TimerState::Ready:
            printLine(0, F("START: begin timer"));
            if (startHoldInProgress(now)) {
                printHoldCountdown(1, now);
            } else {
                printLine(1, F("Hold START 5s: set"));
            }
            if (imuAvailable) {
                printLine(2, F("Shake: reset"));
            } else {
                printLine(2, F("Shake: off (no IMU)"));
            }
            break;

        case TimerState::Running:
            printLine(0, F("START: pause"));
            if (!imuAvailable && startHoldInProgress(now)) {
                printHoldCountdown(1, now);
            } else {
                printResetHint(1);
            }
            printTotalLine(2);
            break;

        case TimerState::Paused:
            printLine(0, F("START: resume"));
            if (!imuAvailable && startHoldInProgress(now)) {
                printHoldCountdown(1, now);
            } else {
                printResetHint(1);
            }
            printTotalLine(2);
            break;

        case TimerState::Setting:
            printLine(0, F("Turn knob: +/-10s"));
            printLine(1, F("Press knob: save"));
            printLine(2, F("Range 00:10 - 10:00"));
            break;

        case TimerState::Finished:
            printLine(0, F("TIME'S UP!"));
            printLine(1, F("START: run again"));
            if (startHoldInProgress(now)) {
                printHoldCountdown(2, now);
            } else {
                printLine(2, F("Hold START 5s: set"));
            }
            break;
    }
}

void updateOLED(bool force) {
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
    oled.setTextWrap(false);
    oled.setTextSize(1);

    // Mode bar: black text on a white strip.
    oled.fillRect(0, 0, OLED_WIDTH, 10, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(2, 1);
    oled.print(F("MODE: "));
    oled.print(stateName());

    if (hardwareFaultActive()) {
        oled.setCursor(OLED_WIDTH - 31, 1);
        oled.print(F("FAULT"));
    }

    // Time: 5 characters x 18 px = 90 px wide, centred.
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(3);
    oled.setCursor(19, 12);
    oled.print(timeText);

    // Bottom three lines, in priority order:
    // stuck button > temporary notice > normal instructions.
    oled.setTextSize(1);

    if (startStuck) {
        printLine(0, F("! START button stuck"));
        printLine(1, F("Release START, or"));
        printLine(2, F("check D5 wiring"));
    } else if (knobStuck) {
        printLine(0, F("! KNOB button stuck"));
        printLine(1, F("Release the knob, or"));
        printLine(2, F("check D4 wiring"));
    } else if (noticeActive(now)) {
        for (uint8_t i = 0; i < 3; i++) {
            printLine(i, notice.lines[i]);
        }
    } else {
        printInstructions(now);
    }

    oled.display();
}

// ============================================================
// SELF-TEST (power-up)
// ============================================================

void printSelfTestLine(uint8_t y, const char* text) {
    oled.setCursor(0, y);
    oled.print(text);
}

void runSelfTest() {
    bool faultFound = hardwareFaultActive() || !oledAvailable;

    // Serial report.
    Serial.println(F("---- SELF TEST ----"));
    Serial.print(F("OLED:  "));
    Serial.println(oledAvailable ? F("OK") : F("NOT FOUND (check SDA/SCL, 5V, GND)"));

    Serial.print(F("IMU:   "));
    switch (imuStatus) {
        case ImuStatus::Ok:
            Serial.println(F("OK"));
            break;
        case ImuStatus::Missing:
            Serial.println(F("NOT FOUND (check GY-87 on A4/A5, VCC_IN to 5V)"));
            break;
        case ImuStatus::WrongChip:
            Serial.print(F("WRONG CHIP ID 0x"));
            Serial.println(imuChipId, HEX);
            break;
    }

    Serial.print(F("START: "));
    Serial.println(startStuck ? F("HELD AT POWER-UP (release it, or check D5)") : F("OK"));
    Serial.print(F("KNOB:  "));
    Serial.println(knobStuck ? F("HELD AT POWER-UP (release it, or check D4)") : F("OK"));
    Serial.println(F("MATRIX: all 128 LEDs should be lit now."));
    Serial.println(F("-------------------"));

    // LED matrix lamp test.
    drawAllLeds();

    // OLED report.
    if (oledAvailable) {
        char line[22];

        oled.clearDisplay();
        oled.setTextWrap(false);
        oled.setTextSize(1);

        oled.fillRect(0, 0, OLED_WIDTH, 10, SSD1306_WHITE);
        oled.setTextColor(SSD1306_BLACK);
        oled.setCursor(2, 1);
        oled.print(faultFound ? F("SELF TEST: FAULT") : F("SELF TEST: PASS"));
        oled.setTextColor(SSD1306_WHITE);

        snprintf(line, sizeof(line), "OLED   OK  (0x%02X)", oledAddress);
        printSelfTestLine(13, line);

        switch (imuStatus) {
            case ImuStatus::Ok:
                snprintf(line, sizeof(line), "IMU    OK  (0x%02X)", imuAddress);
                break;
            case ImuStatus::Missing:
                snprintf(line, sizeof(line), "IMU    NOT FOUND");
                break;
            case ImuStatus::WrongChip:
                snprintf(line, sizeof(line), "IMU    BAD ID 0x%02X", imuChipId);
                break;
        }
        printSelfTestLine(23, line);

        printSelfTestLine(33, startStuck ? "START  HELD/STUCK" : "START  OK");
        printSelfTestLine(43, knobStuck ? "KNOB   HELD/STUCK" : "KNOB   OK");
        printSelfTestLine(53, "All 128 LEDs lit?");

        oled.display();
    }

    // Sound: one long beep if there is no screen to read,
    // the error chirp for any other fault, one short beep if all OK.
    if (!oledAvailable) {
        beep(1, 800);
    } else if (faultFound) {
        errorBeep();
    } else {
        beep(1, 65);
    }

    uint32_t holdMs = faultFound ? SELF_TEST_FAULT_MS : SELF_TEST_OK_MS;
    uint32_t start = millis();

    while (millis() - start < holdMs) {
        updateBeeper();
    }

    stopBeeper();
    matrix.clear();
    matrix.update();
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

    clearNotice();
    beep(2, 65, 100);

    drawHourglass(remainingSeconds);
    updateOLED(true);
}

void pauseTimer() {
    timerState = TimerState::Paused;

    // Keep the part of the current second already elapsed.
    pausedRemainderMs = millis() - lastTimerTickMs;

    clearNotice();
    beep(1, 180);
    updateOLED(true);
}

void resumeTimer() {
    timerState = TimerState::Running;
    lastTimerTickMs = millis() - pausedRemainderMs;
    pausedRemainderMs = 0;

    clearNotice();
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
            if (showNotice(NoticeId::StartNotUsed,
                           "! START not used here", "Turn knob: adjust",
                           "Press knob: save")) {
                errorBeep();
            }
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
    settingActivityMs = millis();

    clearNotice();
    beep(2, 120, 100);

    remainingSeconds = selectedSeconds;
    drawHourglass(remainingSeconds);
    updateOLED(true);
}

void exitSettingMode() {
    timerState = TimerState::Ready;
    remainingSeconds = selectedSeconds;

    clearNotice();
    drawHourglass(remainingSeconds);
    beep(1, 300);
    updateOLED(true);
}

// Saves and leaves SET TIME if the user walks away mid-setting.
void updateSettingTimeout() {
    if (timerState != TimerState::Setting) {
        return;
    }

    if (millis() - settingActivityMs < SETTING_TIMEOUT_MS) {
        return;
    }

    exitSettingMode();
    showNotice(NoticeId::SettingTimedOut,
               "Time auto-saved", "(no input for 30s)", "START: begin timer");
}

// ============================================================
// USER-MISTAKE NOTICES
// ============================================================

void showKnobLockedNotice() {
    bool isNew;

    if (timerState == TimerState::Running || timerState == TimerState::Paused) {
        if (imuAvailable) {
            isNew = showNotice(NoticeId::KnobLocked, "! Knob is locked",
                               "Shake to reset, then", "hold START 5s to set");
        } else {
            isNew = showNotice(NoticeId::KnobLocked, "! Knob is locked",
                               "Hold START 5s: reset", "then again to set");
        }
    } else {
        isNew = showNotice(NoticeId::KnobLocked, "! Knob is locked",
                           "Hold START 5s to set", "the time first");
    }

    if (isNew) {
        errorBeep();
    }
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
        } else if (startStuck) {
            startStuck = false;
            Serial.println(F("START released. Fault cleared."));
            updateOLED(true);
        } else if (!startHoldHandled) {
            handleStartPress();
        }
    }

    if (!pressed) {
        return;
    }

    uint32_t held = now - startPressedAt;

    // Held far longer than any real use: treat as stuck or shorted.
    if (!startStuck && held >= BUTTON_STUCK_MS) {
        startStuck = true;
        startHoldHandled = true;
        Serial.println(F("[ERROR] START held for 20s. Stuck button or D5 shorted to GND?"));
        errorBeep();
        updateOLED(true);
        return;
    }

    if (startHoldHandled || held < SETTING_HOLD_MS) {
        return;
    }

    startHoldHandled = true;

    switch (timerState) {
        case TimerState::Ready:
        case TimerState::Finished:
            enterSettingMode();
            break;

        case TimerState::Running:
        case TimerState::Paused:
            if (imuAvailable) {
                if (showNotice(NoticeId::CantSetNow,
                               "! Can't set time now", "Shake to reset first,",
                               "then hold START 5s")) {
                    errorBeep();
                }
            } else {
                // No IMU means no shake reset, so a long hold resets.
                resetTimer(true);
                showNotice(NoticeId::TimerReset,
                           "Timer reset", "START: begin again", "Hold START 5s: set");
            }
            break;

        case TimerState::Setting:
            break;
    }
}

// ============================================================
// ROTARY ENCODER
// ============================================================

void updateEncoderRotation() {
    int32_t position = readEncoderPosition();

    if (position == handledEncoderPosition) {
        return;
    }

    int32_t delta = position - handledEncoderPosition;
    handledEncoderPosition = position;

    if (timerState != TimerState::Setting) {
        showKnobLockedNotice();
        return;
    }

    settingActivityMs = millis();

    int32_t newTime = static_cast<int32_t>(selectedSeconds) +
                      delta * static_cast<int32_t>(TIME_STEP_SECONDS);

    newTime = constrain(newTime,
                        static_cast<int32_t>(MIN_TIME_SECONDS),
                        static_cast<int32_t>(MAX_TIME_SECONDS));

    if (static_cast<uint32_t>(newTime) == selectedSeconds) {
        // Already at a limit and turning further into it.
        bool isNew;

        if (selectedSeconds >= MAX_TIME_SECONDS) {
            isNew = showNotice(NoticeId::LimitReached, "! Maximum is 10:00",
                               "Turn the other way", "Press knob: save");
        } else {
            isNew = showNotice(NoticeId::LimitReached, "! Minimum is 00:10",
                               "Turn the other way", "Press knob: save");
        }

        if (isNew) {
            errorBeep();
        }
        return;
    }

    clearNotice();

    selectedSeconds = static_cast<uint32_t>(newTime);
    remainingSeconds = selectedSeconds;

    // One short beep per 10 seconds selected (60 s = 6 beeps).
    beep(static_cast<uint16_t>(selectedSeconds / 10), 65, 45);

    drawHourglass(remainingSeconds);
    updateOLED(true);
}

void updateEncoderSwitch()