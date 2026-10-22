#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <MD_MAX72xx.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

// IRremote options must be defined BEFORE the include.
#define DECODE_NEC              // the cheap 17/21-key remotes use the NEC protocol
#define NO_LED_FEEDBACK_CODE    // never touch LED_BUILTIN: on the R4 that is D13 = matrix SPI clock
#include <IRremote.hpp>

// ============================================================
// DIGITAL HOURGLASS - ARDUINO UNO R4 WIFI
// Hardware:
// - 2 daisy-chained MAX7219 8x8 matrices (FC16 modules)
// - 10k linear potentiometer on A0 (sets the time)
// - VS1838B IR receiver on D8 + NEC IR remote (full control)
// - START button on D5
// - Active buzzer on D6 through an NPN transistor
// - GY-87 IMU (MPU6050) for shake-to-reset
// - SSD1306 128x64 I2C OLED
//
// Controls:
// - Turn the pot (READY / FINISHED): time follows the pot, 0:10-10:00 in 10 s steps
// - START or remote PLAY/OK: start / pause / resume
// - Hold START 5 s, shake, or remote RESET: reset the timer
// - Remote digits then OK: type an exact time (1, 3, 0, OK = 1:30)
// - Remote +/- : 10 s, remote PREV/NEXT : 1 min, remote CH: presets
//
// Pot "takeover": after the remote sets a time, the pot is ignored
// until it is turned, so the time never jumps by itself.
// ============================================================

// ---------------- PIN ASSIGNMENTS ----------------

constexpr uint8_t MATRIX_CS_PIN = 10;   // DIN = D11 (MOSI), CLK = D13 (SCK)
constexpr uint8_t START_BUTTON_PIN = 5; // button to GND
constexpr uint8_t BUZZER_PIN = 6;       // to NPN base via 1k
constexpr uint8_t IR_RECEIVE_PIN = 8;   // VS1838B OUT
constexpr uint8_t POT_PIN = A0;         // potentiometer wiper
// I2C: SDA = A4 / SDA header, SCL = A5 / SCL header

// ---------------- MATRIX ----------------

constexpr uint8_t MATRIX_COUNT = 2;
constexpr uint8_t MATRIX_BRIGHTNESS = 3;
MD_MAX72XX matrix(MD_MAX72XX::FC16_HW, MATRIX_CS_PIN, MATRIX_COUNT);

// ---------------- I2C + OLED ----------------

constexpr uint32_t I2C_CLOCK_HZ = 400000UL;
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr int8_t OLED_RESET_PIN = -1;
constexpr uint8_t OLED_ADDRESSES[2] = {0x3C, 0x3D};
constexpr uint32_t OLED_REFRESH_MS = 150;

// Last two arguments keep the bus at 400 kHz after every update.
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN,
                      I2C_CLOCK_HZ, I2C_CLOCK_HZ);
bool oledAvailable = false;
uint8_t oledAddress = 0;

// ---------------- GY-87 IMU ----------------

constexpr uint8_t IMU_ADDRESSES[2] = {0x68, 0x69};
constexpr uint8_t IMU_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t IMU_REG_WHO_AM_I = 0x75;
constexpr uint8_t IMU_EXPECTED_ID = 0x68;
constexpr float IMU_COUNTS_PER_G = 4096.0f;       // +/-8 g range
constexpr float STANDARD_GRAVITY = 9.80665f;
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
constexpr uint32_t RESET_HOLD_MS = 5000;
constexpr uint32_t HOLD_HINT_AFTER_MS = 400;
constexpr uint32_t DEBOUNCE_MS = 35;

// ---------------- POTENTIOMETER ----------------

constexpr uint8_t POT_STEPS = MAX_TIME_SECONDS / TIME_STEP_SECONDS;   // 60 positions
constexpr uint16_t POT_FULL_SCALE = 4096;                              // 12-bit ADC
constexpr float POT_HYSTERESIS = 0.35f;     // fraction of a step to move past a boundary
constexpr float POT_SMOOTHING = 0.25f;      // 0-1, lower = smoother
constexpr uint32_t POT_SAMPLE_MS = 20;

// ---------------- IR REMOTE ----------------

// Two common remotes are supported. Set the one you have; the Serial
// Monitor prints the code of every key, so any remote can be mapped.
enum class RemoteLayout : uint8_t { CarMp3_21Key, Arrows_17Key };
constexpr RemoteLayout REMOTE_LAYOUT = RemoteLayout::CarMp3_21Key;

enum class RemoteKey : uint8_t {
    None, Digit0, Digit1, Digit2, Digit3, Digit4, Digit5, Digit6, Digit7, Digit8, Digit9,
    PlayOk, Plus10, Minus10, PlusMin, MinusMin, Reset, Preset, Clear
};

struct KeyCode { uint8_t command; RemoteKey key; };

// "Car MP3" 21-key remote (CH-/CH/CH+, |<< >>| >||, -/+/EQ, 0/100+/200+, 1-9)
const KeyCode CAR_MP3_KEYS[] = {
    {0x16, RemoteKey::Digit0}, {0x0C, RemoteKey::Digit1}, {0x18, RemoteKey::Digit2}, {0x5E, RemoteKey::Digit3},
    {0x08, RemoteKey::Digit4}, {0x1C, RemoteKey::Digit5}, {0x5A, RemoteKey::Digit6}, {0x42, RemoteKey::Digit7},
    {0x52, RemoteKey::Digit8}, {0x4A, RemoteKey::Digit9},
    {0x43, RemoteKey::PlayOk},   // >||
    {0x15, RemoteKey::Plus10},   // +
    {0x07, RemoteKey::Minus10},  // -
    {0x40, RemoteKey::PlusMin},  // >>|
    {0x44, RemoteKey::MinusMin}, // |<<
    {0x09, RemoteKey::Reset},    // EQ
    {0x46, RemoteKey::Preset},   // CH
    {0x19, RemoteKey::Clear},    // 100+
};

// 17-key remote with arrows (1-9, *, 0, #, arrows, OK)
const KeyCode ARROW_KEYS[] = {
    {0x52, RemoteKey::Digit0}, {0x16, RemoteKey::Digit1}, {0x19, RemoteKey::Digit2}, {0x0D, RemoteKey::Digit3},
    {0x0C, RemoteKey::Digit4}, {0x18, RemoteKey::Digit5}, {0x5E, RemoteKey::Digit6}, {0x08, RemoteKey::Digit7},
    {0x1C, RemoteKey::Digit8}, {0x5A, RemoteKey::Digit9},
    {0x40, RemoteKey::PlayOk},   // OK
    {0x46, RemoteKey::Plus10},   // up
    {0x15, RemoteKey::Minus10},  // down
    {0x43, RemoteKey::PlusMin},  // right
    {0x44, RemoteKey::MinusMin}, // left
    {0x4A, RemoteKey::Reset},    // #
    {0x42, RemoteKey::Clear},    // *
};

const char* RESET_KEY_NAME = (REMOTE_LAYOUT == RemoteLayout::CarMp3_21Key) ? "EQ" : "#";

struct Preset { const char* name; uint32_t seconds; };
const Preset PRESETS[] = {
    {"Brush teeth", 120}, {"Tea", 180}, {"Plank", 60}, {"Soft egg", 360}, {"Pasta", 600}
};
constexpr uint8_t PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);

constexpr uint32_t IR_REPEAT_MS = 200;      // held +/- keys repeat at this rate
constexpr uint32_t ENTRY_TIMEOUT_MS = 6000; // typed digits are cancelled after this

// ---------------- FAULT HANDLING ----------------

constexpr uint32_t BUTTON_STUCK_MS = 20000;
constexpr uint32_t HEALTH_CHECK_MS = 1000;
constexpr uint32_t RECONNECT_MS = 3000;
constexpr uint32_t NOTICE_MS = 3000;
constexpr uint32_t SELF_TEST_OK_MS = 2000;
constexpr uint32_t SELF_TEST_FAULT_MS = 5000;

// ---------------- SHAKE DETECTION ----------------

constexpr float SHAKE_THRESHOLD = 7.0f;
constexpr uint32_t SHAKE_CONFIRM_MS = 450;
constexpr uint32_t SHAKE_COOLDOWN_MS = 1800;
float previousAccelerationMagnitude = STANDARD_GRAVITY;
uint32_t shakeCandidateAt = 0;
uint32_t lastShakeAt = 0;

// ---------------- STATE ----------------

enum class TimerState : uint8_t { Ready, Running, Paused, Finished };
TimerState timerState = TimerState::Ready;

uint32_t selectedSeconds = 60;
uint32_t remainingSeconds = 60;
uint32_t lastTimerTickMs = 0;
uint32_t pausedRemainderMs = 0;
uint32_t lastDisplayUpdateMs = 0;

bool startButtonDown = false;
bool startHoldHandled = false;
bool startStuck = false;
uint32_t startPressedAt = 0;
uint32_t lastStartEdge = 0;

float potFiltered = 0;
uint8_t potStep = 0;              // 0..59 -> 10 s .. 600 s
bool potHasControl = true;        // false after the remote sets a time
uint8_t potStepAtLock = 0;
uint32_t lastPotSampleMs = 0;

bool entryActive = false;         // typing digits on the remote
uint16_t entryValue = 0;          // up to 4 digits, read as MMSS
uint8_t entryDigits = 0;
uint32_t entryLastKeyMs = 0;
uint8_t presetIndex = 0;
uint32_t lastIrActionMs = 0;
RemoteKey lastIrKey = RemoteKey::None;

uint32_t lastHealthCheckMs = 0;
uint32_t lastImuRetryMs = 0;
uint32_t lastOledRetryMs = 0;

// ============================================================
// BUZZER (non-blocking)
// ============================================================

struct Beeper {
    uint16_t totalBeeps = 0, completedBeeps = 0, onTimeMs = 65, offTimeMs = 100;
    uint32_t nextChangeMs = 0;
    bool active = false, sounding = false;
};
Beeper beeper;

void stopBeeper() {
    beeper.active = false;
    beeper.sounding = false;
    digitalWrite(BUZZER_PIN, LOW);
}

void beep(uint16_t count, uint16_t onMs, uint16_t offMs = 100) {
    stopBeeper();
    if (count == 0) return;
    beeper.totalBeeps = count;
    beeper.completedBeeps = 0;
    beeper.onTimeMs = onMs;
    beeper.offTimeMs = offMs;
    beeper.active = true;
    beeper.sounding = true;
    digitalWrite(BUZZER_PIN, HIGH);
    beeper.nextChangeMs = millis() + onMs;
}

void errorBeep() { beep(3, 40, 40); }   // three fast chirps = "no"
void tick() { beep(1, 12); }            // tiny click for pot steps / key presses

void updateBeeper() {
    if (!beeper.active) return;
    if (static_cast<int32_t>(millis() - beeper.nextChangeMs) < 0) return;

    if (beeper.sounding) {
        digitalWrite(BUZZER_PIN, LOW);
        beeper.sounding = false;
        if (++beeper.completedBeeps >= beeper.totalBeeps) {
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

enum class NoticeId : uint8_t {
    None, CantChange, InvalidTime, OutOfRange, TimeSet, PresetSet, Cancelled,
    UnknownKey, PotLocked, TimerReset, ImuConnected, ImuLost, ImuWrongChip
};

struct Notice {
    NoticeId id = NoticeId::None;
    const char* lines[3] = {nullptr, nullptr, nullptr};
    uint32_t shownAt = 0, durationMs = 0;
};
Notice notice;
char noticeText[2][22];       // for notices with numbers in them

void updateOLED(bool force = false);

bool noticeActive(uint32_t now) {
    return notice.id != NoticeId::None && now - notice.shownAt < notice.durationMs;
}

// Returns true if the notice was not already showing (beep only once).
bool showNotice(NoticeId id, const char* l0, const char* l1, const char* l2,
                uint32_t durationMs = NOTICE_MS) {
    uint32_t now = millis();
    bool isNew = !(noticeActive(now) && notice.id == id);
    notice.id = id;
    notice.lines[0] = l0;
    notice.lines[1] = l1;
    notice.lines[2] = l2;
    notice.shownAt = now;
    notice.durationMs = durationMs;

    if (isNew) {
        Serial.print(F("[NOTICE] "));
        for (uint8_t i = 0; i < 3; i++) {
            if (notice.lines[i] != nullptr) {
                if (i > 0) Serial.print(F(" | "));
                Serial.print(notice.lines[i]);
            }
        }
        Serial.println();
    }
    updateOLED(true);
    return isNew;
}

void clearNotice() { notice.id = NoticeId::None; }

bool hardwareFaultActive() { return startStuck || imuStatus != ImuStatus::Ok; }

// ============================================================
// I2C HELPERS + DEVICE CONNECTION
// ============================================================

bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

uint8_t i2cReadRegister(uint8_t address, uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    if (Wire.requestFrom(address, static_cast<size_t>(1)) != 1) return 0xFF;
    return static_cast<uint8_t>(Wire.read());
}

bool connectOled() {
    for (uint8_t address : OLED_ADDRESSES) {
        if (!i2cDevicePresent(address)) continue;
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

void resetShakeState() {
    previousAccelerationMagnitude = STANDARD_GRAVITY;
    shakeCandidateAt = 0;
    imuFailCount = 0;
    imuZeroCount = 0;
}

ImuStatus connectImu() {
    for (uint8_t address : IMU_ADDRESSES) {
        if (!i2cDevicePresent(address)) continue;
        imuAddress = address;
        if (imu.begin(address, &Wire)) {
            imu.setAccelerometerRange(MPU6050_RANGE_8_G);
            imu.setGyroRange(MPU6050_RANGE_500_DEG);
            imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
            resetShakeState();
            return ImuStatus::Ok;
        }
        imuChipId = i2cReadRegister(address, IMU_REG_WHO_AM_I);
        return ImuStatus::WrongChip;
    }
    imuAddress = 0;
    return ImuStatus::Missing;
}

void setImuStatus(ImuStatus status, bool announce) {
    imuAvailable = (status == ImuStatus::Ok);
    if (status == imuStatus) return;
    imuStatus = status;
    if (!announce) return;

    switch (status) {
        case ImuStatus::Ok:
            showNotice(NoticeId::ImuConnected, "IMU connected", "Shake reset is on", nullptr);
            beep(1, 65);
            break;
        case ImuStatus::Missing:
            showNotice(NoticeId::ImuLost, "! IMU disconnected", "Check GY-87 wiring",
                       "Shake reset is off", NOTICE_MS * 2);
            errorBeep();
            break;
        case ImuStatus::WrongChip:
            snprintf(noticeText[0], sizeof(noticeText[0]), "Got ID 0x%02X, not 0x%02X",
                     imuChipId, IMU_EXPECTED_ID);
            showNotice(NoticeId::ImuWrongChip, "! IMU wrong chip", noticeText[0],
                       "Shake reset is off", NOTICE_MS * 2);
            errorBeep();
            break;
    }
}

void updateHealth() {
    uint32_t now = millis();
    if (now - lastHealthCheckMs < HEALTH_CHECK_MS) return;
    lastHealthCheckMs = now;

    if (imuStatus != ImuStatus::Ok && now - lastImuRetryMs >= RECONNECT_MS) {
        lastImuRetryMs = now;
        setImuStatus(connectImu(), true);
    }

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
// LED MATRICES
// ============================================================

void setPixel(uint8_t row, uint16_t column, bool on) {
    if (row < 8 && column < 16) matrix.setPoint(row, column, on);
}

void drawHourglass(uint32_t secondsLeft) {
    matrix.clear();
    if (secondsLeft > selectedSeconds) secondsLeft = selectedSeconds;

    uint32_t elapsed = selectedSeconds - secondsLeft;
    uint8_t sandMoved = static_cast<uint8_t>((elapsed * 64UL) / selectedSeconds);
    uint8_t topSand = 64 - sandMoved;
    uint8_t bottomSand = sandMoved;

    for (uint8_t row = 0; row < 8; row++) {
        for (uint8_t col = 0; col < 8; col++) {
            uint8_t rank = (7 - row) * 8 + col;            // 0 = bottom-left
            setPixel(row, col + 8, rank < topSand);        // upper chamber
            setPixel(row, col, rank < bottomSand);         // lower chamber
        }
    }
    matrix.update();
}

// All LEDs at normal brightness (MAX7219 TEST mode would force full brightness).
void drawAllLeds() {
    for (uint8_t row = 0; row < 8; row++)
        for (uint8_t col = 0; col < 16; col++) setPixel(row, col, true);
    matrix.update();
}

// ============================================================
// OLED
// ============================================================
// y 0-9 mode bar · y 12-35 big time · y 38-63 three instruction lines

constexpr uint8_t OLED_LINE_Y[3] = {38, 47, 56};

const char* stateName() {
    if (entryActive) return "TYPE TIME";
    switch (timerState) {
        case TimerState::Ready:    return "READY";
        case TimerState::Running:  return "RUNNING";
        case TimerState::Paused:   return "PAUSED";
        case TimerState::Finished: return "FINISHED";
    }
    return "";
}

void formatTime(uint32_t seconds, char* buffer, size_t length) {
    snprintf(buffer, length, "%02lu:%02lu",
             static_cast<unsigned long>(seconds / 60), static_cast<unsigned long>(seconds % 60));
}

void printLine(uint8_t line, const char* text) {
    if (text == nullptr) return;
    oled.setCursor(0, OLED_LINE_Y[line]);
    oled.print(text);
}

bool startHoldInProgress(uint32_t now) {
    return startButtonDown && !startHoldHandled && !startStuck &&
           timerState != TimerState::Ready && now - startPressedAt >= HOLD_HINT_AFTER_MS;
}

void printInstructions(uint32_t now) {
    char buf[22];
    char resetLine[22];
    snprintf(resetLine, sizeof(resetLine), imuAvailable ? "Shake / %s: reset" : "Hold START/%s: reset",
             RESET_KEY_NAME);

    if (startHoldInProgress(now)) {
        uint32_t held = now - startPressedAt;
        uint32_t left = held < RESET_HOLD_MS ? (RESET_HOLD_MS - held + 999UL) / 1000UL : 0;
        snprintf(buf, sizeof(buf), "Keep holding: %lus", static_cast<unsigned long>(left));
        printLine(0, "Resetting...");
        printLine(1, buf);
        return;
    }

    switch (timerState) {
        case TimerState::Ready:
            printLine(0, "START/OK: begin");
            printLine(1, potHasControl ? "Turn pot: set time" : "Turn pot to adjust");
            printLine(2, "Remote: 0-9 then OK");
            break;
        case TimerState::Running:
        case TimerState::Paused:
            printLine(0, timerState == TimerState::Running ? "START/OK: pause" : "START/OK: resume");
            printLine(1, resetLine);
            formatTime(selectedSeconds, buf, sizeof(buf));
            {
                char total[22];
                snprintf(total, sizeof(total), "Total %s", buf);
                printLine(2, total);
            }
            break;
        case TimerState::Finished:
            printLine(0, "TIME'S UP!");
            printLine(1, "START/OK: run again");
            printLine(2, "Pot/remote: new time");
            break;
    }
}

void updateOLED(bool force) {
    if (!oledAvailable) return;
    uint32_t now = millis();
    if (!force && now - lastDisplayUpdateMs < OLED_REFRESH_MS) return;
    lastDisplayUpdateMs = now;

    char timeText[8];
    if (entryActive) {
        snprintf(timeText, sizeof(timeText), "%02u:%02u", entryValue / 100, entryValue % 100);
    } else {
        bool counting = timerState == TimerState::Running || timerState == TimerState::Paused;
        formatTime(counting ? remainingSeconds : selectedSeconds, timeText, sizeof(timeText));
    }

    oled.clearDisplay();
    oled.setTextWrap(false);
    oled.setTextSize(1);

    oled.fillRect(0, 0, OLED_WIDTH, 10, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(2, 1);
    oled.print(F("MODE: "));
    oled.print(stateName());
    if (hardwareFaultActive()) {
        oled.setCursor(OLED_WIDTH - 31, 1);
        oled.print(F("FAULT"));
    }

    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(3);
    oled.setCursor(19, 12);
    oled.print(timeText);
    oled.setTextSize(1);

    if (startStuck) {
        printLine(0, "! START button stuck");
        printLine(1, "Release START, or");
        printLine(2, "check D5 wiring");
    } else if (noticeActive(now)) {
        for (uint8_t i = 0; i < 3; i++) printLine(i, notice.lines[i]);
    } else if (entryActive) {
        printLine(0, "OK: set this time");
        printLine(1, "Keep typing digits");
        printLine(2, REMOTE_LAYOUT == RemoteLayout::CarMp3_21Key ? "100+: cancel" : "*: cancel");
    } else {
        printInstructions(now);
    }
    oled.display();
}

// ============================================================
// TIMER CONTROL
// ============================================================

void lockPot() {                  // pot must be turned before it acts again
    potHasControl = false;
    potStepAtLock = potStep;
}

void resetTimer(bool playSound) {
    timerState = TimerState::Ready;
    remainingSeconds = selectedSeconds;
    lastTimerTickMs = millis();
    pausedRemainderMs = 0;
    stopBeeper();
    lockPot();
    drawHourglass(remainingSeconds);
    if (playSound) beep(2, 100, 100);
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
    lockPot();
    drawHourglass(0);
    beep(3, 450, 180);
    updateOLED(true);
}

void startPauseResume() {
    switch (timerState) {
        case TimerState::Ready:
        case TimerState::Finished: startTimer(); break;
        case TimerState::Running:  pauseTimer(); break;
        case TimerState::Paused:   resumeTimer(); break;
    }
}

bool canChangeTime() {
    return timerState == TimerState::Ready || timerState == TimerState::Finished;
}

// Sets a new selected time from the pot or the remote.
void applySelectedTime(uint32_t seconds, bool fromRemote) {
    selectedSeconds = seconds;
    remainingSeconds = seconds;
    timerState = TimerState::Ready;
    if (fromRemote) lockPot();
    drawHourglass(remainingSeconds);
    updateOLED(true);
}

void updateTimer() {
    if (timerState != TimerState::Running) return;
    uint32_t elapsedSeconds = (millis() - lastTimerTickMs) / 1000UL;
    if (elapsedSeconds == 0) return;
    lastTimerTickMs += elapsedSeconds * 1000UL;

    if (elapsedSeconds >= remainingSeconds) {
        finishTimer();
        return;
    }
    uint32_t oldRemaining = remainingSeconds;
    remainingSeconds -= elapsedSeconds;
    if ((oldRemaining + 9) / 10 > (remainingSeconds + 9) / 10) beep(1, 65);   // 50, 40 ... 10
    drawHourglass(remainingSeconds);
    updateOLED(true);
}

// ============================================================
// POTENTIOMETER
// ============================================================

uint8_t potStepFromFiltered(float filtered, uint8_t current) {
    float position = filtered * POT_STEPS / POT_FULL_SCALE;        // 0 .. 60
    float centre = current + 0.5f;
    if (fabsf(position - centre) <= 0.5f + POT_HYSTERESIS) return current;   // dead-band
    int stepIndex = static_cast<int>(position);
    if (stepIndex < 0) stepIndex = 0;
    if (stepIndex > POT_STEPS - 1) stepIndex = POT_STEPS - 1;
    return static_cast<uint8_t>(stepIndex);
}

void initPot() {
    potFiltered = analogRead(POT_PIN);
    int s = static_cast<int>(potFiltered * POT_STEPS / POT_FULL_SCALE);
    potStep = static_cast<uint8_t>(constrain(s, 0, POT_STEPS - 1));
    selectedSeconds = (potStep + 1) * TIME_STEP_SECONDS;     // pot position wins at power-up
    remainingSeconds = selectedSeconds;
    potHasControl = true;
}

void updatePot() {
    uint32_t now = millis();
    if (now - lastPotSampleMs < POT_SAMPLE_MS) return;
    lastPotSampleMs = now;

    potFiltered += (analogRead(POT_PIN) - potFiltered) * POT_SMOOTHING;
    uint8_t newStep = potStepFromFiltered(potFiltered, potStep);
    if (newStep == potStep) return;
    potStep = newStep;

    if (!canChangeTime() || entryActive) {
        if (timerState == TimerState::Running || timerState == TimerState::Paused) {
            if (showNotice(NoticeId::PotLocked, "! Pot locked", "Reset the timer to",
                           "set a new time")) {
                errorBeep();
            }
        }
        lockPot();
        return;
    }

    // Takeover: once the pot moves away from where it was locked, it rules again.
    if (!potHasControl) {
        if (potStep == potStepAtLock) return;
        potHasControl = true;
    }

    clearNotice();
    applySelectedTime((potStep + 1) * TIME_STEP_SECONDS, false);
    tick();
}

// ============================================================
// IR REMOTE
// ============================================================

RemoteKey lookupKey(uint8_t command) {
    if (REMOTE_LAYOUT == RemoteLayout::CarMp3_21Key) {
        for (const KeyCode& k : CAR_MP3_KEYS) if (k.command == command) return k.key;
    } else {
        for (const KeyCode& k : ARROW_KEYS) if (k.command == command) return k.key;
    }
    return RemoteKey::None;
}

void cancelEntry(bool announce) {
    if (!entryActive) return;
    entryActive = false;
    if (announce) showNotice(NoticeId::Cancelled, "Typing cancelled", nullptr, nullptr, 1500);
    updateOLED(true);
}

void showCantChange() {
    if (showNotice(NoticeId::CantChange, "! Can't change now", "Pause + reset first,",
                   "then set a new time")) {
        errorBeep();
    }
}

void setTimeFromRemote(int32_t seconds) {
    if (seconds < static_cast<int32_t>(MIN_TIME_SECONDS) || seconds > static_cast<int32_t>(MAX_TIME_SECONDS)) {
        if (showNotice(NoticeId::OutOfRange, "! Out of range", "Min 00:10", "Max 10:00")) errorBeep();
        return;
    }
    applySelectedTime(static_cast<uint32_t>(seconds), true);
    char t[8];
    formatTime(static_cast<uint32_t>(seconds), t, sizeof(t));
    snprintf(noticeText[0], sizeof(noticeText[0]), "Time set: %s", t);
    showNotice(NoticeId::TimeSet, noticeText[0], "Turn pot to change", nullptr, 2000);
    tick();
}

void confirmEntry() {
    entryActive = false;
    uint16_t minutes = entryValue / 100;
    uint16_t seconds = entryValue % 100;
    if (seconds > 59) {
        if (showNotice(NoticeId::InvalidTime, "! Invalid time", "Seconds must be 0-59",
                       "Try again")) {
            errorBeep();
        }
        updateOLED(true);
        return;
    }
    setTimeFromRemote(minutes * 60 + seconds);
}

void handleRemoteKey(RemoteKey key, bool isRepeat) {
    uint32_t now = millis();

    // Held keys: only +/- auto-repeat, and only every IR_REPEAT_MS.
    if (isRepeat) {
        bool repeatable = key == RemoteKey::Plus10 || key == RemoteKey::Minus10;
        if (!repeatable || now - lastIrActionMs < IR_REPEAT_MS) return;
    }
    lastIrActionMs = now;

    if (key >= RemoteKey::Digit0 && key <= RemoteKey::Digit9) {
        if (!canChangeTime()) { showCantChange(); return; }
        uint8_t digit = static_cast<uint8_t>(key) - static_cast<uint8_t>(RemoteKey::Digit0);
        if (!entryActive) { entryActive = true; entryValue = 0; entryDigits = 0; }
        if (entryDigits < 4) {
            entryValue = entryValue * 10 + digit;
            entryDigits++;
        } else {
            entryValue = (entryValue % 1000) * 10 + digit;   // keep the last 4 digits
        }
        entryLastKeyMs = now;
        clearNotice();
        tick();
        updateOLED(true);
        return;
    }

    switch (key) {
        case RemoteKey::PlayOk:
            if (entryActive) confirmEntry();
            else startPauseResume();
            break;

        case RemoteKey::Clear:
            cancelEntry(true);
            break;

        case RemoteKey::Plus10:
        case RemoteKey::Minus10:
        case RemoteKey::PlusMin:
        case RemoteKey::MinusMin: {
            if (!canChangeTime()) { showCantChange(); break; }
            cancelEntry(false);
            int32_t delta = (key == RemoteKey::Plus10) ? 10 : (key == RemoteKey::Minus10) ? -10
                          : (key == RemoteKey::PlusMin) ? 60 : -60;
            int32_t t = static_cast<int32_t>(selectedSeconds) + delta;
            if (t < static_cast<int32_t>(MIN_TIME_SECONDS) || t > static_cast<int32_t>(MAX_TIME_SECONDS)) {
                if (showNotice(NoticeId::OutOfRange, "! Out of range", "Min 00:10", "Max 10:00")) errorBeep();
                break;
            }
            applySelectedTime(static_cast<uint32_t>(t), true);
            tick();
            break;
        }

        case RemoteKey::Preset: {
            if (!canChangeTime()) { showCantChange(); break; }
            cancelEntry(false);
            const Preset& p = PRESETS[presetIndex];
            presetIndex = (presetIndex + 1) % PRESET_COUNT;
            applySelectedTime(p.seconds, true);
            char t[8];
            formatTime(p.seconds, t, sizeof(t));
            snprintf(noticeText[0], sizeof(noticeText[0]), "Preset: %s", p.name);
            snprintf(noticeText[1], sizeof(noticeText[1]), "%s  (CH = next)", t);
            showNotice(NoticeId::PresetSet, noticeText[0], noticeText[1], "START/OK: begin", 2500);
            tick();
            break;
        }

        case RemoteKey::Reset: {
            cancelEntry(false);
            bool wasActive = timerState != TimerState::Ready;
            resetTimer(true);
            if (wasActive) showNotice(NoticeId::TimerReset, "Reset by remote", "START/OK: begin", nullptr, 2000);
            break;
        }

        default:
            break;
    }
}

// The VS1838B demodulates the remote's 38 kHz light into pulses on D8.
// IRremote samples D8 in a timer interrupt, so a whole NEC frame is
// captured in the background; here we only collect finished frames.
void updateIr() {
    if (!IrReceiver.decode()) {
        if (entryActive && millis() - entryLastKeyMs >= ENTRY_TIMEOUT_MS) cancelEntry(true);
        return;
    }

    IRData& data = IrReceiver.decodedIRData;
    bool isRepeat = data.flags & IRDATA_FLAGS_IS_REPEAT;

    if (data.protocol == NEC) {
        uint8_t command = static_cast<uint8_t>(data.command);
        RemoteKey key = lookupKey(command);

        if (!isRepeat) {
            Serial.print(F("[IR] address 0x"));
            Serial.print(data.address, HEX);
            Serial.print(F("  command 0x"));
            Serial.print(command, HEX);
            Serial.println(key == RemoteKey::None ? F("  (not mapped)") : F(""));
        }

        if (key == RemoteKey::None) {
            if (!isRepeat) {
                snprintf(noticeText[0], sizeof(noticeText[0]), "IR key 0x%02X", command);
                showNotice(NoticeId::UnknownKey, noticeText[0], "is not mapped", "(see Serial Monitor)", 2000);
            }
        } else {
            if (isRepeat && key != lastIrKey) key = RemoteKey::None;   // repeat of an ignored key
            if (key != RemoteKey::None) {
                lastIrKey = key;
                handleRemoteKey(key, isRepeat);
            }
        }
    }
    // Other protocols / noise are ignored silently.
    IrReceiver.resume();
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
            if (entryActive) confirmEntry();
            else startPauseResume();
        }
    }

    if (!pressed) return;
    uint32_t held = now - startPressedAt;

    if (!startStuck && held >= BUTTON_STUCK_MS) {
        startStuck = true;
        startHoldHandled = true;
        Serial.println(F("[ERROR] START held for 20s. Stuck button or D5 shorted to GND?"));
        errorBeep();
        updateOLED(true);
        return;
    }

    // Hold 5 s while running / paused / finished = reset.
    if (!startHoldHandled && timerState != TimerState::Ready && held >= RESET_HOLD_MS) {
        startHoldHandled = true;
        resetTimer(true);
        showNotice(NoticeId::TimerReset, "Timer reset", "START/OK: begin", nullptr, 2000);
    }
}

// ============================================================
// SHAKE DETECTION
// ============================================================

bool readAccelerationMagnitude(float& magnitude) {
    Wire.beginTransmission(imuAddress);
    Wire.write(IMU_REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(imuAddress, static_cast<size_t>(6)) != 6) return false;

    int16_t raw[3];
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t high = static_cast<uint8_t>(Wire.read());
        uint8_t low = static_cast<uint8_t>(Wire.read());
        raw[i] = static_cast<int16_t>((high << 8) | low);
    }

    if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0) {        // chip reset into sleep
        if (++imuZeroCount >= IMU_ZERO_LIMIT) {
            Serial.println(F("[ERROR] IMU reading all zeros. Power lost or reset?"));
            setImuStatus(ImuStatus::Missing, true);
        }
        return false;
    }
    imuZeroCount = 0;

    float ax = raw[0] / IMU_COUNTS_PER_G * STANDARD_GRAVITY;
    float ay = raw[1] / IMU_COUNTS_PER_G * STANDARD_GRAVITY;
    float az = raw[2] / IMU_COUNTS_PER_G * STANDARD_GRAVITY;
    magnitude = sqrtf(ax * ax + ay * ay + az * az);
    return true;
}

bool detectShake() {
    if (!imuAvailable) return false;
    float magnitude;
    if (!readAccelerationMagnitude(magnitude)) {
        if (imuAvailable && imuZeroCount == 0 && ++imuFailCount >= IMU_FAIL_LIMIT) {
            Serial.println(F("[ERROR] IMU stopped answering on I2C."));
            setImuStatus(ImuStatus::Missing, true);
        }
        return false;
    }
    imuFailCount = 0;

    float change = fabsf(magnitude - previousAccelerationMagnitude);
    previousAccelerationMagnitude = magnitude;
    uint32_t now = millis();

    if (change > SHAKE_THRESHOLD) {
        if (shakeCandidateAt != 0 && now - shakeCandidateAt <= SHAKE_CONFIRM_MS &&
            now - lastShakeAt >= SHAKE_COOLDOWN_MS) {
            shakeCandidateAt = 0;
            lastShakeAt = now;
            return true;
        }
        shakeCandidateAt = now;
    }
    if (shakeCandidateAt != 0 && now - shakeCandidateAt > SHAKE_CONFIRM_MS) shakeCandidateAt = 0;
    return false;
}

void updateShakeDetection() {
    if (!detectShake()) return;
    bool wasActive = timerState != TimerState::Ready;
    cancelEntry(false);
    resetTimer(false);
    beep(2, 180, 150);
    if (wasActive) showNotice(NoticeId::TimerReset, "Timer reset (shake)", "START/OK: begin", nullptr, 2000);
}

// ============================================================
// SELF-TEST
// ============================================================

void runSelfTest() {
    bool faultFound = hardwareFaultActive() || !oledAvailable;
    char t[8];
    formatTime(selectedSeconds, t, sizeof(t));

    Serial.println(F("---- SELF TEST ----"));
    Serial.print(F("OLED:  ")); Serial.println(oledAvailable ? F("OK") : F("NOT FOUND (check SDA/SCL, 5V, GND)"));
    Serial.print(F("IMU:   "));
    switch (imuStatus) {
        case ImuStatus::Ok:        Serial.println(F("OK")); break;
        case ImuStatus::Missing:   Serial.println(F("NOT FOUND (check GY-87 on A4/A5, VCC_IN to 5V)")); break;
        case ImuStatus::WrongChip: Serial.print(F("WRONG CHIP ID 0x")); Serial.println(imuChipId, HEX); break;
    }
    Serial.print(F("START: ")); Serial.println(startStuck ? F("HELD AT POWER-UP (release it, or check D5)") : F("OK"));
    Serial.print(F("POT:   ")); Serial.print(t); Serial.print(F("  (raw ")); Serial.print(static_cast<int>(potFiltered)); Serial.println(F(")"));
    Serial.print(F("IR:    receiver on D8, layout "));
    Serial.println(REMOTE_LAYOUT == RemoteLayout::CarMp3_21Key ? F("21-key Car MP3") : F("17-key arrows"));
    Serial.println(F("MATRIX: all 128 LEDs should be lit now."));
    Serial.println(F("-------------------"));

    drawAllLeds();

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
        oled.setCursor(0, 13); oled.print(line);
        if (imuStatus == ImuStatus::Ok) snprintf(line, sizeof(line), "IMU    OK  (0x%02X)", imuAddress);
        else if (imuStatus == ImuStatus::Missing) snprintf(line, sizeof(line), "IMU    NOT FOUND");
        else snprintf(line, sizeof(line), "IMU    BAD ID 0x%02X", imuChipId);
        oled.setCursor(0, 23); oled.print(line);
        oled.setCursor(0, 33); oled.print(startStuck ? "START  HELD/STUCK" : "START  OK");
        snprintf(line, sizeof(line), "POT    %s", t);
        oled.setCursor(0, 43); oled.print(line);
        oled.setCursor(0, 53); oled.print("IR D8 ready. LEDs on?");
        oled.display();
    }

    if (!oledAvailable) beep(1, 800);
    else if (faultFound) errorBeep();
    else beep(1, 65);

    uint32_t holdMs = faultFound ? SELF_TEST_FAULT_MS : SELF_TEST_OK_MS;
    uint32_t start = millis();
    while (millis() - start < holdMs) updateBeeper();
    stopBeeper();
}

// ============================================================
// SETUP + LOOP
// ============================================================

void setup() {
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    analogReadResolution(12);

    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);

    matrix.begin();
    matrix.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
    matrix.control(MD_MAX72XX::INTENSITY, MATRIX_BRIGHTNESS);
    matrix.clear();
    matrix.update();

    if (!connectOled()) Serial.println(F("OLED not found. Messages go to Serial only."));
    setImuStatus(connectImu(), false);
    initPot();

    if (digitalRead(START_BUTTON_PIN) == LOW) {       // held or shorted at power-up
        uint32_t now = millis();
        startButtonDown = true;
        startHoldHandled = true;
        startStuck = true;
        startPressedAt = now;
        lastStartEdge = now;
    }

    runSelfTest();

    IrReceiver.begin(IR_RECEIVE_PIN);                // no LED feedback (D13 is SPI clock)

    lastHealthCheckMs = millis();
    lastImuRetryMs = lastHealthCheckMs;
    lastOledRetryMs = lastHealthCheckMs;

    timerState = TimerState::Ready;
    remainingSeconds = selectedSeconds;
    potHasControl = true;
    drawHourglass(remainingSeconds);
    updateOLED(true);

    Serial.println(F("Digital Hourglass ready."));
    Serial.println(F("Turn the pot to set the time. Press remote keys to see their codes here."));
}

void loop() {
    updateStartButton();
    updatePot();
    updateIr();
    updateTimer();
    updateShakeDetection();
    updateHealth();
    updateBeeper();
    updateOLED();
}