#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <MD_MAX72xx.h>
#include <LiquidCrystal_I2C.h>
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
// - 16x2 LCD with I2C backpack (PCF8574, "I2C LCD 1602")
//
// Features:
// - Encoder counted by interrupt (no missed detents during I2C)
// - MAX7219 manual update mode; one SPI flush per frame
// - I2C at 100 kHz (the LCD backpack chip is rated for 100 kHz)
// - LCD shows the mode and time, and rotates through instructions
// - Self-test at power-up (LCD, IMU, buttons, LED matrix)
// - On-screen messages for user mistakes and hardware faults
// - IMU and LCD are re-detected automatically if unplugged
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

// The PCF8574 chip on the LCD backpack is rated for 100 kHz.
constexpr uint32_t I2C_CLOCK_HZ = 100000UL;

// ---------------- LCD ----------------

constexpr uint8_t LCD_COLUMNS = 16;
constexpr uint8_t LCD_ROWS = 2;

// Most backpacks are 0x27 (PCF8574) or 0x3F (PCF8574A). The rest of
// each chip's address range is checked too, for modules with the
// A0-A2 address pads bridged.
constexpr uint8_t LCD_ADDRESSES[16] = {
    0x27, 0x3F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
    0x26, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E
};

constexpr uint32_t LCD_REFRESH_MS = 200;
constexpr uint32_t LCD_ROTATE_MS = 2000;       // instruction lines
constexpr uint32_t NOTICE_ROTATE_MS = 1500;    // notice detail lines

LiquidCrystal_I2C* lcd = nullptr;   // created once the address is known
bool lcdAvailable = false;
uint8_t lcdAddress = 0;

// What is currently on each LCD row, so only changes are sent.
char lcdShown[LCD_ROWS][LCD_COLUMNS + 1];

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
constexpr uint32_t HOLD_HINT_AFTER_MS = 400;
constexpr uint32_t DEBOUNCE_MS = 35;
constexpr uint32_t ENCODER_DEBOUNCE_US = 1500;

// ---------------- FAULT HANDLING SETTINGS ----------------

constexpr uint32_t BUTTON_STUCK_MS = 20000;
constexpr uint32_t HEALTH_CHECK_MS = 1000;
constexpr uint32_t RECONNECT_MS = 3000;
constexpr uint32_t NOTICE_MS = 4000;
constexpr uint32_t SELF_TEST_ITEM_MS = 1200;
constexpr uint32_t SELF_TEST_FAULT_EXTRA_MS = 2000;

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
uint32_t lastLcdRetryMs = 0;

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
// A notice takes over the whole LCD for a few seconds:
//   row 1: the title (lines[0])
//   row 2: lines[1] and lines[2], alternating
// Each one is also printed to Serial, so messages are still
// visible when no LCD is fitted. Every line is 16 characters max.

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
char imuIdText[LCD_COLUMNS + 1];

void updateLCD(bool force = false);

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

    updateLCD(true);
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
// LCD CONNECTION
// ============================================================

// Forces every row to be rewritten on the next update.
void lcdInvalidate() {
    for (uint8_t row = 0; row < LCD_ROWS; row++) {
        lcdShown[row][0] = '\x01';
        lcdShown[row][1] = '\0';
    }
}

bool connectLcd() {
    for (uint8_t address : LCD_ADDRESSES) {
        if (!i2cDevicePresent(address)) {
            continue;
        }

        if (lcd == nullptr) {
            lcd = new LiquidCrystal_I2C(address, LCD_COLUMNS, LCD_ROWS);
            lcdAddress = address;
        } else if (address != lcdAddress) {
            continue;   // only the first LCD found is used
        }

        lcd->init();                    // also restarts Wire
        Wire.setClock(I2C_CLOCK_HZ);
        lcd->backlight();
        lcd->clear();
        lcdInvalidate();

        lcdAvailable = true;
        Serial.print(F("LCD found at 0x"));
        Serial.println(address, HEX);
        return true;
    }

    lcdAvailable = false;
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
                       "IMU connected", "Shake reset on", nullptr);
            beep(1, 65);
            break;

        case ImuStatus::Missing:
            showNotice(NoticeId::ImuLost,
                       "! IMU lost", "Check GY-87 wire",
                       "Shake reset off", NOTICE_MS * 2);
            errorBeep();
            break;

        case ImuStatus::WrongChip:
            snprintf(imuIdText, sizeof(imuIdText), "ID 0x%02X not 0x%02X",
                     imuChipId, IMU_EXPECTED_ID);
            showNotice(NoticeId::ImuWrongChip,
                       "! IMU wrong chip", imuIdText,
                       "Shake reset off", NOTICE_MS * 2);
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

    // LCD: the library cannot report write failures, so poll it.
    if (lcdAvailable) {
        if (!i2cDevicePresent(lcdAddress)) {
            lcdAvailable = false;
            Serial.println(F("[ERROR] LCD disconnected. Check LCD wiring."));
            beep(1, 800);
        }
    } else if (now - lastLcdRetryMs >= RECONNECT_MS) {
        lastLcdRetryMs = now;

        // A reconnected LCD has lost power, so it is fully re-initialised.
        if (connectLcd()) {
            Serial.println(F("LCD reconnected."));
            updateLCD(true);
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
// LCD DISPLAY
// ============================================================
//
// Normal screen (16 x 2):
//   row 1: "RUNNING  ! 00:45"  (mode, "!" if a fault is active, time)
//   row 2: instructions, rotating every 2 s
//
// Notices and stuck-button warnings take over both rows.

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

// Writes one row, padded to 16 characters, only if it changed.
// Avoiding lcd.clear() stops the screen flickering.
void lcdWriteRow(uint8_t row, const char* text) {
    char padded[LCD_COLUMNS + 1];
    snprintf(padded, sizeof(padded), "%-16s", text == nullptr ? "" : text);

    if (strcmp(padded, lcdShown[row]) == 0) {
        return;
    }

    lcd->setCursor(0, row);
    lcd->print(padded);
    strcpy(lcdShown[row], padded);
}

// Picks one of `count` lines, changing every `periodMs`.
uint8_t rotationIndex(uint8_t count, uint32_t periodMs, uint32_t now) {
    return static_cast<uint8_t>((now / periodMs) % count);
}

// True while START is being held towards a 5-second hold action.
bool startHoldInProgress(uint32_t now) {
    return startButtonDown && !startHoldHandled && !startStuck &&
           timerState != TimerState::Setting &&
           now - startPressedAt >= HOLD_HINT_AFTER_MS;
}

void formatHoldCountdown(char* buffer, size_t length, uint32_t now) {
    uint32_t held = now - startPressedAt;
    uint32_t left = 0;

    if (held < SETTING_HOLD_MS) {
        left = (SETTING_HOLD_MS - held + 999UL) / 1000UL;
    }

    snprintf(buffer, length, "Keep holding: %lus", static_cast<unsigned long>(left));
}

// Builds the instruction for row 2 of the normal screen.
void buildInstruction(char* buffer, size_t length, uint32_t now) {
    // While START is held for a 5 s action, show a steady countdown.
    bool holdMeansSomething =
        timerState == TimerState::Ready || timerState == TimerState::Finished ||
        !imuAvailable;

    if (holdMeansSomething && startHoldInProgress(now)) {
        formatHoldCountdown(buffer, length, now);
        return;
    }

    const char* resetHint = imuAvailable ? "Shake = reset" : "Hold START=reset";

    char totalText[LCD_COLUMNS + 1];
    char timeText[8];
    formatTime(selectedSeconds, timeText, sizeof(timeText));
    snprintf(totalText, sizeof(totalText), "Total %s", timeText);

    const char* lines[3] = {nullptr, nullptr, nullptr};

    switch (timerState) {
        case TimerState::Ready:
            lines[0] = "START = begin";
            lines[1] = "Hold START = set";
            lines[2] = imuAvailable ? "Shake = reset" : "No IMU: no shake";
            break;

        case TimerState::Running:
            lines[0] = "START = pause";
            lines[1] = resetHint;
            lines[2] = totalText;
            break;

        case TimerState::Paused:
            lines[0] = "START = resume";
            lines[1] = resetHint;
            lines[2] = totalText;
            break;

        case TimerState::Setting:
            lines[0] = "Turn = +/-10s";
            lines[1] = "Press knob=save";
            lines[2] = "Range 0:10-10:00";
            break;

        case TimerState::Finished:
            lines[0] = "TIME'S UP!";
            lines[1] = "START = again";
            lines[2] = "Hold START = set";
            break;
    }

    snprintf(buffer, length, "%s", lines[rotationIndex(3, LCD_ROTATE_MS, now)]);
}

void updateLCD(bool force) {
    if (!lcdAvailable) {
        return;
    }

    uint32_t now = millis();

    if (!force && now - lastDisplayUpdateMs < LCD_REFRESH_MS) {
        return;
    }

    lastDisplayUpdateMs = now;

    char top[LCD_COLUMNS + 1];
    char bottom[LCD_COLUMNS + 1];

    // Priority: stuck button > temporary notice > normal screen.
    if (startStuck) {
        snprintf(top, sizeof(top), "! START stuck");
        snprintf(bottom, sizeof(bottom), "%s",
                 rotationIndex(2, NOTICE_ROTATE_MS, now) == 0 ? "Release START" : "or check D5 wire");
    } else if (knobStuck) {
        snprintf(top, sizeof(top), "! KNOB stuck");
        snprintf(bottom, sizeof(bottom), "%s",
                 rotationIndex(2, NOTICE_ROTATE_MS, now) == 0 ? "Release the knob" : "or check D4 wire");
    } else if (noticeActive(now)) {
        snprintf(top, sizeof(top), "%s", notice.lines[0]);

        const char* detail = notice.lines[1];
        if (notice.lines[2] != nullptr &&
            rotationIndex(2, NOTICE_ROTATE_MS, now - notice.shownAt + NOTICE_ROTATE_MS * 2) == 1) {
            detail = notice.lines[2];
        }
        snprintf(bottom, sizeof(bottom), "%s", detail == nullptr ? "" : detail);
    } else {
        uint32_t displaySeconds = selectedSeconds;

        if (timerState == TimerState::Running || timerState == TimerState::Paused) {
            displaySeconds = remainingSeconds;
        }

        char timeText[8];
        formatTime(displaySeconds, timeText, sizeof(timeText));

        // e.g. "RUNNING  ! 00:45" -> mode padded to 9, fault mark, time.
        snprintf(top, sizeof(top), "%-9s%c %s", stateName(),
                 hardwareFaultActive() ? '!' : ' ', timeText);

        buildInstruction(bottom, sizeof(bottom), now);
    }

    lcdWriteRow(0, top);
    lcdWriteRow(1, bottom);
}

// ============================================================
// SELF-TEST (power-up)
// ============================================================

void runSelfTest() {
    bool faultFound = hardwareFaultActive() || !lcdAvailable;

    // Serial report.
    Serial.println(F("---- SELF TEST ----"));
    Serial.print(F("LCD:   "));
    if (lcdAvailable) {
        Serial.print(F("OK at 0x"));
        Serial.println(lcdAddress, HEX);
    } else {
        Serial.println(F("NOT FOUND (check SDA/SCL, 5V, GND)"));
    }

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

    // Sound: one long beep if there is no screen to read,
    // the error chirp for any other fault, one short beep if all OK.
    if (!lcdAvailable) {
        beep(1, 800);
    } else if (faultFound) {
        errorBeep();
    } else {
        beep(1, 65);
    }

    // LCD report: title on row 1, one result at a time on row 2.
    constexpr uint8_t ITEM_COUNT = 5;
    char items[ITEM_COUNT][LCD_COLUMNS + 1];

    snprintf(items[0], sizeof(items[0]), "LCD   OK  0x%02X", lcdAddress);

    switch (imuStatus) {
        case ImuStatus::Ok:
            snprintf(items[1], sizeof(items[1]), "IMU   OK  0x%02X", imuAddress);
            break;
        case ImuStatus::Missing:
            snprintf(items[1], sizeof(items[1]), "IMU   NOT FOUND");
            break;
        case ImuStatus::WrongChip:
            snprintf(items[1], sizeof(items[1]), "IMU   BAD ID %02X", imuChipId);
            break;
    }

    snprintf(items[2], sizeof(items[2]), "%s", startStuck ? "START STUCK" : "START OK");
    snprintf(items[3], sizeof(items[3]), "%s", knobStuck ? "KNOB  STUCK" : "KNOB  OK");
    snprintf(items[4], sizeof(items[4]), "All LEDs lit?");

    if (lcdAvailable) {
        lcdWriteRow(0, faultFound ? "SELF TEST: FAULT" : "SELF TEST: PASS");
    }

    uint32_t totalMs = ITEM_COUNT * SELF_TEST_ITEM_MS +
                       (faultFound ? SELF_TEST_FAULT_EXTRA_MS : 0);
    uint32_t start = millis();

    while (millis() - start < totalMs) {
        updateBeeper();

        if (lcdAvailable) {
            uint8_t item = static_cast<uint8_t>((millis() - start) / SELF_TEST_ITEM_MS);
            if (item >= ITEM_COUNT) {
                item = ITEM_COUNT - 1;
            }
            lcdWriteRow(1, items[item]);
        }
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

    updateLCD(true);
}

void startTimer() {
    remainingSeconds = selectedSeconds;
    timerState = TimerState::Running;
    lastTimerTickMs = millis();
    pausedRemainderMs = 0;

    clearNotice();
    beep(2, 65, 100);

    drawHourglass(remainingSeconds);
    updateLCD(true);
}

void pauseTimer() {
    timerState = TimerState::Paused;

    // Keep the part of the current second already elapsed.
    pausedRemainderMs = millis() - lastTimerTickMs;

    clearNotice();
    beep(1, 180);
    updateLCD(true);
}

void resumeTimer() {
    timerState = TimerState::Running;
    lastTimerTickMs = millis() - pausedRemainderMs;
    pausedRemainderMs = 0;

    clearNotice();
    beep(2, 65, 100);
    updateLCD(true);
}

void finishTimer() {
    timerState = TimerState::Finished;
    remainingSeconds = 0;

    drawHourglass(0);
    beep(3, 450, 180);
    updateLCD(true);
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
                           "! Use the knob", "Turn = adjust",
                           "Press = save")) {
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
    updateLCD(true);
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
    updateLCD(true);
}

void exitSettingMode() {
    timerState = TimerState::Ready;
    remainingSeconds = selectedSeconds;

    clearNotice();
    drawHourglass(remainingSeconds);
    beep(1, 300);
    updateLCD(true);
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
               "Time auto-saved", "No input 30s", "START = begin");
}

// ============================================================
// USER-MISTAKE NOTICES
// ============================================================

void showKnobLockedNotice() {
    bool isNew;

    if (timerState == TimerState::Running || timerState == TimerState::Paused) {
        if (imuAvailable) {
            isNew = showNotice(NoticeId::KnobLocked, "! Knob is locked",
                               "Shake to reset,", "then hold START");
        } else {
            isNew = showNotice(NoticeId::KnobLocked, "! Knob is locked",
                               "Hold START 5s", "to reset first");
        }
    } else {
        isNew = showNotice(NoticeId::KnobLocked, "! Knob is locked",
                           "Hold START 5s", "to set the time");
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
            updateLCD(true);
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
        updateLCD(true);
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
                               "! Can't set now", "Shake to reset,",
                               "then hold START")) {
                    errorBeep();
                }
            } else {
                // No IMU means no shake reset, so a long hold resets.
                resetTimer(true);
                showNotice(NoticeId::TimerReset,
                           "Timer reset", "START = begin", "Hold START = set");
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

    if (newTime < static_cast<int32_t>(MIN_TIME_SECONDS)) {
        newTime = static_cast<int32_t>(MIN_TIME_SECONDS);
    }

    if (newTime > static_cast<int32_t>(MAX_TIME_SECONDS)) {
        newTime = static_cast<int32_t>(MAX_TIME_SECONDS);
    }

    if (static_cast<uint32_t>(newTime) == selectedSeconds) {
        // Already at a limit and turning further into it.
        bool isNew;

        if (selectedSeconds >= MAX_TIME_SECONDS) {
            isNew = showNotice(NoticeId::LimitReached, "! Max is 10:00",
                               "Turn other way", "Press knob=save");
        } else {
            isNew = showNotice(NoticeId::LimitReached, "! Min is 00:10",
                               "Turn other way", "Press knob=save");
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
    updateLCD(true);
}

void updateEncoderSwitch() {
    uint32_t now = millis();
    bool pressed = digitalRead(ENCODER_SW_PIN) == LOW;

    if (pressed != encoderSwitchDown && now - lastEncoderSwitchEdge >= DEBOUNCE_MS) {
        lastEncoderSwitchEdge = now;
        encoderSwitchDown = pressed;

        if (pressed) {
            encoderSwitchPressedAt = now;

            if (timerState == TimerState::Setting) {
                exitSettingMode();
            } else {
                showKnobLockedNotice();
            }
        } else if (knobStuck) {
            knobStuck = false;
            Serial.println(F("KNOB released. Fault cleared."));
            updateLCD(true);
        }
    }

    // Held far longer than any real use: treat as stuck or shorted.
    if (pressed && !knobStuck && now - encoderSwitchPressedAt >= BUTTON_STUCK_MS) {
        knobStuck = true;
        Serial.println(F("[ERROR] KNOB held for 20s. Stuck button or D4 shorted to GND?"));
        errorBeep();
        updateLCD(true);
    }
}

// ============================================================
// GY-87 SHAKE DETECTION
// ============================================================

// Reads the accelerometer straight from the registers so that a
// failed I2C read can be told apart from a real measurement.
bool readAccelerationMagnitude(float& magnitude) {
    Wire.beginTransmission(imuAddress);
    Wire.write(IMU_REG_ACCEL_XOUT_H);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(imuAddress, static_cast<size_t>(6)) != 6) {
        return false;
    }

    int16_t raw[3];

    for (uint8_t i = 0; i < 3; i++) {
        uint8_t high = static_cast<uint8_t>(Wire.read());
        uint8_t low = static_cast<uint8_t>(Wire.read());
        raw[i] = static_cast<int16_t>((high << 8) | low);
    }

    // All zeros = chip reset or lost power (it reads 0 until re-configured).
    if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0) {
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
    if (!imuAvailable) {
        return false;
    }

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
        bool wasActive = (timerState != TimerState::Ready);

        resetTimer(false);
        beep(2, 180, 150);

        if (wasActive) {
            showNotice(NoticeId::TimerReset,
                       "Reset by shake", "START = begin", "Hold START = set");
        }
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

    if (!connectLcd()) {
        Serial.println(F("LCD not found. Messages go to Serial only."));
    }

    setImuStatus(connectImu(), false);

    // A button held at power-up is reported as stuck until released,
    // and that press is never acted on.
    uint32_t now = millis();

    if (digitalRead(START_BUTTON_PIN) == LOW) {
        startButtonDown = true;
        startHoldHandled = true;
        startStuck = true;
        startPressedAt = now;
        lastStartEdge = now;
    }

    if (digitalRead(ENCODER_SW_PIN) == LOW) {
        encoderSwitchDown = true;
        knobStuck = true;
        encoderSwitchPressedAt = now;
        lastEncoderSwitchEdge = now;
    }

    runSelfTest();

    lastHealthCheckMs = millis();
    lastImuRetryMs = lastHealthCheckMs;
    lastLcdRetryMs = lastHealthCheckMs;

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, FALLING);
    handledEncoderPosition = readEncoderPosition();

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
    updateSettingTimeout();

    updateTimer();
    updateShakeDetection();
    updateHealth();

    updateBeeper();
    updateLCD();
}