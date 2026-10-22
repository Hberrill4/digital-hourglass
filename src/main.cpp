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
// - BRIGHTNESS button on D4 (also remote CH- / CH+)
// - Passive buzzer on D6 (tone at BUZZER_FREQ_HZ)
// - GY-87 IMU (MPU6050) for shake-to-reset
// - SSD1306 128x64 I2C OLED
//
// Controls:
// - Turn the pot (READY / FINISHED): time follows the pot, 0:10-10:00 in 10 s steps
// - START or remote PLAY/OK: start / pause / resume
// - Hold START 5 s, shake, or remote RESET: reset the timer
// - Remote digits then OK: type an exact time (1, 3, 0, OK = 1:30)
// - Remote +/- : 10 s, remote PREV/NEXT : 1 min, remote CH: food timers
//
// Food timers: CH picks a food. Its name is beeped in Morse code, the
// matrices show the food and an hourglass, and while the timer runs the
// food drains from the top chamber and rebuilds in the bottom one.
//
// Pot "takeover": after the remote sets a time, the pot is ignored
// until it is turned, so the time never jumps by itself.
// ============================================================

// ---------------- PIN ASSIGNMENTS ----------------

constexpr uint8_t MATRIX_CS_PIN = 10;   // DIN = D11 (MOSI), CLK = D13 (SCK)
constexpr uint8_t START_BUTTON_PIN = 5; // button to GND
constexpr uint8_t BRIGHT_BUTTON_PIN = 4; // button to GND
constexpr uint8_t BUZZER_PIN = 6;       // passive buzzer (directly, or via the NPN driver)
constexpr unsigned int BUZZER_FREQ_HZ = 2300;   // passive buzzers need a tone; ~2-3 kHz is loudest
constexpr uint8_t IR_RECEIVE_PIN = 8;   // VS1838B OUT
constexpr uint8_t POT_PIN = A0;         // potentiometer wiper
// I2C: SDA = A4 / SDA header, SCL = A5 / SCL header

// ---------------- MATRIX ----------------

constexpr uint8_t MATRIX_COUNT = 2;
// Four brightness levels for the matrices (MAX7219 0-15) and the OLED contrast.
// Level 4 stops at 12/15 to stay inside the USB power budget.
constexpr uint8_t BRIGHTNESS_LEVELS = 4;
const uint8_t MATRIX_INTENSITY[BRIGHTNESS_LEVELS] = {1, 3, 7, 12};
const uint8_t OLED_CONTRAST[BRIGHTNESS_LEVELS] = {8, 60, 150, 255};
uint8_t brightnessIndex = 1;            // start at level 2 (the old fixed setting)
MD_MAX72XX matrix(MD_MAX72XX::FC16_HW, MATRIX_CS_PIN, MATRIX_COUNT);

// Physical mounting of the two 8x8 modules (fix here if the self-test arrows don't point up):
//   The matrices are mounted as diamonds (45 degrees), corners meeting at the neck.
//   UPPER_MODULE: which module is the top diamond (0 = first in the chain, 1 = second)
//   UPPER_ROTATION / LOWER_ROTATION: clockwise quarter-turns (0-3) for each module, chosen
//   so the self-test triangle sits in the BOTTOM corner of each diamond.
constexpr uint8_t UPPER_MODULE = 1;
constexpr uint8_t UPPER_ROTATION = 0;
constexpr uint8_t LOWER_ROTATION = 0;

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
enum class RemoteLayout : uint8_t { Mp3_21Key, Arrows_17Key };
constexpr RemoteLayout REMOTE_LAYOUT = RemoteLayout::Mp3_21Key;

enum class RemoteKey : uint8_t {
    None, Digit0, Digit1, Digit2, Digit3, Digit4, Digit5, Digit6, Digit7, Digit8, Digit9,
    PlayOk, Plus10, Minus10, PlusMin, MinusMin, Reset, Preset, Clear, BrightCycle, Mute, Restart
};

struct KeyCode { uint8_t command; RemoteKey key; };

// "SPECIAL FOR MP3" 21-key remote:
//   Power  Mode   Mute
//   >||    |<<    >>|
//   EQ     -      +
//   0      Repeat U/SD
//   1-9
const KeyCode MP3_KEYS[] = {
    {0x16, RemoteKey::Digit0}, {0x0C, RemoteKey::Digit1}, {0x18, RemoteKey::Digit2}, {0x5E, RemoteKey::Digit3},
    {0x08, RemoteKey::Digit4}, {0x1C, RemoteKey::Digit5}, {0x5A, RemoteKey::Digit6}, {0x42, RemoteKey::Digit7},
    {0x52, RemoteKey::Digit8}, {0x4A, RemoteKey::Digit9},
    {0x45, RemoteKey::Reset},       // Power
    {0x46, RemoteKey::Preset},      // Mode   -> next food timer
    {0x47, RemoteKey::Mute},        // Mute   -> sound on/off
    {0x44, RemoteKey::PlayOk},      // >||
    {0x40, RemoteKey::MinusMin},    // |<<    -> -1 min
    {0x43, RemoteKey::PlusMin},     // >>|    -> +1 min
    {0x07, RemoteKey::BrightCycle}, // EQ     -> brightness
    {0x15, RemoteKey::Minus10},     // -
    {0x09, RemoteKey::Plus10},      // +
    {0x19, RemoteKey::Restart},     // Repeat -> restart same time
    {0x0D, RemoteKey::Clear},       // U/SD   -> cancel typing
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

const char* RESET_KEY_NAME = (REMOTE_LAYOUT == RemoteLayout::Mp3_21Key) ? "PWR" : "#";

// ---------------- FOOD TIMERS ----------------
// 8x8 pictures, top row first, leftmost pixel = highest bit.
struct Food {
    const char* name;       // shown on the OLED (max 11 characters)
    const char* morse;      // word beeped in Morse code
    uint32_t seconds;       // normal cooking time
    uint8_t bitmap[8];
};

const Food FOODS[] = {
    // Egg sitting in an egg cup
    {"Soft egg",    "EGG",      360, {0b00011000, 0b00111100, 0b01111110, 0b01111110,
                                      0b11111111, 0b01111110, 0b00011000, 0b00111100}},
    // Fork with spaghetti twirled round it
    {"Spaghetti",   "PASTA",    600, {0b01010100, 0b01010100, 0b01111100, 0b00111000,
                                      0b01111110, 0b11011011, 0b01111110, 0b00010000}},
    // Noodle cup with chopsticks
    {"Noodles",     "NOODLES",  180, {0b00000011, 0b00001100, 0b00110000, 0b11111111,
                                      0b01111110, 0b01011010, 0b01111110, 0b00111100}},
    // Striped bucket overflowing with popcorn
    {"Popcorn",     "POPCORN",  150, {0b01001010, 0b10110101, 0b11111111, 0b01111110,
                                      0b01011010, 0b01011010, 0b00110100, 0b00111100}},
    // Broccoli: bumpy floret on a branching stalk
    {"Broccoli",    "BROCCOLI", 240, {0b01100110, 0b11111111, 0b11111111, 0b01111110,
                                      0b00100100, 0b00011000, 0b00011000, 0b00011000}},
    // Corn cob with rows of kernels and husk leaves
    {"Corn on cob", "CORN",     300, {0b00011000, 0b00110100, 0b00101100, 0b00110100,
                                      0b00101100, 0b01011010, 0b10111101, 0b00011000}},
};
constexpr uint8_t FOOD_COUNT = sizeof(FOODS) / sizeof(FOODS[0]);

const uint8_t HOURGLASS_ICON[8] = {0b11111111, 0b01000010, 0b00100100, 0b00011000,
                                   0b00011000, 0b00100100, 0b01011010, 0b11111111};
constexpr uint32_t FOOD_SPLASH_MS = 2500;

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

// ---------------- FLIP DETECTION ----------------
// Gravity is recorded at power-up as "upright". When it points the other
// way for FLIP_STABLE_MS the hourglass has been turned upside down.
constexpr float FLIP_COS_THRESHOLD = 0.82f;  // must be tilted ~145 degrees to count as a flip
constexpr uint32_t FLIP_STABLE_MS = 700;     // and held there, still, for this long
constexpr float FLIP_STILL_MIN = 7.5f;       // m/s^2: only near 1 g counts as 'held still'
constexpr float FLIP_STILL_MAX = 12.0f;
bool displayFlipped = false;
bool flipReferenceValid = false;
float refAx = 0, refAy = 0, refAz = 0;
float accX = 0, accY = 0, accZ = 0;
bool accFresh = false;
uint32_t flipCandidateSince = 0;
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

bool brightButtonDown = false;
uint32_t lastBrightEdge = 0;
int8_t grainRow = 0;             // falling-grain animation (standard timer)
uint32_t lastAnimMs = 0;
uint32_t finishedAtMs = 0;
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
uint8_t nextFood = 0;            // the food CH selects next
int8_t activeFood = -1;          // -1 = plain sand, else index into FOODS
bool splashActive = false;
uint32_t splashUntilMs = 0;
uint32_t lastIrActionMs = 0;
RemoteKey lastIrKey = RemoteKey::None;

uint32_t lastHealthCheckMs = 0;
uint32_t lastImuRetryMs = 0;
uint32_t lastOledRetryMs = 0;

// ============================================================
// BUZZER (non-blocking sequence player)
// ============================================================
// A sound is a list of (on ms, off ms) steps played in the background.
// beep() plays N identical steps; morseAppend() adds a word in Morse.

constexpr uint8_t BEEP_MAX_STEPS = 80;

// A passive buzzer has no oscillator, so it is driven with a square wave.
bool soundMuted = false;          // remote Mute key
void buzzerOn()  { if (!soundMuted) tone(BUZZER_PIN, BUZZER_FREQ_HZ); }
void buzzerOff() { noTone(BUZZER_PIN); digitalWrite(BUZZER_PIN, LOW); }
constexpr uint16_t MORSE_DOT_MS = 60;      // ~20 words per minute

struct Beeper {
    uint16_t onMs[BEEP_MAX_STEPS];
    uint16_t offMs[BEEP_MAX_STEPS];
    uint8_t count = 0;
    uint8_t index = 0;
    uint32_t nextChangeMs = 0;
    bool active = false, sounding = false;
};
Beeper beeper;

// International Morse code, A-Z
const char* const MORSE_CODE[26] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..", "--",
    "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--.."
};

void stopBeeper() {
    beeper.active = false;
    beeper.sounding = false;
    buzzerOff();
}

void beepClear() {
    stopBeeper();
    beeper.count = 0;
}

void beepAppend(uint16_t onMs, uint16_t offMs) {
    if (beeper.count >= BEEP_MAX_STEPS) return;
    beeper.onMs[beeper.count] = onMs;
    beeper.offMs[beeper.count] = offMs;
    beeper.count++;
}

void beepStart() {
    if (beeper.count == 0) return;
    beeper.index = 0;
    beeper.active = true;
    beeper.sounding = true;
    buzzerOn();
    beeper.nextChangeMs = millis() + beeper.onMs[0];
}

void beep(uint16_t count, uint16_t onMs, uint16_t offMs = 100) {
    beepClear();
    for (uint16_t i = 0; i < count; i++) beepAppend(onMs, offMs);
    beepStart();
}

// Dot = 1 unit, dash = 3, gap inside a letter = 1, between letters = 3.
void morseAppend(const char* word) {
    for (const char* p = word; *p; p++) {
        char c = *p;
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c < 'A' || c > 'Z') continue;
        const char* code = MORSE_CODE[c - 'A'];
        for (const char* e = code; *e; e++) {
            uint16_t on = (*e == '.') ? MORSE_DOT_MS : 3 * MORSE_DOT_MS;
            uint16_t off = (e[1] != '\0') ? MORSE_DOT_MS : 3 * MORSE_DOT_MS;
            beepAppend(on, off);
        }
    }
}

void errorBeep() { beep(3, 40, 40); }   // three fast chirps = "no"
void tick() { beep(1, 12); }            // tiny click for pot steps / key presses

void updateBeeper() {
    if (!beeper.active) return;
    if (static_cast<int32_t>(millis() - beeper.nextChangeMs) < 0) return;

    if (beeper.sounding) {
        buzzerOff();
        beeper.sounding = false;
        if (beeper.index + 1 >= beeper.count) {
            beeper.active = false;
            return;
        }
        beeper.nextChangeMs = millis() + beeper.offMs[beeper.index];
    } else {
        beeper.index++;
        buzzerOn();
        beeper.sounding = true;
        beeper.nextChangeMs = millis() + beeper.onMs[beeper.index];
    }
}

// ============================================================
// ON-SCREEN NOTICES
// ============================================================

enum class NoticeId : uint8_t {
    None, CantChange, InvalidTime, OutOfRange, TimeSet, FoodSet, Cancelled,
    UnknownKey, PotLocked, TimerReset, ImuConnected, ImuLost, ImuWrongChip, Brightness, Sound, Flipped
};

struct Notice {
    NoticeId id = NoticeId::None;
    const char* lines[3] = {nullptr, nullptr, nullptr};
    uint32_t shownAt = 0, durationMs = 0;
};
Notice notice;
char noticeText[3][22];       // for notices with numbers or names in them

void updateOLED(bool force = false);
void drawHourglass(uint32_t secondsLeft);
void changeBrightness(int8_t direction, bool wrap);
void captureUprightReference();

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
            oled.ssd1306_command(SSD1306_SETCONTRAST);
            oled.ssd1306_command(OLED_CONTRAST[brightnessIndex]);
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
            captureUprightReference();
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

// chamber 0 = lower (columns 0-7), 1 = upper (columns 8-15); x 0 = left, y 0 = top
// Every picture is drawn through here. Upside down = a true 180-degree turn:
// the chambers swap and each picture is mirrored left-right and top-bottom.
void setChamberPixel(uint8_t chamber, uint8_t x, uint8_t y, bool on) {
    if (displayFlipped) {
        chamber = 1 - chamber;
        x = 7 - x;
        y = 7 - y;
    }
    uint8_t turns = (chamber == 1) ? UPPER_ROTATION : LOWER_ROTATION;
    for (uint8_t turn = 0; turn < turns; turn++) {             // compensate for module mounting
        uint8_t t = x;
        x = 7 - y;
        y = t;
    }
    uint8_t module = (chamber == 1) ? UPPER_MODULE : 1 - UPPER_MODULE;
    setPixel(y, module * 8 + (7 - x), on);
}

bool bitmapPixel(const uint8_t bitmap[8], uint8_t x, uint8_t y) {
    return bitmap[y] & (0x80 >> x);
}

// The food is the sand: it drains from the top of the upper chamber
// and rebuilds from the bottom of the lower chamber.
void drawFoodHourglass(const Food& food, uint32_t secondsLeft) {
    uint8_t pixels = 0;
    for (uint8_t y = 0; y < 8; y++)
        for (uint8_t x = 0; x < 8; x++) pixels += bitmapPixel(food.bitmap, x, y);

    uint32_t elapsed = selectedSeconds - secondsLeft;
    uint8_t moved = static_cast<uint8_t>((elapsed * pixels) / selectedSeconds);

    uint8_t k = 0;
    for (uint8_t y = 0; y < 8; y++)                       // upper: top rows leave first
        for (uint8_t x = 0; x < 8; x++)
            if (bitmapPixel(food.bitmap, x, y)) setChamberPixel(1, x, y, k++ >= moved);

    k = 0;
    for (int8_t y = 7; y >= 0; y--)                       // lower: bottom rows arrive first
        for (uint8_t x = 0; x < 8; x++)
            if (bitmapPixel(food.bitmap, x, y)) setChamberPixel(0, x, y, k++ < moved);
}

// ---- Standard timer: shaped glass, sand that drains with a dip,
// ---- a mound that builds up, a falling grain, and a sparkle finish.
// Diamond hourglass: each matrix is mounted at 45 degrees, so "down" is the
// diagonal towards its bottom corner, logical cell (7,7). Sand fills by distance
// from that corner, so its surface is always a level diagonal line; within each
// level the middle fills first, giving a mound. The two bottom/top corners meet
// at the neck. All 64 LEDs of each matrix hold sand.
constexpr uint8_t GLASS_PIXELS = 64;
constexpr uint32_t ANIM_MS = 80;
constexpr uint32_t SPARKLE_MS = 6000;

uint8_t sandX[GLASS_PIXELS], sandY[GLASS_PIXELS];   // fill order from the bottom corner
uint8_t sandIndex[8][8];                            // position of each cell in that order

void buildGlassOrder() {
    uint8_t k = 0;
    for (uint8_t level = 0; level <= 14; level++) {              // distance from the bottom corner
        for (uint8_t offset = 0; offset <= 7; offset++) {        // middle of the level first
            for (int8_t side = 1; side >= -1; side -= 2) {
                // cells on this level: (7-a, 7-b) with a+b = level; centre is a == b
                int a2 = level + side * offset;                  // a - b = side*offset  ->  a = (level + side*offset)/2
                if (a2 % 2 != 0 || a2 < 0) continue;
                int a = a2 / 2, b = level - a;
                if (a > 7 || b > 7 || b < 0) continue;
                uint8_t x = static_cast<uint8_t>(7 - a), y = static_cast<uint8_t>(7 - b);
                sandX[k] = x; sandY[k] = y; sandIndex[y][x] = k; k++;
                if (offset == 0) break;                          // the centre cell exists once
            }
        }
    }
}

uint8_t sandMovedFor(uint32_t secondsLeft) {
    uint32_t elapsed = selectedSeconds - secondsLeft;
    return static_cast<uint8_t>((elapsed * GLASS_PIXELS) / selectedSeconds);
}

// The grain falls from the neck corner (0,0) of the lower diamond straight down
// its centre diagonal. Returns the last empty step above the pile (-1 = full).
int8_t grainLandingStep(uint8_t moved) {
    for (int8_t step = 7; step >= 0; step--)
        if (sandIndex[step][step] >= moved) return step;
    return -1;
}

void drawStandardHourglass(uint32_t secondsLeft) {
    uint8_t moved = sandMovedFor(secondsLeft);
    for (uint8_t k = 0; k < GLASS_PIXELS; k++) {
        setChamberPixel(1, sandX[k], sandY[k], k < GLASS_PIXELS - moved);   // upper: rests in its bottom corner
        setChamberPixel(0, sandX[k], sandY[k], k < moved);                  // lower: piles up from its bottom corner
    }

    if (timerState == TimerState::Running && moved < GLASS_PIXELS) {
        int8_t landing = grainLandingStep(moved);
        if (landing >= 0 && grainRow >= 0 && grainRow <= landing)
            setChamberPixel(0, grainRow, grainRow, true);
    }

    if (timerState == TimerState::Finished && millis() - finishedAtMs < SPARKLE_MS) {
        for (uint8_t i = 0; i < 5; i++) {
            uint8_t k = static_cast<uint8_t>(random(GLASS_PIXELS));
            setChamberPixel(0, sandX[k], sandY[k], false);              // twinkle
        }
    }
}

// Runs the falling grain while counting and the sparkle after finishing.
void updateMatrixAnimation() {
    if (activeFood >= 0 || splashActive) return;
    bool running = timerState == TimerState::Running;
    bool sparkling = timerState == TimerState::Finished && millis() - finishedAtMs < SPARKLE_MS;
    bool sparkleJustEnded = timerState == TimerState::Finished && !sparkling &&
                            millis() - finishedAtMs < SPARKLE_MS + ANIM_MS * 2;
    if (!running && !sparkling && !sparkleJustEnded) return;

    uint32_t now = millis();
    if (now - lastAnimMs < ANIM_MS) return;
    lastAnimMs = now;

    if (running) {
        int8_t landing = grainLandingStep(sandMovedFor(remainingSeconds));
        grainRow = (landing < 0 || grainRow >= landing) ? 0 : grainRow + 1;
    }
    drawHourglass(running ? remainingSeconds : 0);
}

void drawHourglass(uint32_t secondsLeft) {
    if (splashActive) return;                             // splash stays up until it ends
    matrix.clear();
    if (secondsLeft > selectedSeconds) secondsLeft = selectedSeconds;

    if (activeFood >= 0) {
        drawFoodHourglass(FOODS[activeFood], secondsLeft);
        matrix.update();
        return;
    }

    drawStandardHourglass(secondsLeft);
    matrix.update();
}

// Food picture in the upper chamber, an hourglass in the lower one.
void showFoodSplash(const Food& food) {
    matrix.clear();
    for (uint8_t y = 0; y < 8; y++) {
        for (uint8_t x = 0; x < 8; x++) {
            setChamberPixel(1, x, y, bitmapPixel(food.bitmap, x, y));
            setChamberPixel(0, x, y, bitmapPixel(HOURGLASS_ICON, x, y));
        }
    }
    matrix.update();
    splashActive = true;
    splashUntilMs = millis() + FOOD_SPLASH_MS;
}

void endSplash() {
    splashActive = false;
}

void updateSplash() {
    if (splashActive && static_cast<int32_t>(millis() - splashUntilMs) >= 0) {
        splashActive = false;
        drawHourglass(remainingSeconds);
    }
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
            if (activeFood >= 0) {
                snprintf(buf, sizeof(buf), "Food: %s", FOODS[activeFood].name);
                printLine(2, buf);
            } else {
                printLine(2, "Remote: 0-9 then OK");
            }
            break;
        case TimerState::Running:
        case TimerState::Paused:
            printLine(0, timerState == TimerState::Running ? "START/OK: pause" : "START/OK: resume");
            printLine(1, resetLine);
            formatTime(selectedSeconds, buf, sizeof(buf));
            {
                char total[22];
                snprintf(total, sizeof(total), "%s %s",
                         activeFood >= 0 ? FOODS[activeFood].name : "Total", buf);
                printLine(2, total);
            }
            break;
        case TimerState::Finished:
            if (activeFood >= 0) {
                snprintf(buf, sizeof(buf), "%s is ready!", FOODS[activeFood].name);
                printLine(0, buf);
            } else {
                printLine(0, "TIME'S UP!");
            }
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

    // Food timer finished: big "TIME TO EAT!" instead of 00:00
    if (timerState == TimerState::Finished && activeFood >= 0 && !entryActive &&
        !startStuck && !noticeActive(now)) {
        oled.setTextSize(2);
        oled.setCursor(22, 13);
        oled.print(F("TIME TO"));
        oled.setCursor(40, 31);
        oled.print(F("EAT!"));
        oled.setTextSize(1);
        char line[22];
        if ((now / 2000) % 2 == 0) snprintf(line, sizeof(line), "%s is ready", FOODS[activeFood].name);
        else snprintf(line, sizeof(line), "START/OK: run again");
        printLine(2, line);
        oled.display();
        return;
    }

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
        printLine(2, REMOTE_LAYOUT == RemoteLayout::Mp3_21Key ? "U/SD: cancel" : "*: cancel");
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
    endSplash();
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
    endSplash();
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
    finishedAtMs = millis();
    remainingSeconds = 0;
    lockPot();
    drawHourglass(0);
    beepClear();                                    // three long beeps ...
    beepAppend(450, 180);
    beepAppend(450, 180);
    beepAppend(450, 600);
    if (activeFood >= 0) morseAppend(FOODS[activeFood].morse);   // ... then the food in Morse
    beepStart();
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
// Any time that is not a food timer switches back to plain sand.
void applySelectedTime(uint32_t seconds, bool fromRemote, bool keepFood = false) {
    if (!keepFood) activeFood = -1;
    endSplash();
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

const char* keyName(RemoteKey key) {
    switch (key) {
        case RemoteKey::PlayOk:      return "START/PAUSE/OK";
        case RemoteKey::Plus10:      return "+10 s";
        case RemoteKey::Minus10:     return "-10 s";
        case RemoteKey::PlusMin:     return "+1 min";
        case RemoteKey::MinusMin:    return "-1 min";
        case RemoteKey::Reset:       return "RESET";
        case RemoteKey::Restart:     return "RESTART";
        case RemoteKey::Preset:      return "NEXT FOOD";
        case RemoteKey::Clear:       return "CANCEL TYPING";
        case RemoteKey::BrightCycle: return "BRIGHTNESS";
        case RemoteKey::Mute:        return "MUTE";
        case RemoteKey::None:        return "(not mapped)";
        default:                     return "DIGIT";
    }
}

RemoteKey lookupKey(uint8_t command) {
    if (REMOTE_LAYOUT == RemoteLayout::Mp3_21Key) {
        for (const KeyCode& k : MP3_KEYS) if (k.command == command) return k.key;
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
            uint8_t index = nextFood;
            nextFood = (nextFood + 1) % FOOD_COUNT;
            const Food& food = FOODS[index];

            activeFood = static_cast<int8_t>(index);
            applySelectedTime(food.seconds, true, true);
            showFoodSplash(food);

            char t[8];
            formatTime(food.seconds, t, sizeof(t));
            snprintf(noticeText[0], sizeof(noticeText[0]), "Food: %s", food.name);
            snprintf(noticeText[1], sizeof(noticeText[1]), "%s  (MODE = next)", t);
            snprintf(noticeText[2], sizeof(noticeText[2]), "Morse: %s", food.morse);
            showNotice(NoticeId::FoodSet, noticeText[0], noticeText[1], noticeText[2], FOOD_SPLASH_MS);

            beepClear();                            // beep the food's name in Morse
            morseAppend(food.morse);
            beepStart();
            break;
        }

        case RemoteKey::BrightCycle:
            changeBrightness(+1, true);
            break;

        case RemoteKey::Mute:
            soundMuted = !soundMuted;
            if (soundMuted) {
                stopBeeper();
                showNotice(NoticeId::Sound, "Sound off", "Mute again: sound on", nullptr, 1500);
            } else {
                showNotice(NoticeId::Sound, "Sound on", nullptr, nullptr, 1500);
                tick();
            }
            break;

        case RemoteKey::Restart:
            cancelEntry(false);
            resetTimer(false);
            startTimer();
            showNotice(NoticeId::TimerReset, "Restarted", "Same time again", nullptr, 1500);
            break;

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
            Serial.print(F("  -> "));
            Serial.println(keyName(key));
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
// BRIGHTNESS (D4 button, remote CH- / CH+)
// ============================================================

void changeBrightness(int8_t direction, bool wrap) {
    int8_t i = static_cast<int8_t>(brightnessIndex) + direction;
    if (i >= BRIGHTNESS_LEVELS) i = wrap ? 0 : BRIGHTNESS_LEVELS - 1;
    if (i < 0) i = 0;
    brightnessIndex = static_cast<uint8_t>(i);

    matrix.control(MD_MAX72XX::INTENSITY, MATRIX_INTENSITY[brightnessIndex]);
    if (oledAvailable) {
        oled.ssd1306_command(SSD1306_SETCONTRAST);
        oled.ssd1306_command(OLED_CONTRAST[brightnessIndex]);
    }

    snprintf(noticeText[0], sizeof(noticeText[0]), "Brightness %u of %u",
             brightnessIndex + 1, BRIGHTNESS_LEVELS);
    char bar[17];
    for (uint8_t b = 0; b < 16; b++) bar[b] = (b < (brightnessIndex + 1) * 4) ? '#' : '.';
    bar[16] = '\0';
    snprintf(noticeText[1], sizeof(noticeText[1]), "[%s]", bar);
    showNotice(NoticeId::Brightness, noticeText[0], noticeText[1], "D4 button or EQ", 1500);
    tick();
}

void updateBrightButton() {
    uint32_t now = millis();
    bool pressed = digitalRead(BRIGHT_BUTTON_PIN) == LOW;
    if (pressed != brightButtonDown && now - lastBrightEdge >= DEBOUNCE_MS) {
        lastBrightEdge = now;
        brightButtonDown = pressed;
        if (pressed) changeBrightness(+1, true);        // each press steps up, then wraps to 1
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
    accX = ax; accY = ay; accZ = az;
    accFresh = true;
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
// FLIP (turn the hourglass upside down)
// ============================================================

void captureUprightReference() {
    flipReferenceValid = false;
    if (!imuAvailable) return;
    float sx = 0, sy = 0, sz = 0, m;
    uint8_t n = 0;
    for (uint8_t i = 0; i < 10; i++) {
        if (readAccelerationMagnitude(m)) { sx += accX; sy += accY; sz += accZ; n++; }
        delay(5);
    }
    if (n < 5) return;
    refAx = sx / n; refAy = sy / n; refAz = sz / n;
    float g = sqrtf(refAx * refAx + refAy * refAy + refAz * refAz);
    flipReferenceValid = g > 5.0f && g < 15.0f;          // roughly 1 g, i.e. sitting still
    resetShakeState();
}

void handleFlip(bool nowFlipped) {
    displayFlipped = nowFlipped;
    lastShakeAt = millis();                               // a flip is not a shake
    shakeCandidateAt = 0;
    if (oledAvailable) oled.setRotation(nowFlipped ? 2 : 0);
    cancelEntry(false);
    endSplash();
    Serial.println(nowFlipped ? F("[FLIP] upside down") : F("[FLIP] upright"));

    switch (timerState) {
        case TimerState::Ready:
        case TimerState::Finished:                        // idle: only turn the picture round
            showNotice(NoticeId::Flipped, "Flipped", "Display turned over", nullptr, 1500);
            break;

        case TimerState::Running:
        case TimerState::Paused: {                        // fallen sand is now on top
            uint32_t newRemaining = selectedSeconds - remainingSeconds;
            if (newRemaining == 0) {                      // flipped straight after starting: start again
                startTimer();
                showNotice(NoticeId::Flipped, "Flipped!", "Timer restarted", nullptr, 1500);
                break;
            }
            remainingSeconds = newRemaining;
            lastTimerTickMs = millis();
            pausedRemainderMs = 0;
            char t[8];
            formatTime(remainingSeconds, t, sizeof(t));
            snprintf(noticeText[0], sizeof(noticeText[0]), "%s left", t);
            showNotice(NoticeId::Flipped, "Flipped! Sand swapped", noticeText[0], nullptr, 1500);
            beep(2, 40, 60);
            break;
        }
    }
    drawHourglass(timerState == TimerState::Finished ? 0 : remainingSeconds);
    updateOLED(true);
}

void updateFlip() {
    if (!imuAvailable || !flipReferenceValid || !accFresh) return;
    accFresh = false;

    float magnitude = sqrtf(accX * accX + accY * accY + accZ * accZ);
    if (magnitude < FLIP_STILL_MIN || magnitude > FLIP_STILL_MAX) {   // being moved: not a flip
        flipCandidateSince = 0;
        return;
    }
    float dot = accX * refAx + accY * refAy + accZ * refAz;
    float norm = magnitude * sqrtf(refAx * refAx + refAy * refAy + refAz * refAz);
    if (norm < 1.0f) return;
    float cosine = dot / norm;                            // +1 upright, -1 upside down

    bool want = displayFlipped;
    if (cosine < -FLIP_COS_THRESHOLD) want = true;
    else if (cosine > FLIP_COS_THRESHOLD) want = false;   // in between: keep current (hysteresis)

    uint32_t now = millis();
    if (want == displayFlipped) { flipCandidateSince = 0; return; }
    if (flipCandidateSince == 0) { flipCandidateSince = now; return; }
    if (now - flipCandidateSince < FLIP_STABLE_MS) return;
    flipCandidateSince = 0;
    handleFlip(want);
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
    Serial.println(REMOTE_LAYOUT == RemoteLayout::Mp3_21Key ? F("21-key SPECIAL FOR MP3") : F("17-key arrows"));
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

    // Orientation check: a small triangle of sand in the bottom corner of each diamond.
    matrix.clear();
    for (uint8_t y = 0; y < 8; y++)
        for (uint8_t x = 0; x < 8; x++) {
            bool corner = (7 - x) + (7 - y) <= 2;
            setChamberPixel(1, x, y, corner);
            setChamberPixel(0, x, y, corner);
        }
    matrix.update();
    Serial.println(F("ORIENTATION: each diamond should show a small triangle in its BOTTOM corner."));
    Serial.println(F("If not, change UPPER_ROTATION / LOWER_ROTATION (0-3) or UPPER_MODULE."));
    start = millis();
    while (millis() - start < 1500) updateBeeper();
}

// ============================================================
// SETUP + LOOP
// ============================================================

void setup() {
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(BRIGHT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    analogReadResolution(12);

    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);

    matrix.begin();
    matrix.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
    matrix.control(MD_MAX72XX::INTENSITY, MATRIX_INTENSITY[brightnessIndex]);
    matrix.clear();
    matrix.update();

    if (!connectOled()) Serial.println(F("OLED not found. Messages go to Serial only."));
    setImuStatus(connectImu(), false);
    initPot();
    captureUprightReference();      // stand the hourglass upright when powering on
    buildGlassOrder();
    randomSeed(analogRead(A1));
    brightButtonDown = digitalRead(BRIGHT_BUTTON_PIN) == LOW;   // ignore a press held at power-up

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
    Serial.println(F("Turn the pot to set the time. Mode on the remote picks a food timer."));
    Serial.println(F("Press remote keys to see their codes here."));
}

void loop() {
    updateStartButton();
    updateBrightButton();
    updatePot();
    updateIr();
    updateTimer();
    updateShakeDetection();
    updateFlip();
    updateHealth();
    updateSplash();
    updateMatrixAnimation();
    updateBeeper();
    updateOLED();
}