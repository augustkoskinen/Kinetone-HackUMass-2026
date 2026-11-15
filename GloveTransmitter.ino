// ==========================================================
// GESTURAL MIDI GLOVE — TRANSMITTER (v6, guided setup + console commands)
//
// One sketch for BOTH gloves. Only HAND_ID differs.
//
// Setup is REQUIRED at every power-up and is guided by tones on the
// Bela (left glove first, then right). No playing data is sent until
// this glove's setup is complete.
//   - Normal start: quick re-zero (palm-down, tap index).
//   - Full 4-step calibration: power on holding ALL fingers 2 s,
//     release, hold ALL again 2 s. (Also automatic if none is saved.)
//   - Setup can also be started at any time from the Bela IDE console
//     (echo setup > /tmp/glove, echo full right > /tmp/glove, ...).
//   - If this glove restarts while the OTHER glove is already playing,
//     it resumes immediately with its last re-zero (warning tone).
//
// Wiring-tolerant: read timeouts, garbage rejection, spike filter,
// hold-then-glide on dropouts, automatic IMU recovery, debounced fingers.
// ==========================================================

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include <L3G.h>      // Pololu L3G library
#include <LSM303.h>   // Pololu LSM303 library

// ==========================================
// THE ONLY SETTING THAT CHANGES PER GLOVE
// ==========================================

#define HAND_ID 1     // 0 = LEFT glove, 1 = RIGHT glove

// ==========================================
// SHARED CONFIGURATION
// ==========================================

#define WIFI_CHANNEL       1
#define SEND_INTERVAL_MS   20    // 50 packets per second
#define IMU_INTERVAL_MS    10    // IMU read 100 times per second
#define DEBUG_SERIAL       1
#define DEBUG_INTERVAL_MS  100

#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_CLOCK_HZ    100000   // slow = tolerant of long / flexing wires
#define I2C_TIMEOUT_MS  5

#define LED_PIN 2     // onboard LED on most ESP32 dev boards (-1 = none)

// Finger pins: index, middle, ring, pinky
const int fingerPins[4] = {25, 26, 27, 14};
const int numFingers = (HAND_ID == 0) ? 3 : 4;
const unsigned long FINGER_DEBOUNCE_MS = 15;

// Roll
const float DEAD_ZONE_DEG = 20.0f;

// Motion
const float MOTION_DECAY_S = 0.3f;
const float MOTION_HP_S    = 0.5f;
const float MOTION_FLOOR   = 0.4f;
const float MOTION_DEFAULT_MAX = 15.0f;

// Dropout handling
const unsigned long HOLD_MS            = 300;
const unsigned long REINIT_AFTER_MS    = 500;
const unsigned long REINIT_INTERVAL_MS = 1000;
const uint16_t STUCK_LIMIT = 50;

const float GRAVITY = 9.80665f;
const uint8_t CAL_VERSION = 2;

// ==========================================
// TYPES (keep above all functions)
// ==========================================

struct Vec3 { float x, y, z; };

enum ImuType : uint8_t { IMU_NONE, IMU_POLOLU_V2, IMU_MPU6050 };

enum SetupState : uint8_t {
  ST_OFFLINE = 0,     // (used by the receiver only)
  ST_WAITING = 1,     // waiting for its turn
  ST_PROMPT = 2,      // waiting for an index tap
  ST_MEASURING = 3,   // capturing after a tap
  ST_READY = 4        // playing
};

enum SetupEvent : uint8_t {
  EV_NONE = 0, EV_FULL_MODE, EV_CAPTURE, EV_STEP_DONE, EV_REDO,
  EV_RESTART, EV_HAND_READY, EV_RESUMED
};

struct Calibration {
  bool    valid;
  uint8_t imuType;
  Vec3    neutral;     // gravity direction (unit) in playing position
  Vec3    axis;        // roll axis (unit), positive = clockwise
  float   maxCW;       // degrees
  float   maxCCW;      // degrees (positive number)
  float   motionMax;   // m/s^2 that maps to 100
  int8_t  gyroSign;    // +1 / -1, or 0 = accelerometer only
  Vec3    gyroBias;    // deg/s
};

// ==========================================
// WIRELESS PACKETS (must match the receiver)
// ==========================================

#define PACKET_MAGIC     0xA7
#define PACKET_VERSION   4
#define FLAG_IMU_VALID   0x01
#define FLAG_CALIBRATED  0x02

struct __attribute__((packed)) GloveData {
  uint8_t  magic;
  uint8_t  version;
  uint8_t  handID;
  uint8_t  fingers;     // bit3 index, bit2 middle, bit1 ring, bit0 pinky
  uint8_t  flags;
  int16_t  roll;        // -180..180
  uint8_t  motion;      // 0..100
  uint8_t  setupState;  // SetupState
  uint8_t  setupStep;   // 1..4 while setting up
  uint8_t  eventCode;   // latest SetupEvent
  uint8_t  eventSeq;    // increments with each event
  uint32_t sequence;
};

#define CTRL_MAGIC    0xA8
#define CTRL_VERSION  2
#define TURN_NONE     255

struct __attribute__((packed)) ControlData {   // receiver -> gloves
  uint8_t  magic;
  uint8_t  version;
  uint8_t  turn;          // 0 = left, 1 = right, 255 = none
  uint8_t  handState[2];  // SetupState of each glove as the receiver sees it
  uint16_t session;       // changes every time the receiver restarts
  uint8_t  cmdSeq[2];     // increments when setup is requested for that glove
  uint8_t  cmdType[2];    // 1 = re-zero setup, 2 = full calibration
};

// ==========================================
// STATE
// ==========================================

GloveData packet = {};
const uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Setup
uint8_t setupState = ST_WAITING;
uint8_t setupStep = 0;
uint8_t eventCode = EV_NONE;
uint8_t eventSeq = 0;
unsigned long lastSendMs = 0;

// From the receiver
volatile uint8_t ctrlTurn = TURN_NONE;
volatile uint8_t ctrlState[2] = {ST_OFFLINE, ST_OFFLINE};
volatile unsigned long ctrlLastMs = 0;
volatile uint16_t ctrlSession = 0;
volatile uint8_t ctrlCmdSeq[2] = {0, 0};
volatile uint8_t ctrlCmdType[2] = {0, 0};

// IMU
ImuType imuType = IMU_NONE;
L3G gyro;
LSM303 compass;
#define MPU_ADDR 0x68

bool imuActive = false;
unsigned long activeSince = 0;
unsigned long lastValidMs = 0;
bool everValid = false;
unsigned long lastReinitAttempt = 0;
uint32_t imuReadErrors = 0;

int16_t lastRaw[6] = {0};
uint16_t stuckCount = 0;
unsigned long lastImuMicros = 0;

Vec3 accHist[2], gyrHist[2];
uint8_t histCount = 0;

// Calibration
Calibration cal = {};
bool calOK = false;
Vec3 neutralProj = {0, 0, 0};
Preferences prefs;
Vec3 gyroBias = {0, 0, 0};

// Orientation / motion
float rollAngle = 0;
bool rollInit = false;
Vec3 accLP = {0, 0, 0};
bool motionInit = false;
float motionEnv = 0;

// Fingers
uint8_t fingerStable = 0;
uint8_t fingerCandidate = 0;
unsigned long fingerCandidateSince = 0;

// Output
float rollOut = 0;
float motionOut = 0;
uint32_t sequenceNumber = 0;
uint32_t sendFailures = 0;

// ==========================================
// VECTOR MATH
// ==========================================

Vec3 vadd(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 vsub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 vscale(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 vcross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
float vlen(Vec3 a) { return sqrtf(vdot(a, a)); }
Vec3 vnorm(Vec3 a) {
  float l = vlen(a);
  return (l > 1e-6f) ? vscale(a, 1.0f / l) : Vec3{0, 0, 0};
}

float med3(float a, float b, float c) {
  return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}
Vec3 med3v(Vec3 a, Vec3 b, Vec3 c) {
  return {med3(a.x, b.x, c.x), med3(a.y, b.y, c.y), med3(a.z, b.z, c.z)};
}

float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float angleAround(Vec3 from, Vec3 to, Vec3 axis) {
  Vec3 pf = vnorm(vsub(from, vscale(axis, vdot(from, axis))));
  Vec3 pt = vnorm(vsub(to,   vscale(axis, vdot(to,   axis))));
  return atan2f(vdot(axis, vcross(pf, pt)), vdot(pf, pt)) * RAD_TO_DEG;
}

// ==========================================
// LED AND FINGERS
// ==========================================

void ledSet(bool on) {
  if (LED_PIN >= 0) digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void ledStepPattern(int n) {
  unsigned long cycle = n * 400UL + 1200UL;
  unsigned long pos = millis() % cycle;
  ledSet(pos < n * 400UL && (pos % 400UL) < 200UL);
}

bool fingerDown(int i) { return digitalRead(fingerPins[i]) == LOW; }

bool allFingersDown() {
  for (int i = 0; i < numFingers; i++) if (!fingerDown(i)) return false;
  return true;
}

bool anyFingerDown() {
  for (int i = 0; i < numFingers; i++) if (fingerDown(i)) return true;
  return false;
}

uint8_t readFingersRaw() {
  uint8_t s = 0;
  for (int i = 0; i < numFingers; i++) if (fingerDown(i)) s |= (1 << (3 - i));
  return s;
}

void updateFingers() {
  uint8_t raw = readFingersRaw();
  if (raw != fingerCandidate) {
    fingerCandidate = raw;
    fingerCandidateSince = millis();
  } else if (raw != fingerStable && millis() - fingerCandidateSince >= FINGER_DEBOUNCE_MS) {
    fingerStable = raw;
  }
}

// ==========================================
// I2C
// ==========================================

void startI2C() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
}

void i2cBusRecover() {
  Wire.end();

  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(10);

  for (int i = 0; i < 9 && digitalRead(I2C_SDA) == LOW; i++) {
    digitalWrite(I2C_SCL, LOW);  delayMicroseconds(10);
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(10);
  }

  pinMode(I2C_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(I2C_SDA, LOW);  delayMicroseconds(10);
  digitalWrite(I2C_SCL, HIGH); delayMicroseconds(10);
  digitalWrite(I2C_SDA, HIGH); delayMicroseconds(10);

  startI2C();
}

bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool mpuReadBytes(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, n) != n) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

int mpuReadReg(uint8_t reg) {
  uint8_t v;
  return mpuReadBytes(reg, &v, 1) ? v : -1;
}

void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// ==========================================
// IMU DRIVERS
// ==========================================

const char *imuName(uint8_t t) {
  if (t == IMU_POLOLU_V2) return "Pololu MinIMU-9 v2";
  if (t == IMU_MPU6050)   return "MPU6050";
  return "none (fingers only)";
}

bool initPololu() {
  if (!i2cDevicePresent(0x6B) && !i2cDevicePresent(0x6A)) return false;
  if (!gyro.init(L3G::device_D20)) {
    Serial.println("Pololu: gyro init failed");
    return false;
  }
  if (!compass.init(LSM303::device_DLHC)) {
    Serial.println("Pololu: accelerometer init failed");
    return false;
  }
  gyro.setTimeout(I2C_TIMEOUT_MS);
  compass.setTimeout(I2C_TIMEOUT_MS);
  gyro.enableDefault();
  compass.enableDefault();
  compass.writeAccReg(LSM303::CTRL_REG1_A, 0x57);    // accel at 100 Hz
  return true;
}

bool initMPU6050() {
  if (!i2cDevicePresent(MPU_ADDR)) return false;

  int id = mpuReadReg(0x75);
  if (id != 0x68) {
    Serial.printf("MPU6050: unexpected WHO_AM_I 0x%02X\n", id);
    return false;
  }

  mpuWriteReg(0x6B, 0x80);
  unsigned long t0 = millis();
  delay(10);
  while (mpuReadReg(0x6B) < 0 && millis() - t0 < 300) delay(5);
  delay(50);
  mpuWriteReg(0x6B, 0x01);
  delay(50);

  const uint8_t cfg[][2] = {
    {0x19, 0x04}, {0x1A, 0x03}, {0x1B, 0x08}, {0x1C, 0x08}, {0x38, 0x01},
  };
  for (auto &c : cfg) {
    mpuWriteReg(c[0], c[1]);
    if (mpuReadReg(c[0]) != c[1]) {
      Serial.printf("MPU6050: register 0x%02X did not stick\n", c[0]);
      return false;
    }
  }

  t0 = millis();
  while (millis() - t0 < 100) {
    int s = mpuReadReg(0x3A);
    if (s >= 0 && (s & 0x01)) return true;
    delay(2);
  }
  Serial.println("MPU6050: never produced data");
  return false;
}

ImuType detectIMU() {
  if (initPololu())  return IMU_POLOLU_V2;
  if (initMPU6050()) return IMU_MPU6050;
  return IMU_NONE;
}

bool readIMURaw(Vec3 &acc, Vec3 &gyr, int16_t raw[6]) {
  if (imuType == IMU_POLOLU_V2) {
    gyro.read();
    compass.readAcc();
    bool gyroTimeout = gyro.timeoutOccurred();
    bool accTimeout  = compass.timeoutOccurred();
    if (gyroTimeout || accTimeout) return false;

    raw[0] = compass.a.x; raw[1] = compass.a.y; raw[2] = compass.a.z;
    raw[3] = gyro.g.x;    raw[4] = gyro.g.y;    raw[5] = gyro.g.z;
    const float ka = 0.001f * GRAVITY;
    acc = {(raw[0] >> 4) * ka, (raw[1] >> 4) * ka, (raw[2] >> 4) * ka};
    gyr = {raw[3] * 0.00875f, raw[4] * 0.00875f, raw[5] * 0.00875f};
    return true;
  }

  if (imuType == IMU_MPU6050) {
    uint8_t b[14];
    if (!mpuReadBytes(0x3B, b, 14)) return false;
    raw[0] = (int16_t)((b[0]  << 8) | b[1]);
    raw[1] = (int16_t)((b[2]  << 8) | b[3]);
    raw[2] = (int16_t)((b[4]  << 8) | b[5]);
    raw[3] = (int16_t)((b[8]  << 8) | b[9]);
    raw[4] = (int16_t)((b[10] << 8) | b[11]);
    raw[5] = (int16_t)((b[12] << 8) | b[13]);
    const float ka = GRAVITY / 8192.0f;
    acc = {raw[0] * ka, raw[1] * ka, raw[2] * ka};
    gyr = {raw[3] / 65.5f, raw[4] / 65.5f, raw[5] / 65.5f};
    return true;
  }

  return false;
}

bool readIMU(Vec3 &acc, Vec3 &gyr, int16_t raw[6]) {
  if (!readIMURaw(acc, gyr, raw)) {
    imuReadErrors++;
    return false;
  }

  bool allSame = true;
  for (int i = 1; i < 6; i++) if (raw[i] != raw[0]) { allSame = false; break; }

  float aMag = vlen(acc);
  if (allSame ||
      aMag < 0.05f * GRAVITY || aMag > 5.0f * GRAVITY ||
      fabsf(gyr.x) > 2500.0f || fabsf(gyr.y) > 2500.0f || fabsf(gyr.z) > 2500.0f) {
    imuReadErrors++;
    return false;
  }
  return true;
}

void activateIMU() {
  imuActive = true;
  activeSince = millis();
  stuckCount = 0;
  histCount = 0;
  rollInit = false;
  motionInit = false;
  lastImuMicros = micros();
}

// ==========================================
// CALIBRATION STORAGE
// ==========================================

void applyCalibration() {
  calOK = cal.valid && cal.imuType == imuType && imuType != IMU_NONE;
  if (calOK) {
    neutralProj = vnorm(vsub(cal.neutral, vscale(cal.axis, vdot(cal.neutral, cal.axis))));
  }
  rollInit = false;
}

void loadCalibration() {
  prefs.begin("glovecal", true);
  uint8_t ver = prefs.getUChar("ver", 0);
  bool sizeOK = prefs.getBytesLength("cal") == sizeof(cal);
  if (ver == CAL_VERSION && sizeOK) prefs.getBytes("cal", &cal, sizeof(cal));
  else cal.valid = false;
  prefs.end();
}

void saveCalibration() {
  prefs.begin("glovecal", false);
  prefs.putBytes("cal", &cal, sizeof(cal));
  prefs.putUChar("ver", CAL_VERSION);
  prefs.end();
}

void printCalibration() {
  if (imuType == IMU_NONE) {
    Serial.println("No IMU - fingers only");
    return;
  }
  if (!cal.valid) {
    Serial.println("Calibration: NONE - full calibration will run during setup");
    return;
  }
  if (cal.imuType != imuType) {
    Serial.printf("Calibration: made with %s, but %s is connected - full calibration will run\n",
                  imuName(cal.imuType), imuName(imuType));
    return;
  }
  Serial.printf("Calibration: OK | clockwise max %.0f deg | counterclockwise max %.0f deg | "
                "motion max %.1f m/s^2 | gyro assist %s\n",
                cal.maxCW, cal.maxCCW, cal.motionMax, cal.gyroSign ? "ON" : "OFF");
}

// ==========================================
// RUNTIME PROCESSING
// ==========================================

void trackGyroBias(Vec3 gyrRaw, Vec3 acc, float dt) {
  static Vec3 accPrev = {0, 0, 0};
  static float stillTime = 0;

  Vec3 d = vsub(gyrRaw, gyroBias);
  bool gyroQuiet = fabsf(d.x) < 8.0f && fabsf(d.y) < 8.0f && fabsf(d.z) < 8.0f;
  bool accSteady = vlen(vsub(acc, accPrev)) < 0.3f && fabsf(vlen(acc) - GRAVITY) < 0.8f;
  accPrev = acc;

  if (gyroQuiet && accSteady) {
    stillTime += dt;
    if (stillTime > 0.5f) gyroBias = vadd(gyroBias, vscale(d, 0.005f));
  } else {
    stillTime = 0;
  }
}

void updateRoll(Vec3 acc, Vec3 gyr, float dt) {
  float aMag = vlen(acc);
  bool accTrusted = aMag > 0.75f * GRAVITY && aMag < 1.25f * GRAVITY;

  Vec3 g = vscale(acc, 1.0f / fmaxf(aMag, 1e-3f));
  Vec3 p = vsub(g, vscale(cal.axis, vdot(g, cal.axis)));
  float pLen = vlen(p);
  if (pLen < 0.35f) accTrusted = false;

  float accAngle = 0;
  if (accTrusted) {
    p = vscale(p, 1.0f / pLen);
    accAngle = atan2f(vdot(cal.axis, vcross(neutralProj, p)), vdot(neutralProj, p)) * RAD_TO_DEG;
  }

  if (!rollInit) {
    if (!accTrusted) return;
    rollAngle = accAngle;
    rollInit = true;
    return;
  }

  if (cal.gyroSign != 0) {
    rollAngle += cal.gyroSign * vdot(gyr, cal.axis) * dt;
    if (accTrusted) rollAngle += 0.02f * wrap180(accAngle - rollAngle);
  } else if (accTrusted) {
    rollAngle += 0.15f * wrap180(accAngle - rollAngle);
  }

  rollAngle = constrain(rollAngle, -300.0f, 300.0f);
}

int16_t mapRoll(float a) {
  if (a > DEAD_ZONE_DEG) {
    float t = (a - DEAD_ZONE_DEG) / (cal.maxCW - DEAD_ZONE_DEG);
    return (int16_t)lroundf(constrain(t, 0.0f, 1.0f) * 180.0f);
  }
  if (a < -DEAD_ZONE_DEG) {
    float t = (-a - DEAD_ZONE_DEG) / (cal.maxCCW - DEAD_ZONE_DEG);
    return -(int16_t)lroundf(constrain(t, 0.0f, 1.0f) * 180.0f);
  }
  return 0;
}

void updateMotion(Vec3 acc, float dt) {
  if (!motionInit) { accLP = acc; motionInit = true; motionEnv = 0; }

  float k = fminf(dt / MOTION_HP_S, 1.0f);
  accLP = vadd(accLP, vscale(vsub(acc, accLP), k));

  float lin = vlen(vsub(acc, accLP)) - MOTION_FLOOR;
  if (lin < 0) lin = 0;

  motionEnv *= expf(-dt / MOTION_DECAY_S);
  if (lin > motionEnv) motionEnv = lin;
}

float motionValue() {
  float maxVal = calOK ? cal.motionMax : MOTION_DEFAULT_MAX;
  return constrain(motionEnv / maxVal * 100.0f, 0.0f, 100.0f);
}

void processIMU() {
  Vec3 acc, gyr;
  int16_t raw[6];
  if (!readIMU(acc, gyr, raw)) return;

  if (memcmp(raw, lastRaw, sizeof(raw)) == 0) {
    if (stuckCount < STUCK_LIMIT) stuckCount++;
  } else {
    stuckCount = 0;
  }
  memcpy(lastRaw, raw, sizeof(raw));
  if (stuckCount >= STUCK_LIMIT) return;

  unsigned long nowMs = millis();

  if (!everValid || nowMs - lastValidMs > HOLD_MS) {
    rollInit = false;
    motionInit = false;
    histCount = 0;
  }

  Vec3 accF = acc, gyrF = gyr;
  if (histCount >= 2) {
    accF = med3v(accHist[0], accHist[1], acc);
    gyrF = med3v(gyrHist[0], gyrHist[1], gyr);
  }
  accHist[0] = accHist[1]; accHist[1] = acc;
  gyrHist[0] = gyrHist[1]; gyrHist[1] = gyr;
  if (histCount < 2) histCount++;

  unsigned long nowUs = micros();
  float dt = (nowUs - lastImuMicros) / 1000000.0f;
  lastImuMicros = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = 0;

  lastValidMs = nowMs;
  everValid = true;

  trackGyroBias(gyrF, accF, dt);
  Vec3 g = vsub(gyrF, gyroBias);

  if (calOK) updateRoll(accF, g, dt);
  updateMotion(accF, dt);
}

void maintainIMU() {
  unsigned long now = millis();

  if (imuActive) {
    unsigned long ref = (lastValidMs > activeSince) ? lastValidMs : activeSince;
    if (now - ref > REINIT_AFTER_MS) {
      imuActive = false;
      lastReinitAttempt = 0;
      Serial.println("IMU lost - trying to reconnect...");
    }
    return;
  }

  if (now - lastReinitAttempt < REINIT_INTERVAL_MS) return;
  lastReinitAttempt = now;

  i2cBusRecover();
  ImuType found = detectIMU();
  if (found == IMU_NONE) return;

  bool changed = (found != imuType);
  imuType = found;
  activateIMU();

  if (changed) {
    Serial.printf("IMU found: %s\n", imuName(imuType));
    applyCalibration();
    printCalibration();
  } else {
    Serial.println("IMU reconnected");
  }
}

// ==========================================
// ESP-NOW AND SENDING
// ==========================================

void onControlRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(ControlData)) return;   // ignores the other glove's packets
  ControlData c;
  memcpy(&c, data, sizeof(c));
  if (c.magic != CTRL_MAGIC || c.version != CTRL_VERSION) return;
  ctrlTurn = c.turn;
  ctrlState[0] = c.handState[0];
  ctrlState[1] = c.handState[1];
  ctrlSession = c.session;
  ctrlCmdSeq[0] = c.cmdSeq[0];
  ctrlCmdSeq[1] = c.cmdSeq[1];
  ctrlCmdType[0] = c.cmdType[0];
  ctrlCmdType[1] = c.cmdType[1];
  ctrlLastMs = millis();
}

bool controlFresh()   { return ctrlLastMs != 0 && millis() - ctrlLastMs < 1000; }
bool myTurn()         { return controlFresh() && ctrlTurn == HAND_ID; }
bool otherHandReady() { return controlFresh() && ctrlState[1 - HAND_ID] == ST_READY; }

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed - restarting");
    delay(2000);
    ESP.restart();
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddress, 6);
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ERROR: could not add broadcast peer - restarting");
    delay(2000);
    ESP.restart();
  }

  esp_now_register_recv_cb(onControlRecv);

  Serial.printf("ESP-NOW ready | MAC %s | channel %d\n",
                WiFi.macAddress().c_str(), WiFi.channel());
}

void sendPacket(unsigned long now) {
  bool ready = (setupState == ST_READY);
  bool fresh = ready && everValid && (now - lastValidMs < HOLD_MS);

  if (!ready) {
    rollOut = 0;
    motionOut = 0;
  } else if (fresh) {
    rollOut   = (calOK && rollInit) ? mapRoll(rollAngle) : 0;
    motionOut = motionValue();
  } else {
    rollOut   *= 0.85f;
    motionOut *= 0.85f;
    if (fabsf(rollOut) < 0.5f) rollOut = 0;
    if (motionOut < 0.5f)      motionOut = 0;
  }

  packet.fingers    = ready ? fingerStable : 0;
  packet.flags      = (fresh ? FLAG_IMU_VALID : 0) | (calOK ? FLAG_CALIBRATED : 0);
  packet.roll       = (int16_t)lroundf(rollOut);
  packet.motion     = (uint8_t)lroundf(motionOut);
  packet.setupState = setupState;
  packet.setupStep  = setupStep;
  packet.eventCode  = eventCode;
  packet.eventSeq   = eventSeq;
  packet.sequence   = sequenceNumber++;

  if (esp_now_send(broadcastAddress, (const uint8_t *)&packet, sizeof(packet)) != ESP_OK)
    sendFailures++;
}

// Keeps fingers and the radio going while setup code waits
void backgroundTick() {
  updateFingers();
  unsigned long now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendPacket(now);
  }
}

void tickDelay(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    backgroundTick();
    delay(2);
  }
}

void ledBlinkTicked(int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    ledSet(true);  tickDelay(onMs);
    ledSet(false); tickDelay(offMs);
  }
}

// Report a one-off event to the Bela (repeated in several packets)
void postEvent(uint8_t code) {
  eventCode = code;
  eventSeq++;
  tickDelay(150);
}

// ==========================================
// SETUP FLOW
// ==========================================

// --- power-up gesture for full calibration (before the radio starts) ---

bool waitAllReleased(unsigned long timeoutMs) {
  unsigned long t0 = millis(), clearSince = 0;
  while (millis() - t0 < timeoutMs) {
    if (anyFingerDown()) {
      clearSince = 0;
    } else {
      if (clearSince == 0) clearSince = millis();
      if (millis() - clearSince > 300) { ledSet(false); return true; }
    }
    ledSet((millis() / 100) % 2);
    delay(10);
  }
  ledSet(false);
  return false;
}

bool holdAllFingers(unsigned long ms) {
  unsigned long t0 = millis(), upSince = 0;
  while (millis() - t0 < ms) {
    if (allFingersDown()) {
      upSince = 0;
    } else {
      if (upSince == 0) upSince = millis();
      if (millis() - upSince > 150) { ledSet(false); return false; }
    }
    ledSet((millis() / 250) % 2);
    delay(10);
  }
  ledSet(false);
  return true;
}

bool fullCalibrationGesture() {
  if (!allFingersDown()) return false;

  Serial.println("All fingers held - keep holding...");
  if (!holdAllFingers(2000)) {
    Serial.println("Released early - normal setup");
    return false;
  }

  Serial.println("Now RELEASE, then hold all fingers again for 2 s to confirm full calibration.");
  if (!waitAllReleased(5000)) {
    Serial.println("Fingers not released - normal setup");
    return false;
  }

  unsigned long t0 = millis();
  while (!allFingersDown()) {
    if (millis() - t0 > 5000) {
      Serial.println("No confirmation - normal setup");
      ledSet(false);
      return false;
    }
    ledSet((millis() / 60) % 2);
    delay(10);
  }

  if (!holdAllFingers(2000)) {
    Serial.println("Released early - normal setup");
    return false;
  }

  Serial.println("FULL CALIBRATION requested");
  waitAllReleased(10000);
  return true;
}

// --- guided steps (radio running, tones on the Bela) ---

void waitForTurn() {
  setupState = ST_WAITING;
  setupStep = 0;
  bool announced = false;
  while (!myTurn()) {
    if (!announced) {
      Serial.println(controlFresh() ? "Waiting for the other glove to finish setup..."
                                    : "Waiting for the receiver...");
      announced = true;
    }
    ledSet((millis() / 1000) % 2);
    tickDelay(20);
  }
  ledSet(false);
}

// Waits (forever) for an index-finger tap. Optionally integrates the gyro.
void waitForTap(int step, Vec3 *rotation) {
  setupState = ST_PROMPT;
  setupStep = step;

  unsigned long downSince = 0, upSince = 0;
  bool armed = false;
  unsigned long lastUs = micros();

  while (true) {
    backgroundTick();
    ledStepPattern(step);

    if (rotation) {
      Vec3 a, g;
      int16_t raw[6];
      if (readIMU(a, g, raw)) {
        unsigned long nowUs = micros();
        float dt = fminf((nowUs - lastUs) / 1000000.0f, 0.05f);
        lastUs = nowUs;
        *rotation = vadd(*rotation, vscale(vsub(g, gyroBias), dt));
      }
    }

    if (fingerDown(0)) {
      upSince = 0;
      if (downSince == 0) downSince = millis();
      if (millis() - downSince >= 50) armed = true;
    } else {
      downSince = 0;
      if (armed) {
        if (upSince == 0) upSince = millis();
        if (millis() - upSince >= 50) {
          ledSet(false);
          return;
        }
      }
    }
    delay(10);
  }
}

// 0 = OK, 1 = hand moved, 2 = too many sensor dropouts
int captureStill(unsigned long ms, Vec3 &accMean, Vec3 &gyrMean) {
  Vec3 sa = {0, 0, 0}, sg = {0, 0, 0};
  Vec3 mn = {1e9f, 1e9f, 1e9f}, mx = {-1e9f, -1e9f, -1e9f};
  int n = 0;

  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    backgroundTick();
    Vec3 a, g;
    int16_t raw[6];
    if (readIMU(a, g, raw)) {
      sa = vadd(sa, a);
      sg = vadd(sg, g);
      mn = {fminf(mn.x, a.x), fminf(mn.y, a.y), fminf(mn.z, a.z)};
      mx = {fmaxf(mx.x, a.x), fmaxf(mx.y, a.y), fmaxf(mx.z, a.z)};
      n++;
    }
    delay(10);
  }

  int expected = ms / 10;
  if (n < expected * 0.6f) return 2;

  accMean = vscale(sa, 1.0f / n);
  gyrMean = vscale(sg, 1.0f / n);

  float spread = fmaxf(mx.x - mn.x, fmaxf(mx.y - mn.y, mx.z - mn.z));
  return (spread < 2.5f) ? 0 : 1;
}

// Tap, tick, measure, chime. Repeats the step (low buzz) until it's good.
void captureStep(int step, unsigned long ms, Vec3 &accMean, Vec3 &gyrMean, Vec3 *rotation) {
  while (true) {
    waitForTap(step, rotation);
    setupState = ST_MEASURING;
    postEvent(EV_CAPTURE);
    ledSet(true);
    tickDelay(300);
    int r = captureStill(ms, accMean, gyrMean);
    ledSet(false);

    if (r == 0) {
      postEvent(EV_STEP_DONE);
      return;
    }

    if (r == 2) {
      Serial.println("  Sensor dropouts - resetting the IMU connection, then redo this step");
      i2cBusRecover();
      ImuType t = detectIMU();
      if (t != IMU_NONE) imuType = t;
    } else {
      Serial.println("  Hand moved - hold still and tap again");
    }
    postEvent(EV_REDO);
  }
}

bool quickRezero() {
  Serial.println("\nRE-ZERO: hold palm-down in playing position, keep still, tap index.");
  Vec3 aN, gN;
  captureStep(1, 1500, aN, gN, nullptr);

  Vec3 g0 = vnorm(aN);
  Vec3 p = vsub(g0, vscale(cal.axis, vdot(g0, cal.axis)));
  if (vlen(p) < 0.5f) {
    Serial.println("Problem: this position doesn't fit the saved calibration "
                   "(forearm not level, or sensor moved). Do it again, or run a full calibration.");
    return false;
  }

  cal.neutral  = g0;
  cal.gyroBias = gN;
  gyroBias     = gN;
  saveCalibration();
  applyCalibration();
  Serial.println("Re-zero saved");
  return true;
}

bool fullCalibrationAttempt() {
  Vec3 aN, gN, aCW, aCCW, gTmp;

  Serial.println("\nFULL STEP 1/4: palm-down in playing position, still, tap index.");
  captureStep(1, 1500, aN, gN, nullptr);
  gyroBias = gN;
  Vec3 g0 = vnorm(aN);

  Serial.println("\nFULL STEP 2/4: rotate CLOCKWISE as far as you play, hold, tap index.");
  Vec3 rot2 = {0, 0, 0};
  captureStep(2, 1000, aCW, gTmp, &rot2);
  Vec3 gCW = vnorm(aCW);

  Serial.println("\nFULL STEP 3/4: rotate COUNTERCLOCKWISE as far as you play, hold, tap index.");
  Vec3 rot3 = {0, 0, 0};
  captureStep(3, 1000, aCCW, gTmp, &rot3);
  Vec3 gCCW = vnorm(aCCW);

  Vec3 cCW  = vcross(g0, gCW);
  Vec3 cCCW = vcross(gCCW, g0);

  if (vdot(cCW, cCCW) <= 0) {
    Serial.println("Problem: steps 2 and 3 weren't opposite directions of the same rotation.");
    return false;
  }

  Vec3 axisSum = vadd(cCW, cCCW);
  if (vlen(axisSum) < 0.3f) {
    Serial.println("Problem: rotation too small, or forearm pointing up/down.");
    return false;
  }

  Vec3 axis = vnorm(axisSum);
  float maxCW  =  angleAround(g0, gCW,  axis);
  float maxCCW = -angleAround(g0, gCCW, axis);
  Serial.printf("Measured: clockwise %.0f deg, counterclockwise %.0f deg\n", maxCW, maxCCW);

  if (maxCW < DEAD_ZONE_DEG + 10.0f || maxCCW < DEAD_ZONE_DEG + 10.0f) {
    Serial.println("Problem: rotate further - each direction must be at least 30 deg.");
    return false;
  }

  float r2 =  vdot(rot2, axis) / maxCW;
  float r3 = -vdot(rot3, axis) / (maxCW + maxCCW);
  int8_t gyroSign = 0;
  if (r2 * r3 > 0 &&
      fabsf(r2) > 0.6f && fabsf(r2) < 1.6f &&
      fabsf(r3) > 0.6f && fabsf(r3) < 1.6f) {
    gyroSign = (r2 > 0) ? 1 : -1;
  }
  Serial.printf("Gyro check: %.2f / %.2f -> gyro assist %s\n",
                r2, r3, gyroSign ? "ON" : "OFF (accelerometer only)");

  Serial.println("\nFULL STEP 4/4: tap index, then shake as hard as you'd ever play for 3 s.");
  waitForTap(4, nullptr);
  setupState = ST_MEASURING;
  postEvent(EV_CAPTURE);
  ledSet(true);

  float peak = 0;
  Vec3 lp = {0, 0, 0};
  bool lpInit = false;
  unsigned long t0 = millis();
  unsigned long lastUs = micros();
  while (millis() - t0 < 3000) {
    backgroundTick();
    Vec3 a, g;
    int16_t raw[6];
    if (readIMU(a, g, raw)) {
      unsigned long nowUs = micros();
      float dt = fminf((nowUs - lastUs) / 1000000.0f, 0.05f);
      lastUs = nowUs;
      if (!lpInit) { lp = a; lpInit = true; }
      float k = fminf(dt / MOTION_HP_S, 1.0f);
      lp = vadd(lp, vscale(vsub(a, lp), k));
      float lin = vlen(vsub(a, lp)) - MOTION_FLOOR;
      if (lin > peak) peak = lin;
    }
    delay(10);
  }
  ledSet(false);
  postEvent(EV_STEP_DONE);

  float motionMax = fmaxf(2.0f, 0.8f * peak);
  Serial.printf("Peak movement %.1f m/s^2 -> 100 at %.1f m/s^2\n", peak, motionMax);

  cal.valid     = true;
  cal.imuType   = imuType;
  cal.neutral   = g0;
  cal.axis      = axis;
  cal.maxCW     = maxCW;
  cal.maxCCW    = maxCCW;
  cal.motionMax = motionMax;
  cal.gyroSign  = gyroSign;
  cal.gyroBias  = gN;
  saveCalibration();
  applyCalibration();

  Serial.println("FULL CALIBRATION SAVED");
  return true;
}

void runSetup(bool full) {
  if (imuType != IMU_NONE && !calOK) full = true;   // re-zero needs a saved full calibration

  while (true) {
    waitForTurn();
    Serial.printf("\n=== SETUP: %s glove, %s ===\n", HAND_ID == 0 ? "LEFT" : "RIGHT",
                  imuType == IMU_NONE ? "confirm only" : (full ? "full calibration" : "re-zero"));

    bool ok;
    if (imuType == IMU_NONE) {
      Serial.println("Tap index to confirm this glove.");
      waitForTap(1, nullptr);
      postEvent(EV_STEP_DONE);
      ok = true;
    } else if (full) {
      ok = fullCalibrationAttempt();
    } else {
      ok = quickRezero();
    }

    if (ok) break;
    postEvent(EV_RESTART);
    tickDelay(1500);
  }

  setupState = ST_READY;
  setupStep = 0;
  postEvent(EV_HAND_READY);
  ledBlinkTicked(3, 300, 150);
}

// ==========================================
// SETUP
// ==========================================

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.printf("\n=== Glove transmitter: %s (HAND_ID %d) ===\n",
                HAND_ID == 0 ? "LEFT" : "RIGHT", HAND_ID);

  for (int i = 0; i < numFingers; i++) pinMode(fingerPins[i], INPUT_PULLUP);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  ledSet(false);

  startI2C();
  imuType = detectIMU();
  Serial.printf("IMU: %s\n", imuName(imuType));
  if (imuType != IMU_NONE) activateIMU();

  loadCalibration();
  if (cal.valid) gyroBias = cal.gyroBias;
  applyCalibration();
  printCalibration();

  bool fullRequested = fullCalibrationGesture();

  setupESPNow();
  packet.magic = PACKET_MAGIC;
  packet.version = PACKET_VERSION;
  packet.handID = HAND_ID;

  setupState = ST_WAITING;
  if (fullRequested) postEvent(EV_FULL_MODE);

  // Listen briefly: is the other glove already playing? (= mid-show restart)
  unsigned long t0 = millis();
  while (!controlFresh() && millis() - t0 < 1500) tickDelay(10);

  bool canResume = !fullRequested && otherHandReady() && (imuType == IMU_NONE || calOK);

  if (canResume) {
    Serial.println("Other glove is already playing - resuming with the last re-zero");
    setupState = ST_READY;
    postEvent(EV_RESUMED);
  } else {
    runSetup(fullRequested);
  }

  if (imuType != IMU_NONE) activateIMU();
  Serial.println("READY - playing");
}

// ==========================================
// MAIN LOOP (playing)
// ==========================================

void loop() {
  static unsigned long lastImu = 0, lastDebug = 0;
  unsigned long now = millis();

  updateFingers();

  if (imuActive && now - lastImu >= IMU_INTERVAL_MS) {
    lastImu = now;
    processIMU();
  }

  maintainIMU();

  // Setup requested from the Bela console
  {
    static bool cmdBaseline = false;
    static uint16_t lastSession = 0;
    static uint8_t lastCmdSeq = 0;

    if (controlFresh()) {
      uint16_t sess = ctrlSession;
      uint8_t seq = ctrlCmdSeq[HAND_ID];

      if (!cmdBaseline || sess != lastSession) {
        // First contact, or the receiver restarted: remember, don't act
        cmdBaseline = true;
        lastSession = sess;
        lastCmdSeq = seq;
      } else if (seq != lastCmdSeq) {
        lastCmdSeq = seq;
        bool full = (ctrlCmdType[HAND_ID] == 2);
        Serial.printf("\nSetup requested from the Bela (%s)\n", full ? "full calibration" : "re-zero");
        if (full) postEvent(EV_FULL_MODE);
        runSetup(full);
        if (imuType != IMU_NONE) activateIMU();
        Serial.println("READY - playing");
      }
    }
  }

  now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendPacket(now);
  }

#if DEBUG_SERIAL
  if (now - lastDebug >= DEBUG_INTERVAL_MS) {
    lastDebug = now;
    bool fresh = everValid && (now - lastValidMs < HOLD_MS);

    Serial.printf("Hand %d | Fingers %d%d%d%d | Roll %4d",
                  HAND_ID,
                  (packet.fingers >> 3) & 1, (packet.fingers >> 2) & 1,
                  (packet.fingers >> 1) & 1, packet.fingers & 1,
                  packet.roll);
    if (calOK && rollInit && fresh) Serial.printf(" (angle %6.1f)", rollAngle);
    Serial.printf(" | Motion %3d", packet.motion);

    if (imuType == IMU_NONE)  Serial.print(" | no IMU");
    else if (!imuActive)      Serial.print(" | IMU LOST (reconnecting)");
    else if (!fresh)          Serial.print(" | IMU dropout (holding)");
    else if (!calOK)          Serial.print(" | NOT CALIBRATED");

    if (sendFailures)  Serial.printf(" | sendFail %lu", (unsigned long)sendFailures);
    if (imuReadErrors) Serial.printf(" | I2C errors %lu", (unsigned long)imuReadErrors);
    Serial.println();
  }
#endif

  delay(1);
}