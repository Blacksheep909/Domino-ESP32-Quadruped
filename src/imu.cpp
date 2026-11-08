#include "imu.h"

#include <Wire.h>
#ifndef DOMINO_SIL
#include "i2c_bus.h"
#include "loop_diagnostics.h"
#endif

// IMU bus wiring.
//
// Default: the MPU6050 shares the PCA9685 servo bus (GPIO 21/22). Servo current
// spikes on the shared wiring can interrupt an IMU read and leave the IMU holding
// SDA low, which stalls the servo driver as well.
//
// Recommended: give the IMU its own bus by moving its SDA/SCL wires to two free
// pins and building with -DDOMINO_IMU_SDA_PIN=32 -DDOMINO_IMU_SCL_PIN=33. The IMU
// then runs on the ESP32's second I2C controller (Wire1) at 400 kHz and is read
// by a background task on core 0, so an IMU fault can never block the servo bus
// or the control loop.
//
// Optional: -DDOMINO_IMU_POWER_PIN=<gpio> feeds the MPU6050's VCC from a GPIO
// (it draws ~4 mA) so a wedged IMU can be power-cycled during recovery.
#if !defined(DOMINO_SIL) && defined(DOMINO_IMU_SDA_PIN) && defined(DOMINO_IMU_SCL_PIN)
#define DOMINO_IMU_OWN_BUS 1
#else
#define DOMINO_IMU_OWN_BUS 0
#endif
#ifndef DOMINO_IMU_I2C_HZ
#define DOMINO_IMU_I2C_HZ 400000
#endif

namespace {
// IMU (MPU6050) configuration.
// AD0 low gives 0x68, AD0 high 0x69; WHO_AM_I reads 0x68 either way.
constexpr uint8_t kMpu6050AddrPrimary = 0x68;
constexpr uint8_t kMpu6050AddrSecondary = 0x69;
constexpr uint8_t kWhoAmIRegister = 0x75;
constexpr uint8_t kWhoAmIValue = 0x68;
// 100 Hz sampling. Reading on every main-loop pass kept the shared bus busy
// with 14-byte bursts almost continuously, which is the window in which servo
// noise can leave the IMU holding SDA low.
constexpr uint32_t kImuSampleIntervalMs = 10;
// Repeated errors pause the IMU, free the bus, then re-identify and re-configure
// it (never "off until reboot": the learned policy needs the IMU). The pause
// doubles on each failed attempt up to kImuCooldownMaxMs and resets after a
// long clean run.
constexpr uint32_t kImuQuarantineErrors = DOMINO_IMU_OWN_BUS ? 5 : 25;
constexpr uint32_t kImuQuarantineWindowMs = DOMINO_IMU_OWN_BUS ? 1000 : 10000;
constexpr uint32_t kImuCooldownBaseMs = DOMINO_IMU_OWN_BUS ? 300 : 3000;
constexpr uint32_t kImuCooldownMaxMs = 30000;
constexpr uint32_t kImuCleanSamplesToResetCooldown = 1000;  // ~10 s at 100 Hz
#ifdef DOMINO_IMU_POWER_PIN
constexpr uint32_t kImuPowerOffMs = 60;     // long enough to brown the MPU out
constexpr uint32_t kImuPowerOnSettleMs = 100;  // MPU6050 start-up time
#endif
// Default full-scale ranges: +/-2g accel, +/-250 deg/s gyro.
constexpr float kMpuAccelScaleInv = 1.0f / 16384.0f;   // LSB/g at +/-2g.
constexpr float kMpuGyroScaleInv = 1.0f / 131.0f;      // LSB/(deg/s) at +/-250 dps.
constexpr float kImuFilterAlpha = 0.1f;                // Low-pass filter coefficient.
constexpr float kYawStationaryThresholdDps = 0.5f;
constexpr float kYawDeadbandDps = 0.08f;
constexpr float kYawBiasAdaptation = 0.002f;
constexpr uint32_t kImuRetryDelayMs = DOMINO_IMU_OWN_BUS ? 20 : 100;

float wrapDegrees(float degrees) {
  while (degrees >= 180.0f) degrees -= 360.0f;
  while (degrees < -180.0f) degrees += 360.0f;
  return degrees;
}

TwoWire &imuBus() {
#if DOMINO_IMU_OWN_BUS
  return Wire1;
#else
  return Wire;
#endif
}
}  // namespace

ImuState gImuState{};
namespace {
uint32_t i2cErrorCount = 0;
uint32_t consecutiveErrorCount = 0;
uint8_t mpu6050Addr = kMpu6050AddrPrimary;
volatile uint32_t lastValidSampleMs = 0;
uint32_t lastReadStartedMs = 0;
uint32_t windowStartedMs = 0;
uint32_t windowErrorCount = 0;
bool fitted = false;  // found at boot; an absent IMU is never retried
volatile bool quarantined = false;
uint32_t resumeAtMs = 0;
uint32_t cooldownMs = kImuCooldownBaseMs;
uint32_t cleanSamples = 0;
uint32_t reinitCount = 0;
uint32_t lastErrorAtMs = 0;
uint32_t nextReadAttemptMs = 0;
uint8_t lastRequestBytes = 0;
uint32_t lastRequestUs = 0;
uint8_t lastFailedRequestBytes = 0;
uint32_t lastFailedRequestUs = 0;
uint32_t maxRequestUs = 0;
#ifdef DOMINO_IMU_POWER_PIN
bool powerOff = false;
uint32_t powerOnAtMs = 0;
#endif
#if DOMINO_IMU_OWN_BUS
// The task samples into taskState; the control loop copies the newest sample
// into gImuState under this lock, so it never waits on the IMU bus.
ImuState taskState{};
ImuState latestState{};
volatile bool latestFresh = false;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
#endif

bool readRegister(uint8_t address, uint8_t reg, uint8_t &value) {
  TwoWire &bus = imuBus();
  bus.beginTransmission(address);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom(address, static_cast<uint8_t>(1), static_cast<uint8_t>(true)) != 1) return false;
  value = static_cast<uint8_t>(bus.read());
  return true;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  TwoWire &bus = imuBus();
  bus.beginTransmission(address);
  bus.write(reg);
  bus.write(value);
  return bus.endTransmission() == 0;
}

// Write a register and read it back, so a corrupted write is caught at boot.
bool configureRegister(uint8_t address, uint8_t reg, uint8_t value) {
  uint8_t actual = 0;
  return writeRegister(address, reg, value) && readRegister(address, reg, actual) && actual == value;
}

bool identifyDevice(uint8_t address) {
  uint8_t identity = 0;
  return readRegister(address, kWhoAmIRegister, identity) && identity == kWhoAmIValue;
}

// Wake with the X-gyro clock, 44 Hz digital low-pass filter, 100 Hz output
// rate, +/-250 deg/s and +/-2 g (the scales used below), all axes on.
bool configureDevice(uint8_t address) {
  return configureRegister(address, 0x6B, 0x01) &&
      configureRegister(address, 0x1A, 0x03) &&
      configureRegister(address, 0x19, 0x09) &&
      configureRegister(address, 0x1B, 0x00) &&
      configureRegister(address, 0x1C, 0x00) &&
      configureRegister(address, 0x6C, 0x00);
}

// Free the IMU's bus if a transaction left SDA held low.
void freeImuBus() {
#if DOMINO_IMU_OWN_BUS
  if (!i2cLinesIdle(DOMINO_IMU_SDA_PIN, DOMINO_IMU_SCL_PIN)) {
    i2cRecoverBus(Wire1, DOMINO_IMU_SDA_PIN, DOMINO_IMU_SCL_PIN, DOMINO_IMU_I2C_HZ, 5);
  }
#elif !defined(DOMINO_SIL)
  i2cBusEnsureIdle();
#endif
}

// Count a failed transaction, free the bus if the IMU left it stuck, and pause
// the IMU (then re-initialise it) if it keeps failing.
void noteReadError(ImuState &st, uint32_t nowMs) {
  ++i2cErrorCount;
  ++consecutiveErrorCount;
  cleanSamples = 0;
  lastErrorAtMs = nowMs;
  nextReadAttemptMs = nowMs + kImuRetryDelayMs;
  if (windowErrorCount == 0 || nowMs - windowStartedMs > kImuQuarantineWindowMs) {
    windowStartedMs = nowMs;
    windowErrorCount = 0;
  }
  ++windowErrorCount;
  freeImuBus();
  if (windowErrorCount >= kImuQuarantineErrors && !quarantined) {
    quarantined = true;
    st.online = false;
    resumeAtMs = nowMs + cooldownMs;
#ifdef DOMINO_IMU_POWER_PIN
    digitalWrite(DOMINO_IMU_POWER_PIN, LOW);
    powerOff = true;
    powerOnAtMs = nowMs + kImuPowerOffMs;
    if (resumeAtMs < powerOnAtMs + kImuPowerOnSettleMs) resumeAtMs = powerOnAtMs + kImuPowerOnSettleMs;
#endif
    Serial.printf("MPU6050 paused for %lu ms after %lu I2C errors in %lu ms; it will be re-initialised.\n",
                  static_cast<unsigned long>(resumeAtMs - nowMs),
                  static_cast<unsigned long>(windowErrorCount),
                  static_cast<unsigned long>(nowMs - windowStartedMs));
  }
}

// While paused: power the IMU back up (if we cut it), then re-identify and
// re-configure it. Returns true once the IMU is back online.
bool tryResume(ImuState &st, uint32_t nowMs) {
#ifdef DOMINO_IMU_POWER_PIN
  if (powerOff && static_cast<int32_t>(nowMs - powerOnAtMs) >= 0) {
    digitalWrite(DOMINO_IMU_POWER_PIN, HIGH);
    powerOff = false;
  }
#endif
  if (static_cast<int32_t>(nowMs - resumeAtMs) < 0) return false;
  freeImuBus();
  if (identifyDevice(mpu6050Addr) && configureDevice(mpu6050Addr)) {
    quarantined = false;
    st.online = true;
    windowErrorCount = 0;
    consecutiveErrorCount = 0;
    nextReadAttemptMs = 0;
    ++reinitCount;
    Serial.printf("MPU6050 re-initialised at 0x%02X (recovery #%lu).\n", mpu6050Addr,
                  static_cast<unsigned long>(reinitCount));
    return true;
  }
  freeImuBus();
  cooldownMs = cooldownMs * 2 > kImuCooldownMaxMs ? kImuCooldownMaxMs : cooldownMs * 2;
  resumeAtMs = nowMs + cooldownMs;
  Serial.printf("MPU6050 re-initialisation failed; next attempt in %lu ms.\n",
                static_cast<unsigned long>(cooldownMs));
  return false;
}

// One sample into `st` (rate limited to kImuSampleIntervalMs). Returns true
// when a new sample was read.
bool readSampleInto(ImuState &st) {
  if (!fitted) return false;
  const uint32_t nowMs = millis();
  if (quarantined) {
    (void)tryResume(st, nowMs);
    return false;
  }
  if (!st.online) return false;
  if (nextReadAttemptMs != 0 &&
      static_cast<int32_t>(nowMs - nextReadAttemptMs) < 0) {
    return false;
  }
  if (lastReadStartedMs != 0 && nowMs - lastReadStartedMs < kImuSampleIntervalMs) {
    return false;
  }
  lastReadStartedMs = nowMs;
  const uint32_t startedUs = micros();
  TwoWire &bus = imuBus();

  // Read accelerometer, temperature, and gyroscope in one burst starting at 0x3B.
  bus.beginTransmission(mpu6050Addr);
  bus.write(0x3B);  // ACCEL_XOUT_H
  if (bus.endTransmission(false) != 0) {
    noteReadError(st, millis());
#if !defined(DOMINO_SIL) && !DOMINO_IMU_OWN_BUS
    loopBlackboxRecord(BlackboxEventKind::ImuError, 1,
                       micros() - startedUs, bus.getTimeOut());
#endif
    return false;
  }

  constexpr uint8_t kReadLen = 14;
  const uint32_t requestStartedUs = micros();
  uint8_t readCount = bus.requestFrom(mpu6050Addr, kReadLen, (uint8_t)true);
  const uint32_t requestDurationUs = micros() - requestStartedUs;
  lastRequestBytes = readCount;
  lastRequestUs = requestDurationUs;
  if (requestDurationUs > maxRequestUs) maxRequestUs = requestDurationUs;
  if (readCount != kReadLen) {
    lastFailedRequestBytes = readCount;
    lastFailedRequestUs = requestDurationUs;
    // Drain whatever arrived so the next burst starts aligned.
    while (bus.available()) (void)bus.read();
    noteReadError(st, millis());
#if !defined(DOMINO_SIL) && !DOMINO_IMU_OWN_BUS
    loopBlackboxRecord(BlackboxEventKind::ImuError, 2,
                       micros() - startedUs, readCount);
#endif
    return false;
  }
  (void)startedUs;

  auto read16 = [&bus]() -> int16_t {
    const int16_t hi = bus.read();
    const int16_t lo = bus.read();
    return static_cast<int16_t>((hi << 8) | lo);
  };

  st.ax_raw = read16();
  st.ay_raw = read16();
  st.az_raw = read16();
  (void)read16();  // temperature, unused
  st.gx_raw = read16();
  st.gy_raw = read16();
  st.gz_raw = read16();
  lastValidSampleMs = millis();
  consecutiveErrorCount = 0;
  nextReadAttemptMs = 0;
  if (++cleanSamples >= kImuCleanSamplesToResetCooldown) cooldownMs = kImuCooldownBaseMs;

  st.ax_g = static_cast<float>(st.ax_raw) * kMpuAccelScaleInv;
  st.ay_g = static_cast<float>(st.ay_raw) * kMpuAccelScaleInv;
  st.az_g = static_cast<float>(st.az_raw) * kMpuAccelScaleInv;

  st.gx_dps = static_cast<float>(st.gx_raw) * kMpuGyroScaleInv;
  st.gy_dps = static_cast<float>(st.gy_raw) * kMpuGyroScaleInv;
  st.gz_dps = static_cast<float>(st.gz_raw) * kMpuGyroScaleInv;

  // First-order low-pass filter for smoother values.
  if (!st.has_sample) {
    st.ax_g_filt = st.ax_g;
    st.ay_g_filt = st.ay_g;
    st.az_g_filt = st.az_g;
    st.gx_dps_filt = st.gx_dps;
    st.gy_dps_filt = st.gy_dps;
    st.gz_dps_filt = st.gz_dps;
    st.has_sample = true;
  } else {
    const float a = kImuFilterAlpha;
    st.ax_g_filt += a * (st.ax_g - st.ax_g_filt);
    st.ay_g_filt += a * (st.ay_g - st.ay_g_filt);
    st.az_g_filt += a * (st.az_g - st.az_g_filt);
    st.gx_dps_filt += a * (st.gx_dps - st.gx_dps_filt);
    st.gy_dps_filt += a * (st.gy_dps - st.gy_dps_filt);
    st.gz_dps_filt += a * (st.gz_dps - st.gz_dps_filt);
  }

  // Domino's mounted body yaw axis (+Z up) maps to negative sensor X.
  // Gravity cannot provide heading, so this remains a relative gyro heading.
  const float mountedYawRateDps = -st.gx_dps_filt;
  if (!st.yaw_initialized) {
    st.yaw_initialized = true;
    st.yaw_sample_ms = nowMs;
    st.yaw_bias_dps = mountedYawRateDps;
    st.yaw_rate_dps = 0.0f;
    st.yaw_deg = 0.0f;
  } else {
    const uint32_t elapsedMs = nowMs - st.yaw_sample_ms;
    st.yaw_sample_ms = nowMs;
    const float dt = fminf(static_cast<float>(elapsedMs), 100.0f) * 0.001f;
    float correctedRateDps = mountedYawRateDps - st.yaw_bias_dps;
    if (fabsf(correctedRateDps) < kYawStationaryThresholdDps) {
      st.yaw_bias_dps += kYawBiasAdaptation * correctedRateDps;
      correctedRateDps = mountedYawRateDps - st.yaw_bias_dps;
    }
    if (fabsf(correctedRateDps) < kYawDeadbandDps) correctedRateDps = 0.0f;
    st.yaw_rate_dps = correctedRateDps;
    st.yaw_deg = wrapDegrees(st.yaw_deg + correctedRateDps * dt);
  }
  return true;
}

#if DOMINO_IMU_OWN_BUS
void imuTask(void *) {
  for (;;) {
    if (readSampleInto(taskState)) {
      portENTER_CRITICAL(&stateMux);
      latestState = taskState;
      latestFresh = true;
      portEXIT_CRITICAL(&stateMux);
    } else if (!taskState.online) {
      // Paused or re-initialising: publish the offline flag too.
      portENTER_CRITICAL(&stateMux);
      latestState.online = taskState.online;
      latestFresh = true;
      portEXIT_CRITICAL(&stateMux);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
#endif
}  // namespace

uint32_t imuI2cErrorCount() { return i2cErrorCount; }
bool imuQuarantined() { return quarantined; }
uint32_t imuReinitCount() { return reinitCount; }
bool imuOwnBus() { return DOMINO_IMU_OWN_BUS; }
uint8_t imuAddress() { return mpu6050Addr; }
uint32_t imuLastErrorAtMs() { return lastErrorAtMs; }
uint32_t imuConsecutiveErrorCount() { return consecutiveErrorCount; }
uint8_t imuLastRequestBytes() { return lastRequestBytes; }
uint32_t imuLastRequestUs() { return lastRequestUs; }
uint8_t imuLastFailedRequestBytes() { return lastFailedRequestBytes; }
uint32_t imuLastFailedRequestUs() { return lastFailedRequestUs; }
uint32_t imuMaxRequestUs() { return maxRequestUs; }
uint32_t imuSampleAgeMs(uint32_t now) {
  return gImuState.has_sample ? now - lastValidSampleMs : UINT32_MAX;
}
bool imuSampleFresh(uint32_t now) {
  return gImuState.online && gImuState.has_sample &&
      imuSampleAgeMs(now) <= IMU_SAMPLE_FRESH_MS;
}

void imuInit() {
  gImuState = ImuState{};
  i2cErrorCount = 0;
  consecutiveErrorCount = 0;
  lastValidSampleMs = 0;
  lastErrorAtMs = 0;
  nextReadAttemptMs = 0;
  lastReadStartedMs = 0;
  windowStartedMs = 0;
  windowErrorCount = 0;
  quarantined = false;
  cooldownMs = kImuCooldownBaseMs;
  cleanSamples = 0;
  reinitCount = 0;
  lastRequestBytes = 0;
  lastRequestUs = 0;
  lastFailedRequestBytes = 0;
  lastFailedRequestUs = 0;
  maxRequestUs = 0;
  mpu6050Addr = kMpu6050AddrPrimary;

#ifdef DOMINO_IMU_POWER_PIN
  pinMode(DOMINO_IMU_POWER_PIN, OUTPUT);
  digitalWrite(DOMINO_IMU_POWER_PIN, HIGH);
  delay(kImuPowerOnSettleMs);
#endif
#if DOMINO_IMU_OWN_BUS
  Wire1.begin(DOMINO_IMU_SDA_PIN, DOMINO_IMU_SCL_PIN, DOMINO_IMU_I2C_HZ);
  Wire1.setTimeOut(5);
#endif
  freeImuBus();

  bool configured = false;
  fitted = false;
  for (uint8_t candidate : {kMpu6050AddrPrimary, kMpu6050AddrSecondary}) {
    if (!identifyDevice(candidate)) continue;
    fitted = true;
    mpu6050Addr = candidate;
    configured = configureDevice(candidate);
    break;
  }
  gImuState.online = configured;
  if (configured) {
    Serial.printf("MPU6050 detected and configured at 0x%02X on %s.\n", mpu6050Addr,
                  DOMINO_IMU_OWN_BUS ? "its own I2C bus" : "the shared servo I2C bus");
  } else if (fitted) {
    // Found but not accepting its configuration: let the recovery path retry.
    Serial.printf("MPU6050 at 0x%02X did not accept its configuration; retrying.\n", mpu6050Addr);
    quarantined = true;
    resumeAtMs = millis() + cooldownMs;
  } else {
    Serial.println("MPU6050 not found at 0x68 or 0x69; IMU disabled.");
  }
  // Probing an absent or misbehaving device must not leave the bus stuck.
  freeImuBus();
#if DOMINO_IMU_OWN_BUS
  if (fitted) {
    taskState = gImuState;
    latestState = gImuState;
    xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 3, nullptr, 0);
  }
#endif
}

bool imuReadSample() {
#if DOMINO_IMU_OWN_BUS
  if (!latestFresh) return false;
  portENTER_CRITICAL(&stateMux);
  const bool hadSample = latestState.has_sample;
  gImuState = latestState;
  latestFresh = false;
  portEXIT_CRITICAL(&stateMux);
  return hadSample && gImuState.online;
#else
  return readSampleInto(gImuState);
#endif
}
