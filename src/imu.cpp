#include "imu.h"

#include <Wire.h>
#ifndef DOMINO_SIL
#include "i2c_bus.h"
#include "loop_diagnostics.h"
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
// Protect the servos: if the IMU keeps failing, stop reading it until reboot.
constexpr uint32_t kImuQuarantineErrors = 25;
constexpr uint32_t kImuQuarantineWindowMs = 10000;
// Default full-scale ranges: +/-2g accel, +/-250 deg/s gyro.
constexpr float kMpuAccelScaleInv = 1.0f / 16384.0f;   // LSB/g at +/-2g.
constexpr float kMpuGyroScaleInv = 1.0f / 131.0f;      // LSB/(deg/s) at +/-250 dps.
constexpr float kImuFilterAlpha = 0.1f;                // Low-pass filter coefficient.
constexpr float kYawStationaryThresholdDps = 0.5f;
constexpr float kYawDeadbandDps = 0.08f;
constexpr float kYawBiasAdaptation = 0.002f;
constexpr uint32_t kImuRetryDelayMs = 100;

float wrapDegrees(float degrees) {
  while (degrees >= 180.0f) degrees -= 360.0f;
  while (degrees < -180.0f) degrees += 360.0f;
  return degrees;
}
}  // namespace

ImuState gImuState{};
namespace {
uint32_t i2cErrorCount = 0;
uint32_t consecutiveErrorCount = 0;
uint8_t mpu6050Addr = kMpu6050AddrPrimary;
uint32_t lastValidSampleMs = 0;
uint32_t lastReadStartedMs = 0;
uint32_t windowStartedMs = 0;
uint32_t windowErrorCount = 0;
bool quarantined = false;
uint32_t lastErrorAtMs = 0;
uint32_t nextReadAttemptMs = 0;
uint8_t lastRequestBytes = 0;
uint32_t lastRequestUs = 0;
uint8_t lastFailedRequestBytes = 0;
uint32_t lastFailedRequestUs = 0;
uint32_t maxRequestUs = 0;

bool readRegister(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, static_cast<uint8_t>(1), static_cast<uint8_t>(true)) != 1) return false;
  value = static_cast<uint8_t>(Wire.read());
  return true;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
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

// Count a failed transaction, free the bus if the IMU left it stuck, and stop
// reading the IMU altogether if it keeps failing.
void noteReadError(uint32_t nowMs) {
  ++i2cErrorCount;
  ++consecutiveErrorCount;
  lastErrorAtMs = nowMs;
  nextReadAttemptMs = nowMs + kImuRetryDelayMs;
  if (windowErrorCount == 0 || nowMs - windowStartedMs > kImuQuarantineWindowMs) {
    windowStartedMs = nowMs;
    windowErrorCount = 0;
  }
  ++windowErrorCount;
#ifndef DOMINO_SIL
  i2cBusEnsureIdle();
#endif
  if (windowErrorCount >= kImuQuarantineErrors) {
    quarantined = true;
    gImuState.online = false;
    Serial.printf("MPU6050 disabled until reboot: %lu I2C errors in %lu ms (protecting the servo bus).\n",
                  static_cast<unsigned long>(windowErrorCount),
                  static_cast<unsigned long>(nowMs - windowStartedMs));
  }
}
}

uint32_t imuI2cErrorCount() { return i2cErrorCount; }
bool imuQuarantined() { return quarantined; }
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
  lastRequestBytes = 0;
  lastRequestUs = 0;
  lastFailedRequestBytes = 0;
  lastFailedRequestUs = 0;
  maxRequestUs = 0;
  mpu6050Addr = kMpu6050AddrPrimary;

  bool found = false;
  bool configured = false;
  for (uint8_t candidate : {kMpu6050AddrPrimary, kMpu6050AddrSecondary}) {
    if (!identifyDevice(candidate)) continue;
    found = true;
    mpu6050Addr = candidate;
    // Wake with the X-gyro clock, 44 Hz digital low-pass filter, 100 Hz
    // output rate, +/-250 deg/s and +/-2 g (the scales used below), all axes on.
    configured = configureRegister(candidate, 0x6B, 0x01) &&
        configureRegister(candidate, 0x1A, 0x03) &&
        configureRegister(candidate, 0x19, 0x09) &&
        configureRegister(candidate, 0x1B, 0x00) &&
        configureRegister(candidate, 0x1C, 0x00) &&
        configureRegister(candidate, 0x6C, 0x00);
    break;
  }
  gImuState.online = configured;
  if (configured) {
    Serial.printf("MPU6050 detected and configured at 0x%02X.\n", mpu6050Addr);
  } else if (found) {
    Serial.printf("MPU6050 at 0x%02X did not accept its configuration; IMU disabled.\n", mpu6050Addr);
  } else {
    Serial.println("MPU6050 not found at 0x68 or 0x69; IMU disabled.");
  }
#ifndef DOMINO_SIL
  // Probing an absent or misbehaving device must not leave the bus stuck for
  // the servo driver.
  i2cBusEnsureIdle();
#endif
}

bool imuReadSample() {
  if (!gImuState.online) {
    return false;
  }
  const uint32_t nowMs = millis();
  if (nextReadAttemptMs != 0 &&
      static_cast<int32_t>(nowMs - nextReadAttemptMs) < 0) {
    return false;
  }
  if (lastReadStartedMs != 0 && nowMs - lastReadStartedMs < kImuSampleIntervalMs) {
    return false;
  }
  lastReadStartedMs = nowMs;
  const uint32_t startedUs = micros();

  // Read accelerometer, temperature, and gyroscope in one burst starting at 0x3B.
  Wire.beginTransmission(mpu6050Addr);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    noteReadError(millis());
#ifndef DOMINO_SIL
    loopBlackboxRecord(BlackboxEventKind::ImuError, 1,
                       micros() - startedUs, Wire.getTimeOut());
#endif
    return false;
  }

  constexpr uint8_t kReadLen = 14;
  const uint32_t requestStartedUs = micros();
  uint8_t readCount = Wire.requestFrom(mpu6050Addr, kReadLen, (uint8_t)true);
  const uint32_t requestDurationUs = micros() - requestStartedUs;
  lastRequestBytes = readCount;
  lastRequestUs = requestDurationUs;
  if (requestDurationUs > maxRequestUs) maxRequestUs = requestDurationUs;
  if (readCount != kReadLen) {
    lastFailedRequestBytes = readCount;
    lastFailedRequestUs = requestDurationUs;
    // Drain whatever arrived so the next burst starts aligned.
    while (Wire.available()) (void)Wire.read();
    noteReadError(millis());
#ifndef DOMINO_SIL
    loopBlackboxRecord(BlackboxEventKind::ImuError, 2,
                       micros() - startedUs, readCount);
#endif
    return false;
  }

  auto read16 = []() -> int16_t {
    const int16_t hi = Wire.read();
    const int16_t lo = Wire.read();
    return static_cast<int16_t>((hi << 8) | lo);
  };

  gImuState.ax_raw = read16();
  gImuState.ay_raw = read16();
  gImuState.az_raw = read16();
  (void)read16();  // temperature, unused
  gImuState.gx_raw = read16();
  gImuState.gy_raw = read16();
  gImuState.gz_raw = read16();
  lastValidSampleMs = millis();
  consecutiveErrorCount = 0;
  nextReadAttemptMs = 0;

  gImuState.ax_g = static_cast<float>(gImuState.ax_raw) * kMpuAccelScaleInv;
  gImuState.ay_g = static_cast<float>(gImuState.ay_raw) * kMpuAccelScaleInv;
  gImuState.az_g = static_cast<float>(gImuState.az_raw) * kMpuAccelScaleInv;

  gImuState.gx_dps = static_cast<float>(gImuState.gx_raw) * kMpuGyroScaleInv;
  gImuState.gy_dps = static_cast<float>(gImuState.gy_raw) * kMpuGyroScaleInv;
  gImuState.gz_dps = static_cast<float>(gImuState.gz_raw) * kMpuGyroScaleInv;

  // First-order low-pass filter for smoother values.
  if (!gImuState.has_sample) {
    gImuState.ax_g_filt = gImuState.ax_g;
    gImuState.ay_g_filt = gImuState.ay_g;
    gImuState.az_g_filt = gImuState.az_g;
    gImuState.gx_dps_filt = gImuState.gx_dps;
    gImuState.gy_dps_filt = gImuState.gy_dps;
    gImuState.gz_dps_filt = gImuState.gz_dps;
    gImuState.has_sample = true;
  } else {
    const float a = kImuFilterAlpha;
    gImuState.ax_g_filt += a * (gImuState.ax_g - gImuState.ax_g_filt);
    gImuState.ay_g_filt += a * (gImuState.ay_g - gImuState.ay_g_filt);
    gImuState.az_g_filt += a * (gImuState.az_g - gImuState.az_g_filt);
    gImuState.gx_dps_filt += a * (gImuState.gx_dps - gImuState.gx_dps_filt);
    gImuState.gy_dps_filt += a * (gImuState.gy_dps - gImuState.gy_dps_filt);
    gImuState.gz_dps_filt += a * (gImuState.gz_dps - gImuState.gz_dps_filt);
  }

  // Domino's mounted body yaw axis (+Z up) maps to negative sensor X.
  // Gravity cannot provide heading, so this remains a relative gyro heading.
  const float mountedYawRateDps = -gImuState.gx_dps_filt;
  if (!gImuState.yaw_initialized) {
    gImuState.yaw_initialized = true;
    gImuState.yaw_sample_ms = nowMs;
    gImuState.yaw_bias_dps = mountedYawRateDps;
    gImuState.yaw_rate_dps = 0.0f;
    gImuState.yaw_deg = 0.0f;
  } else {
    const uint32_t elapsedMs = nowMs - gImuState.yaw_sample_ms;
    gImuState.yaw_sample_ms = nowMs;
    const float dt = fminf(static_cast<float>(elapsedMs), 100.0f) * 0.001f;
    float correctedRateDps = mountedYawRateDps - gImuState.yaw_bias_dps;
    if (fabsf(correctedRateDps) < kYawStationaryThresholdDps) {
      gImuState.yaw_bias_dps += kYawBiasAdaptation * correctedRateDps;
      correctedRateDps = mountedYawRateDps - gImuState.yaw_bias_dps;
    }
    if (fabsf(correctedRateDps) < kYawDeadbandDps) correctedRateDps = 0.0f;
    gImuState.yaw_rate_dps = correctedRateDps;
    gImuState.yaw_deg = wrapDegrees(gImuState.yaw_deg + correctedRateDps * dt);
  }

  return true;
}
