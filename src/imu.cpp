#include "imu.h"

#include <Wire.h>
#ifndef DOMINO_SIL
#include "loop_diagnostics.h"
#endif

namespace {
// IMU (MPU6050) configuration.
constexpr uint8_t kMpu6050Addr = 0x68;
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
uint32_t lastValidSampleMs = 0;
uint32_t lastErrorAtMs = 0;
uint32_t nextReadAttemptMs = 0;
uint8_t lastRequestBytes = 0;
uint32_t lastRequestUs = 0;
uint8_t lastFailedRequestBytes = 0;
uint32_t lastFailedRequestUs = 0;
uint32_t maxRequestUs = 0;
}

uint32_t imuI2cErrorCount() { return i2cErrorCount; }
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
  // Wake up MPU6050 by clearing the sleep bit in PWR_MGMT_1.
  Wire.beginTransmission(kMpu6050Addr);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // set to zero (wakes up the MPU-6050)
  uint8_t status = Wire.endTransmission();
  i2cErrorCount = 0;
  consecutiveErrorCount = 0;
  lastValidSampleMs = 0;
  lastErrorAtMs = 0;
  nextReadAttemptMs = 0;
  lastRequestBytes = 0;
  lastRequestUs = 0;
  lastFailedRequestBytes = 0;
  lastFailedRequestUs = 0;
  maxRequestUs = 0;
  gImuState.online = (status == 0);
  gImuState.has_sample = false;
  gImuState.yaw_initialized = false;
  gImuState.yaw_deg = 0.0f;
  if (gImuState.online) {
    Serial.println("MPU6050 detected and initialized.");
  } else {
    Serial.printf("MPU6050 init failed, I2C status=%u\n", status);
  }
}

bool imuReadSample() {
  if (!gImuState.online) {
    return false;
  }
  if (nextReadAttemptMs != 0 &&
      static_cast<int32_t>(millis() - nextReadAttemptMs) < 0) {
    return false;
  }
  const uint32_t startedUs = micros();

  // Read accelerometer, temperature, and gyroscope in one burst starting at 0x3B.
  Wire.beginTransmission(kMpu6050Addr);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    ++i2cErrorCount;
    ++consecutiveErrorCount;
    lastErrorAtMs = millis();
    nextReadAttemptMs = lastErrorAtMs + kImuRetryDelayMs;
#ifndef DOMINO_SIL
    loopBlackboxRecord(BlackboxEventKind::ImuError, 1,
                       micros() - startedUs, Wire.getTimeOut());
#endif
    return false;
  }

  constexpr uint8_t kReadLen = 14;
  const uint32_t requestStartedUs = micros();
  uint8_t readCount = Wire.requestFrom(kMpu6050Addr, kReadLen, (uint8_t)true);
  const uint32_t requestDurationUs = micros() - requestStartedUs;
  lastRequestBytes = readCount;
  lastRequestUs = requestDurationUs;
  if (requestDurationUs > maxRequestUs) maxRequestUs = requestDurationUs;
  if (readCount != kReadLen) {
    lastFailedRequestBytes = readCount;
    lastFailedRequestUs = requestDurationUs;
    ++i2cErrorCount;
    ++consecutiveErrorCount;
    lastErrorAtMs = millis();
    nextReadAttemptMs = lastErrorAtMs + kImuRetryDelayMs;
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
  const uint32_t nowMs = millis();
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
