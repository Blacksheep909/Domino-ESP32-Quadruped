#include "imu.h"

#include <Wire.h>

namespace {
// IMU (MPU6050) configuration.
constexpr uint8_t kMpu6050Addr = 0x68;
// Default full-scale ranges: +/-2g accel, +/-250 deg/s gyro.
constexpr float kMpuAccelScaleInv = 1.0f / 16384.0f;   // LSB/g at +/-2g.
constexpr float kMpuGyroScaleInv = 1.0f / 131.0f;      // LSB/(deg/s) at +/-250 dps.
constexpr float kImuFilterAlpha = 0.1f;                // Low-pass filter coefficient.
}  // namespace

ImuState gImuState{};

void imuInit() {
  // Wake up MPU6050 by clearing the sleep bit in PWR_MGMT_1.
  Wire.beginTransmission(kMpu6050Addr);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // set to zero (wakes up the MPU-6050)
  uint8_t status = Wire.endTransmission();
  gImuState.online = (status == 0);
  gImuState.has_sample = false;
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

  // Read accelerometer, temperature, and gyroscope in one burst starting at 0x3B.
  Wire.beginTransmission(kMpu6050Addr);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  constexpr uint8_t kReadLen = 14;
  uint8_t readCount = Wire.requestFrom(kMpu6050Addr, kReadLen, (uint8_t)true);
  if (readCount != kReadLen) {
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

  return true;
}
