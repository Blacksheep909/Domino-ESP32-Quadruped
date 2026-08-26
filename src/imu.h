#pragma once

#include <Arduino.h>

// Simple container for the most recent IMU sample in the sensor frame.
struct ImuState {
  bool online;
  bool has_sample;
  int16_t ax_raw;
  int16_t ay_raw;
  int16_t az_raw;
  int16_t gx_raw;
  int16_t gy_raw;
  int16_t gz_raw;
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
  // Simple low-pass filtered values for smoother logging / estimation.
  float ax_g_filt;
  float ay_g_filt;
  float az_g_filt;
  float gx_dps_filt;
  float gy_dps_filt;
  float gz_dps_filt;
  // Relative gyro heading. The MPU6050 has no magnetometer, so yaw can drift.
  bool yaw_initialized;
  uint32_t yaw_sample_ms;
  float yaw_deg;
  float yaw_rate_dps;
  float yaw_bias_dps;
};

extern ImuState gImuState;

// Initialize the MPU6050 on the I2C bus. Safe to call once from setup().
void imuInit();

// Read a fresh sample into gImuState. Returns true on success.
bool imuReadSample();
