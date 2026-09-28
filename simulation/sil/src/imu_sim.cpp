#include "imu.h"

#include "sim_imu.h"

ImuState gImuState{};
namespace { uint32_t lastValidSampleMs = 0; }

uint32_t imuI2cErrorCount() { return 0; }
uint32_t imuLastErrorAtMs() { return 0; }
uint32_t imuConsecutiveErrorCount() { return 0; }
uint32_t imuSampleAgeMs(uint32_t now) {
  return gImuState.has_sample ? now - lastValidSampleMs : UINT32_MAX;
}
bool imuSampleFresh(uint32_t now) {
  return gImuState.online && gImuState.has_sample &&
      imuSampleAgeMs(now) <= IMU_SAMPLE_FRESH_MS;
}

void simSetImuGravity(float axG, float ayG, float azG) {
  gImuState.ax_g = axG;
  gImuState.ay_g = ayG;
  gImuState.az_g = azG;
  gImuState.ax_g_filt = axG;
  gImuState.ay_g_filt = ayG;
  gImuState.az_g_filt = azG;
}

void imuInit() {
  gImuState = ImuState{};
  gImuState.online = true;
  gImuState.has_sample = true;
  lastValidSampleMs = millis();
  simSetImuGravity(-1.0f, 0.0f, 0.0f);
  Serial.println("SIL IMU initialized.");
}

bool imuReadSample() {
  lastValidSampleMs = millis();
  return gImuState.online && gImuState.has_sample;
}
