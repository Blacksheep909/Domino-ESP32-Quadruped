#include "imu.h"

#include "sim_imu.h"

ImuState gImuState{};

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
  simSetImuGravity(-1.0f, 0.0f, 0.0f);
  Serial.println("SIL IMU initialized.");
}

bool imuReadSample() {
  return gImuState.online && gImuState.has_sample;
}
