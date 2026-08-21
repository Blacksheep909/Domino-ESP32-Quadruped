#include "power_monitor.h"

#include <Wire.h>
#include <math.h>

#include "power_monitor_math.h"

#ifndef DOMINO_POWER_MONITOR_ENABLED
#define DOMINO_POWER_MONITOR_ENABLED 0
#endif
#ifndef DOMINO_POWER_MONITOR_ADDRESS
#define DOMINO_POWER_MONITOR_ADDRESS 0x40
#endif
#ifndef DOMINO_POWER_SHUNT_MICRO_OHMS
#define DOMINO_POWER_SHUNT_MICRO_OHMS 2000
#endif
#ifndef DOMINO_POWER_MAX_CURRENT_MILLIAMPS
#define DOMINO_POWER_MAX_CURRENT_MILLIAMPS 30000
#endif

namespace {
constexpr uint8_t kConfigurationRegister = 0x00;
constexpr uint8_t kBusVoltageRegister = 0x02;
constexpr uint8_t kPowerRegister = 0x03;
constexpr uint8_t kCurrentRegister = 0x04;
constexpr uint8_t kCalibrationRegister = 0x05;
constexpr uint16_t kContinuousConfiguration = 0x4127;
constexpr uint32_t kSampleIntervalMs = 100;
constexpr uint32_t kFreshMs = 500;

PowerMonitorSample latest{};
uint32_t lastAttemptMs = 0;
PowerMonitorScale scale{};

bool writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(DOMINO_POWER_MONITOR_ADDRESS);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>(value >> 8));
  Wire.write(static_cast<uint8_t>(value & 0xff));
  return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t reg, uint16_t *value) {
  if (!value) return false;
  Wire.beginTransmission(DOMINO_POWER_MONITOR_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(
      static_cast<uint8_t>(DOMINO_POWER_MONITOR_ADDRESS), static_cast<uint8_t>(2)) != 2) return false;
  *value = static_cast<uint16_t>(Wire.read()) << 8;
  *value |= static_cast<uint16_t>(Wire.read());
  return true;
}
}  // namespace

void powerMonitorBegin() {
#if DOMINO_POWER_MONITOR_ENABLED
  const float shuntOhms = static_cast<float>(DOMINO_POWER_SHUNT_MICRO_OHMS) / 1000000.0f;
  const float maximumCurrentA = static_cast<float>(DOMINO_POWER_MAX_CURRENT_MILLIAMPS) / 1000.0f;
  scale = makeIna226Scale(shuntOhms, maximumCurrentA);
  if (!scale.valid) {
    latest = PowerMonitorSample{};
    return;
  }
  latest.online = writeRegister(kConfigurationRegister, kContinuousConfiguration) &&
      writeRegister(kCalibrationRegister, scale.calibration);
#else
  latest = PowerMonitorSample{};
#endif
}

void powerMonitorUpdate(uint32_t now) {
#if DOMINO_POWER_MONITOR_ENABLED
  if (now - lastAttemptMs < kSampleIntervalMs) return;
  lastAttemptMs = now;
  if (!latest.online) {
    powerMonitorBegin();
    if (!latest.online) return;
  }
  uint16_t busRaw = 0;
  uint16_t currentRaw = 0;
  uint16_t powerRaw = 0;
  if (!readRegister(kBusVoltageRegister, &busRaw) ||
      !readRegister(kCurrentRegister, &currentRaw) ||
      !readRegister(kPowerRegister, &powerRaw)) {
    latest.online = false;
    latest.valid = false;
    return;
  }
  latest.timestampMs = now;
  latest.voltageV = ina226BusVoltageV(busRaw);
  latest.currentA = ina226CurrentA(currentRaw, scale);
  latest.powerW = ina226PowerW(powerRaw, scale);
  latest.valid = isfinite(latest.voltageV) && isfinite(latest.currentA) &&
      isfinite(latest.powerW) && latest.voltageV >= 0.0f;
#else
  (void)now;
#endif
}

PowerMonitorSample powerMonitorSample(uint32_t now) {
  PowerMonitorSample sample = latest;
  sample.valid = sample.valid && now - sample.timestampMs <= kFreshMs;
  return sample;
}
