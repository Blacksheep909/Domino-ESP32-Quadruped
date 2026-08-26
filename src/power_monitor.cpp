#include "power_monitor.h"

#include <Wire.h>
#include <math.h>

#include "power_monitor_math.h"

// Domino PCB V1.1B carries the SpotMicro-style 0-25 V divider module on
// ESP32 SENSOR_VP / GPIO36. Its nominal 30 kOhm / 7.5 kOhm divider is 5:1.
#ifndef DOMINO_VOLTAGE_MONITOR_ENABLED
#define DOMINO_VOLTAGE_MONITOR_ENABLED 1
#endif
#ifndef DOMINO_VOLTAGE_ADC_PIN
#define DOMINO_VOLTAGE_ADC_PIN 36
#endif
#ifndef DOMINO_VOLTAGE_DIVIDER_RATIO_MILLI
#define DOMINO_VOLTAGE_DIVIDER_RATIO_MILLI 5000
#endif
#ifndef DOMINO_VOLTAGE_CALIBRATION_PPM
#define DOMINO_VOLTAGE_CALIBRATION_PPM 1000000
#endif

// Optional INA226 support remains available for measured current and power.
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
constexpr uint8_t kAdcSamples = 16;

PowerMonitorSample latest{};
uint32_t lastAttemptMs = 0;
PowerMonitorScale scale{};
bool inaOnline = false;

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

bool configureIna226() {
#if DOMINO_POWER_MONITOR_ENABLED
  return scale.valid && writeRegister(kConfigurationRegister, kContinuousConfiguration) &&
      writeRegister(kCalibrationRegister, scale.calibration);
#else
  return false;
#endif
}
}  // namespace

void powerMonitorBegin() {
  latest = PowerMonitorSample{};
#if DOMINO_VOLTAGE_MONITOR_ENABLED
  pinMode(DOMINO_VOLTAGE_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(DOMINO_VOLTAGE_ADC_PIN, ADC_11db);
  latest.online = true;
#endif
#if DOMINO_POWER_MONITOR_ENABLED
  const float shuntOhms = static_cast<float>(DOMINO_POWER_SHUNT_MICRO_OHMS) / 1000000.0f;
  const float maximumCurrentA = static_cast<float>(DOMINO_POWER_MAX_CURRENT_MILLIAMPS) / 1000.0f;
  scale = makeIna226Scale(shuntOhms, maximumCurrentA);
  inaOnline = configureIna226();
  latest.online = latest.online || inaOnline;
#endif
}

void powerMonitorUpdate(uint32_t now) {
  if (now - lastAttemptMs < kSampleIntervalMs) return;
  lastAttemptMs = now;

  latest.voltageValid = false;
  latest.currentValid = false;
  latest.powerValid = false;

#if DOMINO_VOLTAGE_MONITOR_ENABLED
  uint32_t adcTotalMv = 0;
  for (uint8_t index = 0; index < kAdcSamples; ++index) {
    adcTotalMv += analogReadMilliVolts(DOMINO_VOLTAGE_ADC_PIN);
  }
  latest.voltageV = dividerBatteryVoltageV(
      adcTotalMv / kAdcSamples,
      static_cast<float>(DOMINO_VOLTAGE_DIVIDER_RATIO_MILLI) / 1000.0f,
      static_cast<float>(DOMINO_VOLTAGE_CALIBRATION_PPM) / 1000000.0f);
  latest.voltageValid = isfinite(latest.voltageV) && latest.voltageV >= 0.0f &&
      latest.voltageV <= 25.5f;
#endif

#if DOMINO_POWER_MONITOR_ENABLED
  if (!inaOnline) inaOnline = configureIna226();
  uint16_t busRaw = 0;
  uint16_t currentRaw = 0;
  uint16_t powerRaw = 0;
  if (inaOnline && (!readRegister(kBusVoltageRegister, &busRaw) ||
      !readRegister(kCurrentRegister, &currentRaw) ||
      !readRegister(kPowerRegister, &powerRaw))) {
    inaOnline = false;
  }
  if (inaOnline) {
#if !DOMINO_VOLTAGE_MONITOR_ENABLED
    latest.voltageV = ina226BusVoltageV(busRaw);
    latest.voltageValid = isfinite(latest.voltageV) && latest.voltageV >= 0.0f;
#endif
    latest.currentA = ina226CurrentA(currentRaw, scale);
    latest.powerW = ina226PowerW(powerRaw, scale);
    latest.currentValid = isfinite(latest.currentA);
    latest.powerValid = isfinite(latest.powerW);
  }
#endif

  latest.online = latest.online || inaOnline;
  latest.valid = latest.voltageValid || latest.currentValid || latest.powerValid;
  if (latest.valid) latest.timestampMs = now;
}

PowerMonitorSample powerMonitorSample(uint32_t now) {
  PowerMonitorSample sample = latest;
  if (!sample.valid || now - sample.timestampMs > kFreshMs) {
    sample.valid = false;
    sample.voltageValid = false;
    sample.currentValid = false;
    sample.powerValid = false;
  }
  return sample;
}
