#include "power_monitor_math.h"

#include <math.h>

PowerMonitorScale makeIna226Scale(float shuntOhms, float maximumCurrentA) {
  PowerMonitorScale scale{};
  if (!isfinite(shuntOhms) || !isfinite(maximumCurrentA) ||
      shuntOhms <= 0.0f || maximumCurrentA <= 0.0f) return scale;
  scale.currentLsbA = maximumCurrentA / 32768.0f;
  const float calibration = 0.00512f / (scale.currentLsbA * shuntOhms);
  const uint32_t rounded = static_cast<uint32_t>(lroundf(calibration));
  if (!isfinite(calibration) || rounded == 0 || rounded > 65535) return PowerMonitorScale{};
  scale.calibration = static_cast<uint16_t>(rounded);
  scale.valid = true;
  return scale;
}

float ina226BusVoltageV(uint16_t raw) { return static_cast<float>(raw) * 0.00125f; }

float ina226CurrentA(uint16_t raw, const PowerMonitorScale &scale) {
  return scale.valid ? static_cast<float>(static_cast<int16_t>(raw)) * scale.currentLsbA : 0.0f;
}

float ina226PowerW(uint16_t raw, const PowerMonitorScale &scale) {
  return scale.valid ? static_cast<float>(raw) * 25.0f * scale.currentLsbA : 0.0f;
}
