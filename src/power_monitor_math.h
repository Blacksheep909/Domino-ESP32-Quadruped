#pragma once

#include <stdint.h>

struct PowerMonitorScale {
  bool valid;
  float currentLsbA;
  uint16_t calibration;
};

PowerMonitorScale makeIna226Scale(float shuntOhms, float maximumCurrentA);
float ina226BusVoltageV(uint16_t raw);
float ina226CurrentA(uint16_t raw, const PowerMonitorScale &scale);
float ina226PowerW(uint16_t raw, const PowerMonitorScale &scale);
