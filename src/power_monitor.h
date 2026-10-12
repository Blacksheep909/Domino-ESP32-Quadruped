#pragma once

#include <Arduino.h>

struct PowerMonitorSample {
  bool online = false;
  bool valid = false;
  bool voltageValid = false;
  bool currentValid = false;
  bool powerValid = false;
  uint32_t timestampMs = 0;
  float voltageV = 0.0f;
  float currentA = 0.0f;
  float powerW = 0.0f;
};

void powerMonitorBegin();
void powerMonitorUpdate(uint32_t now);
PowerMonitorSample powerMonitorSample(uint32_t now);
