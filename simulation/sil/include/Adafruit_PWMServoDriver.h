#pragma once

#include <stdint.h>

#include "sim_pwm.h"

class Adafruit_PWMServoDriver {
 public:
  Adafruit_PWMServoDriver() {}

  bool begin() { return true; }
  void setOscillatorFrequency(uint32_t) {}
  void setPWMFreq(float) {}

  void writeMicroseconds(uint8_t channel, uint16_t microseconds) {
    simRecordServoPulse(channel, microseconds);
  }
};
