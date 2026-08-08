#pragma once

#include <stdint.h>

class TwoWire {
 public:
  void begin() {}
  void beginTransmission(uint8_t) {}
  void write(uint8_t) {}
  uint8_t endTransmission(bool = true) { return 0; }
  uint8_t requestFrom(uint8_t, uint8_t length, uint8_t) { return length; }
  int read() { return 0; }
};

extern TwoWire Wire;
