#pragma once

#include <stdint.h>

class PowerFaultGuard {
 public:
  PowerFaultGuard(float criticalVoltageV, float recoveryVoltageV, uint32_t holdMs)
      : criticalVoltageV_(criticalVoltageV), recoveryVoltageV_(recoveryVoltageV), holdMs_(holdMs) {}

  bool observe(uint32_t now, bool armed, bool sampleValid, float voltageV);
  bool canAcknowledge(bool sampleValid, float voltageV) const;
  bool acknowledge(bool sampleValid, float voltageV);
  bool latched() const { return latched_; }
  float tripVoltageV() const { return tripVoltageV_; }

 private:
  float criticalVoltageV_;
  float recoveryVoltageV_;
  uint32_t holdMs_;
  bool lowWindowActive_ = false;
  uint32_t lowWindowStartedMs_ = 0;
  bool latched_ = false;
  float tripVoltageV_ = 0.0f;
};
