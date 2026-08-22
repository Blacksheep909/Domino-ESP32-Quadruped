#include "power_fault_guard.h"

#include <math.h>

bool PowerFaultGuard::observe(uint32_t now, bool armed, bool sampleValid, float voltageV) {
  if (latched_) return false;
  if (!armed || !sampleValid || !isfinite(voltageV) || voltageV > criticalVoltageV_) {
    lowWindowActive_ = false;
    lowWindowStartedMs_ = 0;
    return false;
  }
  if (!lowWindowActive_) {
    lowWindowActive_ = true;
    lowWindowStartedMs_ = now;
    return false;
  }
  if (now - lowWindowStartedMs_ < holdMs_) return false;
  latched_ = true;
  tripVoltageV_ = voltageV;
  return true;
}

bool PowerFaultGuard::canAcknowledge(bool sampleValid, float voltageV) const {
  return latched_ && sampleValid && isfinite(voltageV) && voltageV >= recoveryVoltageV_;
}

bool PowerFaultGuard::acknowledge(bool sampleValid, float voltageV) {
  if (!canAcknowledge(sampleValid, voltageV)) return false;
  latched_ = false;
  lowWindowActive_ = false;
  lowWindowStartedMs_ = 0;
  tripVoltageV_ = 0.0f;
  return true;
}
