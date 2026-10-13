#pragma once

#include <stdint.h>

// Calibration authority is renewed only by calibration enter, valid jog, and
// explicit calibration keepalive commands. Unrelated commands must not renew it.
class CalibrationLease {
 public:
  static constexpr uint32_t kTimeoutMs = 2000;

  void refresh(uint32_t now) {
    active_ = true;
    lastRenewalMs_ = now;
  }

  void revoke() { active_ = false; }

  bool expired(uint32_t now) const {
    return active_ && static_cast<uint32_t>(now - lastRenewalMs_) >= kTimeoutMs;
  }

 private:
  bool active_ = false;
  uint32_t lastRenewalMs_ = 0;
};
