#include "sim_pwm.h"

#include <math.h>

namespace {
constexpr uint8_t kChannelCount = 16;
constexpr float kMinPulseUs = 500.0f;
constexpr float kMaxPulseUs = 2500.0f;
constexpr float kMaxAngleDeg = 270.0f;

uint16_t gPulseUs[kChannelCount] = {0};
uint32_t gWriteCount[kChannelCount] = {0};
}

void simRecordServoPulse(uint8_t channel, uint16_t microseconds) {
  if (channel >= kChannelCount) {
    return;
  }
  gPulseUs[channel] = microseconds;
  ++gWriteCount[channel];
}

uint16_t simServoPulseUs(uint8_t channel) {
  return channel < kChannelCount ? gPulseUs[channel] : 0;
}

float simServoAngleDeg(uint8_t channel) {
  const uint16_t pulse = simServoPulseUs(channel);
  if (pulse == 0) {
    return 0.0f;
  }
  const float ratio = (static_cast<float>(pulse) - kMinPulseUs) / (kMaxPulseUs - kMinPulseUs);
  return ratio * kMaxAngleDeg;
}

uint32_t simServoWriteCount(uint8_t channel) {
  return channel < kChannelCount ? gWriteCount[channel] : 0;
}

void simResetServoOutputs() {
  for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
    gPulseUs[channel] = 0;
    gWriteCount[channel] = 0;
  }
}
