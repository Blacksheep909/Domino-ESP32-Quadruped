#include "servo_calibration.h"

#include <math.h>

namespace {
constexpr uint8_t kChannels[DOMINO_DRIVEN_JOINT_COUNT] = {
    0, 1, 2, 3, 4, 7, 8, 9, 10, 11, 14, 15,
};
constexpr float kNeutralDeg[DOMINO_SERVO_CHANNEL_COUNT] = {
    129.87f, 171.59f, 113.53f, 105.44f,
    78.30f, 0.0f, 0.0f, 151.60f,
    98.55f, 115.42f, 78.30f, 136.35f,
    0.0f, 0.0f, 119.88f, 128.38f,
};
constexpr int8_t kDefaultDirection[DOMINO_SERVO_CHANNEL_COUNT] = {
    1, -1, -1, 1, 1, 0, 0, -1,
    -1, 1, 1, 1, 0, 0, 1, 1,
};

bool finite(float value) { return isfinite(value); }
bool hipChannel(uint8_t channel) {
  return channel == 0 || channel == 3 || channel == 9 || channel == 14;
}
}  // namespace

ServoCalibrationProfile defaultServoCalibrationProfile() {
  ServoCalibrationProfile profile{};
  profile.schemaVersion = DOMINO_CALIBRATION_SCHEMA_VERSION;
  for (uint8_t index = 0; index < DOMINO_DRIVEN_JOINT_COUNT; ++index) {
    const uint8_t channel = kChannels[index];
    const float travel = hipChannel(channel) ? 30.0f : 45.0f;
    profile.joints[index] = {channel, channel, 0.0f, kDefaultDirection[channel], -travel, travel};
  }
  return profile;
}

const ServoCalibrationJoint* findServoCalibrationJoint(
    const ServoCalibrationProfile &profile, uint8_t logicalChannel) {
  for (const ServoCalibrationJoint &joint : profile.joints) {
    if (joint.logicalChannel == logicalChannel) return &joint;
  }
  return nullptr;
}

uint8_t servoCalibrationPhysicalChannel(const ServoCalibrationProfile &profile,
                                        uint8_t logicalChannel) {
  const ServoCalibrationJoint *joint = findServoCalibrationJoint(profile, logicalChannel);
  return joint ? joint->channel : logicalChannel;
}

bool validateServoCalibrationProfile(const ServoCalibrationProfile &profile) {
  if (profile.schemaVersion != DOMINO_CALIBRATION_SCHEMA_VERSION) return false;
  bool seenLogical[DOMINO_SERVO_CHANNEL_COUNT] = {};
  bool seenPhysical[DOMINO_SERVO_CHANNEL_COUNT] = {};
  for (const ServoCalibrationJoint &joint : profile.joints) {
    if (joint.logicalChannel >= DOMINO_SERVO_CHANNEL_COUNT ||
        servoCalibrationDefaultDirection(joint.logicalChannel) == 0 ||
        joint.channel >= DOMINO_SERVO_CHANNEL_COUNT ||
        seenLogical[joint.logicalChannel] || seenPhysical[joint.channel] ||
        !finite(joint.offsetDeg) || joint.offsetDeg < -30.0f || joint.offsetDeg > 30.0f ||
        (joint.direction != -1 && joint.direction != 1) ||
        !finite(joint.minimumDeg) || !finite(joint.maximumDeg) ||
        joint.minimumDeg < -90.0f || joint.minimumDeg > 89.0f ||
        joint.maximumDeg < joint.minimumDeg + 1.0f || joint.maximumDeg > 90.0f) {
      return false;
    }
    seenLogical[joint.logicalChannel] = true;
    seenPhysical[joint.channel] = true;
  }
  for (uint8_t channel : kChannels) if (!seenLogical[channel]) return false;
  return true;
}

float applyServoCalibration(const ServoCalibrationProfile &profile,
                            uint8_t logicalChannel,
                            float uncalibratedServoDeg) {
  const ServoCalibrationJoint *joint = findServoCalibrationJoint(profile, logicalChannel);
  const int8_t defaultDirection = servoCalibrationDefaultDirection(logicalChannel);
  if (!joint || defaultDirection == 0 || !finite(uncalibratedServoDeg)) return uncalibratedServoDeg;
  const float neutral = servoCalibrationNeutralDeg(logicalChannel);
  float logicalDelta = (uncalibratedServoDeg - neutral) / static_cast<float>(defaultDirection);
  if (hipChannel(logicalChannel)) {
    if (logicalDelta < -30.0f) logicalDelta = -30.0f;
    if (logicalDelta > 30.0f) logicalDelta = 30.0f;
  }
  if (logicalDelta < joint->minimumDeg) logicalDelta = joint->minimumDeg;
  if (logicalDelta > joint->maximumDeg) logicalDelta = joint->maximumDeg;
  return neutral + joint->offsetDeg + static_cast<float>(joint->direction) * logicalDelta;
}

float servoCalibrationNeutralDeg(uint8_t channel) {
  return channel < DOMINO_SERVO_CHANNEL_COUNT ? kNeutralDeg[channel] : 0.0f;
}

int8_t servoCalibrationDefaultDirection(uint8_t channel) {
  return channel < DOMINO_SERVO_CHANNEL_COUNT ? kDefaultDirection[channel] : 0;
}
