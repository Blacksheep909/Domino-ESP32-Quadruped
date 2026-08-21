#pragma once

#include <stdint.h>

constexpr uint8_t DOMINO_DRIVEN_JOINT_COUNT = 12;
constexpr uint8_t DOMINO_SERVO_CHANNEL_COUNT = 16;
constexpr uint16_t DOMINO_CALIBRATION_SCHEMA_VERSION = 2;

struct ServoCalibrationJoint {
  uint8_t logicalChannel;
  uint8_t channel;
  float offsetDeg;
  int8_t direction;
  float minimumDeg;
  float maximumDeg;
};

struct ServoCalibrationProfile {
  uint16_t schemaVersion;
  uint64_t savedAt;
  ServoCalibrationJoint joints[DOMINO_DRIVEN_JOINT_COUNT];
};

ServoCalibrationProfile defaultServoCalibrationProfile();
bool validateServoCalibrationProfile(const ServoCalibrationProfile &profile);
float applyServoCalibration(const ServoCalibrationProfile &profile,
                            uint8_t channel,
                            float uncalibratedServoDeg);
const ServoCalibrationJoint* findServoCalibrationJoint(
    const ServoCalibrationProfile &profile, uint8_t logicalChannel);
uint8_t servoCalibrationPhysicalChannel(const ServoCalibrationProfile &profile,
                                        uint8_t logicalChannel);
float servoCalibrationNeutralDeg(uint8_t channel);
int8_t servoCalibrationDefaultDirection(uint8_t channel);
