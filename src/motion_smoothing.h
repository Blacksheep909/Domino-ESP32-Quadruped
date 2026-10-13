#pragma once

#include <stdint.h>

constexpr uint16_t DOMINO_MOTION_SMOOTHING_SCHEMA_VERSION = 2;

struct MotionSmoothingSettings {
  uint16_t schemaVersion;
  uint64_t updatedAt;
  float heightRateMmPerSec;
  float rollRateDegPerSec;
  float pitchRateDegPerSec;
  float yawRateDegPerSec;
  float tiltInputDeadband;
  bool enabled;
};

class rampFloat;

MotionSmoothingSettings defaultMotionSmoothingSettings();
bool validateMotionSmoothingSettings(const MotionSmoothingSettings &settings);
const MotionSmoothingSettings &motionSmoothingSettings();
bool setMotionSmoothingSettings(const MotionSmoothingSettings &settings);
float commandWithMotionSmoothing(rampFloat &ramp, float target, float rate, bool enabled);
