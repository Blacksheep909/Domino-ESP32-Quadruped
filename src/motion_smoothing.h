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

// Right stick X while trotting: how much it steers (instead of sidestepping)
// once the forward stick is pushed. 1 = steer when moving, sidestep when
// standing; 0 = always sidestep (Spot-style, turn with the left stick).
// Saved beside the smoothing settings (optional field, so older Studio and
// firmware ignore it); defaults to 1.
constexpr float kDefaultRightStickSteerBlend = 1.0f;
float rightStickSteerBlend();
bool validRightStickSteerBlend(float value);
bool setRightStickSteerBlend(float value);
