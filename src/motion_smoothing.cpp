#include "motion_smoothing.h"

#include <Ramp.h>
#include <math.h>

namespace {
MotionSmoothingSettings activeSettings = defaultMotionSmoothingSettings();

bool bounded(float value, float minimum, float maximum) {
  return isfinite(value) && value >= minimum && value <= maximum;
}
}  // namespace

MotionSmoothingSettings defaultMotionSmoothingSettings() {
  return {DOMINO_MOTION_SMOOTHING_SCHEMA_VERSION, 0, 120.0f, 180.0f,
          120.0f, 225.0f, 0.02f, true};
}

bool validateMotionSmoothingSettings(const MotionSmoothingSettings &settings) {
  return settings.schemaVersion == DOMINO_MOTION_SMOOTHING_SCHEMA_VERSION &&
         bounded(settings.heightRateMmPerSec, 40.0f, 300.0f) &&
         bounded(settings.rollRateDegPerSec, 40.0f, 360.0f) &&
         bounded(settings.pitchRateDegPerSec, 30.0f, 240.0f) &&
         bounded(settings.yawRateDegPerSec, 40.0f, 450.0f) &&
         bounded(settings.tiltInputDeadband, 0.0f, 0.08f);
}

const MotionSmoothingSettings &motionSmoothingSettings() { return activeSettings; }

bool setMotionSmoothingSettings(const MotionSmoothingSettings &settings) {
  if (!validateMotionSmoothingSettings(settings)) return false;
  activeSettings = settings;
  return true;
}

float commandWithMotionSmoothing(rampFloat &ramp, float target, float rate, bool enabled) {
  if (!enabled) {
    // Track the direct command so a later re-enable starts from this pose.
    ramp.go(target);
    return target;
  }
  ramp.move(target);
  ramp.setSpeed(rate);
  return ramp.update();
}
