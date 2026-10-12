#include "gait_profile.h"

#include <math.h>
#include <string.h>

namespace {
GaitProfile activeProfile = defaultGaitProfile();

bool bounded(float value, float minimum, float maximum) {
  return isfinite(value) && value >= minimum && value <= maximum;
}

bool terminated(const char *value, size_t capacity) {
  return value && memchr(value, '\0', capacity) != nullptr;
}
}  // namespace

GaitProfile defaultGaitProfile() {
  GaitProfile profile{};
  profile.schemaVersion = DOMINO_GAIT_SCHEMA_VERSION;
  strcpy(profile.name, "Firmware safe");
  profile.settings.enabled = true;
  strcpy(profile.settings.preset, "stable");
  profile.settings.cadenceHz = 0.70f;
  profile.settings.strideMm = 66.0f;
  profile.settings.liftMm = 30.0f;
  profile.settings.dutyFactor = 0.68f;
  profile.settings.bodyHeightMm = 265.0f;
  profile.settings.stanceWidthMm = 38.0f;
  profile.settings.turnGain = 0.67f;
  profile.settings.responseMs = 650.0f;
  profile.settings.swingShape = 2.0f;
  profile.settings.diagonalPhase = 0.50f;
  profile.settings.touchdownXMm = -15.75f;
  profile.settings.maxForwardScale = 0.55f;
  profile.settings.maxTurnScale = 0.45f;
  return profile;
}

bool validateGaitProfile(const GaitProfile &profile) {
  const GaitProfileSettings &settings = profile.settings;
  return profile.schemaVersion == DOMINO_GAIT_SCHEMA_VERSION &&
      terminated(profile.name, sizeof(profile.name)) && profile.name[0] != '\0' &&
      terminated(settings.preset, sizeof(settings.preset)) && settings.preset[0] != '\0' &&
      bounded(settings.cadenceHz, 0.35f, 2.50f) &&
      bounded(settings.strideMm, 24.0f, 120.0f) &&
      bounded(settings.liftMm, 8.0f, 70.0f) &&
      bounded(settings.dutyFactor, 0.50f, 0.82f) &&
      bounded(settings.bodyHeightMm, 220.0f, 280.0f) &&
      bounded(settings.stanceWidthMm, 34.0f, 70.0f) &&
      bounded(settings.turnGain, 0.0f, 1.50f) &&
      bounded(settings.responseMs, 60.0f, 700.0f) &&
      bounded(settings.swingShape, 0.80f, 3.0f) &&
      bounded(settings.diagonalPhase, 0.40f, 0.60f) &&
      bounded(settings.touchdownXMm, -35.0f, 10.0f) &&
      bounded(settings.maxForwardScale, 0.20f, 1.0f) &&
      bounded(settings.maxTurnScale, 0.20f, 1.0f);
}

const GaitProfile &gaitProfile() { return activeProfile; }

bool setGaitProfile(const GaitProfile &profile) {
  if (!validateGaitProfile(profile)) return false;
  activeProfile = profile;
  return true;
}
