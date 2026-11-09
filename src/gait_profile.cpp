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
  // Used until a profile is applied from Studio. Same as Studio's Balanced
  // preset (gait-lab.js). The old fallback (0.7 Hz, 30 mm lift, 650 ms
  // response, 55% stick) barely moved: its swing feet dragged on the floor.
  strcpy(profile.name, "Firmware default");
  profile.settings.enabled = true;
  strcpy(profile.settings.preset, "balanced");
  profile.settings.cadenceHz = 1.30f;
  profile.settings.strideMm = 88.0f;
  profile.settings.liftMm = 45.0f;
  profile.settings.dutyFactor = 0.62f;
  profile.settings.bodyHeightMm = 245.0f;
  profile.settings.stanceWidthMm = 60.0f;
  profile.settings.turnGain = 0.90f;
  profile.settings.responseMs = 180.0f;
  profile.settings.swingShape = 1.7f;
  profile.settings.diagonalPhase = 0.50f;
  profile.settings.touchdownXMm = -15.75f;
  profile.settings.maxForwardScale = 1.0f;
  profile.settings.maxTurnScale = 0.75f;
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
      // Each diagonal's swing must fit between the other's swing, with a
      // small four-foot overlap. Bounds on either field alone cannot ensure it.
      settings.dutyFactor >= fmaxf(settings.diagonalPhase, 1.0f - settings.diagonalPhase) + 0.02f &&
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
