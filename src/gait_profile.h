#pragma once

#include <stdint.h>

constexpr uint16_t DOMINO_GAIT_SCHEMA_VERSION = 2;

struct GaitProfileSettings {
  bool enabled;
  char preset[12];
  float cadenceHz;
  float strideMm;
  float liftMm;
  float dutyFactor;
  float bodyHeightMm;
  float stanceWidthMm;
  float turnGain;
  float responseMs;
  float swingShape;
  float diagonalPhase;
  float touchdownXMm;
  float maxForwardScale;
  float maxTurnScale;
};

struct GaitProfile {
  uint16_t schemaVersion;
  uint64_t updatedAt;
  char name[33];
  GaitProfileSettings settings;
};

GaitProfile defaultGaitProfile();
bool validateGaitProfile(const GaitProfile &profile);
const GaitProfile &gaitProfile();
bool setGaitProfile(const GaitProfile &profile);
