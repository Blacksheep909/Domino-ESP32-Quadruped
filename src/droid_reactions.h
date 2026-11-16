#pragma once

// Droid reactions: turns IMU readings into "pushed" and "fallen" events so the
// robot can react with its voice. FIRMWARE STEP "face & voice 2 of 6" (see
// docs/face-voice-firmware-steps.md).
//
// The detector is plain maths on one sample at a time (no Arduino, no I2C), so
// it can be exercised on a PC. Thresholds are first guesses and have not been
// tuned on the robot; every one can be overridden with a build flag.

#include <stdint.h>

enum class DroidReaction : uint8_t { None, Pushed, Fallen };

struct DroidReactionInput {
  uint32_t nowMs;
  bool fresh;          // false while the IMU has no recent sample
  float upG;           // gravity along body up (about +1 when standing level)
  float forwardG;
  float leftG;
  float rollRateDps;   // rate about the body forward axis
  float pitchRateDps;  // rate about the body left axis
};

struct DroidReactionState {
  bool fallen = false;
  uint32_t tiltedSinceMs = 0;
  bool tilted = false;
  uint32_t uprightSinceMs = 0;
  bool upright = false;
  uint32_t lastPushMs = 0;
  bool everPushed = false;
};

// Feed one sample; returns at most one event per call.
DroidReaction droidReactionStep(DroidReactionState &state, const DroidReactionInput &input);

// Reads the IMU and plays the matching emote. Call once per control loop,
// after the IMU sample has been refreshed. Does nothing without an IMU.
void droidReactionsUpdate(uint32_t nowMs);

// The last push / fall event since the previous call (then cleared). The face
// uses it for wide eyes on a shove and squeezed eyes after a fall.
DroidReaction droidReactionsTakeEvent();
