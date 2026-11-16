#include "droid_reactions.h"

#include <math.h>

// Lying over: tilt from upright beyond this for kFallHoldMs.
#ifndef DOMINO_REACT_FALL_TILT_DEG
#define DOMINO_REACT_FALL_TILT_DEG 60.0f
#endif
// Back on its feet: tilt below this for kRecoverHoldMs re-arms the detector.
#ifndef DOMINO_REACT_RECOVER_TILT_DEG
#define DOMINO_REACT_RECOVER_TILT_DEG 25.0f
#endif
// A shove: roll/pitch rate above this while still roughly upright.
#ifndef DOMINO_REACT_PUSH_RATE_DPS
#define DOMINO_REACT_PUSH_RATE_DPS 150.0f
#endif
#ifndef DOMINO_REACT_PUSH_MAX_TILT_DEG
#define DOMINO_REACT_PUSH_MAX_TILT_DEG 35.0f
#endif
// Quiet time between two "pushed" reactions.
#ifndef DOMINO_REACT_PUSH_HOLDOFF_MS
#define DOMINO_REACT_PUSH_HOLDOFF_MS 3000
#endif

namespace {
constexpr uint32_t kFallHoldMs = 300;
constexpr uint32_t kRecoverHoldMs = 1000;
constexpr float kRadToDeg = 57.2957795f;
}  // namespace

DroidReaction droidReactionStep(DroidReactionState &state, const DroidReactionInput &input) {
  if (!input.fresh) {
    state.tilted = false;
    state.upright = false;
    return DroidReaction::None;
  }
  const float magnitude =
      sqrtf(input.upG * input.upG + input.forwardG * input.forwardG + input.leftG * input.leftG);
  // In free fall or a hard knock the gravity direction means nothing.
  if (magnitude < 0.5f || magnitude > 1.6f) return DroidReaction::None;
  float cosTilt = input.upG / magnitude;
  if (cosTilt > 1.0f) cosTilt = 1.0f;
  if (cosTilt < -1.0f) cosTilt = -1.0f;
  const float tiltDeg = acosf(cosTilt) * kRadToDeg;

  if (!state.fallen) {
    if (tiltDeg > DOMINO_REACT_FALL_TILT_DEG) {
      if (!state.tilted) {
        state.tilted = true;
        state.tiltedSinceMs = input.nowMs;
      } else if (input.nowMs - state.tiltedSinceMs >= kFallHoldMs) {
        state.fallen = true;
        state.upright = false;
        return DroidReaction::Fallen;
      }
    } else {
      state.tilted = false;
    }
  } else {
    if (tiltDeg < DOMINO_REACT_RECOVER_TILT_DEG) {
      if (!state.upright) {
        state.upright = true;
        state.uprightSinceMs = input.nowMs;
      } else if (input.nowMs - state.uprightSinceMs >= kRecoverHoldMs) {
        state.fallen = false;
        state.tilted = false;
      }
    } else {
      state.upright = false;
    }
    return DroidReaction::None;
  }

  const float rate = sqrtf(input.rollRateDps * input.rollRateDps + input.pitchRateDps * input.pitchRateDps);
  const bool holdoffOver = !state.everPushed || (input.nowMs - state.lastPushMs >= DOMINO_REACT_PUSH_HOLDOFF_MS);
  if (rate > DOMINO_REACT_PUSH_RATE_DPS && tiltDeg < DOMINO_REACT_PUSH_MAX_TILT_DEG && holdoffOver) {
    state.everPushed = true;
    state.lastPushMs = input.nowMs;
    return DroidReaction::Pushed;
  }
  return DroidReaction::None;
}

#if DOMINO_IMU_ENABLED && !defined(DOMINO_SIL)

#include "droid_voice.h"
#include "personality.h"
#include "imu.h"

static DroidReaction gLastEvent = DroidReaction::None;

DroidReaction droidReactionsTakeEvent() {
  const DroidReaction e = gLastEvent;
  gLastEvent = DroidReaction::None;
  return e;
}

void droidReactionsUpdate(uint32_t nowMs) {
  static DroidReactionState state;
  DroidReactionInput input = {};
  input.nowMs = nowMs;
  input.fresh = gImuState.online && gImuState.has_sample && imuSampleFresh(nowMs);
  // Same sensor-to-body mapping as the balance mode and the policy runner:
  // forward = +ay, left = +az, up = -ax.
  input.forwardG = gImuState.ay_g_filt;
  input.leftG = gImuState.az_g_filt;
  input.upG = -gImuState.ax_g_filt;
  input.rollRateDps = gImuState.gy_dps;
  input.pitchRateDps = gImuState.gz_dps;
  const DroidReaction reaction = droidReactionStep(state, input);
  if (reaction != DroidReaction::None) gLastEvent = reaction;
  if (!personalitySoundAllowed()) return;  // owner turned emotes & sounds off
  switch (reaction) {
    case DroidReaction::Pushed: droidVoicePlay(DROID_EMOTE_PUSHED); break;
    case DroidReaction::Fallen: droidVoicePlay(DROID_EMOTE_FALLEN); break;
    case DroidReaction::None: break;
  }
}

#else

void droidReactionsUpdate(uint32_t) {}
DroidReaction droidReactionsTakeEvent() { return DroidReaction::None; }

#endif
