// High‑level control for a 3‑DoF quadruped with CRSF input and a simple
// "body tilt" mode. This file is intentionally verbose with comments to
// capture the mechanical / kinematic assumptions for future maintainers.
//
// Coordinate frames
// -----------------
// World/body frame:
//   - Origin: center of the hip rectangle in X/Y.
//   - +X = forward (toward the head).
//   - +Y = left (dog's left when viewed from behind).
//   - +Z = up.
//
// Leg frame (what IK() expects):
//   - Origin at the hip rotation center.
//   - +X forward, +Y left, +Z downward from the hip toward the ground.
//
// Body plate geometry
// -------------------
// The body is approximated as a rigid plate with 4 hip locations defined
// from CAD. The constants below encode half‑length/width of that plate and
// the nominal stand foot locations.
//
// Hip positions in body frame (x, y, z):
//   - FL: (+BODY_HALF_LENGTH_X, +BODY_HALF_WIDTH_Y, HIP_HEIGHT_Z)
//   - FR: (+BODY_HALF_LENGTH_X, -BODY_HALF_WIDTH_Y, HIP_HEIGHT_Z)
//   - BL: (-BODY_HALF_LENGTH_X, +BODY_HALF_WIDTH_Y, HIP_HEIGHT_Z)
//   - BR: (-BODY_HALF_LENGTH_X, -BODY_HALF_WIDTH_Y, HIP_HEIGHT_Z)
//
// Stand feet use the CAD neutral location relative to each hip, with
// STAND_HEIGHT_Z as the nominal command height.
//
// All physical hip shafts face the same way. Canonical hip coordinates are
// outward-positive on each side; the active calibration maps these coordinates
// to shaft direction. The owner verified roll with FL and BL hips inverted.
// Keep that mapping in calibration rather than changing the saved-profile basis.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ramp.h>
#include <math.h>

#include "crsf.h"
#include "gait_profile.h"
#include "leg_controller.h"
#include "motion_smoothing.h"
#include "i2c_bus.h"
#include "imu.h"
#include "policy_runner.h"
#ifndef DOMINO_SIL
#include "live_robot_endpoint.h"
#include "loop_diagnostics.h"
#endif

#ifndef DOMINO_IMU_ENABLED
#define DOMINO_IMU_ENABLED 1
#endif

enum LegIndex { LEG_FL = 0, LEG_FR = 1, LEG_BL = 2, LEG_BR = 3 };
constexpr int kLegCount = 4;

enum StandState { STOW = 0, SIT = 1, STAND = 2 };
StandState standState = STOW;
bool tiltModeActive = false;
extern float currentTargetZ;
float lastPoseZ = 0.0f;

constexpr float BODY_HALF_LENGTH_X = 167.5f;  // CAD half distance front <-> back hips
constexpr float BODY_HALF_WIDTH_Y = 62.375f;  // CAD half distance left <-> right hips
constexpr float HIP_HEIGHT_Z = 10.5f;         // CAD hip axis above the body-centre plane
constexpr float FOOT_OUT_OFFSET_Y = 100.0f;   // lateral distance from body center line
constexpr float STAND_HEIGHT_Z = -280.0f;     // nominal foot Z in world frame (down)
// The CAD neutral foot is ahead of the hip. The legacy -15.75 mm command X
// maps to this CAD location through leg_controller.cpp.
constexpr float CAD_NEUTRAL_FOOT_X_FROM_HIP = 2.541356f;
constexpr float CAD_NEUTRAL_FOOT_Y_FROM_HIP = 38.1f;
constexpr float CAD_NEUTRAL_FOOT_Z_FROM_HIP = -275.984374f;
constexpr float COMMAND_NEUTRAL_FOOT_X_FROM_HIP = -15.75f;
constexpr float COMMAND_NEUTRAL_FOOT_Y_FROM_HIP = 38.0f;
constexpr float CAD_TO_COMMAND_X =
    COMMAND_NEUTRAL_FOOT_X_FROM_HIP - CAD_NEUTRAL_FOOT_X_FROM_HIP;
constexpr float kStandPoseToleranceMm = 5.0f;

namespace {
#ifndef DOMINO_SIL
LiveManualControlSnapshot activeManualControl{};
#endif
constexpr uint32_t kOscillatorFreqHz = 27000000;
constexpr float kServoPwmFreqHz = 50.0f;

constexpr float kNeutralHeightMm = 280.0f;
constexpr float kNeutralX = 0.0f;
constexpr float kNeutralY = 0.0f;
// IK() assumes +Z is downward from the hip pivot, so reducing Z folds the
// legs and lowers the chassis. The CAD linkage solver verifies the deep stow
// below at full command without changing assembly branch.
constexpr float kNeutralZ = kNeutralHeightMm;
constexpr uint32_t kControlIntervalMs = 20;

constexpr float kCloserPoseZ = 160.0f;

// STOW is a deep sit rather than the old 235 mm shallow crouch. Keeping X at
// the standing footprint holds the feet beneath their hip pivots instead of
// sweeping them out from under the body. All four legs use the same validated
// fold depth. The rendered CAD closure solve keeps the feet beneath the hips
// at 160 mm while all driven joints remain inside +/-45 degrees.
constexpr float kSitFrontX = COMMAND_NEUTRAL_FOOT_X_FROM_HIP;
constexpr float kSitRearX = COMMAND_NEUTRAL_FOOT_X_FROM_HIP;
constexpr float kSitFrontZ = 160.0f;
constexpr float kSitRearZ = 160.0f;
static_assert(kSitFrontZ >= 160.0f && kSitRearZ >= 160.0f,
              "Deep stow must remain inside the CAD linkage clearance envelope");
constexpr float kSitBlendSpeedPerSec = 1.5f;
float sitBlend = 1.0f;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

constexpr int ROLL_CH_INDEX = 0;
constexpr int PITCH_CH_INDEX = 1;
constexpr int YAW_CH_INDEX = 3;
// The Boxer left-stick vertical is CRSF channel 3 and maps continuously to
// the CAD-validated ride-height range.
constexpr int RIDE_HEIGHT_CH_INDEX = 2;
constexpr int SD_CH_INDEX = 7;
constexpr int SC_CAREFUL_MIN_US = 1300;
constexpr int SC_TROT_MIN_US = 1700;
constexpr int SD_ON_THRESHOLD_US = 1600;
constexpr int SD_OFF_THRESHOLD_US = 1400;
constexpr float kStickCenterUs = 1500.0f;
constexpr float kStickHalfRangeUs = 500.0f;
constexpr float kModeInputCenterDeadband = 0.14f;
constexpr uint32_t kModeInputCenterHoldMs = 120;
// These are mandatory mode-change limits, independent of optional stick
// smoothing. Time advances by at most one control frame after a delayed loop.
constexpr float kTransitionFootSpeedMmPerSec = 80.0f;
constexpr float kTransitionAngleSpeedDegPerSec = 25.0f;
constexpr float kTransitionMinimumSeconds = 0.35f;
constexpr uint32_t kMotionCommandTimeoutMs = 250;

constexpr float kMaxRollDeg = 20.0f;
constexpr float kMaxPitchDeg = 10.0f;
constexpr float kMaxYawDeg = 25.0f;
// Balance mode can use a larger tilt range than stick-driven tilt to make
// corrective leg motion more visible.
constexpr float kBalanceMaxRollDeg = 30.0f;
constexpr float kBalanceMaxPitchDeg = 20.0f;
// In balance mode we primarily adjust per-leg Z heights to keep the body
// approximately level. These constants convert virtual roll/pitch (deg) into
// foot Z changes using the actual foot XY geometry and an overall gain.
constexpr float kBalanceMaxZOffsetMm = 80.0f;
constexpr float kBalanceZGain = 2.0f;
// Safety: if the body tilt exceeds this (in any axis), balance mode
// stops trying to correct so we don't fight a fall or extreme pose.
constexpr float kBalanceCancelDeg = 45.0f;
// Balance controller gains (proportional only for now). Values near 1.0 aim
// to roughly cancel measured tilt; tune down if the motion feels too aggressive.
constexpr float kBalanceKpRoll = 1.0f;
constexpr float kBalanceKpPitch = 1.0f;
// Small deadband around zero error so we don't hunt, but keep it tiny so
// balance corrections feel effectively "real-time".
constexpr float kBalanceDeadbandDeg = 0.25f;
// Additional smoothing on the commanded body tilt in balance mode. Set to 1.0
// so new commands are applied immediately (no extra lag).
constexpr float kBalanceOutputAlpha = 1.0f;
constexpr uint32_t kLinkLossStowDelayMs = 1000;
constexpr uint32_t kSwitchDebounceDefaultMs = 200;  // Baseline debounce for SA/SD switches.
constexpr uint32_t kStandCommandHoldMs = 250;        // Additional guard before moving legs when stand toggles.
constexpr uint32_t kTiltToggleDebounceMs = 300;      // Hold tilt candidate before entering/exiting tilt.
constexpr bool kDebugLoggingEnabled = false;
constexpr bool kBalanceDebugLoggingEnabled = false;

// First gait milestone: a deliberately slow diagonal trot. The diagonal pairs
// use opposite phases. Each foot has a planted rearward stance followed by a
// smooth raised return arc, with brief four-foot support overlap. Right-stick
// vertical commands travel; left-stick horizontal commands differential
// stride for turning.
constexpr float kGaitStickDeadband = 0.12f;
constexpr uint32_t kGaitToggleDebounceMs = 200;

// SC middle selects a four-beat crawl. Each leg owns one quarter of the cycle,
// but swings for less than that quarter, leaving at least three commanded
// stance feet and a short four-foot settle. Physical balance still needs a
// supported center of mass and actual ground contact.

// Ride height configuration (stand-only, continuous CH3 command).
// The Boxer left-stick vertical maps linearly across the CAD-validated
// standing range. The stow pose remains a separate fixed safety pose.

// High-level body mode. This is the top-level "menu" state that we will
// extend as more behaviors are added (gaits, diagnostics, etc.).
enum BodyMode : uint8_t {
  BODY_STOW = 0,
  BODY_STAND = 1,
  BODY_TILT = 2,
  BODY_BALANCE = 3,
  BODY_GAIT = 4,
  BODY_CAREFUL = 5,
  BODY_TRANSITION = 6,
  // Learned Isaac Lab policy (SB fully down). Sticks still command walking
  // velocity and CH3 still sets ride height.
  BODY_POLICY = 7,
};

bool isWalkingMode(BodyMode mode) {
  return mode == BODY_GAIT || mode == BODY_CAREFUL;
}

struct LegTarget {
  float x;
  float y;
  float z;
};
LegTarget lastLegTargets[kLegCount] = {};
BodyMode outputMode = BODY_STOW;
float outputHeightMm = kCloserPoseZ;
float outputRollDeg = 0.0f;
float outputPitchDeg = 0.0f;
float outputYawDeg = 0.0f;
float outputBodyX = 0.0f;
float outputBodyY = 0.0f;
bool outputUsesBodyPose = false;
bool motionIkFault = false;

enum TransitionStage : uint8_t { TRANSITION_NONE, TRANSITION_LAND,
                                 TRANSITION_NEUTRAL, TRANSITION_WALK_STANCE };
struct ModeTransition {
  TransitionStage stage = TRANSITION_NONE;
  LegTarget from[kLegCount] = {};
  LegTarget to[kLegCount] = {};
  float fromHeight = 0.0f;
  float toHeight = 0.0f;
  float fromRoll = 0.0f;
  float fromPitch = 0.0f;
  float fromYaw = 0.0f;
  float fromBodyX = 0.0f;
  float fromBodyY = 0.0f;
  float seconds = 0.0f;
  float duration = 0.0f;
  bool bodyPose = false;
} modeTransition;

bool modeTransitionActive() {
  return modeTransition.stage != TRANSITION_NONE;
}

enum MotionInputProfile : uint8_t {
  MOTION_INPUT_NONE = 0,
  MOTION_INPUT_TILT = 1,
  MOTION_INPUT_WALK = 2,
};

// Offsets from kNeutralZ (mm). Positive values move the feet further from
// the body; negative values bring them closer. Keep the minimum inside the
// measured linkage workspace.
constexpr float kRideHeightOffsetHighMm = 0.0f;
constexpr float kRideHeightOffsetLowMm = -60.0f;
constexpr float kRideHeightMaxMm = kNeutralZ + kRideHeightOffsetHighMm;
constexpr float kRideHeightMinMm = kNeutralZ + kRideHeightOffsetLowMm;

static_assert(kRideHeightMinMm >= 220.0f,
              "Lowest ride height must remain inside the CAD linkage workspace");

using MoveLegFn = void (*)(Adafruit_PWMServoDriver &, float, float, float);

constexpr MoveLegFn kMoveLegFns[kLegCount] = {
    moveLegFL,
    moveLegFR,
    moveLegBL,
    moveLegBR,
};

#ifdef DOMINO_SIL
float silLegCommandX[kLegCount] = {};
float silLegCommandY[kLegCount] = {};
float silLegCommandZ[kLegCount] = {};
float silBodyRollDeg = 0.0f;
float silBodyPitchDeg = 0.0f;
float silBodyYawDeg = 0.0f;
#endif

void commandLeg(Adafruit_PWMServoDriver &driver,
                int legIndex,
                float x,
                float y,
                float z) {
  const uint32_t failuresBefore = legIkFailureCount();
#ifndef DOMINO_SIL
  liveRobotEndpointSetExpectedFootTarget(static_cast<uint8_t>(legIndex), x, y, z);
#endif
  kMoveLegFns[legIndex](driver, x, y, z);
  if (legIkFailureCount() != failuresBefore) {
    motionIkFault = true;
    return;
  }
  lastLegTargets[legIndex] = {x, y, z};
#ifdef DOMINO_SIL
  silLegCommandX[legIndex] = x;
  silLegCommandY[legIndex] = y;
  silLegCommandZ[legIndex] = z;
#endif
}

// Nominal foot positions in the body/world frame when the body is level.
// Used by the balance controller to compute per-leg Z corrections that
// keep the body plane level while feet remain at fixed XY.
constexpr float kFootPosXBody[kLegCount] = {
    +BODY_HALF_LENGTH_X + CAD_NEUTRAL_FOOT_X_FROM_HIP,  // FL
    +BODY_HALF_LENGTH_X + CAD_NEUTRAL_FOOT_X_FROM_HIP,  // FR
    -BODY_HALF_LENGTH_X + CAD_NEUTRAL_FOOT_X_FROM_HIP,  // BL
    -BODY_HALF_LENGTH_X + CAD_NEUTRAL_FOOT_X_FROM_HIP   // BR
};

constexpr float kFootPosYBody[kLegCount] = {
    +FOOT_OUT_OFFSET_Y,   // FL
    -FOOT_OUT_OFFSET_Y,   // FR
    +FOOT_OUT_OFFSET_Y,   // BL
    -FOOT_OUT_OFFSET_Y    // BR
};

bool sdCommandActive = false;
float currentRideHeightMm = kRideHeightMaxMm;

// Unified "menu" view of the robot's state based purely on RC inputs and
// link/failsafe information. The control loop consumes this instead of
// talking directly to raw switches.
struct MenuState {
  BodyMode mode;
  float rideHeightMm;
  bool linkAlive;
  bool failsafeActive;
};

// Reference orientation captured when entering balance mode so the robot
// does not "snap" to level on entry. We cancel deviations relative to this.
float balanceRefRollDeg = 0.0f;
float balanceRefPitchDeg = 0.0f;
bool balanceRefValid = false;
// Filtered body tilt commands used in balance mode to further suppress
// oscillations and jerkiness.
float balanceRollCmdFilt = 0.0f;
float balancePitchCmdFilt = 0.0f;
float gaitPhaseRad = 0.0f;
float gaitForwardCommand = 0.0f;
float gaitTurnCommand = 0.0f;
float gaitAmplitude = 0.0f;
float gaitForwardStride = 0.0f;
float gaitTurnStride = 0.0f;
float gaitStrafeCommand = 0.0f;  // trot sidestep, right positive
float gaitStrafeStride = 0.0f;
float gaitBodyZ = 265.0f;
float gaitSupportZ[kLegCount] = {};
rampFloat tiltRollRamp;
rampFloat tiltPitchRamp;
rampFloat tiltYawRamp;
bool motionInputAwaitingCenter = false;
uint32_t motionInputCenteredSinceMs = 0;
// Soft takeover into tilt: instead of waiting for centred sticks, the stick
// command is faded in from zero over kTiltTakeoverMs once the mode change has
// finished, so the body glides to a held stick rather than snapping to it.
constexpr uint32_t kTiltTakeoverMs = 700;
MotionInputProfile activeMotionProfile = MOTION_INPUT_NONE;
uint32_t tiltTakeoverStartedMs = 0;  // 0 = not started yet
float halfCosine01(float value);

float normalizeChannel(float chValueUs) {
  const float normalized = (chValueUs - kStickCenterUs) / kStickHalfRangeUs;
  return constrain(normalized, -1.0f, 1.0f);
}

float applyTiltInputDeadband(float value) {
  const MotionSmoothingSettings &smoothing = motionSmoothingSettings();
  if (!smoothing.enabled) return value;
  const float kTiltInputDeadband = smoothing.tiltInputDeadband;
  const float magnitude = fabsf(value);
  if (magnitude <= kTiltInputDeadband) return 0.0f;
  return copysignf((magnitude - kTiltInputDeadband) /
                       (1.0f - kTiltInputDeadband),
                   value);
}

MotionInputProfile motionInputProfileForMode(BodyMode mode) {
#ifndef DOMINO_SIL
  if (mode == BODY_STAND && activeManualControl.active) return MOTION_INPUT_TILT;
#endif
  if (mode == BODY_TILT) {
    return MOTION_INPUT_TILT;
  }
  if (mode == BODY_GAIT || mode == BODY_CAREFUL || mode == BODY_POLICY) {
    return MOTION_INPUT_WALK;
  }
  return MOTION_INPUT_NONE;
}

bool motionInputsAreCentered() {
#ifndef DOMINO_SIL
  if (activeManualControl.active) {
    return fabsf(activeManualControl.roll) <= kModeInputCenterDeadband &&
           fabsf(activeManualControl.forward) <= kModeInputCenterDeadband &&
           fabsf(activeManualControl.turn) <= kModeInputCenterDeadband &&
           fabsf(activeManualControl.pitch) <= kModeInputCenterDeadband &&
           fabsf(activeManualControl.yaw) <= kModeInputCenterDeadband &&
           fabsf(activeManualControl.bodyX) <= kModeInputCenterDeadband &&
           fabsf(activeManualControl.bodyY) <= kModeInputCenterDeadband;
  }
#endif
  return fabsf(normalizeChannel(ch_us[ROLL_CH_INDEX])) <= kModeInputCenterDeadband &&
         fabsf(normalizeChannel(ch_us[PITCH_CH_INDEX])) <= kModeInputCenterDeadband &&
         fabsf(normalizeChannel(ch_us[YAW_CH_INDEX])) <= kModeInputCenterDeadband;
}

// Which newly entered input profiles wait for centred sticks before acting.
// None do any more. Tilt maps the sticks straight to a body angle, so a held
// stick would snap the body; it now fades in instead (tiltTakeoverGain), which
// is as safe and not finicky. (Firmware <= 0.8.29 made tilt wait until every
// stick was within 14% of centre.) Walking starts from a zeroed gait command
// that ramps toward the stick at the profile's response rate.
bool motionProfileNeedsCentredSticks(MotionInputProfile) {
  return false;
}

// 0..1 multiplier on the motion sticks: 1 except while tilt is taking over.
float tiltTakeoverGain(uint32_t now) {
  if (activeMotionProfile != MOTION_INPUT_TILT) return 1.0f;
  if (tiltTakeoverStartedMs == 0) return 0.0f;
  return halfCosine01(static_cast<float>(now - tiltTakeoverStartedMs) /
                      static_cast<float>(kTiltTakeoverMs));
}

void updateMotionInputInterlock(BodyMode previousMode, BodyMode nextMode, uint32_t now) {
  const MotionInputProfile previousProfile = motionInputProfileForMode(previousMode);
  const MotionInputProfile nextProfile = motionInputProfileForMode(nextMode);
  if (previousProfile != nextProfile) {
    motionInputAwaitingCenter = motionProfileNeedsCentredSticks(nextProfile);
    motionInputCenteredSinceMs = 0;
    tiltTakeoverStartedMs = 0;
  }
  activeMotionProfile = nextProfile;
  // The fade starts once the mode change has finished (sticks read zero until
  // then). With the sticks already centred there is nothing to snap to, so
  // control is immediate; only a held stick is faded in.
  if (nextProfile == MOTION_INPUT_TILT && tiltTakeoverStartedMs == 0 && !modeTransitionActive()) {
    const uint32_t started = motionInputsAreCentered() ? now - kTiltTakeoverMs : now;
    tiltTakeoverStartedMs = started == 0 ? 1 : started;
  }

  if (!motionInputAwaitingCenter) {
    return;
  }
  if (modeTransitionActive()) {
    motionInputCenteredSinceMs = 0;
    return;
  }
  if (!motionInputsAreCentered()) {
    motionInputCenteredSinceMs = 0;
    return;
  }
  if (motionInputCenteredSinceMs == 0) {
    motionInputCenteredSinceMs = now;
    return;
  }
  if ((now - motionInputCenteredSinceMs) >= kModeInputCenterHoldMs) {
    motionInputAwaitingCenter = false;
  }
}

float readMotionChannelNormalized(int channelIndex) {
#ifndef DOMINO_SIL
  if (activeManualControl.active) {
    if (motionInputAwaitingCenter) return 0.0f;
    const float gain = tiltTakeoverGain(millis());
    if (channelIndex == PITCH_CH_INDEX) return activeManualControl.forward * gain;
    if (channelIndex == YAW_CH_INDEX) return activeManualControl.turn * gain;
    if (channelIndex == ROLL_CH_INDEX) return activeManualControl.roll * gain;
  }
#endif
  // Switches still use the more heavily filtered integer channels. Motion
  // axes use the first CRSF filter stage; the tilt ramp limits their speed.
  const bool commandsFresh = crsfHasReceivedFrame() &&
      millis() - lastCrsfMs < kMotionCommandTimeoutMs;
  return motionInputAwaitingCenter || modeTransitionActive() || !commandsFresh
      ? 0.0f : normalizeChannel(ch_us_filt[channelIndex]) * tiltTakeoverGain(millis());
}

float manualRollFootZOffset(float lateralMm) {
#ifndef DOMINO_SIL
  if (activeManualControl.active && activeManualControl.deadman) {
    return sinf(activeManualControl.roll * kMaxRollDeg * kDegToRad) * lateralMm;
  }
#else
  (void)lateralMm;
#endif
  return 0.0f;
}

constexpr float kManualMaxPitchDeg = 8.0f;
constexpr float kManualMaxYawDeg = 8.0f;
constexpr float kManualMaxBodyXMm = 15.0f;
constexpr float kManualMaxBodyYMm = 12.0f;

void logDebugState(uint32_t now,
                   bool linkAlive,
                   bool standCommand,
                   bool standPoseReady,
                   bool sdDown,
                   bool tiltCandidateRaw,
                   bool tiltCandidateFiltered,
                   bool tiltActive) {
  if (!kDebugLoggingEnabled) {
    return;
  }

  static uint32_t lastLogMs = 0;
  static bool firstLog = true;
  constexpr uint32_t kLogIntervalMs = 100;
  if (!firstLog && (now - lastLogMs) < kLogIntervalMs) {
    return;
  }
  firstLog = false;
  lastLogMs = now;

  const int saUs = ch_us[SA_CH_INDEX];
  const int sdUs = ch_us[SD_CH_INDEX];
  const int rollUs = ch_us[ROLL_CH_INDEX];
  const int pitchUs = ch_us[PITCH_CH_INDEX];
  const int yawUs = ch_us[YAW_CH_INDEX];

  const float rollNorm = normalizeChannel(rollUs);
  const float pitchNorm = normalizeChannel(pitchUs);
  const float yawNorm = normalizeChannel(yawUs);

  const float rollDeg = rollNorm * kMaxRollDeg;
  const float pitchDeg = pitchNorm * kMaxPitchDeg;
  const float yawDeg = -yawNorm * kMaxYawDeg;

  Serial.printf(
      "DBG t=%lu link=%d lastCrsf=%lu standCmd=%d poseReady=%d standState=%d sdDown=%d tiltCandRaw=%d "
      "tiltCandFilt=%d tiltActive=%d targetZ=%.1f | SA=%d SD=%d roll=%d pitch=%d yaw=%d | rollNorm=%.2f "
      "pitchNorm=%.2f yawNorm=%.2f | rollDeg=%.1f pitchDeg=%.1f yawDeg=%.1f | poseZ=%.1f | "
      "imuOk=%d axG=%.3f ayG=%.3f azG=%.3f gxDps=%.1f gyDps=%.1f gzDps=%.1f\n",
      static_cast<unsigned long>(now),
      linkAlive ? 1 : 0,
      static_cast<unsigned long>(lastCrsfMs),
      standCommand ? 1 : 0,
      standPoseReady ? 1 : 0,
      static_cast<int>(standState),
      sdDown ? 1 : 0,
      tiltCandidateRaw ? 1 : 0,
      tiltCandidateFiltered ? 1 : 0,
      tiltActive ? 1 : 0,
      currentTargetZ,
      saUs,
      sdUs,
      rollUs,
      pitchUs,
      yawUs,
      rollNorm,
      pitchNorm,
      yawNorm,
      rollDeg,
      pitchDeg,
      yawDeg,
      lastPoseZ,
      gImuState.online ? 1 : 0,
      gImuState.ax_g_filt,
      gImuState.ay_g_filt,
      gImuState.az_g_filt,
      gImuState.gx_dps_filt,
      gImuState.gy_dps_filt,
      gImuState.gz_dps_filt);
}

void moveAllLegs(Adafruit_PWMServoDriver &driver, float x, float y, float z) {
  for (int i = 0; i < kLegCount; ++i) {
    commandLeg(driver, i, x, y, z);
  }
}

struct SwitchDebounceState {
  bool pending = false;
  uint32_t pendingSince = 0;
  bool initialized = false;
};

bool applySwitchDebounce(bool currentState,
                         bool desiredState,
                         uint32_t now,
                         SwitchDebounceState *state,
                         uint32_t debounceMs) {
  if (!state->initialized) {
    state->pending = currentState;
    state->pendingSince = now;
    state->initialized = true;
  }

  if (desiredState == currentState) {
    state->pending = desiredState;
    state->pendingSince = now;
    return currentState;
  }

  if (desiredState != state->pending) {
    state->pending = desiredState;
    state->pendingSince = now;
    return currentState;
  }

  if ((now - state->pendingSince) >= debounceMs) {
    return desiredState;
  }
  
  return currentState;
}

// Forward declarations for switch handling helpers used in the menu layer.
bool updateStandCommand(bool standActive, uint32_t now);
bool updateSdCommand(bool sdActive, bool controlsEnabled, uint32_t now);

float rideHeightMmFromChannelUs(int heightValueUs) {
  const float clampedUs = fminf(2000.0f, fmaxf(1000.0f, static_cast<float>(heightValueUs)));
  const float normalized = (clampedUs - 1000.0f) / 1000.0f;
  return kRideHeightMinMm + normalized * (kRideHeightMaxMm - kRideHeightMinMm);
}

float readRideHeightMm() {
  return rideHeightMmFromChannelUs(ch_us[RIDE_HEIGHT_CH_INDEX]);
}

float computeStandTargetZ(float rideHeightMm) {
  return fminf(kRideHeightMaxMm, fmaxf(kRideHeightMinMm, rideHeightMm));
}

// Raw RC -> menu inputs -------------------------------------------------

struct MenuInputs {
  bool haveLink = false;
  bool motionCommandsFresh = false;
  bool failsafeActive = false;
  bool standRequested = false;
  bool tiltSwitchDown = false;
  bool balanceRequested = false;
  bool gaitRequested = false;
  bool carefulWalkRequested = false;
  bool policyRequested = false;
  float rideHeightMm = kRideHeightMaxMm;
};

MenuInputs readMenuInputs(uint32_t now, uint32_t* lastLinkAliveMs, bool* failsafeState) {
  MenuInputs inputs{};

  const bool linkAliveNow = crsfLinkAlive(now);
  const bool hasReceivedFrame = crsfHasReceivedFrame();
  if (linkAliveNow) {
    *lastLinkAliveMs = now;
  }
  inputs.haveLink = linkAliveNow;

  const bool nextFailsafeState =
      !hasReceivedFrame || (!linkAliveNow && ((now - *lastLinkAliveMs) > kLinkLossStowDelayMs));
  if (nextFailsafeState != *failsafeState) {
    *failsafeState = nextFailsafeState;
#ifndef DOMINO_SIL
    loopBlackboxRecord(BlackboxEventKind::Failsafe, 0,
                       *failsafeState ? 1U : 0U,
                       static_cast<uint16_t>(crsfHasReceivedFrame()
                           ? (now - lastCrsfMs < UINT16_MAX ? now - lastCrsfMs : UINT16_MAX)
                           : UINT16_MAX));
#endif
    Serial.printf("CRSF failsafe %s (last=%lu now=%lu)\n",
                  *failsafeState ? "ACTIVE" : "CLEARED",
                  static_cast<unsigned long>(*lastLinkAliveMs),
                  static_cast<unsigned long>(now));
  }
  inputs.failsafeActive = *failsafeState;

  // Stand request from SA (with existing debouncing logic).
  static bool saStandCommand = false;
  const bool nextSaStand = updateStandCommand(saStandCommand, now);
  if (nextSaStand != saStandCommand) {
    saStandCommand = nextSaStand;
  }
  inputs.standRequested = saStandCommand && !inputs.failsafeActive;

  // Tilt enable from SD. Keep it latched through short CRSF gaps; only force it
  // low when the longer failsafe state is actually active.
  const bool nextSdCommand = updateSdCommand(sdCommandActive, !inputs.failsafeActive, now);
  if (nextSdCommand != sdCommandActive) {
    sdCommandActive = nextSdCommand;
    Serial.printf("SD=%d -> sdDown=%d\n", ch_us[SD_CH_INDEX], sdCommandActive ? 1 : 0);
  }
  inputs.tiltSwitchDown = sdCommandActive;

  // SC is a three-position walk selector: low=stand, middle=careful crawl,
  // high=diagonal trot. The mode-level debounce below prevents switch travel
  // through the midpoint from briefly starting the careful gait.
  const int scValueUs = ch_us[SC_CH_INDEX];
  inputs.balanceRequested = false;
  inputs.carefulWalkRequested =
      scValueUs >= SC_CAREFUL_MIN_US && scValueUs < SC_TROT_MIN_US && !inputs.failsafeActive;
  inputs.gaitRequested = scValueUs >= SC_TROT_MIN_US && !inputs.failsafeActive;
  static int lastWalkSelection = -1;
  const int walkSelection = inputs.gaitRequested ? 2 : inputs.carefulWalkRequested ? 1 : 0;
  if (walkSelection != lastWalkSelection) {
    lastWalkSelection = walkSelection;
    Serial.printf("SC=%d -> walkMode=%d\n", scValueUs, walkSelection);
  }

  // SB fully down: learned policy ("simulation mode"), with hysteresis.
  static bool sbPolicyDown = false;
  const int sbValueUs = ch_us[SB_CH_INDEX];
  const bool nextSbPolicyDown = sbPolicyDown ? sbValueUs >= SB_POLICY_OFF_US
                                             : sbValueUs >= SB_POLICY_ON_US;
  if (nextSbPolicyDown != sbPolicyDown) {
    sbPolicyDown = nextSbPolicyDown;
    Serial.printf("SB=%d -> policyMode=%d\n", sbValueUs, sbPolicyDown ? 1 : 0);
  }
  inputs.policyRequested = sbPolicyDown && !inputs.failsafeActive;

  // Continuous ride height from Boxer left-stick vertical / CRSF CH3.
  inputs.rideHeightMm = readRideHeightMm();

  // Stop motion on a short command gap. The existing longer timeout still
  // owns stow; a recovered link must pass the normal centering interlock.
  inputs.motionCommandsFresh = hasReceivedFrame && now - lastCrsfMs < kMotionCommandTimeoutMs;
  if (!inputs.motionCommandsFresh) {
    inputs.tiltSwitchDown = false;
    inputs.carefulWalkRequested = false;
    inputs.gaitRequested = false;
    inputs.policyRequested = false;
  }

  return inputs;
}

void bodyKinematicsSimple(LegIndex leg,
                          float bodyX,
                          float bodyY,
                          float bodyZ,
                          float rollDeg,
                          float pitchDeg,
                          float yawDeg,
                          float *outX,
                          float *outY,
                          float *outZ) {
  // This helper implements a simplified "body tilt" kinematic model:
  //   - The body is approximated as a rigid plate with 4 hips at the CAD
  //     locations described at the top of this file.
  //   - rollDeg / pitchDeg / yawDeg describe the body orientation about
  //     the body center in the world frame.
  //   - bodyX/Y/Z translate the body center in world space.
  //   - The feet are assumed to be at a nominal stand pose given by
  //     FOOT_* and STAND_HEIGHT_Z; we do not currently capture the true
  //     footWorldPos[] at tilt‑mode entry.
  //
  // The output (outX/outY/outZ) is the vector from hip to foot expressed
  // in the leg frame that IK() expects (X forward, Y left, Z down).

  const float hx = (leg == LEG_FL || leg == LEG_FR) ? +BODY_HALF_LENGTH_X : -BODY_HALF_LENGTH_X;
  const float hy = (leg == LEG_FL || leg == LEG_BL) ? +BODY_HALF_WIDTH_Y : -BODY_HALF_WIDTH_Y;
  const float hz = HIP_HEIGHT_Z;

  const float r = rollDeg * kDegToRad;
  const float p = pitchDeg * kDegToRad;
  const float y = yawDeg * kDegToRad;

  const float cr = cosf(r);
  const float sr = sinf(r);
  const float cp = cosf(p);
  const float sp = sinf(p);
  const float cy = cosf(y);
  const float sy = sinf(y);

  const float R00 = cy * cp;
  const float R01 = cy * sp * sr - sy * cr;
  const float R02 = cy * sp * cr + sy * sr;
  const float R10 = sy * cp;
  const float R11 = sy * sp * sr + cy * cr;
  const float R12 = sy * sp * cr - cy * sr;
  const float R20 = -sp;
  const float R21 = cp * sr;
  const float R22 = cp * cr;

  const float hxR = R00 * hx + R01 * hy + R02 * hz;
  const float hyR = R10 * hx + R11 * hy + R12 * hz;
  const float hzR = R20 * hx + R21 * hy + R22 * hz;

  const float hipWorldX = hxR + bodyX;
  const float hipWorldY = hyR + bodyY;
  const float hipWorldZ = hzR + bodyZ;

  const bool leftLeg = (leg == LEG_FL || leg == LEG_BL);
  const float side = leftLeg ? 1.0f : -1.0f;
  const float footBaseX =
      (leg == LEG_FL || leg == LEG_FR) ? +BODY_HALF_LENGTH_X : -BODY_HALF_LENGTH_X;
  const float footWorldX = footBaseX + CAD_NEUTRAL_FOOT_X_FROM_HIP;
  const float footWorldY =
      side * (BODY_HALF_WIDTH_Y + CAD_NEUTRAL_FOOT_Y_FROM_HIP);
  const float footWorldZ = HIP_HEIGHT_Z + CAD_NEUTRAL_FOOT_Z_FROM_HIP;

  const float legX_world = footWorldX - hipWorldX;
  const float legY_world = footWorldY - hipWorldY;
  const float legZ_worldUp = footWorldZ - hipWorldZ;

  // IK is solved in the local leg/body frame, not the world frame. The hip
  // plane was rotated into world space above, so rotate the hip-to-foot vector
  // back through R^T before converting world +Z-up into IK +Z-down.
  const float legX_local = R00 * legX_world + R10 * legY_world + R20 * legZ_worldUp;
  const float legY_local = R01 * legX_world + R11 * legY_world + R21 * legZ_worldUp;
  const float legZ_localUp = R02 * legX_world + R12 * legY_world + R22 * legZ_worldUp;

  *outX = legX_local + CAD_TO_COMMAND_X;
  *outY =
      legY_local +
      side * (COMMAND_NEUTRAL_FOOT_Y_FROM_HIP -
              CAD_NEUTRAL_FOOT_Y_FROM_HIP);
  *outZ =
      -legZ_localUp +
      ((-STAND_HEIGHT_Z) + CAD_NEUTRAL_FOOT_Z_FROM_HIP);
}

// Move all four legs given a simple body pose expressed in the world/body frame.
// This wraps bodyKinematicsSimple() and feeds the resulting leg-frame targets
// into the per-leg move functions (which in turn call IK()).
void moveLegsFromBodyPose(Adafruit_PWMServoDriver &driver,
                          float bodyX,
                          float bodyY,
                          float bodyZ,
                          float rollDeg,
                          float pitchDeg,
                          float yawDeg) {
  outputHeightMm = bodyZ - STAND_HEIGHT_Z;
  outputRollDeg = rollDeg;
  outputPitchDeg = pitchDeg;
  outputYawDeg = yawDeg;
  outputBodyX = bodyX;
  outputBodyY = bodyY;
  outputUsesBodyPose = true;
#ifndef DOMINO_SIL
  LiveRobotPoseSnapshot livePose;
  livePose.rollDeg = rollDeg;
  livePose.pitchDeg = pitchDeg;
  livePose.yawDeg = yawDeg;
  livePose.heightMm = bodyZ - STAND_HEIGHT_Z;
  liveRobotEndpointSetExpectedPose(livePose);
#endif
  for (int i = 0; i < kLegCount; ++i) {
    float xLeg = 0.0f;
    float yLeg = 0.0f;
    float zLeg = 0.0f;
    bodyKinematicsSimple(static_cast<LegIndex>(i),
                         bodyX,
                         bodyY,
                         bodyZ,
                         rollDeg,
                         pitchDeg,
                         yawDeg,
                         &xLeg,
                         &yLeg,
                         &zLeg);
    // The lateral/fore-aft components above are intentional. They are the
    // body-frame coordinates required to keep each foot fixed in world space
    // while the body rotates around its centre. Replacing them with only a
    // left/right or front/rear Z difference makes the physical chassis swing
    // around an effective pivot near the support plane instead.
    commandLeg(driver, i, xLeg, yLeg, zLeg);
  }
}

void moveLegsToSitBlend(Adafruit_PWMServoDriver &driver, float commonZ, float blend) {
  outputHeightMm = commonZ + blend * (kSitFrontZ - commonZ);
  outputRollDeg = outputPitchDeg = outputYawDeg = 0.0f;
  outputBodyX = outputBodyY = 0.0f;
  outputUsesBodyPose = false;
  const float t = constrain(blend, 0.0f, 1.0f);
  for (int i = 0; i < kLegCount; ++i) {
    const bool frontLeg = (i == LEG_FL || i == LEG_FR);
    const bool leftLeg = (i == LEG_FL || i == LEG_BL);
    const float sitX = frontLeg ? kSitFrontX : kSitRearX;
    const float sitZ = frontLeg ? kSitFrontZ : kSitRearZ;
    const float x = COMMAND_NEUTRAL_FOOT_X_FROM_HIP +
        t * (sitX - COMMAND_NEUTRAL_FOOT_X_FROM_HIP);
    const float y = leftLeg
        ? (FOOT_OUT_OFFSET_Y - BODY_HALF_WIDTH_Y)
        : -(FOOT_OUT_OFFSET_Y - BODY_HALF_WIDTH_Y);
    const float z = commonZ + t * (sitZ - commonZ);
    commandLeg(driver, i, x, y, z);
  }
}

// Variant used in balance mode: keep the commanded body orientation level
// (roll=pitch=yaw=0 in the body model) and instead adjust each leg's Z
// height individually in world space to counter measured tilt.
void moveLegsFromBodyPoseWithZOffsets(Adafruit_PWMServoDriver &driver,
                                      float bodyX,
                                      float bodyY,
                                      float bodyZ,
                                      const float zOffsetsWorld[kLegCount]) {
  outputHeightMm = bodyZ - STAND_HEIGHT_Z;
  outputRollDeg = outputPitchDeg = outputYawDeg = 0.0f;
  outputBodyX = outputBodyY = 0.0f;
  outputUsesBodyPose = false;
  for (int i = 0; i < kLegCount; ++i) {
    float xLeg = 0.0f;
    float yLeg = 0.0f;
    float zLeg = 0.0f;
    bodyKinematicsSimple(static_cast<LegIndex>(i),
                         bodyX,
                         bodyY,
                         bodyZ,
                         0.0f,
                         0.0f,
                         0.0f,
                         &xLeg,
                         &yLeg,
                         &zLeg);
    const float zAdj = zLeg + zOffsetsWorld[i];
    commandLeg(driver, i, xLeg, yLeg, zAdj);
  }
}

float applyGaitDeadband(float value) {
  const float magnitude = fabsf(value);
  if (magnitude <= kGaitStickDeadband) {
    return 0.0f;
  }
  const float scaled = (magnitude - kGaitStickDeadband) / (1.0f - kGaitStickDeadband);
  return copysignf(constrain(scaled, 0.0f, 1.0f), value);
}

// Walking commands from the sticks (Boxer, mode 2):
//  - right stick Y (CH2) drives forward/back;
//  - left stick X (CH4) spins the body on the spot about Z;
//  - right stick X (CH1), trot: sidesteps when there is no forward stick and
//    hands over to steering as forward stick grows (splitRightStickX); the
//    careful crawl only steers with it, scaled by speed.
// Positive turn is clockwise (to the right) and positive strafe is to the
// right, matching the operator's stick.
float readGaitForwardInput(const GaitProfileSettings &profile) {
  return profile.enabled
      ? applyGaitDeadband(readMotionChannelNormalized(PITCH_CH_INDEX)) * profile.maxForwardScale : 0.0f;
}

float readGaitTurnInput(const GaitProfileSettings &profile, float forward) {
  if (!profile.enabled) return 0.0f;
  const float spin = applyGaitDeadband(readMotionChannelNormalized(YAW_CH_INDEX));
  float steer = 0.0f;
#ifndef DOMINO_SIL
  // LIVE manual control uses its roll axis for body roll while walking.
  if (!activeManualControl.active)
#endif
  {
    const float speed = profile.maxForwardScale > 0.0f
        ? constrain(fabsf(forward) / profile.maxForwardScale, 0.0f, 1.0f) : 0.0f;
    steer = applyGaitDeadband(readMotionChannelNormalized(ROLL_CH_INDEX)) * speed;
  }
  return constrain(spin + steer, -1.0f, 1.0f) * profile.maxTurnScale;
}

// A full sidestep travels this share of the forward stride: the legs reach
// less far sideways than fore/aft (keep in step with GAIT_LATERAL_STRIDE_SCALE
// in simulation/standalone/web/src/gait-lab.js).
constexpr float kGaitLateralStrideScale = 0.6f;
// Below this much forward stick (as a share of full), right-stick X is mostly
// a sidestep.
constexpr float kGaitSteerFullAtForward = 0.4f;
// Extra rearward foot landing at a full sidestep, blended by the sidestep share
// of the motion (keep in step with GAIT_SIDESTEP_TOUCHDOWN_SHIFT_MM).
constexpr float kGaitSidestepTouchdownShiftMm = -12.0f;

// Gyro heading hold (keep in step with createHeadingHold in gait-lab.js).
// Uneven front/rear foot grip makes the body slowly rotate while sidestepping,
// so a sidestep right then left traced a V that drifted backwards. While the
// operator gives no turn command and the robot is sidestepping, this holds the
// heading captured when the turn stick was released, using the IMU's yaw
// (counter-clockwise positive). It is scaled by the sidestep share of the
// motion: a forward trot wags its nose every step and barely drifts, and
// correcting that only pushed the path sideways. Output is a turn command
// (clockwise positive, -1..1, before maxTurnScale), capped so it can never
// out-steer the operator. Without a fresh IMU sample it does nothing.
// If it ever turns the robot away from its heading instead of back, the IMU
// yaw sign is reversed.
struct HeadingHold {
  static constexpr float kKpPerDeg = 0.02f;
  static constexpr float kMaxCorrection = 0.2f;
  static constexpr float kReleaseTurn = 0.04f;
  static constexpr float kRecaptureSeconds = 0.25f;
  bool held = false;
  float heldYawDeg = 0.0f;
  float settleSeconds = 0.0f;

  void reset() {
    held = false;
    settleSeconds = 0.0f;
  }

  float update(bool measurementFresh, float yawDeg, float operatorTurn, bool sidestepping,
               float sidestepShare, float deltaSeconds) {
    if (!measurementFresh || !isfinite(yawDeg)) {
      held = false;
      return 0.0f;
    }
    if (fabsf(operatorTurn) > kReleaseTurn || !sidestepping) {
      held = false;
      settleSeconds = kRecaptureSeconds;
      return 0.0f;
    }
    if (settleSeconds > 0.0f) {
      settleSeconds = fmaxf(0.0f, settleSeconds - deltaSeconds);
      return 0.0f;
    }
    if (!held) {
      held = true;
      heldYawDeg = yawDeg;
    }
    float error = fmodf(yawDeg - heldYawDeg, 360.0f);
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    // Turned counter-clockwise (positive error) -> turn clockwise (positive).
    const float correction = constrain(kKpPerDeg * error, -kMaxCorrection, kMaxCorrection);
    return correction * constrain(sidestepShare, 0.0f, 1.0f);
  }
};
HeadingHold gaitHeadingHold;

// Right stick X while trotting (keep in step with splitRightStickX in
// gait-lab.js). `speed` is the forward stick, 0..1. With no forward stick it
// sidesteps; as forward stick grows it hands over to steering by `steerBlend`
// (1 = full hand-over, 0 = always sidestep, Spot-style).
void splitRightStickX(float speed, float rightX, float steerBlend, float *steer, float *strafe) {
  const float share = constrain(steerBlend, 0.0f, 1.0f) *
      halfCosine01(constrain(speed / kGaitSteerFullAtForward, 0.0f, 1.0f));
  *steer = rightX * speed * share;
  *strafe = rightX * (1.0f - share);
}

// Trot commands: turn (spin + right-stick steering) and strafe, both scaled
// by the profile. `forward` is readGaitForwardInput()'s result.
// Direction intent on the right stick (keep in step with rightStickIntent in
// gait-lab.js). The angle runs from straight sideways (0 deg) to straight
// forward/back (90 deg). Within ~15 deg of sideways the forward part is
// dropped (fading back in by 30 deg), so a push that is "mostly left" is a
// clean sidestep. Within ~10 deg of straight forward the sidestep part is
// dropped (70-80 deg fade), so walking forward does not crab. Steering is
// unaffected: a small sideways nudge while walking still steers.
constexpr float kStickSidewaysIntentLowDeg = 15.0f;
constexpr float kStickSidewaysIntentHighDeg = 30.0f;
constexpr float kStickForwardIntentLowDeg = 70.0f;
constexpr float kStickForwardIntentHighDeg = 80.0f;

void rightStickIntent(float *forward, float rightX, float *strafeWeight) {
  *strafeWeight = 1.0f;
  if (*forward == 0.0f || rightX == 0.0f) return;
  const float angleDeg = atan2f(fabsf(*forward), fabsf(rightX)) * (180.0f / kPi);
  *forward *= halfCosine01((angleDeg - kStickSidewaysIntentLowDeg) /
                           (kStickSidewaysIntentHighDeg - kStickSidewaysIntentLowDeg));
  *strafeWeight = 1.0f - halfCosine01((angleDeg - kStickForwardIntentLowDeg) /
                                      (kStickForwardIntentHighDeg - kStickForwardIntentLowDeg));
}

// Trot commands from the sticks: forward (in/out: readGaitForwardInput()'s
// result, cleared when the right stick is pushed mostly sideways), turn (spin
// + right-stick steering) and strafe, all scaled by the profile.
void readGaitTrotInputs(const GaitProfileSettings &profile, float *forward, float *turn, float *strafe) {
  *turn = 0.0f;
  *strafe = 0.0f;
  if (!profile.enabled) return;
  const float spin = applyGaitDeadband(readMotionChannelNormalized(YAW_CH_INDEX));
  float rightX = 0.0f;
#ifndef DOMINO_SIL
  // LIVE manual control uses its roll axis for body roll while walking.
  if (!activeManualControl.active)
#endif
  {
    rightX = applyGaitDeadband(readMotionChannelNormalized(ROLL_CH_INDEX));
  }
  float forwardStick = profile.maxForwardScale > 0.0f
      ? constrain(*forward / profile.maxForwardScale, -1.0f, 1.0f) : 0.0f;
  float strafeWeight = 1.0f;
  rightStickIntent(&forwardStick, rightX, &strafeWeight);
  *forward = forwardStick * profile.maxForwardScale;
  float steer = 0.0f;
  float strafeShare = 0.0f;
  splitRightStickX(fabsf(forwardStick), rightX, rightStickSteerBlend(), &steer, &strafeShare);
  *turn = constrain(spin + steer, -1.0f, 1.0f) * profile.maxTurnScale;
  *strafe = constrain(strafeShare * strafeWeight, -1.0f, 1.0f) * profile.maxForwardScale;
}

// Per-leg stride vector (full stance travel, mm) in the leg command frame.
// Forward motion moves every foot straight back; turning moves each planted
// foot along the circle around the body centre (tangent to its radius), so a
// pure turn rotates the robot on the spot instead of skidding the feet
// sideways like a tank. The tangent is normalised so that a full turn command
// gives the same foot travel as a full forward command at the same stride.
// A combined command that would exceed the stride is scaled down evenly, which
// keeps the path's curvature.
struct GaitStride { float x; float y; };

void computeGaitStrides(const GaitProfileSettings &profile, float forward, float turn,
                        float strafe, GaitStride strides[kLegCount]) {
  float px[kLegCount];
  float py[kLegCount];
  float radiusSum = 0.0f;
  for (int i = 0; i < kLegCount; ++i) {
    const bool front = i == LEG_FL || i == LEG_FR;
    const bool left = i == LEG_FL || i == LEG_BL;
    px[i] = (front ? BODY_HALF_LENGTH_X : -BODY_HALF_LENGTH_X) + profile.touchdownXMm;
    py[i] = (left ? 1.0f : -1.0f) * (BODY_HALF_WIDTH_Y + profile.stanceWidthMm);
    radiusSum += sqrtf(px[i] * px[i] + py[i] * py[i]);
  }
  // One shared turn rate for every foot: each foot's sweep is proportional to
  // its own distance from the body centre. (0.8.26 swept every foot the same
  // distance, so the front feet, which sit closer to the centre, tried to turn
  // the body faster than the rear feet and the difference was scuffed off.)
  const float referenceRadius = fmaxf(1.0f, radiusSum / kLegCount);
  float largest = 0.0f;
  for (int i = 0; i < kLegCount; ++i) {
    // Clockwise (positive) turn: the body point at (px, py) moves along (py, -px).
    strides[i].x = profile.strideMm * (forward + turn * profile.turnGain * py[i] / referenceRadius);
    // Strafe right (positive) moves every planted foot left relative to the body.
    strides[i].y = profile.strideMm * (turn * profile.turnGain * -px[i] / referenceRadius -
                                       strafe * kGaitLateralStrideScale);
    largest = fmaxf(largest, sqrtf(strides[i].x * strides[i].x + strides[i].y * strides[i].y));
  }
  const float limit = profile.strideMm * 1.05f;
  if (largest > limit) {
    const float scale = limit / largest;
    for (int i = 0; i < kLegCount; ++i) {
      strides[i].x *= scale;
      strides[i].y *= scale;
    }
  }
}

float approachGaitCommand(float current, float target, float maximumStep) {
  if (target > current) {
    return fminf(target, current + maximumStep);
  }
  return fmaxf(target, current - maximumStep);
}

void resetGaitState() {
  gaitHeadingHold.reset();
  gaitPhaseRad = 0.0f;
  gaitForwardCommand = 0.0f;
  gaitTurnCommand = 0.0f;
  gaitStrafeCommand = 0.0f;
  gaitAmplitude = 0.0f;
  gaitForwardStride = gaitTurnStride = gaitStrafeStride = 0.0f;
  gaitBodyZ = lastPoseZ;
}

void updateGaitStride(float activity, float maximumStep) {
  // Normalizing a command near zero can flip its direction while the lift
  // amplitude is still fading. Slew the effective stride vector itself, so
  // release/reverse cannot collapse or reverse the whole foot stroke in a frame.
  const float scale = activity > 0.001f ? gaitAmplitude / activity : 0.0f;
  gaitForwardStride = approachGaitCommand(
      gaitForwardStride, gaitForwardCommand * scale, maximumStep);
  gaitTurnStride = approachGaitCommand(
      gaitTurnStride, gaitTurnCommand * scale, maximumStep);
  gaitStrafeStride = approachGaitCommand(
      gaitStrafeStride, gaitStrafeCommand * scale, maximumStep);
}

float halfCosine01(float value) {
  const float t = constrain(value, 0.0f, 1.0f);
  return 0.5f - 0.5f * cosf(kPi * t);
}

float smootherstep01(float value) {
  const float t = constrain(value, 0.0f, 1.0f);
  return t * t * t * (t * (6.0f * t - 15.0f) + 10.0f);
}

void sampleGaitFootPath(float cycle,
                        float halfStrideMm,
                        float liftMm,
                        float stanceFraction,
                        float swingShape,
                        float *xOffsetMm,
                        float *zOffsetMm) {
  cycle -= floorf(cycle);
  if (cycle < stanceFraction) {
    // A planted foot must move rearward at constant speed relative to the
    // body. The old half-cosine repeatedly accelerated and decelerated the
    // loaded foot across the floor, which produced visible scrub and sideways
    // reaction forces even for a steady forward command.
    const float stance = cycle / stanceFraction;
    *xOffsetMm = halfStrideMm * (1.0f - 2.0f * stance);
    *zOffsetMm = 0.0f;
    return;
  }

  // Return the unloaded foot from rear to front (keep in step with
  // trotSwingPath in simulation/standalone/web/src/gait-lab.js).
  //  - Height: lift fast, hold through the middle, come down over the last
  //    part. The old sin^shape curve crept off the floor, so feet scraped at
  //    the start and end of every step. swingShape sets the rise/fall share.
  //  - Fore/aft: the foot leaves and lands moving rearward at the stance speed
  //    (standing still over the ground), via a cubic Hermite with matched end
  //    velocities. The old curve stopped the foot relative to the body at both
  //    ends, so it touched down moving forward at walking speed and skidded.
  const float swing = constrain((cycle - stanceFraction) / (1.0f - stanceFraction), 0.0f, 1.0f);
  const float rise = constrain(0.18f * swingShape, 0.15f, 0.45f);
  const float height = fminf(smootherstep01(swing / rise), smootherstep01((1.0f - swing) / rise));
  const float duty = constrain(stanceFraction, 0.30f, 0.95f);
  const float groundSpeed = -2.0f * (1.0f - duty) / duty;  // half strides per unit swing
  const float s2 = swing * swing;
  const float s3 = s2 * swing;
  const float h00 = 2.0f * s3 - 3.0f * s2 + 1.0f;
  const float h10 = s3 - 2.0f * s2 + swing;
  const float h01 = -2.0f * s3 + 3.0f * s2;
  const float h11 = s3 - s2;
  *xOffsetMm = halfStrideMm * (-h00 + groundSpeed * h10 + h01 + groundSpeed * h11);
  *zOffsetMm = -liftMm * height;
}

void applySinusoidalGait(Adafruit_PWMServoDriver &driver, float baseZ) {
  const GaitProfileSettings &profile = gaitProfile().settings;
  constexpr float kControlStepSeconds = static_cast<float>(kControlIntervalMs) / 1000.0f;
  const float gaitBodyTargetZ = fminf(baseZ, profile.bodyHeightMm);
  const float responseSeconds = profile.responseMs / 1000.0f;
  const float commandSlewPerSec = 1.0f / responseSeconds;
  const float bodySlewMmPerSec = 60.0f / responseSeconds;
  gaitBodyZ = approachGaitCommand(
      gaitBodyZ,
      gaitBodyTargetZ,
      bodySlewMmPerSec * kControlStepSeconds);
  float targetForward = readGaitForwardInput(profile);
  float targetTurn = 0.0f;
  float targetStrafe = 0.0f;
  readGaitTrotInputs(profile, &targetForward, &targetTurn, &targetStrafe);
  {
    // Gyro heading hold while sidestepping (see HeadingHold).
    const uint32_t nowMs = millis();
    const bool imuFresh = imuSampleFresh(nowMs) && gImuState.yaw_initialized;
    const float operatorTurn = profile.maxTurnScale > 0.0f ? targetTurn / profile.maxTurnScale : 0.0f;
    const float strafeMagnitude = fabsf(targetStrafe);
    const float sidestepShare = strafeMagnitude / (fabsf(targetForward) + strafeMagnitude + 1e-6f);
    const float correction = gaitHeadingHold.update(
        imuFresh, gImuState.yaw_deg, operatorTurn,
        strafeMagnitude > 0.02f * profile.maxForwardScale, sidestepShare, kControlStepSeconds);
    targetTurn = constrain(operatorTurn + correction, -1.0f, 1.0f) * profile.maxTurnScale;
  }
  const float maximumCommandStep = commandSlewPerSec * kControlStepSeconds;
  gaitForwardCommand = approachGaitCommand(gaitForwardCommand, targetForward, maximumCommandStep);
  gaitTurnCommand = approachGaitCommand(gaitTurnCommand, targetTurn, maximumCommandStep);
  gaitStrafeCommand = approachGaitCommand(gaitStrafeCommand, targetStrafe, maximumCommandStep);

  const float activity = constrain(
      sqrtf(gaitForwardCommand * gaitForwardCommand +
            gaitTurnCommand * gaitTurnCommand +
            gaitStrafeCommand * gaitStrafeCommand),
      0.0f,
      1.0f);
  const float startupBlend = halfCosine01(activity / 0.25f);
  const float targetAmplitude =
      startupBlend * (0.65f + 0.35f * activity);
  gaitAmplitude = approachGaitCommand(
      gaitAmplitude,
      targetAmplitude,
      1.35f * commandSlewPerSec * kControlStepSeconds);
  updateGaitStride(activity, 1.35f * maximumCommandStep);
  if (activity > 0.001f || gaitAmplitude > 0.001f) {
    const float frequencyHz = profile.cadenceHz;
    gaitPhaseRad = fmodf(gaitPhaseRad + 2.0f * kPi * frequencyHz * kControlStepSeconds,
                         2.0f * kPi);
  } else {
    gaitPhaseRad = 0.0f;
  }

  const float normalizedCycle = gaitPhaseRad / (2.0f * kPi);
  outputHeightMm = gaitBodyZ;
  outputRollDeg = outputPitchDeg = outputYawDeg = 0.0f;
  outputBodyX = outputBodyY = 0.0f;
  outputUsesBodyPose = false;
  GaitStride strides[kLegCount];
  computeGaitStrides(profile, gaitForwardStride, gaitTurnStride, gaitStrafeStride, strides);
  // Sidestepping loads the feet differently from walking forward: landing them
  // further back while sidestepping stops the body creeping backwards.
  const float sidestepShareOfMotion = activity > 0.001f
      ? constrain(fabsf(gaitStrafeCommand) / activity, 0.0f, 1.0f) : 0.0f;
  const float touchdownXMm = profile.touchdownXMm +
      kGaitSidestepTouchdownShiftMm * sidestepShareOfMotion;
  for (int i = 0; i < kLegCount; ++i) {
    const bool diagonalA = (i == LEG_FL || i == LEG_BR);
    const bool leftLeg = (i == LEG_FL || i == LEG_BL);
    const float sideSign = leftLeg ? 1.0f : -1.0f;
    // The path is linear in the half stride: sample it once at unit length,
    // then lay it along this leg's stride vector.
    float progress = 0.0f;
    float zOffsetMm = 0.0f;
    sampleGaitFootPath(normalizedCycle + (diagonalA ? 0.0f : profile.diagonalPhase),
                       1.0f,
                       profile.liftMm * gaitAmplitude,
                       profile.dutyFactor,
                       profile.swingShape,
                       &progress,
                       &zOffsetMm);
    const float x = touchdownXMm + 0.5f * strides[i].x * progress;
    const float y = sideSign * profile.stanceWidthMm + 0.5f * strides[i].y * progress;
    gaitSupportZ[i] = gaitBodyZ + manualRollFootZOffset(y);
    const float z = gaitSupportZ[i] + zOffsetMm;
    commandLeg(driver, i, x, y, z);
  }
}

// Static crawl: one foot in the air at a time, with all four feet down between
// steps. Before each foot lifts, the body shifts away from it, toward the middle
// of the three feet that stay down. Without that shift the centre of mass sits
// on the line between the two diagonal support feet, the robot tips toward the
// lifted corner, and that foot drags and catches. Legs lift in the lateral
// order back-left, front-left, back-right, front-right, which keeps the most
// stability margin for a slow walk.
constexpr LegIndex kCrawlOrder[kLegCount] = {LEG_BL, LEG_FL, LEG_BR, LEG_FR};
constexpr float kCrawlSlot = 0.25f;                // each leg owns a quarter cycle
constexpr float kCrawlMaxSwingFraction = 0.17f;    // of the full cycle, per leg
constexpr float kCrawlBodyShiftMm = 30.0f;         // body shift away from the lifting leg

// Unit vector from the body centre toward a foot's nominal position.
void crawlFootDirection(const GaitProfileSettings &profile, LegIndex leg, float *ux, float *uy) {
  const bool front = leg == LEG_FL || leg == LEG_FR;
  const bool left = leg == LEG_FL || leg == LEG_BL;
  const float px = (front ? BODY_HALF_LENGTH_X : -BODY_HALF_LENGTH_X) + profile.touchdownXMm;
  const float py = (left ? 1.0f : -1.0f) * (BODY_HALF_WIDTH_Y + profile.stanceWidthMm);
  const float radius = fmaxf(1.0f, sqrtf(px * px + py * py));
  *ux = px / radius;
  *uy = py / radius;
}

// Foot offset (in the leg frame) that moves the body away from the lifting
// leg. Holds the full shift while a leg is in the air and blends to the next
// leg's shift during the four-feet-down gap between steps.
void crawlBodyShift(const GaitProfileSettings &profile, float cycle, float swingFraction,
                    float amplitude, float *dx, float *dy) {
  const float gap = kCrawlSlot - swingFraction;
  // Slot k: [k*0.25, (k+1)*0.25). Its swing is centred, with half the gap on each side.
  float t = cycle - floorf(cycle);
  const int slot = static_cast<int>(t / kCrawlSlot) % kLegCount;
  const float inSlot = t - slot * kCrawlSlot;
  int from = slot;
  int to = slot;
  float blend = 0.0f;
  if (inSlot < gap * 0.5f) {
    from = (slot + kLegCount - 1) % kLegCount;
    blend = 0.5f + 0.5f * (inSlot / (gap * 0.5f));
  } else if (inSlot > kCrawlSlot - gap * 0.5f) {
    to = (slot + 1) % kLegCount;
    blend = 0.5f * (inSlot - (kCrawlSlot - gap * 0.5f)) / (gap * 0.5f);
  }
  const float eased = halfCosine01(blend);
  float fx, fy, tx, ty;
  crawlFootDirection(profile, kCrawlOrder[from], &fx, &fy);
  crawlFootDirection(profile, kCrawlOrder[to], &tx, &ty);
  // Feet move toward the lifting leg's side relative to the body, which moves
  // the body (and its centre of mass) away from that leg.
  *dx = amplitude * kCrawlBodyShiftMm * (fx + (tx - fx) * eased);
  *dy = amplitude * kCrawlBodyShiftMm * (fy + (ty - fy) * eased);
}

void applyCarefulWalk(Adafruit_PWMServoDriver &driver, float baseZ) {
  const GaitProfileSettings &profile = gaitProfile().settings;
  constexpr float kControlStepSeconds = static_cast<float>(kControlIntervalMs) / 1000.0f;
  const float gaitBodyTargetZ = fminf(baseZ, profile.bodyHeightMm);
  const float responseSeconds = profile.responseMs / 1000.0f;
  const float commandSlewPerSec = 1.0f / responseSeconds;
  gaitBodyZ = approachGaitCommand(
      gaitBodyZ, gaitBodyTargetZ, (60.0f / responseSeconds) * kControlStepSeconds);
  const float targetForward = readGaitForwardInput(profile);
  const float targetTurn = readGaitTurnInput(profile, targetForward);
  const float maximumCommandStep = commandSlewPerSec * kControlStepSeconds;
  gaitForwardCommand = approachGaitCommand(gaitForwardCommand, targetForward, maximumCommandStep);
  gaitTurnCommand = approachGaitCommand(gaitTurnCommand, targetTurn, maximumCommandStep);

  const float activity = constrain(
      sqrtf(gaitForwardCommand * gaitForwardCommand + gaitTurnCommand * gaitTurnCommand),
      0.0f, 1.0f);
  const float targetAmplitude = halfCosine01(activity / 0.20f) * (0.72f + 0.28f * activity);
  gaitAmplitude = approachGaitCommand(
      gaitAmplitude, targetAmplitude, 1.35f * commandSlewPerSec * kControlStepSeconds);
  updateGaitStride(activity, 1.35f * maximumCommandStep);
  if (activity > 0.001f || gaitAmplitude > 0.001f) {
    const float frequencyHz = profile.cadenceHz;
    gaitPhaseRad = fmodf(
        gaitPhaseRad + 2.0f * kPi * frequencyHz * kControlStepSeconds, 2.0f * kPi);
  } else {
    gaitPhaseRad = 0.0f;
  }

  const float normalizedCycle = gaitPhaseRad / (2.0f * kPi);
  outputHeightMm = gaitBodyZ;
  outputRollDeg = outputPitchDeg = outputYawDeg = 0.0f;
  outputBodyX = outputBodyY = 0.0f;
  outputUsesBodyPose = false;
  const float swingFraction = fminf(kCrawlMaxSwingFraction, 1.0f - profile.dutyFactor);
  const float gap = kCrawlSlot - swingFraction;
  GaitStride strides[kLegCount];
  computeGaitStrides(profile, gaitForwardStride, gaitTurnStride, 0.0f, strides);
  float shiftX = 0.0f;
  float shiftY = 0.0f;
  crawlBodyShift(profile, normalizedCycle, swingFraction, gaitAmplitude, &shiftX, &shiftY);
  for (int orderIndex = 0; orderIndex < kLegCount; ++orderIndex) {
    const LegIndex leg = kCrawlOrder[orderIndex];
    const bool leftLeg = leg == LEG_FL || leg == LEG_BL;
    const float sideSign = leftLeg ? 1.0f : -1.0f;
    // This leg's swing starts half a gap into its slot.
    float legCycle = normalizedCycle - kCrawlSlot * static_cast<float>(orderIndex) - gap * 0.5f;
    legCycle -= floorf(legCycle);
    float progress = 0.0f;
    float zOffsetMm = 0.0f;
    if (legCycle < swingFraction) {
      const float swing = legCycle / swingFraction;
      progress = -1.0f + 2.0f * halfCosine01(swing);
      const float liftWave = sinf(kPi * swing);
      zOffsetMm = -profile.liftMm * gaitAmplitude *
          powf(fmaxf(0.0f, liftWave), profile.swingShape);
    } else {
      const float stance = (legCycle - swingFraction) / (1.0f - swingFraction);
      progress = 1.0f - 2.0f * stance;
    }
    gaitSupportZ[leg] = gaitBodyZ +
        manualRollFootZOffset(sideSign * profile.stanceWidthMm);
    commandLeg(driver,
               leg,
               profile.touchdownXMm + 0.5f * strides[leg].x * progress + shiftX,
               sideSign * profile.stanceWidthMm + 0.5f * strides[leg].y * progress + shiftY,
               gaitSupportZ[leg] + zOffsetMm);
  }
}

bool updateStandCommand(bool standActive, uint32_t now) {
  static SwitchDebounceState saDebounce;
  const int saValueUs = ch_us[SA_CH_INDEX];
  bool desiredState = standActive;
  if (standActive) {
    if (saValueUs < SA_OFF_THRESHOLD_US) {
      desiredState = false;
    }
  } else {
    if (saValueUs > SA_ON_THRESHOLD_US) {
      desiredState = true;
    }
  }

  return applySwitchDebounce(standActive, desiredState, now, &saDebounce, kSwitchDebounceDefaultMs);
}

bool updateSdCommand(bool sdActive, bool controlsEnabled, uint32_t now) {
  static SwitchDebounceState sdDebounce;
  if (!controlsEnabled) {
    sdDebounce.initialized = false;
    return false;
  }

  const int sdValueUs = ch_us[SD_CH_INDEX];
  bool desiredState = sdActive;
  if (sdActive) {
    if (sdValueUs < SD_OFF_THRESHOLD_US) {
      desiredState = false;
    }
  } else {
    if (sdValueUs > SD_ON_THRESHOLD_US) {
      desiredState = true;
    }
  }

  return applySwitchDebounce(sdActive, desiredState, now, &sdDebounce, kSwitchDebounceDefaultMs);
}

void applyTiltPose(Adafruit_PWMServoDriver &driver) {
  // Tilt mode: interpret stick inputs as a small body pose (roll, pitch, yaw)
  // about the body center while the feet stay approximately in the nominal
  // stand locations derived from BODY_HALF_* and FOOT_* constants.
  //
  // roll: right stick X (lean left/right)
  // pitch: right stick Y (lean forward/back)
  // yaw: left stick X (twist the body about Z)
  const float yawNorm = applyTiltInputDeadband(readMotionChannelNormalized(YAW_CH_INDEX));
  float rollNorm = applyTiltInputDeadband(readMotionChannelNormalized(ROLL_CH_INDEX));
  float pitchNorm = applyTiltInputDeadband(readMotionChannelNormalized(PITCH_CH_INDEX));
  float yawNormLimited = yawNorm;

  // Keep diagonal stick commands inside one combined body-pose envelope.
  // Without this, maximum roll, pitch, and yaw could all be applied at once,
  // producing a much larger compound rotation than any individual limit.
  const float commandMagnitude =
      sqrtf(rollNorm * rollNorm + pitchNorm * pitchNorm + yawNormLimited * yawNormLimited);
  if (commandMagnitude > 1.0f) {
    const float scale = 1.0f / commandMagnitude;
    rollNorm *= scale;
    pitchNorm *= scale;
    yawNormLimited *= scale;
  }

  const MotionSmoothingSettings &smoothing = motionSmoothingSettings();
  const float rollTargetDeg = rollNorm * kMaxRollDeg;
  const float pitchTargetDeg = pitchNorm * kMaxPitchDeg;
  // CH4/operator turn is positive to the right. Body +Y points left, so
  // positive mathematical yaw turns left: convert only the body-pose target.
  // Gait steering already advances the left side for positive/right input.
  const float yawTargetDeg = -yawNormLimited * kMaxYawDeg;
  const float rollDeg = commandWithMotionSmoothing(
      tiltRollRamp, rollTargetDeg, smoothing.rollRateDegPerSec, smoothing.enabled);
  const float pitchDeg = commandWithMotionSmoothing(
      tiltPitchRamp, pitchTargetDeg, smoothing.pitchRateDegPerSec, smoothing.enabled);
  const float yawDeg = commandWithMotionSmoothing(
      tiltYawRamp, yawTargetDeg, smoothing.yawRateDegPerSec, smoothing.enabled);

#ifdef DOMINO_SIL
  silBodyRollDeg = rollDeg;
  silBodyPitchDeg = pitchDeg;
  silBodyYawDeg = yawDeg;
#endif

  const float bodyX = 0.0f;
  const float bodyY = 0.0f;
  // Map the current leg Z target (lastPoseZ) into a body Z so that the
  // neutral tilt height matches the stand height driven by zRamp.
  const float bodyZ = lastPoseZ + STAND_HEIGHT_Z;

  moveLegsFromBodyPose(driver, bodyX, bodyY, bodyZ, rollDeg, pitchDeg, yawDeg);
}

void enterTiltMode() {
  tiltRollRamp.go(0.0f);
  tiltPitchRamp.go(0.0f);
  tiltYawRamp.go(0.0f);
  tiltModeActive = true;
  Serial.printf("Tilt mode enabled (SA=%d SD=%d)\n", ch_us[SA_CH_INDEX], ch_us[SD_CH_INDEX]);
}

void exitTiltMode() {
  tiltModeActive = false;
  tiltRollRamp.go(0.0f);
  tiltPitchRamp.go(0.0f);
  tiltYawRamp.go(0.0f);
  Serial.printf("Tilt mode disabled (SA=%d SD=%d)\n", ch_us[SA_CH_INDEX], ch_us[SD_CH_INDEX]);
}

float transitionBlend(float value) {
  const float t = constrain(value, 0.0f, 1.0f);
  // Quintic position easing: zero slope and curvature at either endpoint.
  return t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
}

void beginTransitionStage(TransitionStage stage, const LegTarget targets[kLegCount],
                          float height, bool bodyPose) {
  modeTransition.stage = stage;
  modeTransition.fromHeight = outputHeightMm;
  modeTransition.toHeight = height;
  modeTransition.fromRoll = outputRollDeg;
  modeTransition.fromPitch = outputPitchDeg;
  modeTransition.fromYaw = outputYawDeg;
  modeTransition.fromBodyX = outputBodyX;
  modeTransition.fromBodyY = outputBodyY;
  modeTransition.bodyPose = bodyPose;
  modeTransition.seconds = 0.0f;
  float distance = fabsf(height - outputHeightMm);
  for (int i = 0; i < kLegCount; ++i) {
    modeTransition.from[i] = lastLegTargets[i];
    modeTransition.to[i] = targets[i];
    const LegTarget &from = modeTransition.from[i];
    const LegTarget &to = targets[i];
    distance = fmaxf(distance, sqrtf((to.x - from.x) * (to.x - from.x) +
                                    (to.y - from.y) * (to.y - from.y) +
                                    (to.z - from.z) * (to.z - from.z)));
  }
  const float angle = fmaxf(fabsf(outputRollDeg),
      fmaxf(fabsf(outputPitchDeg), fabsf(outputYawDeg)));
  // 1.875 is the peak slope of the quintic blend. Body rotations have a
  // curved endpoint path; the additional margin bounds its sampled speed.
  modeTransition.duration = fmaxf(kTransitionMinimumSeconds,
      fmaxf(2.5f * distance / kTransitionFootSpeedMmPerSec,
            1.875f * angle / kTransitionAngleSpeedDegPerSec));
  motionInputAwaitingCenter = true;
  motionInputCenteredSinceMs = 0;
  gaitForwardCommand = gaitTurnCommand = gaitStrafeCommand = gaitAmplitude = 0.0f;
  gaitForwardStride = gaitTurnStride = gaitStrafeStride = 0.0f;
}

void beginNeutralTransition(BodyMode requestedMode, float rideHeight) {
  // Stow returns to level at the current height before lowering; it must not
  // first raise a low robot to the radio's standing height on link loss.
  const float height = requestedMode == BODY_STOW ? outputHeightMm
      : computeStandTargetZ(rideHeight);
  LegTarget targets[kLegCount];
  for (int i = 0; i < kLegCount; ++i) {
    bodyKinematicsSimple(static_cast<LegIndex>(i), 0.0f, 0.0f,
                         height + STAND_HEIGHT_Z, 0.0f, 0.0f, 0.0f,
                         &targets[i].x, &targets[i].y, &targets[i].z);
  }
  beginTransitionStage(TRANSITION_NEUTRAL, targets, height, outputUsesBodyPose);
}

void beginModeTransition(BodyMode requestedMode, float rideHeight) {
  if (outputMode == BODY_POLICY) {
    // A learned step can leave feet lifted or displaced. Lower every foot to
    // at least the support height in place, then recentre (TRANSITION_LAND
    // continues into the neutral stage).
    LegTarget targets[kLegCount];
    for (int i = 0; i < kLegCount; ++i) {
      targets[i] = lastLegTargets[i];
      targets[i].z = fmaxf(lastLegTargets[i].z, outputHeightMm);
    }
    beginTransitionStage(TRANSITION_LAND, targets, outputHeightMm, false);
    return;
  }
  if (isWalkingMode(outputMode)) {
    // Freeze fore/aft and lateral coordinates while lowered feet regain
    // support. Never recenter a lifted foot and collapse the body together.
    LegTarget targets[kLegCount];
    for (int i = 0; i < kLegCount; ++i) {
      targets[i] = lastLegTargets[i];
      // Use the last emitted support plane, even if a LIVE roll command or
      // its deadman changes on this frame. Landing must not lift planted feet.
      targets[i].z = fmaxf(lastLegTargets[i].z, gaitSupportZ[i]);
    }
    beginTransitionStage(TRANSITION_LAND, targets, gaitBodyZ, false);
  } else {
    beginNeutralTransition(requestedMode, rideHeight);
  }
}

bool applyModeTransition(Adafruit_PWMServoDriver &driver,
                         BodyMode requestedMode, float rideHeight) {
  if (!modeTransitionActive()) return false;
  modeTransition.seconds += static_cast<float>(kControlIntervalMs) / 1000.0f;
  const float t = transitionBlend(modeTransition.seconds / modeTransition.duration);
  const float height = modeTransition.fromHeight +
      t * (modeTransition.toHeight - modeTransition.fromHeight);
  if (modeTransition.bodyPose) {
    moveLegsFromBodyPose(driver, (1.0f - t) * modeTransition.fromBodyX,
                        (1.0f - t) * modeTransition.fromBodyY, height + STAND_HEIGHT_Z,
                        (1.0f - t) * modeTransition.fromRoll,
                        (1.0f - t) * modeTransition.fromPitch,
                        (1.0f - t) * modeTransition.fromYaw);
  } else {
    outputHeightMm = height;
    outputRollDeg = outputPitchDeg = outputYawDeg = 0.0f;
    outputBodyX = outputBodyY = 0.0f;
    outputUsesBodyPose = false;
    for (int i = 0; i < kLegCount; ++i) {
      const LegTarget &from = modeTransition.from[i];
      const LegTarget &to = modeTransition.to[i];
      commandLeg(driver, i, from.x + t * (to.x - from.x),
                  from.y + t * (to.y - from.y), from.z + t * (to.z - from.z));
    }
  }
  lastPoseZ = outputHeightMm;
  if (modeTransition.seconds < modeTransition.duration) return true;

  // Re-read the requested mode at stage boundaries. Rapid toggles replace the
  // pending destination without restarting a trajectory from an old pose.
  if (modeTransition.stage == TRANSITION_LAND) {
    beginNeutralTransition(requestedMode, rideHeight);
  } else if (modeTransition.stage == TRANSITION_NEUTRAL &&
             requestedMode != BODY_STOW && fabsf(outputHeightMm - rideHeight) > 0.5f) {
    beginNeutralTransition(requestedMode, rideHeight);
  } else if (modeTransition.stage == TRANSITION_NEUTRAL && isWalkingMode(requestedMode)) {
    const GaitProfileSettings &profile = gaitProfile().settings;
    const float gaitHeight = fminf(outputHeightMm, profile.bodyHeightMm);
    LegTarget targets[kLegCount];
    for (int i = 0; i < kLegCount; ++i) {
      const float side = (i == LEG_FL || i == LEG_BL) ? 1.0f : -1.0f;
      targets[i] = {profile.touchdownXMm, side * profile.stanceWidthMm, gaitHeight};
    }
    beginTransitionStage(TRANSITION_WALK_STANCE, targets, gaitHeight, false);
  } else if (modeTransition.stage == TRANSITION_WALK_STANCE && !isWalkingMode(requestedMode)) {
    beginNeutralTransition(requestedMode, rideHeight);
  } else {
    modeTransition.stage = TRANSITION_NONE;
    outputMode = requestedMode;
    resetGaitState();
    tiltRollRamp.go(0.0f);
    tiltPitchRamp.go(0.0f);
    tiltYawRamp.go(0.0f);
    motionInputAwaitingCenter = motionProfileNeedsCentredSticks(motionInputProfileForMode(outputMode));
    motionInputCenteredSinceMs = 0;
  }
  return true;
}
}  // namespace

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();
rampFloat zRamp;
float currentTargetZ = kCloserPoseZ;
MenuState menuState{BODY_STOW, kRideHeightMaxMm, false, true};

// ---------------------------------------------------------------------------
// Learned policy ("simulation mode", SB fully down)
// ---------------------------------------------------------------------------
// Maximum stick commands. Keep inside the ranges the policy was trained on
// (simulation/isaac_lab/config/training.json "commands").
constexpr float kPolicyMaxForwardMps = 0.50f;
constexpr float kPolicyMaxReverseMps = 0.30f;
constexpr float kPolicyMaxLateralMps = 0.25f;
constexpr float kPolicyMaxYawRadps = 1.50f;
constexpr uint32_t kPolicyDebounceMs = 200;
bool policyFaultLatched = false;
uint32_t policyInferLastUs = 0;
uint32_t policyInferMaxUs = 0;
uint32_t policyStepCount = 0;

// MPU6050 sensor frame -> body frame (+x forward, +y left, +z up). Same
// mounting transform as the balance mode: body = (ay, az, -ax).
PolicyImu readPolicyImu(uint32_t now) {
  PolicyImu imu{};
  constexpr float kDpsToRadps = kPi / 180.0f;
  imu.fresh = gImuState.online && gImuState.has_sample && imuSampleFresh(now);
  imu.accelG[0] = gImuState.ay_g;
  imu.accelG[1] = gImuState.az_g;
  imu.accelG[2] = -gImuState.ax_g;
  imu.gyroRadps[0] = gImuState.gy_dps * kDpsToRadps;
  imu.gyroRadps[1] = gImuState.gz_dps * kDpsToRadps;
  imu.gyroRadps[2] = -gImuState.gx_dps * kDpsToRadps;
  return imu;
}

PolicyCommand readPolicyCommand(float heightMm) {
  PolicyCommand cmd{};
  const float forward = applyGaitDeadband(readMotionChannelNormalized(PITCH_CH_INDEX));
  const float lateral = applyGaitDeadband(readMotionChannelNormalized(ROLL_CH_INDEX));
  const float turn = applyGaitDeadband(readMotionChannelNormalized(YAW_CH_INDEX));
  cmd.vxMps = forward >= 0.0f ? forward * kPolicyMaxForwardMps : forward * kPolicyMaxReverseMps;
  cmd.vyMps = -lateral * kPolicyMaxLateralMps;  // stick right = move right (-y)
  cmd.wzRadps = -turn * kPolicyMaxYawRadps;     // stick right = clockwise
  cmd.heightMm = heightMm;
  return cmd;
}

void updatePolicyMode(BodyMode previousMode, const MenuInputs &inputs, bool standPoseReady,
                      uint32_t now) {
  static SwitchDebounceState policyDebounce;
  if (!inputs.policyRequested) policyFaultLatched = false;
  const bool wasActive = previousMode == BODY_POLICY;
  const PolicyImu imu = readPolicyImu(now);
  const bool allowed = (menuState.mode == BODY_STAND || menuState.mode == BODY_GAIT ||
                        menuState.mode == BODY_CAREFUL || wasActive) &&
      menuState.mode != BODY_STOW && !tiltModeActive && !inputs.failsafeActive &&
      inputs.motionCommandsFresh && !motionIkFault && !policyFaultLatched &&
      policyStatus().selfTestOk && imu.fresh;
  const bool raw = inputs.policyRequested && allowed && (wasActive || standPoseReady);
  bool filtered = applySwitchDebounce(wasActive, raw, now, &policyDebounce, kPolicyDebounceMs);
  if (!allowed || !inputs.policyRequested) filtered = false;
  if (filtered) {
    menuState.mode = BODY_POLICY;
  } else if (menuState.mode == BODY_POLICY) {
    menuState.mode = BODY_STAND;
  }
  if (!wasActive && menuState.mode == BODY_POLICY) {
    Serial.printf("Entering learned policy %s (SB=%d)\n", policyId(), ch_us[SB_CH_INDEX]);
  } else if (wasActive && menuState.mode != BODY_POLICY) {
    Serial.printf("Exiting learned policy (fault=%s)\n", policyFaultName(policyStatus().fault));
    policyStop();
  }
}

// The policy was trained with the ride-height command slewed at
// policyHeightSlewMmPerSec() (the CH3 stick can jump; the trainer never saw
// instant height steps).
float policyHeightMm = kRideHeightMaxMm;

void applyPolicyStep(Adafruit_PWMServoDriver &driver, float heightMm, uint32_t now) {
  const PolicyImu imu = readPolicyImu(now);
  if (!policyStatus().active) {
    policyReset(imu);
    policyInferMaxUs = 0;
    policyHeightMm = heightMm;  // entered from a stand at this height
  }
  const float maxStep = policyHeightSlewMmPerSec() * static_cast<float>(kControlIntervalMs) / 1000.0f;
  policyHeightMm += fminf(maxStep, fmaxf(-maxStep, heightMm - policyHeightMm));
  heightMm = policyHeightMm;
  const PolicyCommand cmd = readPolicyCommand(heightMm);
  PolicyFeet feet{};
  const uint32_t t0 = micros();
  const bool ok = policyStep(cmd, imu, static_cast<float>(kControlIntervalMs) / 1000.0f, &feet);
  policyInferLastUs = micros() - t0;
  if (policyInferLastUs > policyInferMaxUs) policyInferMaxUs = policyInferLastUs;
  ++policyStepCount;
  if (!ok) {
    // Hold the last commanded feet; the mode logic lands and recentres them.
    policyFaultLatched = true;
    Serial.printf("Learned policy fault: %s\n", policyFaultName(policyStatus().fault));
    return;
  }
  outputHeightMm = heightMm;
  outputRollDeg = outputPitchDeg = outputYawDeg = 0.0f;
  outputBodyX = outputBodyY = 0.0f;
  outputUsesBodyPose = false;
  for (int i = 0; i < kLegCount; ++i) {
    const float side = (i == LEG_FL || i == LEG_BL) ? 1.0f : -1.0f;
    commandLeg(driver, i, feet.xMm[i], side * feet.yOutMm[i], feet.zMm[i]);
  }
}

extern "C" void dominoPolicyTelemetry(uint32_t *lastUs, uint32_t *maxUs, uint32_t *steps, bool *latched) {
  *lastUs = policyInferLastUs;
  *maxUs = policyInferMaxUs;
  *steps = policyStepCount;
  *latched = policyFaultLatched;
}

void setup() {
  // LIVE telemetry contains full controller, pose and servo snapshots. Use
  // large rings in both directions: profile-write commands are roughly 1.6 KB
  // and can otherwise overrun the ESP32's small default USB-UART RX buffer
  // before the control loop drains it.
#ifndef DOMINO_SIL
  Serial.setTxBufferSize(4096);
  Serial.setRxBufferSize(4096);
#endif
  Serial.begin(460800);
#ifndef DOMINO_SIL
  loopDiagnosticsInit();
#endif
#ifndef DOMINO_SIL
  // 100 kHz with a bounded timeout, and free the bus first in case a device
  // was left mid-transfer by a reset (it would otherwise hold SDA low).
  i2cBusBegin();
  i2cBusEnsureIdle();
#else
  Wire.begin();
#endif

#if DOMINO_IMU_ENABLED
  imuInit();
#endif

#ifndef DOMINO_SIL
  Serial2.setRxBufferSize(CRSF_RX_BUFFER_BYTES);
#endif
  Serial2.begin(CRSF_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  initCrsfState();

  pca.begin();
  pca.setOscillatorFrequency(kOscillatorFreqHz);
  pca.setPWMFreq(kServoPwmFreqHz);
  delay(10);
#ifndef DOMINO_SIL
  // Read the prescale once so servo writes don't re-read it every time.
  setServoPwmTiming(pca.readPrescale(), kOscillatorFreqHz);
#endif

#ifndef DOMINO_SIL
  liveRobotEndpointBegin(pca);
#endif

  const MotionSmoothingSettings &smoothing = motionSmoothingSettings();
  zRamp.setSpeed(smoothing.heightRateMmPerSec);
  zRamp.go(kCloserPoseZ);
  tiltRollRamp.setSpeed(smoothing.rollRateDegPerSec);
  tiltPitchRamp.setSpeed(smoothing.pitchRateDegPerSec);
  tiltYawRamp.setSpeed(smoothing.yawRateDegPerSec);
  tiltRollRamp.go(0.0f);
  tiltPitchRamp.go(0.0f);
  tiltYawRamp.go(0.0f);
  lastPoseZ = kCloserPoseZ;
  sitBlend = 1.0f;
  moveLegsToSitBlend(pca, lastPoseZ, sitBlend);
  standState = STOW;
  const bool policyOk = policySelfTest();
  Serial.printf("Learned policy: %s id=%s run=%s it=%d self-test=%s (err %.2e)\n",
                policyStatus().available ? "installed" : "not installed", policyId(), policyRunName(),
                policyIteration(), policyOk ? "ok" : "FAILED", policyStatus().selfTestMaxError);
  Serial.println("All legs initialized at stow pose (waiting for CRSF link).");
  Serial.println("Debug logging ready.");
  logDebugState(millis(), false, false, false, false, false, false, tiltModeActive);
}

void loop() {
#ifndef DOMINO_SIL
  if (liveRobotEndpointReceiverPassthroughActive()) {
    liveRobotEndpointReceiverPassthroughLoop();
    return;
  }
#endif
  static uint32_t lastControlMs = 0;
  static uint32_t lastLinkAliveMs = 0;
  static bool failsafeActive = true;
  static SwitchDebounceState standModeDebounce;
  static SwitchDebounceState tiltDebounce;
  static SwitchDebounceState gaitDebounce;

  uint32_t now = millis();
  {
#ifndef DOMINO_SIL
    ScopedLoopStage stage(LoopStage::Crsf);
#endif
    processCrsfFrames(now);
  }
#ifndef DOMINO_SIL
  static uint32_t lastUartOverflowCount = 0;
  const uint32_t uartOverflowCount = crsfUartOverflowCount();
  if (uartOverflowCount != lastUartOverflowCount) {
    const uint32_t increase = uartOverflowCount - lastUartOverflowCount;
    loopBlackboxRecord(BlackboxEventKind::UartOverflow, 0, uartOverflowCount,
                       static_cast<uint16_t>(increase < UINT16_MAX ? increase : UINT16_MAX));
    lastUartOverflowCount = uartOverflowCount;
  }
#endif
#if DOMINO_IMU_ENABLED
  // Refresh IMU sample; failures simply leave the previous values.
  {
#ifndef DOMINO_SIL
    ScopedLoopStage stage(LoopStage::Imu);
#endif
    (void)imuReadSample();
  }
#endif
  // A delayed UART or I2C read must not make the remainder of this pass use
  // a timestamp captured before the delay.
  now = millis();
#ifndef DOMINO_SIL
  {
    ScopedLoopStage stage(LoopStage::Live);
    liveRobotEndpointLoop(now, pca);
  }
  if (liveRobotEndpointReceiverPassthroughActive()) return;
  activeManualControl = liveRobotEndpointManualControl();
  // A transport read may take time; schedule the servo pass against the
  // current clock rather than the timestamp captured before that read.
  now = millis();
#endif

  // 1) Read current RC "menu inputs" (switches, link status).
  MenuInputs inputs = readMenuInputs(now, &lastLinkAliveMs, &failsafeActive);
#ifndef DOMINO_SIL
  if (activeManualControl.active) {
    // The Boxer link remains the physical safety prerequisite, but the
    // session-bound browser lease owns motion mode and bounded axes.
    inputs.standRequested = true;
    inputs.tiltSwitchDown = false;
    inputs.balanceRequested = false;
    inputs.carefulWalkRequested = activeManualControl.deadman &&
        activeManualControl.mode == LiveManualMode::Careful;
    inputs.gaitRequested = activeManualControl.deadman &&
        activeManualControl.mode == LiveManualMode::Trot;
    inputs.policyRequested = false;
    inputs.rideHeightMm = kRideHeightMinMm +
        (activeManualControl.height + 1.0f) * 0.5f *
        (kRideHeightMaxMm - kRideHeightMinMm);
  }
#endif
  if (motionIkFault) {
    // Stop further gait/tilt requests after an unreachable target. Re-arm
    // requires a valid neutral stand with both motion switches off.
    inputs.tiltSwitchDown = false;
    inputs.carefulWalkRequested = inputs.gaitRequested = false;
    inputs.policyRequested = false;
    inputs.balanceRequested = false;
    if (!modeTransitionActive() && outputMode == BODY_STAND &&
        ch_us[SC_CH_INDEX] < SC_CAREFUL_MIN_US && ch_us[SD_CH_INDEX] <= SD_OFF_THRESHOLD_US &&
        motionInputsAreCentered()) {
      bool neutral = fabsf(outputHeightMm - inputs.rideHeightMm) < 0.5f;
      for (int i = 0; i < kLegCount; ++i) {
        const float side = (i == LEG_FL || i == LEG_BL) ? 1.0f : -1.0f;
        neutral = neutral && fabsf(lastLegTargets[i].x - COMMAND_NEUTRAL_FOOT_X_FROM_HIP) < 0.5f &&
            fabsf(lastLegTargets[i].y - side * COMMAND_NEUTRAL_FOOT_Y_FROM_HIP) < 0.5f &&
            fabsf(lastLegTargets[i].z - inputs.rideHeightMm) < 0.5f;
      }
      if (neutral) motionIkFault = false;
    }
  }
  menuState.linkAlive = inputs.haveLink;
  menuState.failsafeActive = inputs.failsafeActive;

  // 2) Update the high-level body mode (stow / stand / tilt) with existing
  //    debouncing semantics, but represented as a single menuState.mode.
  const bool desiredStandCommand = inputs.standRequested;
  const bool filteredStandCommand =
      applySwitchDebounce(menuState.mode != BODY_STOW,
                          desiredStandCommand,
                          now,
                          &standModeDebounce,
                          kStandCommandHoldMs);

  BodyMode previousMode = menuState.mode;
  if (!filteredStandCommand || inputs.failsafeActive) {
    // For safety, force tilt mode off immediately when stowing or entering
    // failsafe so we do not continue applying tilt poses while ramping down.
    if (tiltModeActive) {
      exitTiltMode();
    }
    menuState.mode = BODY_STOW;
  } else {
    // Stand is requested; tilt may override below if allowed.
    menuState.mode = BODY_STAND;
  }

  // 3) Always remember and report the continuous CH3 height command. Stow
  //    still uses its fixed compact pose, but the selected height is retained
  //    for the next stand command.
  if (fabsf(inputs.rideHeightMm - currentRideHeightMm) >= 0.1f) {
    currentRideHeightMm = inputs.rideHeightMm;
    Serial.printf("Ride height target=%.1fmm CH3_HEIGHT=%d\n",
                  currentRideHeightMm,
                  ch_us[RIDE_HEIGHT_CH_INDEX]);
  }
  menuState.rideHeightMm = currentRideHeightMm;

  // 4) Compute the current Z target from the menu state.
  if (menuState.mode == BODY_STOW) {
    currentTargetZ = kCloserPoseZ;
  } else {
    currentTargetZ = computeStandTargetZ(menuState.rideHeightMm);
  }
  zRamp.move(currentTargetZ);

  // Maintain legacy standState for any existing debug/logic that still uses it.
  standState = (menuState.mode == BODY_STOW) ? STOW : STAND;

  // 5) Tilt candidate: entering tilt requires the pose to be "ready" at the
  //    chosen height, but once in tilt we keep it latched as long as the
  //    stand command and tilt switch remain active. This avoids dropping out
  //    of tilt while the requested ride height ramps continuously.
  const bool standPoseReady =
      (menuState.mode != BODY_STOW) &&
      (fabsf(lastPoseZ - currentTargetZ) <= kStandPoseToleranceMm) &&
      (sitBlend <= 0.05f);

  bool desiredTiltActive = false;
  if (tiltModeActive) {
    // Already in tilt: keep it as long as we are effectively "standing"
    // and the tilt switch is held (no pose-ready requirement during ramps).
    desiredTiltActive = (standState == STAND) && inputs.tiltSwitchDown && !inputs.failsafeActive;
  } else {
    // Entering tilt: require pose ready at the requested ride height so we
    // don't tilt while legs are still moving into position.
    desiredTiltActive =
        (standState == STAND) && inputs.tiltSwitchDown && standPoseReady && !inputs.failsafeActive;
  }

  const bool tiltCandidateRaw = desiredTiltActive;
  const bool tiltCandidateFiltered = inputs.motionCommandsFresh &&
      applySwitchDebounce(tiltModeActive, tiltCandidateRaw, now, &tiltDebounce, kTiltToggleDebounceMs);
  if (tiltCandidateFiltered && !tiltModeActive) {
    enterTiltMode();
  } else if (!tiltCandidateFiltered && tiltModeActive) {
    exitTiltMode();
  }
  menuState.mode = tiltModeActive ? BODY_TILT : menuState.mode;

  // 6) Walk mode: SC middle requests the stable crawl and SC high requests
  //    the diagonal trot. SD/tilt always wins. A single walking debounce also
  //    suppresses the transient middle position while moving the SC switch.
  const bool walkWasActive = previousMode == BODY_GAIT || previousMode == BODY_CAREFUL;
  const bool walkRequested = inputs.gaitRequested || inputs.carefulWalkRequested;
#ifndef DOMINO_SIL
  const bool manualOwnsMotion = activeManualControl.active;
#else
  constexpr bool manualOwnsMotion = false;
#endif
  const bool walkCandidateRaw =
      menuState.mode == BODY_STAND &&
      walkRequested &&
      (manualOwnsMotion || ch_us[SD_CH_INDEX] < SD_ON_THRESHOLD_US) &&
      !inputs.tiltSwitchDown &&
      !tiltModeActive &&
      (walkWasActive || standPoseReady) &&
      !inputs.failsafeActive && inputs.motionCommandsFresh;
  bool walkCandidateFiltered =
      applySwitchDebounce(walkWasActive,
                          walkCandidateRaw,
                          now,
                          &gaitDebounce,
                          kGaitToggleDebounceMs);
  if ((!manualOwnsMotion && ch_us[SD_CH_INDEX] >= SD_ON_THRESHOLD_US) ||
      inputs.tiltSwitchDown || tiltModeActive || inputs.failsafeActive ||
      !inputs.motionCommandsFresh) {
    walkCandidateFiltered = false;
  }
  if (walkCandidateFiltered && menuState.mode == BODY_STAND) {
    menuState.mode = walkRequested
        ? (inputs.gaitRequested ? BODY_GAIT : BODY_CAREFUL) : previousMode;
  }

  // 7) Balance mode: only allowed when standing (not stow, tilt, or gait),
  //    and when the SC switch is in the middle position.
  if (!tiltModeActive && menuState.mode == BODY_STAND && inputs.balanceRequested &&
      !inputs.failsafeActive) {
    menuState.mode = BODY_BALANCE;
  }
  // 8) Simulation (learned policy) mode: SB fully down. It outranks the SC
  //    walk selector but not tilt, failsafe, a stale link or an IK fault. A
  //    policy fault latches until SB is released.
  updatePolicyMode(previousMode, inputs, standPoseReady, now);
  updateMotionInputInterlock(previousMode, menuState.mode, now);
#ifndef DOMINO_SIL
  liveRobotEndpointSetControlState(static_cast<uint8_t>(
      modeTransitionActive() ? BODY_TRANSITION : outputMode), motionInputAwaitingCenter);
#endif
  // Reset balance reference when entering or leaving balance mode.
  if (previousMode != BODY_BALANCE && menuState.mode == BODY_BALANCE) {
    Serial.printf("Entering BODY_BALANCE (SC=%d, imuOk=%d)\n",
                  ch_us[SC_CH_INDEX],
                  (gImuState.online && gImuState.has_sample) ? 1 : 0);
    balanceRefValid = false;
    balanceRollCmdFilt = 0.0f;
    balancePitchCmdFilt = 0.0f;
  } else if (previousMode == BODY_BALANCE && menuState.mode != BODY_BALANCE) {
    Serial.printf("Exiting BODY_BALANCE (newMode=%d)\n", static_cast<int>(menuState.mode));
    balanceRefValid = false;
    balanceRollCmdFilt = 0.0f;
    balancePitchCmdFilt = 0.0f;
  }
  const bool walkingNow = menuState.mode == BODY_GAIT || menuState.mode == BODY_CAREFUL;
  if (!walkWasActive && walkingNow) {
    Serial.printf("Entering walk mode=%d (SC=%d, SD=%d)\n",
                  static_cast<int>(menuState.mode),
                  ch_us[SC_CH_INDEX],
                  ch_us[SD_CH_INDEX]);
  } else if (walkWasActive && !walkingNow) {
    Serial.printf("Exiting walk mode (newMode=%d)\n", static_cast<int>(menuState.mode));
  } else if (walkWasActive && walkingNow && previousMode != menuState.mode) {
    // A different foot schedule requires landing and a neutral handoff.
    Serial.printf("Changing walk mode=%d (SC=%d)\n",
                  static_cast<int>(menuState.mode), ch_us[SC_CH_INDEX]);
  }

  logDebugState(now,
                inputs.haveLink,
                (menuState.mode != BODY_STOW),
                standPoseReady,
                inputs.tiltSwitchDown,
                tiltCandidateRaw,
                tiltCandidateFiltered,
                tiltModeActive);

  if ((now - lastControlMs) >= kControlIntervalMs) {
#ifndef DOMINO_SIL
    ScopedLoopStage controlStage(LoopStage::Control);
#endif
    lastControlMs = now;
#ifndef DOMINO_SIL
    // If anything left the shared I2C bus stuck (SDA held low), free it before
    // this tick's servo writes instead of letting every write time out.
    i2cBusEnsureIdle();
    if (liveRobotEndpointCalibrationOwnsOutputs() ||
        !liveRobotEndpointAllowsRadioControl()) return;
    if (!servoOutputsEnabled()) setServoOutputsEnabled(pca, true);
#endif
    const bool bodyPoseDisplaced = fabsf(outputRollDeg) + fabsf(outputPitchDeg) +
        fabsf(outputYawDeg) + fabsf(outputBodyX) + fabsf(outputBodyY) > 0.05f;
#ifndef DOMINO_SIL
    const bool manualPoseReleased = !activeManualControl.active || !activeManualControl.deadman;
#else
    constexpr bool manualPoseReleased = false;
#endif
    const bool returnManualPose = outputMode == BODY_STAND && bodyPoseDisplaced &&
        (manualPoseReleased || !inputs.motionCommandsFresh);
    if (!modeTransitionActive() && (outputMode != menuState.mode || returnManualPose ||
                                    (motionIkFault && bodyPoseDisplaced))) {
      if (motionInputProfileForMode(outputMode) != MOTION_INPUT_NONE ||
          isWalkingMode(menuState.mode) || outputMode == BODY_BALANCE || bodyPoseDisplaced) {
        beginModeTransition(menuState.mode, menuState.rideHeightMm);
      } else {
        outputMode = menuState.mode;
        if (outputMode == BODY_TILT) {
          tiltRollRamp.go(0.0f);
          tiltPitchRamp.go(0.0f);
          tiltYawRamp.go(0.0f);
        }
      }
    }
    if (applyModeTransition(pca, menuState.mode, menuState.rideHeightMm)) {
      // Keep the height ramp anchored to the pose actually emitted, including
      // the gait's lower body height. Its old stand value cannot leak through
      // on the first frame after a handoff.
      zRamp.go(lastPoseZ);
      sitBlend = 0.0f;
      return;
    }
    const MotionSmoothingSettings &smoothing = motionSmoothingSettings();
    const float rampZ = commandWithMotionSmoothing(
        zRamp, currentTargetZ, smoothing.heightRateMmPerSec, true);
    lastPoseZ = rampZ;
    const float sitBlendTarget = (menuState.mode == BODY_STOW) ? 1.0f : 0.0f;
    const float sitBlendStep = kSitBlendSpeedPerSec * (static_cast<float>(kControlIntervalMs) / 1000.0f);
    if (sitBlend < sitBlendTarget) {
      sitBlend = fminf(sitBlendTarget, sitBlend + sitBlendStep);
    } else if (sitBlend > sitBlendTarget) {
      sitBlend = fmaxf(sitBlendTarget, sitBlend - sitBlendStep);
    }

    if (menuState.mode == BODY_TILT) {
      applyTiltPose(pca);
    } else if (sitBlend > 0.001f) {
#ifdef DOMINO_SIL
      silBodyRollDeg = 0.0f;
      silBodyPitchDeg = 0.0f;
      silBodyYawDeg = 0.0f;
#endif
      moveLegsToSitBlend(pca, rampZ, sitBlend);
    } else if (menuState.mode == BODY_POLICY) {
#ifdef DOMINO_SIL
      silBodyRollDeg = 0.0f;
      silBodyPitchDeg = 0.0f;
      silBodyYawDeg = 0.0f;
#endif
      applyPolicyStep(pca, rampZ, now);
    } else if (menuState.mode == BODY_GAIT || menuState.mode == BODY_CAREFUL) {
#ifdef DOMINO_SIL
      silBodyRollDeg = 0.0f;
      silBodyPitchDeg = 0.0f;
      silBodyYawDeg = 0.0f;
#endif
      if (menuState.mode == BODY_CAREFUL) {
        applyCarefulWalk(pca, rampZ);
      } else {
        applySinusoidalGait(pca, rampZ);
      }
    } else {
#ifdef DOMINO_SIL
      silBodyRollDeg = 0.0f;
      silBodyPitchDeg = 0.0f;
      silBodyYawDeg = 0.0f;
#endif
      // Stand / stow / balance: use the same body-plate model as tilt mode.
      const float bodyZ = rampZ + STAND_HEIGHT_Z;
      float rollDeg = 0.0f;
      float pitchDeg = 0.0f;
      float yawDeg = 0.0f;
      float bodyX = 0.0f;
      float bodyY = 0.0f;
#ifndef DOMINO_SIL
      if (activeManualControl.active && activeManualControl.deadman && !motionIkFault &&
          inputs.motionCommandsFresh && !motionInputAwaitingCenter) {
        // Studio's manual pose fades in on entry, like the radio sticks.
        const float gain = tiltTakeoverGain(now);
        rollDeg = activeManualControl.roll * kMaxRollDeg * gain;
        pitchDeg = activeManualControl.pitch * kManualMaxPitchDeg * gain;
        yawDeg = activeManualControl.yaw * kManualMaxYawDeg * gain;
        bodyX = activeManualControl.bodyX * kManualMaxBodyXMm * gain;
        bodyY = activeManualControl.bodyY * kManualMaxBodyYMm * gain;
      }
#endif
      bool useBalanceZOffsets = false;
      float balanceZOffsets[kLegCount] = {0.0f, 0.0f, 0.0f, 0.0f};

      if (menuState.mode == BODY_BALANCE && !imuSampleFresh(now)) {
        menuState.mode = BODY_STAND;
        balanceRefValid = false;
      }
      if (menuState.mode == BODY_BALANCE && imuSampleFresh(now)) {
        // Map IMU accel (sensor frame) into body frame and estimate roll/pitch
        // from gravity. The IMU board is mounted with a 90-degree yaw offset,
        // so first form an intermediate body-like vector and then rotate it
        // -90 degrees about +Z to align with the body frame.
        const float gx_body_meas = -gImuState.az_g_filt;   // intermediate forward
        const float gy_body_meas =  gImuState.ay_g_filt;   // intermediate left
        const float gz_body_meas = -gImuState.ax_g_filt;   // intermediate up

        // Apply a -90 deg rotation about +Z: x' = y, y' = -x.
        const float gx_body =  gy_body_meas;               // forward (+X_body)
        const float gy_body = -gx_body_meas;               // left    (+Y_body)
        const float gz_body =  gz_body_meas;               // up      (+Z_body)

        const float rollRad = atan2f(gy_body, gz_body);
        const float pitchRad = atan2f(-gx_body, sqrtf(gy_body * gy_body + gz_body * gz_body));

        const float rollDegMeas = rollRad * (180.0f / kPi);
        const float pitchDegMeas = pitchRad * (180.0f / kPi);

        // If the body is tilted beyond the safety threshold, drop back to
        // normal stand (no active balancing) so we don't fight a fall.
        if ((fabsf(rollDegMeas) > kBalanceCancelDeg) ||
            (fabsf(pitchDegMeas) > kBalanceCancelDeg)) {
          if (menuState.mode == BODY_BALANCE) {
            menuState.mode = BODY_STAND;
            Serial.println("Balance mode cancelled: tilt beyond safety threshold.");
          }
        } else {
          // Capture reference at entry so we hold the current body "level"
          // and avoid a sudden snap when balance is enabled.
          if (!balanceRefValid) {
            balanceRefRollDeg = rollDegMeas;
            balanceRefPitchDeg = pitchDegMeas;
            balanceRefValid = true;
          }

          const float rollErr = rollDegMeas - balanceRefRollDeg;
          const float pitchErr = pitchDegMeas - balanceRefPitchDeg;

          // Apply a proportional correction on the deviation from the
          // reference. Use a small deadband to avoid hunting around zero.
          float rollCmd = 0.0f;
          float pitchCmd = 0.0f;
          if (fabsf(rollErr) > kBalanceDeadbandDeg) {
            rollCmd = -kBalanceKpRoll * rollErr;
          }
          if (fabsf(pitchErr) > kBalanceDeadbandDeg) {
            pitchCmd = -kBalanceKpPitch * pitchErr;
          }

          // Smooth the output commands to avoid exciting servo dynamics.
          balanceRollCmdFilt += kBalanceOutputAlpha * (rollCmd - balanceRollCmdFilt);
          balancePitchCmdFilt += kBalanceOutputAlpha * (pitchCmd - balancePitchCmdFilt);

          float rollOutDeg = balanceRollCmdFilt;
          float pitchOutDeg = balancePitchCmdFilt;

          // Clamp to conservative limits so we don't exceed mechanical tilt range.
          rollOutDeg = constrain(rollOutDeg, -kBalanceMaxRollDeg, kBalanceMaxRollDeg);
          pitchOutDeg = constrain(pitchOutDeg, -kBalanceMaxPitchDeg, kBalanceMaxPitchDeg);

          // Map the virtual roll/pitch command into per-leg Z offsets in the
          // world frame using the actual foot XY geometry so the body stays
          // level while feet remain at fixed XY.
          const float rollOutRad = rollOutDeg * kDegToRad;
          const float pitchOutRad = pitchOutDeg * kDegToRad;
          for (int i = 0; i < kLegCount; ++i) {
            const float x = kFootPosXBody[i];
            const float y = kFootPosYBody[i];
            // Small-angle approximation: vertical displacement at (x,y) from a
            // roll about +X and pitch about +Y is z ≈ roll*y - pitch*x.
            float zOffset = kBalanceZGain * (rollOutRad * y - pitchOutRad * x);
            zOffset = constrain(zOffset, -kBalanceMaxZOffsetMm, kBalanceMaxZOffsetMm);
            balanceZOffsets[i] = zOffset;
          }
          useBalanceZOffsets = true;

          // Periodic balance-mode debug log for offline analysis.
          static uint32_t lastBalanceLogMs = 0;
          constexpr uint32_t kBalanceLogIntervalMs = 50;
          if (kBalanceDebugLoggingEnabled && (now - lastBalanceLogMs) >= kBalanceLogIntervalMs) {
            lastBalanceLogMs = now;
            Serial.printf(
                "BAL t=%lu rollMeas=%.2f pitchMeas=%.2f refRoll=%.2f refPitch=%.2f "
                "rollErr=%.2f pitchErr=%.2f rollCmd=%.2f pitchCmd=%.2f rollOut=%.2f "
                "pitchOut=%.2f | ax=%.3f ay=%.3f az=%.3f\n",
                static_cast<unsigned long>(now),
                rollDegMeas,
                pitchDegMeas,
                balanceRefRollDeg,
                balanceRefPitchDeg,
                rollErr,
                pitchErr,
                rollCmd,
                pitchCmd,
                rollOutDeg,
                pitchOutDeg,
                gImuState.ax_g_filt,
                gImuState.ay_g_filt,
                gImuState.az_g_filt);
          }
        }
      }

      if (useBalanceZOffsets) {
        // Keep body orientation level and adjust each leg's Z independently.
        moveLegsFromBodyPoseWithZOffsets(pca, 0.0f, 0.0f, bodyZ, balanceZOffsets);
      } else {
        moveLegsFromBodyPose(pca, bodyX, bodyY, bodyZ, rollDeg, pitchDeg, yawDeg);
      }
    }
  }
}

#ifdef DOMINO_SIL
// Desktop simulation instrumentation. These accessors expose only state that
// the native monitor needs; production ESP32 builds do not contain them.
extern "C" int dominoSilBodyMode() {
  return static_cast<int>(modeTransitionActive() ? BODY_TRANSITION : outputMode);
}

extern "C" int dominoSilRequestedMode() { return static_cast<int>(menuState.mode); }
extern "C" int dominoSilTransitionStage() { return static_cast<int>(modeTransition.stage); }

extern "C" float dominoSilTargetZ() {
  if (modeTransitionActive()) return modeTransition.toHeight;
  return (menuState.mode == BODY_GAIT || menuState.mode == BODY_CAREFUL)
             ? fminf(currentTargetZ, gaitProfile().settings.bodyHeightMm)
             : currentTargetZ;
}

extern "C" float dominoSilPoseZ() {
  return outputHeightMm;
}

extern "C" bool dominoSilTiltActive() {
  return !modeTransitionActive() && outputMode == BODY_TILT;
}

extern "C" float dominoSilRideHeightMm() {
  return currentRideHeightMm;
}

extern "C" bool dominoSilGaitActive() {
  return !modeTransitionActive() && isWalkingMode(outputMode);
}

extern "C" bool dominoSilMotionInputArmed() {
  return !modeTransitionActive() && !motionInputAwaitingCenter;
}

extern "C" float dominoSilGaitPhaseRad() {
  return gaitPhaseRad;
}

extern "C" float dominoSilGaitForwardCommand() {
  return gaitForwardCommand;
}

extern "C" float dominoSilGaitTurnCommand() {
  return gaitTurnCommand;
}

extern "C" float dominoSilLegCommandX(int legIndex) {
  return (legIndex >= 0 && legIndex < kLegCount) ? silLegCommandX[legIndex] : 0.0f;
}

extern "C" float dominoSilLegCommandY(int legIndex) {
  return (legIndex >= 0 && legIndex < kLegCount) ? silLegCommandY[legIndex] : 0.0f;
}

extern "C" float dominoSilLegCommandZ(int legIndex) {
  return (legIndex >= 0 && legIndex < kLegCount) ? silLegCommandZ[legIndex] : 0.0f;
}

extern "C" float dominoSilBodyRollDeg() {
  return outputRollDeg;
}

extern "C" float dominoSilBodyPitchDeg() {
  return outputPitchDeg;
}

extern "C" float dominoSilBodyYawDeg() {
  return outputYawDeg;
}
#endif
