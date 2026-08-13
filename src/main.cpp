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
// Feet are placed roughly FOOT_OUT_OFFSET_Y laterally and FOOT_BACK_OFFSET_X
// behind the hip in X, with STAND_HEIGHT_Z as the nominal ground height.
//
// Important hardware detail: all hip servos are oriented the same way in
// world space. This is reflected in leg_controller.cpp where hipDir = +1
// for every leg; left legs mirror only the upper/lower joints.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ramp.h>
#include <math.h>

#include "crsf.h"
#include "leg_controller.h"
#include "imu.h"

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
constexpr float FOOT_BACK_OFFSET_X = -15.75f; // foot is slightly behind the hip in X
constexpr float FOOT_OUT_OFFSET_Y = 100.0f;   // lateral distance from body center line
constexpr float STAND_HEIGHT_Z = -280.0f;     // nominal foot Z in world frame (down)
// The command-space neutral above comes from the original virtual leg. The
// authored CAD foot is physically ahead of the hip and slightly closer to it.
// Body rotations must use these real offsets, then map the result back into
// the legacy command frame so the level-stand servo pulse remains unchanged.
constexpr float CAD_NEUTRAL_FOOT_X_FROM_HIP = 2.541356f;
constexpr float CAD_NEUTRAL_FOOT_Y_FROM_HIP = 38.1f;
constexpr float CAD_NEUTRAL_FOOT_Z_FROM_HIP = -275.984374f;
constexpr float COMMAND_NEUTRAL_FOOT_Y_FROM_HIP = 38.0f;
constexpr float kStandPoseToleranceMm = 5.0f;

namespace {
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
constexpr float kRampSpeedMmPerSec = 120.0f;

constexpr float kCloserPoseZ = 160.0f;

// STOW is a deep sit rather than the old 235 mm shallow crouch. Keeping X at
// the standing footprint holds the feet beneath their hip pivots instead of
// sweeping them out from under the body. All four legs use the same validated
// fold depth. The rendered CAD closure solve keeps the feet beneath the hips
// at 160 mm while all driven joints remain inside +/-45 degrees.
constexpr float kSitFrontX = FOOT_BACK_OFFSET_X;
constexpr float kSitRearX = FOOT_BACK_OFFSET_X;
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
// vertical commands travel; right-stick horizontal commands differential
// stride for turning.
constexpr float kGaitStickDeadband = 0.12f;
constexpr float kGaitMaxStrideMm = 33.0f;
constexpr float kGaitMaxTurnStrideMm = 22.0f;
constexpr float kGaitMaxLiftMm = 30.0f;
constexpr float kGaitMinFrequencyHz = 0.54f;
constexpr float kGaitMaxFrequencyHz = 0.76f;
constexpr float kGaitStanceFraction = 0.68f;
constexpr float kGaitCommandSlewPerSec = 1.5f;
constexpr float kGaitAmplitudeSlewPerSec = 2.0f;
constexpr float kGaitBodyHeightMm = 265.0f;
constexpr float kGaitBodyHeightSlewMmPerSec = 80.0f;
constexpr uint32_t kGaitToggleDebounceMs = 200;

// SC middle selects a statically stable four-beat crawl. Each leg owns one
// quarter of the cycle, but swings for less than that quarter, guaranteeing
// at least three planted feet and a short four-foot settle between steps.
constexpr float kCarefulMaxStrideMm = 22.0f;
constexpr float kCarefulMaxTurnStrideMm = 14.0f;
constexpr float kCarefulMaxLiftMm = 20.0f;
constexpr float kCarefulMinFrequencyHz = 0.30f;
constexpr float kCarefulMaxFrequencyHz = 0.42f;
constexpr float kCarefulSwingFraction = 0.19f;

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
};

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
#ifdef DOMINO_SIL
  silLegCommandX[legIndex] = x;
  silLegCommandY[legIndex] = y;
  silLegCommandZ[legIndex] = z;
#endif
  kMoveLegFns[legIndex](driver, x, y, z);
}

// Nominal foot positions in the body/world frame when the body is level.
// Used by the balance controller to compute per-leg Z corrections that
// keep the body plane level while feet remain at fixed XY.
constexpr float kFootPosXBody[kLegCount] = {
    +BODY_HALF_LENGTH_X + FOOT_BACK_OFFSET_X,  // FL
    +BODY_HALF_LENGTH_X + FOOT_BACK_OFFSET_X,  // FR
    -BODY_HALF_LENGTH_X + FOOT_BACK_OFFSET_X,  // BL
    -BODY_HALF_LENGTH_X + FOOT_BACK_OFFSET_X   // BR
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
float gaitBodyZ = kGaitBodyHeightMm;
bool motionInputAwaitingCenter = false;
uint32_t motionInputCenteredSinceMs = 0;

float normalizeChannel(int chValueUs) {
  const float normalized = (static_cast<float>(chValueUs) - kStickCenterUs) / kStickHalfRangeUs;
  return constrain(normalized, -1.0f, 1.0f);
}

MotionInputProfile motionInputProfileForMode(BodyMode mode) {
  if (mode == BODY_TILT) {
    return MOTION_INPUT_TILT;
  }
  if (mode == BODY_GAIT || mode == BODY_CAREFUL) {
    return MOTION_INPUT_WALK;
  }
  return MOTION_INPUT_NONE;
}

bool motionInputsAreCentered() {
  return fabsf(normalizeChannel(ch_us[ROLL_CH_INDEX])) <= kModeInputCenterDeadband &&
         fabsf(normalizeChannel(ch_us[PITCH_CH_INDEX])) <= kModeInputCenterDeadband &&
         fabsf(normalizeChannel(ch_us[YAW_CH_INDEX])) <= kModeInputCenterDeadband;
}

void updateMotionInputInterlock(BodyMode previousMode, BodyMode nextMode, uint32_t now) {
  const MotionInputProfile previousProfile = motionInputProfileForMode(previousMode);
  const MotionInputProfile nextProfile = motionInputProfileForMode(nextMode);
  if (previousProfile != nextProfile) {
    motionInputAwaitingCenter = nextProfile != MOTION_INPUT_NONE;
    motionInputCenteredSinceMs = 0;
  }

  if (!motionInputAwaitingCenter) {
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
  return motionInputAwaitingCenter ? 0.0f : normalizeChannel(ch_us[channelIndex]);
}

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
  const float yawDeg = yawNorm * kMaxYawDeg;

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
  bool failsafeActive = false;
  bool standRequested = false;
  bool tiltSwitchDown = false;
  bool balanceRequested = false;
  bool gaitRequested = false;
  bool carefulWalkRequested = false;
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

  // Continuous ride height from Boxer left-stick vertical / CRSF CH3.
  inputs.rideHeightMm = readRideHeightMm();

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

  *outX =
      legX_local +
      (FOOT_BACK_OFFSET_X - CAD_NEUTRAL_FOOT_X_FROM_HIP);
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
    commandLeg(driver, i, xLeg, yLeg, zLeg);
  }
}

void moveLegsToSitBlend(Adafruit_PWMServoDriver &driver, float commonZ, float blend) {
  const float t = constrain(blend, 0.0f, 1.0f);
  for (int i = 0; i < kLegCount; ++i) {
    const bool frontLeg = (i == LEG_FL || i == LEG_FR);
    const bool leftLeg = (i == LEG_FL || i == LEG_BL);
    const float sitX = frontLeg ? kSitFrontX : kSitRearX;
    const float sitZ = frontLeg ? kSitFrontZ : kSitRearZ;
    const float x = FOOT_BACK_OFFSET_X + t * (sitX - FOOT_BACK_OFFSET_X);
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

float approachGaitCommand(float current, float target, float maximumStep) {
  if (target > current) {
    return fminf(target, current + maximumStep);
  }
  return fmaxf(target, current - maximumStep);
}

void resetGaitState() {
  gaitPhaseRad = 0.0f;
  gaitForwardCommand = 0.0f;
  gaitTurnCommand = 0.0f;
  gaitAmplitude = 0.0f;
  gaitBodyZ = lastPoseZ;
}

float halfCosine01(float value) {
  const float t = constrain(value, 0.0f, 1.0f);
  return 0.5f - 0.5f * cosf(kPi * t);
}

void sampleGaitFootPath(float cycle,
                        float halfStrideMm,
                        float liftMm,
                        float *xOffsetMm,
                        float *zOffsetMm) {
  cycle -= floorf(cycle);
  if (cycle < kGaitStanceFraction) {
    // A planted foot must move rearward at constant speed relative to the
    // body. The old half-cosine repeatedly accelerated and decelerated the
    // loaded foot across the floor, which produced visible scrub and sideways
    // reaction forces even for a steady forward command.
    const float stance = cycle / kGaitStanceFraction;
    *xOffsetMm = halfStrideMm * (1.0f - 2.0f * stance);
    *zOffsetMm = 0.0f;
    return;
  }

  // Return the unloaded foot from rear to front. The smooth swing curve keeps
  // touchdown and lift-off gentle; only the unloaded leg uses this variable
  // speed profile.
  const float swing = (cycle - kGaitStanceFraction) / (1.0f - kGaitStanceFraction);
  const float swingPosition = halfCosine01(swing);
  const float liftWave = sinf(kPi * swing);
  *xOffsetMm = halfStrideMm * (-1.0f + 2.0f * swingPosition);
  *zOffsetMm = -liftMm * liftWave * liftWave;
}

void applySinusoidalGait(Adafruit_PWMServoDriver &driver, float baseZ) {
  constexpr float kControlStepSeconds = static_cast<float>(kControlIntervalMs) / 1000.0f;
  const float gaitBodyTargetZ = fminf(baseZ, kGaitBodyHeightMm);
  gaitBodyZ = approachGaitCommand(
      gaitBodyZ,
      gaitBodyTargetZ,
      kGaitBodyHeightSlewMmPerSec * kControlStepSeconds);
  const float targetForward = applyGaitDeadband(readMotionChannelNormalized(PITCH_CH_INDEX));
  const float targetTurn = applyGaitDeadband(readMotionChannelNormalized(YAW_CH_INDEX));
  const float maximumCommandStep = kGaitCommandSlewPerSec * kControlStepSeconds;
  gaitForwardCommand = approachGaitCommand(gaitForwardCommand, targetForward, maximumCommandStep);
  gaitTurnCommand = approachGaitCommand(gaitTurnCommand, targetTurn, maximumCommandStep);

  const float activity = constrain(
      sqrtf(gaitForwardCommand * gaitForwardCommand +
            gaitTurnCommand * gaitTurnCommand),
      0.0f,
      1.0f);
  const float startupBlend = halfCosine01(activity / 0.25f);
  const float targetAmplitude =
      startupBlend * (0.65f + 0.35f * activity);
  gaitAmplitude = approachGaitCommand(
      gaitAmplitude,
      targetAmplitude,
      kGaitAmplitudeSlewPerSec * kControlStepSeconds);
  if (activity > 0.001f) {
    const float frequencyHz =
        kGaitMinFrequencyHz + activity * (kGaitMaxFrequencyHz - kGaitMinFrequencyHz);
    gaitPhaseRad = fmodf(gaitPhaseRad + 2.0f * kPi * frequencyHz * kControlStepSeconds,
                         2.0f * kPi);
  } else {
    gaitPhaseRad = 0.0f;
  }

  const float normalizedCycle = gaitPhaseRad / (2.0f * kPi);
  const float commandScale = activity > 0.001f ? 1.0f / activity : 0.0f;
  const float forwardDirection = gaitForwardCommand * commandScale;
  const float turnDirection = gaitTurnCommand * commandScale;
  for (int i = 0; i < kLegCount; ++i) {
    const bool diagonalA = (i == LEG_FL || i == LEG_BR);
    const bool leftLeg = (i == LEG_FL || i == LEG_BL);
    const float sideSign = leftLeg ? 1.0f : -1.0f;
    const float halfStrideMm = gaitAmplitude * (
        forwardDirection * kGaitMaxStrideMm +
        sideSign * turnDirection * kGaitMaxTurnStrideMm);
    float xOffsetMm = 0.0f;
    float zOffsetMm = 0.0f;
    sampleGaitFootPath(normalizedCycle + (diagonalA ? 0.0f : 0.5f),
                       halfStrideMm,
                       kGaitMaxLiftMm * gaitAmplitude,
                       &xOffsetMm,
                       &zOffsetMm);
    const float x = FOOT_BACK_OFFSET_X + xOffsetMm;
    const float y = sideSign * (FOOT_OUT_OFFSET_Y - BODY_HALF_WIDTH_Y);
    const float z = gaitBodyZ + zOffsetMm;
    commandLeg(driver, i, x, y, z);
  }
}

void applyCarefulWalk(Adafruit_PWMServoDriver &driver, float baseZ) {
  constexpr float kControlStepSeconds = static_cast<float>(kControlIntervalMs) / 1000.0f;
  const float gaitBodyTargetZ = fminf(baseZ, kGaitBodyHeightMm);
  gaitBodyZ = approachGaitCommand(
      gaitBodyZ, gaitBodyTargetZ, kGaitBodyHeightSlewMmPerSec * kControlStepSeconds);
  const float targetForward = applyGaitDeadband(readMotionChannelNormalized(PITCH_CH_INDEX));
  const float targetTurn = applyGaitDeadband(readMotionChannelNormalized(YAW_CH_INDEX));
  const float maximumCommandStep = kGaitCommandSlewPerSec * kControlStepSeconds;
  gaitForwardCommand = approachGaitCommand(gaitForwardCommand, targetForward, maximumCommandStep);
  gaitTurnCommand = approachGaitCommand(gaitTurnCommand, targetTurn, maximumCommandStep);

  const float activity = constrain(
      sqrtf(gaitForwardCommand * gaitForwardCommand + gaitTurnCommand * gaitTurnCommand),
      0.0f, 1.0f);
  const float targetAmplitude = halfCosine01(activity / 0.20f) * (0.72f + 0.28f * activity);
  gaitAmplitude = approachGaitCommand(
      gaitAmplitude, targetAmplitude, kGaitAmplitudeSlewPerSec * kControlStepSeconds);
  if (activity > 0.001f) {
    const float frequencyHz =
        kCarefulMinFrequencyHz + activity * (kCarefulMaxFrequencyHz - kCarefulMinFrequencyHz);
    gaitPhaseRad = fmodf(
        gaitPhaseRad + 2.0f * kPi * frequencyHz * kControlStepSeconds, 2.0f * kPi);
  } else {
    gaitPhaseRad = 0.0f;
  }

  const float normalizedCycle = gaitPhaseRad / (2.0f * kPi);
  const float commandScale = activity > 0.001f ? 1.0f / activity : 0.0f;
  const float forwardDirection = gaitForwardCommand * commandScale;
  const float turnDirection = gaitTurnCommand * commandScale;
  constexpr LegIndex kSwingOrder[kLegCount] = {LEG_FL, LEG_BR, LEG_FR, LEG_BL};
  for (int orderIndex = 0; orderIndex < kLegCount; ++orderIndex) {
    const LegIndex leg = kSwingOrder[orderIndex];
    const bool leftLeg = leg == LEG_FL || leg == LEG_BL;
    const float sideSign = leftLeg ? 1.0f : -1.0f;
    const float halfStrideMm = gaitAmplitude * (
        forwardDirection * kCarefulMaxStrideMm +
        sideSign * turnDirection * kCarefulMaxTurnStrideMm);
    float legCycle = normalizedCycle - 0.25f * static_cast<float>(orderIndex);
    legCycle -= floorf(legCycle);
    float xOffsetMm = 0.0f;
    float zOffsetMm = 0.0f;
    if (legCycle < kCarefulSwingFraction) {
      const float swing = legCycle / kCarefulSwingFraction;
      xOffsetMm = halfStrideMm * (-1.0f + 2.0f * halfCosine01(swing));
      const float liftWave = sinf(kPi * swing);
      zOffsetMm = -kCarefulMaxLiftMm * gaitAmplitude * liftWave * liftWave;
    } else {
      const float stance = (legCycle - kCarefulSwingFraction) / (1.0f - kCarefulSwingFraction);
      xOffsetMm = halfStrideMm * (1.0f - 2.0f * stance);
    }
    commandLeg(driver,
               leg,
               FOOT_BACK_OFFSET_X + xOffsetMm,
               sideSign * (FOOT_OUT_OFFSET_Y - BODY_HALF_WIDTH_Y),
               gaitBodyZ + zOffsetMm);
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
  // pitch: right stick Y (future; currently unused in the radio mapping)
  // yaw: left stick X (twist the body about Z)
  const float yawNorm = readMotionChannelNormalized(YAW_CH_INDEX);
  float rollNorm = readMotionChannelNormalized(ROLL_CH_INDEX);
  float pitchNorm = readMotionChannelNormalized(PITCH_CH_INDEX);
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

  const float rollDeg = rollNorm * kMaxRollDeg;
  const float pitchDeg = pitchNorm * kMaxPitchDeg;
  const float yawDeg = yawNormLimited * kMaxYawDeg;

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
  tiltModeActive = true;
  Serial.printf("Tilt mode enabled (SA=%d SD=%d)\n", ch_us[SA_CH_INDEX], ch_us[SD_CH_INDEX]);
}

void exitTiltMode() {
  tiltModeActive = false;
  Serial.printf("Tilt mode disabled (SA=%d SD=%d)\n", ch_us[SA_CH_INDEX], ch_us[SD_CH_INDEX]);
}
}  // namespace

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();
rampFloat zRamp;
float currentTargetZ = kCloserPoseZ;
MenuState menuState{BODY_STOW, kRideHeightMaxMm, false, true};

void setup() {
  Serial.begin(115200);
  Wire.begin();

  imuInit();

  Serial2.begin(CRSF_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  initCrsfState();

  pca.begin();
  pca.setOscillatorFrequency(kOscillatorFreqHz);
  pca.setPWMFreq(kServoPwmFreqHz);
  delay(10);

  zRamp.setSpeed(kRampSpeedMmPerSec);
  zRamp.go(kCloserPoseZ);
  lastPoseZ = kCloserPoseZ;
  sitBlend = 1.0f;
  moveLegsToSitBlend(pca, lastPoseZ, sitBlend);
  standState = STOW;
  Serial.println("All legs initialized at stow pose (waiting for CRSF link).");
  Serial.println("Debug logging ready.");
  logDebugState(millis(), false, false, false, false, false, false, tiltModeActive);
}

void loop() {
  static uint32_t lastControlMs = 0;
  static uint32_t lastLinkAliveMs = 0;
  static bool failsafeActive = true;
  static SwitchDebounceState standModeDebounce;
  static SwitchDebounceState tiltDebounce;
  static SwitchDebounceState gaitDebounce;

  const uint32_t now = millis();
  processCrsfFrames(now);
  // Refresh IMU sample; failures simply leave the previous values.
  (void)imuReadSample();

  // 1) Read current RC "menu inputs" (switches, link status).
  const MenuInputs inputs = readMenuInputs(now, &lastLinkAliveMs, &failsafeActive);
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
  const bool tiltCandidateFiltered =
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
  const bool walkCandidateRaw =
      menuState.mode == BODY_STAND &&
      walkRequested &&
      ch_us[SD_CH_INDEX] < SD_ON_THRESHOLD_US &&
      !inputs.tiltSwitchDown &&
      !tiltModeActive &&
      (walkWasActive || standPoseReady) &&
      !inputs.failsafeActive;
  bool walkCandidateFiltered =
      applySwitchDebounce(walkWasActive,
                          walkCandidateRaw,
                          now,
                          &gaitDebounce,
                          kGaitToggleDebounceMs);
  if (ch_us[SD_CH_INDEX] >= SD_ON_THRESHOLD_US ||
      inputs.tiltSwitchDown || tiltModeActive || inputs.failsafeActive) {
    walkCandidateFiltered = false;
  }
  if (walkCandidateFiltered && menuState.mode == BODY_STAND) {
    menuState.mode = inputs.gaitRequested ? BODY_GAIT : BODY_CAREFUL;
  }

  // 7) Balance mode: only allowed when standing (not stow, tilt, or gait),
  //    and when the SC switch is in the middle position.
  if (!tiltModeActive && menuState.mode == BODY_STAND && inputs.balanceRequested &&
      !inputs.failsafeActive) {
    menuState.mode = BODY_BALANCE;
  }
  updateMotionInputInterlock(previousMode, menuState.mode, now);
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
    resetGaitState();
    Serial.printf("Entering walk mode=%d (SC=%d, SD=%d)\n",
                  static_cast<int>(menuState.mode),
                  ch_us[SC_CH_INDEX],
                  ch_us[SD_CH_INDEX]);
  } else if (walkWasActive && !walkingNow) {
    resetGaitState();
    Serial.printf("Exiting walk mode (newMode=%d)\n", static_cast<int>(menuState.mode));
  } else if (walkWasActive && walkingNow && previousMode != menuState.mode) {
    // Careful and trot use the same stick semantics. Preserve phase and
    // filtered commands so moving SC through the two walk positions does not
    // stop and restart the legs mid-stride.
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
    lastControlMs = now;
    const float rampZ = zRamp.update();
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
      bool useBalanceZOffsets = false;
      float balanceZOffsets[kLegCount] = {0.0f, 0.0f, 0.0f, 0.0f};

      if (menuState.mode == BODY_BALANCE && gImuState.online && gImuState.has_sample) {
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
        moveLegsFromBodyPose(pca, 0.0f, 0.0f, bodyZ, rollDeg, pitchDeg, 0.0f);
      }
    }
  }
}

#ifdef DOMINO_SIL
// Desktop simulation instrumentation. These accessors expose only state that
// the native monitor needs; production ESP32 builds do not contain them.
extern "C" int dominoSilBodyMode() {
  return static_cast<int>(menuState.mode);
}

extern "C" float dominoSilTargetZ() {
  return (menuState.mode == BODY_GAIT || menuState.mode == BODY_CAREFUL)
             ? fminf(currentTargetZ, kGaitBodyHeightMm)
             : currentTargetZ;
}

extern "C" float dominoSilPoseZ() {
  return (menuState.mode == BODY_GAIT || menuState.mode == BODY_CAREFUL) ? gaitBodyZ : lastPoseZ;
}

extern "C" bool dominoSilTiltActive() {
  return tiltModeActive;
}

extern "C" float dominoSilRideHeightMm() {
  return currentRideHeightMm;
}

extern "C" bool dominoSilGaitActive() {
  return menuState.mode == BODY_GAIT || menuState.mode == BODY_CAREFUL;
}

extern "C" bool dominoSilMotionInputArmed() {
  return !motionInputAwaitingCenter;
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
  return silBodyRollDeg;
}

extern "C" float dominoSilBodyPitchDeg() {
  return silBodyPitchDeg;
}

extern "C" float dominoSilBodyYawDeg() {
  return silBodyYawDeg;
}
#endif
