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
// The Boxer left-stick vertical is CRSF channel 3. The firmware uses its
// three travel bands as the high / medium / low ride-height command.
constexpr int RIDE_HEIGHT_CH_INDEX = 2;
constexpr int SD_CH_INDEX = 7;
constexpr int SD_ON_THRESHOLD_US = 1600;
constexpr int SD_OFF_THRESHOLD_US = 1400;
constexpr float kStickCenterUs = 1500.0f;
constexpr float kStickHalfRangeUs = 500.0f;

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

// Ride height configuration (stand-only, 3 presets driven by SB).
// "High" is the default neutral stand height; "Medium" and "Low"
// bring the feet closer to the body (smaller Z in IK frame).
enum RideHeightPreset : uint8_t { RIDE_HIGH = 0, RIDE_MEDIUM = 1, RIDE_LOW = 2 };

// High-level body mode. This is the top-level "menu" state that we will
// extend as more behaviors are added (gaits, diagnostics, etc.).
enum BodyMode : uint8_t {
  BODY_STOW = 0,
  BODY_STAND = 1,
  BODY_TILT = 2,
  BODY_BALANCE = 3,
};

// Microsecond bands for SB positions (up / mid / down).
// These values assume ~1000 us = switch fully up, ~1500 us = middle,
// ~2000 us = fully down. Tweak if your radio outputs different ranges.
constexpr int RIDE_HEIGHT_UPPER_POS_MAX_US = 1200;
constexpr int RIDE_HEIGHT_LOWER_POS_MIN_US = 1800;

// Offsets from kNeutralZ (mm) for each ride height preset.
// Positive values move the feet further from the body; negative values
// move them closer. Keep results >= 0 to stay within IK assumptions.
constexpr float kRideHeightOffsetHighMm = 0.0f;     // SB up  : default stand height.
constexpr float kRideHeightOffsetLowMm = -60.0f;    // SB down: lowest CAD-validated linkage reach.
constexpr float kRideHeightOffsetMediumMm =
    0.5f * (kRideHeightOffsetHighMm + kRideHeightOffsetLowMm);  // SB mid : halfway between.

constexpr float kRideHeightOffsetsMm[3] = {
    kRideHeightOffsetHighMm,
    kRideHeightOffsetMediumMm,
    kRideHeightOffsetLowMm,
};

static_assert((kNeutralZ + kRideHeightOffsetLowMm) >= 220.0f,
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
RideHeightPreset currentRidePreset = RIDE_HIGH;

// Unified "menu" view of the robot's state based purely on RC inputs and
// link/failsafe information. The control loop consumes this instead of
// talking directly to raw switches.
struct MenuState {
  BodyMode mode;
  RideHeightPreset rideHeight;
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

float normalizeChannel(int chValueUs) {
  const float normalized = (static_cast<float>(chValueUs) - kStickCenterUs) / kStickHalfRangeUs;
  return constrain(normalized, -1.0f, 1.0f);
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

RideHeightPreset detectRideHeightPresetFromChannelUs(int heightValueUs) {
  if (heightValueUs <= RIDE_HEIGHT_UPPER_POS_MAX_US) {
    return RIDE_HIGH;
  }
  if (heightValueUs >= RIDE_HEIGHT_LOWER_POS_MIN_US) {
    return RIDE_LOW;
  }
  return RIDE_MEDIUM;
}

RideHeightPreset readRideHeightPreset() {
  const int heightValueUs = ch_us[RIDE_HEIGHT_CH_INDEX];
  return detectRideHeightPresetFromChannelUs(heightValueUs);
}

float computeStandTargetZForPreset(RideHeightPreset preset) {
  const int index = static_cast<int>(preset);
  const float offset = kRideHeightOffsetsMm[index];
  const float candidate = kNeutralZ + offset;
  return (candidate >= 0.0f) ? candidate : 0.0f;
}

// Raw RC -> menu inputs -------------------------------------------------

struct MenuInputs {
  bool haveLink = false;
  bool failsafeActive = false;
  bool standRequested = false;
  bool tiltSwitchDown = false;
  bool balanceRequested = false;
  RideHeightPreset rideRequested = RIDE_HIGH;
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

  // Balance request from SC (middle position only).
  const int scValueUs = ch_us[SC_CH_INDEX];
  // Assume 3-position: ~1000 (up), ~1500 (mid), ~2000 (down).
  // Treat 1400-1600 us as the middle position.
  inputs.balanceRequested = (scValueUs > 1400 && scValueUs < 1600);
  static bool lastBalanceRequested = false;
  if (inputs.balanceRequested != lastBalanceRequested) {
    lastBalanceRequested = inputs.balanceRequested;
    Serial.printf("SC=%d -> balanceRequested=%d\n", scValueUs, inputs.balanceRequested ? 1 : 0);
  }

  // Ride height preset from Boxer left-stick vertical / CRSF CH3. The
  // firmware intentionally keeps the existing three preset bands so the
  // same command is deterministic on the physical robot and in SIL.
  inputs.rideRequested = readRideHeightPreset();

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
  const float yawNorm = normalizeChannel(ch_us[YAW_CH_INDEX]);
  float rollNorm = normalizeChannel(ch_us[ROLL_CH_INDEX]);
  float pitchNorm = normalizeChannel(ch_us[PITCH_CH_INDEX]);
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
MenuState menuState{BODY_STOW, RIDE_HIGH, false, true};

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

  // 3) Always remember and report SB. Stow still uses its fixed compact pose,
  //    but the selected ride height is retained for the next stand command.
  if (inputs.rideRequested != currentRidePreset) {
    currentRidePreset = inputs.rideRequested;
    Serial.printf("Ride height preset=%d CH3_HEIGHT=%d\n",
                  static_cast<int>(currentRidePreset),
                  ch_us[RIDE_HEIGHT_CH_INDEX]);
  }
  menuState.rideHeight = currentRidePreset;

  // 4) Compute the current Z target from the menu state.
  if (menuState.mode == BODY_STOW) {
    currentTargetZ = kCloserPoseZ;
  } else {
    currentTargetZ = computeStandTargetZForPreset(menuState.rideHeight);
  }
  zRamp.move(currentTargetZ);

  // Maintain legacy standState for any existing debug/logic that still uses it.
  standState = (menuState.mode == BODY_STOW) ? STOW : STAND;

  // 5) Tilt candidate: entering tilt requires the pose to be "ready" at the
  //    chosen height, but once in tilt we keep it latched as long as the
  //    stand command and tilt switch remain active. This avoids dropping out
  //    of tilt while ride height ramps between presets.
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
    // Entering tilt: require pose ready at the selected ride height so we
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

  // 6) Balance mode: only allowed when standing (not stow, not tilt),
  //    and when the SC switch is in the middle position.
  if (!tiltModeActive && menuState.mode == BODY_STAND && inputs.balanceRequested &&
      !inputs.failsafeActive) {
    menuState.mode = BODY_BALANCE;
  }
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
  return currentTargetZ;
}

extern "C" float dominoSilPoseZ() {
  return lastPoseZ;
}

extern "C" bool dominoSilTiltActive() {
  return tiltModeActive;
}

extern "C" int dominoSilRideHeight() {
  return static_cast<int>(menuState.rideHeight);
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
