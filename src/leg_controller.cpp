#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#include "leg_controller.h"

#include "ik.h"
#include "servo_calibration.h"

namespace {
constexpr float kServoCenterDeg = 135.0f;
// IK theta2/theta3 describe the virtual 160/153 mm leg, while Domino's two
// physical servo arms drive the CAD four-bar linkage. These neutral-preserving
// gains were solved against the rendered STL hole centers and foot endpoint.
// The exported closure mesh orders its two holes opposite to the URDF's
// parent/child naming, so calibration must follow the physical mesh pivots.
// Keeping the neutral terms explicit means the proven 280 mm stand pulse is
// unchanged while the full 160 mm sit remains within +/-45 degrees.
constexpr float kNeutralTheta2Deg = -36.628307f;
constexpr float kNeutralTheta3Deg = 31.397787f;
constexpr float kUpperCadDriveGain = 0.66f;
constexpr float kLowerCadDriveGain = 1.32f;
constexpr float kNeutralCommandXmm = -15.75f;
constexpr float kNeutralCommandYmm = 38.0f;
constexpr float kNeutralCommandZmm = 280.0f;
constexpr float kCadNeutralFootYmm = 38.1f;
constexpr float kMaxMechanicalDeltaDeg = 45.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kMinPulseUs = 500.0f;
constexpr float kMaxPulseUs = 2500.0f;
constexpr float kMaxAngleDeg = 270.0f;
constexpr uint8_t kPcaChannelCount = 16;
constexpr uint8_t kLegCount = 4;

struct Point2 {
  float x;
  float z;
};

struct CadGeometry {
  Point2 lowerDrive;
  Point2 upperDrive;
  Point2 lowerPassive;
  Point2 lowerClosureDriver;
  Point2 upperClosureDriver;
  Point2 upperClosureCoupler;
  Point2 lowerCoupler;
  Point2 lowerClosureDiagonal;
  Point2 foot;
  float neutralLowerOffsetDeg;
  Point2 neutralFoot;
};

struct CadPose {
  Point2 lowerPassive;
  Point2 lowerClosureDriver;
  Point2 upperClosureDriver;
  Point2 upperClosureCoupler;
  Point2 lowerCoupler;
  Point2 lowerClosureDiagonal;
};

// Local X/Z hole centres measured from the authored right and left CAD legs.
// Front and rear mechanisms are translations of the same side-specific linkage.
constexpr CadGeometry kRightCad{
    {56.5f, -21.0f},
    {80.5f, 0.0f},
    {56.5f, -21.0f},
    {-56.185f, -134.392f},
    {80.5f, 40.0f},
    {56.487f, 19.0f},
    {36.489f, 13.635f},
    {-76.196f, -99.757f},
    {2.545f, -275.993f},
    0.0f,
    {2.545f, -275.993f},
};

constexpr CadGeometry kLeftCad{
    {56.5f, -21.0f},
    {80.5f, 0.0f},
    {56.5f, -21.0f},
    {-64.157f, -125.867f},
    {80.5f, 40.0f},
    {56.487f, 19.0f},
    {36.489f, 13.635f},
    {-84.168f, -91.232f},
    {-5.427f, -267.460f},
    4.18f,
    {2.537712f, -275.975748f},
};

float gCadUpperSeedDeg[kLegCount] = {};
float gCadLowerSeedDeg[kLegCount] = {};
float gCommandedServoAnglesDeg[kPcaChannelCount] = {};
uint16_t gCommandedServoPulseUs[kPcaChannelCount] = {};
bool gServoOutputsEnabled = false;
uint32_t gServoSafetyClipCount = 0;
ServoCalibrationProfile gServoCalibration = defaultServoCalibrationProfile();

struct ServoAngleLimit {
  float minDeg;
  float maxDeg;
};

// Per-channel hard safety limits. These were seeded from the current
// stow/stand/tilt/balance pose envelope plus margin, so a bad mode transition
// cannot command the full 0-270 degree electrical range on startup or during
// operation. Tighten these during calibration once each joint's true mechanical
// limits are known.
constexpr ServoAngleLimit kServoAngleLimits[kPcaChannelCount] = {
    {87.0f, 156.0f},   // CH0  FL hip
    {109.0f, 254.0f},  // CH1  FL upper
    {29.0f, 171.0f},   // CH2  FL lower
    {64.0f, 133.0f},   // CH3  FR hip
    {0.0f, 141.0f},    // CH4  FR upper
    {0.0f, 270.0f},    // CH5  unused
    {0.0f, 270.0f},    // CH6  unused
    {96.0f, 234.0f},   // CH7  BL upper
    {23.0f, 155.0f},   // CH8  BL lower
    {74.0f, 143.0f},   // CH9  BR hip
    {0.0f, 134.0f},    // CH10 BR upper
    {80.0f, 212.0f},   // CH11 BR lower
    {0.0f, 270.0f},    // CH12 unused
    {0.0f, 270.0f},    // CH13 unused
    {77.0f, 146.0f},   // CH14 BL hip
    {71.0f, 213.0f},   // CH15 FR lower
};

float clamp270(float angle) { return constrain(angle, 0.0f, kMaxAngleDeg); }

float distance2(Point2 a, Point2 b) {
  return hypotf(b.x - a.x, b.z - a.z);
}

float orientation(Point2 a, Point2 b, Point2 point) {
  return (b.x - a.x) * (point.z - a.z) -
         (b.z - a.z) * (point.x - a.x);
}

Point2 rotate2(Point2 point, Point2 pivot, float angle) {
  const float x = point.x - pivot.x;
  const float z = point.z - pivot.z;
  const float cosine = cosf(angle);
  const float sine = sinf(angle);
  return {
      pivot.x + cosine * x - sine * z,
      pivot.z + sine * x + cosine * z,
  };
}

bool circleIntersection(Point2 centerA,
                        float radiusA,
                        Point2 centerB,
                        float radiusB,
                        Point2 preferred,
                        float assemblyOrientation,
                        Point2 *result) {
  const float dx = centerB.x - centerA.x;
  const float dz = centerB.z - centerA.z;
  const float centerDistance = hypotf(dx, dz);
  const float minimumDistance = fabsf(radiusA - radiusB);
  const float maximumDistance = radiusA + radiusB;
  if (centerDistance < 1e-5f ||
      centerDistance < minimumDistance - 1e-4f ||
      centerDistance > maximumDistance + 1e-4f) {
    return false;
  }

  const float distance = constrain(centerDistance, minimumDistance, maximumDistance);
  const float along =
      (radiusA * radiusA - radiusB * radiusB + distance * distance) /
      (2.0f * distance);
  const float height = sqrtf(fmaxf(0.0f, radiusA * radiusA - along * along));
  const float nx = dx / centerDistance;
  const float nz = dz / centerDistance;
  const Point2 base{centerA.x + along * nx, centerA.z + along * nz};
  const Point2 candidates[2] = {
      {base.x - height * nz, base.z + height * nx},
      {base.x + height * nz, base.z - height * nx},
  };

  if (fabsf(assemblyOrientation) > 1e-5f && height > 1e-5f) {
    const bool expectedPositive = assemblyOrientation > 0.0f;
    for (const Point2 &candidate : candidates) {
      if ((orientation(centerA, centerB, candidate) > 0.0f) == expectedPositive) {
        *result = candidate;
        return true;
      }
    }
  }

  *result = distance2(candidates[0], preferred) <= distance2(candidates[1], preferred)
      ? candidates[0]
      : candidates[1];
  return true;
}

bool solveCadPose(const CadGeometry &geometry,
                  float upperDeltaDeg,
                  float lowerDeltaDeg,
                  CadPose *pose) {
  const float upperDelta = -upperDeltaDeg * kDegToRad;
  const float lowerDelta =
      (-lowerDeltaDeg + geometry.neutralLowerOffsetDeg) * kDegToRad;

  pose->lowerPassive =
      rotate2(geometry.lowerPassive, geometry.lowerDrive, lowerDelta);
  pose->lowerClosureDriver =
      rotate2(geometry.lowerClosureDriver, geometry.lowerDrive, lowerDelta);
  pose->upperClosureDriver =
      rotate2(geometry.upperClosureDriver, geometry.upperDrive, upperDelta);

  const float upperCouplerRadius =
      distance2(geometry.lowerPassive, geometry.upperClosureCoupler);
  const float upperClosureLength =
      distance2(geometry.upperClosureCoupler, geometry.upperClosureDriver);
  if (!circleIntersection(
          pose->lowerPassive,
          upperCouplerRadius,
          pose->upperClosureDriver,
          upperClosureLength,
          geometry.upperClosureCoupler,
          orientation(
              geometry.lowerPassive,
              geometry.upperClosureDriver,
              geometry.upperClosureCoupler),
          &pose->upperClosureCoupler)) {
    return false;
  }

  const float originalCouplerAngle = atan2f(
      geometry.upperClosureCoupler.z - geometry.lowerPassive.z,
      geometry.upperClosureCoupler.x - geometry.lowerPassive.x);
  const float currentCouplerAngle = atan2f(
      pose->upperClosureCoupler.z - pose->lowerPassive.z,
      pose->upperClosureCoupler.x - pose->lowerPassive.x);
  pose->lowerCoupler = rotate2(
      geometry.lowerCoupler,
      geometry.lowerPassive,
      currentCouplerAngle - originalCouplerAngle);
  pose->lowerCoupler.x += pose->lowerPassive.x - geometry.lowerPassive.x;
  pose->lowerCoupler.z += pose->lowerPassive.z - geometry.lowerPassive.z;

  const float diagonalLength =
      distance2(geometry.lowerCoupler, geometry.lowerClosureDiagonal);
  const float lowerClosureLength =
      distance2(geometry.lowerClosureDriver, geometry.lowerClosureDiagonal);
  return circleIntersection(
      pose->lowerCoupler,
      diagonalLength,
      pose->lowerClosureDriver,
      lowerClosureLength,
      geometry.lowerClosureDiagonal,
      orientation(
          geometry.lowerCoupler,
          geometry.lowerClosureDriver,
          geometry.lowerClosureDiagonal),
      &pose->lowerClosureDiagonal);
}

bool cadFootForServoDeltas(const CadGeometry &geometry,
                          float upperDeltaDeg,
                          float lowerDeltaDeg,
                          Point2 *foot) {
  CadPose pose{};
  if (!solveCadPose(geometry, upperDeltaDeg, lowerDeltaDeg, &pose)) {
    return false;
  }

  const float originalAngle = atan2f(
      geometry.lowerClosureDiagonal.z - geometry.lowerClosureDriver.z,
      geometry.lowerClosureDiagonal.x - geometry.lowerClosureDriver.x);
  const float currentAngle = atan2f(
      pose.lowerClosureDiagonal.z - pose.lowerClosureDriver.z,
      pose.lowerClosureDiagonal.x - pose.lowerClosureDriver.x);
  const float angle = currentAngle - originalAngle;
  const float localX = geometry.foot.x - geometry.lowerClosureDriver.x;
  const float localZ = geometry.foot.z - geometry.lowerClosureDriver.z;
  foot->x = pose.lowerClosureDriver.x +
            cosf(angle) * localX -
            sinf(angle) * localZ;
  foot->z = pose.lowerClosureDriver.z +
            sinf(angle) * localX +
            cosf(angle) * localZ;
  return true;
}

bool solveCadPlanar(const CadGeometry &geometry,
                    float targetX,
                    float targetZ,
                    uint8_t solverIndex,
                    float *upperDeltaDeg,
                    float *lowerDeltaDeg,
                    Point2 *solvedFoot) {
  float upper = gCadUpperSeedDeg[solverIndex];
  float lower = gCadLowerSeedDeg[solverIndex];
  constexpr float kFiniteDifferenceDeg = 0.05f;

  for (int iteration = 0; iteration < 10; ++iteration) {
    Point2 current{};
    Point2 upperStep{};
    Point2 lowerStep{};
    if (!cadFootForServoDeltas(geometry, upper, lower, &current) ||
        !cadFootForServoDeltas(
            geometry, upper + kFiniteDifferenceDeg, lower, &upperStep) ||
        !cadFootForServoDeltas(
            geometry, upper, lower + kFiniteDifferenceDeg, &lowerStep)) {
      return false;
    }

    const float errorX = targetX - current.x;
    const float errorZ = targetZ - current.z;
    if (hypotf(errorX, errorZ) <= 0.05f) {
      *upperDeltaDeg = upper;
      *lowerDeltaDeg = lower;
      *solvedFoot = current;
      gCadUpperSeedDeg[solverIndex] = upper;
      gCadLowerSeedDeg[solverIndex] = lower;
      return true;
    }

    const float j00 = (upperStep.x - current.x) / kFiniteDifferenceDeg;
    const float j10 = (upperStep.z - current.z) / kFiniteDifferenceDeg;
    const float j01 = (lowerStep.x - current.x) / kFiniteDifferenceDeg;
    const float j11 = (lowerStep.z - current.z) / kFiniteDifferenceDeg;
    const float determinant = j00 * j11 - j01 * j10;
    if (fabsf(determinant) < 1e-6f) {
      return false;
    }

    const float stepUpper = (errorX * j11 - j01 * errorZ) / determinant;
    const float stepLower = (j00 * errorZ - errorX * j10) / determinant;
    upper = constrain(
        upper + constrain(stepUpper, -8.0f, 8.0f),
        -kMaxMechanicalDeltaDeg,
        kMaxMechanicalDeltaDeg);
    lower = constrain(
        lower + constrain(stepLower, -8.0f, 8.0f),
        -kMaxMechanicalDeltaDeg,
        kMaxMechanicalDeltaDeg);
  }

  if (!cadFootForServoDeltas(geometry, upper, lower, solvedFoot) ||
      hypotf(targetX - solvedFoot->x, targetZ - solvedFoot->z) > 1.0f) {
    return false;
  }
  *upperDeltaDeg = upper;
  *lowerDeltaDeg = lower;
  gCadUpperSeedDeg[solverIndex] = upper;
  gCadLowerSeedDeg[solverIndex] = lower;
  return true;
}

bool solveCadEndpoint(const LegConfig &leg,
                      float x,
                      float y,
                      float z,
                      float *shoulderDeltaDeg,
                      float *upperDeltaDeg,
                      float *lowerDeltaDeg) {
  const bool left = leg.lateralDir > 0;
  const CadGeometry &geometry = left ? kLeftCad : kRightCad;
  const float side = static_cast<float>(leg.lateralDir);
  const float neutralY = side * kCadNeutralFootYmm;
  const float targetX =
      geometry.neutralFoot.x + (x - kNeutralCommandXmm);
  const float targetY =
      neutralY + (y - side * kNeutralCommandYmm);
  const float targetZ =
      geometry.neutralFoot.z - (z - kNeutralCommandZmm);
  const float targetRadius = hypotf(targetY, targetZ);
  const float targetPlanarZ = -sqrtf(
      fmaxf(targetRadius * targetRadius - neutralY * neutralY, 0.0f));

  Point2 solvedFoot{};
  if (!solveCadPlanar(
          geometry,
          targetX,
          targetPlanarZ,
          leg.solverIndex,
          upperDeltaDeg,
          lowerDeltaDeg,
          &solvedFoot)) {
    return false;
  }

  const float baseAngle = atan2f(solvedFoot.z, neutralY);
  const float targetAngle = atan2f(targetZ, targetY);
  float rootAngle = targetAngle - baseAngle;
  while (rootAngle > kPi) rootAngle -= 2.0f * kPi;
  while (rootAngle < -kPi) rootAngle += 2.0f * kPi;
  *shoulderDeltaDeg = constrain(
      -rootAngle * kRadToDeg / side,
      -kMaxMechanicalDeltaDeg,
      kMaxMechanicalDeltaDeg);
  return true;
}

float applyServoSafetyLimit(uint8_t channel, float angleDegrees) {
  const float clamped = clamp270(angleDegrees);
  if (channel >= kPcaChannelCount) {
    return clamped;
  }
  const ServoAngleLimit &limit = kServoAngleLimits[channel];
  const float limited = constrain(clamped, limit.minDeg, limit.maxDeg);
  if (fabsf(limited - angleDegrees) > 0.001f) ++gServoSafetyClipCount;
  return limited;
}

uint16_t angleToPulse(float angleDegrees) {
  const float clamped = clamp270(angleDegrees);
  const float ratio = clamped / kMaxAngleDeg;
  const float microseconds = kMinPulseUs + (kMaxPulseUs - kMinPulseUs) * ratio;
  return static_cast<uint16_t>(microseconds);
}

void write270(Adafruit_PWMServoDriver &driver, uint8_t logicalChannel, float angleDegrees) {
  const float calibratedAngle = applyServoCalibration(gServoCalibration, logicalChannel, angleDegrees);
  const float safeAngle = applyServoSafetyLimit(logicalChannel, calibratedAngle);
  const uint8_t physicalChannel = servoCalibrationPhysicalChannel(gServoCalibration, logicalChannel);
  const uint16_t pulseUs = angleToPulse(safeAngle);
  if (logicalChannel < kPcaChannelCount) {
    gCommandedServoAnglesDeg[logicalChannel] = safeAngle;
    gCommandedServoPulseUs[logicalChannel] = pulseUs;
  }
#ifdef DOMINO_SIL
  driver.writeMicroseconds(physicalChannel, pulseUs);
#else
  if (gServoOutputsEnabled) driver.writeMicroseconds(physicalChannel, pulseUs);
#endif
}

// Leg channel assignments:
// FL: hip=0,  upper=1,  lower=2
// FR: hip=3,  upper=4,  lower=15
// BL: hip=14, upper=7,  lower=8
// BR: hip=9,  upper=10, lower=11

// Right legs use +1 directions; left legs share the same hip direction, but mirror upper/lower.
const LegConfig FR_leg{
    "FR",                // name
    3,                   // hipChannel
    4,                   // upperChannel
    15,                  // lowerChannel
    FR_HIP_TRIM_DEG,     // hipTrimDeg
    FR_UPPER_TRIM_DEG,   // upperTrimDeg
    FR_LOWER_TRIM_DEG,   // lowerTrimDeg
    +1,                  // hipDir
    +1,                  // upperDir
    +1,                  // lowerDir
    -1,                  // lateralDir (right side)
    0.0f,                // hx
    0.0f,                // hy
    0};                   // solverIndex

const LegConfig BR_leg{
    "BR",                // name
    9,                   // hipChannel
    10,                  // upperChannel
    11,                  // lowerChannel
    BR_HIP_TRIM_DEG,     // hipTrimDeg
    BR_UPPER_TRIM_DEG,   // upperTrimDeg
    BR_LOWER_TRIM_DEG,   // lowerTrimDeg
    +1,                  // hipDir
    +1,                  // upperDir
    +1,                  // lowerDir
    -1,                  // lateralDir (right side)
    0.0f,                // hx
    0.0f,                // hy
    1};                   // solverIndex

const LegConfig FL_leg{
    "FL",                // name
    0,                   // hipChannel
    1,                   // upperChannel
    2,                   // lowerChannel
    FL_HIP_TRIM_DEG,     // hipTrimDeg
    FL_UPPER_TRIM_DEG,   // upperTrimDeg
    FL_LOWER_TRIM_DEG,   // lowerTrimDeg
    +1,                  // hipDir (all hips oriented the same way in hardware)
    -1,                  // upperDir
    -1,                  // lowerDir
    +1,                  // lateralDir (left side)
    0.0f,                // hx
    0.0f,                // hy
    2};                   // solverIndex

const LegConfig BL_leg{
    "BL",                // name
    14,                  // hipChannel
    7,                   // upperChannel
    8,                   // lowerChannel
    BL_HIP_TRIM_DEG,     // hipTrimDeg
    BL_UPPER_TRIM_DEG,   // upperTrimDeg
    BL_LOWER_TRIM_DEG,   // lowerTrimDeg
    +1,                  // hipDir (all hips oriented the same way in hardware)
    -1,                  // upperDir
    -1,                  // lowerDir
    +1,                  // lateralDir (left side)
    0.0f,                // hx
    0.0f,                // hy
    3};                   // solverIndex
}  // namespace

void moveLeg(const LegConfig &leg, Adafruit_PWMServoDriver &driver, float x, float y, float z) {
  float shoulderDelta = 0.0f;
  float upperDelta = 0.0f;
  float lowerDelta = 0.0f;
  const bool cadSolved = solveCadEndpoint(
      leg,
      x,
      y,
      z,
      &shoulderDelta,
      &upperDelta,
      &lowerDelta);

  if (!cadSolved) {
    // Retain the old virtual-chain mapping as a bounded fallback if a target is
    // outside the measured four-bar workspace or the numerical solve becomes
    // singular. Normal stand, sit, and tilt poses use the CAD inverse above.
    float theta1 = 0.0f;
    float theta2 = 0.0f;
    float theta3 = 0.0f;
    IK(x, y * static_cast<float>(leg.lateralDir), z, &theta1, &theta2, &theta3);
    shoulderDelta = theta1;
    upperDelta = kUpperCadDriveGain * (theta2 - kNeutralTheta2Deg);
    lowerDelta = kLowerCadDriveGain * (theta3 - kNeutralTheta3Deg);
  }

  const float hipAngle =
      kServoCenterDeg + leg.hipTrimDeg +
      static_cast<float>(leg.hipDir) * shoulderDelta;
  const float calibratedTheta2 = kNeutralTheta2Deg + upperDelta;
  const float calibratedTheta3 = kNeutralTheta3Deg + lowerDelta;
  const float upperAngle =
      kServoCenterDeg + leg.upperTrimDeg + static_cast<float>(leg.upperDir) * calibratedTheta2;
  const float lowerAngle =
      kServoCenterDeg + leg.lowerTrimDeg + static_cast<float>(leg.lowerDir) * calibratedTheta3;

  write270(driver, leg.hipChannel, hipAngle);
  write270(driver, leg.upperChannel, upperAngle);
  write270(driver, leg.lowerChannel, lowerAngle);
}

// Wrappers per leg
void moveLegFR(Adafruit_PWMServoDriver &driver, float x, float y, float z) { moveLeg(FR_leg, driver, x, y, z); }
void moveLegFL(Adafruit_PWMServoDriver &driver, float x, float y, float z) { moveLeg(FL_leg, driver, x, y, z); }
void moveLegBR(Adafruit_PWMServoDriver &driver, float x, float y, float z) { moveLeg(BR_leg, driver, x, y, z); }
void moveLegBL(Adafruit_PWMServoDriver &driver, float x, float y, float z) { moveLeg(BL_leg, driver, x, y, z); }

// Stub: future body pose handling (keep feet fixed, move body)
void setBodyPose(const BodyPose & /*pose*/, Adafruit_PWMServoDriver & /*driver*/) {
  // Intentionally empty for now.
}

void setServoOutputsEnabled(Adafruit_PWMServoDriver &driver, bool enabled) {
  gServoOutputsEnabled = enabled;
#ifdef DOMINO_SIL
  (void)driver;
#endif
  if (!enabled) {
#ifndef DOMINO_SIL
    for (uint8_t channel = 0; channel < kPcaChannelCount; ++channel) {
      driver.setPWM(channel, 0, 4096);
    }
#endif
  }
}

bool servoOutputsEnabled() { return gServoOutputsEnabled; }
const float* commandedServoAnglesDeg() { return gCommandedServoAnglesDeg; }
const uint16_t* commandedServoPulseUs() { return gCommandedServoPulseUs; }
uint32_t servoSafetyClipCount() { return gServoSafetyClipCount; }

bool commandCalibrationServoAngle(Adafruit_PWMServoDriver &driver,
                                  uint8_t logicalChannel,
                                  uint8_t physicalChannel,
                                  float angleDeg) {
  if (!gServoOutputsEnabled || logicalChannel >= kPcaChannelCount ||
      physicalChannel >= kPcaChannelCount ||
      !findServoCalibrationJoint(gServoCalibration, logicalChannel)) return false;
  // Bench targets are already absolute calibrated servo angles from the wizard.
  const float safeAngle = applyServoSafetyLimit(logicalChannel, angleDeg);
  const uint16_t pulseUs = angleToPulse(safeAngle);
  gCommandedServoAnglesDeg[logicalChannel] = safeAngle;
  gCommandedServoPulseUs[logicalChannel] = pulseUs;
  driver.writeMicroseconds(physicalChannel, pulseUs);
  return true;
}

void disableServoOutputPhysicalChannel(Adafruit_PWMServoDriver &driver, uint8_t physicalChannel) {
  if (physicalChannel >= kPcaChannelCount) return;
#ifndef DOMINO_SIL
  driver.setPWM(physicalChannel, 0, 4096);
#else
  (void)driver;
#endif
}

bool setServoCalibrationProfile(const ServoCalibrationProfile &profile) {
  if (!validateServoCalibrationProfile(profile)) return false;
  gServoCalibration = profile;
  return true;
}

const ServoCalibrationProfile& servoCalibrationProfile() { return gServoCalibration; }
