#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#include "leg_controller.h"

#include "ik.h"

namespace {
constexpr float kServoCenterDeg = 135.0f;
constexpr float kMinPulseUs = 500.0f;
constexpr float kMaxPulseUs = 2500.0f;
constexpr float kMaxAngleDeg = 270.0f;
constexpr uint8_t kPcaChannelCount = 16;

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

float applyServoSafetyLimit(uint8_t channel, float angleDegrees) {
  const float clamped = clamp270(angleDegrees);
  if (channel >= kPcaChannelCount) {
    return clamped;
  }
  const ServoAngleLimit &limit = kServoAngleLimits[channel];
  return constrain(clamped, limit.minDeg, limit.maxDeg);
}

uint16_t angleToPulse(float angleDegrees) {
  const float clamped = clamp270(angleDegrees);
  const float ratio = clamped / kMaxAngleDeg;
  const float microseconds = kMinPulseUs + (kMaxPulseUs - kMinPulseUs) * ratio;
  return static_cast<uint16_t>(microseconds);
}

void write270(Adafruit_PWMServoDriver &driver, uint8_t channel, float angleDegrees) {
  driver.writeMicroseconds(channel, angleToPulse(applyServoSafetyLimit(channel, angleDegrees)));
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
    0.0f,                // hx
    0.0f};               // hy

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
    0.0f,                // hx
    0.0f};               // hy

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
    0.0f,                // hx
    0.0f};               // hy

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
    0.0f,                // hx
    0.0f};               // hy
}  // namespace

void moveLeg(const LegConfig &leg, Adafruit_PWMServoDriver &driver, float x, float y, float z) {
  float theta1 = 0.0f;
  float theta2 = 0.0f;
  float theta3 = 0.0f;
  IK(x, y, z, &theta1, &theta2, &theta3);

  const float hipAngle = kServoCenterDeg + leg.hipTrimDeg + static_cast<float>(leg.hipDir) * theta1;
  const float upperAngle = kServoCenterDeg + leg.upperTrimDeg + static_cast<float>(leg.upperDir) * theta2;
  const float lowerAngle = kServoCenterDeg + leg.lowerTrimDeg + static_cast<float>(leg.lowerDir) * theta3;

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
