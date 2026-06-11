#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#include "leg_controller.h"

#include "ik.h"

namespace {
constexpr float kServoCenterDeg = 135.0f;
constexpr float kMinPulseUs = 500.0f;
constexpr float kMaxPulseUs = 2500.0f;
constexpr float kMaxAngleDeg = 270.0f;

float clamp270(float angle) { return constrain(angle, 0.0f, kMaxAngleDeg); }

uint16_t angleToPulse(float angleDegrees) {
  const float clamped = clamp270(angleDegrees);
  const float ratio = clamped / kMaxAngleDeg;
  const float microseconds = kMinPulseUs + (kMaxPulseUs - kMinPulseUs) * ratio;
  return static_cast<uint16_t>(microseconds);
}

void write270(Adafruit_PWMServoDriver &driver, uint8_t channel, float angleDegrees) {
  driver.writeMicroseconds(channel, angleToPulse(angleDegrees));
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

  const float hipAngle = clamp270(kServoCenterDeg + leg.hipTrimDeg + static_cast<float>(leg.hipDir) * theta1);
  const float upperAngle = clamp270(kServoCenterDeg + leg.upperTrimDeg + static_cast<float>(leg.upperDir) * theta2);
  const float lowerAngle = clamp270(kServoCenterDeg + leg.lowerTrimDeg + static_cast<float>(leg.lowerDir) * theta3);

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
