#pragma once

#include <Adafruit_PWMServoDriver.h>
#include "servo_calibration.h"

// Per-leg trim values (degrees). These are calibration offsets for the current
// Domino mechanism and should be adjusted only after testing one servo/leg at a time.
// Right-side trim values include the 15.457 degree neutral compensation from
// the old body-frame Y input. This preserves the calibrated high-stand pulse
// after converting hip IK to an outward-positive leg-local Y coordinate.
static constexpr float FR_HIP_TRIM_DEG = -29.542726f;  // CH3
static constexpr float FR_UPPER_TRIM_DEG = -20.0f;  // CH4
static constexpr float FR_LOWER_TRIM_DEG = -38.0f;  // CH15

static constexpr float FL_HIP_TRIM_DEG = -5.0f;  // CH0
static constexpr float FL_UPPER_TRIM_DEG = 0.0f;  // CH1
static constexpr float FL_LOWER_TRIM_DEG = 10.0f;  // CH2

static constexpr float BR_HIP_TRIM_DEG = -19.542726f;  // CH9
static constexpr float BR_UPPER_TRIM_DEG = -20.0f;  // CH10
static constexpr float BR_LOWER_TRIM_DEG = -30.0f;  // CH11

static constexpr float BL_HIP_TRIM_DEG = -15.0f;  // CH14
static constexpr float BL_UPPER_TRIM_DEG = -20.0f;  // CH7
static constexpr float BL_LOWER_TRIM_DEG = -5.0f;  // CH8

struct LegConfig {
  const char* name;
  uint8_t hipChannel;
  uint8_t upperChannel;
  uint8_t lowerChannel;
  float hipTrimDeg;
  float upperTrimDeg;
  float lowerTrimDeg;
  int hipDir;    // +1 or -1
  int upperDir;  // +1 or -1
  int lowerDir;  // +1 or -1
  int lateralDir;  // body-frame Y to outward-positive leg-local Y
  float hx;      // hip offset in x (for future use)
  float hy;      // hip offset in y (for future use)
  uint8_t solverIndex;  // persistent seed for the CAD four-bar inverse
};

// Generic leg mover
void moveLeg(const LegConfig &leg, Adafruit_PWMServoDriver &driver, float x, float y, float z);

// Wrappers per leg
void moveLegFR(Adafruit_PWMServoDriver &driver, float x, float y, float z);
void moveLegFL(Adafruit_PWMServoDriver &driver, float x, float y, float z);
void moveLegBR(Adafruit_PWMServoDriver &driver, float x, float y, float z);
void moveLegBL(Adafruit_PWMServoDriver &driver, float x, float y, float z);

// Future: body pose interface to tilt/yaw/roll with fixed feet
struct BodyPose {
  float x;     // body translation in world frame
  float y;
  float z;
  float roll;  // rotation around x
  float pitch; // rotation around y
  float yaw;   // rotation around z
};

void setBodyPose(const BodyPose &pose, Adafruit_PWMServoDriver &driver);

// The cache is command feedback, not physical joint feedback. Outputs boot
// disabled and are switched fully off for disarm, E-stop and watchdog states.
void setServoOutputsEnabled(Adafruit_PWMServoDriver &driver, bool enabled);
bool servoOutputsEnabled();
const float* commandedServoAnglesDeg();
// Final pulse after calibration and safety limiting, indexed by logical joint.
// Use servoCalibrationPhysicalChannel() to identify the PCA9685 output used.
const uint16_t* commandedServoPulseUs();
uint32_t servoSafetyClipCount();
bool commandCalibrationServoAngle(Adafruit_PWMServoDriver &driver,
                                  uint8_t logicalChannel,
                                  uint8_t physicalChannel,
                                  float angleDeg);
void disableServoOutputPhysicalChannel(Adafruit_PWMServoDriver &driver, uint8_t physicalChannel);
bool setServoCalibrationProfile(const ServoCalibrationProfile &profile);
const ServoCalibrationProfile& servoCalibrationProfile();
