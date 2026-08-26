#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include "manual_control_guard.h"

enum class LiveRobotState : uint8_t {
  Disarmed,
  Armed,
  Estopped,
  Watchdog,
  Fault,
};

struct LiveRobotPoseSnapshot {
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  float heightMm = 0.0f;
};

void liveRobotEndpointBegin(Adafruit_PWMServoDriver &driver);
void liveRobotEndpointLoop(uint32_t now, Adafruit_PWMServoDriver &driver);
void liveRobotEndpointSetExpectedPose(const LiveRobotPoseSnapshot &pose);
void liveRobotEndpointSetExpectedFootTarget(uint8_t legIndex, float xMm, float yMm, float zMm);
LiveRobotState liveRobotEndpointState();
bool liveRobotEndpointAllowsLocomotion();
// The CRSF receiver remains usable without a PC. LIVE can explicitly inhibit
// that path through its safety and calibration state machines.
bool liveRobotEndpointAllowsRadioControl();
bool liveRobotEndpointCalibrationOwnsOutputs();
LiveManualControlSnapshot liveRobotEndpointManualControl();
