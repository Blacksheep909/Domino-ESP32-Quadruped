#pragma once

// Learned walking policy for the "simulation" mode (SB fully down).
//
// The network is trained in Isaac Lab (simulation/isaac_lab) and exported to
// src/policy_weights.h by scripts/export_policy.py. This module is plain C++
// with no Arduino dependency so the native tests can run it on a PC.
//
// Contract with the trainer (see simulation/isaac_lab/domino_lab/env_cfg.py):
//   * 50 Hz steps (kControlIntervalMs = 20).
//   * Observation = [gyro x3 history, gravity x3 history, previous actions x3
//     history, velocity command, ride-height command, gait clock] (60 floats).
//   * Output = 12 foot offsets (FL, FR, BL, BR x dx, dy, dz) scaled by
//     kPolicyActionScaleMm around the neutral stand at the commanded height,
//     clamped to the same box the trainer uses. main.cpp feeds them to
//     commandLeg(), so the CAD linkage solver, calibration and servo limits
//     still own every pulse.
//
// Budget on the ESP32-DevKitC (240 MHz LX6, 520 KB SRAM, no PSRAM): weights
// live in flash (const), RAM use is the 60-float history plus two activation
// buffers (< 2 KB); inference is ~18k multiply-adds, well under 1 ms.

#include <stdint.h>

struct PolicyCommand {
  float vxMps;       // forward (+x)
  float vyMps;       // left (+y)
  float wzRadps;     // counter-clockwise (+z)
  float heightMm;    // ride height, hip axis to foot centre
};

// IMU sample already rotated into the body frame (+x forward, +y left, +z up).
struct PolicyImu {
  float gyroRadps[3];
  float accelG[3];   // specific force; level and still = (0, 0, +1)
  bool fresh;
};

struct PolicyFeet {
  // Firmware leg command frame (mm): x forward, y outward-positive, z down.
  float xMm[4];
  float yOutMm[4];
  float zMm[4];
};

enum class PolicyFault : uint8_t {
  None = 0,
  NotAvailable,
  SelfTestFailed,
  ImuStale,
  Tilt,
  NonFinite,
};

struct PolicyStatus {
  bool available;
  bool selfTestRan;
  bool selfTestOk;
  float selfTestMaxError;
  bool active;
  uint32_t steps;
  PolicyFault fault;
  float gravity[3];
  float tiltDeg;
  float lastAction[12];
  float clockPhase;
};

const char *policyId();
const char *policyRunName();
int policyIteration();
int policyObsDim();

// Ride-height command slew the policy was trained with (mm/s).
float policyHeightSlewMmPerSec();
const char *policyFaultName(PolicyFault fault);

// Runs the exported self-test vector through the network. Call once at boot.
bool policySelfTest();

// Start a fresh episode: clears action history, seeds the gravity estimate
// from the accelerometer and fills the observation history.
void policyReset(const PolicyImu &imu);

// One 50 Hz step. Returns false (and leaves `feet` at the neutral stand) on
// any fault; the caller must then leave the mode.
bool policyStep(const PolicyCommand &command, const PolicyImu &imu, float dtSeconds,
                PolicyFeet *feet);

void policyStop();
const PolicyStatus &policyStatus();

// Helpers shared with the tests.
void policyNeutralFeet(float heightMm, PolicyFeet *feet);
void policyActionsToFeet(const float actions[12], float heightMm, PolicyFeet *feet);
void policyForward(const float *obs, float *actionsOut);
