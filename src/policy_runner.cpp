#include "policy_runner.h"

#include <math.h>
#include <string.h>

#include "policy_weights.h"

// Exported policies say whether they observe the gait clock (older exports
// always did) and how fast the trainer slewed the ride-height command.
#ifndef DOMINO_POLICY_GAIT_CLOCK
#define DOMINO_POLICY_GAIT_CLOCK 1
#endif
#ifndef DOMINO_POLICY_HEIGHT_SLEW_MM_S
#define DOMINO_POLICY_HEIGHT_SLEW_MM_S 80.0f
#endif

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr int kHistory = 3;
constexpr int kGyroOffset = 0;                       // 3 x 3
constexpr int kGravityOffset = kGyroOffset + 9;      // 3 x 3
constexpr int kActionOffset = kGravityOffset + 9;    // 3 x 12
constexpr int kCommandOffset = kActionOffset + 36;   // 3
constexpr int kHeightOffset = kCommandOffset + 3;    // 1
constexpr int kClockOffset = kHeightOffset + 1;      // 2
constexpr int kObsDim = kClockOffset + (DOMINO_POLICY_GAIT_CLOCK ? 2 : 0);  // 60 or 58

// Keep in sync with simulation/isaac_lab/domino_lab/geometry.py.
constexpr float kNeutralXmm = -15.75f;
constexpr float kNeutralYmm = 38.0f;
constexpr float kFootDxMinMm = -80.0f;
constexpr float kFootDxMaxMm = 80.0f;
constexpr float kFootYMinMm = 8.0f;
constexpr float kFootYMaxMm = 78.0f;
constexpr float kFootZMinMm = 165.0f;
constexpr float kFootZMaxMm = 300.0f;
constexpr float kHeightMidM = 0.25f;
constexpr float kHeightHalfM = 0.03f;
constexpr float kGyroScale = 0.25f;

// Gravity estimate: integrate the gyro, pull slowly toward the accelerometer.
constexpr float kAccelBlendPerSecond = 2.0f;
constexpr float kMaxTiltDeg = 55.0f;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

#if DOMINO_POLICY_AVAILABLE
static_assert(kPolicyObsDim == kObsDim, "policy_weights.h does not match the firmware observation layout");
static_assert(kPolicyActDim == 12, "policy must output 12 foot offsets");
#endif

PolicyStatus gStatus{};
float gGyroHist[kHistory][3];
float gGravityHist[kHistory][3];
float gActionHist[kHistory][12];
// Low-passed actions that drive the feet (trainer: FootSpaceAction._filtered).
float gFilteredAction[12];
float gGravity[3] = {0.0f, 0.0f, -1.0f};
float gPhase = 0.0f;

void normalize3(float v[3]) {
  const float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (n > 1e-6f) {
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
  }
}

void gravityFromAccel(const PolicyImu &imu, float out[3]) {
  out[0] = -imu.accelG[0];
  out[1] = -imu.accelG[1];
  out[2] = -imu.accelG[2];
  normalize3(out);
}

void updateGravity(const PolicyImu &imu, float dt) {
  // d(g_body)/dt = g_body x omega
  const float *w = imu.gyroRadps;
  float g[3] = {gGravity[0], gGravity[1], gGravity[2]};
  g[0] += (gGravity[1] * w[2] - gGravity[2] * w[1]) * dt;
  g[1] += (gGravity[2] * w[0] - gGravity[0] * w[2]) * dt;
  g[2] += (gGravity[0] * w[1] - gGravity[1] * w[0]) * dt;
  float a[3];
  gravityFromAccel(imu, a);
  const float alpha = clampf(kAccelBlendPerSecond * dt, 0.0f, 1.0f);
  for (int i = 0; i < 3; ++i) gGravity[i] = (1.0f - alpha) * g[i] + alpha * a[i];
  normalize3(gGravity);
}

void pushHistory(float hist[][3], const float v[3]) {
  memmove(hist[0], hist[1], sizeof(float) * 3 * (kHistory - 1));
  memcpy(hist[kHistory - 1], v, sizeof(float) * 3);
}

void pushActionHistory(const float a[12]) {
  memmove(gActionHist[0], gActionHist[1], sizeof(float) * 12 * (kHistory - 1));
  memcpy(gActionHist[kHistory - 1], a, sizeof(float) * 12);
}

void buildObservation(const PolicyCommand &cmd, float phase, float *obs) {
  for (int h = 0; h < kHistory; ++h) {
    for (int i = 0; i < 3; ++i) {
      obs[kGyroOffset + h * 3 + i] = gGyroHist[h][i];
      obs[kGravityOffset + h * 3 + i] = gGravityHist[h][i];
    }
    for (int i = 0; i < 12; ++i) obs[kActionOffset + h * 12 + i] = gActionHist[h][i];
  }
  obs[kCommandOffset + 0] = cmd.vxMps;
  obs[kCommandOffset + 1] = cmd.vyMps;
  obs[kCommandOffset + 2] = cmd.wzRadps;
  obs[kHeightOffset] = (cmd.heightMm * 0.001f - kHeightMidM) / kHeightHalfM;
#if DOMINO_POLICY_GAIT_CLOCK
  obs[kClockOffset + 0] = sinf(phase);
  obs[kClockOffset + 1] = cosf(phase);
#else
  (void)phase;
#endif
}

}  // namespace

#if DOMINO_POLICY_AVAILABLE
const char *policyId() { return DOMINO_POLICY_ID; }
const char *policyRunName() { return DOMINO_POLICY_RUN; }
int policyIteration() { return DOMINO_POLICY_ITERATION; }

void policyForward(const float *obs, float *actionsOut) {
  float bufA[kPolicyMaxWidth];
  float bufB[kPolicyMaxWidth];
  for (int i = 0; i < kPolicyObsDim; ++i) bufA[i] = (obs[i] - kPolicyObsMean[i]) * kPolicyObsInvStd[i];
  float *in = bufA;
  float *out = bufB;
  for (int layer = 0; layer < kPolicyLayerCount; ++layer) {
    const int nIn = kPolicyLayerDims[layer];
    const int nOut = kPolicyLayerDims[layer + 1];
    const float *w = kPolicyWeights[layer];
    const float *b = kPolicyBiases[layer];
    const bool last = layer == kPolicyLayerCount - 1;
    for (int o = 0; o < nOut; ++o) {
      const float *row = w + o * nIn;
      float acc = b[o];
      for (int i = 0; i < nIn; ++i) acc += row[i] * in[i];
      out[o] = (last || acc > 0.0f) ? acc : expm1f(acc);
    }
    float *t = in;
    in = out;
    out = t;
  }
  for (int i = 0; i < 12; ++i) actionsOut[i] = in[i];
}

bool policySelfTest() {
  float action[12];
  policyForward(kPolicySelfTestObs, action);
  float maxErr = 0.0f;
  for (int i = 0; i < 12; ++i) {
    const float err = fabsf(action[i] - kPolicySelfTestAction[i]);
    if (!(err == err)) maxErr = 1e9f;  // NaN
    else if (err > maxErr) maxErr = err;
  }
  gStatus.available = true;
  gStatus.selfTestRan = true;
  gStatus.selfTestMaxError = maxErr;
  gStatus.selfTestOk = maxErr < 1e-3f;
  return gStatus.selfTestOk;
}
#else
const char *policyId() { return "none"; }
const char *policyRunName() { return "none"; }
int policyIteration() { return 0; }
void policyForward(const float *, float *actionsOut) {
  for (int i = 0; i < 12; ++i) actionsOut[i] = 0.0f;
}
bool policySelfTest() {
  gStatus.available = false;
  gStatus.selfTestRan = true;
  gStatus.selfTestOk = false;
  gStatus.fault = PolicyFault::NotAvailable;
  return false;
}
#endif

int policyObsDim() { return kObsDim; }

float policyHeightSlewMmPerSec() { return DOMINO_POLICY_HEIGHT_SLEW_MM_S; }

const char *policyFaultName(PolicyFault fault) {
  switch (fault) {
    case PolicyFault::None: return "none";
    case PolicyFault::NotAvailable: return "no-policy";
    case PolicyFault::SelfTestFailed: return "self-test";
    case PolicyFault::ImuStale: return "imu-stale";
    case PolicyFault::Tilt: return "tilt";
    case PolicyFault::NonFinite: return "non-finite";
  }
  return "unknown";
}

void policyNeutralFeet(float heightMm, PolicyFeet *feet) {
  const float zero[12] = {};
  policyActionsToFeet(zero, heightMm, feet);
}

void policyActionsToFeet(const float actions[12], float heightMm, PolicyFeet *feet) {
#if DOMINO_POLICY_AVAILABLE
  const float *scale = kPolicyActionScaleMm;
  const float clip = kPolicyActionClip;
#else
  const float scale[3] = {40.0f, 20.0f, 40.0f};
  const float clip = 1.5f;
#endif
  for (int leg = 0; leg < 4; ++leg) {
    const float ax = clampf(actions[leg * 3 + 0], -clip, clip);
    const float ay = clampf(actions[leg * 3 + 1], -clip, clip);
    const float az = clampf(actions[leg * 3 + 2], -clip, clip);
    feet->xMm[leg] = clampf(kNeutralXmm + scale[0] * ax, kNeutralXmm + kFootDxMinMm, kNeutralXmm + kFootDxMaxMm);
    feet->yOutMm[leg] = clampf(kNeutralYmm + scale[1] * ay, kFootYMinMm, kFootYMaxMm);
    feet->zMm[leg] = clampf(heightMm + scale[2] * az, kFootZMinMm, kFootZMaxMm);
  }
}

void policyReset(const PolicyImu &imu) {
  gravityFromAccel(imu, gGravity);
  float gyro[3];
  for (int i = 0; i < 3; ++i) gyro[i] = imu.gyroRadps[i] * kGyroScale;
  for (int h = 0; h < kHistory; ++h) {
    memcpy(gGyroHist[h], gyro, sizeof(gyro));
    memcpy(gGravityHist[h], gGravity, sizeof(gGravity));
    memset(gActionHist[h], 0, sizeof(gActionHist[h]));
  }
  memset(gStatus.lastAction, 0, sizeof(gStatus.lastAction));
  memset(gFilteredAction, 0, sizeof(gFilteredAction));
  gStatus.steps = 0;
  gStatus.clockPhase = 0.0f;
  gPhase = 0.0f;
  gStatus.fault = PolicyFault::None;
  gStatus.active = true;
}

void policyStop() { gStatus.active = false; }

const PolicyStatus &policyStatus() { return gStatus; }

bool policyStep(const PolicyCommand &command, const PolicyImu &imu, float dt, PolicyFeet *feet) {
  policyNeutralFeet(command.heightMm, feet);
  auto fail = [&](PolicyFault fault) {
    gStatus.fault = fault;
    gStatus.active = false;
    return false;
  };
#if !DOMINO_POLICY_AVAILABLE
  return fail(PolicyFault::NotAvailable);
#else
  if (!gStatus.selfTestOk) return fail(PolicyFault::SelfTestFailed);
  if (!imu.fresh) return fail(PolicyFault::ImuStale);

  updateGravity(imu, dt);
  memcpy(gStatus.gravity, gGravity, sizeof(gGravity));
  gStatus.tiltDeg = acosf(clampf(-gGravity[2], -1.0f, 1.0f)) * 180.0f / kPi;
  if (gStatus.tiltDeg > kMaxTiltDeg) return fail(PolicyFault::Tilt);

  float gyro[3];
  for (int i = 0; i < 3; ++i) gyro[i] = imu.gyroRadps[i] * kGyroScale;
  pushHistory(gGyroHist, gyro);
  pushHistory(gGravityHist, gGravity);

  // Gait clock, same rule and order as the trainer: the observation uses the
  // current phase (0 on entry), then the phase advances at a rate that follows
  // the stick (geometry.gait_frequency_hz) as the action is applied.
  gStatus.clockPhase = gPhase;
  float obs[kObsDim];
  buildObservation(command, gPhase, obs);
  const float level = fminf(1.0f, fmaxf(fmaxf(fabsf(command.vxMps) / kPolicyFullCommand[0],
                                              fabsf(command.vyMps) / kPolicyFullCommand[1]),
                                        fabsf(command.wzRadps) / kPolicyFullCommand[2]));
  const float hz = kPolicyClockHzMin + (kPolicyClockHzMax - kPolicyClockHzMin) * level;
  gPhase = fmodf(gPhase + 2.0f * kPi * hz * dt, 2.0f * kPi);

  float action[12];
  policyForward(obs, action);
  for (int i = 0; i < 12; ++i) {
    if (!(action[i] == action[i]) || isinf(action[i])) return fail(PolicyFault::NonFinite);
  }
  // The trainer feeds back the raw network output (last_action), so do the same.
  pushActionHistory(action);
  memcpy(gStatus.lastAction, action, sizeof(action));
  // Same first-order low-pass as the trainer, applied after the raw output
  // is stored in the history (the network sees its unfiltered actions).
  for (int i = 0; i < 12; ++i) {
    gFilteredAction[i] = kPolicyActionFilterAlpha * action[i] + (1.0f - kPolicyActionFilterAlpha) * gFilteredAction[i];
  }
  policyActionsToFeet(gFilteredAction, command.heightMm, feet);
  ++gStatus.steps;
  return true;
#endif
}
