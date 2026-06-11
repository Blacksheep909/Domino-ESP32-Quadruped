// Inverse kinematics for a single 3‑DoF leg.
//
// Coordinate frame (leg frame):
//   - Origin at hip joint rotation center.
//   - +X forward (toward the head in body/world space).
//   - +Y to the dog's left.
//   - +Z downward from the hip pivot toward the ground.
//
// Geometry:
//   - kL2, kL3: upper and lower link lengths of the virtual 2‑link chain.
//   - kRy, kRz: lateral and vertical offsets from the hip pivot to the
//     simplified 2‑link reference line used in the CAD derivation.
//
// Output angles (degrees, leg frame):
//   - theta1: hip abduction/adduction around the X axis.
//   - theta2: hip pitch around the Y axis.
//   - theta3: knee linkage (parallelogram) angle around the Y axis.
//
// This is a direct transcription of the analytic solution derived from the
// CAD model. Do not change the math unless the mechanical geometry changes.

#include <Arduino.h>
#include <math.h>

#include "ik.h"

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kL2 = 160.0f;  // upper leg link length (mm)
constexpr float kL3 = 153.0f;  // lower leg link length (mm)
constexpr float kRz = 21.0f;   // vertical offset hip -> 2‑link reference (mm)
constexpr float kRy = 38.0f;   // lateral offset hip -> 2‑link reference (mm)

float clampToAcosDomain(float value) {
  if (value > 1.0f) {
    return 1.0f;
  }
  if (value < -1.0f) {
    return -1.0f;
  }
  return value;
}
}  // namespace

void IK(float x, float y, float z, float *theta1, float *theta2, float *theta3) {
  // Distance from hip pivot to foot in the Y‑Z plane.
  const float r1Squared = y * y + z * z;
  const float r1 = sqrtf(r1Squared);
  const float safeR1 = fmaxf(r1, kRy + 1e-3f);
  // Hip abduction is solved by splitting into two angles:
  //  - beta1: raw direction to the foot in the Y‑Z plane.
  //  - beta2: correction for the lateral offset kRy.
  const float beta1 = atan2f(y, z);
  const float beta2 = acosf(clampToAcosDomain(kRy / safeR1));
  *theta1 = kPi / 2.0f - beta1 - beta2;

  // Project into the virtual 2‑link plane after removing kRy and kRz offsets.
  const float r23 = sqrtf(fmaxf(r1Squared - kRy * kRy, 0.0f)) - kRz;
  const float planeZ = r23;
  const float l23 = sqrtf(x * x + planeZ * planeZ);
  const float safeL23 = fmaxf(l23, 1e-3f);
  const float gamma3 = atan2f(x, planeZ);

  const float cosAlpha23 =
      clampToAcosDomain((kL2 * kL2 + kL3 * kL3 - safeL23 * safeL23) / (2.0f * kL2 * kL3));
  const float alpha23 = acosf(cosAlpha23);
  *theta3 = kPi - alpha23;

  const float cosAlpha2 =
      clampToAcosDomain((kL2 * kL2 + safeL23 * safeL23 - kL3 * kL3) / (2.0f * kL2 * safeL23));
  const float alpha2 = acosf(cosAlpha2);
  *theta2 = gamma3 - alpha2;

  // Parallelogram linkage: the knee servo angle is the sum of two internal
  // link angles, so theta3 is coupled to theta2.
  *theta3 += *theta2;

  const float radToDeg = 180.0f / kPi;
  *theta1 *= radToDeg;
  *theta2 *= radToDeg;
  *theta3 *= radToDeg;
}
