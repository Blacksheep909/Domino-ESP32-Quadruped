// Port of the ear springs in src/personality.cpp (personalityEarTargets /
// personalityEarStep): between emotes the antennas chase a target on a
// spring-damper, so they overshoot a little and settle like floppy ears. In
// tilt mode they perk up and follow the body pose. Constants are checked
// against src/personality.h by a test.

export const EAR = Object.freeze({
  restDeg: -45,
  tiltBaseDeg: -15,
  pitchGain: 1.6,
  rollGain: 1.8,
  yawGain: 0.6,
  lyingDeg: -80,
  springHz: 2.1,
  damping: 0.32,
});

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

export function earTargets({ tilt = false, lying = false, rollDeg = 0, pitchDeg = 0, yawDeg = 0 } = {}) {
  if (lying) return [EAR.lyingDeg, EAR.lyingDeg];
  if (!tilt) return [EAR.restDeg, EAR.restDeg];
  const common = EAR.tiltBaseDeg + EAR.pitchGain * pitchDeg;
  const split = EAR.rollGain * rollDeg + EAR.yawGain * yawDeg;
  return [clamp(common + split, -85, 40), clamp(common - split, -85, 40)];
}

export function createEarSpring(angleDeg = EAR.restDeg) {
  return { angleDeg, velocityDegPerSec: 0 };
}

export function stepEarSpring(ear, targetDeg, dt) {
  const w = 2 * Math.PI * EAR.springHz;
  let left = dt;
  while (left > 0) {
    const h = Math.min(0.005, left);
    left -= h;
    const accel = w * w * (targetDeg - ear.angleDeg) - 2 * EAR.damping * w * ear.velocityDegPerSec;
    ear.velocityDegPerSec += accel * h;
    ear.angleDeg += ear.velocityDegPerSec * h;
    if (ear.angleDeg > 40) { ear.angleDeg = 40; ear.velocityDegPerSec = 0; }
    if (ear.angleDeg < -85) { ear.angleDeg = -85; ear.velocityDegPerSec = 0; }
  }
  return ear;
}
