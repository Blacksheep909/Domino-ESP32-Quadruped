export const MOTION_SMOOTHING_FIELDS = Object.freeze([
  Object.freeze({ key: "heightRateMmPerSec", label: "Stand height", min: 40, max: 300, step: 10, unit: "mm/s" }),
  Object.freeze({ key: "rollRateDegPerSec", label: "Tilt roll", min: 40, max: 360, step: 10, unit: "°/s" }),
  Object.freeze({ key: "pitchRateDegPerSec", label: "Tilt pitch", min: 30, max: 240, step: 10, unit: "°/s" }),
  Object.freeze({ key: "yawRateDegPerSec", label: "Tilt yaw", min: 40, max: 450, step: 5, unit: "°/s" }),
  Object.freeze({ key: "tiltInputDeadband", label: "Tilt center deadband", min: 0, max: 0.08, step: 0.005, unit: "%" }),
]);

// The preview uses the same 20 ms control interval and rate-limit rule as the
// firmware's rampFloat. It models commanded pose, not measured servo motion.
export const MOTION_SMOOTHING_CONTROL_STEP_MS = 20;
export const MOTION_SMOOTHING_PREVIEW_AXES = Object.freeze([
  Object.freeze({ key: "rollRateDegPerSec", label: "Roll", unit: "°", target: 16, maximum: 20, step: 1, windowMs: 600 }),
  Object.freeze({ key: "pitchRateDegPerSec", label: "Pitch", unit: "°", target: 8, maximum: 10, step: 0.5, windowMs: 500 }),
  Object.freeze({ key: "yawRateDegPerSec", label: "Yaw", unit: "°", target: 20, maximum: 25, step: 1, windowMs: 700 }),
  Object.freeze({ key: "heightRateMmPerSec", label: "Height", unit: "mm", target: 60, maximum: 80, step: 5, windowMs: 1600 }),
]);

export const DEFAULT_MOTION_SMOOTHING = Object.freeze({
  schemaVersion: 2,
  robot: "domino-esp32-quadruped",
  updatedAt: 0,
  enabled: true,
  heightRateMmPerSec: 120,
  rollRateDegPerSec: 180,
  pitchRateDegPerSec: 120,
  yawRateDegPerSec: 225,
  tiltInputDeadband: 0.02,
});

function validMotionSmoothingCore(candidate) {
  return candidate &&
    candidate.robot === "domino-esp32-quadruped" &&
    Number.isSafeInteger(candidate.updatedAt) && candidate.updatedAt >= 0 &&
    MOTION_SMOOTHING_FIELDS.every(({ key, min, max }) =>
      typeof candidate[key] === "number" && Number.isFinite(candidate[key]) &&
      candidate[key] >= min && candidate[key] <= max);
}

export function validMotionSmoothingSettings(candidate) {
  return candidate?.schemaVersion === 2 &&
    typeof candidate.enabled === "boolean" &&
    validMotionSmoothingCore(candidate);
}

export function validMotionSmoothingDraft(candidate) {
  return validMotionSmoothingSettings(candidate) ||
    (candidate?.schemaVersion === 1 && validMotionSmoothingCore(candidate));
}

export function motionSmoothingDraft(candidate = DEFAULT_MOTION_SMOOTHING) {
  if (!validMotionSmoothingDraft(candidate)) return { ...DEFAULT_MOTION_SMOOTHING };
  return Object.fromEntries(Object.keys(DEFAULT_MOTION_SMOOTHING)
    .map((key) => [key, candidate.schemaVersion === 1 && key === "schemaVersion" ? 2 :
      candidate.schemaVersion === 1 && key === "enabled" ? true : candidate[key]]));
}

export function motionSmoothingRampPreview(settings, axisKey, target) {
  const axis = MOTION_SMOOTHING_PREVIEW_AXES.find((item) => item.key === axisKey);
  if (!axis || !validMotionSmoothingSettings(settings) ||
      !Number.isFinite(target) || target < 0 || target > axis.maximum) return null;
  const enabled = settings.enabled;
  const rate = settings[axisKey];
  const step = enabled ? rate * MOTION_SMOOTHING_CONTROL_STEP_MS / 1000 : target;
  const settleMs = target === 0 ? 0 : enabled
    ? Math.ceil(target / step) * MOTION_SMOOTHING_CONTROL_STEP_MS
    : MOTION_SMOOTHING_CONTROL_STEP_MS;
  const points = [{ ms: 0, value: 0 }];
  let value = 0;
  for (let ms = MOTION_SMOOTHING_CONTROL_STEP_MS; ms <= axis.windowMs;
       ms += MOTION_SMOOTHING_CONTROL_STEP_MS) {
    value = enabled ? Math.min(target, value + step) : target;
    points.push({ ms, value });
  }
  return { axis, target, enabled, rate, step, settleMs, points };
}

export function motionSmoothingDeadbandPreview(deadband, enabled = true) {
  if (!Number.isFinite(deadband) || deadband < 0 || deadband > 0.08) return null;
  const shaped = (input) => !enabled ? input :
    input <= deadband ? 0 : (input - deadband) / (1 - deadband);
  return {
    enabled,
    deadband,
    points: Array.from({ length: 21 }, (_, index) => {
      const input = index * 0.005;
      return { input, output: shaped(input) };
    }),
    tenPercentOutput: shaped(0.1),
  };
}
