import { legs, standServoReference } from "./domino-config.js";
import { point2, solveLinkagePose } from "./linkage.js";

const DEG = Math.PI / 180;
const TWO_PI = Math.PI * 2;
const COMMAND_X_MM = -15.75;
const COMMAND_Y_MM = 38;
const COMMAND_Z_MM = 280;
const CAD_FOOT_Y_MM = 38.1;
const MAX_SERVO_DELTA_DEG = 45;
const COMMAND_INDEX = { FL: 0, FR: 1, BL: 2, BR: 3 };
// Keep in step with src/main.cpp (kCrawlOrder, kCrawlMaxSwingFraction,
// kCrawlBodyShiftMm, BODY_HALF_LENGTH_X / BODY_HALF_WIDTH_Y).
export const CAREFUL_ORDER = Object.freeze(["BL", "FL", "BR", "FR"]);
const CRAWL_SLOT = 0.25;
const CRAWL_MAX_SWING_FRACTION = 0.17;
const CRAWL_BODY_SHIFT_MM = 30;
const BODY_HALF_LENGTH_X_MM = 167.5;
const BODY_HALF_WIDTH_Y_MM = 62.375;

// Nominal foot position relative to the body centre (x forward, y left).
function footOffsetFromCentre(label, settings) {
  const front = label === "FL" || label === "FR";
  const left = label === "FL" || label === "BL";
  return {
    x: (front ? BODY_HALF_LENGTH_X_MM : -BODY_HALF_LENGTH_X_MM) + settings.touchdownXMm,
    y: (left ? 1 : -1) * (BODY_HALF_WIDTH_Y_MM + settings.stanceWidthMm),
  };
}

// Per-leg stride vector, as in the firmware: forward moves every foot straight
// back; turning moves each planted foot along the circle around the body
// centre, so a pure turn spins the robot on the spot. Positive turn is
// clockwise (to the right). Combined commands longer than the stride are
// scaled down evenly to keep the path's curvature.
// A full sidestep travels this share of the forward stride: the legs reach
// less far sideways than fore/aft (keep in step with kGaitLateralStrideScale
// in src/main.cpp).
export const GAIT_LATERAL_STRIDE_SCALE = 0.6;
// Below this much forward stick, right-stick X is mostly a sidestep.
export const GAIT_STEER_FULL_AT_FORWARD = 0.4;

// Right stick X while walking (keep in step with readGaitCommands in
// src/main.cpp). With no forward stick it sidesteps. As forward stick grows it
// hands over to steering, by `steerBlend`: 1 = full hand-over (steer when
// moving, sidestep when not), 0 = always sidestep (Spot-style: turn with the
// left stick). Positive is to the right for both.
export function splitRightStickX(forward, rightX, steerBlend = 1, strafeWeight = 1) {
  const speed = clamp(Math.abs(Number(forward) || 0), 0, 1);
  const x = clamp(Number(rightX) || 0, -1, 1);
  const share = clamp(Number(steerBlend), 0, 1) * smooth01(clamp(speed / GAIT_STEER_FULL_AT_FORWARD, 0, 1));
  return { steer: x * speed * share, strafe: x * (1 - share) * clamp(Number(strafeWeight), 0, 1) };
}

// Direction intent on the right stick (keep in step with rightStickIntent in
// src/main.cpp). Angles are measured from straight sideways (0 deg) to
// straight forward/back (90 deg).
//  - Within ~15 deg of sideways the forward part is dropped, so a push that is
//    "mostly left" is a clean sidestep; it fades back in by 30 deg.
//  - Within ~10 deg of straight forward the sidestep part is dropped (70-80
//    deg fade), so walking forward does not crab. Steering is not affected:
//    a small sideways nudge while walking still steers.
export const STICK_SIDEWAYS_INTENT_DEG = Object.freeze([15, 30]);
export const STICK_FORWARD_INTENT_DEG = Object.freeze([70, 80]);

export function rightStickIntent(forward, rightX) {
  const y = clamp(Number(forward) || 0, -1, 1);
  const x = clamp(Number(rightX) || 0, -1, 1);
  if (y === 0 || x === 0) return { forward: y, rightX: x, strafeWeight: 1 };
  const angle = Math.atan2(Math.abs(y), Math.abs(x)) / DEG;
  const [sideLow, sideHigh] = STICK_SIDEWAYS_INTENT_DEG;
  const [forwardLow, forwardHigh] = STICK_FORWARD_INTENT_DEG;
  const forwardWeight = smooth01(clamp((angle - sideLow) / (sideHigh - sideLow), 0, 1));
  const strafeWeight = 1 - smooth01(clamp((angle - forwardLow) / (forwardHigh - forwardLow), 0, 1));
  return { forward: y * forwardWeight, rightX: x, strafeWeight };
}

export function gaitStrides(settings, forward, turn, strafe = 0) {
  const strides = {};
  let largest = 0;
  const labels = ["FL", "FR", "BL", "BR"];
  // One shared turn rate: each foot sweeps in proportion to its own radius.
  const radius = Math.max(1, labels.reduce((sum, label) => {
    const { x, y } = footOffsetFromCentre(label, settings);
    return sum + Math.hypot(x, y);
  }, 0) / labels.length);
  for (const label of labels) {
    const { x, y } = footOffsetFromCentre(label, settings);
    // Strafe right (positive) moves every planted foot left relative to the body.
    const stride = {
      x: settings.strideMm * (forward + turn * settings.turnGain * y / radius),
      y: settings.strideMm * (turn * settings.turnGain * -x / radius - strafe * GAIT_LATERAL_STRIDE_SCALE),
    };
    strides[label] = stride;
    largest = Math.max(largest, Math.hypot(stride.x, stride.y));
  }
  const limit = settings.strideMm * 1.05;
  if (largest > limit) {
    const scale = limit / largest;
    Object.values(strides).forEach((stride) => { stride.x *= scale; stride.y *= scale; });
  }
  return strides;
}

// Body motion that a set of strides produces: forward speed (mm/s) and turn
// rate (deg/s, clockwise positive), from how far a planted foot travels per
// stance phase.
export function gaitBodyVelocity(settings, strides, stanceFraction) {
  const stanceSeconds = Math.max(0.05, stanceFraction / Math.max(0.05, settings.cadenceHz));
  const fl = footOffsetFromCentre("FL", settings);
  const average = (key) => ["FL", "FR", "BL", "BR"].reduce((sum, label) => sum + strides[label][key], 0) / 4;
  const forwardMmPerSec = average("x") / stanceSeconds;
  // Sideways: turning moves front and rear feet opposite ways, so the average
  // is the sidestep alone. Positive is to the right.
  const rightMmPerSec = -average("y") / stanceSeconds;
  // The left-minus-right forward difference is the turning part of the stride;
  // it equals the turn rate times the feet's lateral offset.
  const turnMm = ((strides.FL.x + strides.BL.x) - (strides.FR.x + strides.BR.x)) / 4;
  const yawRadPerSec = (turnMm / stanceSeconds) / Math.abs(fl.y);
  return { forwardMmPerSec, rightMmPerSec, yawDegPerSec: yawRadPerSec / DEG };
}

// Foot offset (leg frame) that moves the body away from the lifting leg during
// the careful crawl, blending to the next leg in the four-feet-down gap.
function crawlBodyShift(settings, cycle, swingFraction, amplitude) {
  const gap = CRAWL_SLOT - swingFraction;
  const t = wrap01(cycle);
  const slot = Math.floor(t / CRAWL_SLOT) % 4;
  const inSlot = t - slot * CRAWL_SLOT;
  let from = slot;
  let to = slot;
  let blend = 0;
  if (inSlot < gap / 2) {
    from = (slot + 3) % 4;
    blend = 0.5 + 0.5 * (inSlot / (gap / 2));
  } else if (inSlot > CRAWL_SLOT - gap / 2) {
    to = (slot + 1) % 4;
    blend = 0.5 * (inSlot - (CRAWL_SLOT - gap / 2)) / (gap / 2);
  }
  const direction = (label) => {
    const { x, y } = footOffsetFromCentre(label, settings);
    const radius = Math.max(1, Math.hypot(x, y));
    return { x: x / radius, y: y / radius };
  };
  const a = direction(CAREFUL_ORDER[from]);
  const b = direction(CAREFUL_ORDER[to]);
  const eased = 0.5 - 0.5 * Math.cos(Math.PI * clamp(blend, 0, 1));
  return {
    x: amplitude * CRAWL_BODY_SHIFT_MM * (a.x + (b.x - a.x) * eased),
    y: amplitude * CRAWL_BODY_SHIFT_MM * (a.y + (b.y - a.y) * eased),
  };
}

export const GAIT_LAB_STORAGE_KEY = "domino-gait-lab-v1";

// Tuned with gait-quality.mjs (physics with the display assists off, so the
// legs carry the robot): higher foot lift so the swing foot clears the floor,
// a wider stance so the body does not roll onto the swing side and drift, and
// a slightly lower body. Old Balanced reached 30% of its commanded speed and
// drifted 61 mm sideways per metre; this one reaches 60% with ~4 mm/m.
export const gaitLabPresets = Object.freeze({
  stable: {
    cadenceHz: 1.0,
    strideMm: 70,
    liftMm: 42,
    dutyFactor: 0.68,
    bodyHeightMm: 250,
    stanceWidthMm: 60,
    turnGain: 1.1,
    responseMs: 280,
    swingShape: 1.8,
    diagonalPhase: 0.5,
    touchdownXMm: -20,
    maxForwardScale: 0.75,
    maxTurnScale: 0.8,
  },
  balanced: {
    cadenceHz: 1.3,
    strideMm: 88,
    liftMm: 45,
    dutyFactor: 0.62,
    bodyHeightMm: 245,
    stanceWidthMm: 60,
    turnGain: 1.2,
    responseMs: 180,
    swingShape: 1.7,
    diagonalPhase: 0.5,
    touchdownXMm: -20,
    maxForwardScale: 1,
    maxTurnScale: 1,
  },
  fast: {
    cadenceHz: 1.4,
    strideMm: 104,
    liftMm: 46,
    dutyFactor: 0.62,
    bodyHeightMm: 245,
    stanceWidthMm: 60,
    turnGain: 1.2,
    responseMs: 150,
    swingShape: 1.7,
    diagonalPhase: 0.5,
    touchdownXMm: -20,
    maxForwardScale: 1,
    maxTurnScale: 1,
  },
});

export const defaultGaitLabSettings = Object.freeze({
  enabled: true,
  preset: "balanced",
  ...gaitLabPresets.balanced,
});

export const gaitLabControls = Object.freeze([
  {
    key: "cadenceHz", label: "Cadence", min: 0.35, max: 2.5, step: 0.05, unit: "Hz", decimals: 2,
    description: "Complete gait cycles per second. Higher values move the legs faster, but demand quicker servo response and leave less recovery time.",
  },
  {
    key: "strideMm", label: "Stride", min: 24, max: 120, step: 2, unit: "mm", decimals: 0,
    description: "Front-to-back foot travel during each cycle. A longer stride increases speed, but can approach the linkage limits or cause slipping.",
  },
  {
    key: "liftMm", label: "Clearance", min: 8, max: 70, step: 1, unit: "mm", decimals: 0,
    description: "Peak height of a swinging foot above its stance path. Increase it for obstacles; reduce it for smoother, more efficient flat-ground motion.",
  },
  {
    key: "dutyFactor", label: "Ground time", min: 0.5, max: 0.82, step: 0.01, unit: "%", scale: 100, decimals: 0,
    description: "Percentage of each cycle that a foot remains planted. Higher values improve support and stability; lower values permit a quicker gait.",
  },
  {
    key: "bodyHeightMm", label: "Gait height", min: 220, max: 280, step: 1, unit: "mm", decimals: 0,
    description: "Target body-to-foot distance while walking. Lower values crouch the robot; higher values increase clearance but reduce available leg travel.",
  },
  {
    key: "stanceWidthMm", label: "Stance width", min: 34, max: 70, step: 1, unit: "mm", decimals: 0,
    description: "Lateral distance of each foot from the body centreline. Wider is generally more stable, but requires more shoulder travel.",
  },
  {
    key: "turnGain", label: "Turn gain", min: 0, max: 1.5, step: 0.05, unit: "x", decimals: 2,
    description: "Scales yaw input into left-versus-right stride difference. Higher values turn more sharply and can destabilise fast forward motion.",
  },
  {
    key: "responseMs", label: "Response", min: 60, max: 700, step: 10, unit: "ms", decimals: 0,
    description: "Input and setting smoothing time. Lower values react quickly but can snap; higher values transition more gently with additional lag.",
  },
  {
    key: "swingShape", label: "Swing shape", min: 0.8, max: 3, step: 0.1, unit: "", decimals: 1,
    description: "Shapes the vertical swing arc. Higher values narrow the lift around mid-swing; lower values create a broader, rounder foot arc.",
  },
  {
    key: "diagonalPhase", label: "Diagonal phase", min: 0.4, max: 0.6, step: 0.01, unit: "%", scale: 100, decimals: 0,
    description: "Timing offset between the two diagonal leg pairs in trot. Fifty percent gives even alternation; offsets bias the contact timing.",
  },
  {
    key: "touchdownXMm", label: "Foot X neutral", min: -35, max: 10, step: 1, unit: "mm", decimals: 0, expert: true,
    description: "Nominal fore-aft foot position below each hip. Negative values place the feet behind the hip; preview IK must remain valid before apply.",
  },
  {
    key: "maxForwardScale", label: "Forward limit", min: 0.2, max: 1, step: 0.05, unit: "%", scale: 100, decimals: 0, expert: true,
    description: "Maximum fraction of forward/reverse stick accepted by this profile. Use a reduced limit for first physical tests.",
  },
  {
    key: "maxTurnScale", label: "Turn limit", min: 0.2, max: 1, step: 0.05, unit: "%", scale: 100, decimals: 0, expert: true,
    description: "Maximum fraction of yaw stick accepted by this profile. This bounds turning before turn gain and stride are applied.",
  },
]);

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function wrap01(value) {
  return value - Math.floor(value);
}

function smooth01(value) {
  const t = clamp(value, 0, 1);
  return 0.5 - 0.5 * Math.cos(Math.PI * t);
}

function transitionEase(value) {
  const t = clamp(value, 0, 1);
  return t ** 3 * (10 + t * (-15 + 6 * t));
}

function transitionDuration(source, target) {
  const distance = Math.max(0, ...target.leg_command_xyz_mm.map((command, index) =>
    Math.hypot(...command.map((value, axis) =>
      value - (source.leg_command_xyz_mm?.[index]?.[axis] ?? value)))));
  const angle = Math.max(0, ...(source.body_pose_rpy_deg || [0, 0, 0]).map(Math.abs));
  return Math.max(0.35, 2.5 * distance / 80, 1.875 * angle / 25);
}

function approach(current, target, blend) {
  return current + (target - current) * blend;
}

function mixNumber(source, target, blend) {
  const a = Number(source);
  const b = Number(target);
  if (!Number.isFinite(a)) return Number.isFinite(b) ? b : source;
  if (!Number.isFinite(b)) return a;
  return approach(a, b, blend);
}

function mixVector(source, target, blend) {
  if (!Array.isArray(source) || !Array.isArray(target)) return target ?? source;
  return target.map((value, index) => mixNumber(source[index], value, blend));
}

function mixMotionState(baseState, sourceState, targetState, blend) {
  const amount = transitionEase(blend);
  const output = {
    ...baseState,
    servo_angle_deg: mixVector(
      sourceState.servo_angle_deg,
      targetState.servo_angle_deg,
      amount,
    ),
    leg_command_xyz_mm: targetState.leg_command_xyz_mm.map((command, index) =>
      mixVector(sourceState.leg_command_xyz_mm?.[index], command, amount)),
    body_pose_rpy_deg: mixVector(
      sourceState.body_pose_rpy_deg,
      targetState.body_pose_rpy_deg,
      amount,
    ),
    pose_z_mm: mixNumber(sourceState.pose_z_mm, targetState.pose_z_mm, amount),
    target_z_mm: mixNumber(sourceState.target_z_mm, targetState.target_z_mm, amount),
    ride_height_mm: mixNumber(sourceState.ride_height_mm, targetState.ride_height_mm, amount),
    gait_phase_rad: targetState.gait_phase_rad,
    gait_command: targetState.gait_command,
    gait_lab: { ...targetState.gait_lab, transition: amount },
  };
  // Solve the blended endpoints. Interpolating angles independently would
  // give the renderer a different foot path from the one used by physics.
  for (const leg of legs) {
    const upper = (output.servo_angle_deg[leg.channels.upper] - standServoReference[leg.channels.upper]) /
      leg.directions.upper;
    const lower = (output.servo_angle_deg[leg.channels.lower] - standServoReference[leg.channels.lower]) /
      leg.directions.lower;
    const solved = solveCadEndpoint(leg, output.leg_command_xyz_mm[COMMAND_INDEX[leg.label]], { upper, lower });
    if (!solved.reachable) continue;
    for (const joint of ["shoulder", "upper", "lower"]) {
      const channel = leg.channels[joint];
      output.servo_angle_deg[channel] = standServoReference[channel] + leg.directions[joint] * solved[joint];
    }
  }
  return output;
}

function transformedFoot(points, solution) {
  const originalA = point2(points.lower_closure_diagonal);
  const originalB = point2(points.lower_closure_driver);
  const foot = point2(points.foot_tip);
  const originalAngle = Math.atan2(originalB.z - originalA.z, originalB.x - originalA.x);
  const currentAngle = Math.atan2(
    solution.lowerClosureDiagonal.z - solution.lowerClosureDriver.z,
    solution.lowerClosureDiagonal.x - solution.lowerClosureDriver.x,
  );
  const angle = currentAngle - originalAngle;
  const localX = foot.x - originalA.x;
  const localZ = foot.z - originalA.z;
  return {
    x: solution.lowerClosureDriver.x + Math.cos(angle) * localX - Math.sin(angle) * localZ,
    z: solution.lowerClosureDriver.z + Math.sin(angle) * localX + Math.cos(angle) * localZ,
  };
}

function footForServoDeltas(leg, upperDeltaDeg, lowerDeltaDeg) {
  const offset = leg.neutralLinkageOffsetDeg || { upper: 0, lower: 0 };
  const solution = solveLinkagePose(
    leg.points,
    (-upperDeltaDeg + offset.upper) * DEG,
    (-lowerDeltaDeg + offset.lower) * DEG,
  );
  const foot = transformedFoot(leg.points, solution);
  return {
    x: (foot.x - leg.points.hip_origin[0]) * 1000,
    z: (foot.z - leg.points.hip_origin[2]) * 1000,
  };
}

function solveCadPlanar(leg, targetX, targetZ, seed) {
  let upper = seed.upper;
  let lower = seed.lower;
  const finiteDifferenceDeg = 0.05;

  for (let iteration = 0; iteration < 10; iteration += 1) {
    const current = footForServoDeltas(leg, upper, lower);
    const upperStep = footForServoDeltas(leg, upper + finiteDifferenceDeg, lower);
    const lowerStep = footForServoDeltas(leg, upper, lower + finiteDifferenceDeg);
    const errorX = targetX - current.x;
    const errorZ = targetZ - current.z;
    if (Math.hypot(errorX, errorZ) <= 0.05) {
      return { upper, lower, foot: current, reachable: true };
    }

    const j00 = (upperStep.x - current.x) / finiteDifferenceDeg;
    const j10 = (upperStep.z - current.z) / finiteDifferenceDeg;
    const j01 = (lowerStep.x - current.x) / finiteDifferenceDeg;
    const j11 = (lowerStep.z - current.z) / finiteDifferenceDeg;
    const determinant = j00 * j11 - j01 * j10;
    if (Math.abs(determinant) < 1e-6) break;
    const stepUpper = (errorX * j11 - j01 * errorZ) / determinant;
    const stepLower = (j00 * errorZ - errorX * j10) / determinant;
    upper = clamp(upper + clamp(stepUpper, -8, 8), -MAX_SERVO_DELTA_DEG, MAX_SERVO_DELTA_DEG);
    lower = clamp(lower + clamp(stepLower, -8, 8), -MAX_SERVO_DELTA_DEG, MAX_SERVO_DELTA_DEG);
  }

  const foot = footForServoDeltas(leg, upper, lower);
  return { upper, lower, foot, reachable: Math.hypot(targetX - foot.x, targetZ - foot.z) <= 1 };
}

function solveCadEndpoint(leg, command, seed) {
  const side = leg.label.endsWith("L") ? 1 : -1;
  const neutralFoot = footForServoDeltas(leg, 0, 0);
  const neutralY = side * CAD_FOOT_Y_MM;
  const targetX = neutralFoot.x + (command[0] - COMMAND_X_MM);
  const targetY = neutralY + (command[1] - side * COMMAND_Y_MM);
  const targetZ = neutralFoot.z - (command[2] - COMMAND_Z_MM);
  const targetRadius = Math.hypot(targetY, targetZ);
  const targetPlanarZ = -Math.sqrt(Math.max(targetRadius ** 2 - neutralY ** 2, 0));
  const planar = solveCadPlanar(leg, targetX, targetPlanarZ, seed);
  const baseAngle = Math.atan2(planar.foot.z, neutralY);
  const targetAngle = Math.atan2(targetZ, targetY);
  let rootAngle = targetAngle - baseAngle;
  while (rootAngle > Math.PI) rootAngle -= TWO_PI;
  while (rootAngle < -Math.PI) rootAngle += TWO_PI;
  return {
    shoulder: clamp(-rootAngle / DEG / side, -MAX_SERVO_DELTA_DEG, MAX_SERVO_DELTA_DEG),
    upper: planar.upper,
    lower: planar.lower,
    reachable: planar.reachable,
  };
}

function smootherstep01(value) {
  const t = clamp(value, 0, 1);
  return t * t * t * (t * (6 * t - 15) + 10);
}

// Swing foot path for the trot (keep in step with sampleGaitFootPath in
// src/main.cpp). `swing` runs 0..1 over the airborne part of the cycle.
//  - Height: the foot leaves the floor fast, holds its height through the
//    middle of the step and comes down over the last part. The old sin^shape
//    curve crept off the floor, so feet scraped at the start and end of every
//    step. `swingShape` sets how much of the step the rise and fall take.
//  - Fore/aft: the foot leaves and lands moving rearward at the stance speed,
//    i.e. standing still over the ground (cubic Hermite with matched end
//    velocities). The old curve stopped the foot relative to the body at both
//    ends, so it hit the floor moving forward at walking speed and skidded.
export function trotSwingPath(swing, halfStride, lift, dutyFactor, swingShape) {
  const s = clamp(swing, 0, 1);
  const rise = clamp(0.18 * swingShape, 0.15, 0.45);
  const height = Math.min(smootherstep01(s / rise), smootherstep01((1 - s) / rise));
  const duty = clamp(dutyFactor, 0.3, 0.95);
  // Stance travel per unit of swing time, in half strides (rearward).
  const groundSpeed = -2 * (1 - duty) / duty;
  const s2 = s * s;
  const s3 = s2 * s;
  const h00 = 2 * s3 - 3 * s2 + 1;
  const h10 = s3 - 2 * s2 + s;
  const h01 = -2 * s3 + 3 * s2;
  const h11 = s3 - s2;
  return {
    x: halfStride * (-h00 + groundSpeed * h10 + h01 + groundSpeed * h11),
    z: -lift * height,
  };
}

function sampleTrot(cycle, halfStride, lift, dutyFactor, swingShape) {
  if (cycle < dutyFactor) {
    const stance = cycle / dutyFactor;
    return { x: halfStride * (1 - 2 * stance), z: 0, stance: true };
  }
  const swing = (cycle - dutyFactor) / (1 - dutyFactor);
  return { ...trotSwingPath(swing, halfStride, lift, dutyFactor, swingShape), stance: false };
}

function sampleCareful(cycle, halfStride, lift, dutyFactor, swingShape) {
  const swingFraction = Math.min(CRAWL_MAX_SWING_FRACTION, 1 - dutyFactor);
  if (cycle < swingFraction) {
    const swing = cycle / swingFraction;
    return {
      x: halfStride * (-1 + 2 * smooth01(swing)),
      z: -lift * Math.pow(Math.max(0, Math.sin(Math.PI * swing)), swingShape),
      stance: false,
    };
  }
  const stance = (cycle - swingFraction) / (1 - swingFraction);
  return { x: halfStride * (1 - 2 * stance), z: 0, stance: true };
}

export function sanitizeGaitLabSettings(candidate = {}) {
  const presetDefaults = Object.hasOwn(gaitLabPresets, candidate.preset)
    ? gaitLabPresets[candidate.preset]
    : defaultGaitLabSettings;
  const settings = { ...defaultGaitLabSettings, ...presetDefaults, ...candidate };
  gaitLabControls.forEach(({ key, min, max }) => {
    const numericValue = Number(settings[key]);
    settings[key] = clamp(
      Number.isFinite(numericValue) ? numericValue : defaultGaitLabSettings[key],
      min,
      max,
    );
  });
  settings.enabled = settings.enabled !== false;
  settings.dutyFactor = Math.max(settings.dutyFactor,
    Math.max(settings.diagonalPhase, 1 - settings.diagonalPhase) + 0.02);
  settings.preset = Object.hasOwn(gaitLabPresets, settings.preset) ? settings.preset : "custom";
  return settings;
}

export function createGaitLab(initialSettings = {}) {
  let settings = sanitizeGaitLabSettings(initialSettings);
  let motionSettings = { ...settings };
  let phase = 0;
  let filteredForward = 0;
  let filteredTurn = 0;
  let filteredStrafe = 0;
  let transitionProgress = 1;
  let transitionFromState = null;
  let lastGaitState = null;
  let lastOutputState = null;
  let activeMode = null;
  let recovery = null;
  let nativeWasTransitioning = false;
  let waitForNative = false;
  let handoffMode = null;
  const seeds = Object.fromEntries(legs.map((leg) => [leg.label, { upper: 0, lower: 0 }]));
  let telemetry = { active: false, phase: 0, speedMmPerSec: 0, stanceCount: 4, reachableCount: 4, legDetails: [] };

  function setSettings(nextSettings) {
    settings = sanitizeGaitLabSettings(nextSettings);
  }

  function update(deltaSeconds, sourceState, input = {}) {
    if (!sourceState) return sourceState;
    const delta = clamp(Number(deltaSeconds) || 0, 0, 0.05);
    const responseSeconds = Math.max(0.06, settings.responseMs / 1000);
    let transitionSeconds = Math.max(0.4, responseSeconds * 1.5);
    const walking = sourceState.mode === "GAIT" || sourceState.mode === "CAREFUL";
    const active = settings.enabled && walking && !sourceState.tilt_active;
    const nativeTransitioning = sourceState.transition_active === true;
    const pendingMode = nativeTransitioning ? sourceState.requested_mode : sourceState.mode;
    const nativeTransitionStarted = nativeTransitioning && !nativeWasTransitioning;
    nativeWasTransitioning = nativeTransitioning;

    function beginRecovery(stage, from) {
      const landing = stage === "land";
      const height = landing || pendingMode === "STOW" ? from.pose_z_mm
        : Number(sourceState.ride_height_mm ?? sourceState.pose_z_mm);
      const target = {
        ...sourceState,
        mode: "TRANSITION",
        body_pose_rpy_deg: [0, 0, 0],
        pose_z_mm: height,
        target_z_mm: height,
        leg_command_xyz_mm: from.leg_command_xyz_mm.map((command, index) =>
          landing ? [command[0], command[1], height]
            : [COMMAND_X_MM, index === 0 || index === 2 ? COMMAND_Y_MM : -COMMAND_Y_MM, height]),
      };
      recovery = { stage, from, target, elapsed: 0, duration: transitionDuration(from, target) };
    }

    const switchingGait = activeMode !== null && activeMode !== sourceState.mode;
    const leavingTilt = active && lastOutputState?.mode === "TILT" && handoffMode !== sourceState.mode;
    if (!recovery && (switchingGait || leavingTilt || nativeTransitionStarted)) {
      const from = lastOutputState || sourceState;
      beginRecovery(activeMode !== null ? "land" : "neutral", from);
      activeMode = null;
      phase = filteredForward = filteredTurn = filteredStrafe = 0;
      transitionFromState = null;
      transitionProgress = 0;
      handoffMode = null;
    }
    if (recovery) {
      recovery.elapsed += delta;
      const output = mixMotionState(sourceState, recovery.from, recovery.target,
        recovery.elapsed / recovery.duration);
      output.mode = "TRANSITION";
      output.transition_active = true;
      output.requested_mode = pendingMode;
      output.transition_stage = recovery.stage === "land" ? 1 : 2;
      output.tilt_active = output.gait_active = false;
      output.motion_input_armed = false;
      output.gait_command = [0, 0];
      telemetry = { ...telemetry, active: false, speedMmPerSec: 0, transition: 0 };
      output.gait_lab = telemetry;
      lastOutputState = output;
      if (recovery.elapsed >= recovery.duration) {
        if (recovery.stage === "land") {
          beginRecovery("neutral", output);
        } else if (pendingMode !== "STOW" &&
                   Math.abs(output.pose_z_mm - Number(sourceState.ride_height_mm ?? sourceState.pose_z_mm)) > 0.5) {
          beginRecovery("neutral", output);
        } else {
          recovery = null;
          handoffMode = pendingMode;
          waitForNative = nativeTransitioning;
          transitionFromState = output;
          transitionProgress = 0;
        }
      }
      return output;
    }
    if (waitForNative && nativeTransitioning) return lastOutputState;
    waitForNative = false;
    if (!active) {
      if (activeMode !== null) {
        transitionFromState = lastOutputState || lastGaitState || sourceState;
        transitionProgress = 0;
        activeMode = null;
      }
      if (transitionFromState && transitionProgress < 1) {
        transitionSeconds = Math.max(transitionSeconds, transitionDuration(transitionFromState, sourceState));
        transitionProgress = Math.min(1, transitionProgress + delta / transitionSeconds);
        telemetry = {
          ...telemetry,
          active: false,
          speedMmPerSec: 0,
          transition: 1 - smooth01(transitionProgress),
        };
        const transitioningState = mixMotionState(
          sourceState,
          transitionFromState,
          sourceState,
          transitionProgress,
        );
        transitioningState.gait_lab = telemetry;
        lastOutputState = transitioningState;
        if (transitionProgress >= 1) {
          transitionFromState = null;
          lastGaitState = null;
        }
        return transitioningState;
      }
      motionSettings = { ...settings };
      telemetry = {
        ...telemetry,
        active: false,
        speedMmPerSec: 0,
        stanceCount: 4,
        transition: 0,
      };
      lastOutputState = sourceState;
      lastGaitState = null;
      return sourceState;
    }

    if (activeMode !== sourceState.mode) {
      const enteringWalk = activeMode === null;
      transitionFromState = lastOutputState || sourceState;
      transitionProgress = 0;
      activeMode = sourceState.mode;
      if (enteringWalk) phase = 0;
    }
    const stanceTarget = {
      ...sourceState,
      body_pose_rpy_deg: [0, 0, 0],
      leg_command_xyz_mm: Array.from({ length: 4 }, (_, index) => [
        settings.touchdownXMm,
        (index === 0 || index === 2 ? 1 : -1) * settings.stanceWidthMm,
        Math.min(Number(sourceState.ride_height_mm ?? sourceState.pose_z_mm), settings.bodyHeightMm),
      ]),
    };
    transitionSeconds = Math.max(transitionSeconds,
      transitionDuration(transitionFromState || sourceState, stanceTarget));
    transitionProgress = Math.min(1, transitionProgress + delta / transitionSeconds);
    const settingsBlend = 1 - Math.exp(-delta / Math.max(0.12, motionSettings.responseMs / 1000));
    gaitLabControls.forEach(({ key }) => {
      motionSettings[key] = approach(motionSettings[key], settings[key], settingsBlend);
    });
    const blend = 1 - Math.exp(-delta / responseSeconds);
    const motionAllowed = sourceState.motion_input_armed !== false &&
      sourceState.link_alive !== false && !sourceState.transition_active && transitionProgress >= 1;
    // Left stick X spins on the spot. Right stick X sidesteps at a standstill
    // and hands over to steering as forward stick grows (splitRightStickX).
    // The trot reads the right stick's direction (rightStickIntent): a mostly
    // sideways push is a pure sidestep. The careful crawl keeps plain
    // speed-scaled steering.
    const careful = sourceState.mode === "CAREFUL";
    const rawForward = clamp(Number(input.forward) || 0, -1, 1);
    const intent = careful
      ? { forward: rawForward, rightX: clamp(Number(input.steer) || 0, -1, 1), strafeWeight: 1 }
      : rightStickIntent(rawForward, input.steer);
    const forwardInput = intent.forward;
    filteredForward = approach(filteredForward, motionAllowed
      ? forwardInput * motionSettings.maxForwardScale : 0, blend);
    const rightStick = careful
      ? { steer: intent.rightX * Math.abs(forwardInput), strafe: 0 }
      : splitRightStickX(forwardInput, intent.rightX, input.steerBlend ?? 1, intent.strafeWeight);
    const turnInput = clamp((Number(input.turn) || 0) + rightStick.steer, -1, 1);
    filteredTurn = approach(filteredTurn, motionAllowed ? turnInput * motionSettings.maxTurnScale : 0, blend);
    filteredStrafe = approach(filteredStrafe,
      motionAllowed ? clamp(rightStick.strafe, -1, 1) * motionSettings.maxForwardScale : 0, blend);
    const activity = clamp(Math.hypot(filteredForward, filteredTurn, filteredStrafe), 0, 1);
    if (activity > 0.02) {
      phase = wrap01(
        phase + motionSettings.cadenceHz * (0.35 + 0.65 * activity) * delta,
      );
    }

    const directionScale = activity > 0.001 ? 1 / activity : 0;
    const forwardDirection = filteredForward * directionScale;
    const turnDirection = filteredTurn * directionScale;
    const strafeDirection = filteredStrafe * directionScale;
    const amplitude = smooth01(clamp(activity / 0.22, 0, 1)) * (0.62 + 0.38 * activity);
    const commands = Array.from(
      { length: 4 },
      () => [COMMAND_X_MM, 0, motionSettings.bodyHeightMm],
    );
    const servoAngles = Array.isArray(sourceState.servo_angle_deg)
      ? [...sourceState.servo_angle_deg]
      : [...standServoReference];
    let stanceCount = 0;
    let reachableCount = 0;
    const legDetails = [];
    const bodyHeight = Math.min(Number(sourceState.ride_height_mm ?? sourceState.pose_z_mm), motionSettings.bodyHeightMm);
    const strides = gaitStrides(motionSettings, amplitude * forwardDirection, amplitude * turnDirection,
      amplitude * strafeDirection);
    const swingFraction = Math.min(CRAWL_MAX_SWING_FRACTION, 1 - motionSettings.dutyFactor);
    const crawlGap = CRAWL_SLOT - swingFraction;
    const shift = careful ? crawlBodyShift(motionSettings, phase, swingFraction, amplitude) : { x: 0, y: 0 };
    const bodyVelocity = gaitBodyVelocity(
      motionSettings, strides, careful ? 1 - swingFraction : motionSettings.dutyFactor);
    const cycleSpeed = activity > 0.02 ? 0.35 + 0.65 * activity : 0;

    for (const leg of legs) {
      const left = leg.label.endsWith("L");
      const side = left ? 1 : -1;
      let cycle;
      let foot;
      if (careful) {
        const orderIndex = CAREFUL_ORDER.indexOf(leg.label);
        cycle = wrap01(phase - orderIndex * CRAWL_SLOT - crawlGap / 2);
        foot = sampleCareful(cycle, 1, amplitude * motionSettings.liftMm,
          motionSettings.dutyFactor, motionSettings.swingShape);
      } else {
        const diagonalA = leg.label === "FL" || leg.label === "BR";
        cycle = wrap01(phase + (diagonalA ? 0 : motionSettings.diagonalPhase));
        foot = sampleTrot(cycle, 1, amplitude * motionSettings.liftMm,
          motionSettings.dutyFactor, motionSettings.swingShape);
      }
      if (foot.stance) stanceCount += 1;
      const stride = strides[leg.label];
      const command = [
        motionSettings.touchdownXMm + stride.x / 2 * foot.x + shift.x,
        side * motionSettings.stanceWidthMm + stride.y / 2 * foot.x + shift.y,
        bodyHeight + foot.z,
      ];
      commands[COMMAND_INDEX[leg.label]] = command;
      const solved = solveCadEndpoint(leg, command, seeds[leg.label]);
      if (solved.reachable) reachableCount += 1;
      seeds[leg.label] = { upper: solved.upper, lower: solved.lower };
      for (const joint of ["shoulder", "upper", "lower"]) {
        const channel = leg.channels[joint];
        servoAngles[channel] = standServoReference[channel] + leg.directions[joint] * solved[joint];
      }
      const jointDeltaDeg = {
        shoulder: solved.shoulder,
        upper: solved.upper,
        lower: solved.lower,
      };
      legDetails.push({
        leg: leg.label,
        targetMm: [...command],
        reachable: solved.reachable,
        stance: foot.stance,
        jointDeltaDeg,
        limitedJoints: Object.entries(jointDeltaDeg)
          .filter(([, value]) => Math.abs(value) >= MAX_SERVO_DELTA_DEG - 0.01)
          .map(([joint]) => joint),
      });
    }

    telemetry = {
      active: true,
      phase,
      speedMmPerSec: motionSettings.cadenceHz * motionSettings.strideMm * activity,
      // The preview phase advances at cadence * cycleSpeed; scale the body
      // motion the same way so the floor and path match the feet.
      bodyVelocity: {
        forwardMmPerSec: bodyVelocity.forwardMmPerSec * cycleSpeed,
        rightMmPerSec: bodyVelocity.rightMmPerSec * cycleSpeed,
        yawDegPerSec: bodyVelocity.yawDegPerSec * cycleSpeed,
      },
      stanceCount,
      reachableCount,
      legDetails,
      activity,
      transition: smooth01(transitionProgress),
    };
    lastGaitState = {
      ...sourceState,
      servo_angle_deg: servoAngles,
      leg_command_xyz_mm: commands,
      body_pose_rpy_deg: [0, 0, 0],
      pose_z_mm: bodyHeight,
      target_z_mm: bodyHeight,
      gait_phase_rad: phase * TWO_PI,
      gait_command: [filteredForward, filteredTurn],
      gait_lab: telemetry,
    };
    const outputState = mixMotionState(
      sourceState,
      transitionFromState || sourceState,
      lastGaitState,
      transitionProgress,
    );
    lastOutputState = outputState;
    if (transitionProgress >= 1) transitionFromState = null;
    return outputState;
  }

  return {
    update,
    setSettings,
    getSettings: () => ({ ...settings }),
    getTelemetry: () => ({
      ...telemetry,
      legDetails: telemetry.legDetails.map((detail) => ({
        ...detail,
        targetMm: [...detail.targetMm],
        jointDeltaDeg: { ...detail.jointDeltaDeg },
        limitedJoints: [...detail.limitedJoints],
      })),
    }),
    reset: () => {
      phase = 0;
      filteredForward = 0;
      filteredTurn = 0;
      filteredStrafe = 0;
      motionSettings = { ...settings };
      transitionProgress = 1;
      transitionFromState = null;
      lastGaitState = null;
      lastOutputState = null;
      activeMode = null;
      recovery = null;
      nativeWasTransitioning = waitForNative = false;
      handoffMode = null;
      Object.values(seeds).forEach((seed) => {
        seed.upper = 0;
        seed.lower = 0;
      });
    },
  };
}
