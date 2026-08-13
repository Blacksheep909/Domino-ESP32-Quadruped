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
const CAREFUL_ORDER = ["FL", "BR", "FR", "BL"];

export const GAIT_LAB_STORAGE_KEY = "domino-gait-lab-v1";

export const gaitLabPresets = Object.freeze({
  stable: {
    cadenceHz: 0.78,
    strideMm: 50,
    liftMm: 25,
    dutyFactor: 0.72,
    bodyHeightMm: 255,
    stanceWidthMm: 42,
    turnGain: 0.72,
    responseMs: 360,
    swingShape: 2.0,
    diagonalPhase: 0.5,
  },
  balanced: {
    cadenceHz: 1.25,
    strideMm: 72,
    liftMm: 34,
    dutyFactor: 0.64,
    bodyHeightMm: 260,
    stanceWidthMm: 40,
    turnGain: 0.9,
    responseMs: 220,
    swingShape: 1.7,
    diagonalPhase: 0.5,
  },
  fast: {
    cadenceHz: 1.85,
    strideMm: 92,
    liftMm: 39,
    dutyFactor: 0.58,
    bodyHeightMm: 250,
    stanceWidthMm: 42,
    turnGain: 1.05,
    responseMs: 130,
    swingShape: 1.45,
    diagonalPhase: 0.5,
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
  const amount = smooth01(clamp(blend, 0, 1));
  return {
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

function sampleTrot(cycle, halfStride, lift, dutyFactor, swingShape) {
  if (cycle < dutyFactor) {
    const stance = cycle / dutyFactor;
    return { x: halfStride * (1 - 2 * stance), z: 0, stance: true };
  }
  const swing = (cycle - dutyFactor) / (1 - dutyFactor);
  const liftWave = Math.sin(Math.PI * swing);
  return {
    x: halfStride * (-1 + 2 * smooth01(swing)),
    z: -lift * Math.pow(Math.max(0, liftWave), swingShape),
    stance: false,
  };
}

function sampleCareful(cycle, halfStride, lift, dutyFactor, swingShape) {
  const swingFraction = Math.min(0.245, 1 - dutyFactor);
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
  const settings = { ...defaultGaitLabSettings, ...candidate };
  gaitLabControls.forEach(({ key, min, max }) => {
    const numericValue = Number(settings[key]);
    settings[key] = clamp(
      Number.isFinite(numericValue) ? numericValue : defaultGaitLabSettings[key],
      min,
      max,
    );
  });
  settings.enabled = settings.enabled !== false;
  settings.preset = Object.hasOwn(gaitLabPresets, settings.preset) ? settings.preset : "custom";
  return settings;
}

export function createGaitLab(initialSettings = {}) {
  let settings = sanitizeGaitLabSettings(initialSettings);
  let motionSettings = { ...settings };
  let phase = 0;
  let filteredForward = 0;
  let filteredTurn = 0;
  let transitionProgress = 1;
  let transitionFromState = null;
  let lastGaitState = null;
  let lastOutputState = null;
  let activeMode = null;
  const seeds = Object.fromEntries(legs.map((leg) => [leg.label, { upper: 0, lower: 0 }]));
  let telemetry = { active: false, phase: 0, speedMmPerSec: 0, stanceCount: 4, reachableCount: 4 };

  function setSettings(nextSettings) {
    settings = sanitizeGaitLabSettings(nextSettings);
  }

  function update(deltaSeconds, sourceState, input = {}) {
    if (!sourceState) return sourceState;
    const delta = clamp(Number(deltaSeconds) || 0, 0, 0.05);
    const responseSeconds = Math.max(0.06, settings.responseMs / 1000);
    const transitionSeconds = Math.max(0.4, responseSeconds * 1.5);
    const walking = sourceState.mode === "GAIT" || sourceState.mode === "CAREFUL";
    const active = settings.enabled && walking && !sourceState.tilt_active;
    if (!active) {
      if (activeMode !== null) {
        transitionFromState = lastOutputState || lastGaitState || sourceState;
        transitionProgress = 0;
        activeMode = null;
      }
      if (transitionFromState && transitionProgress < 1) {
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
      // Careful and trot share the same command axes. Keeping their phase
      // continuous avoids snapping all four endpoints back to cycle zero
      // when SC moves between the two walking positions.
      if (enteringWalk) phase = 0;
    }
    transitionProgress = Math.min(1, transitionProgress + delta / transitionSeconds);
    const settingsBlend = 1 - Math.exp(-delta / Math.max(0.12, motionSettings.responseMs / 1000));
    gaitLabControls.forEach(({ key }) => {
      motionSettings[key] = approach(motionSettings[key], settings[key], settingsBlend);
    });
    const blend = 1 - Math.exp(-delta / responseSeconds);
    filteredForward = approach(filteredForward, clamp(Number(input.forward) || 0, -1, 1), blend);
    filteredTurn = approach(filteredTurn, clamp(Number(input.turn) || 0, -1, 1), blend);
    const activity = clamp(Math.hypot(filteredForward, filteredTurn), 0, 1);
    if (activity > 0.02) {
      phase = wrap01(
        phase + motionSettings.cadenceHz * (0.35 + 0.65 * activity) * delta,
      );
    }

    const directionScale = activity > 0.001 ? 1 / activity : 0;
    const forwardDirection = filteredForward * directionScale;
    const turnDirection = filteredTurn * directionScale;
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

    for (const leg of legs) {
      const left = leg.label.endsWith("L");
      const side = left ? 1 : -1;
      let cycle;
      let foot;
      if (sourceState.mode === "CAREFUL") {
        const orderIndex = CAREFUL_ORDER.indexOf(leg.label);
        cycle = wrap01(phase - orderIndex * 0.25);
        foot = sampleCareful(
          cycle,
          amplitude * motionSettings.strideMm / 2 *
            (forwardDirection + side * turnDirection * motionSettings.turnGain),
          amplitude * motionSettings.liftMm,
          motionSettings.dutyFactor,
          motionSettings.swingShape,
        );
      } else {
        const diagonalA = leg.label === "FL" || leg.label === "BR";
        cycle = wrap01(phase + (diagonalA ? 0 : motionSettings.diagonalPhase));
        foot = sampleTrot(
          cycle,
          amplitude * motionSettings.strideMm / 2 *
            (forwardDirection + side * turnDirection * motionSettings.turnGain),
          amplitude * motionSettings.liftMm,
          motionSettings.dutyFactor,
          motionSettings.swingShape,
        );
      }
      if (foot.stance) stanceCount += 1;
      const command = [
        COMMAND_X_MM + foot.x,
        side * motionSettings.stanceWidthMm,
        motionSettings.bodyHeightMm + foot.z,
      ];
      commands[COMMAND_INDEX[leg.label]] = command;
      const solved = solveCadEndpoint(leg, command, seeds[leg.label]);
      if (solved.reachable) reachableCount += 1;
      seeds[leg.label] = { upper: solved.upper, lower: solved.lower };
      for (const joint of ["shoulder", "upper", "lower"]) {
        const channel = leg.channels[joint];
        servoAngles[channel] = standServoReference[channel] + leg.directions[joint] * solved[joint];
      }
    }

    telemetry = {
      active: true,
      phase,
      speedMmPerSec: motionSettings.cadenceHz * motionSettings.strideMm * activity,
      stanceCount,
      reachableCount,
      activity,
      transition: smooth01(transitionProgress),
    };
    lastGaitState = {
      ...sourceState,
      servo_angle_deg: servoAngles,
      leg_command_xyz_mm: commands,
      pose_z_mm: motionSettings.bodyHeightMm,
      target_z_mm: motionSettings.bodyHeightMm,
      ride_height_mm: motionSettings.bodyHeightMm,
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
    getTelemetry: () => ({ ...telemetry }),
    reset: () => {
      phase = 0;
      filteredForward = 0;
      filteredTurn = 0;
      motionSettings = { ...settings };
      transitionProgress = 1;
      transitionFromState = null;
      lastGaitState = null;
      lastOutputState = null;
      activeMode = null;
      Object.values(seeds).forEach((seed) => {
        seed.upper = 0;
        seed.lower = 0;
      });
    },
  };
}
