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
    touchdownXMm: -15.75,
    maxForwardScale: 0.55,
    maxTurnScale: 0.45,
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
    touchdownXMm: -15.75,
    maxForwardScale: 0.8,
    maxTurnScale: 0.7,
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
    touchdownXMm: -15.75,
    maxForwardScale: 1,
    maxTurnScale: 0.9,
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
      phase = filteredForward = filteredTurn = 0;
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
    filteredForward = approach(filteredForward, motionAllowed
      ? clamp(Number(input.forward) || 0, -1, 1) * motionSettings.maxForwardScale : 0, blend);
    filteredTurn = approach(filteredTurn, motionAllowed
      ? clamp(Number(input.turn) || 0, -1, 1) * motionSettings.maxTurnScale : 0, blend);
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
    const legDetails = [];
    const bodyHeight = Math.min(Number(sourceState.ride_height_mm ?? sourceState.pose_z_mm), motionSettings.bodyHeightMm);

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
        motionSettings.touchdownXMm + foot.x,
        side * motionSettings.stanceWidthMm,
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
