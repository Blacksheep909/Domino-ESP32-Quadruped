// Gait quality measurement with real weight on the legs.
//
// Runs the Studio gait (gait-lab.js, which mirrors the firmware trot in
// src/main.cpp) through the Rapier physics with the Studio's display assists
// switched off, so the legs carry the robot and planted feet can slip. Reports
// the numbers that decide whether a walk looks and works well:
//   speed        actual forward speed vs. what the gait commands
//   drift        sideways travel and heading change per metre forward
//   slip         how far planted feet slide (forward/back and sideways)
//   wobble       body roll / pitch swing and vertical bob
//
// Usage: node gait-quality.mjs [forward=1] [seconds=8] [preset or JSON settings]
import * as THREE from "three";

import { legs, standServoReference } from "./web/src/domino-config.js";
import { createGaitLab, gaitLabPresets } from "./web/src/gait-lab.js";
import { createDominoPhysics } from "./web/src/physics.js";

const STEP = 1 / 120;

function standState(heightMm = 280) {
  return {
    mode: "STAND",
    pose_z_mm: heightMm,
    target_z_mm: heightMm,
    ride_height_mm: heightMm,
    tilt_active: false,
    body_pose_rpy_deg: [0, 0, 0],
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, heightMm], [-15.75, -38, heightMm],
      [-15.75, 38, heightMm], [-15.75, -38, heightMm],
    ],
  };
}

function rollPitchDeg(quaternion) {
  const q = new THREE.Quaternion(...quaternion);
  const euler = new THREE.Euler().setFromQuaternion(q, "YXZ");
  // World: +Y up, robot forward is +X. Roll about X, pitch about Z.
  return { roll: THREE.MathUtils.radToDeg(euler.x), pitch: THREE.MathUtils.radToDeg(euler.z), yaw: euler.y };
}

// What the robot's gyro would report, from the physics body: yaw (counter-
// clockwise positive, degrees) and yaw rate (deg/s). Feeds heading hold.
export function measuredHeading(state) {
  if (!state?.baseQuaternion || !state?.angularVelocity) return {};
  return {
    measuredYawDeg: THREE.MathUtils.radToDeg(rollPitchDeg(state.baseQuaternion).yaw),
    measuredYawRateDps: THREE.MathUtils.radToDeg(state.angularVelocity[1]),
  };
}

export async function measureGait({ settings = {}, forward = 1, turn = 0, steer = 0, steerBlend = 1, headingHold = false, seconds = 8, warmup = 2 } = {}) {
  const physics = await createDominoPhysics({ assists: false });
  const lab = createGaitLab({ ...settings, enabled: true });
  const stand = standState();
  let state;
  for (let i = 0; i < 240; i += 1) state = physics.update(STEP, stand, legs, standServoReference);

  const gaitSource = { ...stand, mode: "GAIT" };
  const frames = Math.round((warmup + seconds) * 120);
  const warmFrames = Math.round(warmup * 120);
  let startPos = null;
  let startYaw = 0;
  let heading = null;
  let previous = null;
  let plantedTime = 0;
  let slipForward = 0;
  let slipSideways = 0;
  let maxContactsLost = 0;
  let minContacts = 4;
  let fell = false;
  const rolls = [];
  const pitches = [];
  const heights = [];
  let commandedSpeedSum = 0;
  let commandedSamples = 0;
  let commandedRightSum = 0;

  for (let frame = 0; frame < frames; frame += 1) {
    const output = lab.update(STEP, gaitSource, { forward, turn, steer, steerBlend, ...(headingHold ? measuredHeading(state) : {}) });
    state = physics.update(STEP, output, legs, standServoReference);
    if (state.bodyHeight < 0.12 || state.baseTiltDegrees > 45) {
      fell = true;
      break;
    }
    if (frame < warmFrames) continue;
    const attitude = rollPitchDeg(state.baseQuaternion);
    if (!startPos) {
      startPos = [...state.basePosition];
      startYaw = attitude.yaw;
      heading = new THREE.Vector3(1, 0, 0).applyQuaternion(new THREE.Quaternion(...state.baseQuaternion));
      heading.y = 0;
      heading.normalize();
    }
    const side = new THREE.Vector3(-heading.z, 0, heading.x);
    rolls.push(attitude.roll);
    pitches.push(attitude.pitch);
    heights.push(state.bodyHeight);
    minContacts = Math.min(minContacts, state.contactCount);
    const telemetry = lab.getTelemetry();
    if (telemetry.active) {
      commandedSpeedSum += Math.abs(telemetry.bodyVelocity?.forwardMmPerSec || 0);
      commandedRightSum += telemetry.bodyVelocity?.rightMmPerSec || 0;
      commandedSamples += 1;
    }
    if (previous) {
      state.footPositions.forEach((position, index) => {
        if (!state.footContacts[index] || !previous.footContacts[index]) return;
        const delta = new THREE.Vector3(
          position[0] - previous.footPositions[index][0], 0,
          position[2] - previous.footPositions[index][2],
        );
        slipForward += Math.abs(delta.dot(heading));
        slipSideways += Math.abs(delta.dot(side));
        plantedTime += STEP;
      });
    }
    previous = state;
  }

  const spread = (values) => (values.length ? Math.max(...values) - Math.min(...values) : 0);
  const rms = (values) => {
    if (!values.length) return 0;
    const mean = values.reduce((a, b) => a + b, 0) / values.length;
    return Math.sqrt(values.reduce((a, b) => a + (b - mean) ** 2, 0) / values.length);
  };
  const end = state.basePosition;
  const travel = startPos
    ? new THREE.Vector3(end[0] - startPos[0], 0, end[2] - startPos[2])
    : new THREE.Vector3();
  const forwardM = heading ? travel.dot(heading) : 0;
  const sidewaysM = heading ? travel.dot(new THREE.Vector3(-heading.z, 0, heading.x)) : 0;
  const yawChangeDeg = startPos ? THREE.MathUtils.radToDeg(rollPitchDeg(state.baseQuaternion).yaw - startYaw) : 0;
  const plantedFeetSeconds = Math.max(plantedTime, 1e-6);
  return {
    fell,
    speedMmPerSec: (forwardM / seconds) * 1000,
    commandedMmPerSec: commandedSamples ? commandedSpeedSum / commandedSamples : 0,
    commandedRightMmPerSec: commandedSamples ? commandedRightSum / commandedSamples : 0,
    sidewaysMmPerMetre: forwardM > 0.01 ? (Math.abs(sidewaysM) / forwardM) * 1000 : NaN,
    sidewaysSignedMm: sidewaysM * 1000,
    yawSignedDeg: yawChangeDeg,
    headingDegPerMetre: forwardM > 0.01 ? Math.abs(yawChangeDeg) / forwardM : NaN,
    slipForwardMmPerSec: (slipForward / plantedFeetSeconds) * 1000,
    slipSidewaysMmPerSec: (slipSideways / plantedFeetSeconds) * 1000,
    rollSwingDeg: spread(rolls),
    rollRmsDeg: rms(rolls),
    pitchSwingDeg: spread(pitches),
    bobMm: spread(heights) * 1000,
    minContacts,
  };
}

// Turning on the spot (left stick X): how fast the body actually rotates,
// how far its centre wanders off the spot, and how much planted feet slide.
export async function measureSpin({ settings = {}, turn = 1, headingHold = false, seconds = 8, warmup = 2 } = {}) {
  const physics = await createDominoPhysics({ assists: false });
  const lab = createGaitLab({ ...settings, enabled: true });
  const stand = standState();
  let state;
  for (let i = 0; i < 240; i += 1) state = physics.update(STEP, stand, legs, standServoReference);
  const gaitSource = { ...stand, mode: "GAIT" };
  let yawTotal = 0;
  let previousYaw = null;
  let start = null;
  let wander = 0;
  let previous = null;
  let slip = 0;
  let plantedTime = 0;
  let commandedSum = 0;
  let commandedSamples = 0;
  for (let frame = 0; frame < Math.round((warmup + seconds) * 120); frame += 1) {
    const output = lab.update(STEP, gaitSource, { forward: 0, turn, steer: 0, ...(headingHold ? measuredHeading(state) : {}) });
    state = physics.update(STEP, output, legs, standServoReference);
    if (state.bodyHeight < 0.12 || state.baseTiltDegrees > 45) return { fell: true };
    if (frame < warmup * 120) continue;
    const { yaw } = rollPitchDeg(state.baseQuaternion);
    if (previousYaw !== null) {
      let delta = yaw - previousYaw;
      if (delta > Math.PI) delta -= 2 * Math.PI;
      if (delta < -Math.PI) delta += 2 * Math.PI;
      yawTotal += delta;
    }
    previousYaw = yaw;
    if (!start) start = [...state.basePosition];
    wander = Math.max(wander, Math.hypot(state.basePosition[0] - start[0], state.basePosition[2] - start[2]));
    const telemetry = lab.getTelemetry();
    if (telemetry.active) {
      commandedSum += Math.abs(telemetry.bodyVelocity?.yawDegPerSec || 0);
      commandedSamples += 1;
    }
    if (previous) {
      state.footPositions.forEach((position, index) => {
        if (!state.footContacts[index] || !previous.footContacts[index]) return;
        slip += Math.hypot(position[0] - previous.footPositions[index][0], position[2] - previous.footPositions[index][2]);
        plantedTime += STEP;
      });
    }
    previous = state;
  }
  return {
    fell: false,
    yawDegPerSec: Math.abs(THREE.MathUtils.radToDeg(yawTotal)) / seconds,
    commandedYawDegPerSec: commandedSamples ? commandedSum / commandedSamples : 0,
    wanderMm: wander * 1000,
    slipMmPerSec: (slip / Math.max(plantedTime, 1e-6)) * 1000,
  };
}

// Sidestep right then left from a standing start (the case that traced a V).
// Reports how far the body ends behind its start line, the worst point, how
// far it reached sideways, and the largest rotation, along the starting heading.
export async function measureSidestepSequence({ settings = {}, seconds = 12, headingHold = true } = {}) {
  const physics = await createDominoPhysics({ assists: false });
  const lab = createGaitLab({ ...settings, enabled: true });
  const stand = standState();
  let state;
  for (let i = 0; i < 240; i += 1) state = physics.update(STEP, stand, legs, standServoReference);
  const start = [...state.basePosition];
  const startYaw = rollPitchDeg(state.baseQuaternion).yaw;
  const heading = new THREE.Vector3(Math.cos(startYaw), 0, -Math.sin(startYaw));
  const side = new THREE.Vector3(-heading.z, 0, heading.x);
  const gaitSource = { ...stand, mode: "GAIT" };
  const frames = Math.round(seconds * 120);
  let worstBehind = 0;
  let reach = 0;
  let maxYawDeg = 0;
  for (let frame = 0; frame < frames; frame += 1) {
    const steer = frame < frames / 2 ? 1 : -1;
    const output = lab.update(STEP, gaitSource, { forward: 0, turn: 0, steer, ...(headingHold ? measuredHeading(state) : {}) });
    state = physics.update(STEP, output, legs, standServoReference);
    if (state.bodyHeight < 0.12 || state.baseTiltDegrees > 45) return { fell: true };
    const travel = new THREE.Vector3(state.basePosition[0] - start[0], 0, state.basePosition[2] - start[2]);
    worstBehind = Math.min(worstBehind, travel.dot(heading));
    reach = Math.max(reach, Math.abs(travel.dot(side)));
    maxYawDeg = Math.max(maxYawDeg, Math.abs(THREE.MathUtils.radToDeg(rollPitchDeg(state.baseQuaternion).yaw - startYaw)));
  }
  const travel = new THREE.Vector3(state.basePosition[0] - start[0], 0, state.basePosition[2] - start[2]);
  return {
    fell: false,
    endForeAftMm: travel.dot(heading) * 1000,
    worstBehindMm: worstBehind * 1000,
    sidewaysReachMm: reach * 1000,
    maxYawDeg,
  };
}

function format(result) {
  const f = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : "n/a");
  return [
    result.fell ? "FELL" : "ok",
    `speed ${f(result.speedMmPerSec, 0)} of ${f(result.commandedMmPerSec, 0)} mm/s`,
    `sideways ${f(result.sidewaysMmPerMetre, 0)} mm/m`,
    `heading ${f(result.headingDegPerMetre)} deg/m`,
    `slip fwd ${f(result.slipForwardMmPerSec)} side ${f(result.slipSidewaysMmPerSec)} mm/s`,
    `roll swing ${f(result.rollSwingDeg)} deg`,
    `pitch swing ${f(result.pitchSwingDeg)} deg`,
    `bob ${f(result.bobMm)} mm`,
    `min contacts ${result.minContacts}`,
  ].join(" | ");
}

if (process.argv[1]?.endsWith("gait-quality.mjs")) {
  const forward = Number(process.argv[2] ?? 1);
  const seconds = Number(process.argv[3] ?? 8);
  const spec = process.argv[4] ?? "balanced";
  const settings = gaitLabPresets[spec] ? { ...gaitLabPresets[spec] } : JSON.parse(spec);
  const result = await measureGait({ settings, forward, seconds });
  console.log(`${typeof spec === "string" && gaitLabPresets[spec] ? spec : "custom"}: ${format(result)}`);
}
