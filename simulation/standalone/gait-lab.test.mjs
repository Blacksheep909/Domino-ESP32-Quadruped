import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

import { legs, standServoReference } from "./web/src/domino-config.js";
import {
  createGaitLab,
  defaultGaitLabSettings,
  gaitLabControls,
  gaitLabPresets,
} from "./web/src/gait-lab.js";

function state(mode = "GAIT", heightMm = 260) {
  return {
    mode,
    tilt_active: false,
    body_pose_rpy_deg: [0, 0, 0],
    pose_z_mm: heightMm,
    target_z_mm: heightMm,
    ride_height_mm: heightMm,
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, heightMm],
      [-15.75, -38, heightMm],
      [-15.75, 38, heightMm],
      [-15.75, -38, heightMm],
    ],
  };
}

test("every shared gait setting has a Simulation editor control", () => {
  const html = readFileSync(new URL("./web/index.html", import.meta.url), "utf8");
  gaitLabControls.forEach(({ key }) => {
    assert.match(html, new RegExp(`data-gait-setting=["']${key}["']`), `${key} is missing from Simulation`);
  });
});

test("gait lab is simulation-only and does not mutate firmware state", () => {
  const source = state();
  const snapshot = structuredClone(source);
  const lab = createGaitLab(defaultGaitLabSettings);
  const output = lab.update(0.05, source, { forward: 1, turn: 0 });
  assert.deepEqual(source, snapshot);
  assert.notEqual(output, source);
  assert.equal(output.gait_lab.active, true);
});

test("gait lab keeps all twelve driven channels inside the mechanical envelope", () => {
  const lab = createGaitLab({ ...gaitLabPresets.fast, enabled: true });
  let output = state();
  for (let frame = 0; frame < 80; frame += 1) {
    output = lab.update(1 / 120, state(), { forward: 0.85, turn: 0.25 });
  }
  for (const leg of legs) {
    for (const joint of ["shoulder", "upper", "lower"]) {
      const channel = leg.channels[joint];
      assert.ok(
        Math.abs(output.servo_angle_deg[channel] - standServoReference[channel]) <= 45.0001,
        `${leg.label} ${joint} exceeded the 45 degree envelope`,
      );
    }
  }
});

test("IK inspector telemetry reports each foot target, solve and joint delta", () => {
  const lab = createGaitLab(defaultGaitLabSettings);
  for (let frame = 0; frame < 60; frame += 1) {
    lab.update(1 / 120, state(), { forward: 0.75, turn: 0.2 });
  }
  const telemetry = lab.getTelemetry();
  assert.equal(telemetry.legDetails.length, 4);
  assert.deepEqual(telemetry.legDetails.map((detail) => detail.leg).sort(), ["BL", "BR", "FL", "FR"]);
  telemetry.legDetails.forEach((detail) => {
    assert.equal(detail.targetMm.length, 3);
    assert.equal(typeof detail.reachable, "boolean");
    assert.equal(typeof detail.stance, "boolean");
    assert.deepEqual(Object.keys(detail.jointDeltaDeg), ["shoulder", "upper", "lower"]);
    assert.ok(Object.values(detail.jointDeltaDeg).every(Number.isFinite));
  });
});

test("higher cadence advances the gait phase faster", () => {
  const slow = createGaitLab({ ...defaultGaitLabSettings, cadenceHz: 0.5 });
  const fast = createGaitLab({ ...defaultGaitLabSettings, cadenceHz: 2 });
  slow.update(0.05, state(), { forward: 1, turn: 0 });
  fast.update(0.05, state(), { forward: 1, turn: 0 });
  assert.ok(fast.getTelemetry().phase > slow.getTelemetry().phase * 3.5);
});

test("expert gait limits bound command axes and shift nominal foot X", () => {
  const lab = createGaitLab({
    ...defaultGaitLabSettings,
    responseMs: 60,
    touchdownXMm: -30,
    maxForwardScale: 0.35,
    maxTurnScale: 0.25,
  });
  let output;
  for (let frame = 0; frame < 180; frame += 1) {
    output = lab.update(1 / 120, state(), { forward: 1, turn: 1 });
  }
  assert.ok(Math.abs(output.gait_command[0]) <= 0.3501);
  assert.ok(Math.abs(output.gait_command[1]) <= 0.2501);
  const meanTargetX = output.leg_command_xyz_mm.reduce((sum, command) => sum + command[0], 0) / 4;
  assert.ok(meanTargetX < -25);
});

test("careful gait keeps at least three feet in support", () => {
  const lab = createGaitLab({ ...defaultGaitLabSettings, dutyFactor: 0.6 });
  for (let frame = 0; frame < 300; frame += 1) {
    lab.update(1 / 120, state("CAREFUL"), { forward: 0.8, turn: 0 });
    assert.ok(lab.getTelemetry().stanceCount >= 3);
  }
});

test("stand and careful modes blend without a one-frame pose snap", () => {
  const lab = createGaitLab({
    ...defaultGaitLabSettings,
    bodyHeightMm: 220,
    responseMs: 60,
  });
  const stand = state("STAND", 280);
  const careful = state("CAREFUL", 280);
  const firstCarefulFrame = lab.update(1 / 120, careful, { forward: 0.7, turn: 0 });
  assert.ok(firstCarefulFrame.pose_z_mm > 279.5);

  let settledCareful = firstCarefulFrame;
  for (let frame = 0; frame < 80; frame += 1) {
    settledCareful = lab.update(1 / 120, careful, { forward: 0.7, turn: 0 });
  }
  assert.ok(settledCareful.pose_z_mm < 221);

  const firstStandFrame = lab.update(1 / 120, stand, { forward: 0, turn: 0 });
  assert.ok(Math.abs(firstStandFrame.pose_z_mm - settledCareful.pose_z_mm) < 0.5);
  const largestServoStep = Math.max(...firstStandFrame.servo_angle_deg.map(
    (angle, channel) => Math.abs(angle - settledCareful.servo_angle_deg[channel]),
  ));
  assert.ok(largestServoStep < 1);
});

test("careful and trot trajectories crossfade without an actuator snap", () => {
  const lab = createGaitLab(defaultGaitLabSettings);
  let carefulOutput;
  for (let frame = 0; frame < 90; frame += 1) {
    carefulOutput = lab.update(1 / 120, state("CAREFUL"), { forward: 0.8, turn: 0.1 });
  }
  const firstTrotFrame = lab.update(1 / 120, state("GAIT"), { forward: 0.8, turn: 0.1 });
  const largestServoStep = Math.max(...firstTrotFrame.servo_angle_deg.map(
    (angle, channel) => Math.abs(angle - carefulOutput.servo_angle_deg[channel]),
  ));
  assert.ok(largestServoStep < 1);
});

test("careful and trot preserve gait phase while switching", () => {
  const lab = createGaitLab(defaultGaitLabSettings);
  for (let frame = 0; frame < 73; frame += 1) {
    lab.update(1 / 120, state("CAREFUL"), { forward: 0.8, turn: 0.1 });
  }
  const phaseBefore = lab.getTelemetry().phase;
  lab.update(1 / 120, state("GAIT"), { forward: 0.8, turn: 0.1 });
  const phaseAfter = lab.getTelemetry().phase;
  const circularAdvance = (phaseAfter - phaseBefore + 1) % 1;
  assert.ok(
    circularAdvance < 0.05,
    `mode switch reset or jumped phase by ${circularAdvance}`,
  );
});

test("live gait setting edits are smoothed while walking", () => {
  const lab = createGaitLab(defaultGaitLabSettings);
  let beforeEdit;
  for (let frame = 0; frame < 90; frame += 1) {
    beforeEdit = lab.update(1 / 120, state("GAIT"), { forward: 0.7, turn: 0 });
  }
  lab.setSettings({ ...defaultGaitLabSettings, bodyHeightMm: 220, strideMm: 120 });
  const firstEditedFrame = lab.update(1 / 120, state("GAIT"), { forward: 0.7, turn: 0 });
  assert.ok(Math.abs(firstEditedFrame.pose_z_mm - beforeEdit.pose_z_mm) < 4);
  const largestServoStep = Math.max(...firstEditedFrame.servo_angle_deg.map(
    (angle, channel) => Math.abs(angle - beforeEdit.servo_angle_deg[channel]),
  ));
  assert.ok(largestServoStep < 5);
});

test("leaving a gait clears speed and eases into a tilt pose", () => {
  const lab = createGaitLab(defaultGaitLabSettings);
  for (let frame = 0; frame < 90; frame += 1) {
    lab.update(1 / 120, state("GAIT"), { forward: 1, turn: 0 });
  }
  const tilt = state("TILT");
  tilt.tilt_active = true;
  tilt.body_pose_rpy_deg = [20, 0, 0];
  const firstTiltFrame = lab.update(1 / 120, tilt, { forward: 0, turn: 0 });
  assert.equal(lab.getTelemetry().speedMmPerSec, 0);
  assert.ok(Math.abs(firstTiltFrame.body_pose_rpy_deg[0]) < 1);
});
