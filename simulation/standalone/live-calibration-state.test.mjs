import test from "node:test";
import assert from "node:assert/strict";

import {
  calibrationPreviewServoAngles,
  calibrationProfileJson,
  createCalibrationBenchCommand,
  createLiveCalibrationProfile,
  createLiveCalibrationState,
  jogCalibrationJoint,
  LIVE_CALIBRATION_JOG_LIMIT_DEG,
  LIVE_CALIBRATION_JOINTS,
  parseCalibrationProfileJson,
  selectCalibrationJoint,
  selectCalibrationStep,
  updateCalibrationJoint,
} from "./web/src/live-calibration-state.js";
import {
  validCalibrationAcknowledgement,
  validCalibrationCommand,
} from "./web/src/live-calibration-protocol.js";

test("creates one bounded calibration record for every driven Domino joint", () => {
  const profile = createLiveCalibrationProfile();
  assert.equal(profile.joints.length, 12);
  assert.deepEqual(
    profile.joints.map((joint) => joint.channel),
    LIVE_CALIBRATION_JOINTS.map((joint) => joint.channel),
  );
});

test("selects wizard steps and joints without accepting unknown values", () => {
  const state = createLiveCalibrationState();
  assert.equal(selectCalibrationStep(state, "neutral"), true);
  assert.equal(state.step, 2);
  assert.equal(selectCalibrationStep(state, "wireless-magic"), false);
  assert.equal(selectCalibrationJoint(state, 15), true);
  assert.equal(selectCalibrationJoint(state, 6), false);
});

test("calibration edits are clamped to conservative configuration bounds", () => {
  const state = createLiveCalibrationState();
  updateCalibrationJoint(state, {
    offsetDeg: 200,
    direction: -1,
    minimumDeg: -500,
    maximumDeg: 500,
  });
  const joint = state.profile.joints.find((candidate) => candidate.channel === state.selectedChannel);
  assert.deepEqual(joint, {
    channel: state.selectedChannel,
    offsetDeg: 30,
    direction: -1,
    minimumDeg: -90,
    maximumDeg: 90,
  });
  assert.equal(state.dirty, true);
});

test("preview jogging is limited and only changes the selected servo", () => {
  const state = createLiveCalibrationState();
  const baseline = calibrationPreviewServoAngles(state);
  jogCalibrationJoint(state, 100);
  const preview = calibrationPreviewServoAngles(state);
  assert.equal(state.jogOffsetDeg, LIVE_CALIBRATION_JOG_LIMIT_DEG);
  assert.equal(preview[state.selectedChannel] - baseline[state.selectedChannel], LIVE_CALIBRATION_JOG_LIMIT_DEG);
  assert.equal(preview.filter((angle, channel) => angle !== baseline[channel]).length, 1);
});

test("calibration JSON round-trips and rejects other schemas or robots", () => {
  const profile = createLiveCalibrationProfile({
    joints: [{ channel: 0, offsetDeg: 2.5, direction: -1, minimumDeg: -40, maximumDeg: 42 }],
  });
  const restored = parseCalibrationProfileJson(calibrationProfileJson(profile));
  assert.deepEqual(restored, profile);
  assert.throws(() => parseCalibrationProfileJson('{"schemaVersion":2,"robot":"domino-esp32-quadruped"}'));
  assert.throws(() => parseCalibrationProfileJson('{"schemaVersion":1,"robot":"other"}'));
});

test("bench commands carry explicit robot-side safety limits", () => {
  const state = createLiveCalibrationState();
  jogCalibrationJoint(state, 1);
  const command = createCalibrationBenchCommand(state, "jog", "request-1", 1234);
  assert.equal(command.type, "live-calibration-command");
  assert.equal(command.safety.benchModeRequired, true);
  assert.equal(command.safety.maxSpeedDegPerSec, 5);
  assert.equal(command.timestampMs, 1234);
  assert.equal(validCalibrationCommand(command), true);
  assert.equal(validCalibrationCommand({ ...command, safety: { ...command.safety, maxSpeedDegPerSec: 6 } }), false);
  assert.equal(validCalibrationAcknowledgement({
    type: "live-calibration-ack",
    action: "enter",
    requestId: "request-1",
    accepted: true,
  }), true);
  assert.equal(validCalibrationAcknowledgement({
    type: "live-calibration-ack",
    action: "sweep",
    requestId: "request-1",
    accepted: true,
  }), false);
  assert.equal(createCalibrationBenchCommand(state, "unsafe-sweep", "request-2"), null);
});
