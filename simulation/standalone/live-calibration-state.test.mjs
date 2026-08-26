import test from "node:test";
import assert from "node:assert/strict";

import {
  calibrationPreviewServoAngles,
  calibrationChannelMap,
  calibrationChannelMapIssues,
  calibrationProfileJson,
  createCalibrationBenchCommand,
  createLiveCalibrationProfile,
  createLiveCalibrationState,
  jogCalibrationJoint,
  LIVE_CALIBRATION_JOG_LIMIT_DEG,
  LIVE_CALIBRATION_JOINTS,
  parseCalibrationProfileJson,
  restoreCalibrationDefaults,
  restoreSelectedCalibrationJoint,
  selectCalibrationJoint,
  selectCalibrationStep,
  trimCalibrationJoint,
  updateCalibrationJoint,
  updateCalibrationChannelMap,
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

test("all four physical hip servos share the same normal direction", () => {
  const hips = LIVE_CALIBRATION_JOINTS.filter((joint) => joint.joint === "shoulder");
  assert.deepEqual(hips.map((joint) => joint.leg).sort(), ["BL", "BR", "FL", "FR"]);
  assert.ok(hips.every((joint) => joint.defaultDirection === 1));

  const profile = createLiveCalibrationProfile();
  const hipChannels = new Set(hips.map((joint) => joint.channel));
  assert.ok(profile.joints
    .filter((joint) => hipChannels.has(joint.logicalChannel))
    .every((joint) => joint.direction === 1
      && joint.minimumDeg === -30
      && joint.maximumDeg === 30));
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
    logicalChannel: state.selectedChannel,
    channel: state.selectedChannel,
    offsetDeg: 30,
    direction: -1,
    minimumDeg: -30,
    maximumDeg: 30,
  });
  assert.equal(state.dirty, true);
});

test("restores one joint without changing its physical servo assignment", () => {
  const state = createLiveCalibrationState();
  const remapped = calibrationChannelMap(state.profile);
  [remapped[0], remapped[11]] = [remapped[11], remapped[0]];
  updateCalibrationChannelMap(state, remapped);
  updateCalibrationJoint(state, { offsetDeg: 12, direction: -1, minimumDeg: -20, maximumDeg: 30 });
  state.jogOffsetDeg = 4;
  assert.equal(restoreSelectedCalibrationJoint(state), true);
  const joint = state.profile.joints.find((candidate) => candidate.logicalChannel === state.selectedChannel);
  const definition = LIVE_CALIBRATION_JOINTS.find((candidate) => candidate.channel === state.selectedChannel);
  assert.equal(joint.channel, remapped[0]);
  assert.equal(joint.offsetDeg, 0);
  assert.equal(joint.direction, definition.defaultDirection);
  assert.equal(joint.minimumDeg, -30);
  assert.equal(joint.maximumDeg, 30);
  assert.equal(state.jogOffsetDeg, 0);
});

test("restores all tuning defaults while resetting wiring only when requested", () => {
  const state = createLiveCalibrationState();
  const remapped = calibrationChannelMap(state.profile);
  [remapped[0], remapped[11]] = [remapped[11], remapped[0]];
  updateCalibrationChannelMap(state, remapped);
  updateCalibrationJoint(state, { offsetDeg: 9, minimumDeg: -12, maximumDeg: 22 });
  assert.equal(restoreCalibrationDefaults(state), true);
  assert.deepEqual(calibrationChannelMap(state.profile), remapped);
  const hipChannels = new Set(
    LIVE_CALIBRATION_JOINTS
      .filter((joint) => joint.joint === "shoulder")
      .map((joint) => joint.channel),
  );
  assert.ok(state.profile.joints.every((joint) => {
    const travel = hipChannels.has(joint.logicalChannel) ? 30 : 45;
    return joint.offsetDeg === 0
      && joint.minimumDeg === -travel
      && joint.maximumDeg === travel;
  }));
  assert.equal(restoreCalibrationDefaults(state, true), true);
  assert.deepEqual(calibrationChannelMap(state.profile), LIVE_CALIBRATION_JOINTS.map((joint) => joint.channel));
});

test("channel mapping is atomic, unique and retained in versioned JSON", () => {
  const state = createLiveCalibrationState();
  const remapped = calibrationChannelMap(state.profile);
  [remapped[0], remapped[11]] = [remapped[11], remapped[0]];
  assert.deepEqual(calibrationChannelMapIssues(remapped), []);
  assert.equal(updateCalibrationChannelMap(state, remapped).accepted, true);
  assert.deepEqual(calibrationChannelMap(state.profile), remapped);
  assert.equal(state.dirty, true);

  const duplicate = [...remapped];
  duplicate[1] = duplicate[0];
  const rejected = updateCalibrationChannelMap(state, duplicate);
  assert.equal(rejected.accepted, false);
  assert.match(rejected.issues[0], /assigned more than once/i);
  assert.deepEqual(calibrationChannelMap(state.profile), remapped);

  const restored = parseCalibrationProfileJson(calibrationProfileJson(state.profile));
  assert.deepEqual(calibrationChannelMap(restored), remapped);
});

test("version 1 calibration imports migrate fixed channels into version 2", () => {
  const legacy = createLiveCalibrationProfile();
  const legacyJson = JSON.stringify({
    ...legacy,
    schemaVersion: 1,
    joints: legacy.joints.map(({ logicalChannel, ...joint }) => joint),
  });
  const migrated = parseCalibrationProfileJson(legacyJson);
  assert.equal(migrated.schemaVersion, 2);
  assert.deepEqual(calibrationChannelMap(migrated), LIVE_CALIBRATION_JOINTS.map((joint) => joint.channel));
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

test("neutral trim works in 0.1 degree steps and becomes the physical neutral target", () => {
  const state = createLiveCalibrationState();
  state.jogOffsetDeg = 3;
  assert.equal(trimCalibrationJoint(state, 0.1), true);
  assert.equal(trimCalibrationJoint(state, 0.1), true);
  const joint = state.profile.joints.find(
    (candidate) => candidate.logicalChannel === state.selectedChannel,
  );
  assert.equal(joint.offsetDeg, 0.2);
  assert.equal(state.jogOffsetDeg, 0);
  assert.equal(state.dirty, true);
  const command = createCalibrationBenchCommand(state, "jog", "trim-1", 1234);
  assert.equal(command.jogOffsetDeg, 0);
  assert.equal(command.targetServoDeg, LIVE_CALIBRATION_JOINTS[0].neutralServoDeg + 0.2);
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
  assert.equal(validCalibrationCommand({ ...command, physicalChannel: command.selectedChannel }), true);
  assert.equal(validCalibrationCommand({ ...command, physicalChannel: 16 }), false);
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

test("robot persistence commands always carry a concrete save timestamp", () => {
  const state = createLiveCalibrationState();
  const command = createCalibrationBenchCommand(state, "save-profile", "save-timestamp", 1234);
  assert.equal(command.profile.savedAt, 1234);
});

test("a complete calibration save envelope fits the firmware USB receive ring", () => {
  const state = createLiveCalibrationState();
  const command = createCalibrationBenchCommand(state, "save-profile", "save-size", 1234);
  const physicalEnvelope = {
    type: "companion-command",
    protocol: "domino-robot-link-v1",
    kind: "calibration",
    action: command.action,
    requestId: command.requestId,
    timestampMs: 1234,
    payload: command,
  };
  const bytes = Buffer.byteLength(JSON.stringify(physicalEnvelope), "utf8") + 1;
  assert.ok(bytes > 1024, "the regression command must exceed the former 1 KB reservation");
  assert.ok(bytes <= 4096, "profile writes must remain inside the firmware's 4 KB RX ring");
});
