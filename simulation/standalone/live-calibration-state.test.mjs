import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import vm from "node:vm";

import {
  DOMINO_BASELINE_CALIBRATION,
  acceptRobotCalibrationProfile,
  acceptCalibrationSaveReadback,
  beginRobotCalibrationSave,
  acknowledgeRobotCalibrationSave,
  calibrationControlBlockReason,
  calibrationInspectionModelAngles,
  calibrationOutputForModelChannel,
  calibrationPreviewModelAngles,
  calibrationPreviewServoAngles,
  calibrationChannelMap,
  calibrationChannelMapIssues,
  calibrationProfileJson,
  createCalibrationBenchCommand,
  createLiveCalibrationProfile,
  createLiveCalibrationState,
  forgetRobotCalibrationProfile,
  jogCalibrationJoint,
  LIVE_CALIBRATION_JOG_LIMIT_DEG,
  LIVE_CALIBRATION_JOINTS,
  LIVE_CALIBRATION_OUTPUTS,
  parseCalibrationProfileJson,
  robotCalibrationReady,
  resetLiveCalibrationConnection,
  saveLocalCalibrationProfile,
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
import { legs, standServoReference } from "./web/src/domino-config.js";

test("rear-right calibration identifies the existing outputs that move each physical joint", () => {
  const rearRight = LIVE_CALIBRATION_OUTPUTS.filter((output) => output.leg === "BR");
  assert.deepEqual(rearRight.map(({ label, channel }) => [label, channel]), [
    ["BR Hip", 9], ["BR Lower", 10], ["BR Upper", 11],
  ]);
  assert.equal(calibrationOutputForModelChannel(10).channel, 11);
  assert.equal(calibrationOutputForModelChannel(11).channel, 10);
  assert.equal(calibrationOutputForModelChannel(9).channel, 9);
  assert.equal(calibrationOutputForModelChannel(6), undefined);
});

test("a calibration preview follows the firmware cross-reversed physical response on every leg", () => {
  const state = createLiveCalibrationState();
  for (const leg of legs) {
    for (const [output, physicalJoint, sign] of [
      ["shoulder", "shoulder", 1], ["upper", "lower", -1], ["lower", "upper", -1],
    ]) {
      selectCalibrationJoint(state, leg.channels[output]);
      const angles = calibrationInspectionModelAngles(state, 6);
      const physicalChannel = leg.channels[physicalJoint];
      assert.equal((angles[physicalChannel] - standServoReference[physicalChannel]) / leg.directions[physicalJoint], sign * 6);
      assert.equal(angles.filter((angle, channel) => angle !== standServoReference[channel]).length, 1);
    }
  }
});

test("model inspection preserves read-back calibration, electrical neutral targets and saved routes", () => {
  const active = createLiveCalibrationProfile({ savedAt: 1770000000000 });
  active.joints.forEach((joint, index) => {
    joint.offsetDeg = index / 10;
    joint.direction = index % 2 ? -1 : 1;
    joint.minimumDeg = -17;
    joint.maximumDeg = 23;
  });
  [active.joints[0].channel, active.joints[11].channel] = [active.joints[11].channel, active.joints[0].channel];
  const state = createLiveCalibrationState();
  assert.equal(acceptRobotCalibrationProfile(state, active, "existing-robot").accepted, true);
  const before = calibrationProfileJson(state.profile);
  const neutralOutputs = calibrationPreviewServoAngles(state);
  for (const output of LIVE_CALIBRATION_OUTPUTS) {
    selectCalibrationJoint(state, output.channel);
    calibrationInspectionModelAngles(state, -12.3);
    calibrationInspectionModelAngles(state, 14.7);
    assert.deepEqual(calibrationPreviewServoAngles(state), neutralOutputs);
    assert.equal(calibrationProfileJson(state.profile), before);
  }
  assert.equal(state.dirty, false);
});

test("creates one bounded calibration record for every driven Domino joint", () => {
  const profile = createLiveCalibrationProfile();
  assert.equal(profile.joints.length, 12);
  assert.deepEqual(
    profile.joints.map((joint) => joint.channel),
    LIVE_CALIBRATION_JOINTS.map((joint) => joint.channel),
  );
});

test("the CAD basis has one hip direction, but the installed baseline inverts the left hips", () => {
  const hips = LIVE_CALIBRATION_JOINTS.filter((joint) => joint.joint === "shoulder");
  assert.deepEqual(hips.map((joint) => joint.leg).sort(), ["BL", "BR", "FL", "FR"]);
  assert.ok(hips.every((joint) => joint.defaultDirection === 1), "CAD basis is canonical");

  // The robot's left hip shafts (CH0 front-left, CH14 back-left) are mounted
  // inverted. Defaulting to the CAD basis lost this and broke tilt.
  const profile = createLiveCalibrationProfile();
  const direction = (channel) => profile.joints.find((joint) => joint.logicalChannel === channel).direction;
  assert.equal(direction(0), -1);
  assert.equal(direction(14), -1);
  assert.equal(direction(3), 1);
  assert.equal(direction(9), 1);
  const hipChannels = new Set(hips.map((joint) => joint.channel));
  assert.ok(profile.joints
    .filter((joint) => hipChannels.has(joint.logicalChannel))
    .every((joint) => joint.minimumDeg === -30 && joint.maximumDeg === 30));
});

test("the Studio baseline is exactly the verified profile and matches the firmware table", () => {
  const good = parseCalibrationProfileJson(JSON.stringify({
    schemaVersion: 2, robot: "domino-esp32-quadruped", savedAt: 1791620392254,
    joints: [[0, 0, -1], [1, 4, -1], [2, 0, -1], [3, 12, 1], [4, 0, 1], [15, 0, 1],
      [14, 0, -1], [7, -6, -1], [8, 6, -1], [9, 12, 1], [10, -3, 1], [11, 12, 1]]
      .map(([channel, offsetDeg, direction]) => ({
        logicalChannel: channel, channel, offsetDeg, direction,
        minimumDeg: LIVE_CALIBRATION_JOINTS.find((j) => j.channel === channel).joint === "shoulder" ? -30 : -45,
        maximumDeg: LIVE_CALIBRATION_JOINTS.find((j) => j.channel === channel).joint === "shoulder" ? 30 : 45,
      })),
  }));
  const defaults = createLiveCalibrationProfile();
  for (const joint of good.joints) {
    const actual = defaults.joints.find((candidate) => candidate.logicalChannel === joint.logicalChannel);
    assert.equal(actual.offsetDeg, joint.offsetDeg, `offset ch${joint.logicalChannel}`);
    assert.equal(actual.direction, joint.direction, `direction ch${joint.logicalChannel}`);
    assert.equal(actual.channel, joint.channel, `channel ch${joint.logicalChannel}`);
  }

  // The same numbers must be compiled into the firmware default profile.
  const source = readFileSync(new URL("../../src/servo_calibration.cpp", import.meta.url), "utf8");
  const table = (name) => source
    .match(new RegExp(`${name}\\[[^\\]]*\\]\\s*=\\s*\\{([^}]*)\\}`))[1]
    .split(",").map((entry) => entry.trim().replace(/f$/, "")).filter(Boolean).map(Number);
  const offsets = table("kBaselineOffsetDeg");
  const directions = table("kBaselineDirection");
  assert.equal(offsets.length, 16);
  assert.equal(directions.length, 16);
  for (const joint of defaults.joints) {
    assert.equal(offsets[joint.logicalChannel], joint.offsetDeg, `firmware offset ch${joint.logicalChannel}`);
    assert.equal(directions[joint.logicalChannel], joint.direction, `firmware direction ch${joint.logicalChannel}`);
  }
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
  const baseline = DOMINO_BASELINE_CALIBRATION[state.selectedChannel];
  assert.equal(joint.channel, remapped[0]);
  assert.equal(joint.offsetDeg, baseline.offsetDeg);
  assert.equal(joint.direction, baseline.direction);
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
    const baseline = DOMINO_BASELINE_CALIBRATION[joint.logicalChannel];
    return joint.offsetDeg === baseline.offsetDeg
      && joint.direction === baseline.direction
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
  // The servo moves by the joint's direction times the jog (inverted hips go the other way).
  const direction = state.profile.joints.find((joint) => joint.logicalChannel === state.selectedChannel).direction;
  assert.equal(preview[state.selectedChannel] - baseline[state.selectedChannel],
    direction * LIVE_CALIBRATION_JOG_LIMIT_DEG);
  assert.equal(preview.filter((angle, channel) => angle !== baseline[channel]).length, 1);
});

test("saved physical trim defines the new CAD center without moving the model", () => {
  const state = createLiveCalibrationState();
  const baselineModel = calibrationPreviewModelAngles(state);
  const baselinePhysical = calibrationPreviewServoAngles(state);
  updateCalibrationJoint(state, { offsetDeg: 8.5, direction: -1 });
  assert.deepEqual(calibrationPreviewModelAngles(state), baselineModel);
  assert.equal(
    calibrationPreviewServoAngles(state)[state.selectedChannel],
    baselinePhysical[state.selectedChannel] + 8.5,
  );

  jogCalibrationJoint(state, 2);
  const definition = LIVE_CALIBRATION_JOINTS.find((joint) => joint.channel === state.selectedChannel);
  assert.equal(
    calibrationPreviewModelAngles(state)[state.selectedChannel],
    baselineModel[state.selectedChannel] + definition.defaultDirection * 2,
  );
  assert.equal(
    calibrationPreviewServoAngles(state)[state.selectedChannel],
    baselinePhysical[state.selectedChannel] + 8.5 - 2,
  );
  assert.equal(
    calibrationPreviewModelAngles(state).filter((angle, channel) => angle !== baselineModel[channel]).length,
    1,
  );
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

test("active calibration readback is a valid command without bench mode", () => {
  const command = createCalibrationBenchCommand(
    createLiveCalibrationState(), "request-profile", "read-active", 1234,
  );
  assert.equal(command.safety.benchModeRequired, false);
  assert.equal(validCalibrationCommand(command), true);
  assert.equal(validCalibrationCommand({
    ...command, safety: { ...command.safety, benchModeRequired: true },
  }), false);
});

test("robot persistence commands always carry a concrete save timestamp", () => {
  const state = createLiveCalibrationState();
  assert.equal(createCalibrationBenchCommand(state, "save-profile", "unsynced", 1234), null);
  const active = createLiveCalibrationProfile({ savedAt: 1200 });
  active.joints.find((joint) => joint.logicalChannel === 0).direction = -1;
  active.joints.find((joint) => joint.logicalChannel === 14).direction = -1;
  const accepted = acceptRobotCalibrationProfile(state, active, "session-1");
  assert.deepEqual({ accepted: accepted.accepted, adopted: accepted.adopted },
    { accepted: true, adopted: true });
  assert.equal(state.profile.joints.find((joint) => joint.logicalChannel === 0).direction, -1);
  assert.equal(robotCalibrationReady(state, "session-1"), true);
  const command = createCalibrationBenchCommand(state, "save-profile", "save-timestamp", 1234);
  assert.equal(command.profile.savedAt, 1234);
  assert.equal(command.baseSavedAt, 1200);
  assert.equal(validCalibrationCommand(command), true);
  assert.equal(validCalibrationCommand({ ...command, baseSavedAt: undefined }), false);
  forgetRobotCalibrationProfile(state);
  assert.equal(robotCalibrationReady(state, "session-1"), false);
  assert.equal(createCalibrationBenchCommand(state, "save-profile", "stale", 1235), null);
});

test("a robot readback preserves unsaved calibration edits", () => {
  const state = createLiveCalibrationState();
  updateCalibrationJoint(state, { offsetDeg: 4 });
  const active = createLiveCalibrationProfile({ savedAt: 1000 });
  // The robot reports the opposite of the baseline hip direction (-1); the draft must win.
  active.joints.find((joint) => joint.logicalChannel === 0).direction = 1;
  const readback = acceptRobotCalibrationProfile(state, active, "session-2");
  assert.deepEqual({ accepted: readback.accepted, adopted: readback.adopted },
    { accepted: true, adopted: false });
  assert.equal(state.profile.joints[0].offsetDeg, 4);
  assert.equal(state.profile.joints[0].direction, -1);
  assert.equal(createCalibrationBenchCommand(state, "save-profile", "edit", 1001).baseSavedAt, 1000);
});

test("disconnect resets bench permission and jog while retaining calibration work", () => {
  const state = createLiveCalibrationState();
  acceptRobotCalibrationProfile(state, createLiveCalibrationProfile({ savedAt: 1000 }), "before");
  selectCalibrationStep(state, "limits");
  selectCalibrationJoint(state, 15);
  const map = calibrationChannelMap(state.profile);
  [map[0], map[11]] = [map[11], map[0]];
  updateCalibrationChannelMap(state, map);
  updateCalibrationJoint(state, { offsetDeg: 4.5, direction: -1, minimumDeg: -20, maximumDeg: 25 });
  state.benchModeAcknowledged = true;
  state.safetyConfirmed = true;
  state.jogOffsetDeg = 7;
  const before = calibrationProfileJson(state.profile);
  resetLiveCalibrationConnection(state);
  assert.equal(state.benchModeAcknowledged, false);
  assert.equal(state.jogOffsetDeg, 0);
  assert.equal(state.step, 3);
  assert.equal(state.selectedChannel, 15);
  assert.equal(state.dirty, true);
  assert.equal(calibrationProfileJson(state.profile), before);
  assert.equal(robotCalibrationReady(state, "before"), false);
  assert.equal(createCalibrationBenchCommand(state, "save-profile", "stale-save"), null);
  const afterReconnect = acceptRobotCalibrationProfile(state, createLiveCalibrationProfile({ savedAt: 2000 }), "after");
  assert.deepEqual({ accepted: afterReconnect.accepted, adopted: afterReconnect.adopted },
    { accepted: true, adopted: false });
  assert.equal(calibrationProfileJson(state.profile), before);
  assert.equal(robotCalibrationReady(state, "after"), true);
});

test("saving a local backup does not allow reconnect to overwrite unsent edits", () => {
  const state = createLiveCalibrationState();
  acceptRobotCalibrationProfile(state, createLiveCalibrationProfile({ savedAt: 1000 }), "before");
  trimCalibrationJoint(state, 2.5);
  const backup = saveLocalCalibrationProfile(state, 1500);
  assert.equal(state.dirty, true);
  assert.equal(parseCalibrationProfileJson(backup).joints[0].offsetDeg, 2.5);
  resetLiveCalibrationConnection(state);
  acceptRobotCalibrationProfile(state, createLiveCalibrationProfile({ savedAt: 1000 }), "after");
  assert.equal(calibrationProfileJson(state.profile), backup);
});

// Exercise the actual browser lifecycle functions with a minimal link surface.
// This checks cancellation/late ACK behavior beyond the pure profile reducer.
function browserCalibrationHarness() {
  const source = readFileSync(new URL("./web/src/main.js", import.meta.url), "utf8");
  const functions = [
    ["resetCalibrationLink", "resetLiveCommandPermissions"],
    ["serviceCalibrationProfileSync", "serviceCalibrationBenchKeepalive"],
    ["serviceCalibrationBenchKeepalive", "acceptCalibrationAcknowledgement"],
    ["acceptCalibrationAcknowledgement", "formatDiagnosticUptime"],
  ].map(([name, next]) => {
    const start = source.indexOf(`function ${name}(`);
    const end = source.indexOf(`function ${next}(`, start);
    assert.ok(start >= 0 && end > start);
    return source.slice(start, end);
  }).join("\n");
  const connection = { adapterId: "adapter", sessionId: "session" };
  const harness = {
    liveCalibrationState: createLiveCalibrationState(), resetLiveCalibrationConnection,
    calibrationPendingRequestId: "enter-before-disconnect", calibrationPendingAction: "enter",
    calibrationPendingPurpose: "manual", calibrationPendingProfile: null,
    calibrationSyncRetryAt: 0,
    calibrationPendingSmoothing: null, motionSmoothingRobot: null,
    motionSmoothingRobotSessionId: "", motionSmoothingVerify: null, motionSmoothingStatus: "",
    motionSmoothingSyncRetryAt: 0, robotCalibrationReady, forgetRobotCalibrationProfile,
    calibrationObservedSessionId: "session", calibrationLastKeepaliveAt: 0,
    calibrationOpenWiringAfterExit: false, calibrationRequestTimeout: 123,
    calibrationBenchFeedback: "", liveConnectionState: {},
    applicationState: { workspace: "real-robot" }, WORKSPACE_REAL_ROBOT: "real-robot",
    liveViewState: { selected: "calibration" }, LIVE_VIEW_CALIBRATION: "calibration",
    document: { hidden: false }, socket: null, WebSocket: { OPEN: 1 },
    crypto: { randomUUID: () => "exit-request" }, createCalibrationBenchCommand,
    calibrationSaveAdapterId: "adapter",
    liveConnectionEnvelope: () => connection, liveConnectionIsReady: () => true,
    liveTelemetryState: {}, liveComparisonSnapshot: () => ({ lastRobotPacketAgeMs: 1500 }),
    updateLiveComparisonUi: () => {}, clearTimeout: () => {},
  };
  vm.createContext(harness);
  // main.js is an ES module: preserve its strict semantics so an obsolete
  // variable cannot silently become a global in this browser harness.
  vm.runInContext(`"use strict";\n${functions}`, harness);
  return harness;
}

test("leaving calibration cancels an in-flight enter and sends exit before ignoring late ACK", () => {
  const browser = browserCalibrationHarness();
  const sent = [];
  browser.socket = { readyState: 1, send: payload => sent.push(JSON.parse(payload)) };
  updateCalibrationJoint(browser.liveCalibrationState, { offsetDeg: 12 });
  browser.releaseCalibrationBench("Page closed");
  assert.equal(sent.length, 1);
  assert.equal(sent[0].action, "exit");
  assert.equal(sent[0].sessionId, "session");
  assert.equal(browser.calibrationPendingRequestId, "");
  assert.equal(browser.acceptCalibrationAcknowledgement({
    type: "live-calibration-ack", adapterId: "adapter", sessionId: "session",
    requestId: "enter-before-disconnect", action: "enter", accepted: true,
    benchMode: true, supportsSafeJog: true, maxSpeedDegPerSec: 5,
  }), false);
  assert.equal(browser.liveCalibrationState.profile.joints[0].offsetDeg, 12);
});

test("hidden, simulation, and unconfirmed support each release bench without losing the draft", () => {
  for (const change of [
    browser => { browser.document.hidden = true; },
    browser => { browser.applicationState.workspace = "simulation"; },
    browser => { browser.liveCalibrationState.safetyConfirmed = false; },
  ]) {
    const browser = browserCalibrationHarness();
    browser.liveCalibrationState.benchModeAcknowledged = true;
    browser.liveCalibrationState.safetyConfirmed = true;
    updateCalibrationJoint(browser.liveCalibrationState, { offsetDeg: 12 });
    const sent = [];
    browser.socket = { readyState: 1, send: payload => sent.push(JSON.parse(payload)) };
    change(browser);
    assert.equal(browser.serviceCalibrationBenchKeepalive(), false);
    assert.equal(sent[0].action, "exit");
    assert.equal(browser.liveCalibrationState.benchModeAcknowledged, false);
    assert.equal(browser.liveCalibrationState.profile.joints[0].offsetDeg, 12);
  }
});

test("reboot during save preserves imported hips and verifies the write on reconnect", () => {
  const state = createLiveCalibrationState();
  acceptRobotCalibrationProfile(state, createLiveCalibrationProfile({ savedAt: 1000 }), "before");
  updateCalibrationJoint(state, { offsetDeg: 12 });
  const command = createCalibrationBenchCommand(state, "save-profile", "save", 2000);
  beginRobotCalibrationSave(state, command.profile);
  acknowledgeRobotCalibrationSave(state, true, true);
  resetLiveCalibrationConnection(state);
  assert.equal(state.savePhase, "unconfirmed");
  acceptRobotCalibrationProfile(state, createLiveCalibrationProfile({ savedAt: 1000 }), "after");
  assert.equal(state.profile.joints[0].offsetDeg, 12);
  assert.equal(state.dirty, true);
  assert.deepEqual(acceptCalibrationSaveReadback(state, command.profile, "after"),
    { accepted: true, matched: true, adopted: true });
  assert.equal(state.profile.joints[0].offsetDeg, 12);
  assert.equal(state.dirty, false);
  assert.equal(state.savePhase, "verified");
});

test("a failed save readback keeps imported hips and never labels the old robot values saved", () => {
  const state = createLiveCalibrationState();
  const old = createLiveCalibrationProfile({ savedAt: 1000 });
  acceptRobotCalibrationProfile(state, old, "session");
  updateCalibrationJoint(state, { offsetDeg: 12 });
  const command = createCalibrationBenchCommand(state, "save-profile", "save", 2000);
  beginRobotCalibrationSave(state, command.profile);
  acknowledgeRobotCalibrationSave(state, true, true);
  assert.deepEqual(acceptCalibrationSaveReadback(state, old, "session"),
    { accepted: true, matched: false, adopted: false });
  assert.equal(state.profile.joints[0].offsetDeg, 12);
  assert.equal(state.dirty, true);
  assert.equal(state.savePhase, "mismatch");
  assert.equal(robotCalibrationReady(state, "session"), false);
});

test("import rejects invalid numeric fields and preserves valid firmware hip limits", () => {
  const valid = createLiveCalibrationProfile({ savedAt: 1000 });
  valid.joints[0].minimumDeg = -60;
  valid.joints[0].maximumDeg = 60;
  assert.deepEqual(parseCalibrationProfileJson(calibrationProfileJson(valid)), valid);
  for (const [key, value] of [["offsetDeg", 31], ["offsetDeg", "12"],
    ["direction", 0], ["minimumDeg", -91], ["maximumDeg", 91]]) {
    const invalid = structuredClone(valid);
    invalid.joints[0][key] = value;
    assert.throws(() => parseCalibrationProfileJson(JSON.stringify(invalid)));
  }
});

test("physical calibration blockers identify armed, stale, missing profile and unsupported states", () => {
  const state = createLiveCalibrationState();
  acceptRobotCalibrationProfile(state, createLiveCalibrationProfile({ savedAt: 1000 }), "session");
  state.safetyConfirmed = true;
  const context = { connectionReady: true, telemetryAgeMs: 0, robotState: "disarmed",
    calibrationSupported: true, sessionId: "session", workspaceActive: true, batteryLocked: false, pending: false };
  assert.equal(calibrationControlBlockReason(state, context), "");
  assert.match(calibrationControlBlockReason(state, { ...context, robotState: "armed" }), /Disarm/);
  assert.match(calibrationControlBlockReason(state, { ...context, telemetryAgeMs: 1001 }), /telemetry/);
  assert.match(calibrationControlBlockReason(state, { ...context, calibrationSupported: false }), /support/);
  assert.match(calibrationControlBlockReason(state, { ...context, sessionId: "other" }), /profile/);
  assert.match(calibrationControlBlockReason(state, { ...context, batteryLocked: true }), /battery/);
  assert.match(calibrationControlBlockReason(state, context, "jog"), /bench/);
});

test("browser disconnect cancels a pending bench request and ignores its late ACK", () => {
  const browser = browserCalibrationHarness();
  updateCalibrationJoint(browser.liveCalibrationState, { offsetDeg: 6 });
  browser.liveCalibrationState.benchModeAcknowledged = true;
  const draft = calibrationProfileJson(browser.liveCalibrationState.profile);
  browser.resetCalibrationLink("Disconnected");
  assert.equal(browser.calibrationPendingRequestId, "");
  assert.equal(browser.calibrationPendingAction, "");
  assert.equal(browser.calibrationRequestTimeout, null);
  assert.equal(browser.acceptCalibrationAcknowledgement({
    type: "live-calibration-ack", adapterId: "adapter", sessionId: "session",
    requestId: "enter-before-disconnect", action: "enter", accepted: true,
    benchMode: true, supportsSafeJog: true, maxSpeedDegPerSec: 5,
  }), false);
  assert.equal(browser.liveCalibrationState.benchModeAcknowledged, false);
  assert.equal(browser.serviceCalibrationBenchKeepalive(), false);
  assert.equal(calibrationProfileJson(browser.liveCalibrationState.profile), draft);
});

test("browser stops renewing bench permission when telemetry goes stale", () => {
  const browser = browserCalibrationHarness();
  updateCalibrationJoint(browser.liveCalibrationState, { offsetDeg: 3 });
  browser.liveCalibrationState.benchModeAcknowledged = true;
  browser.liveCalibrationState.safetyConfirmed = true;
  browser.liveCalibrationState.jogOffsetDeg = 5;
  assert.equal(browser.serviceCalibrationBenchKeepalive(), false);
  assert.equal(browser.liveCalibrationState.benchModeAcknowledged, false);
  assert.equal(browser.liveCalibrationState.jogOffsetDeg, 0);
  assert.equal(browser.calibrationPendingRequestId, "");
  assert.equal(browser.liveCalibrationState.profile.joints[0].offsetDeg, 3);
  assert.equal(browser.liveCalibrationState.dirty, true);
});

test("a newly paired session resets link state and requests its robot profile", () => {
  const browser = browserCalibrationHarness();
  browser.calibrationObservedSessionId = "previous-session";
  browser.liveConnectionState = {
    selectedAdapterId: "adapter", robotState: "disarmed",
    adapters: { adapter: { capabilities: { calibration: true } } },
  };
  const actions = [];
  browser.sendCalibrationCommand = (action, purpose) => {
    actions.push({ action, purpose });
    return true;
  };
  updateCalibrationJoint(browser.liveCalibrationState, { offsetDeg: 6 });
  const draft = calibrationProfileJson(browser.liveCalibrationState.profile);
  browser.serviceCalibrationProfileSync();
  assert.equal(browser.calibrationObservedSessionId, "session");
  assert.deepEqual(actions, [{ action: "request-profile", purpose: "sync" }]);
  assert.equal(calibrationProfileJson(browser.liveCalibrationState.profile), draft);
  assert.equal(browser.liveCalibrationState.dirty, true);
});

test("a complete calibration save envelope fits the firmware USB receive ring", () => {
  const state = createLiveCalibrationState();
  acceptRobotCalibrationProfile(state, createLiveCalibrationProfile({ savedAt: 1000 }), "session-1");
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

// --- Calibration origin: a robot on compiled defaults must never be mistaken for a saved profile ---
import * as originState from "./web/src/live-calibration-state.js";
import assertOrigin from "node:assert/strict";

function originOwnerProfile(savedAt) {
  const profile = originState.createLiveCalibrationProfile({ savedAt });
  profile.joints.find((joint) => joint.logicalChannel === 0).direction = -1;
  profile.joints.find((joint) => joint.logicalChannel === 14).direction = -1;
  profile.joints.find((joint) => joint.logicalChannel === 3).offsetDeg = 7;
  return profile;
}

test("a robot that lost its profile never overwrites the saved browser copy with defaults", () => {
  const state = originState.createLiveCalibrationState(originOwnerProfile(5000));
  const result = originState.acceptRobotCalibrationProfile(
    state, originState.createLiveCalibrationProfile(), "session-lost",
    { source: "default-fallback", detail: "active2:bad-checksum" },
  );
  assertOrigin.equal(result.robotUsingDefaults, true);
  assertOrigin.equal(result.restored, true);
  assertOrigin.equal(result.adopted, false);
  assertOrigin.equal(state.dirty, true, "restored draft must require review and an explicit save");
  assertOrigin.equal(state.profile.joints.find((joint) => joint.logicalChannel === 0).direction, -1);
  assertOrigin.equal(state.profile.joints.find((joint) => joint.logicalChannel === 3).offsetDeg, 7);
  const warning = originState.calibrationOriginWarning(state);
  assertOrigin.equal(warning.level, "warning", "reporting firmware falls back to the verified baseline");
  assertOrigin.match(warning.detail, /active2:bad-checksum/);
  assertOrigin.match(warning.detail, /review it, then save/);
});

test("a default robot restores from the last robot-read profile when the browser copy is empty", () => {
  const state = originState.createLiveCalibrationState();
  const result = originState.acceptRobotCalibrationProfile(
    state, originState.createLiveCalibrationProfile(), "session-empty-browser",
    { source: "default-empty" }, originOwnerProfile(4000),
  );
  assertOrigin.equal(result.restored, true);
  assertOrigin.equal(state.profile.joints.find((joint) => joint.logicalChannel === 0).direction, -1);
  assertOrigin.equal(state.restorableProfile, true);
});

test("a default robot with no earlier saved profile adopts the defaults and still warns", () => {
  const state = originState.createLiveCalibrationState();
  const result = originState.acceptRobotCalibrationProfile(
    state, originState.createLiveCalibrationProfile(), "session-fresh", { source: "default-empty" },
  );
  assertOrigin.equal(result.adopted, true);
  assertOrigin.equal(result.restored, false);
  assertOrigin.equal(originState.calibrationOriginWarning(state).level, "warning");
  assertOrigin.match(originState.calibrationOriginWarning(state).detail, /built-in baseline/);

  // Firmware that cannot report its origin (0.8.18 and earlier) falls back to
  // plain CAD defaults, which do not invert the left hips: that stays a danger.
  const legacy = originState.createLiveCalibrationState();
  originState.acceptRobotCalibrationProfile(legacy, originState.createLiveCalibrationProfile(), "old-firmware");
  assertOrigin.equal(originState.calibrationOriginWarning(legacy).level, "danger");
  assertOrigin.match(originState.calibrationOriginWarning(legacy).detail, /compiled defaults/);
});

test("a saved robot profile replaces the browser copy and shows no warning", () => {
  const state = originState.createLiveCalibrationState(originOwnerProfile(1000));
  const result = originState.acceptRobotCalibrationProfile(
    state, originOwnerProfile(9000), "session-ok", { source: "nvs", savedAt: 9000 },
  );
  assertOrigin.equal(result.adopted, true);
  assertOrigin.equal(result.robotUsingDefaults, false);
  assertOrigin.equal(state.profile.savedAt, 9000);
  assertOrigin.equal(originState.calibrationOriginWarning(state), null);
});

test("backup, legacy and unreported origins are classified without false default alarms", () => {
  const backup = originState.createLiveCalibrationState();
  originState.acceptRobotCalibrationProfile(backup, originOwnerProfile(2000), "s", { source: "nvs-backup", detail: "active2:bad-magic" });
  assertOrigin.equal(originState.calibrationOriginWarning(backup).level, "warning");
  const legacy = originState.createLiveCalibrationState();
  originState.acceptRobotCalibrationProfile(legacy, originOwnerProfile(2000), "s", { source: "nvs-legacy" });
  assertOrigin.equal(originState.calibrationOriginWarning(legacy).level, "notice");
  assertOrigin.equal(originState.normalizeCalibrationOrigin(undefined, originOwnerProfile(3000)).usingDefaults, false);
  assertOrigin.equal(originState.normalizeCalibrationOrigin(undefined, originState.createLiveCalibrationProfile()).usingDefaults, true);
  assertOrigin.equal(originState.normalizeCalibrationOrigin({ source: "<script>" }, originOwnerProfile(3000)).known, false);
  originState.forgetRobotCalibrationProfile(legacy);
  assertOrigin.equal(originState.calibrationOriginWarning(legacy), null);
});

test("unsaved browser edits are never replaced when the robot reports defaults", () => {
  const state = originState.createLiveCalibrationState(originOwnerProfile(5000));
  state.dirty = true;
  state.profile.joints[0].offsetDeg = 4;
  const result = originState.acceptRobotCalibrationProfile(
    state, originState.createLiveCalibrationProfile(), "session-dirty", { source: "default-fallback" },
  );
  assertOrigin.equal(result.adopted, false);
  assertOrigin.equal(result.restored, false);
  assertOrigin.equal(state.profile.joints[0].offsetDeg, 4);
});

// --- Pre-send change summary: tonight's mistake (sending a defaults-based draft over the good profile) must be loud ---
test("the pre-send summary flags hip direction flips and zeroed trims against the robot's profile", () => {
  const robot = originState.createLiveCalibrationProfile();           // the verified baseline
  const draft = originState.createLiveCalibrationProfile();
  for (const joint of draft.joints) {
    joint.offsetDeg = 0;                                              // restore-defaults style draft
    joint.direction = [0, 14].includes(joint.logicalChannel) ? 1 : joint.direction;
  }
  const summary = originState.calibrationChangeSummary(robot, draft);
  assertOrigin.equal(summary.comparable, true);
  assertOrigin.deepEqual(summary.directionChanges.map((c) => c.label.match(/CH\d+/)[0]).sort(), ["CH0", "CH14"]);
  assertOrigin.equal(summary.directionChanges.find((c) => /CH0\)/.test(c.label)).from, "-");
  assertOrigin.equal(summary.directionChanges.find((c) => /CH0\)/.test(c.label)).to, "+");
  assertOrigin.ok(summary.offsetChanges.length >= 5, "the baseline trims being zeroed is listed");
  assertOrigin.ok(summary.total >= 7);
});

test("the pre-send summary reports no changes for an identical profile and cannot compare without a robot read", () => {
  const profile = originState.createLiveCalibrationProfile();
  assertOrigin.equal(originState.calibrationChangeSummary(profile, originState.createLiveCalibrationProfile()).total, 0);
  assertOrigin.equal(originState.calibrationChangeSummary(null, profile).comparable, false);
  const remapped = originState.createLiveCalibrationProfile();
  remapped.joints.find((joint) => joint.logicalChannel === 4).channel = 6;
  assertOrigin.equal(originState.calibrationChangeSummary(profile, remapped).channelChanges.length, 1);
});
// --- Saving to flash must not depend on bench mode, and a blocked send must say why ---
import { firmwareSavesWithoutBench as savesWithoutBench } from "./web/src/live-calibration-protocol.js";

const saveContext = (overrides = {}) => ({
  connectionReady: true, telemetryAgeMs: 100, robotState: "disarmed", calibrationSupported: true,
  sessionId: "s", workspaceActive: true, batteryLocked: false, pending: false, saveWithoutBench: false,
  ...overrides,
});
function readySaveState() {
  const state = originState.createLiveCalibrationState();
  originState.acceptRobotCalibrationProfile(state, originState.createLiveCalibrationProfile({ savedAt: 1000 }), "s");
  return state;
}

test("firmware 0.8.21 and later is recognised as saving without bench mode", () => {
  for (const version of ["0.8.21", "0.8.22", "0.9.0", "1.0.0", "0.8.21-beta"]) assertOrigin.equal(savesWithoutBench(version), true, version);
  for (const version of ["0.8.20", "0.8.19", "0.8.18", "0.7.99", "unknown", "", undefined, null, "test"]) assertOrigin.equal(savesWithoutBench(version), false, String(version));
});

test("old firmware still needs bench mode to save, and says so", () => {
  const state = readySaveState();
  state.safetyConfirmed = true;
  const reason = originState.calibrationControlBlockReason(state, saveContext(), "save-profile");
  assertOrigin.match(reason, /bench mode/i);
  state.benchModeAcknowledged = true;
  assertOrigin.equal(originState.calibrationControlBlockReason(state, saveContext(), "save-profile"), "");
});

test("supporting firmware saves without bench mode, the support checkbox or a healthy battery", () => {
  const state = readySaveState();
  assertOrigin.equal(state.benchModeAcknowledged, false);
  assertOrigin.equal(state.safetyConfirmed, false);
  assertOrigin.equal(
    originState.calibrationControlBlockReason(state, saveContext({ saveWithoutBench: true, batteryLocked: true }), "save-profile"), "");
  // Jogging is unaffected: it still needs all of them.
  assertOrigin.notEqual(
    originState.calibrationControlBlockReason(state, saveContext({ saveWithoutBench: true }), "jog"), "");
});

test("a flash-only save still requires a healthy link, a disarmed robot and a profile read", () => {
  const state = readySaveState();
  const ctx = (o) => saveContext({ saveWithoutBench: true, ...o });
  assertOrigin.match(originState.calibrationControlBlockReason(state, ctx({ connectionReady: false }), "save-profile"), /Connect/);
  assertOrigin.match(originState.calibrationControlBlockReason(state, ctx({ telemetryAgeMs: 5000 }), "save-profile"), /telemetry/i);
  assertOrigin.match(originState.calibrationControlBlockReason(state, ctx({ robotState: "armed" }), "save-profile"), /Disarm/);
  assertOrigin.match(originState.calibrationControlBlockReason(state, ctx({ pending: true }), "save-profile"), /Waiting/);
  assertOrigin.match(originState.calibrationControlBlockReason(state, ctx({ sessionId: "another" }), "save-profile"), /Read the active robot profile/);
});

test("the save command states whether it needs bench mode and both forms validate", () => {
  const state = readySaveState();
  const plain = originState.createCalibrationBenchCommand(state, "save-profile", "a", 5000);
  const flash = originState.createCalibrationBenchCommand(state, "save-profile", "b", 5000, null, { saveWithoutBench: true });
  assertOrigin.equal(plain.safety.benchModeRequired, true);
  assertOrigin.equal(flash.safety.benchModeRequired, false);
  assertOrigin.equal(validCalibrationCommand(plain), true);
  assertOrigin.equal(validCalibrationCommand(flash), true);
  assertOrigin.equal(validCalibrationCommand({ ...flash, safety: { ...flash.safety, benchModeRequired: "no" } }), false);
  // Other actions keep their fixed requirement.
  const jog = originState.createCalibrationBenchCommand(state, "jog", "c", 5000, null, { saveWithoutBench: true });
  assertOrigin.equal(jog.safety.benchModeRequired, true);
});