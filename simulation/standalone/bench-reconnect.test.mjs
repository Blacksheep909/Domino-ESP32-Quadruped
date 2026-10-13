import assert from "node:assert/strict";
import test from "node:test";

import { DOMINO_ROBOT_LINK_PROTOCOL, LiveCompanionCore } from "./live-companion-core.mjs";
import { createLiveCalibrationProfile } from "./web/src/live-calibration-state.js";
import { LIVE_CALIBRATION_NO_BENCH_ACTIONS } from "./web/src/live-calibration-protocol.js";

const adapterId = "adapter-bench";
const caps = { telemetry: true, calibration: true, motionSmoothing: true, gaitProfiles: true, persistentProfiles: true, manualControl: true };
const pose = (t) => ({ timestampMs: t, servoAngleDeg: Array(16).fill(135), body: { rollDeg: 0, pitchDeg: 0, yawDeg: 0, heightMm: 280 } });
const telemetry = (t) => ({
  type: "robot-telemetry", protocol: DOMINO_ROBOT_LINK_PROTOCOL, robotState: "disarmed", robotTimeMs: t,
  expected: pose(t), measured: pose(t),
  controller: { source: "boxer-elrs", frameTimestampMs: t, packetRateHz: 150, frameLossCount: 0, failsafe: false, failsafeCount: 0, linkQualityPercent: 95, rssi1Dbm: -62, rssi2Dbm: -65, snrDb: 8, rfMode: "250hz", txPowerMw: 100, activeAntenna: 1, receiverVoltageV: 5.1, channelsUs: Array(16).fill(1500) },
  power: { voltageV: 15.8, currentA: 1.2, powerW: 18.96 }, diagnostics: { robotState: "disarmed" },
});

function newCore(now) {
  const core = new LiveCompanionCore({ adapterId, transport: "usb", endpoint: "COM9" });
  core.handleRobot({ type: "robot-hello", protocol: DOMINO_ROBOT_LINK_PROTOCOL, robotId: "d", robotName: "D", firmwareVersion: "0.8.16", robotState: "disarmed", capabilities: caps, gaitProfile: { schemaVersion: 2, robot: "d", name: "x" } }, now);
  core.handleRobot(telemetry(now), now);
  return core;
}
const connect = (core, now, id) => core.handleRelay({ type: "live-connection-command", action: "connect", requestId: id, timestampMs: now, adapterId, transport: "usb", safety: { readOnlyHandshake: true, commandsBlockedUntilStateKnown: true } }, now);
const cal = (core, action, id, now, extra = {}) => core.handleRelay({
  type: "live-calibration-command", action, requestId: id, adapterId, sessionId: core.sessionId, timestampMs: now,
  safety: { benchModeRequired: !LIVE_CALIBRATION_NO_BENCH_ACTIONS.includes(action), maxSpeedDegPerSec: 5, jogLimitDeg: 10 },
  selectedChannel: 0, physicalChannel: 0, jogOffsetDeg: 0, targetServoDeg: 129.87, ...extra,
}, now);
const robotAck = (core, action, id, now, extra = {}) => core.handleRobot({
  protocol: DOMINO_ROBOT_LINK_PROTOCOL, type: "robot-ack", kind: "calibration", action, requestId: id,
  accepted: true, robotState: "disarmed", benchMode: action === "enter", supportsSafeJog: true, maxSpeedDegPerSec: 5, persisted: false, ...extra,
}, now);

test("bench mode can be re-entered after the engineering session is disconnected and reconnected", () => {
  const core = newCore(1_000);
  connect(core, 1_000, "c1");
  assert.equal(cal(core, "enter", "e1", 1_100).robot.length, 1);
  robotAck(core, "enter", "e1", 1_110);
  assert.equal(core.benchMode, true);

  // Studio disconnects; the physical robot link stays open.
  const first = core.sessionId;
  const out = core.handleRelay({ type: "live-connection-command", action: "disconnect", requestId: "d1", timestampMs: 1_200, adapterId, sessionId: first }, 1_200);
  assert.equal(core.benchMode, false);
  // The robot must be told to leave bench mode and de-energise its outputs.
  const exit = out.robot.find((message) => message.kind === "calibration" && message.action === "exit");
  assert.ok(exit, "disconnecting must send a calibration exit to the robot");

  connect(core, 1_300, "c2");
  assert.notEqual(core.sessionId, first);
  assert.equal(cal(core, "request-profile", "p2", 1_400).robot.length, 1);
  robotAck(core, "request-profile", "p2", 1_410, { profile: createLiveCalibrationProfile() });
  assert.equal(cal(core, "enter", "e2", 1_500).robot.length, 1, "enter must be forwarded in the new session");
  const result = robotAck(core, "enter", "e2", 1_510);
  assert.equal(result.relay[0].accepted, true);
  assert.equal(core.benchMode, true);
});

test("the robot link going stale clears bench mode and the next session can enter again", () => {
  const core = newCore(1_000);
  connect(core, 1_000, "c1");
  cal(core, "enter", "e1", 1_100);
  robotAck(core, "enter", "e1", 1_110);
  core.disconnectRobot(5_000, "robot-link-closed");
  assert.equal(core.benchMode, false);
  core.handleRobot({ type: "robot-hello", protocol: DOMINO_ROBOT_LINK_PROTOCOL, robotId: "d", robotName: "D", firmwareVersion: "0.8.16", robotState: "disarmed", capabilities: caps, gaitProfile: { schemaVersion: 2, robot: "d", name: "x" } }, 5_100);
  core.handleRobot(telemetry(5_100), 5_100);
  assert.equal(connect(core, 5_200, "c2").relay[0].accepted, true);
  assert.equal(cal(core, "enter", "e2", 5_300).robot.length, 1);
});

test("a profile can be saved to the robot without entering bench mode", () => {
  const core = newCore(1_000);
  connect(core, 1_000, "c1");
  const profile = createLiveCalibrationProfile({ savedAt: 2_000 });
  const forwarded = cal(core, "save-profile", "s1", 1_100, { baseSavedAt: 0, profile });
  assert.equal(forwarded.robot.length, 1, "save-profile must not require bench mode");
  assert.equal(forwarded.relay.length, 0);
});
