import assert from "node:assert/strict";
import test from "node:test";

import { DOMINO_ROBOT_LINK_PROTOCOL, LiveCompanionCore } from "./live-companion-core.mjs";

const adapterId = "adapter-test";
const hello = (robotState = "disarmed") => ({
  type: "robot-hello", protocol: DOMINO_ROBOT_LINK_PROTOCOL,
  robotId: "domino-test", robotName: "Domino test", firmwareVersion: "test", robotState,
  capabilities: { telemetry: true, calibration: true, gaitProfiles: true, persistentProfiles: true, manualControl: true },
});
const pose = (timestampMs) => ({
  timestampMs,
  servoAngleDeg: Array(16).fill(135),
  body: { rollDeg: 0, pitchDeg: 0, yawDeg: 0, heightMm: 280 },
});
const controller = (timestampMs) => ({
  source: "boxer-elrs", frameTimestampMs: timestampMs, packetRateHz: 150,
  frameLossCount: 0, failsafe: false, failsafeCount: 0,
  linkQualityPercent: 95, rssi1Dbm: -62, rssi2Dbm: -65,
  snrDb: 8, rfMode: "250hz", txPowerMw: 100,
  activeAntenna: 1, receiverVoltageV: 5.1, channelsUs: Array(16).fill(1500),
});
const telemetry = (timestampMs, robotState = "disarmed") => ({
  type: "robot-telemetry", protocol: DOMINO_ROBOT_LINK_PROTOCOL, robotState,
  robotTimeMs: timestampMs,
  expected: pose(timestampMs), measured: pose(timestampMs), controller: controller(timestampMs),
  power: { voltageV: 15.8, currentA: 1.2, powerW: 18.96 },
  diagnostics: { robotState },
});

function connectedCore(now = 1_000) {
  const core = new LiveCompanionCore({ adapterId, transport: "wifi", endpoint: "192.168.4.1:8766" });
  core.handleRobot(hello(), now);
  core.handleRobot(telemetry(now), now);
  const result = core.handleRelay({
    type: "live-connection-command", action: "connect", requestId: "connect-1",
    timestampMs: now, adapterId, transport: "wifi",
    safety: { readOnlyHandshake: true, commandsBlockedUntilStateKnown: true },
  }, now);
  assert.equal(result.relay[0].accepted, true);
  return core;
}

test("announces only a fresh physical robot and negotiates a read-only session", () => {
  const core = new LiveCompanionCore({ adapterId, transport: "wifi" });
  assert.equal(core.announcement(1_000).state, "error");
  core.handleRobot(hello(), 1_000);
  core.handleRobot(telemetry(1_000), 1_000);
  assert.equal(core.announcement(1_100).state, "available");
  assert.equal(core.announcement(1_100).capabilities.manualControl, true);
  const connected = connectedCore();
  assert.ok(connected.sessionId);
  assert.equal(connected.announcement(1_050).state, "connected");
});

test("telemetry is published only after session negotiation", () => {
  const core = new LiveCompanionCore({ adapterId });
  assert.equal(core.handleRobot(telemetry(1_000), 1_000).relay.length, 0);
  const connected = connectedCore();
  const result = connected.handleRobot(telemetry(1_100), 1_100);
  assert.equal(result.relay[0].type, "live-telemetry");
  assert.equal(result.relay[0].sessionId, connected.sessionId);
});

test("robot monotonic timestamps are translated into the host clock domain", () => {
  const core = connectedCore(10_000);
  const result = core.handleRobot(telemetry(10_020), 1_720_000_000_000);
  assert.equal(result.relay[0].expected.timestampMs, 1_720_000_000_000);
  assert.equal(result.relay[0].controller.frameTimestampMs, 1_720_000_000_000);
});

test("arming is rejected without a fresh healthy controller link", () => {
  const core = connectedCore();
  core.controller = { ...core.controller, failsafe: true };
  const result = core.handleRelay({
    type: "live-safety-command", action: "arm", requestId: "arm-1", timestampMs: 1_050,
    adapterId, sessionId: core.sessionId,
    safety: { holdMs: 1_500, requiresDisarmed: true, requiresFreshTelemetry: true, requiresDriveLink: true, watchdogMs: 400 },
  }, 1_050);
  assert.equal(result.relay[0].accepted, false);
  assert.match(result.relay[0].reason, /Boxer\/ELRS/);
  assert.equal(result.robot.length, 0);
});

test("physical acknowledgements gate safety state and heartbeat responses", () => {
  const core = connectedCore();
  const command = {
    type: "live-safety-command", action: "arm", requestId: "arm-1", timestampMs: 1_050,
    adapterId, sessionId: core.sessionId,
    safety: { holdMs: 1_500, requiresDisarmed: true, requiresFreshTelemetry: true, requiresDriveLink: true, watchdogMs: 400 },
  };
  const forwarded = core.handleRelay(command, 1_050);
  assert.equal(forwarded.relay.length, 0);
  assert.equal(forwarded.robot[0].kind, "safety");
  const ack = core.handleRobot({
    type: "robot-ack", protocol: DOMINO_ROBOT_LINK_PROTOCOL, kind: "safety",
    action: "arm", requestId: "arm-1", accepted: true, robotState: "armed",
  }, 1_080);
  assert.equal(ack.relay[0].accepted, true);
  assert.equal(core.robotState, "armed");
  const heartbeat = core.handleRelay({
    type: "live-safety-heartbeat", adapterId, sessionId: core.sessionId,
    sequence: 2, timestampMs: 1_100, workspaceActive: true,
  }, 1_100);
  assert.equal(heartbeat.robot[0].kind, "safety-heartbeat");
  const replay = core.handleRelay({
    type: "live-safety-heartbeat", adapterId, sessionId: core.sessionId,
    sequence: 2, timestampMs: 1_110, workspaceActive: true,
  }, 1_110);
  assert.equal(replay.robot.length, 0);
  assert.equal(core.tick(1_501).robot.at(-1).action, "watchdog");
  assert.equal(core.robotState, "watchdog");
});

test("manual control requires robot authority and neutralizes after 250 ms", () => {
  const core = connectedCore();
  core.robotState = "armed";
  core.handleRobot(telemetry(1_020, "armed"), 1_020);
  const request = {
    type: "live-manual-authority-command", action: "request-authority", requestId: "manual-1",
    timestampMs: 1_030, adapterId, sessionId: core.sessionId, requestedLeaseMs: 30_000,
    safety: { requiresArmed: true, requiresDeadman: true, neutralOnRelease: true, commandTimeoutMs: 250 },
  };
  const forwarded = core.handleRelay(request, 1_030);
  const token = forwarded.robot[0].authorityToken;
  assert.ok(token);
  const acknowledged = core.handleRobot({
    type: "robot-ack", protocol: DOMINO_ROBOT_LINK_PROTOCOL, kind: "manual-authority",
    action: "request-authority", requestId: "manual-1", accepted: true, robotState: "armed",
  }, 1_040);
  assert.equal(acknowledged.relay[0].authorityToken, token);
  const frame = core.handleRelay({
    type: "live-manual-control-frame", adapterId, sessionId: core.sessionId,
    authorityToken: token, sequence: 0, timestampMs: 1_050, deadman: true, neutral: false,
    mode: "careful", axes: { forward: 0.2, turn: -0.1, roll: 0, height: 0 },
    safety: { requiresArmed: true, neutralOnExpiry: true, timeoutMs: 250 },
  }, 1_050);
  assert.equal(frame.robot[0].kind, "manual-control");
  assert.equal(frame.robot[0].deadman, true);
  const futureFrame = core.handleRelay({
    type: "live-manual-control-frame", adapterId, sessionId: core.sessionId,
    authorityToken: token, sequence: 1, timestampMs: 2_000, deadman: true, neutral: false,
    mode: "careful", axes: { forward: 0.2, turn: 0, roll: 0, height: 0 },
    safety: { requiresArmed: true, neutralOnExpiry: true, timeoutMs: 250 },
  }, 1_060);
  assert.equal(futureFrame.robot.length, 0);
  const timedOut = core.tick(1_301);
  assert.equal(timedOut.robot[0].action, "neutral");
  assert.equal(core.manualAuthority, null);
});

test("calibration and gait writes fail closed while armed", () => {
  const core = connectedCore();
  core.robotState = "armed";
  const calibration = core.handleRelay({
    type: "live-calibration-command", action: "enter", requestId: "cal-1",
    adapterId, sessionId: core.sessionId,
    safety: { benchModeRequired: true, maxSpeedDegPerSec: 5, jogLimitDeg: 10 },
  }, 1_050);
  assert.equal(calibration.relay[0].accepted, false);
  const gait = core.handleRelay({
    type: "live-gait-command", action: "apply-profile", requestId: "gait-1",
    adapterId, sessionId: core.sessionId,
    safety: { requiresDisarmed: true, twoStageApply: true },
    profile: { schemaVersion: 1, robot: "domino-esp32-quadruped" },
  }, 1_050);
  assert.equal(gait.relay[0].accepted, false);
  assert.equal(gait.robot.length, 0);
});

test("physical request timeouts return explicit rejection acknowledgements", () => {
  const core = connectedCore();
  core.handleRelay({
    type: "live-safety-command", action: "request-state", requestId: "state-1", timestampMs: 1_050,
    adapterId, sessionId: core.sessionId,
  }, 1_050);
  const result = core.tick(3_051);
  assert.equal(result.relay[0].accepted, false);
  assert.match(result.relay[0].reason, /timed out/);
});

test("malformed physical messages cannot keep the robot or controller link fresh", () => {
  const core = connectedCore();
  const lastRobotMessageAt = core.lastRobotMessageAt;
  core.handleRobot({ protocol: DOMINO_ROBOT_LINK_PROTOCOL, type: "robot-telemetry", robotState: "disarmed" }, 1_500);
  core.handleRobot({ protocol: DOMINO_ROBOT_LINK_PROTOCOL, type: "made-up" }, 1_600);
  assert.equal(core.lastRobotMessageAt, lastRobotMessageAt);
  assert.equal(core.robotFresh(2_001), false);
});
