import test from "node:test";
import assert from "node:assert/strict";

import {
  acceptLiveTelemetryPacket,
  createLiveTelemetryState,
  liveCommandDisplayFrame,
  liveComparisonSnapshot,
  liveModelServoAngles,
  signedAngleErrorDeg,
} from "./web/src/live-telemetry-state.js";

const pose = (timestampMs, angle = 0, body = {}) => ({
  timestampMs,
  servoAngleDeg: Array(16).fill(angle),
  body: { rollDeg: 0, pitchDeg: 0, yawDeg: 0, heightMm: 260, ...body },
  footTargetMm: [[-15, 38, 280], [-15, -38, 280], [-15, 38, 280], [-15, -38, 280]],
});

test("renders the robot's model commands without replacing electrical evidence", () => {
  const state = createLiveTelemetryState();
  const expected = { ...pose(10_000, 140), modelServoAngleDeg: Array(16).fill(135) };
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry", sequence: 1, expected,
  }, 20_000), true);
  expected.modelServoAngleDeg[0] = 200;
  const current = liveComparisonSnapshot(state, 20_050).expected;
  assert.equal(liveModelServoAngles(current)[0], 135);
  assert.equal(current.servoAngleDeg[0], 140);
  // A legacy or reconnecting robot must not inherit a stale model command.
  acceptLiveTelemetryPacket(state, {
    type: "live-telemetry", sequence: 2, expected: pose(10_100, 141),
  }, 20_100);
  assert.equal(state.expected.modelServoAngleDeg, null);
  assert.equal(liveModelServoAngles(state.expected)[0], 141);
});

test("live display advances body and calibrated joints as one telemetry frame", () => {
  const state = createLiveTelemetryState();
  const axes = ["rollDeg", "pitchDeg", "yawDeg"];
  for (let index = 0; index < axes.length; index += 1) {
    const angle = 10 + index;
    const modelAngle = 130 + index;
    const expected = {
      ...pose(10_000 + index * 100, 150 + index, { [axes[index]]: angle }),
      modelServoAngleDeg: Array(16).fill(modelAngle),
    };
    assert.equal(acceptLiveTelemetryPacket(state, {
      type: "live-telemetry", sequence: index + 1, expected,
    }, 20_000 + index * 100), true);
    const frame = liveCommandDisplayFrame(
      liveComparisonSnapshot(state, 20_000 + index * 100).expected,
    );
    assert.equal(frame.body[axes[index]], angle);
    assert.equal(frame.servoAngleDeg[0], modelAngle);
  }
});

test("rejects an incomplete model command instead of drawing a mixed pose", () => {
  const state = createLiveTelemetryState();
  for (const invalid of [Array(12).fill(135), Array(16).fill(null)]) {
    assert.equal(acceptLiveTelemetryPacket(state, {
      type: "live-telemetry", sequence: 1,
      expected: { ...pose(10_000), modelServoAngleDeg: invalid },
    }, 20_000), false);
  }
});

test("preserves calibrated pulse and mapped PCA output metadata", () => {
  const state = createLiveTelemetryState();
  const expected = {
    ...pose(10_000, 135),
    servoPulseUs: Array.from({ length: 16 }, (_, channel) => 1400 + channel),
    servoPhysicalChannel: Array.from({ length: 16 }, (_, channel) => 15 - channel),
  };
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 1,
    expected,
  }, 20_000), true);
  const snapshot = liveComparisonSnapshot(state, 20_100);
  assert.equal(snapshot.expected.servoPulseUs[4], 1404);
  assert.equal(snapshot.expected.servoPhysicalChannel[4], 11);
});

test("retains slow pose details across lean fast telemetry packets", () => {
  const state = createLiveTelemetryState();
  const detailed = {
    ...pose(10_000, 135),
    servoPulseUs: Array.from({ length: 16 }, (_, channel) => 1400 + channel),
    servoPhysicalChannel: Array.from({ length: 16 }, (_, channel) => channel),
  };
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry", sequence: 1, expected: detailed,
  }, 20_000), true);
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 2,
    expected: {
      timestampMs: 10_100,
      servoAngleDeg: Array(16).fill(136),
      body: { rollDeg: 1, pitchDeg: 2, yawDeg: 0, heightMm: 260 },
    },
  }, 20_100), true);

  const snapshot = liveComparisonSnapshot(state, 20_150);
  assert.equal(snapshot.expected.servoAngleDeg[4], 136);
  assert.equal(snapshot.expected.servoPulseUs[4], 1404);
  assert.equal(snapshot.expected.servoPhysicalChannel[4], 4);
  assert.equal(snapshot.expected.footTargetMm[0][2], 280);
  assert.equal(snapshot.expected.detailsTimestampMs, 10_000);
  assert.equal(snapshot.expected.timestampMs, 10_100);
});

test("accepts independently timestamped expected and measured poses", () => {
  const state = createLiveTelemetryState();
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 1,
    expected: pose(10_000, 5),
    measured: pose(10_012, 7),
    power: { voltageV: 15.2, currentA: 3 },
  }, 20_000), true);
  const snapshot = liveComparisonSnapshot(state, 20_100);
  assert.equal(snapshot.paired, true);
  assert.equal(snapshot.alignmentMs, 12);
  assert.equal(snapshot.worstJointErrorDeg, 2);
  assert.equal(snapshot.expected.footTargetMm[0][2], 280);
  assert.ok(Math.abs(snapshot.power.powerW - 45.6) < 1e-9);
});

test("comparison snapshots retain same-packet reboot and controller evidence", () => {
  const state = createLiveTelemetryState();
  const controller = { source: "boxer-elrs", frameTimestampMs: 9_995, packetRateHz: 250,
    channelsUs: Array(16).fill(1_500), linkQualityPercent: 99, rssi1Dbm: -60, failsafe: false };
  const diagnostics = { uptimeMs: 8_000, bootId: 4, resetReason: 9,
    outputsEnabled: false, crsfUartOverflows: 7, servoLimitClipCount: 2 };
  acceptLiveTelemetryPacket(state, {
    type: "live-telemetry", sequence: 1, expected: pose(10_000),
    measured: { timestampMs: 10_000, body: { rollDeg: 1, pitchDeg: 2 } },
    diagnostics, controller,
  }, 20_000);
  controller.channelsUs[0] = 2_000;
  diagnostics.bootId = 99;
  const snapshot = liveComparisonSnapshot(state, 20_050);
  assert.equal(snapshot.controller.channelsUs[0], 1_500);
  assert.equal(snapshot.diagnostics.bootId, 4);
  assert.equal(snapshot.diagnostics.outputsEnabled, false);
  assert.equal(snapshot.diagnostics.crsfUartOverflows, 7);
  assert.equal(snapshot.diagnostics.jointLimitClips, 2);
});

test("accepts PCB voltage telemetry without inventing current or watts", () => {
  const state = createLiveTelemetryState();
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 1,
    power: { voltageV: 15.84 },
  }, 20_000), true);
  const snapshot = liveComparisonSnapshot(state, 20_100);
  assert.equal(snapshot.power.voltageV, 15.84);
  assert.equal(snapshot.power.currentA, null);
  assert.equal(snapshot.power.powerW, null);
});

test("accepts honest IMU-only measured attitude without fabricating joint feedback", () => {
  const state = createLiveTelemetryState();
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 1,
    expected: pose(10_000, 5),
    measured: { timestampMs: 10_004, body: { rollDeg: 1.5, pitchDeg: -2 } },
  }, 20_000), true);
  const snapshot = liveComparisonSnapshot(state, 20_100);
  assert.equal(snapshot.measuredFresh, true);
  assert.equal(snapshot.bodyError.rollDeg, 1.5);
  assert.equal(snapshot.bodyError.pitchDeg, -2);
  assert.equal(snapshot.bodyError.yawDeg, null);
  assert.equal(snapshot.worstJointErrorDeg, null);
  assert.equal(snapshot.measured.servoAngleDeg, null);
});

test("accepts camera telemetry in both the navigation envelope and legacy top-level slot", () => {
  const state = createLiveTelemetryState();
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 1,
    camera: { timestampMs: 10_000, yawDeg: 12, pitchDeg: -6, fovDeg: 88, fps: 30 },
  }, 20_000), true);
  let snapshot = liveComparisonSnapshot(state, 20_100);
  assert.equal(snapshot.navigation.camera.yawDeg, 12);
  assert.equal(snapshot.navigation.camera.fps, 30);
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 2,
    navigation: { camera: { timestampMs: 10_100, yawDeg: 18 } },
  }, 20_100), true);
  snapshot = liveComparisonSnapshot(state, 20_150);
  assert.equal(snapshot.navigation.camera.yawDeg, 18);
  assert.equal(snapshot.navigation.camera.fps, 30);
});

test("rejects malformed and out-of-order robot packets", () => {
  const state = createLiveTelemetryState();
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 2,
    expected: pose(10_000),
  }, 20_000), true);
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 2,
    measured: pose(10_010),
  }, 20_010), false);
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 3,
    measured: { ...pose(10_020), servoAngleDeg: [0, 1] },
  }, 20_020), false);
  assert.equal(acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 4,
    expected: { ...pose(10_030), footTargetMm: [[0, 0]] },
  }, 20_030), false);
});

test("stale streams disappear instead of masquerading as live telemetry", () => {
  const state = createLiveTelemetryState();
  acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 1,
    expected: pose(10_000),
    measured: pose(10_000),
  }, 20_000);
  const snapshot = liveComparisonSnapshot(state, 21_001);
  assert.equal(snapshot.expectedFresh, false);
  assert.equal(snapshot.measuredFresh, false);
  assert.equal(snapshot.paired, false);
  assert.equal(snapshot.worstJointErrorDeg, null);
});

test("angular errors take the shortest signed path across 360 degrees", () => {
  assert.equal(signedAngleErrorDeg(1, 359), 2);
  assert.equal(signedAngleErrorDeg(359, 1), -2);
});

test("body and joint errors remain unavailable until both streams are fresh", () => {
  const state = createLiveTelemetryState();
  acceptLiveTelemetryPacket(state, {
    type: "live-telemetry",
    sequence: 1,
    measured: pose(10_000, 8, { pitchDeg: 4 }),
  }, 20_000);
  const snapshot = liveComparisonSnapshot(state, 20_100);
  assert.equal(snapshot.measuredFresh, true);
  assert.equal(snapshot.expectedFresh, false);
  assert.equal(snapshot.bodyError, null);
  assert.deepEqual(snapshot.jointErrorsDeg.filter(Number.isFinite), []);
});
