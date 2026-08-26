import test from "node:test";
import assert from "node:assert/strict";

import {
  acceptLiveTelemetryPacket,
  createLiveTelemetryState,
  liveComparisonSnapshot,
  signedAngleErrorDeg,
} from "./web/src/live-telemetry-state.js";

const pose = (timestampMs, angle = 0, body = {}) => ({
  timestampMs,
  servoAngleDeg: Array(16).fill(angle),
  body: { rollDeg: 0, pitchDeg: 0, yawDeg: 0, heightMm: 260, ...body },
  footTargetMm: [[-15, 38, 280], [-15, -38, 280], [-15, 38, 280], [-15, -38, 280]],
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
