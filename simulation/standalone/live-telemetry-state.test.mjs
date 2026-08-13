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
  assert.ok(Math.abs(snapshot.power.powerW - 45.6) < 1e-9);
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
