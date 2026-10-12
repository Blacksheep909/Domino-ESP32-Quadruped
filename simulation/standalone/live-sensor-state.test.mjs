import test from "node:test";
import assert from "node:assert/strict";

import {
  captureLiveSensorLevel,
  createLiveSensorAttitudeFilterState,
  createLiveSensorCalibrationState,
  filterLiveSensorAttitude,
  liveSensorSnapshot,
  resetLiveSensorLevel,
} from "./web/src/live-sensor-state.js";

const comparison = {
  measuredFresh: true,
  measured: { body: { rollDeg: 12, pitchDeg: -4, yawDeg: 37 } },
};
const diagnostics = { imuOnline: true, imuAxG: 0.1, imuAyG: 0.2, imuAzG: 0.97 };

test("produces honest live IMU attitude and gravity magnitude", () => {
  const snapshot = liveSensorSnapshot(comparison, diagnostics, createLiveSensorCalibrationState());
  assert.equal(snapshot.online, true);
  assert.equal(snapshot.rollDeg, 12);
  assert.equal(snapshot.pitchDeg, -4);
  assert.equal(snapshot.yawDeg, 37);
  assert.ok(Math.abs(snapshot.gravityMagnitudeG - Math.sqrt(0.1 ** 2 + 0.2 ** 2 + 0.97 ** 2)) < 1e-9);
});

test("captures and resets a browser-side level reference", () => {
  const calibration = createLiveSensorCalibrationState();
  const raw = liveSensorSnapshot(comparison, diagnostics, calibration);
  assert.equal(captureLiveSensorLevel(calibration, raw, 1234), true);
  const level = liveSensorSnapshot(comparison, diagnostics, calibration);
  assert.equal(level.rollDeg, 0);
  assert.equal(level.pitchDeg, 0);
  assert.equal(level.yawDeg, 0);
  assert.equal(level.capturedAt, 1234);
  assert.equal(resetLiveSensorLevel(calibration), true);
  assert.equal(liveSensorSnapshot(comparison, diagnostics, calibration).levelCaptured, false);
});

test("refuses calibration when fresh physical IMU telemetry is absent", () => {
  const calibration = createLiveSensorCalibrationState();
  const offline = liveSensorSnapshot({ ...comparison, measuredFresh: false }, diagnostics, calibration);
  assert.equal(offline.online, false);
  assert.equal(captureLiveSensorLevel(calibration, offline), false);
});

test("does not fabricate zero yaw for firmware that omits heading", () => {
  const legacy = liveSensorSnapshot({
    ...comparison,
    measured: { body: { rollDeg: 12, pitchDeg: -4, yawDeg: null } },
  }, diagnostics, createLiveSensorCalibrationState());
  assert.equal(legacy.online, true);
  assert.equal(legacy.yawDeg, null);
});

test("unwraps the Euler boundary instead of animating a 358 degree flip", () => {
  const filter = createLiveSensorAttitudeFilterState();
  const first = filterLiveSensorAttitude(filter, { online: true, rollDeg: 179, pitchDeg: 0, yawDeg: 179 }, 1_000);
  assert.equal(first.displayRollDeg, 179);
  const crossed = filterLiveSensorAttitude(filter, { online: true, rollDeg: -179, pitchDeg: 0, yawDeg: -179 }, 1_100);
  assert.ok(crossed.displayRollDeg > 179 && crossed.displayRollDeg < 181);
  assert.ok(crossed.displayYawDeg > 179 && crossed.displayYawDeg < 181);
});

test("rate limits implausible attitude jumps", () => {
  const filter = createLiveSensorAttitudeFilterState();
  filterLiveSensorAttitude(filter, { online: true, rollDeg: 0, pitchDeg: 0, yawDeg: 0 }, 1_000);
  const jumped = filterLiveSensorAttitude(filter, { online: true, rollDeg: 120, pitchDeg: -120, yawDeg: 120 }, 1_100);
  assert.ok(Math.abs(jumped.displayRollDeg) <= 24);
  assert.ok(Math.abs(jumped.displayPitchDeg) <= 24);
  assert.ok(Math.abs(jumped.displayYawDeg) <= 24);
});
