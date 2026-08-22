import test from "node:test";
import assert from "node:assert/strict";

import {
  archiveLiveSession,
  analyzeLiveSession,
  compareLiveSessions,
  createLiveSessionState,
  liveSessionCsv,
  liveSessionJson,
  liveSessionSummary,
  mergeArchivedLiveSessions,
  recordLiveComparisonSample,
  removeArchivedLiveSession,
  sanitizeArchivedLiveSession,
  selectLiveSessionPlotSamples,
  startLiveSession,
  stopLiveSession,
} from "./web/src/live-session-state.js";

const snapshot = (expectedTimestampMs = 1_000, measuredTimestampMs = 1_012) => ({
  paired: true,
  expected: {
    timestampMs: expectedTimestampMs,
    body: { rollDeg: 0, pitchDeg: 1, yawDeg: 2, heightMm: 260 },
    servoAngleDeg: Array.from({ length: 16 }, (_, index) => 130 + index),
    servoPulseUs: Array.from({ length: 16 }, (_, index) => 1400 + index),
    servoPhysicalChannel: Array.from({ length: 16 }, (_, index) => 15 - index),
    footTargetMm: [[-15, 38, 280], [-15, -38, 281], [-15, 38, 282], [-15, -38, 283]],
  },
  measured: {
    timestampMs: measuredTimestampMs,
    body: { rollDeg: 0.5, pitchDeg: 2, yawDeg: 3, heightMm: 257 },
    servoAngleDeg: Array.from({ length: 16 }, (_, index) => 131 + index),
  },
  bodyError: { rollDeg: 0.5, pitchDeg: 1, yawDeg: 1, heightMm: -3 },
  alignmentMs: measuredTimestampMs - expectedTimestampMs,
  jointErrorsDeg: Array.from({ length: 16 }, (_, index) => index / 10),
  worstJointErrorDeg: 1.5,
  power: { voltageV: 15.2, currentA: 3, powerW: 45.6 },
});

test("validates restored sessions and merges newest-first without duplicates", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  recordLiveComparisonSample(session, snapshot(), 10_100);
  stopLiveSession(session, 10_500);
  const persisted = archiveLiveSession([], session, "run-1");
  assert.ok(sanitizeArchivedLiveSession(persisted));
  assert.equal(sanitizeArchivedLiveSession({ ...persisted, samples: [] }), null);
  const archive = [{ ...persisted, stoppedAt: 10_400 }];
  assert.equal(mergeArchivedLiveSessions(archive, [persisted, { nonsense: true }]), 1);
  assert.equal(archive.length, 1);
  assert.equal(archive[0].stoppedAt, 10_500);
});

test("analyzes and compares optimization metrics including integrated energy", () => {
  const build = (id, powerW, pitchError) => {
    const session = createLiveSessionState();
    startLiveSession(session, 10_000);
    const first = snapshot(1_000, 1_010);
    first.power.powerW = powerW;
    first.bodyError.pitchDeg = pitchError;
    recordLiveComparisonSample(session, first, 10_000);
    const second = snapshot(2_000, 2_010);
    second.power.powerW = powerW;
    second.bodyError.pitchDeg = pitchError;
    recordLiveComparisonSample(session, second, 13_600);
    stopLiveSession(session, 13_600);
    return archiveLiveSession([], session, id);
  };
  const baseline = build("baseline", 50, 2);
  const candidate = build("candidate", 40, 1);
  const analysis = analyzeLiveSession(baseline);
  assert.equal(analysis.meanAbsPitchErrorDeg, 2);
  assert.equal(analysis.energyWh, 0.05);
  const comparison = compareLiveSessions(baseline, candidate);
  assert.equal(comparison.delta.meanAbsPitchErrorDeg, -1);
  assert.equal(comparison.delta.averagePowerW, -10);
});

test("records each synchronized source pair once", () => {
  const session = createLiveSessionState();
  assert.equal(startLiveSession(session, 10_000), true);
  assert.equal(recordLiveComparisonSample(session, snapshot(), 10_100), true);
  assert.equal(recordLiveComparisonSample(session, snapshot(), 10_200), false);
  assert.equal(recordLiveComparisonSample(session, snapshot(1_100, 1_115), 10_300), true);
  assert.equal(session.samples.length, 2);
  assert.equal(session.samples[0].expectedJointAnglesDeg[0], 130);
  assert.equal(session.samples[0].measuredJointAnglesDeg[0], 131);
  assert.equal(session.samples[0].expectedFootTargetsMm[3][2], 283);
  assert.equal(session.samples[0].expectedServoPulseUs[1], 1401);
  assert.equal(session.samples[0].expectedServoPhysicalChannels[1], 14);
});

test("does not record unpaired or stopped telemetry", () => {
  const session = createLiveSessionState();
  assert.equal(recordLiveComparisonSample(session, snapshot()), false);
  startLiveSession(session, 10_000);
  assert.equal(recordLiveComparisonSample(session, { paired: false }, 10_100), false);
  stopLiveSession(session, 10_200);
  assert.equal(recordLiveComparisonSample(session, snapshot(), 10_300), false);
});

test("bounds the in-memory recording buffer", () => {
  const session = createLiveSessionState(2);
  startLiveSession(session, 10_000);
  recordLiveComparisonSample(session, snapshot(1_000, 1_010), 10_100);
  recordLiveComparisonSample(session, snapshot(1_100, 1_110), 10_200);
  recordLiveComparisonSample(session, snapshot(1_200, 1_210), 10_300);
  assert.equal(session.samples.length, 2);
  assert.equal(session.samples[0].expectedTimestampMs, 1_100);
});

test("reports duration and sample count for recording and stopped sessions", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  recordLiveComparisonSample(session, snapshot(), 10_100);
  assert.deepEqual(liveSessionSummary(session, 10_450), {
    status: "recording",
    sampleCount: 1,
    durationMs: 450,
  });
  stopLiveSession(session, 10_700);
  assert.equal(liveSessionSummary(session, 20_000).durationMs, 700);
});

test("selects true time-based scope windows and downsamples without losing endpoints", () => {
  const samples = Array.from({ length: 121 }, (_, index) => ({ elapsedMs: index * 1_000 }));
  const thirtySeconds = selectLiveSessionPlotSamples(samples, 30);
  assert.equal(thirtySeconds.length, 31);
  assert.equal(thirtySeconds[0].elapsedMs, 90_000);
  assert.equal(thirtySeconds.at(-1).elapsedMs, 120_000);
  const fullDownsampled = selectLiveSessionPlotSamples(samples, "all", 10);
  assert.equal(fullDownsampled.length, 10);
  assert.equal(fullDownsampled[0].elapsedMs, 0);
  assert.equal(fullDownsampled.at(-1).elapsedMs, 120_000);
});

test("exports analysis-ready CSV with power, pose, timing and joint error", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  recordLiveComparisonSample(session, snapshot(), 10_100);
  const csv = liveSessionCsv(session);
  assert.match(csv, /expected_pitch_deg/);
  assert.match(csv, /joint_15_error_deg/);
  assert.match(csv, /joint_15_expected_deg/);
  assert.match(csv, /joint_15_command_pulse_us/);
  assert.match(csv, /joint_15_pca_output/);
  assert.match(csv, /fl_foot_target_z_mm/);
  assert.match(csv, /45\.6000/);
  assert.equal(csv.split("\n").length, 2);
});

test("exports a versioned engineering JSON package with raw samples and analysis", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  recordLiveComparisonSample(session, snapshot(), 10_100);
  stopLiveSession(session, 10_500);
  const exported = JSON.parse(liveSessionJson(session, 20_000));
  assert.equal(exported.schema, "domino-live-engineering-session");
  assert.equal(exported.version, 1);
  assert.equal(exported.exportedAt, 20_000);
  assert.equal(exported.session.samples[0].expectedServoPulseUs[1], 1401);
  assert.equal(exported.session.samples[0].expectedServoPhysicalChannels[1], 14);
  assert.equal(exported.analysis.sampleCount, 1);
  assert.match(exported.signalSemantics.measured, /physical feedback/);
  assert.equal(liveSessionJson({ samples: [] }), "");
});

test("archives stopped sessions without sharing mutable sample objects", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  recordLiveComparisonSample(session, snapshot(), 10_100);
  stopLiveSession(session, 10_500);
  const archive = [];
  const entry = archiveLiveSession(archive, session, "run-1");
  assert.equal(entry.id, "run-1");
  assert.equal(archive.length, 1);
  session.samples[0].bodyError.pitchDeg = 99;
  assert.equal(entry.samples[0].bodyError.pitchDeg, 1);
  assert.equal(removeArchivedLiveSession(archive, "run-1"), true);
  assert.equal(archive.length, 0);
});
