import test from "node:test";
import assert from "node:assert/strict";

import {
  archiveLiveSession,
  analyzeLiveSession,
  compareLiveSessions,
  createLiveSessionState,
  clearLiveSession,
  liveSessionCsv,
  liveSessionJson,
  liveSessionSummary,
  mergeArchivedLiveSessions,
  parseLiveSessionJson,
  recordLiveComparisonSample,
  removeArchivedLiveSession,
  sanitizeArchivedLiveSession,
  selectLiveSessionPlotSamples,
  startLiveSession,
  stopLiveSession,
} from "./web/src/live-session-state.js";

test("clears a stopped active session without clearing while recording", () => {
  const session = createLiveSessionState();
  session.status = "recording";
  session.samples = [{ elapsedMs: 0 }];
  assert.equal(clearLiveSession(session), false);
  session.status = "stopped";
  assert.equal(clearLiveSession(session), true);
  assert.equal(session.status, "idle");
  assert.deepEqual(session.samples, []);
});

const snapshot = (expectedTimestampMs = 1_000, measuredTimestampMs = 1_012) => ({
  expectedFresh: true,
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

test("exports model and electrical commands separately through recording and import", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  const source = snapshot();
  source.expected.modelServoAngleDeg = Array.from({ length: 16 }, (_, index) => 120 + index);
  recordLiveComparisonSample(session, source, 10_100);
  stopLiveSession(session, 10_200);
  const archived = archiveLiveSession([], session, "model-basis");
  source.expected.modelServoAngleDeg[0] = 250;
  session.samples[0].expectedModelJointAnglesDeg[0] = 240;
  const imported = parseLiveSessionJson(liveSessionJson(archived));
  assert.equal(imported.samples[0].expectedModelJointAnglesDeg[0], 120);
  assert.equal(imported.samples[0].expectedJointAnglesDeg[0], 130);
  const [header, row] = liveSessionCsv(imported).split("\n").map((line) => line.split(","));
  assert.equal(header.length, row.length);
  assert.equal(Number(row[header.indexOf("ch15_model_command_deg")]), 135);
  delete archived.samples[0].expectedModelJointAnglesDeg;
  assert.equal(parseLiveSessionJson(liveSessionJson(archived)).samples[0].expectedModelJointAnglesDeg, null);
});

test("retains reset and stage timing evidence in JSON and CSV", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  const source = snapshot();
  source.diagnostics = {
    bootId: 42, resetReason: 6, priorResetStage: 5, maximumLoopGapMs: 2712,
    maxImuStageUs: 850000, maxServoWriteUs: 75000, imuI2cErrors: 2,
    imuLastRequestBytes: 14, imuLastRequestUs: 380,
    imuLastFailedRequestBytes: 0, imuLastFailedRequestUs: 1003522,
    imuMaxRequestUs: 1003522,
    crsfBudgetHits: 3, crsfPendingBytes: 320, imuSampleAgeMs: 410,
  };
  recordLiveComparisonSample(session, source, 10_100);
  stopLiveSession(session, 10_200);
  const imported = parseLiveSessionJson(liveSessionJson(archiveLiveSession([], session, "cutout")));
  assert.equal(imported.samples[0].diagnostics.priorResetStage, 5);
  assert.equal(imported.samples[0].diagnostics.maxImuStageUs, 850000);
  assert.equal(imported.samples[0].diagnostics.imuLastFailedRequestBytes, 0);
  assert.equal(imported.samples[0].diagnostics.imuLastFailedRequestUs, 1003522);
  const [header, row] = liveSessionCsv(imported).split("\n").map((line) => line.split(","));
  assert.equal(header.length, row.length);
  assert.equal(Number(row[header.indexOf("max_imu_stage_us")]), 850000);
  assert.equal(Number(row[header.indexOf("imu_i2c_errors")]), 2);
  assert.equal(Number(row[header.indexOf("imu_last_request_bytes")]), 14);
  assert.equal(Number(row[header.indexOf("imu_last_failed_request_bytes")]), 0);
  assert.equal(Number(row[header.indexOf("imu_last_failed_request_us")]), 1003522);
  assert.equal(Number(row[header.indexOf("crsf_budget_hits")]), 3);
  assert.equal(Number(row[header.indexOf("crsf_pending_bytes")]), 320);
  assert.equal(Number(row[header.indexOf("imu_sample_age_ms")]), 410);
});

test("engineering export deduplicates onboard blackbox events and preserves them on import", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  const source = snapshot();
  source.diagnostics = { bootId: 42, blackboxEvents: [
    [1, 1, 0, 1, 0, 1, 0],
    [2, 1, 1500, 3, 2, 1003500, 20],
  ] };
  recordLiveComparisonSample(session, source, 10_100);
  source.expected.timestampMs += 100;
  source.measured.timestampMs += 100;
  recordLiveComparisonSample(session, source, 10_200);
  stopLiveSession(session, 10_300);
  const archived = archiveLiveSession([], session, "blackbox");
  const exported = JSON.parse(liveSessionJson(archived));
  assert.equal(exported.blackbox.events.length, 2);
  assert.equal(exported.blackbox.events[1].value, 1003500);
  const imported = parseLiveSessionJson(JSON.stringify(exported));
  assert.deepEqual(imported.samples[0].diagnostics.blackboxEvents[1],
    [2, 1, 1500, 3, 2, 1003500, 20]);
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
  assert.equal(session.samples[0].power.cellCount, 4);
  assert.equal(session.samples[0].power.averageCellVoltageV, 3.8);
  assert.equal(session.samples[0].power.estimatedChargePercent, 40);
});

test("preserves USB sensor evidence without reporting it as pack charge or minimum pack voltage", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  const usb = snapshot();
  usb.power.voltageV = 2.5;
  usb.diagnostics = { robotState: "disarmed" };
  assert.equal(recordLiveComparisonSample(session, usb, 10_100), true);
  assert.equal(session.samples[0].power.voltageV, 2.5);
  assert.equal(session.samples[0].power.packDetected, false);
  assert.equal(session.samples[0].power.estimatedChargePercent, null);
  assert.equal(analyzeLiveSession(session).minimumVoltageV, null);

  const pack = snapshot(1_100, 1_112);
  pack.power.voltageV = 13.2;
  assert.equal(recordLiveComparisonSample(session, pack, 10_200), true);
  assert.equal(analyzeLiveSession(session).minimumVoltageV, 13.2);
});

test("does not record unpaired or stopped telemetry", () => {
  const session = createLiveSessionState();
  assert.equal(recordLiveComparisonSample(session, snapshot()), false);
  startLiveSession(session, 10_000);
  assert.equal(recordLiveComparisonSample(session, { paired: false }, 10_100), false);
  stopLiveSession(session, 10_200);
  assert.equal(recordLiveComparisonSample(session, snapshot(), 10_300), false);
});

test("records diagnostics and commands without an IMU measurement", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  const source = snapshot();
  source.paired = false;
  source.measured = null;
  source.bodyError = null;
  source.diagnostics = { bootId: 77, imuOnline: false, crsfUartOverflows: 0 };
  assert.equal(recordLiveComparisonSample(session, source, 10_100), true);
  assert.equal(recordLiveComparisonSample(session, source, 10_200), false);
  stopLiveSession(session, 10_300);
  const imported = parseLiveSessionJson(liveSessionJson(session));
  assert.equal(imported.samples[0].measuredTimestampMs, null);
  assert.equal(imported.samples[0].measuredBody.pitchDeg, null);
  assert.equal(imported.samples[0].diagnostics.imuOnline, false);
  assert.equal(imported.samples[0].expectedBody.pitchDeg, 1);
  const [header, row] = liveSessionCsv(imported).split("\n").map((line) => line.split(","));
  assert.equal(header.length, row.length);
  assert.equal(row[header.indexOf("measured_timestamp_ms")], "");
  assert.equal(row[header.indexOf("measured_pitch_deg")], "");
  assert.equal(Number(row[header.indexOf("robot_boot_id")]), 77);
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
  assert.match(csv, /average_cell_voltage_v/);
  assert.match(csv, /estimated_charge_percent/);
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

test("imports a versioned engineering session for offline comparison", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  recordLiveComparisonSample(session, snapshot(), 10_100);
  stopLiveSession(session, 10_500);
  const exported = liveSessionJson({ ...session, id: "portable-run" }, 20_000);
  const imported = parseLiveSessionJson(exported);
  assert.equal(imported.id, "portable-run");
  assert.equal(imported.samples.length, 1);
  assert.equal(imported.samples[0].power.powerW, 45.6);
  assert.equal(imported.samples[0].expectedServoPhysicalChannels[1], 14);
});

test("session import fails closed for unknown, foreign, and malformed packages", () => {
  assert.throws(() => parseLiveSessionJson("not json"), /not valid JSON/);
  assert.throws(() => parseLiveSessionJson(JSON.stringify({ schema: "something-else", version: 1 })), /not a Domino/);
  assert.throws(() => parseLiveSessionJson(JSON.stringify({
    schema: "domino-live-engineering-session", version: 2, robotId: "domino-esp32-quadruped",
  })), /Unsupported/);
  assert.throws(() => parseLiveSessionJson(JSON.stringify({
    schema: "domino-live-engineering-session", version: 1, robotId: "another-robot",
  })), /different robot/);
  assert.throws(() => parseLiveSessionJson(JSON.stringify({
    schema: "domino-live-engineering-session", version: 1, robotId: "domino-esp32-quadruped",
    session: { id: "empty", startedAt: 1, stoppedAt: 2, samples: [] },
  })), /incomplete/);
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

test("physical IMU-only sessions round-trip without inventing height or joint feedback", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  const sample = snapshot();
  sample.measured.body.heightMm = null;
  sample.bodyError.heightMm = null;
  sample.bodyError.yawDeg = null;
  sample.measured.servoAngleDeg = null;
  sample.jointErrorsDeg.fill(null);
  sample.worstJointErrorDeg = null;
  recordLiveComparisonSample(session, sample, 10_100);
  stopLiveSession(session, 10_500);
  const imported = parseLiveSessionJson(liveSessionJson(session));
  assert.equal(imported.samples[0].measuredBody.heightMm, null);
  assert.equal(imported.samples[0].measuredJointAnglesDeg, null);
  assert.equal(analyzeLiveSession(imported).meanAbsHeightErrorMm, null);
  assert.equal(analyzeLiveSession(imported).meanAbsYawErrorDeg, null);
});

test("records reset, radio and detail timing evidence in independent JSON and CSV samples", () => {
  const session = createLiveSessionState();
  startLiveSession(session, 10_000);
  const sample = snapshot(10_000, 10_000);
  sample.expected.detailsTimestampMs = 9_300;
  sample.diagnostics = {
    bootId: 42, resetReason: 9, uptimeMs: 2_300, maximumLoopGapMs: 71,
    outputsEnabled: true, bodyMode: 2, radioControlEnabled: true,
    motionInputAwaitingCenter: false, crsfAcceptedFrames: 550,
    crsfCrcErrors: 3, crsfUartOverflows: 2, commandLatencyMs: 5,
  };
  sample.controller = {
    source: "boxer-elrs", frameTimestampMs: 9_995, packetRateHz: 250,
    linkQualityPercent: 99, rssi1Dbm: -60, failsafe: false,
    channelsUs: Array(16).fill(1_500),
  };
  recordLiveComparisonSample(session, sample, 10_100);
  stopLiveSession(session, 10_500);
  const archived = archiveLiveSession([], session);
  sample.controller.channelsUs[0] = 2_000;
  session.samples[0].controller.channelsUs[1] = 1_000;
  session.samples[0].diagnostics.bootId = 99;
  const imported = parseLiveSessionJson(liveSessionJson(archived));
  assert.equal(imported.samples[0].diagnostics.bootId, 42);
  assert.equal(imported.samples[0].controller.channelsUs[0], 1_500);
  assert.equal(imported.samples[0].controller.channelsUs[1], 1_500);
  assert.equal(imported.samples[0].expectedDetailsTimestampMs, 9_300);
  const [headers, values] = liveSessionCsv(imported).split("\n").map((line) => line.split(","));
  assert.equal(values.length, headers.length);
  const exported = Object.fromEntries(headers.map((key, index) => [key, values[index]]));
  assert.equal(exported.esp32_reset_reason, "9");
  assert.equal(exported.crsf_uart_overflows, "2");
  assert.equal(exported.awaiting_stick_center, "0");
  assert.equal(exported.crsf_ch16_us, "1500");
  assert.equal(exported.expected_details_timestamp_ms, "9300");
});
