import { LIVE_SERVO_CHANNELS } from "./live-telemetry-state.js";

export const LIVE_SESSION_MAX_SAMPLES = 18_000;

export function createLiveSessionState(maxSamples = LIVE_SESSION_MAX_SAMPLES) {
  return {
    status: "idle",
    startedAt: null,
    stoppedAt: null,
    samples: [],
    maxSamples: Math.max(1, Math.floor(Number(maxSamples) || LIVE_SESSION_MAX_SAMPLES)),
    lastSourceKey: "",
  };
}

export function startLiveSession(state, startedAt = Date.now()) {
  if (!state || state.status === "recording") return false;
  state.status = "recording";
  state.startedAt = startedAt;
  state.stoppedAt = null;
  state.samples = [];
  state.lastSourceKey = "";
  return true;
}

export function stopLiveSession(state, stoppedAt = Date.now()) {
  if (!state || state.status !== "recording") return false;
  state.status = "stopped";
  state.stoppedAt = stoppedAt;
  return true;
}

export function recordLiveComparisonSample(state, snapshot, capturedAt = Date.now()) {
  if (!state || state.status !== "recording" || !snapshot?.paired) return false;
  const sourceKey = `${snapshot.expected.timestampMs}:${snapshot.measured.timestampMs}`;
  if (sourceKey === state.lastSourceKey) return false;
  const sample = {
    capturedAt,
    elapsedMs: Math.max(0, capturedAt - state.startedAt),
    expectedTimestampMs: snapshot.expected.timestampMs,
    measuredTimestampMs: snapshot.measured.timestampMs,
    alignmentMs: snapshot.alignmentMs,
    expectedBody: { ...snapshot.expected.body },
    measuredBody: { ...snapshot.measured.body },
    bodyError: { ...snapshot.bodyError },
    worstJointErrorDeg: snapshot.worstJointErrorDeg,
    jointErrorsDeg: LIVE_SERVO_CHANNELS.map((channel) => snapshot.jointErrorsDeg[channel]),
    power: snapshot.power ? { ...snapshot.power } : null,
  };
  state.samples.push(sample);
  if (state.samples.length > state.maxSamples) {
    state.samples.splice(0, state.samples.length - state.maxSamples);
  }
  state.lastSourceKey = sourceKey;
  return true;
}

export function liveSessionSummary(state, now = Date.now()) {
  if (!state) return { status: "idle", sampleCount: 0, durationMs: 0 };
  const end = state.status === "recording" ? now : state.stoppedAt;
  return {
    status: state.status,
    sampleCount: state.samples.length,
    durationMs: state.startedAt && end ? Math.max(0, end - state.startedAt) : 0,
  };
}

export function archiveLiveSession(archive, state, identifier = `session-${Date.now()}`, maximumEntries = 20) {
  if (!Array.isArray(archive) || !state || state.status === "recording" || state.samples.length === 0) {
    return null;
  }
  const entry = {
    id: String(identifier),
    startedAt: state.startedAt,
    stoppedAt: state.stoppedAt,
    samples: state.samples.map((sample) => ({
      ...sample,
      expectedBody: { ...sample.expectedBody },
      measuredBody: { ...sample.measuredBody },
      bodyError: { ...sample.bodyError },
      jointErrorsDeg: [...sample.jointErrorsDeg],
      power: sample.power ? { ...sample.power } : null,
    })),
  };
  archive.unshift(entry);
  archive.splice(Math.max(1, Math.floor(maximumEntries)));
  return entry;
}

export function removeArchivedLiveSession(archive, identifier) {
  if (!Array.isArray(archive)) return false;
  const index = archive.findIndex((entry) => entry.id === identifier);
  if (index < 0) return false;
  archive.splice(index, 1);
  return true;
}

const csvNumber = (value, digits = 4) => Number.isFinite(value) ? Number(value).toFixed(digits) : "";

export function liveSessionCsv(state) {
  const jointHeaders = LIVE_SERVO_CHANNELS.map((channel) => `joint_${channel}_error_deg`);
  const headers = [
    "captured_at_iso",
    "elapsed_ms",
    "expected_timestamp_ms",
    "measured_timestamp_ms",
    "alignment_ms",
    "expected_roll_deg",
    "measured_roll_deg",
    "roll_error_deg",
    "expected_pitch_deg",
    "measured_pitch_deg",
    "pitch_error_deg",
    "expected_yaw_deg",
    "measured_yaw_deg",
    "yaw_error_deg",
    "expected_height_mm",
    "measured_height_mm",
    "height_error_mm",
    "worst_joint_error_deg",
    "voltage_v",
    "current_a",
    "power_w",
    ...jointHeaders,
  ];
  const rows = (state?.samples || []).map((sample) => [
    new Date(sample.capturedAt).toISOString(),
    Math.round(sample.elapsedMs),
    Math.round(sample.expectedTimestampMs),
    Math.round(sample.measuredTimestampMs),
    csvNumber(sample.alignmentMs, 2),
    csvNumber(sample.expectedBody.rollDeg),
    csvNumber(sample.measuredBody.rollDeg),
    csvNumber(sample.bodyError.rollDeg),
    csvNumber(sample.expectedBody.pitchDeg),
    csvNumber(sample.measuredBody.pitchDeg),
    csvNumber(sample.bodyError.pitchDeg),
    csvNumber(sample.expectedBody.yawDeg),
    csvNumber(sample.measuredBody.yawDeg),
    csvNumber(sample.bodyError.yawDeg),
    csvNumber(sample.expectedBody.heightMm, 2),
    csvNumber(sample.measuredBody.heightMm, 2),
    csvNumber(sample.bodyError.heightMm, 2),
    csvNumber(sample.worstJointErrorDeg),
    csvNumber(sample.power?.voltageV),
    csvNumber(sample.power?.currentA),
    csvNumber(sample.power?.powerW),
    ...sample.jointErrorsDeg.map((value) => csvNumber(value)),
  ]);
  return [headers.join(","), ...rows.map((row) => row.join(","))].join("\n");
}
