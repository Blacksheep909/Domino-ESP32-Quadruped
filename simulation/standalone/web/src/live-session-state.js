import { LIVE_SERVO_CHANNELS } from "./live-telemetry-state.js";

export const LIVE_SESSION_MAX_SAMPLES = 18_000;
export const LIVE_SESSION_MAX_ARCHIVE_ENTRIES = 20;

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
    expectedJointAnglesDeg: LIVE_SERVO_CHANNELS.map((channel) => snapshot.expected.servoAngleDeg?.[channel] ?? null),
    expectedServoPulseUs: LIVE_SERVO_CHANNELS.map((channel) => snapshot.expected.servoPulseUs?.[channel] ?? null),
    expectedServoPhysicalChannels: LIVE_SERVO_CHANNELS.map((channel) => snapshot.expected.servoPhysicalChannel?.[channel] ?? null),
    measuredJointAnglesDeg: Array.isArray(snapshot.measured.servoAngleDeg)
      ? LIVE_SERVO_CHANNELS.map((channel) => snapshot.measured.servoAngleDeg[channel])
      : null,
    expectedFootTargetsMm: Array.isArray(snapshot.expected.footTargetMm)
      ? snapshot.expected.footTargetMm.map((target) => [...target])
      : null,
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
      expectedJointAnglesDeg: [...sample.expectedJointAnglesDeg],
      expectedServoPulseUs: [...sample.expectedServoPulseUs],
      expectedServoPhysicalChannels: [...sample.expectedServoPhysicalChannels],
      measuredJointAnglesDeg: sample.measuredJointAnglesDeg ? [...sample.measuredJointAnglesDeg] : null,
      expectedFootTargetsMm: sample.expectedFootTargetsMm?.map((target) => [...target]) || null,
      power: sample.power ? { ...sample.power } : null,
    })),
  };
  archive.unshift(entry);
  archive.splice(Math.max(1, Math.floor(maximumEntries)));
  return entry;
}

const finiteBody = (body) => body && ["rollDeg", "pitchDeg", "yawDeg", "heightMm"].every((key) => Number.isFinite(body[key]));

export function sanitizeArchivedLiveSession(candidate) {
  if (!candidate || typeof candidate !== "object" || !String(candidate.id || "").trim()) return null;
  if (!Number.isFinite(candidate.startedAt) || !Number.isFinite(candidate.stoppedAt) || candidate.stoppedAt < candidate.startedAt) return null;
  if (!Array.isArray(candidate.samples) || candidate.samples.length === 0 || candidate.samples.length > LIVE_SESSION_MAX_SAMPLES) return null;
  const samples = [];
  for (const sample of candidate.samples) {
    if (!sample || !Number.isFinite(sample.capturedAt) || !Number.isFinite(sample.elapsedMs) ||
      !finiteBody(sample.expectedBody) || !finiteBody(sample.measuredBody) || !finiteBody(sample.bodyError) ||
      !Array.isArray(sample.jointErrorsDeg) || sample.jointErrorsDeg.length !== LIVE_SERVO_CHANNELS.length) return null;
    samples.push({
      ...sample,
      expectedBody: { ...sample.expectedBody },
      measuredBody: { ...sample.measuredBody },
      bodyError: { ...sample.bodyError },
      jointErrorsDeg: sample.jointErrorsDeg.map((value) => Number.isFinite(value) ? value : null),
      expectedJointAnglesDeg: Array.isArray(sample.expectedJointAnglesDeg)
        ? sample.expectedJointAnglesDeg.map((value) => Number.isFinite(value) ? value : null)
        : Array(LIVE_SERVO_CHANNELS.length).fill(null),
      expectedServoPulseUs: Array.isArray(sample.expectedServoPulseUs)
        ? sample.expectedServoPulseUs.slice(0, LIVE_SERVO_CHANNELS.length).map((value) => Number.isFinite(value) ? value : null)
        : Array(LIVE_SERVO_CHANNELS.length).fill(null),
      expectedServoPhysicalChannels: Array.isArray(sample.expectedServoPhysicalChannels)
        ? sample.expectedServoPhysicalChannels.slice(0, LIVE_SERVO_CHANNELS.length).map((value) => Number.isInteger(value) && value >= 0 && value < 16 ? value : null)
        : Array(LIVE_SERVO_CHANNELS.length).fill(null),
      measuredJointAnglesDeg: Array.isArray(sample.measuredJointAnglesDeg)
        ? sample.measuredJointAnglesDeg.map((value) => Number.isFinite(value) ? value : null)
        : null,
      expectedFootTargetsMm: Array.isArray(sample.expectedFootTargetsMm)
        ? sample.expectedFootTargetsMm.map((target) => Array.isArray(target) ? target.map((value) => Number.isFinite(value) ? value : null) : [null, null, null])
        : null,
      power: sample.power && typeof sample.power === "object" ? { ...sample.power } : null,
    });
  }
  return { id: String(candidate.id).slice(0, 120), startedAt: candidate.startedAt, stoppedAt: candidate.stoppedAt, samples };
}

export function mergeArchivedLiveSessions(archive, candidates, maximumEntries = LIVE_SESSION_MAX_ARCHIVE_ENTRIES) {
  if (!Array.isArray(archive) || !Array.isArray(candidates)) return 0;
  const byId = new Map(archive.map((entry) => [entry.id, entry]));
  let accepted = 0;
  for (const candidate of candidates) {
    const session = sanitizeArchivedLiveSession(candidate);
    if (!session) continue;
    byId.set(session.id, session);
    accepted += 1;
  }
  archive.splice(0, archive.length, ...[...byId.values()]
    .sort((left, right) => right.stoppedAt - left.stoppedAt)
    .slice(0, Math.max(1, Math.floor(maximumEntries))));
  return accepted;
}

const average = (values) => values.length ? values.reduce((total, value) => total + value, 0) / values.length : null;
const percentile = (values, fraction) => {
  if (!values.length) return null;
  const sorted = [...values].sort((left, right) => left - right);
  return sorted[Math.min(sorted.length - 1, Math.max(0, Math.ceil(sorted.length * fraction) - 1))];
};

export function analyzeLiveSession(session) {
  const samples = Array.isArray(session?.samples) ? session.samples : [];
  const finite = (selector) => samples.map(selector).filter(Number.isFinite);
  const worstJoint = finite((sample) => sample.worstJointErrorDeg);
  const power = finite((sample) => sample.power?.powerW);
  const voltage = finite((sample) => sample.power?.voltageV);
  const current = finite((sample) => sample.power?.currentA);
  let energyWh = 0;
  for (let index = 1; index < samples.length; index += 1) {
    const prior = samples[index - 1];
    const currentSample = samples[index];
    const deltaHours = Math.max(0, currentSample.elapsedMs - prior.elapsedMs) / 3_600_000;
    if (Number.isFinite(prior.power?.powerW) && Number.isFinite(currentSample.power?.powerW)) {
      energyWh += ((prior.power.powerW + currentSample.power.powerW) / 2) * deltaHours;
    }
  }
  return {
    sampleCount: samples.length,
    durationMs: Number.isFinite(session?.startedAt) && Number.isFinite(session?.stoppedAt) ? Math.max(0, session.stoppedAt - session.startedAt) : 0,
    meanAbsPitchErrorDeg: average(finite((sample) => Math.abs(sample.bodyError?.pitchDeg))),
    meanAbsRollErrorDeg: average(finite((sample) => Math.abs(sample.bodyError?.rollDeg))),
    meanAbsYawErrorDeg: average(finite((sample) => Math.abs(sample.bodyError?.yawDeg))),
    meanAbsHeightErrorMm: average(finite((sample) => Math.abs(sample.bodyError?.heightMm))),
    peakJointErrorDeg: worstJoint.length ? Math.max(...worstJoint) : null,
    p95JointErrorDeg: percentile(worstJoint, 0.95),
    averagePowerW: average(power),
    energyWh: power.length > 1 ? energyWh : null,
    minimumVoltageV: voltage.length ? Math.min(...voltage) : null,
    peakCurrentA: current.length ? Math.max(...current) : null,
  };
}

export function compareLiveSessions(baseline, candidate) {
  const baselineMetrics = analyzeLiveSession(baseline);
  const candidateMetrics = analyzeLiveSession(candidate);
  const delta = {};
  for (const key of ["meanAbsPitchErrorDeg", "meanAbsRollErrorDeg", "meanAbsYawErrorDeg", "meanAbsHeightErrorMm", "peakJointErrorDeg", "p95JointErrorDeg", "averagePowerW", "energyWh", "minimumVoltageV", "peakCurrentA"]) {
    delta[key] = Number.isFinite(baselineMetrics[key]) && Number.isFinite(candidateMetrics[key])
      ? candidateMetrics[key] - baselineMetrics[key]
      : null;
  }
  return { baseline: baselineMetrics, candidate: candidateMetrics, delta };
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
  const jointCommandHeaders = LIVE_SERVO_CHANNELS.map((channel) => `joint_${channel}_expected_deg`);
  const servoPulseHeaders = LIVE_SERVO_CHANNELS.map((channel) => `joint_${channel}_command_pulse_us`);
  const servoOutputHeaders = LIVE_SERVO_CHANNELS.map((channel) => `joint_${channel}_pca_output`);
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
    "fl_foot_target_z_mm",
    "fr_foot_target_z_mm",
    "bl_foot_target_z_mm",
    "br_foot_target_z_mm",
    ...jointCommandHeaders,
    ...servoPulseHeaders,
    ...servoOutputHeaders,
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
    ...Array.from({ length: 4 }, (_, leg) => csvNumber(sample.expectedFootTargetsMm?.[leg]?.[2])),
    ...sample.expectedJointAnglesDeg.map((value) => csvNumber(value)),
    ...sample.expectedServoPulseUs.map((value) => csvNumber(value, 0)),
    ...sample.expectedServoPhysicalChannels.map((value) => Number.isInteger(value) ? value : ""),
    ...sample.jointErrorsDeg.map((value) => csvNumber(value)),
  ]);
  return [headers.join(","), ...rows.map((row) => row.join(","))].join("\n");
}
