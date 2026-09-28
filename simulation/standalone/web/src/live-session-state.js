import { LIVE_SERVO_CHANNELS } from "./live-telemetry-state.js";
import { deriveLiveBatteryState } from "./live-battery-state.js";
import { sanitizeDiagnostics } from "./live-diagnostics-state.js";
import { sanitizeLiveControllerTelemetry } from "./live-controller-state.js";

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

export function clearLiveSession(state) {
  if (!state || state.status === "recording") return false;
  state.status = "idle";
  state.startedAt = null;
  state.stoppedAt = null;
  state.samples = [];
  state.lastSourceKey = "";
  return true;
}

export function recordLiveComparisonSample(state, snapshot, capturedAt = Date.now()) {
  if (!state || state.status !== "recording" || !snapshot?.expectedFresh || !snapshot.expected) return false;
  const measured = snapshot.paired ? snapshot.measured : null;
  const sourceKey = `${snapshot.expected.timestampMs}:${measured?.timestampMs ?? "none"}`;
  if (sourceKey === state.lastSourceKey) return false;
  const battery = deriveLiveBatteryState(snapshot.power?.voltageV, undefined, {
    armed: snapshot.diagnostics?.robotState === "armed",
  });
  const sample = {
    capturedAt,
    elapsedMs: Math.max(0, capturedAt - state.startedAt),
    expectedTimestampMs: snapshot.expected.timestampMs,
    expectedDetailsTimestampMs: snapshot.expected.detailsTimestampMs ?? null,
    measuredTimestampMs: measured?.timestampMs ?? null,
    alignmentMs: measured ? snapshot.alignmentMs : null,
    expectedBody: { ...snapshot.expected.body },
    measuredBody: measured ? { ...measured.body } : { rollDeg: null, pitchDeg: null, yawDeg: null, heightMm: null },
    bodyError: measured && snapshot.bodyError ? { ...snapshot.bodyError }
      : { rollDeg: null, pitchDeg: null, yawDeg: null, heightMm: null },
    worstJointErrorDeg: measured ? snapshot.worstJointErrorDeg : null,
    jointErrorsDeg: LIVE_SERVO_CHANNELS.map((channel) => measured ? snapshot.jointErrorsDeg[channel] : null),
    expectedJointAnglesDeg: LIVE_SERVO_CHANNELS.map((channel) => snapshot.expected.servoAngleDeg?.[channel] ?? null),
    expectedModelJointAnglesDeg: Array.isArray(snapshot.expected.modelServoAngleDeg)
      ? LIVE_SERVO_CHANNELS.map((channel) => snapshot.expected.modelServoAngleDeg[channel])
      : null,
    expectedServoPulseUs: LIVE_SERVO_CHANNELS.map((channel) => snapshot.expected.servoPulseUs?.[channel] ?? null),
    expectedServoPhysicalChannels: LIVE_SERVO_CHANNELS.map((channel) => snapshot.expected.servoPhysicalChannel?.[channel] ?? null),
    measuredJointAnglesDeg: Array.isArray(measured?.servoAngleDeg)
      ? LIVE_SERVO_CHANNELS.map((channel) => measured.servoAngleDeg[channel])
      : null,
    expectedFootTargetsMm: Array.isArray(snapshot.expected.footTargetMm)
      ? snapshot.expected.footTargetMm.map((target) => [...target])
      : null,
    power: snapshot.power ? {
      ...snapshot.power,
      cellCount: battery.cellCount,
      packDetected: battery.packDetected,
      averageCellVoltageV: battery.averageCellVoltageV,
      estimatedChargePercent: battery.estimatedChargePercent,
    } : null,
    link: snapshot.link ? { ...snapshot.link } : null,
    diagnostics: sanitizeDiagnostics(snapshot.diagnostics),
    controller: sanitizeLiveControllerTelemetry(snapshot.controller),
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

export function selectLiveSessionPlotSamples(samples, windowSeconds = 30, maximumPoints = 1_200) {
  if (!Array.isArray(samples) || samples.length === 0) return [];
  const seconds = Number(windowSeconds);
  const latestElapsedMs = Number(samples.at(-1)?.elapsedMs);
  const windowed = Number.isFinite(seconds) && seconds > 0 && Number.isFinite(latestElapsedMs)
    ? samples.filter((sample) => Number(sample?.elapsedMs) >= latestElapsedMs - seconds * 1_000)
    : samples.slice();
  const limit = Math.max(2, Math.floor(Number(maximumPoints) || 1_200));
  if (windowed.length <= limit) return windowed;
  const selected = [];
  const lastIndex = windowed.length - 1;
  for (let index = 0; index < limit; index += 1) {
    selected.push(windowed[Math.round((index * lastIndex) / (limit - 1))]);
  }
  return selected;
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
      expectedModelJointAnglesDeg: sample.expectedModelJointAnglesDeg ? [...sample.expectedModelJointAnglesDeg] : null,
      expectedServoPulseUs: [...sample.expectedServoPulseUs],
      expectedServoPhysicalChannels: [...sample.expectedServoPhysicalChannels],
      measuredJointAnglesDeg: sample.measuredJointAnglesDeg ? [...sample.measuredJointAnglesDeg] : null,
      expectedFootTargetsMm: sample.expectedFootTargetsMm?.map((target) => [...target]) || null,
      power: sample.power ? { ...sample.power } : null,
      link: sample.link ? { ...sample.link } : null,
      diagnostics: sanitizeDiagnostics(sample.diagnostics),
      controller: sanitizeLiveControllerTelemetry(sample.controller),
    })),
  };
  archive.unshift(entry);
  archive.splice(Math.max(1, Math.floor(maximumEntries)));
  return entry;
}

const finiteBody = (body, allowMissing = false) => body &&
  ["rollDeg", "pitchDeg", "yawDeg", "heightMm"].every((key) =>
    Number.isFinite(body[key]) || (allowMissing && body[key] === null));

export function sanitizeArchivedLiveSession(candidate) {
  if (!candidate || typeof candidate !== "object" || !String(candidate.id || "").trim()) return null;
  if (!Number.isFinite(candidate.startedAt) || !Number.isFinite(candidate.stoppedAt) || candidate.stoppedAt < candidate.startedAt) return null;
  if (!Array.isArray(candidate.samples) || candidate.samples.length === 0 || candidate.samples.length > LIVE_SESSION_MAX_SAMPLES) return null;
  const samples = [];
  for (const sample of candidate.samples) {
    if (!sample || !Number.isFinite(sample.capturedAt) || !Number.isFinite(sample.elapsedMs) ||
      !finiteBody(sample.expectedBody) || !finiteBody(sample.measuredBody, true) || !finiteBody(sample.bodyError, true) ||
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
      expectedModelJointAnglesDeg: Array.isArray(sample.expectedModelJointAnglesDeg) &&
          sample.expectedModelJointAnglesDeg.length === LIVE_SERVO_CHANNELS.length &&
          sample.expectedModelJointAnglesDeg.every(Number.isFinite)
        ? [...sample.expectedModelJointAnglesDeg]
        : null,
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
      link: sample.link && typeof sample.link === "object" ? { ...sample.link } : null,
      diagnostics: sanitizeDiagnostics(sample.diagnostics),
      controller: sanitizeLiveControllerTelemetry(sample.controller),
      expectedDetailsTimestampMs: Number.isFinite(sample.expectedDetailsTimestampMs) ? sample.expectedDetailsTimestampMs : null,
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
  const voltage = finite((sample) => deriveLiveBatteryState(sample.power?.voltageV, undefined, {
    armed: sample.diagnostics?.robotState === "armed",
  }).packDetected ? sample.power.voltageV : null);
  const current = finite((sample) => sample.power?.currentA);
  const absoluteError = (key) => average(finite((sample) =>
    Number.isFinite(sample.bodyError?.[key]) ? Math.abs(sample.bodyError[key]) : null));
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
    meanAbsPitchErrorDeg: absoluteError("pitchDeg"),
    meanAbsRollErrorDeg: absoluteError("rollDeg"),
    meanAbsYawErrorDeg: absoluteError("yawDeg"),
    meanAbsHeightErrorMm: absoluteError("heightMm"),
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
    "average_cell_voltage_v",
    "estimated_charge_percent",
    "current_a",
    "power_w",
    "engineering_packet_rate_hz",
    "engineering_packet_age_ms",
    "telemetry_payload_bytes",
    "dropped_packets_total",
    "rejected_packets_total",
    "esp32_loop_hz",
    "expected_details_timestamp_ms",
    "robot_uptime_ms", "robot_boot_id", "esp32_reset_reason", "maximum_loop_gap_ms", "telemetry_tx_skipped", "body_mode",
    "prior_reset_stage", "last_slow_stage", "last_slow_stage_us", "slow_stage_count",
    "max_crsf_stage_us", "max_imu_stage_us", "max_live_stage_us", "max_control_stage_us", "max_servo_write_us", "imu_i2c_errors",
    "imu_last_request_bytes", "imu_last_request_us", "imu_last_failed_request_bytes", "imu_last_failed_request_us", "imu_max_request_us",
    "last_slow_stage_at_ms", "max_crsf_stage_at_ms", "max_imu_stage_at_ms", "imu_last_error_at_ms",
    "imu_sample_age_ms", "imu_consecutive_errors", "crsf_budget_hits", "crsf_last_budget_hit_at_ms",
    "crsf_pending_bytes", "crsf_max_pending_bytes", "crsf_last_pass_bytes",
    "servo_outputs_enabled", "radio_control_enabled", "awaiting_stick_center",
    "crsf_accepted_frames", "crsf_crc_errors", "crsf_uart_overflows", "crsf_frame_age_ms",
    "crsf_packet_rate_hz", "crsf_failsafe", "crsf_link_quality_percent",
    ...Array.from({ length: 16 }, (_, index) => `crsf_ch${index + 1}_us`),
    "fl_foot_target_z_mm",
    "fr_foot_target_z_mm",
    "bl_foot_target_z_mm",
    "br_foot_target_z_mm",
    ...jointCommandHeaders,
    ...LIVE_SERVO_CHANNELS.map((channel) => `ch${channel}_model_command_deg`),
    ...servoPulseHeaders,
    ...servoOutputHeaders,
    ...jointHeaders,
  ];
  const rows = (state?.samples || []).map((sample) => [
    new Date(sample.capturedAt).toISOString(),
    Math.round(sample.elapsedMs),
    Math.round(sample.expectedTimestampMs),
    csvNumber(sample.measuredTimestampMs, 0),
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
    csvNumber(sample.power?.averageCellVoltageV),
    csvNumber(sample.power?.estimatedChargePercent, 1),
    csvNumber(sample.power?.currentA),
    csvNumber(sample.power?.powerW),
    csvNumber(sample.link?.packetRateHz),
    csvNumber(sample.link?.packetAgeMs, 1),
    csvNumber(sample.link?.packetBytes, 0),
    csvNumber(sample.link?.droppedPackets, 0),
    csvNumber(sample.link?.rejectedPackets, 0),
    csvNumber(sample.link?.esp32LoopHz),
    csvNumber(sample.expectedDetailsTimestampMs, 0),
    ...["uptimeMs", "bootId", "resetReason", "maximumLoopGapMs", "telemetryTxSkipped", "bodyMode"]
      .map((key) => csvNumber(sample.diagnostics?.[key], 0)),
    ...["priorResetStage", "lastSlowStage", "lastSlowStageUs", "slowStageCount",
      "maxCrsfStageUs", "maxImuStageUs", "maxLiveStageUs", "maxControlStageUs", "maxServoWriteUs", "imuI2cErrors"]
      .map((key) => csvNumber(sample.diagnostics?.[key], 0)),
    ...["imuLastRequestBytes", "imuLastRequestUs", "imuLastFailedRequestBytes", "imuLastFailedRequestUs", "imuMaxRequestUs"]
      .map((key) => csvNumber(sample.diagnostics?.[key], 0)),
    ...["lastSlowStageAtMs", "maxCrsfStageAtMs", "maxImuStageAtMs", "imuLastErrorAtMs",
      "imuSampleAgeMs", "imuConsecutiveErrors", "crsfBudgetHits", "crsfLastBudgetHitAtMs",
      "crsfPendingBytes", "crsfMaxPendingBytes", "crsfLastPassBytes"]
      .map((key) => csvNumber(sample.diagnostics?.[key], 0)),
    ...["outputsEnabled", "radioControlEnabled", "motionInputAwaitingCenter"]
      .map((key) => typeof sample.diagnostics?.[key] === "boolean" ? Number(sample.diagnostics[key]) : ""),
    ...["crsfAcceptedFrames", "crsfCrcErrors", "crsfUartOverflows", "commandLatencyMs"]
      .map((key) => csvNumber(sample.diagnostics?.[key], 0)),
    csvNumber(sample.controller?.packetRateHz),
    typeof sample.controller?.failsafe === "boolean" ? Number(sample.controller.failsafe) : "",
    csvNumber(sample.controller?.linkQualityPercent),
    ...Array.from({ length: 16 }, (_, index) => csvNumber(sample.controller?.channelsUs?.[index], 0)),
    ...Array.from({ length: 4 }, (_, leg) => csvNumber(sample.expectedFootTargetsMm?.[leg]?.[2])),
    ...sample.expectedJointAnglesDeg.map((value) => csvNumber(value)),
    ...LIVE_SERVO_CHANNELS.map((_, index) => csvNumber(sample.expectedModelJointAnglesDeg?.[index])),
    ...sample.expectedServoPulseUs.map((value) => csvNumber(value, 0)),
    ...sample.expectedServoPhysicalChannels.map((value) => Number.isInteger(value) ? value : ""),
    ...sample.jointErrorsDeg.map((value) => csvNumber(value)),
  ]);
  return [headers.join(","), ...rows.map((row) => row.join(","))].join("\n");
}

function collectLiveBlackboxEvents(samples) {
  const events = [];
  const seen = new Set();
  for (const sample of samples) {
    for (const record of sample.diagnostics?.blackboxEvents || []) {
      const key = record.join(":");
      if (seen.has(key)) continue;
      seen.add(key);
      const [sequence, bootSequence, atMs, kind, stage, value, aux] = record;
      events.push({ sequence, bootSequence, atMs, kind, stage, value, aux,
        firstObservedAt: sample.capturedAt });
    }
  }
  return events;
}

export function liveSessionJson(session, exportedAt = Date.now()) {
  if (!session || !Array.isArray(session.samples) || session.samples.length === 0) return "";
  const identifier = String(session.id || `live-${session.startedAt || exportedAt}`).slice(0, 120);
  return JSON.stringify({
    schema: "domino-live-engineering-session",
    version: 1,
    exportedAt,
    robotId: "domino-esp32-quadruped",
    session: {
      id: identifier,
      startedAt: Number.isFinite(session.startedAt) ? session.startedAt : null,
      stoppedAt: Number.isFinite(session.stoppedAt) ? session.stoppedAt : null,
      samples: session.samples,
    },
    analysis: analyzeLiveSession(session),
    blackbox: {
      format: 1,
      retention: "ESP32 RTC memory; survives many warm resets, not guaranteed across power loss",
      events: collectLiveBlackboxEvents(session.samples),
    },
    signalSemantics: {
      expected: "Robot-reported commanded state after firmware processing",
      measured: "Independent physical feedback only; unavailable values remain null",
      voltageV: "Raw PCB voltage-divider reading; USB power may leave a residual reading below the detectable 4S pack range. Pack charge and minimum pack voltage exclude that reading while disarmed.",
      servoPulseUs: "Commanded calibrated PWM pulse, not encoder feedback",
      expectedModelJointAnglesDeg: "Limited servo commands in the fixed CAD reference after removing the robot's active trim and direction; not measured joint positions; null on older firmware",
      servoPhysicalChannel: "Mapped PCA9685 output for each logical joint",
      expectedDetailsTimestampMs: "Source time of retained pulse and foot-target details; null means unknown in older exports",
      resetReason: "ESP-IDF esp_reset_reason enum for the current boot; 9 = brownout, 5/6/7 = watchdog",
      loopStage: "0 = none/unknown, 1 = CRSF, 2 = IMU I2C, 3 = LIVE transport, 4 = control update, 5 = servo I2C write; priorResetStage is retained best effort",
      stageTiming: "Maxima and lastSlowStageUs are microseconds since this boot; a slow stage is at least 100000 microseconds",
      busRecovery: "Firmware 0.8.7 caps CRSF work at 384 bytes or 1000 us per pass; firmware 0.8.8 removes the 1000 us limit while retaining the 384-byte cap. Budget hits mean queued RX work remained. IMU sample age and event timestamps use ESP32 milliseconds since boot; zero event timestamps mean no event yet",
      blackbox: "Firmware 0.8.9+ records the latest 16 RTC events. kind: 1 boot (value=reset reason, stage=prior active stage), 2 slow stage (value=duration us), 3 IMU read error (stage=1 register select or 2 data read, value=duration us; in 0.8.10+ aux=bytes returned for stage 2, otherwise aux=Wire timeout ms), 4 CRSF UART overflow (value=total, aux=new errors), 5 failsafe change (value=1 active or 0 clear, aux=CRSF frame age ms). atMs is ESP32 uptime for the event; firstObservedAt is Studio wall-clock time.",
      imuRequest: "Firmware 0.8.10 records the last MPU6050 Wire.requestFrom result (bytes returned out of 14 and call duration in microseconds), the last failed result, and the maximum request duration this boot.",
      bodyMode: "0 = stow, 1 = stand, 2 = tilt, 3 = balance, 4 = trot, 5 = careful walk",
      measuredYaw: "Relative integrated gyro heading since boot; not an absolute heading or a translation measurement",
    },
  }, null, 2);
}

export function parseLiveSessionJson(contents) {
  let candidate;
  try {
    candidate = JSON.parse(String(contents));
  } catch {
    throw new Error("Session file is not valid JSON.");
  }
  if (candidate?.schema !== "domino-live-engineering-session") {
    throw new Error("This is not a Domino LIVE engineering session.");
  }
  if (candidate.version !== 1) {
    throw new Error("Unsupported engineering session version.");
  }
  if (candidate.robotId !== "domino-esp32-quadruped") {
    throw new Error("This engineering session targets a different robot.");
  }
  const session = sanitizeArchivedLiveSession(candidate.session);
  if (!session) {
    throw new Error("Engineering session data is incomplete, malformed, or exceeds the recording limit.");
  }
  return session;
}
