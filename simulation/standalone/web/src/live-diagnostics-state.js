import { deriveLiveBatteryState } from "./live-battery-state.js";

export const LIVE_DIAGNOSTIC_MAX_EVENTS = 250;
export const LIVE_DIAGNOSTIC_RATE_WINDOW_MS = 5_000;

const finite = (value) => value !== null && value !== undefined && Number.isFinite(Number(value))
  ? Number(value)
  : null;
const bool = (value) => typeof value === "boolean" ? value : null;

function addEvent(state, severity, message, timestampMs = Date.now(), source = "live") {
  const event = {
    id: `${timestampMs}-${state.nextEventId++}`,
    timestampMs,
    severity: ["info", "warning", "fault"].includes(severity) ? severity : "info",
    source: String(source).slice(0, 32),
    message: String(message).slice(0, 500),
  };
  state.events.unshift(event);
  state.events.splice(state.maximumEvents);
  return event;
}

function conditionEvent(state, key, active, severity, message, timestampMs) {
  const previous = state.conditions[key] === true;
  state.conditions[key] = active === true;
  if (active && !previous) addEvent(state, severity, message, timestampMs);
  if (!active && previous) addEvent(state, "info", `${message} cleared`, timestampMs);
}

export function sanitizeDiagnostics(diagnostics) {
  if (!diagnostics || typeof diagnostics !== "object") return null;
  return {
    controllerHz: finite(diagnostics.controllerHz),
    commandHz: finite(diagnostics.commandHz),
    transmitHz: finite(diagnostics.transmitHz),
    acknowledgementHz: finite(diagnostics.acknowledgementHz),
    esp32LoopHz: finite(diagnostics.esp32LoopHz),
    commandLatencyMs: finite(diagnostics.commandLatencyMs),
    uptimeMs: finite(diagnostics.uptimeMs),
    bootId: finite(diagnostics.bootId),
    resetReason: finite(diagnostics.resetReason),
    maximumLoopGapMs: finite(diagnostics.maximumLoopGapMs),
    priorResetStage: finite(diagnostics.priorResetStage),
    lastSlowStage: finite(diagnostics.lastSlowStage),
    lastSlowStageUs: finite(diagnostics.lastSlowStageUs),
    lastSlowStageAtMs: finite(diagnostics.lastSlowStageAtMs),
    slowStageCount: finite(diagnostics.slowStageCount),
    maxCrsfStageUs: finite(diagnostics.maxCrsfStageUs),
    maxCrsfStageAtMs: finite(diagnostics.maxCrsfStageAtMs),
    maxImuStageUs: finite(diagnostics.maxImuStageUs),
    maxImuStageAtMs: finite(diagnostics.maxImuStageAtMs),
    maxLiveStageUs: finite(diagnostics.maxLiveStageUs),
    maxControlStageUs: finite(diagnostics.maxControlStageUs),
    maxServoWriteUs: finite(diagnostics.maxServoWriteUs),
    blackboxEvents: Array.isArray(diagnostics.blackboxEvents)
      ? diagnostics.blackboxEvents.slice(-16).filter((event) =>
          Array.isArray(event) && event.length === 7 &&
          event.every((value) => Number.isInteger(value) && value >= 0))
        .map((event) => [...event])
      : [],
    imuI2cErrors: finite(diagnostics.imuI2cErrors),
    imuLastErrorAtMs: finite(diagnostics.imuLastErrorAtMs),
    imuLastRequestBytes: finite(diagnostics.imuLastRequestBytes),
    imuLastRequestUs: finite(diagnostics.imuLastRequestUs),
    imuLastFailedRequestBytes: finite(diagnostics.imuLastFailedRequestBytes),
    imuLastFailedRequestUs: finite(diagnostics.imuLastFailedRequestUs),
    imuMaxRequestUs: finite(diagnostics.imuMaxRequestUs),
    imuConsecutiveErrors: finite(diagnostics.imuConsecutiveErrors),
    imuSampleAgeMs: finite(diagnostics.imuSampleAgeMs),
    telemetryTxSkipped: finite(diagnostics.telemetryTxSkipped),
    bodyMode: finite(diagnostics.bodyMode),
    outputsEnabled: bool(diagnostics.outputsEnabled),
    radioControlEnabled: bool(diagnostics.radioControlEnabled),
    motionInputAwaitingCenter: bool(diagnostics.motionInputAwaitingCenter),
    crsfAcceptedFrames: finite(diagnostics.crsfAcceptedFrames),
    crsfCrcErrors: finite(diagnostics.crsfCrcErrors),
    crsfUartOverflows: finite(diagnostics.crsfUartOverflows),
    crsfBudgetHits: finite(diagnostics.crsfBudgetHits),
    crsfLastBudgetHitAtMs: finite(diagnostics.crsfLastBudgetHitAtMs),
    crsfPendingBytes: finite(diagnostics.crsfPendingBytes),
    crsfMaxPendingBytes: finite(diagnostics.crsfMaxPendingBytes),
    crsfLastPassBytes: finite(diagnostics.crsfLastPassBytes),
    manualOverrideActive: bool(diagnostics.manualOverrideActive),
    manualFrameAgeMs: finite(diagnostics.manualFrameAgeMs),
    faultReason: typeof diagnostics.faultReason === "string" ? diagnostics.faultReason.slice(0, 120) : null,
    robotState: typeof diagnostics.robotState === "string"
      ? diagnostics.robotState.slice(0, 32).toLowerCase()
      : null,
    driveLinkAlive: bool(diagnostics.driveLinkAlive),
    drivePacketAgeMs: finite(diagnostics.drivePacketAgeMs),
    driveRssiDbm: finite(diagnostics.driveRssiDbm),
    driveLinkQualityPercent: finite(diagnostics.driveLinkQualityPercent),
    gaitTargetValid: bool(diagnostics.gaitTargetValid),
    ikValid: bool(diagnostics.ikValid),
    jointLimitClips: finite(diagnostics.jointLimitClips ?? diagnostics.servoLimitClipCount),
    servoOutputChannels: finite(diagnostics.servoOutputChannels),
    imuOnline: bool(diagnostics.imuOnline),
    imuAxG: finite(diagnostics.imuAxG),
    imuAyG: finite(diagnostics.imuAyG),
    imuAzG: finite(diagnostics.imuAzG),
  };
}

export function createLiveDiagnosticsState(maximumEvents = LIVE_DIAGNOSTIC_MAX_EVENTS) {
  return {
    maximumEvents: Math.max(10, Math.floor(Number(maximumEvents) || LIVE_DIAGNOSTIC_MAX_EVENTS)),
    nextEventId: 1,
    acceptedPackets: 0,
    rejectedPackets: 0,
    lastRejectedEventAt: 0,
    droppedPackets: 0,
    staleTransitions: 0,
    lastSequence: -1,
    packetTimes: [],
    lastPacketAt: 0,
    lastPacket: null,
    telemetry: null,
    events: [],
    conditions: {},
    linkWasFresh: false,
  };
}

export function observeLiveDiagnosticPacket(state, packet, accepted, receivedAt = Date.now()) {
  if (!state || !packet || packet.type !== "live-telemetry") return false;
  if (!accepted) {
    state.rejectedPackets += 1;
    if (!state.lastRejectedEventAt || receivedAt - state.lastRejectedEventAt >= 1_000) {
      addEvent(state, "warning", "Rejected one or more malformed, unbound, or out-of-order telemetry packets.", receivedAt, "transport");
      state.lastRejectedEventAt = receivedAt;
    }
    return false;
  }
  const sequence = Number(packet.sequence);
  if (state.lastSequence >= 0 && sequence > state.lastSequence + 1) {
    const dropped = sequence - state.lastSequence - 1;
    state.droppedPackets += dropped;
    addEvent(state, "warning", `Detected ${dropped} missing telemetry packet${dropped === 1 ? "" : "s"}.`, receivedAt, "transport");
  }
  const previous = state.telemetry;
  state.lastSequence = sequence;
  state.acceptedPackets += 1;
  state.lastPacketAt = receivedAt;
  state.packetTimes.push(receivedAt);
  state.packetTimes = state.packetTimes.filter((time) => receivedAt - time <= LIVE_DIAGNOSTIC_RATE_WINDOW_MS);
  state.telemetry = sanitizeDiagnostics(packet.diagnostics);
  const current = state.telemetry;
  if (previous && current && previous.bootId !== null && current.bootId !== null &&
      current.bootId !== previous.bootId) {
    addEvent(state, "fault", `ESP32 restarted: reset reason ${current.resetReason ?? "unknown"}; prior active stage ${loopStageName(current.priorResetStage)}.`, receivedAt, "firmware");
  } else if (previous && current && current.maximumLoopGapMs !== null &&
      current.maximumLoopGapMs > (previous.maximumLoopGapMs ?? 0) && current.maximumLoopGapMs >= 250) {
    addEvent(state, "warning", `ESP32 loop gap reached ${current.maximumLoopGapMs} ms; longest CRSF/IMU/LIVE/control/servo stages are ${[
      current.maxCrsfStageUs, current.maxImuStageUs, current.maxLiveStageUs,
      current.maxControlStageUs, current.maxServoWriteUs,
    ].map((value) => value === null ? "?" : Math.round(value / 1000)).join("/")} ms.`, receivedAt, "firmware");
  }
  if (previous && current && current.imuI2cErrors !== null &&
      current.imuI2cErrors > (previous.imuI2cErrors ?? 0)) {
    addEvent(state, "warning", `IMU I²C read errors increased to ${current.imuI2cErrors}.`, receivedAt, "firmware");
  }
  if (previous && current && current.crsfBudgetHits !== null &&
      current.crsfBudgetHits > (previous.crsfBudgetHits ?? 0)) {
    addEvent(state, "warning", `CRSF receive reached its work limit; ${current.crsfPendingBytes ?? "?"} bytes remain queued.`, receivedAt, "firmware");
  }
  conditionEvent(state, "imu-stale", current?.imuOnline === false && current?.imuSampleAgeMs !== null,
    "warning", "IMU sample became stale.", receivedAt);
  state.lastPacket = {
    type: packet.type,
    sequence,
    receivedAt,
    payloadBytes: JSON.stringify(packet).length,
    expectedTimestampMs: finite(packet.expected?.timestampMs),
    measuredTimestampMs: finite(packet.measured?.timestampMs),
    voltageV: finite(packet.power?.voltageV),
    currentA: finite(packet.power?.currentA),
    diagnostics: state.telemetry,
  };
  if (state.acceptedPackets === 1) addEvent(state, "info", "Engineering telemetry stream detected.", receivedAt, "transport");
  conditionEvent(state, "ik", state.telemetry?.ikValid === false, "fault", "IK reported an unreachable target.", receivedAt);
  conditionEvent(
    state,
    "limits",
    Number(state.telemetry?.jointLimitClips) > 0,
    "warning",
    "One or more joint targets were clipped by mechanical limits.",
    receivedAt,
  );
  conditionEvent(
    state,
    "servo-output",
    state.telemetry?.servoOutputChannels !== null && state.telemetry.servoOutputChannels < 12,
    "fault",
    "Fewer than 12 servo output channels are active.",
    receivedAt,
  );
  conditionEvent(
    state,
    "low-voltage",
    deriveLiveBatteryState(state.lastPacket.voltageV, undefined, {
      armed: state.telemetry?.robotState === "armed",
    }).packDetected && state.lastPacket.voltageV < 14,
    "warning",
    "Battery voltage is below the 14.0 V diagnostic threshold.",
    receivedAt,
  );
  return true;
}

export function loopStageName(value) {
  return ["none/unknown", "CRSF receive", "IMU I²C", "LIVE transport", "control update", "servo I²C write"][value] || "unknown";
}

function stage(id, label, status, detail, rateHz = null) {
  return { id, label, status, detail, rateHz };
}

function booleanStage(id, label, value, okDetail, faultDetail) {
  return value === true
    ? stage(id, label, "ok", okDetail)
    : value === false
      ? stage(id, label, "fault", faultDetail)
      : stage(id, label, "unavailable", "Not reported by the robot adapter");
}

export function liveDiagnosticsSnapshot(state, liveSnapshot, now = Date.now()) {
  const packetAgeMs = state?.lastPacketAt > 0 ? Math.max(0, now - state.lastPacketAt) : null;
  const linkFresh = packetAgeMs !== null && packetAgeMs <= 1_000;
  if (state && !linkFresh && state.linkWasFresh) {
    state.staleTransitions += 1;
    addEvent(state, "fault", "Engineering telemetry stream became stale.", now, "transport");
  } else if (state && linkFresh && !state.linkWasFresh && state.staleTransitions > 0) {
    addEvent(state, "info", "Engineering telemetry stream recovered.", now, "transport");
  }
  if (state) state.linkWasFresh = linkFresh;
  const times = state?.packetTimes?.filter((time) => now - time <= LIVE_DIAGNOSTIC_RATE_WINDOW_MS) || [];
  const packetRateHz = times.length > 1
    ? ((times.length - 1) * 1_000) / Math.max(1, times.at(-1) - times[0])
    : 0;
  const telemetry = state?.telemetry;
  const stages = [
    stage(
      "engineering",
      "Engineering packet",
      linkFresh ? "ok" : state?.acceptedPackets ? "fault" : "unavailable",
      linkFresh ? `Sequence ${state.lastSequence}` : state?.acceptedPackets ? "Packet stream is stale" : "No packet received",
      packetRateHz,
    ),
    stage(
      "command",
      "Expected command",
      liveSnapshot?.expectedFresh ? "ok" : state?.acceptedPackets ? "fault" : "unavailable",
      liveSnapshot?.expectedFresh ? "Command pose is current" : "Expected pose is unavailable",
      telemetry?.commandHz,
    ),
    stage(
      "measured",
      "Measured telemetry",
      liveSnapshot?.measuredFresh ? "ok" : state?.acceptedPackets ? "fault" : "unavailable",
      liveSnapshot?.measuredFresh ? "Physical pose is current" : "Measured pose is unavailable",
      packetRateHz,
    ),
    stage(
      "ack",
      "ESP32 acknowledgement",
      telemetry?.acknowledgementHz === null || telemetry?.acknowledgementHz === undefined
        ? "unavailable"
        : telemetry.acknowledgementHz >= 20 ? "ok" : "warning",
      telemetry?.acknowledgementHz === null || telemetry?.acknowledgementHz === undefined
        ? "Not reported by the robot adapter"
        : `${telemetry.acknowledgementHz.toFixed(1)} Hz`,
      telemetry?.acknowledgementHz,
    ),
    booleanStage("gait", "Gait target", telemetry?.gaitTargetValid, "Target generated", "Gait target generation failed"),
    booleanStage("ik", "IK solve", telemetry?.ikValid, "All leg targets reachable", "Unreachable target reported"),
    stage(
      "limits",
      "Joint limit check",
      telemetry?.jointLimitClips === null || telemetry?.jointLimitClips === undefined
        ? "unavailable"
        : telemetry.jointLimitClips > 0 ? "warning" : "ok",
      telemetry?.jointLimitClips === null || telemetry?.jointLimitClips === undefined
        ? "Not reported by the robot adapter"
        : `${telemetry.jointLimitClips} clipped target${telemetry.jointLimitClips === 1 ? "" : "s"}`,
    ),
    stage(
      "servo",
      "Servo command output",
      telemetry?.servoOutputChannels === null || telemetry?.servoOutputChannels === undefined
        ? "unavailable"
        : telemetry.servoOutputChannels === 12 ? "ok" : "fault",
      telemetry?.servoOutputChannels === null || telemetry?.servoOutputChannels === undefined
        ? "Not reported by the robot adapter"
        : `${telemetry.servoOutputChannels} / 12 channels`,
    ),
  ];
  const firstBrokenStage = stages.find((candidate) => candidate.status === "fault") ||
    stages.find((candidate) => candidate.status === "warning") || null;
  return {
    linkFresh,
    packetAgeMs,
    packetRateHz,
    acceptedPackets: state?.acceptedPackets || 0,
    rejectedPackets: state?.rejectedPackets || 0,
    droppedPackets: state?.droppedPackets || 0,
    staleTransitions: state?.staleTransitions || 0,
    telemetry,
    stages,
    firstBrokenStage,
    events: [...(state?.events || [])],
    lastPacket: state?.lastPacket ? { ...state.lastPacket } : null,
  };
}

export function liveDiagnosticBundle(state, liveSnapshot, context = {}, now = Date.now()) {
  return {
    schemaVersion: 1,
    generatedAt: new Date(now).toISOString(),
    application: "Domino Virtual Lab",
    context,
    diagnostics: liveDiagnosticsSnapshot(state, liveSnapshot, now),
  };
}
