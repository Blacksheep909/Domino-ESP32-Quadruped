export const LIVE_CONTROLLER_CHANNEL_COUNT = 16;
export const LIVE_CONTROLLER_FRESH_MS = 500;
export const LIVE_CONTROLLER_MAX_EVENTS = 100;

const finite = (value) => Number.isFinite(Number(value)) ? Number(value) : null;
const integer = (value) => Number.isSafeInteger(Number(value)) ? Number(value) : null;
const boundedText = (value, fallback = "", maximum = 64) =>
  typeof value === "string" ? value.slice(0, maximum) : fallback;
const within = (value, minimum, maximum) => {
  const numeric = finite(value);
  return numeric !== null && numeric >= minimum && numeric <= maximum ? numeric : null;
};

function sanitizeChannels(channels) {
  if (!Array.isArray(channels) || channels.length !== LIVE_CONTROLLER_CHANNEL_COUNT) return null;
  const sanitized = channels.map((value) => within(value, 800, 2_200));
  return sanitized.every((value) => value !== null) ? sanitized : null;
}

export function sanitizeLiveControllerTelemetry(candidate) {
  if (!candidate || typeof candidate !== "object") return null;
  const channelsUs = sanitizeChannels(candidate.channelsUs);
  const frameTimestampMs = finite(candidate.frameTimestampMs);
  const packetRateHz = within(candidate.packetRateHz, 0, 500);
  const linkQualityPercent = within(candidate.linkQualityPercent, 0, 100);
  const rssi1Dbm = within(candidate.rssi1Dbm, -150, 0);
  if (!channelsUs || frameTimestampMs === null || frameTimestampMs <= 0 || packetRateHz === null || linkQualityPercent === null || rssi1Dbm === null) {
    return null;
  }
  return {
    source: ["crsf-radio", "boxer-elrs"].includes(candidate.source) ? candidate.source : "unknown",
    transmitterName: boundedText(candidate.transmitterName, "CRSF transmitter"),
    receiverName: boundedText(candidate.receiverName, "ExpressLRS receiver"),
    frameTimestampMs,
    channelsUs,
    packetRateHz,
    frameLossCount: Math.max(0, integer(candidate.frameLossCount) ?? 0),
    failsafe: candidate.failsafe === true,
    failsafeCount: Math.max(0, integer(candidate.failsafeCount) ?? 0),
    linkQualityPercent,
    rssi1Dbm,
    rssi2Dbm: within(candidate.rssi2Dbm, -150, 0),
    activeAntenna: [1, 2].includes(integer(candidate.activeAntenna)) ? integer(candidate.activeAntenna) : null,
    snrDb: within(candidate.snrDb, -40, 40),
    rfMode: boundedText(candidate.rfMode, "Unknown", 32),
    txPowerMw: within(candidate.txPowerMw, 0, 10_000),
    receiverVoltageV: within(candidate.receiverVoltageV, 0, 20),
  };
}

function addControllerEvent(state, severity, message, timestampMs) {
  state.events.unshift({
    id: `${timestampMs}-${state.nextEventId++}`,
    timestampMs,
    severity,
    message,
  });
  state.events.splice(LIVE_CONTROLLER_MAX_EVENTS);
}

export function createLiveControllerState() {
  return {
    telemetry: null,
    receivedAt: 0,
    events: [],
    nextEventId: 1,
    previous: {
      linkReady: false,
      failsafe: false,
      failsafeCount: 0,
      frameLossCount: 0,
    },
  };
}

export function acceptLiveControllerTelemetry(state, packet, receivedAt = Date.now()) {
  if (!state || !packet || packet.type !== "live-telemetry") return false;
  const telemetry = sanitizeLiveControllerTelemetry(packet.controller);
  if (!telemetry) return false;
  const frameAgeMs = receivedAt - telemetry.frameTimestampMs;
  const linkReady = ["crsf-radio", "boxer-elrs"].includes(telemetry.source) &&
    frameAgeMs >= 0 &&
    frameAgeMs <= LIVE_CONTROLLER_FRESH_MS &&
    !telemetry.failsafe &&
    telemetry.linkQualityPercent >= 50 &&
    telemetry.rssi1Dbm >= -105;
  if (linkReady && !state.previous.linkReady) addControllerEvent(state, "info", "CRSF / ELRS control link became ready.", receivedAt);
  if (!linkReady && state.previous.linkReady) addControllerEvent(state, "fault", "CRSF / ELRS control link became unavailable.", receivedAt);
  if (telemetry.failsafe && !state.previous.failsafe) addControllerEvent(state, "fault", "Receiver failsafe asserted.", receivedAt);
  if (!telemetry.failsafe && state.previous.failsafe) addControllerEvent(state, "info", "Receiver failsafe cleared.", receivedAt);
  if (telemetry.failsafeCount > state.previous.failsafeCount) {
    addControllerEvent(state, "warning", `Failsafe counter increased to ${telemetry.failsafeCount}.`, receivedAt);
  }
  const lostFrames = telemetry.frameLossCount - state.previous.frameLossCount;
  if (lostFrames > 0) addControllerEvent(state, "warning", `${lostFrames} additional CRSF frame${lostFrames === 1 ? "" : "s"} lost.`, receivedAt);
  state.telemetry = telemetry;
  state.receivedAt = receivedAt;
  state.previous = {
    linkReady,
    failsafe: telemetry.failsafe,
    failsafeCount: telemetry.failsafeCount,
    frameLossCount: telemetry.frameLossCount,
  };
  return true;
}

export function liveControllerSnapshot(state, now = Date.now()) {
  const telemetry = state?.telemetry;
  const transportAgeMs = state?.receivedAt > 0 && now >= state.receivedAt ? now - state.receivedAt : null;
  const frameAgeMs = telemetry && now >= telemetry.frameTimestampMs ? now - telemetry.frameTimestampMs : null;
  const fresh = Boolean(
    telemetry &&
    transportAgeMs !== null &&
    transportAgeMs <= LIVE_CONTROLLER_FRESH_MS &&
    frameAgeMs !== null &&
    frameAgeMs <= LIVE_CONTROLLER_FRESH_MS
  );
  const linkReady = Boolean(
    fresh &&
    ["crsf-radio", "boxer-elrs"].includes(telemetry.source) &&
    !telemetry.failsafe &&
    telemetry.linkQualityPercent >= 50 &&
    telemetry.rssi1Dbm >= -105
  );
  const quality = !fresh
    ? "offline"
    : telemetry.failsafe
      ? "failsafe"
      : telemetry.linkQualityPercent < 50 || telemetry.rssi1Dbm < -105
        ? "poor"
        : telemetry.linkQualityPercent < 80 || telemetry.rssi1Dbm < -90
          ? "degraded"
          : "good";
  if (!linkReady && state?.previous?.linkReady) {
    addControllerEvent(state, "fault", "CRSF / ELRS control evidence became stale or unsafe.", now);
    state.previous.linkReady = false;
  }
  return {
    fresh,
    linkReady,
    quality,
    transportAgeMs,
    frameAgeMs,
    telemetry: fresh ? telemetry : null,
    events: state?.events || [],
  };
}

export function liveControllerDiagnosticExport(state, now = Date.now()) {
  const snapshot = liveControllerSnapshot(state, now);
  return {
    generatedAt: now,
    linkReady: snapshot.linkReady,
    quality: snapshot.quality,
    frameAgeMs: snapshot.frameAgeMs,
    transportAgeMs: snapshot.transportAgeMs,
    telemetry: state?.telemetry || null,
    events: (state?.events || []).map((event) => ({ ...event })),
  };
}
