export const HEARTBEAT_INTERVAL_MS = 500;
export const HEARTBEAT_DEGRADED_AFTER_MS = 1_500;
export const HEARTBEAT_FAULT_AFTER_MS = 3_000;

export function createHeartbeatState() {
  return {
    socketConnected: false,
    openedAt: 0,
    sequence: 0,
    lastSentAt: 0,
    lastAckAt: 0,
    lastAckSequence: 0,
    roundTripMs: null,
  };
}

export function markHeartbeatSocketOpen(state, now = Date.now()) {
  state.socketConnected = true;
  state.openedAt = now;
  state.lastSentAt = 0;
  state.lastAckAt = 0;
  state.lastAckSequence = 0;
  state.roundTripMs = null;
  return state;
}

export function markHeartbeatSocketClosed(state) {
  state.socketConnected = false;
  return state;
}

export function createHeartbeatMessage(state, now = Date.now()) {
  state.sequence += 1;
  state.lastSentAt = now;
  return {
    type: "heartbeat",
    sequence: state.sequence,
    clientSentAt: now,
  };
}

export function acceptHeartbeatAcknowledgement(state, message, now = Date.now()) {
  const sequence = Number(message?.sequence);
  const clientSentAt = Number(message?.clientSentAt);
  if (!Number.isSafeInteger(sequence) || sequence <= state.lastAckSequence) return false;
  if (!Number.isFinite(clientSentAt) || clientSentAt <= 0 || clientSentAt > now) return false;

  const measuredRoundTrip = Math.max(0, now - clientSentAt);
  state.lastAckSequence = sequence;
  state.lastAckAt = now;
  state.roundTripMs = state.roundTripMs === null
    ? measuredRoundTrip
    : state.roundTripMs * 0.7 + measuredRoundTrip * 0.3;
  return true;
}

export function heartbeatStatus(state, now = Date.now()) {
  if (!state.socketConnected) return "disconnected";
  const reference = state.lastAckAt || state.openedAt;
  const age = Math.max(0, now - reference);
  if (!state.lastAckAt && age < HEARTBEAT_DEGRADED_AFTER_MS) return "connecting";
  if (age >= HEARTBEAT_FAULT_AFTER_MS) return "fault";
  if (age >= HEARTBEAT_DEGRADED_AFTER_MS) return "degraded";
  return "connected";
}

export function packetAgeMs(updatedAt, now = Date.now()) {
  const timestamp = Number(updatedAt);
  if (!Number.isFinite(timestamp) || timestamp <= 0) return null;
  return Math.max(0, now - timestamp);
}
