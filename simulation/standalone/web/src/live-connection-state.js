import {
  LIVE_CONNECTION_TRANSPORTS,
  validLiveAdapterAnnouncement,
  validLiveConnectionAcknowledgement,
} from "./live-connection-protocol.js";

export const LIVE_ADAPTER_FRESH_MS = 4_000;

const safeText = (value, fallback = "") => typeof value === "string" ? value.slice(0, 96) : fallback;

function sanitizeAdapter(message, receivedAt) {
  return {
    adapterId: message.adapterId,
    name: message.name,
    transport: message.transport,
    state: message.state,
    signalPercent: Number.isFinite(Number(message.signalPercent))
      ? Math.max(0, Math.min(100, Number(message.signalPercent)))
      : null,
    endpoint: safeText(message.endpoint),
    robot: {
      id: safeText(message.robot?.id),
      name: safeText(message.robot?.name, "Domino"),
      firmwareVersion: safeText(message.robot?.firmwareVersion, "Unknown"),
    },
    capabilities: {
      telemetry: message.capabilities?.telemetry === true,
      calibration: message.capabilities?.calibration === true,
      gaitProfiles: message.capabilities?.gaitProfiles === true,
      persistentProfiles: message.capabilities?.persistentProfiles === true,
      manualControl: message.capabilities?.manualControl === true,
    },
    receivedAt,
  };
}

export function createLiveConnectionState() {
  return {
    bridgeConnected: false,
    transportFilter: "auto",
    adapters: {},
    selectedAdapterId: "",
    phase: "disconnected",
    sessionId: "",
    robotState: "unknown",
    pendingRequestId: "",
    pendingAction: "",
    lastDiscoveryAt: 0,
    connectedAt: 0,
    status: "Choose a transport and search for a compatible Domino adapter.",
    error: "",
  };
}

export function setLiveConnectionBridge(state, connected) {
  if (!state) return false;
  state.bridgeConnected = Boolean(connected);
  if (!connected) {
    state.phase = state.sessionId ? "lost" : "disconnected";
    state.sessionId = "";
    state.pendingRequestId = "";
    state.pendingAction = "";
    state.robotState = "unknown";
    state.status = "The local bridge is offline. Robot commands remain blocked.";
  }
  return true;
}

export function setLiveConnectionTransport(state, transport) {
  if (!state || (transport !== "auto" && !LIVE_CONNECTION_TRANSPORTS.includes(transport))) return false;
  state.transportFilter = transport;
  return true;
}

export function acceptLiveAdapterAnnouncement(state, message, receivedAt = Date.now()) {
  if (!state || !validLiveAdapterAnnouncement(message)) return false;
  const adapter = sanitizeAdapter(message, receivedAt);
  state.adapters[adapter.adapterId] = adapter;
  if (!state.selectedAdapterId) state.selectedAdapterId = adapter.adapterId;
  if (state.sessionId && state.selectedAdapterId === adapter.adapterId && adapter.state === "error") {
    state.phase = "lost";
    state.sessionId = "";
    state.robotState = "unknown";
    state.status = `${adapter.name} reported a connection fault.`;
  }
  return true;
}

export function selectLiveAdapter(state, adapterId) {
  if (!state || !state.adapters[adapterId] || state.sessionId) return false;
  state.selectedAdapterId = adapterId;
  state.error = "";
  return true;
}

export function visibleLiveAdapters(state, now = Date.now()) {
  if (!state) return [];
  return Object.values(state.adapters)
    .filter((adapter) => now - adapter.receivedAt <= LIVE_ADAPTER_FRESH_MS)
    .filter((adapter) => state.transportFilter === "auto" || adapter.transport === state.transportFilter)
    .sort((left, right) => left.name.localeCompare(right.name));
}

export function pruneLiveAdapters(state, now = Date.now()) {
  if (!state) return false;
  let changed = false;
  Object.values(state.adapters).forEach((adapter) => {
    if (now - adapter.receivedAt <= LIVE_ADAPTER_FRESH_MS) return;
    delete state.adapters[adapter.adapterId];
    changed = true;
    if (state.selectedAdapterId === adapter.adapterId && state.sessionId) {
      state.phase = "lost";
      state.sessionId = "";
      state.robotState = "unknown";
      state.status = `Lost the ${adapter.name} adapter heartbeat. Commands are blocked.`;
    }
  });
  if (state.selectedAdapterId && !state.adapters[state.selectedAdapterId] && !state.sessionId) {
    state.selectedAdapterId = "";
  }
  return changed;
}

export function removeLiveAdapter(state, adapterId, reason = "offline") {
  if (!state?.adapters?.[adapterId]) return false;
  const adapter = state.adapters[adapterId];
  delete state.adapters[adapterId];
  if (state.selectedAdapterId === adapterId && state.sessionId) {
    state.phase = "lost";
    state.sessionId = "";
    state.robotState = "unknown";
    state.status = `The ${adapter.name} adapter went ${reason}. Commands are blocked.`;
  }
  if (state.selectedAdapterId === adapterId && !state.sessionId) state.selectedAdapterId = "";
  return true;
}

export function createLiveConnectionCommand(state, action, requestId, now = Date.now()) {
  if (!state || !state.bridgeConnected || state.pendingRequestId) return null;
  const base = {
    type: "live-connection-command",
    action,
    requestId: String(requestId),
    timestampMs: now,
  };
  if (action === "discover") {
    state.phase = "discovering";
    state.lastDiscoveryAt = now;
    state.status = "Searching for compatible adapters on the selected transport...";
    return { ...base, transport: state.transportFilter };
  }
  const adapter = state.adapters[state.selectedAdapterId];
  if (!adapter) return null;
  if (action === "connect" && !state.sessionId) {
    state.phase = "connecting";
    state.status = `Requesting a read-only handshake with ${adapter.name}...`;
    return {
      ...base,
      adapterId: adapter.adapterId,
      transport: adapter.transport,
      safety: { readOnlyHandshake: true, commandsBlockedUntilStateKnown: true },
    };
  }
  if (action === "disconnect" && state.sessionId) {
    state.phase = "disconnecting";
    state.status = `Closing the ${adapter.name} engineering session...`;
    return { ...base, adapterId: adapter.adapterId, sessionId: state.sessionId };
  }
  return null;
}

export function markLiveConnectionPending(state, command) {
  if (!state || !command) return false;
  state.pendingRequestId = command.requestId;
  state.pendingAction = command.action;
  state.error = "";
  return true;
}

export function acceptLiveConnectionAcknowledgement(state, message, receivedAt = Date.now()) {
  if (
    !state ||
    !validLiveConnectionAcknowledgement(message) ||
    message.requestId !== state.pendingRequestId ||
    message.action !== state.pendingAction
  ) return false;
  state.pendingRequestId = "";
  state.pendingAction = "";
  if (!message.accepted) {
    state.phase = state.sessionId ? "connected" : "disconnected";
    state.error = safeText(message.reason, "The adapter rejected the request.");
    state.status = state.error;
    return true;
  }
  if (message.action === "discover") {
    state.phase = state.sessionId ? "connected" : "disconnected";
    state.status = "Discovery complete. Select an adapter to continue.";
  } else if (message.action === "connect") {
    state.selectedAdapterId = message.adapterId;
    state.sessionId = message.sessionId;
    state.robotState = message.robotState;
    state.phase = "connected";
    state.connectedAt = receivedAt;
    state.status = message.robotState === "disarmed"
      ? "Engineering session established. Robot commands remain explicitly gated."
      : `Connected while the robot reports ${message.robotState.toUpperCase()}. Motion commands are blocked.`;
  } else {
    state.phase = "disconnected";
    state.sessionId = "";
    state.robotState = "unknown";
    state.status = "Engineering session disconnected safely.";
  }
  return true;
}

export function liveConnectionIsReady(state, now = Date.now()) {
  const adapter = state?.adapters?.[state.selectedAdapterId];
  return Boolean(
    state?.bridgeConnected &&
    state.phase === "connected" &&
    state.sessionId &&
    adapter &&
    now - adapter.receivedAt <= LIVE_ADAPTER_FRESH_MS
  );
}

export function telemetryBelongsToLiveConnection(state, packet, now = Date.now()) {
  return Boolean(
    liveConnectionIsReady(state, now) &&
    packet?.adapterId === state.selectedAdapterId &&
    packet?.sessionId === state.sessionId
  );
}

export function liveConnectionEnvelope(state) {
  if (!state?.selectedAdapterId || !state?.sessionId) return null;
  return { adapterId: state.selectedAdapterId, sessionId: state.sessionId };
}

export function failLiveConnectionRequest(state, requestId, reason) {
  if (!state || state.pendingRequestId !== requestId) return false;
  state.pendingRequestId = "";
  state.pendingAction = "";
  state.phase = state.sessionId ? "connected" : "disconnected";
  state.error = reason;
  state.status = reason;
  return true;
}
