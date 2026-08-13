export const LIVE_CONNECTION_TRANSPORTS = Object.freeze(["wifi", "bluetooth", "usb"]);
export const LIVE_CONNECTION_ACTIONS = Object.freeze(["discover", "connect", "disconnect"]);
export const LIVE_ADAPTER_STATES = Object.freeze(["available", "connecting", "connected", "error"]);

const boundedString = (value, maximum = 64) =>
  typeof value === "string" && value.length > 0 && value.length <= maximum;

export function validLiveAdapterAnnouncement(message) {
  return Boolean(
    message &&
    message.type === "live-adapter-announce" &&
    boundedString(message.adapterId) &&
    boundedString(message.name) &&
    LIVE_CONNECTION_TRANSPORTS.includes(message.transport) &&
    LIVE_ADAPTER_STATES.includes(message.state) &&
    Number.isFinite(Number(message.timestampMs)) &&
    Number(message.timestampMs) > 0 &&
    (!message.robot || typeof message.robot === "object") &&
    (!message.capabilities || typeof message.capabilities === "object")
  );
}

export function validLiveConnectionCommand(message) {
  if (
    !message ||
    message.type !== "live-connection-command" ||
    !LIVE_CONNECTION_ACTIONS.includes(message.action) ||
    !boundedString(message.requestId) ||
    !Number.isFinite(Number(message.timestampMs))
  ) return false;
  if (message.action === "discover") {
    return message.transport === "auto" || LIVE_CONNECTION_TRANSPORTS.includes(message.transport);
  }
  if (!boundedString(message.adapterId)) return false;
  if (message.action === "connect") {
    return Boolean(
      LIVE_CONNECTION_TRANSPORTS.includes(message.transport) &&
      message.safety?.readOnlyHandshake === true &&
      message.safety?.commandsBlockedUntilStateKnown === true
    );
  }
  return boundedString(message.sessionId, 96);
}

export function validLiveConnectionAcknowledgement(message) {
  if (
    !message ||
    message.type !== "live-connection-ack" ||
    !LIVE_CONNECTION_ACTIONS.includes(message.action) ||
    !boundedString(message.requestId) ||
    typeof message.accepted !== "boolean"
  ) return false;
  if (message.action === "discover") return true;
  if (!boundedString(message.adapterId)) return false;
  if (message.action === "connect" && message.accepted) {
    return Boolean(
      boundedString(message.sessionId, 96) &&
      ["disarmed", "armed", "estopped", "fault"].includes(message.robotState)
    );
  }
  if (message.action === "disconnect") return boundedString(message.sessionId, 96);
  return true;
}

export function validSessionEnvelope(message) {
  return Boolean(
    message &&
    boundedString(message.adapterId) &&
    boundedString(message.sessionId, 96)
  );
}
