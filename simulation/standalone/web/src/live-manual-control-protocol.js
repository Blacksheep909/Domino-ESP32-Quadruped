import { validSessionEnvelope } from "./live-connection-protocol.js";

export const LIVE_MANUAL_AUTHORITY_ACTIONS = Object.freeze(["request-authority", "release-authority"]);
export const LIVE_MANUAL_MODES = Object.freeze(["stand", "careful", "trot"]);
export const LIVE_MANUAL_TIMEOUT_MS = 250;
export const LIVE_MANUAL_FRAME_INTERVAL_MS = 50;
export const LIVE_MANUAL_MAX_LEASE_MS = 30_000;

const boundedString = (value, maximum = 128) =>
  typeof value === "string" && value.length > 0 && value.length <= maximum;
const boundedAxis = (value) => Number.isFinite(Number(value)) && Number(value) >= -1 && Number(value) <= 1;

export function validLiveManualAuthorityCommand(message) {
  if (
    !message ||
    message.type !== "live-manual-authority-command" ||
    !LIVE_MANUAL_AUTHORITY_ACTIONS.includes(message.action) ||
    !boundedString(message.requestId) ||
    !validSessionEnvelope(message) ||
    !Number.isFinite(Number(message.timestampMs))
  ) return false;
  if (message.action === "request-authority") {
    return Boolean(
      message.safety?.requiresArmed === true &&
      message.safety?.requiresDeadman === true &&
      message.safety?.neutralOnRelease === true &&
      Number(message.safety?.commandTimeoutMs) > 0 &&
      Number(message.safety?.commandTimeoutMs) <= LIVE_MANUAL_TIMEOUT_MS &&
      Number(message.requestedLeaseMs) > 0 &&
      Number(message.requestedLeaseMs) <= LIVE_MANUAL_MAX_LEASE_MS
    );
  }
  return boundedString(message.authorityToken);
}

export function validLiveManualAuthorityAcknowledgement(message) {
  if (
    !message ||
    message.type !== "live-manual-authority-ack" ||
    !LIVE_MANUAL_AUTHORITY_ACTIONS.includes(message.action) ||
    !boundedString(message.requestId) ||
    !validSessionEnvelope(message) ||
    typeof message.accepted !== "boolean"
  ) return false;
  if (message.action === "request-authority" && message.accepted) {
    return Boolean(
      boundedString(message.authorityToken) &&
      Number(message.leaseMs) > 0 &&
      Number(message.leaseMs) <= LIVE_MANUAL_MAX_LEASE_MS &&
      message.robotState === "armed"
    );
  }
  if (message.action === "release-authority") return boundedString(message.authorityToken);
  return true;
}

export function validLiveManualControlFrame(message) {
  if (
    !message ||
    message.type !== "live-manual-control-frame" ||
    !validSessionEnvelope(message) ||
    !boundedString(message.authorityToken) ||
    !Number.isSafeInteger(Number(message.sequence)) ||
    Number(message.sequence) < 0 ||
    !Number.isFinite(Number(message.timestampMs)) ||
    !LIVE_MANUAL_MODES.includes(message.mode) ||
    !boundedAxis(message.axes?.forward) ||
    !boundedAxis(message.axes?.turn) ||
    !boundedAxis(message.axes?.roll) ||
    !boundedAxis(message.axes?.height) ||
    message.safety?.requiresArmed !== true ||
    message.safety?.neutralOnExpiry !== true ||
    Number(message.safety?.timeoutMs) <= 0 ||
    Number(message.safety?.timeoutMs) > LIVE_MANUAL_TIMEOUT_MS
  ) return false;
  if (message.deadman === true) return message.neutral === false;
  return Boolean(
    message.deadman === false &&
    message.neutral === true &&
    message.mode === "stand" &&
    Object.values(message.axes).every((value) => Number(value) === 0)
  );
}
