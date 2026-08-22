import { validSessionEnvelope } from "./live-connection-protocol.js";

export const LIVE_SAFETY_ACTIONS = Object.freeze(["request-state", "arm", "disarm", "estop", "reset-estop", "acknowledge-fault"]);
export const LIVE_ROBOT_STATES = Object.freeze(["unknown", "disarmed", "arming", "armed", "disarming", "estopped", "fault", "watchdog"]);
export const LIVE_ARM_HOLD_MS = 1_500;
export const LIVE_SAFETY_HEARTBEAT_INTERVAL_MS = 100;
export const LIVE_SAFETY_WATCHDOG_MS = 400;

const requestIdValid = (message) => typeof message?.requestId === "string" && message.requestId.length > 0;

export function validLiveSafetyCommand(message) {
  if (
    !message ||
    message.type !== "live-safety-command" ||
    !LIVE_SAFETY_ACTIONS.includes(message.action) ||
    !requestIdValid(message) ||
    !validSessionEnvelope(message) ||
    !Number.isFinite(Number(message.timestampMs))
  ) return false;
  if (message.action === "arm") {
    return Boolean(
      Number(message.safety?.holdMs) >= LIVE_ARM_HOLD_MS &&
      message.safety?.requiresDisarmed === true &&
      message.safety?.requiresFreshTelemetry === true &&
      message.safety?.requiresDriveLink === true &&
      Number(message.safety?.watchdogMs) > 0 &&
      Number(message.safety?.watchdogMs) <= LIVE_SAFETY_WATCHDOG_MS
    );
  }
  if (message.action === "reset-estop") return message.safety?.physicalResetRequired === true;
  if (message.action === "acknowledge-fault") return message.safety?.requiresFaultState === true;
  return true;
}

export function validLiveSafetyAcknowledgement(message) {
  return Boolean(
    message &&
    message.type === "live-safety-ack" &&
    LIVE_SAFETY_ACTIONS.includes(message.action) &&
    requestIdValid(message) &&
    validSessionEnvelope(message) &&
    typeof message.accepted === "boolean" &&
    LIVE_ROBOT_STATES.includes(message.robotState)
  );
}

export function validLiveSafetyHeartbeat(message) {
  return Boolean(
    message &&
    message.type === "live-safety-heartbeat" &&
    validSessionEnvelope(message) &&
    Number.isSafeInteger(Number(message.sequence)) &&
    Number(message.sequence) >= 0 &&
    Number.isFinite(Number(message.timestampMs)) &&
    message.workspaceActive === true
  );
}

export function validLiveSafetyHeartbeatAcknowledgement(message) {
  return Boolean(
    message &&
    message.type === "live-safety-heartbeat-ack" &&
    validSessionEnvelope(message) &&
    Number.isSafeInteger(Number(message.sequence)) &&
    Number(message.sequence) >= 0 &&
    LIVE_ROBOT_STATES.includes(message.robotState) &&
    Number(message.watchdogRemainingMs) >= 0 &&
    Number(message.watchdogRemainingMs) <= LIVE_SAFETY_WATCHDOG_MS
  );
}
