import { validMotionSmoothingSettings } from "./live-motion-smoothing-state.js";

export const LIVE_CALIBRATION_JOG_LIMIT_DEG = 10;
export const LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC = 5;
export const LIVE_CALIBRATION_ACTIONS = Object.freeze([
  "request-profile", "request-smoothing", "save-smoothing", "enter", "exit", "jog", "save-profile", "keepalive",
]);

// Firmware 0.8.21 and later saves the profile to flash without bench mode: the
// save turns every output off first and never energizes a servo, so a broken
// bench handshake can no longer make a calibration impossible to persist.
// Older firmware refuses a save unless bench mode is active.
export function firmwareSavesWithoutBench(version) {
  const parts = String(version ?? "").match(/^(\d+)\.(\d+)\.(\d+)/);
  if (!parts) return false;
  const [major, minor, patch] = parts.slice(1).map(Number);
  return major > 0 || minor > 8 || (minor === 8 && patch >= 21);
}

function benchFlagValid(message) {
  // Saving states for itself whether the robot must be in bench mode; every
  // other action has a fixed requirement.
  if (message.action === "save-profile") return typeof message.safety?.benchModeRequired === "boolean";
  return message.safety?.benchModeRequired ===
    !["request-profile", "request-smoothing", "save-smoothing"].includes(message.action);
}

export function validCalibrationCommand(message) {
  const envelopeValid = Boolean(
    message &&
    message.type === "live-calibration-command" &&
    LIVE_CALIBRATION_ACTIONS.includes(message.action) &&
    typeof message.requestId === "string" &&
    message.requestId.length > 0 &&
    benchFlagValid(message) &&
    Number(message.safety?.maxSpeedDegPerSec) > 0 &&
    Number(message.safety?.maxSpeedDegPerSec) <= LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC &&
    Number(message.safety?.jogLimitDeg) > 0 &&
    Number(message.safety?.jogLimitDeg) <= LIVE_CALIBRATION_JOG_LIMIT_DEG
  );
  if (!envelopeValid) return false;
  if (message.action === "keepalive") return Number.isFinite(Number(message.timestampMs));
  if (message.action === "jog") {
    return Number.isInteger(Number(message.selectedChannel)) &&
      Number(message.selectedChannel) >= 0 && Number(message.selectedChannel) < 16 &&
      Number.isInteger(Number(message.physicalChannel)) &&
      Number(message.physicalChannel) >= 0 && Number(message.physicalChannel) < 16 &&
      Number.isFinite(Number(message.targetServoDeg));
  }
  if (message.action === "save-profile") {
    const joints = message.profile?.joints;
    const physicalChannels = Array.isArray(joints) ? joints.map((joint) => Number(joint?.channel)) : [];
    const logicalChannels = Array.isArray(joints) ? joints.map((joint) => Number(joint?.logicalChannel)) : [];
    return Number.isSafeInteger(message.baseSavedAt) && message.baseSavedAt >= 0 &&
      Number.isSafeInteger(message.profile?.savedAt) && message.profile.savedAt > message.baseSavedAt &&
      message.profile?.schemaVersion === 2 && Array.isArray(joints) && joints.length === 12 &&
      physicalChannels.every((channel) => Number.isInteger(channel) && channel >= 0 && channel < 16) &&
      logicalChannels.every((channel) => Number.isInteger(channel) && channel >= 0 && channel < 16) &&
      new Set(physicalChannels).size === 12 && new Set(logicalChannels).size === 12;
  }
  if (message.action === "save-smoothing") {
    return Number.isSafeInteger(message.baseUpdatedAt) && message.baseUpdatedAt >= 0 &&
      validMotionSmoothingSettings(message.smoothing) &&
      message.smoothing.updatedAt > message.baseUpdatedAt;
  }
  return true;
}

export function validCalibrationAcknowledgement(message) {
  return Boolean(
    message &&
    message.type === "live-calibration-ack" &&
    LIVE_CALIBRATION_ACTIONS.includes(message.action) &&
    typeof message.requestId === "string" &&
    message.requestId.length > 0 &&
    typeof message.accepted === "boolean"
  );
}
