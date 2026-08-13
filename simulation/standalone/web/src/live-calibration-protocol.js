export const LIVE_CALIBRATION_JOG_LIMIT_DEG = 10;
export const LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC = 5;
export const LIVE_CALIBRATION_ACTIONS = Object.freeze(["enter", "exit", "jog", "save-profile"]);

export function validCalibrationCommand(message) {
  return Boolean(
    message &&
    message.type === "live-calibration-command" &&
    LIVE_CALIBRATION_ACTIONS.includes(message.action) &&
    typeof message.requestId === "string" &&
    message.requestId.length > 0 &&
    message.safety?.benchModeRequired === true &&
    Number(message.safety?.maxSpeedDegPerSec) > 0 &&
    Number(message.safety?.maxSpeedDegPerSec) <= LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC &&
    Number(message.safety?.jogLimitDeg) > 0 &&
    Number(message.safety?.jogLimitDeg) <= LIVE_CALIBRATION_JOG_LIMIT_DEG
  );
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
