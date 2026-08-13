export const LIVE_GAIT_ACTIONS = Object.freeze([
  "request-profile",
  "apply-profile",
  "revert-profile",
]);

export function validLiveGaitCommand(message) {
  if (
    !message ||
    message.type !== "live-gait-command" ||
    !LIVE_GAIT_ACTIONS.includes(message.action) ||
    typeof message.requestId !== "string" ||
    !message.requestId
  ) return false;
  if (message.action === "apply-profile") {
    return Boolean(
      message.safety?.requiresDisarmed === true &&
      message.safety?.twoStageApply === true &&
      message.profile?.schemaVersion === 1 &&
      message.profile?.robot === "domino-esp32-quadruped",
    );
  }
  return true;
}

export function validLiveGaitAcknowledgement(message) {
  return Boolean(
    message &&
    message.type === "live-gait-ack" &&
    LIVE_GAIT_ACTIONS.includes(message.action) &&
    typeof message.requestId === "string" &&
    message.requestId &&
    typeof message.accepted === "boolean"
  );
}
