export const LIVE_GAIT_ACTIONS = Object.freeze([
  "request-profile",
  "apply-profile",
  "revert-profile",
]);

const gaitBounds = Object.freeze({
  cadenceHz: [0.35, 2.5], strideMm: [24, 120], liftMm: [8, 70],
  dutyFactor: [0.5, 0.82], bodyHeightMm: [220, 280], stanceWidthMm: [34, 70],
  turnGain: [0, 1.5], responseMs: [60, 700], swingShape: [0.8, 3],
  diagonalPhase: [0.4, 0.6],
});

export function validLiveGaitProfile(profile) {
  if (
    profile?.schemaVersion !== 1 || profile?.robot !== "domino-esp32-quadruped" ||
    typeof profile.name !== "string" || !profile.name.trim() || profile.name.length > 32 ||
    typeof profile.settings?.enabled !== "boolean" ||
    typeof profile.settings?.preset !== "string" || !profile.settings.preset ||
    profile.settings.preset.length > 11
  ) return false;
  return Object.entries(gaitBounds).every(([key, [minimum, maximum]]) => {
    const value = profile.settings[key];
    return typeof value === "number" && Number.isFinite(value) && value >= minimum && value <= maximum;
  });
}

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
      validLiveGaitProfile(message.profile),
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
