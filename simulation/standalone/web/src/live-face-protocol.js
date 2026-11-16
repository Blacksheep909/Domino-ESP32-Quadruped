// OLED eyes (LIVE / SCREENS). Studio reads and saves the robot's screen
// settings (NVS record "face", see src/face.*) and can ask it to show the
// battery now. Settings shape: face-engine.js DEFAULT_FACE_SETTINGS.

import { sanitizeFaceSettings, validFaceSettings } from "./face-engine.js";

export const LIVE_FACE_ACTIONS = Object.freeze(["request-settings", "apply-settings", "show-battery"]);

export function validLiveFaceCommand(message) {
  if (
    !message ||
    message.type !== "live-face-command" ||
    !LIVE_FACE_ACTIONS.includes(message.action) ||
    typeof message.requestId !== "string" ||
    !message.requestId
  ) return false;
  if (message.action === "apply-settings") return validFaceSettings(message.settings);
  return true;
}

export function validLiveFaceAcknowledgement(message) {
  return Boolean(
    message &&
    message.type === "live-face-ack" &&
    LIVE_FACE_ACTIONS.includes(message.action) &&
    typeof message.requestId === "string" &&
    message.requestId &&
    typeof message.accepted === "boolean" &&
    (message.settings === undefined || validFaceSettings(message.settings))
  );
}

export function createLiveFaceCommand(action, requestId, { settings } = {}) {
  const command = { type: "live-face-command", action, requestId };
  if (action === "apply-settings") command.settings = sanitizeFaceSettings(settings);
  return validLiveFaceCommand(command) ? command : null;
}

export function describeFaceSettings(settings) {
  const s = sanitizeFaceSettings(settings);
  if (s.mode === "off") return "Screens off.";
  if (s.mode === "telemetry") return "Telemetry on both screens: power and link on the left, motion on the right.";
  const parts = [`${s.style === "burne" ? "BURN-E" : s.style} eyes`];
  parts.push(s.lowBatteryPercent ? `low battery flashes at ${s.lowBatteryPercent}% on the ${s.batteryEye} eye` : "no low-battery warning");
  parts.push(s.reminderMinutes ? `battery shown when idle, every ${s.reminderMinutes} min at most` : "no idle battery reminder");
  return `${parts.join(", ")}.`;
}
