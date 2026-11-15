// Emotes & sounds (LIVE / EMOTES). Studio reads and saves the robot's
// personality settings and previews emotes. The robot stores the settings in
// its own flash record ("personality"), separate from the servo calibration,
// so turning everything off survives a restart.

export const LIVE_PERSONALITY_ACTIONS = Object.freeze([
  "request-settings",
  "apply-settings",
  "preview-emote",
]);

// Must match personalityEmoteName() in src/personality.cpp.
export const PERSONALITY_EMOTES = Object.freeze([
  { id: "curious", label: "Curious", detail: "Antennas perk up, lopsided head-tilt, rises a little." },
  { id: "happy", label: "Happy", detail: "Quick antenna flutter with two small bobs." },
  { id: "bow", label: "Play bow", detail: "Dips low with antennas forward, then back up." },
  { id: "look", label: "Look around", detail: "Antennas scan left and right like ears." },
  { id: "shy", label: "Shy", detail: "Antennas flat back, sinks low, slowly recovers." },
]);

export const PERSONALITY_CHATTER = Object.freeze(["off", "rare", "normal"]);

export const DEFAULT_PERSONALITY_SETTINGS = Object.freeze({
  enabled: true,
  soundEnabled: true,
  volumePercent: 45,
  idleChatter: "rare",
  emoteChannel: 8,
  fixedEmote: "random",
});

export function validPersonalitySettings(settings) {
  return Boolean(
    settings &&
    typeof settings.enabled === "boolean" &&
    typeof settings.soundEnabled === "boolean" &&
    Number.isInteger(settings.volumePercent) && settings.volumePercent >= 0 && settings.volumePercent <= 100 &&
    PERSONALITY_CHATTER.includes(settings.idleChatter) &&
    Number.isInteger(settings.emoteChannel) && settings.emoteChannel >= 0 && settings.emoteChannel <= 16 &&
    (settings.fixedEmote === "random" || PERSONALITY_EMOTES.some((emote) => emote.id === settings.fixedEmote))
  );
}

export function sanitizePersonalitySettings(candidate = {}) {
  const merged = { ...DEFAULT_PERSONALITY_SETTINGS, ...(candidate || {}) };
  const settings = {
    enabled: merged.enabled !== false,
    soundEnabled: merged.soundEnabled !== false,
    volumePercent: Math.min(100, Math.max(0, Math.round(Number(merged.volumePercent) || 0))),
    idleChatter: PERSONALITY_CHATTER.includes(merged.idleChatter) ? merged.idleChatter : "rare",
    emoteChannel: Math.min(16, Math.max(0, Math.round(Number(merged.emoteChannel) || 0))),
    fixedEmote: PERSONALITY_EMOTES.some((emote) => emote.id === merged.fixedEmote) ? merged.fixedEmote : "random",
  };
  return settings;
}

export function validLivePersonalityCommand(message) {
  if (
    !message ||
    message.type !== "live-personality-command" ||
    !LIVE_PERSONALITY_ACTIONS.includes(message.action) ||
    typeof message.requestId !== "string" ||
    !message.requestId
  ) return false;
  if (message.action === "apply-settings") return validPersonalitySettings(message.settings);
  if (message.action === "preview-emote") {
    return message.emote === "random" || PERSONALITY_EMOTES.some((emote) => emote.id === message.emote);
  }
  return true;
}

export function validLivePersonalityAcknowledgement(message) {
  return Boolean(
    message &&
    message.type === "live-personality-ack" &&
    LIVE_PERSONALITY_ACTIONS.includes(message.action) &&
    typeof message.requestId === "string" &&
    message.requestId &&
    typeof message.accepted === "boolean" &&
    (message.settings === undefined || validPersonalitySettings(message.settings))
  );
}

export function createLivePersonalityCommand(action, requestId, { settings, emote } = {}) {
  const command = { type: "live-personality-command", action, requestId };
  if (action === "apply-settings") command.settings = sanitizePersonalitySettings(settings);
  if (action === "preview-emote") command.emote = emote;
  return validLivePersonalityCommand(command) ? command : null;
}

// Plain-language summary for the tab header.
export function describePersonalitySettings(settings) {
  const s = sanitizePersonalitySettings(settings);
  if (!s.enabled) return "Off: silent and still. No emotes, reactions or idle sounds.";
  const switchText = s.emoteChannel ? `CH${s.emoteChannel} switch plays ${s.fixedEmote === "random" ? "a random emote" : `"${s.fixedEmote}"`}` : "no emote switch";
  const soundText = s.soundEnabled ? `sound ${s.volumePercent}%` : "sound muted";
  const chatterText = { off: "no idle sounds", rare: "rare idle sounds", normal: "occasional idle sounds" }[s.idleChatter];
  return `On: ${switchText}, ${soundText}, ${chatterText}.`;
}
