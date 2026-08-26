export const LIVE_NAVIGATION_ACTIVITY_LIMIT = 40;

const activityTones = new Set(["info", "success", "warning", "fault"]);

export function createLiveNavigationActivityState(maximumEvents = LIVE_NAVIGATION_ACTIVITY_LIMIT) {
  return {
    maximumEvents: Math.max(1, Math.floor(Number(maximumEvents) || LIVE_NAVIGATION_ACTIVITY_LIMIT)),
    nextEventId: 1,
    events: [],
  };
}

export function recordLiveNavigationActivity(state, kind, message, tone = "info", timestampMs = Date.now()) {
  if (!state || typeof message !== "string" || !message.trim()) return null;
  const event = {
    id: `${timestampMs}-${state.nextEventId++}`,
    timestampMs: Number.isFinite(Number(timestampMs)) ? Number(timestampMs) : Date.now(),
    kind: String(kind || "activity").slice(0, 32),
    tone: activityTones.has(tone) ? tone : "info",
    message: message.trim().slice(0, 240),
  };
  state.events.unshift(event);
  state.events.splice(state.maximumEvents);
  return event;
}

export function clearLiveNavigationActivity(state) {
  if (!state) return false;
  state.events = [];
  return true;
}

export function liveNavigationActivitySnapshot(state) {
  return [...(state?.events || [])];
}

export function liveNavigationActivityBundle(state, context = {}, now = Date.now()) {
  return {
    schemaVersion: 1,
    generatedAt: new Date(now).toISOString(),
    application: "Domino Virtual Lab",
    context,
    events: liveNavigationActivitySnapshot(state),
  };
}
