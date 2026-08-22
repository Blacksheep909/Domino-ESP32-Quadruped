import { EXPERIENCE_EXPERT, EXPERIENCE_SIMPLE } from "./app-state.js";
import { ENABLED_LIVE_VIEWS, LIVE_VIEW_COMPARE } from "./live-view-state.js";

export const PRESENTATION_PREFERENCES_STORAGE_KEY = "domino-presentation-preferences-v1";
export const PRESENTATION_PREFERENCES_SCHEMA_VERSION = 1;

const defaultPreferences = () => ({
  schemaVersion: PRESENTATION_PREFERENCES_SCHEMA_VERSION,
  experience: EXPERIENCE_SIMPLE,
  liveView: LIVE_VIEW_COMPARE,
  calibrationFloat: true,
});

export function sanitizePresentationPreferences(candidate) {
  const defaults = defaultPreferences();
  if (!candidate || candidate.schemaVersion !== PRESENTATION_PREFERENCES_SCHEMA_VERSION) return defaults;
  return {
    schemaVersion: PRESENTATION_PREFERENCES_SCHEMA_VERSION,
    experience: [EXPERIENCE_SIMPLE, EXPERIENCE_EXPERT].includes(candidate.experience)
      ? candidate.experience
      : defaults.experience,
    liveView: ENABLED_LIVE_VIEWS.includes(candidate.liveView)
      ? candidate.liveView
      : defaults.liveView,
    calibrationFloat: typeof candidate.calibrationFloat === "boolean"
      ? candidate.calibrationFloat
      : defaults.calibrationFloat,
  };
}

export function parsePresentationPreferencesJson(contents) {
  try {
    return sanitizePresentationPreferences(JSON.parse(String(contents)));
  } catch {
    return defaultPreferences();
  }
}

export function presentationPreferencesJson(candidate) {
  return JSON.stringify(sanitizePresentationPreferences({
    schemaVersion: PRESENTATION_PREFERENCES_SCHEMA_VERSION,
    ...candidate,
  }));
}
