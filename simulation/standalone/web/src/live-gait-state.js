import {
  defaultGaitLabSettings,
  gaitLabControls,
  gaitLabPresets,
  sanitizeGaitLabSettings,
} from "./gait-lab.js";

export const LIVE_GAIT_SCHEMA_VERSION = 1;
export const LIVE_GAIT_LIBRARY_KEY = "domino-gait-profiles-v1";

const editableSettings = (settings = {}) => Object.fromEntries(
  gaitLabControls.map(({ key }) => [key, sanitizeGaitLabSettings(settings)[key]]),
);

const invalidatePreviewAssessment = (state) => {
  state.previewAssessment = { available: false, safe: false, reachableCount: 0, limitedJointCount: 0 };
};

export function createLiveGaitProfile(candidate = {}, fallbackName = "Untitled gait") {
  const settingsSource = candidate.settings && typeof candidate.settings === "object"
    ? candidate.settings
    : candidate;
  const sanitized = sanitizeGaitLabSettings(settingsSource);
  return {
    schemaVersion: LIVE_GAIT_SCHEMA_VERSION,
    robot: "domino-esp32-quadruped",
    name: String(candidate.name || fallbackName).trim().slice(0, 32) || fallbackName,
    updatedAt: candidate.updatedAt !== null && candidate.updatedAt !== undefined && Number.isFinite(Number(candidate.updatedAt))
      ? Number(candidate.updatedAt)
      : null,
    source: ["simulation", "live", "robot", "import"].includes(candidate.source)
      ? candidate.source
      : "simulation",
    settings: {
      enabled: sanitized.enabled,
      preset: sanitized.preset,
      ...editableSettings(sanitized),
    },
  };
}

export function createLiveGaitState(initialProfile = createLiveGaitProfile(defaultGaitLabSettings, "Balanced")) {
  const draft = createLiveGaitProfile(initialProfile, initialProfile.name);
  return {
    draft,
    robotProfile: null,
    previousRobotProfile: null,
    selectedLibraryName: "",
    dirty: false,
    previewEnabled: true,
    previewMode: "trot",
    previewForward: 0.65,
    previewTurn: 0,
    previewAssessment: { available: false, safe: false, reachableCount: 0, limitedJointCount: 0 },
    robotState: "unknown",
    persistentApplySupported: false,
    pendingRequestId: "",
    pendingAction: "",
    status: "Previewing a local draft. Nothing has been sent to the robot.",
  };
}

export function updateLiveGaitDraft(state, patch = {}) {
  if (!state?.draft) return false;
  state.draft = createLiveGaitProfile({
    ...state.draft,
    source: "live",
    settings: { ...state.draft.settings, ...patch, preset: "custom" },
  }, state.draft.name);
  state.dirty = true;
  invalidatePreviewAssessment(state);
  return true;
}

export function selectLiveGaitPreset(state, preset) {
  if (!state || !Object.hasOwn(gaitLabPresets, preset)) return false;
  state.draft = createLiveGaitProfile({
    ...state.draft,
    source: "live",
    settings: { ...gaitLabPresets[preset], enabled: true, preset },
  }, state.draft.name);
  state.dirty = true;
  invalidatePreviewAssessment(state);
  return true;
}

export function replaceLiveGaitDraft(state, candidate, source = "import") {
  if (!state) return false;
  state.draft = createLiveGaitProfile({ ...candidate, source }, candidate?.name || "Imported gait");
  state.dirty = source !== "robot";
  invalidatePreviewAssessment(state);
  return true;
}

export function acceptRobotGaitProfile(state, candidate) {
  if (!state || !candidate) return false;
  const profile = createLiveGaitProfile({ ...candidate, source: "robot" }, "Robot profile");
  const unchanged = state.robotProfile &&
    state.robotProfile.name === profile.name &&
    JSON.stringify(state.robotProfile.settings) === JSON.stringify(profile.settings);
  if (unchanged) return false;
  if (state.robotProfile) state.previousRobotProfile = state.robotProfile;
  state.robotProfile = profile;
  return true;
}

export function liveGaitDiff(state) {
  const robotSettings = state?.robotProfile?.settings;
  return gaitLabControls.map(({ key, label, unit, scale = 1, decimals = 0 }) => {
    const draft = Number(state?.draft?.settings?.[key]);
    const robot = Number(robotSettings?.[key]);
    return {
      key,
      label,
      unit,
      scale,
      decimals,
      draft,
      robot: Number.isFinite(robot) ? robot : null,
      delta: Number.isFinite(robot) ? draft - robot : null,
      changed: Number.isFinite(robot) ? Math.abs(draft - robot) > 1e-9 : null,
    };
  });
}

export function liveGaitRiskAssessment(profile) {
  const settings = createLiveGaitProfile(profile).settings;
  const findings = [];
  if (settings.cadenceHz > 1.8) findings.push({ severity: "warning", message: "Cadence is in the high-speed test range." });
  if (settings.strideMm > 95) findings.push({ severity: "warning", message: "Stride approaches the maximum linkage envelope." });
  if (settings.dutyFactor < 0.58) findings.push({ severity: "warning", message: "Low ground time reduces the support margin." });
  if (settings.bodyHeightMm < 230 || settings.bodyHeightMm > 272) findings.push({ severity: "warning", message: "Body height is near an edge of the validated range." });
  if (settings.responseMs < 100) findings.push({ severity: "warning", message: "Fast response may produce abrupt actuator commands." });
  if (!findings.length) findings.push({ severity: "ok", message: "Settings remain inside the normal bounded tuning envelope." });
  return findings;
}

export function liveGaitCanApply(state) {
  return Boolean(
    state &&
    state.robotState === "disarmed" &&
    state.persistentApplySupported &&
    !state.pendingRequestId,
  );
}

export function updateLiveGaitPreviewAssessment(state, telemetry) {
  if (!state) return false;
  const details = Array.isArray(telemetry?.legDetails) ? telemetry.legDetails : [];
  const reachableCount = details.filter((detail) => detail?.reachable === true).length;
  const limitedJointCount = details.reduce(
    (count, detail) => count + (Array.isArray(detail?.limitedJoints) ? detail.limitedJoints.length : 0),
    0,
  );
  state.previewAssessment = {
    available: details.length === 4,
    safe: details.length === 4 && reachableCount === 4 && limitedJointCount === 0,
    reachableCount,
    limitedJointCount,
  };
  return state.previewAssessment.safe;
}

export function liveGaitCanApplyDraft(state) {
  return liveGaitCanApply(state) && state.previewAssessment?.safe === true;
}

export function createLiveGaitCommand(state, action, requestId, now = Date.now()) {
  if (!state || !["request-profile", "apply-profile", "revert-profile"].includes(action)) return null;
  const command = {
    type: "live-gait-command",
    action,
    requestId: String(requestId),
    timestampMs: now,
  };
  if (action === "apply-profile") {
    command.safety = { requiresDisarmed: true, twoStageApply: true };
    command.profile = createLiveGaitProfile({ ...state.draft, updatedAt: now, source: "live" }, state.draft.name);
  }
  return command;
}

export function liveGaitProfileJson(profile) {
  return `${JSON.stringify(createLiveGaitProfile(profile, profile?.name), null, 2)}\n`;
}

export function parseLiveGaitProfileJson(text) {
  const candidate = JSON.parse(String(text));
  if (candidate?.schemaVersion === undefined && candidate && typeof candidate === "object") {
    return createLiveGaitProfile({ settings: candidate, source: "import" }, "Imported simulation gait");
  }
  if (candidate?.schemaVersion !== LIVE_GAIT_SCHEMA_VERSION) throw new Error("Unsupported gait profile version.");
  if (candidate?.robot !== "domino-esp32-quadruped") throw new Error("This gait profile targets a different robot.");
  return createLiveGaitProfile({ ...candidate, source: "import" }, candidate.name);
}

export function readLiveGaitLibrary(raw) {
  if (!raw || typeof raw !== "object") return {};
  return Object.fromEntries(Object.entries(raw).slice(0, 40).map(([name, candidate]) => [
    String(name).slice(0, 32),
    createLiveGaitProfile(
      candidate?.schemaVersion ? candidate : { name, settings: candidate, source: "simulation" },
      name,
    ),
  ]));
}

export function writeLiveGaitLibrary(library) {
  return Object.fromEntries(Object.entries(library).map(([name, profile]) => [
    name,
    createLiveGaitProfile(profile, name),
  ]));
}
