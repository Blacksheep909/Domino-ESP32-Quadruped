import { defaultGaitLabSettings, sanitizeGaitLabSettings } from "./gait-lab.js";
import {
  DEFAULT_GAMEPAD_MAPPING,
  readGamepadMappings,
  sanitizeGamepadMapping,
} from "./gamepad-profile.js";
import {
  createLiveGaitProfile,
} from "./live-gait-state.js";
import {
  createLiveCalibrationProfile,
  parseCalibrationProfileJson,
} from "./live-calibration-state.js";
import {
  createLiveNavigationState,
  parseNavigationMissionJson,
} from "./live-navigation-state.js";
import {
  MAX_NAVIGATION_PLAN_LIBRARY_ENTRIES,
  navigationPlanLibraryJson,
  parseNavigationPlanLibraryJson,
} from "./navigation-plan-library.js";

export const PROJECT_BUNDLE_SCHEMA_VERSION = 1;
export const PROJECT_BUNDLE_TYPE = "domino-quadruped-project";
export const PROJECT_ROBOT_ID = "domino-esp32-quadruped";
export const PROJECT_NAME_MAX_LENGTH = 80;

const MAX_GAIT_PROFILES = 40;
const MAX_GAMEPAD_MAPPINGS = 32;
const DEFAULT_PROJECT_NAME = "Domino V2";

const boundedNumber = (value, fallback, minimum, maximum) => {
  const number = Number(value);
  return Number.isFinite(number) ? Math.max(minimum, Math.min(maximum, number)) : fallback;
};

export const DOMINO_PROJECT_ROBOT = Object.freeze({
  id: PROJECT_ROBOT_ID,
  name: "Domino V2",
  type: "quadruped",
  legs: 4,
  jointsPerLeg: 3,
  degreesOfFreedom: 12,
  servoOutputs: 16,
  model: "Domino CAD / ESP32 / PCA9685",
});

function finiteTimestamp(value, fallback = null) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
}

function clonePlain(value) {
  return JSON.parse(JSON.stringify(value));
}

export function sanitizeProjectName(value, fallback = DEFAULT_PROJECT_NAME) {
  const name = String(value ?? "")
    .replace(/[\u0000-\u001f\u007f]/g, " ")
    .replace(/\s+/g, " ")
    .trim()
    .slice(0, PROJECT_NAME_MAX_LENGTH);
  return name || fallback;
}

function sanitizeGaitProfiles(raw) {
  if (!raw || typeof raw !== "object" || Array.isArray(raw)) return {};
  return Object.fromEntries(
    Object.entries(raw)
      .filter(([name, settings]) => typeof name === "string" && name.trim() && settings && typeof settings === "object")
      .slice(0, MAX_GAIT_PROFILES)
      .map(([name, settings]) => [
        name.trim().slice(0, 32),
        sanitizeGaitLabSettings(settings.schemaVersion && settings.settings ? settings.settings : settings),
      ]),
  );
}

function sanitizeGamepadMappingLibrary(raw) {
  if (!raw || typeof raw !== "object" || Array.isArray(raw)) return {};
  return Object.fromEntries(
    Object.entries(raw)
      .slice(0, MAX_GAMEPAD_MAPPINGS)
      .map(([id, mapping]) => [
        String(id).slice(0, 160),
        sanitizeGamepadMapping(mapping || DEFAULT_GAMEPAD_MAPPING),
      ]),
  );
}

function requireObject(value, message) {
  if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error(message);
  return value;
}

function normalizeCalibrationProfile(candidate) {
  const profile = requireObject(candidate, "Project bundle is missing its calibration profile.");
  if (!Array.isArray(profile.joints) || profile.joints.length !== DOMINO_PROJECT_ROBOT.degreesOfFreedom) {
    throw new Error("Project calibration must contain all 12 logical joints.");
  }
  // Use the calibration parser's strict channel-map validation. Project
  // imports should never silently repair a user-authored physical mapping.
  return parseCalibrationProfileJson(JSON.stringify(profile));
}

function normalizeLiveGaitLibrary(raw) {
  const library = requireObject(raw, "Project bundle is missing its LIVE gait library.");
  return Object.fromEntries(
    Object.entries(library)
      .slice(0, MAX_GAIT_PROFILES)
      .map(([name, candidate]) => [
        String(name).slice(0, 32),
        createLiveGaitProfile(
          candidate?.settings && typeof candidate.settings === "object"
            ? { ...candidate, name: candidate.name || name }
            : { name, settings: candidate },
          name,
        ),
      ]),
  );
}

function normalizeLiveGaitDraft(raw) {
  return createLiveGaitProfile(
    requireObject(raw, "Project bundle is missing its LIVE gait draft."),
    "Imported gait",
  );
}

function normalizeNavigationPlan(raw) {
  const defaults = createLiveNavigationState();
  const source = raw && typeof raw === "object" && !Array.isArray(raw) ? raw : {};
  const parsed = parseNavigationMissionJson(JSON.stringify({
    name: source.name ?? source.missionName ?? defaults.missionName,
    mission: Array.isArray(source.mission) ? source.mission : Array.isArray(source.missionDraft) ? source.missionDraft : [],
    loopCount: source.loopCount ?? defaults.loopCount,
    plannerOrigin: source.plannerOrigin || null,
    plannerRangeM: source.plannerRangeM ?? defaults.plannerRangeM,
    geofence: source.geofence || null,
    obstacleBehavior: source.obstacleBehavior || null,
  }));
  const geofenceSource = source.geofence && typeof source.geofence === "object" ? source.geofence : {};
  const obstacleSource = source.obstacleBehavior && typeof source.obstacleBehavior === "object"
    ? source.obstacleBehavior
    : {};
  return {
    missionName: parsed.name,
    missionDraft: parsed.mission,
    loopCount: parsed.loopCount,
    plannerOrigin: parsed.plannerOrigin,
    plannerRangeM: parsed.plannerRangeM,
    geofence: {
      enabled: geofenceSource.enabled === true,
      maxRadiusM: boundedNumber(geofenceSource.maxRadiusM, defaults.geofence.maxRadiusM, 5, 500),
    },
    obstacleBehavior: {
      enabled: obstacleSource.enabled !== false,
      stopDistanceM: boundedNumber(obstacleSource.stopDistanceM, defaults.obstacleBehavior.stopDistanceM, 0.1, 5),
      slowDistanceM: boundedNumber(obstacleSource.slowDistanceM, defaults.obstacleBehavior.slowDistanceM, 0.2, 10),
      maxSpeedMps: boundedNumber(obstacleSource.maxSpeedMps, defaults.obstacleBehavior.maxSpeedMps, 0.05, 5),
    },
  };
}

function normalizeNavigationPlanLibrary(raw) {
  if (raw === undefined || raw === null) return [];
  if (!Array.isArray(raw) || raw.length > MAX_NAVIGATION_PLAN_LIBRARY_ENTRIES) {
    throw new Error(`Project route library must contain up to ${MAX_NAVIGATION_PLAN_LIBRARY_ENTRIES} saved plans.`);
  }
  return parseNavigationPlanLibraryJson(navigationPlanLibraryJson(raw));
}

function normalizeRobot(candidate) {
  const robot = requireObject(candidate, "Project bundle is missing its robot definition.");
  if (robot.id !== PROJECT_ROBOT_ID) throw new Error("This project targets a different robot.");
  if (robot.type !== DOMINO_PROJECT_ROBOT.type || Number(robot.degreesOfFreedom) !== DOMINO_PROJECT_ROBOT.degreesOfFreedom) {
    throw new Error("This project is not a compatible 12-joint quadruped project.");
  }
  return { ...DOMINO_PROJECT_ROBOT };
}

function normalizeBundle(candidate, now = Date.now()) {
  const source = requireObject(candidate, "Project bundle must be a JSON object.");
  if (source.type !== PROJECT_BUNDLE_TYPE) throw new Error("This file is not a Domino project bundle.");
  if (source.schemaVersion !== PROJECT_BUNDLE_SCHEMA_VERSION) {
    throw new Error(`Unsupported project bundle version: ${source.schemaVersion ?? "unknown"}.`);
  }

  const project = requireObject(source.project, "Project bundle is missing project metadata.");
  const simulation = requireObject(source.simulation, "Project bundle is missing simulation settings.");
  const controller = requireObject(source.controller, "Project bundle is missing controller settings.");
  const live = requireObject(source.live, "Project bundle is missing LIVE settings.");
  const projectId = String(project.id || "domino-v2").trim();
  if (projectId !== "domino-v2") throw new Error("This project has an unsupported project id.");

  const calibration = normalizeCalibrationProfile(live.calibration || source.calibration);
  const gaitProfiles = sanitizeGaitProfiles(simulation.gaitProfiles);
  const liveGaitLibrary = normalizeLiveGaitLibrary(live.gaitLibrary);
  const liveGaitDraft = normalizeLiveGaitDraft(live.gaitDraft);
  const navigation = normalizeNavigationPlan(live.navigation);
  const navigationPlanLibrary = normalizeNavigationPlanLibrary(live.navigationPlanLibrary);
  const exportedAt = finiteTimestamp(source.exportedAt, finiteTimestamp(project.updatedAt, now));

  return {
    schemaVersion: PROJECT_BUNDLE_SCHEMA_VERSION,
    type: PROJECT_BUNDLE_TYPE,
    exportedAt,
    project: {
      id: "domino-v2",
      name: sanitizeProjectName(project.name),
    },
    robot: normalizeRobot(source.robot),
    simulation: {
      gaitLabSettings: sanitizeGaitLabSettings(simulation.gaitLabSettings || defaultGaitLabSettings),
      gaitProfiles,
    },
    controller: {
      gamepadMappings: readGamepadMappings(sanitizeGamepadMappingLibrary(controller.gamepadMappings)),
    },
    live: {
      calibration,
      gaitLibrary: liveGaitLibrary,
      gaitDraft: liveGaitDraft,
      navigation,
      navigationPlanLibrary,
    },
  };
}

export function createDominoProjectBundle({
  name = DEFAULT_PROJECT_NAME,
  exportedAt = Date.now(),
  gaitLabSettings = defaultGaitLabSettings,
  gaitProfiles = {},
  gamepadMappings = {},
  calibrationProfile = {},
  liveGaitLibrary = {},
  liveGaitDraft = createLiveGaitProfile(defaultGaitLabSettings, "Balanced"),
  navigationPlan = null,
  navigationPlanLibrary = [],
} = {}) {
  return normalizeBundle({
    schemaVersion: PROJECT_BUNDLE_SCHEMA_VERSION,
    type: PROJECT_BUNDLE_TYPE,
    exportedAt,
    project: { id: "domino-v2", name },
    robot: DOMINO_PROJECT_ROBOT,
    simulation: {
      gaitLabSettings,
      gaitProfiles,
    },
    controller: { gamepadMappings },
    live: {
      calibration: createLiveCalibrationProfile(calibrationProfile),
      gaitLibrary: liveGaitLibrary,
      gaitDraft: liveGaitDraft,
      navigation: navigationPlan,
      navigationPlanLibrary,
    },
  }, exportedAt);
}

export function projectBundleJson(bundle) {
  return `${JSON.stringify(normalizeBundle(bundle), null, 2)}\n`;
}

export function parseProjectBundleJson(text) {
  let candidate;
  try {
    candidate = JSON.parse(String(text));
  } catch {
    throw new Error("Project file is not valid JSON.");
  }
  return normalizeBundle(candidate);
}

export function projectBundleSummary(bundle) {
  const normalized = normalizeBundle(bundle);
  return {
    name: normalized.project.name,
    gaitProfileCount: Object.keys(normalized.simulation.gaitProfiles).length,
    liveGaitProfileCount: Object.keys(normalized.live.gaitLibrary).length,
    gamepadMappingCount: Object.keys(normalized.controller.gamepadMappings).length,
    calibratedJointCount: normalized.live.calibration.joints.length,
    routeWaypointCount: normalized.live.navigation.missionDraft.length,
    routeLibraryCount: normalized.live.navigationPlanLibrary.length,
  };
}

export function projectBundleFileName(name = DEFAULT_PROJECT_NAME) {
  const slug = sanitizeProjectName(name, DEFAULT_PROJECT_NAME)
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-+|-+$/g, "") || "domino-v2";
  return `${slug}.qstudio.json`;
}

export function cloneProjectBundle(bundle) {
  return clonePlain(normalizeBundle(bundle));
}
