import { legs, standServoReference } from "./domino-config.js";
import {
  LIVE_CALIBRATION_JOG_LIMIT_DEG,
  LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC,
} from "./live-calibration-protocol.js";

export const LIVE_CALIBRATION_SCHEMA_VERSION = 2;
export const LIVE_CALIBRATION_STORAGE_KEY = "domino-live-calibration-v2";
export const LIVE_CALIBRATION_LEGACY_STORAGE_KEY = "domino-live-calibration-v1";
export const LIVE_CALIBRATION_PRE_SYNC_BACKUP_KEY = "domino-live-calibration-before-robot-sync-v1";
export const LIVE_CALIBRATION_LAST_ROBOT_KEY = "domino-live-calibration-last-robot-v1";
export const LIVE_CALIBRATION_CORRUPT_BACKUP_KEY = "domino-live-calibration-unreadable-v1";
export { LIVE_CALIBRATION_JOG_LIMIT_DEG, LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC };

export const LIVE_CALIBRATION_STEPS = Object.freeze([
  Object.freeze({ id: "safety", label: "Safety" }),
  Object.freeze({ id: "select", label: "Select joint" }),
  Object.freeze({ id: "neutral", label: "Set neutral" }),
  Object.freeze({ id: "limits", label: "Set limits" }),
  Object.freeze({ id: "review", label: "Review" }),
]);

const jointLabels = Object.freeze({
  shoulder: "Hip",
  upper: "Upper",
  lower: "Lower",
});

export const LIVE_CALIBRATION_JOINTS = Object.freeze(
  legs.flatMap((leg) => ["shoulder", "upper", "lower"].map((joint) => Object.freeze({
    id: `${leg.label.toLowerCase()}-${joint}`,
    leg: leg.label,
    joint,
    label: `${leg.label} ${jointLabels[joint]}`,
    channel: leg.channels[joint],
    neutralServoDeg: standServoReference[leg.channels[joint]],
    defaultDirection: leg.directions[joint],
  }))).sort((a, b) => a.channel - b.channel),
);

// Physical firmware uses DOMINO_PLANAR_DRIVE_CROSS_REVERSED=1. Its correction
// changes drive deltas, not output channels or electrical neutral values.
// Keep the persisted profile contract above unchanged, and describe the CAD
// joint each existing output actually moves in a separate presentation map.
export const LIVE_CALIBRATION_OUTPUTS = Object.freeze(
  LIVE_CALIBRATION_JOINTS.map((definition) => {
    const leg = legs.find((candidate) => candidate.label === definition.leg);
    const modelJoint = definition.joint === "upper" ? "lower"
      : definition.joint === "lower" ? "upper" : "shoulder";
    const modelChannel = leg.channels[modelJoint];
    return Object.freeze({
      ...definition,
      joint: modelJoint,
      label: `${definition.leg} ${jointLabels[modelJoint]}`,
      modelChannel,
      modelDirection: leg.directions[modelJoint],
      modelPoseSign: modelJoint === "shoulder" ? 1 : -1,
    });
  }),
);

export function calibrationOutputForModelChannel(modelChannel) {
  return LIVE_CALIBRATION_OUTPUTS.find((definition) => definition.modelChannel === Number(modelChannel));
}

export function calibrationInspectionModelAngles(state, poseDeg = state?.jogOffsetDeg ?? 0) {
  const angles = [...standServoReference];
  const definition = LIVE_CALIBRATION_OUTPUTS.find((joint) => joint.channel === state?.selectedChannel);
  if (definition && Number.isFinite(poseDeg)) {
    angles[definition.modelChannel] += definition.modelDirection * definition.modelPoseSign * poseDeg;
  }
  return angles;
}

const finite = (value, fallback) => Number.isFinite(Number(value)) ? Number(value) : fallback;
const clamp = (value, minimum, maximum) => Math.min(maximum, Math.max(minimum, value));

function defaultJointCalibration(definition) {
  const travel = definition.joint === "shoulder" ? 30 : 45;
  return {
    logicalChannel: definition.channel,
    channel: definition.channel,
    offsetDeg: 0,
    direction: definition.defaultDirection,
    minimumDeg: -travel,
    maximumDeg: travel,
  };
}

export function createLiveCalibrationProfile(candidate = {}) {
  const supplied = Array.isArray(candidate.joints) ? candidate.joints : [];
  const suppliedByLogicalChannel = new Map(supplied.map((joint) => [
    Number(joint?.logicalChannel ?? joint?.channel),
    joint,
  ]));
  const requestedMap = LIVE_CALIBRATION_JOINTS.map((definition) => {
    const source = suppliedByLogicalChannel.get(definition.channel);
    return Number(source?.channel ?? definition.channel);
  });
  const validRequestedMap = calibrationChannelMapIssues(requestedMap).length === 0;
  return {
    schemaVersion: LIVE_CALIBRATION_SCHEMA_VERSION,
    robot: "domino-esp32-quadruped",
    savedAt: candidate.savedAt !== null && candidate.savedAt !== undefined && Number.isFinite(Number(candidate.savedAt))
      ? Number(candidate.savedAt)
      : null,
    joints: LIVE_CALIBRATION_JOINTS.map((definition) => {
      const defaults = defaultJointCalibration(definition);
      const source = suppliedByLogicalChannel.get(definition.channel) || {};
      const travel = definition.joint === "shoulder" ? 30 : 90;
      const minimumDeg = clamp(finite(source.minimumDeg, defaults.minimumDeg), -travel, travel - 1);
      const maximumDeg = clamp(finite(source.maximumDeg, defaults.maximumDeg), minimumDeg + 1, travel);
      return {
        logicalChannel: definition.channel,
        channel: validRequestedMap ? Number(source.channel ?? definition.channel) : definition.channel,
        offsetDeg: clamp(finite(source.offsetDeg, defaults.offsetDeg), -30, 30),
        direction: Number(source.direction ?? defaults.direction) === -1 ? -1 : 1,
        minimumDeg,
        maximumDeg,
      };
    }),
  };
}

export function createLiveCalibrationState(profile = createLiveCalibrationProfile()) {
  return {
    step: 0,
    selectedChannel: LIVE_CALIBRATION_JOINTS[0].channel,
    jogOffsetDeg: 0,
    previewEnabled: true,
    safetyConfirmed: false,
    benchModeAcknowledged: false,
    dirty: false,
    profile: createLiveCalibrationProfile(profile),
    robotProfileSessionId: "",
    robotProfileSavedAt: null,
    robotOrigin: null,
    restorableProfile: false,
  };
}

export function forgetRobotCalibrationProfile(state) {
  if (!state) return;
  state.robotProfileSessionId = "";
  state.robotProfileSavedAt = null;
  state.robotOrigin = null;
  state.restorableProfile = false;
}

const DEFAULT_ORIGIN_SOURCES = new Set(["default-empty", "default-fallback"]);
const KNOWN_ORIGIN_SOURCES = new Set([...DEFAULT_ORIGIN_SOURCES, "nvs", "nvs-backup", "nvs-legacy"]);

// Firmware >= 0.8.16 reports which stored record produced the active profile.
// Older firmware sends nothing, in which case `known` is false and a profile
// with savedAt === 0 is still treated as the compiled default.
export function normalizeCalibrationOrigin(origin, profile = null) {
  const source = KNOWN_ORIGIN_SOURCES.has(origin?.source) ? origin.source : "";
  const savedAt = profile?.savedAt ?? origin?.savedAt ?? 0;
  const usingDefaults = source
    ? DEFAULT_ORIGIN_SOURCES.has(source)
    : !(Number.isFinite(Number(savedAt)) && Number(savedAt) > 0);
  return {
    known: Boolean(source),
    source: source || (usingDefaults ? "default-unreported" : "nvs-unreported"),
    usingDefaults,
    detail: typeof origin?.detail === "string" ? origin.detail.slice(0, 96) : "",
  };
}

export function calibrationOriginWarning(state) {
  const origin = state?.robotOrigin;
  if (!origin) return null;
  if (origin.usingDefaults) {
    const why = origin.source === "default-fallback"
      ? `A stored profile exists but could not be read${origin.detail ? ` (${origin.detail})` : ""}.`
      : "No saved profile was found on the robot.";
    return {
      level: "danger",
      title: "ROBOT IS RUNNING DEFAULT CALIBRATION",
      detail: state.restorableProfile
        ? `${why} Your last known Studio profile is loaded as an unsaved draft: review it, then save it to the robot.`
        : `${why} Servo centers, directions and limits are the compiled defaults, not your calibration.`,
    };
  }
  if (origin.source === "nvs-backup") {
    return {
      level: "warning",
      title: "ROBOT RECOVERED ITS PREVIOUS PROFILE",
      detail: `The newest stored profile was unreadable${origin.detail ? ` (${origin.detail})` : ""}; the robot is using its previous backup. Review and save to refresh it.`,
    };
  }
  if (origin.source === "nvs-legacy") {
    return {
      level: "notice",
      title: "ROBOT PROFILE IS IN THE OLD FORMAT",
      detail: "Save the profile once to upgrade it to the versioned format with a backup copy.",
    };
  }
  return null;
}

// `lastKnownGood` is the newest profile previously read from a robot that was
// NOT on defaults. It lets a robot that lost its flash profile be restored
// without trusting an overwritten browser copy.
export function acceptRobotCalibrationProfile(state, profile, sessionId, origin = null, lastKnownGood = null) {
  if (!state || !sessionId) return { accepted: false, adopted: false };
  const active = parseCalibrationProfileJson(JSON.stringify(profile));
  const robotOrigin = normalizeCalibrationOrigin(origin, active);
  state.robotOrigin = robotOrigin;
  state.restorableProfile = false;
  let adopted = !state.dirty;
  let restored = false;
  if (robotOrigin.usingDefaults && !state.dirty) {
    // The robot lost its profile. Never overwrite a previously saved browser
    // copy with defaults; keep it (or the last robot-read copy) as an unsaved
    // draft so one review-and-save restores the robot.
    const localSaved = Number(state.profile?.savedAt) > 0 ? state.profile : null;
    const candidate = localSaved || (lastKnownGood && Number(lastKnownGood.savedAt) > 0 ? lastKnownGood : null);
    if (candidate) {
      state.profile = parseCalibrationProfileJson(JSON.stringify(candidate));
      state.dirty = true;
      state.restorableProfile = true;
      state.jogOffsetDeg = 0;
      adopted = false;
      restored = true;
    }
  }
  if (adopted) {
    state.profile = active;
    state.jogOffsetDeg = 0;
  }
  state.robotProfileSessionId = sessionId;
  state.robotProfileSavedAt = active.savedAt ?? 0;
  return { accepted: true, adopted, restored, robotUsingDefaults: robotOrigin.usingDefaults };
}

export function robotCalibrationReady(state, sessionId) {
  return Boolean(state && sessionId && state.robotProfileSessionId === sessionId &&
    Number.isSafeInteger(state.robotProfileSavedAt) && state.robotProfileSavedAt >= 0);
}

export function selectCalibrationStep(state, step) {
  const namedStep = typeof step === "string"
    ? LIVE_CALIBRATION_STEPS.findIndex((candidate) => candidate.id === step)
    : -1;
  const next = namedStep >= 0 ? namedStep : Math.floor(Number(step));
  if (!state || !Number.isInteger(next) || next < 0 || next >= LIVE_CALIBRATION_STEPS.length) return false;
  state.step = next;
  return true;
}

export function selectCalibrationJoint(state, channel) {
  const numericChannel = Number(channel);
  if (!state || !LIVE_CALIBRATION_JOINTS.some((joint) => joint.channel === numericChannel)) return false;
  state.selectedChannel = numericChannel;
  state.jogOffsetDeg = 0;
  return true;
}

export function updateCalibrationJoint(state, patch = {}) {
  const joint = state?.profile?.joints?.find((candidate) => candidate.logicalChannel === state.selectedChannel);
  if (!joint) return false;
  const definition = LIVE_CALIBRATION_JOINTS.find((candidate) => candidate.channel === joint.logicalChannel);
  const travel = definition?.joint === "shoulder" ? 30 : 90;
  const minimumDeg = clamp(finite(patch.minimumDeg, joint.minimumDeg), -travel, travel - 1);
  const maximumDeg = clamp(finite(patch.maximumDeg, joint.maximumDeg), minimumDeg + 1, travel);
  joint.offsetDeg = clamp(finite(patch.offsetDeg, joint.offsetDeg), -30, 30);
  joint.direction = Number(patch.direction ?? joint.direction) === -1 ? -1 : 1;
  joint.minimumDeg = minimumDeg;
  joint.maximumDeg = maximumDeg;
  state.dirty = true;
  return true;
}

export function restoreSelectedCalibrationJoint(state) {
  const definition = LIVE_CALIBRATION_JOINTS.find((joint) => joint.channel === state?.selectedChannel);
  const joint = state?.profile?.joints?.find((candidate) => candidate.logicalChannel === state.selectedChannel);
  if (!definition || !joint) return false;
  const physicalChannel = joint.channel;
  Object.assign(joint, defaultJointCalibration(definition), { channel: physicalChannel });
  state.jogOffsetDeg = 0;
  state.dirty = true;
  return true;
}

export function restoreCalibrationDefaults(state, restoreChannelMap = false) {
  if (!state?.profile) return false;
  const physicalChannels = calibrationChannelMap(state.profile);
  state.profile = createLiveCalibrationProfile();
  if (!restoreChannelMap) {
    state.profile.joints.forEach((joint, index) => { joint.channel = physicalChannels[index]; });
  }
  state.jogOffsetDeg = 0;
  state.dirty = true;
  return true;
}

export function jogCalibrationJoint(state, incrementDeg) {
  if (!state) return false;
  const increment = finite(incrementDeg, 0);
  state.jogOffsetDeg = clamp(
    state.jogOffsetDeg + increment,
    -LIVE_CALIBRATION_JOG_LIMIT_DEG,
    LIVE_CALIBRATION_JOG_LIMIT_DEG,
  );
  return true;
}

export function trimCalibrationJoint(state, incrementDeg) {
  const joint = state?.profile?.joints?.find(
    (candidate) => candidate.logicalChannel === state.selectedChannel,
  );
  if (!joint) return false;
  const increment = finite(incrementDeg, 0);
  joint.offsetDeg = clamp(
    Math.round((joint.offsetDeg + increment) * 10) / 10,
    -30,
    30,
  );
  state.jogOffsetDeg = 0;
  state.dirty = true;
  return true;
}

export function calibrationPreviewServoAngles(state) {
  // Electrical targets include the trim: the physical servo must move so its
  // new center matches the unchanged CAD neutral pose.
  const angles = [...standServoReference];
  state?.profile?.joints?.forEach((joint) => {
    const jog = joint.logicalChannel === state.selectedChannel ? state.jogOffsetDeg : 0;
    angles[joint.logicalChannel] += joint.offsetDeg + joint.direction * jog;
  });
  return angles;
}

export function calibrationPreviewModelAngles(state) {
  // Saved trim and shaft direction define the physical output basis. They do
  // not change the robot geometry or the center shown by the CAD preview.
  return calibrationInspectionModelAngles(state);
}

export function calibrationProfileJson(profile) {
  return `${JSON.stringify(createLiveCalibrationProfile(profile), null, 2)}\n`;
}

export function parseCalibrationProfileJson(text) {
  const candidate = JSON.parse(String(text));
  if (![1, LIVE_CALIBRATION_SCHEMA_VERSION].includes(candidate?.schemaVersion)) {
    throw new Error("Unsupported calibration profile version.");
  }
  if (candidate?.robot !== "domino-esp32-quadruped") {
    throw new Error("This calibration profile is for a different robot.");
  }
  if (!Array.isArray(candidate.joints) || candidate.joints.length !== LIVE_CALIBRATION_JOINTS.length) {
    throw new Error("Calibration profile must contain all 12 logical joints.");
  }
  const requestedMap = LIVE_CALIBRATION_JOINTS.map((definition) => {
    const source = candidate.joints.find((joint) =>
      Number(joint?.logicalChannel ?? joint?.channel) === definition.channel,
    );
    return Number(source?.channel);
  });
  const issues = calibrationChannelMapIssues(requestedMap);
  if (issues.length > 0) throw new Error(issues[0]);
  return createLiveCalibrationProfile(candidate);
}

export function calibrationChannelMap(profile) {
  return LIVE_CALIBRATION_JOINTS.map((definition) => {
    const joint = profile?.joints?.find((candidate) => candidate.logicalChannel === definition.channel);
    return Number(joint?.channel ?? definition.channel);
  });
}

export function calibrationChannelMapIssues(channelMap) {
  if (!Array.isArray(channelMap) || channelMap.length !== LIVE_CALIBRATION_JOINTS.length) {
    return ["Assign one physical channel to each of the 12 logical joints."];
  }
  const channels = channelMap.map(Number);
  const invalid = channels.find((channel) => !Number.isInteger(channel) || channel < 0 || channel > 15);
  if (invalid !== undefined) return ["Servo channels must be whole numbers from 0 to 15."];
  const duplicates = [...new Set(channels.filter((channel, index) => channels.indexOf(channel) !== index))];
  if (duplicates.length > 0) {
    return [`Each servo needs its own output. Channel ${duplicates.join(", ")} is assigned more than once.`];
  }
  return [];
}

export function updateCalibrationChannelMap(state, channelMap) {
  const issues = calibrationChannelMapIssues(channelMap);
  if (!state?.profile || issues.length > 0) return { accepted: false, issues };
  const previousSelected = state.profile.joints.find(
    (joint) => joint.logicalChannel === state.selectedChannel,
  );
  LIVE_CALIBRATION_JOINTS.forEach((definition, index) => {
    const joint = state.profile.joints.find(
      (candidate) => candidate.logicalChannel === definition.channel,
    );
    joint.channel = Number(channelMap[index]);
  });
  state.selectedChannel = previousSelected?.logicalChannel ?? LIVE_CALIBRATION_JOINTS[0].channel;
  state.jogOffsetDeg = 0;
  state.dirty = true;
  return { accepted: true, issues: [] };
}

export function createCalibrationBenchCommand(state, action, requestId, now = Date.now(), smoothing = null) {
  const allowedActions = ["request-profile", "request-smoothing", "save-smoothing", "enter", "exit", "jog", "save-profile"];
  if (!state || !allowedActions.includes(action)) return null;
  if (action === "save-profile" && !robotCalibrationReady(state, state.robotProfileSessionId)) return null;
  const joint = state.profile.joints.find((candidate) => candidate.logicalChannel === state.selectedChannel);
  if (!joint) return null;
  const persistedProfile = action === "save-profile"
    ? createLiveCalibrationProfile({ ...state.profile, savedAt: Math.max(now, state.robotProfileSavedAt + 1) })
    : undefined;
  return {
    type: "live-calibration-command",
    action,
    requestId: String(requestId),
    timestampMs: now,
    safety: {
      benchModeRequired: !["request-profile", "request-smoothing", "save-smoothing"].includes(action),
      maxSpeedDegPerSec: LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC,
      jogLimitDeg: LIVE_CALIBRATION_JOG_LIMIT_DEG,
    },
    selectedChannel: state.selectedChannel,
    physicalChannel: joint.channel,
    jogOffsetDeg: state.jogOffsetDeg,
    targetServoDeg: calibrationPreviewServoAngles(state)[state.selectedChannel],
    ...(action === "save-profile" ? { baseSavedAt: state.robotProfileSavedAt } : {}),
    ...(action === "save-smoothing" && smoothing ? {
      baseUpdatedAt: smoothing.baseUpdatedAt,
      smoothing: smoothing.settings,
    } : {}),
    profile: persistedProfile,
  };
}
