import { legs, standServoReference } from "./domino-config.js";
import {
  LIVE_CALIBRATION_JOG_LIMIT_DEG,
  LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC,
} from "./live-calibration-protocol.js";

export const LIVE_CALIBRATION_SCHEMA_VERSION = 2;
export const LIVE_CALIBRATION_STORAGE_KEY = "domino-live-calibration-v2";
export const LIVE_CALIBRATION_LEGACY_STORAGE_KEY = "domino-live-calibration-v1";
export const LIVE_CALIBRATION_PRE_SYNC_BACKUP_KEY = "domino-live-calibration-before-robot-sync-v1";
export const LIVE_CALIBRATION_DRAFT_STORAGE_KEY = "domino-live-calibration-draft-v1";
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
  // Preserve complete, validated profiles exactly, including valid firmware
  // travel limits wider than the editor's conservative hip bounds.
  if (candidate.schemaVersion && candidate.robot && Array.isArray(candidate.joints)) {
    try { return parseCalibrationProfileJson(JSON.stringify(candidate)); } catch { /* Sanitize internal candidates below. */ }
  }
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
    robotProfile: null,
    savePhase: "idle",
    saveExpectedProfile: null,
    saveDraftProfile: null,
  };
}

export function forgetRobotCalibrationProfile(state) {
  if (!state) return;
  state.robotProfileSessionId = "";
  state.robotProfileSavedAt = null;
}

export function resetLiveCalibrationConnection(state) {
  if (!state) return;
  state.benchModeAcknowledged = false;
  state.jogOffsetDeg = 0;
  if (["sending", "verifying"].includes(state.savePhase)) state.savePhase = "unconfirmed";
  forgetRobotCalibrationProfile(state);
}

export function saveLocalCalibrationProfile(state, now = Date.now()) {
  state.profile.savedAt = now;
  // A browser backup does not make edits synchronized with robot NVS.
  // Keep the dirty flag so reconnect readback cannot replace this draft.
  return calibrationProfileJson(state.profile);
}

export function acceptRobotCalibrationProfile(state, profile, sessionId) {
  if (!state || !sessionId) return { accepted: false, adopted: false };
  const active = parseCalibrationProfileJson(JSON.stringify(profile));
  const adopted = !state.dirty;
  if (adopted) {
    state.profile = active;
    state.jogOffsetDeg = 0;
  }
  state.robotProfileSessionId = sessionId;
  state.robotProfileSavedAt = active.savedAt ?? 0;
  state.robotProfile = active;
  return { accepted: true, adopted };
}

// ESP32 stores angles as float32. Compare their values, rather than serialized
// JSON text, without treating a different revision or channel map as a match.
export function calibrationProfilesMatch(left, right, includeSavedAt = true) {
  if (!left || !right || left.schemaVersion !== right.schemaVersion || left.robot !== right.robot ||
      (includeSavedAt && left.savedAt !== right.savedAt) ||
      !Array.isArray(left.joints) || !Array.isArray(right.joints) ||
      left.joints.length !== 12 || right.joints.length !== 12) return false;
  return left.joints.every((joint) => {
    const other = right.joints.find((candidate) => candidate.logicalChannel === joint.logicalChannel);
    return other && joint.channel === other.channel && joint.direction === other.direction &&
      ["offsetDeg", "minimumDeg", "maximumDeg"].every((key) =>
        Number.isFinite(joint[key]) && Number.isFinite(other[key]) &&
        Math.abs(joint[key] - other[key]) < 0.0001);
  });
}

export function beginRobotCalibrationSave(state, profile) {
  state.saveDraftProfile = createLiveCalibrationProfile(state.profile);
  state.saveExpectedProfile = createLiveCalibrationProfile(profile);
  state.savePhase = "sending";
  state.dirty = true;
}

export function acknowledgeRobotCalibrationSave(state, accepted, persisted) {
  state.benchModeAcknowledged = false;
  state.dirty = true;
  state.savePhase = accepted && persisted && state.saveExpectedProfile ? "verifying" : "rejected";
  forgetRobotCalibrationProfile(state);
}

export function acceptCalibrationSaveReadback(state, profile, sessionId) {
  const active = parseCalibrationProfileJson(JSON.stringify(profile));
  if (!state.saveExpectedProfile || !sessionId) return { accepted: false, matched: false, adopted: false };
  const matched = calibrationProfilesMatch(active, state.saveExpectedProfile);
  const draftUnchanged = calibrationProfilesMatch(state.profile, state.saveDraftProfile, false);
  // Keep the imported draft through the compact save ACK, readback, retries and
  // reconnects. Edits made while the request was in flight belong to the user.
  state.dirty = true;
  acceptRobotCalibrationProfile(state, active, sessionId);
  const adopted = matched && draftUnchanged;
  if (adopted) {
    state.profile = active;
    state.jogOffsetDeg = 0;
    state.dirty = false;
  }
  state.savePhase = matched ? "verified" : "mismatch";
  if (matched) {
    state.saveExpectedProfile = null;
    state.saveDraftProfile = null;
  } else {
    forgetRobotCalibrationProfile(state);
  }
  return { accepted: true, matched, adopted };
}

export function calibrationDraftJson(state) {
  return state?.dirty ? calibrationProfileJson(state.profile) : null;
}

export function restoreCalibrationDraft(state, contents) {
  if (!contents) return false;
  state.profile = parseCalibrationProfileJson(contents);
  state.dirty = true;
  return true;
}

export function robotCalibrationReady(state, sessionId) {
  return Boolean(state && sessionId && state.robotProfileSessionId === sessionId &&
    Number.isSafeInteger(state.robotProfileSavedAt) && state.robotProfileSavedAt >= 0);
}

export function calibrationControlBlockReason(state, context, action = "enter") {
  if (!context.connectionReady) return "Connect the robot before requesting physical calibration.";
  if (!Number.isFinite(context.telemetryAgeMs) || context.telemetryAgeMs > 1000 || context.telemetryAgeMs < 0) {
    return "Waiting for fresh robot telemetry.";
  }
  if (context.robotState !== "disarmed") return "Disarm the robot before physical calibration.";
  if (!context.calibrationSupported) return "This robot adapter does not support calibration.";
  if (!robotCalibrationReady(state, context.sessionId)) return "Read the active robot profile before physical calibration.";
  if (!state.safetyConfirmed) return "Confirm that the robot is supported before requesting bench mode.";
  if (!context.workspaceActive) return "Open the LIVE calibration page to request physical calibration.";
  if (context.batteryLocked) return "Robot battery is too low for physical calibration.";
  if (context.pending) return "Waiting for the current robot calibration request.";
  if (action === "enter" && state.benchModeAcknowledged) return "Bench mode is already acknowledged.";
  if (action !== "enter" && !state.benchModeAcknowledged) return "Request bench mode before sending physical calibration commands.";
  return "";
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
  const joint = state.profile?.joints?.find(candidate => candidate.logicalChannel === state.selectedChannel);
  if (!joint) return false;
  const minimum = Math.max(-LIVE_CALIBRATION_JOG_LIMIT_DEG, joint.minimumDeg);
  const maximum = Math.min(LIVE_CALIBRATION_JOG_LIMIT_DEG, joint.maximumDeg);
  if (minimum > maximum) return false;
  const increment = finite(incrementDeg, 0);
  state.jogOffsetDeg = clamp(
    state.jogOffsetDeg + increment,
    minimum,
    maximum,
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
  if (candidate.savedAt !== null && candidate.savedAt !== undefined &&
      (!Number.isSafeInteger(candidate.savedAt) || candidate.savedAt < 0)) {
    throw new Error("Calibration revision must be a nonnegative whole number.");
  }
  const joints = LIVE_CALIBRATION_JOINTS.map(definition => {
    const joint = candidate.joints.find(source => (source.logicalChannel ?? source.channel) === definition.channel);
    if (!joint || !Number.isInteger(joint.channel) ||
        (joint.direction !== -1 && joint.direction !== 1) ||
        ![joint.offsetDeg, joint.minimumDeg, joint.maximumDeg].every(Number.isFinite) ||
        Math.abs(joint.offsetDeg) > 30 || joint.minimumDeg < -90 || joint.minimumDeg > 89 ||
        joint.maximumDeg > 90 || joint.maximumDeg < joint.minimumDeg + 1) {
      throw new Error(`Invalid calibration values for ${definition.label}. Values were not imported.`);
    }
    return { logicalChannel: definition.channel, channel: joint.channel, offsetDeg: joint.offsetDeg,
      direction: joint.direction, minimumDeg: joint.minimumDeg, maximumDeg: joint.maximumDeg };
  });
  return { schemaVersion: LIVE_CALIBRATION_SCHEMA_VERSION, robot: candidate.robot,
    savedAt: candidate.savedAt ?? null, joints };
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
  const allowedActions = ["request-profile", "request-smoothing", "save-smoothing", "enter", "exit", "jog", "save-profile", "keepalive"];
  if (!state || !allowedActions.includes(action)) return null;
  if (action === "save-profile" && !robotCalibrationReady(state, state.robotProfileSessionId)) return null;
  const joint = state.profile.joints.find((candidate) => candidate.logicalChannel === state.selectedChannel);
  if (!joint) return null;
  if (action === "jog" && (state.jogOffsetDeg < joint.minimumDeg || state.jogOffsetDeg > joint.maximumDeg)) return null;
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
