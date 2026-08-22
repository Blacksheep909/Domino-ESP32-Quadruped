import { legs, standServoReference } from "./domino-config.js";
import {
  LIVE_CALIBRATION_JOG_LIMIT_DEG,
  LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC,
} from "./live-calibration-protocol.js";

export const LIVE_CALIBRATION_SCHEMA_VERSION = 2;
export const LIVE_CALIBRATION_STORAGE_KEY = "domino-live-calibration-v2";
export const LIVE_CALIBRATION_LEGACY_STORAGE_KEY = "domino-live-calibration-v1";
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

const finite = (value, fallback) => Number.isFinite(Number(value)) ? Number(value) : fallback;
const clamp = (value, minimum, maximum) => Math.min(maximum, Math.max(minimum, value));

function defaultJointCalibration(definition) {
  return {
    logicalChannel: definition.channel,
    channel: definition.channel,
    offsetDeg: 0,
    direction: definition.defaultDirection,
    minimumDeg: -45,
    maximumDeg: 45,
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
      const minimumDeg = clamp(finite(source.minimumDeg, defaults.minimumDeg), -90, 89);
      const maximumDeg = clamp(finite(source.maximumDeg, defaults.maximumDeg), minimumDeg + 1, 90);
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
  };
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
  const minimumDeg = clamp(finite(patch.minimumDeg, joint.minimumDeg), -90, 89);
  const maximumDeg = clamp(finite(patch.maximumDeg, joint.maximumDeg), minimumDeg + 1, 90);
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

export function calibrationPreviewServoAngles(state) {
  const angles = [...standServoReference];
  state?.profile?.joints?.forEach((joint) => {
    const jog = joint.logicalChannel === state.selectedChannel ? state.jogOffsetDeg : 0;
    angles[joint.logicalChannel] += joint.offsetDeg + joint.direction * jog;
  });
  return angles;
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

export function createCalibrationBenchCommand(state, action, requestId, now = Date.now()) {
  const allowedActions = ["enter", "exit", "jog", "save-profile"];
  if (!state || !allowedActions.includes(action)) return null;
  const joint = state.profile.joints.find((candidate) => candidate.logicalChannel === state.selectedChannel);
  if (!joint) return null;
  return {
    type: "live-calibration-command",
    action,
    requestId: String(requestId),
    timestampMs: now,
    safety: {
      benchModeRequired: true,
      maxSpeedDegPerSec: LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC,
      jogLimitDeg: LIVE_CALIBRATION_JOG_LIMIT_DEG,
    },
    selectedChannel: state.selectedChannel,
    physicalChannel: joint.channel,
    jogOffsetDeg: state.jogOffsetDeg,
    targetServoDeg: calibrationPreviewServoAngles(state)[state.selectedChannel],
    profile: action === "save-profile" ? createLiveCalibrationProfile(state.profile) : undefined,
  };
}
