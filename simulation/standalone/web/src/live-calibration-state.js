import { legs, standServoReference } from "./domino-config.js";
import {
  LIVE_CALIBRATION_JOG_LIMIT_DEG,
  LIVE_CALIBRATION_MAX_SPEED_DEG_PER_SEC,
} from "./live-calibration-protocol.js";

export const LIVE_CALIBRATION_SCHEMA_VERSION = 1;
export const LIVE_CALIBRATION_STORAGE_KEY = "domino-live-calibration-v1";
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
    channel: definition.channel,
    offsetDeg: 0,
    direction: definition.defaultDirection,
    minimumDeg: -45,
    maximumDeg: 45,
  };
}

export function createLiveCalibrationProfile(candidate = {}) {
  const supplied = Array.isArray(candidate.joints) ? candidate.joints : [];
  const suppliedByChannel = new Map(supplied.map((joint) => [Number(joint?.channel), joint]));
  return {
    schemaVersion: LIVE_CALIBRATION_SCHEMA_VERSION,
    robot: "domino-esp32-quadruped",
    savedAt: candidate.savedAt !== null && candidate.savedAt !== undefined && Number.isFinite(Number(candidate.savedAt))
      ? Number(candidate.savedAt)
      : null,
    joints: LIVE_CALIBRATION_JOINTS.map((definition) => {
      const defaults = defaultJointCalibration(definition);
      const source = suppliedByChannel.get(definition.channel) || {};
      const minimumDeg = clamp(finite(source.minimumDeg, defaults.minimumDeg), -90, 89);
      const maximumDeg = clamp(finite(source.maximumDeg, defaults.maximumDeg), minimumDeg + 1, 90);
      return {
        channel: definition.channel,
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
  const joint = state?.profile?.joints?.find((candidate) => candidate.channel === state.selectedChannel);
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
    const jog = joint.channel === state.selectedChannel ? state.jogOffsetDeg : 0;
    angles[joint.channel] += joint.offsetDeg + joint.direction * jog;
  });
  return angles;
}

export function calibrationProfileJson(profile) {
  return `${JSON.stringify(createLiveCalibrationProfile(profile), null, 2)}\n`;
}

export function parseCalibrationProfileJson(text) {
  const candidate = JSON.parse(String(text));
  if (candidate?.schemaVersion !== LIVE_CALIBRATION_SCHEMA_VERSION) {
    throw new Error("Unsupported calibration profile version.");
  }
  if (candidate?.robot !== "domino-esp32-quadruped") {
    throw new Error("This calibration profile is for a different robot.");
  }
  return createLiveCalibrationProfile(candidate);
}

export function createCalibrationBenchCommand(state, action, requestId, now = Date.now()) {
  const allowedActions = ["enter", "exit", "jog", "save-profile"];
  if (!state || !allowedActions.includes(action)) return null;
  const joint = state.profile.joints.find((candidate) => candidate.channel === state.selectedChannel);
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
    jogOffsetDeg: state.jogOffsetDeg,
    targetServoDeg: calibrationPreviewServoAngles(state)[state.selectedChannel],
    profile: action === "save-profile" ? createLiveCalibrationProfile(state.profile) : undefined,
  };
}
