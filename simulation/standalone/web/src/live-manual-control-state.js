import {
  LIVE_MANUAL_MAX_LEASE_MS,
  LIVE_MANUAL_TIMEOUT_MS,
  validLiveManualAuthorityAcknowledgement,
} from "./live-manual-control-protocol.js";

const clampAxis = (value) => Math.max(-1, Math.min(1, Number(value) || 0));

export function createLiveManualControlState() {
  return {
    supported: false,
    safetyConfirmed: false,
    phase: "unavailable",
    authorityToken: "",
    authorityExpiresAt: 0,
    pendingRequestId: "",
    pendingAction: "",
    deadmanActive: false,
    frameSequence: 0,
    mode: "stand",
    axes: { forward: 0, turn: 0, roll: 0, pitch: 0, yaw: 0, bodyX: 0, bodyY: 0, height: 0 },
    status: "Connect an armed, compatible robot to request browser control.",
  };
}

export function setLiveManualSupport(state, supported) {
  if (!state) return false;
  state.supported = Boolean(supported);
  if (!state.supported) revokeLiveManualControl(state, "The selected adapter does not advertise guarded manual control.");
  else if (state.phase === "unavailable") state.phase = "available";
  return true;
}

export function liveManualCanRequest(state, context) {
  return Boolean(
    state?.supported &&
    state.safetyConfirmed &&
    !state.pendingRequestId &&
    !state.authorityToken &&
    context?.connectionReady === true &&
    context?.robotState === "armed" &&
    context?.telemetryFresh === true &&
    context?.controllerLinkReady === true &&
    context?.workspaceActive === true
  );
}

export function createLiveManualAuthorityCommand(state, action, connection, requestId, now = Date.now()) {
  if (!state || !connection || state.pendingRequestId) return null;
  const command = {
    type: "live-manual-authority-command",
    action,
    requestId: String(requestId),
    timestampMs: now,
    ...connection,
  };
  if (action === "request-authority") {
    command.requestedLeaseMs = LIVE_MANUAL_MAX_LEASE_MS;
    command.safety = {
      requiresArmed: true,
      requiresDeadman: true,
      neutralOnRelease: true,
      commandTimeoutMs: LIVE_MANUAL_TIMEOUT_MS,
    };
  } else if (action === "release-authority" && state.authorityToken) {
    command.authorityToken = state.authorityToken;
  } else return null;
  return command;
}

export function markLiveManualPending(state, command) {
  if (!state || !command) return false;
  state.pendingRequestId = command.requestId;
  state.pendingAction = command.action;
  state.phase = command.action === "request-authority" ? "requesting" : "releasing";
  state.status = command.action === "request-authority"
    ? "Waiting for the robot to grant a time-limited manual-control lease..."
    : "Releasing manual-control authority and commanding neutral...";
  return true;
}

export function acceptLiveManualAuthorityAcknowledgement(state, message, connection, receivedAt = Date.now()) {
  if (
    !state ||
    !validLiveManualAuthorityAcknowledgement(message) ||
    !connection ||
    message.adapterId !== connection.adapterId ||
    message.sessionId !== connection.sessionId ||
    message.requestId !== state.pendingRequestId ||
    message.action !== state.pendingAction
  ) return false;
  state.pendingRequestId = "";
  state.pendingAction = "";
  if (!message.accepted) {
    state.phase = "available";
    state.status = message.reason || "The robot rejected browser control authority.";
    return true;
  }
  if (message.action === "request-authority") {
    state.authorityToken = message.authorityToken;
    state.authorityExpiresAt = receivedAt + message.leaseMs;
    state.phase = "ready";
    state.status = "Authority granted. Commands stream only while the deadman is held.";
  } else {
    revokeLiveManualControl(state, "Manual-control authority released. Robot command is neutral.");
  }
  return true;
}

export function updateLiveManualAxes(state, patch = {}) {
  if (!state) return false;
  state.axes = {
    forward: clampAxis(patch.forward ?? state.axes.forward),
    turn: clampAxis(patch.turn ?? state.axes.turn),
    roll: clampAxis(patch.roll ?? state.axes.roll),
    pitch: clampAxis(patch.pitch ?? state.axes.pitch),
    yaw: clampAxis(patch.yaw ?? state.axes.yaw),
    bodyX: clampAxis(patch.bodyX ?? state.axes.bodyX),
    bodyY: clampAxis(patch.bodyY ?? state.axes.bodyY),
    height: clampAxis(patch.height ?? state.axes.height),
  };
  if (["stand", "careful", "trot"].includes(patch.mode)) state.mode = patch.mode;
  return true;
}

export function beginLiveManualDeadman(state, context, now = Date.now()) {
  if (
    !state?.authorityToken ||
    state.phase !== "ready" ||
    now >= state.authorityExpiresAt ||
    context?.robotState !== "armed" ||
    context?.connectionReady !== true ||
    context?.telemetryFresh !== true ||
    context?.controllerLinkReady !== true ||
    context?.workspaceActive !== true
  ) return false;
  state.deadmanActive = true;
  state.phase = "controlling";
  state.status = "Deadman held. Session-bound browser commands are streaming.";
  return true;
}

export function endLiveManualDeadman(state, reason = "Deadman released. A neutral stand frame was sent.") {
  if (!state) return false;
  state.deadmanActive = false;
  state.phase = state.authorityToken ? "ready" : state.supported ? "available" : "unavailable";
  state.status = reason;
  return true;
}

export function createLiveManualControlFrame(state, connection, now = Date.now(), forceNeutral = false) {
  if (!state?.authorityToken || !connection || now >= state.authorityExpiresAt) return null;
  const neutral = forceNeutral || !state.deadmanActive;
  return {
    type: "live-manual-control-frame",
    ...connection,
    authorityToken: state.authorityToken,
    sequence: state.frameSequence++,
    timestampMs: now,
    deadman: !neutral,
    neutral,
    mode: neutral ? "stand" : state.mode,
    axes: neutral ? { forward: 0, turn: 0, roll: 0, pitch: 0, yaw: 0, bodyX: 0, bodyY: 0, height: 0 } : { ...state.axes },
    safety: {
      requiresArmed: true,
      neutralOnExpiry: true,
      timeoutMs: LIVE_MANUAL_TIMEOUT_MS,
    },
  };
}

export function tickLiveManualControl(state, context, now = Date.now()) {
  if (!state?.authorityToken) return false;
  if (
    now >= state.authorityExpiresAt ||
    context?.robotState !== "armed" ||
    context?.connectionReady !== true ||
    context?.telemetryFresh !== true ||
    context?.controllerLinkReady !== true ||
    context?.workspaceActive !== true
  ) {
    revokeLiveManualControl(state, "Manual authority was revoked because a safety prerequisite expired.");
    return true;
  }
  return false;
}

export function revokeLiveManualControl(state, reason = "Manual-control authority revoked.") {
  if (!state) return false;
  state.authorityToken = "";
  state.authorityExpiresAt = 0;
  state.pendingRequestId = "";
  state.pendingAction = "";
  state.deadmanActive = false;
  state.phase = state.supported ? "available" : "unavailable";
  state.mode = "stand";
  state.axes = { forward: 0, turn: 0, roll: 0, pitch: 0, yaw: 0, bodyX: 0, bodyY: 0, height: 0 };
  state.status = reason;
  return true;
}

export function failLiveManualRequest(state, requestId, reason) {
  if (!state || state.pendingRequestId !== requestId) return false;
  state.pendingRequestId = "";
  state.pendingAction = "";
  state.phase = state.authorityToken ? "ready" : "available";
  state.status = reason;
  return true;
}
