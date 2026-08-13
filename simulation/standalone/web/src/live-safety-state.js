import {
  LIVE_ARM_HOLD_MS,
  LIVE_ROBOT_STATES,
  LIVE_SAFETY_WATCHDOG_MS,
  validLiveSafetyAcknowledgement,
  validLiveSafetyHeartbeatAcknowledgement,
} from "./live-safety-protocol.js";

export function createLiveSafetyState() {
  return {
    robotState: "disconnected",
    pendingRequestId: "",
    pendingAction: "",
    armHoldStartedAt: 0,
    armHoldProgress: 0,
    heartbeatSequence: 0,
    lastHeartbeatAckAt: 0,
    lastHeartbeatSequence: -1,
    watchdogRemainingMs: 0,
    watchdogTripped: false,
    status: "Connect a robot to inspect its safety state.",
  };
}

export function setLiveSafetyRobotState(state, robotState, source = "telemetry") {
  if (!state || !LIVE_ROBOT_STATES.includes(robotState)) return false;
  if (state.watchdogTripped && robotState === "armed") return false;
  const changed = state.robotState !== robotState;
  state.robotState = robotState;
  if (robotState !== "armed") {
    state.watchdogTripped = robotState === "watchdog";
    state.lastHeartbeatAckAt = 0;
    state.watchdogRemainingMs = 0;
  }
  if (changed && source === "telemetry") state.status = `Robot reports ${robotState.toUpperCase()}.`;
  return changed;
}

export function lockLiveSafetyState(state, reason = "The engineering session is disconnected.") {
  if (!state) return false;
  state.robotState = "disconnected";
  state.pendingRequestId = "";
  state.pendingAction = "";
  state.armHoldStartedAt = 0;
  state.armHoldProgress = 0;
  state.lastHeartbeatAckAt = 0;
  state.watchdogRemainingMs = 0;
  state.status = reason;
  return true;
}

export function liveSafetyCanArm(state, context) {
  return Boolean(
    state &&
    !state.pendingRequestId &&
    state.robotState === "disarmed" &&
    context?.connectionReady === true &&
    context?.telemetryFresh === true &&
    context?.driveLinkAlive === true
  );
}

export function beginLiveArmHold(state, context, now = Date.now()) {
  if (!liveSafetyCanArm(state, context)) return false;
  state.armHoldStartedAt = now;
  state.armHoldProgress = 0;
  state.status = "Keep holding while the arm prerequisites remain healthy.";
  return true;
}

export function updateLiveArmHold(state, context, now = Date.now()) {
  if (!state?.armHoldStartedAt) return { active: false, complete: false, progress: 0 };
  if (!liveSafetyCanArm(state, context)) {
    cancelLiveArmHold(state, "Arm hold cancelled because a safety prerequisite changed.");
    return { active: false, complete: false, progress: 0 };
  }
  state.armHoldProgress = Math.max(0, Math.min(1, (now - state.armHoldStartedAt) / LIVE_ARM_HOLD_MS));
  return { active: true, complete: state.armHoldProgress >= 1, progress: state.armHoldProgress };
}

export function cancelLiveArmHold(state, reason = "Arm hold cancelled. Robot remains disarmed.") {
  if (!state) return false;
  state.armHoldStartedAt = 0;
  state.armHoldProgress = 0;
  state.status = reason;
  return true;
}

export function createLiveSafetyCommand(state, action, connection, requestId, now = Date.now()) {
  if (!state || !connection || state.pendingRequestId) return null;
  const command = {
    type: "live-safety-command",
    action,
    requestId: String(requestId),
    timestampMs: now,
    ...connection,
  };
  if (action === "arm") {
    if (state.armHoldProgress < 1 || state.robotState !== "disarmed") return null;
    command.safety = {
      holdMs: LIVE_ARM_HOLD_MS,
      requiresDisarmed: true,
      requiresFreshTelemetry: true,
      requiresDriveLink: true,
      watchdogMs: LIVE_SAFETY_WATCHDOG_MS,
    };
  } else if (action === "disarm" && state.robotState !== "armed") return null;
  else if (action === "reset-estop") command.safety = { physicalResetRequired: true };
  else if (!['request-state', 'estop'].includes(action)) return null;
  return command;
}

export function markLiveSafetyPending(state, command) {
  if (!state || !command) return false;
  state.pendingRequestId = command.requestId;
  state.pendingAction = command.action;
  state.armHoldStartedAt = 0;
  state.armHoldProgress = 0;
  state.status = `Waiting for the robot to acknowledge ${command.action.toUpperCase()}...`;
  return true;
}

export function acceptLiveSafetyAcknowledgement(state, message, connection, receivedAt = Date.now()) {
  if (
    !state ||
    !validLiveSafetyAcknowledgement(message) ||
    !connection ||
    message.adapterId !== connection.adapterId ||
    message.sessionId !== connection.sessionId ||
    message.requestId !== state.pendingRequestId ||
    message.action !== state.pendingAction
  ) return false;
  state.pendingRequestId = "";
  state.pendingAction = "";
  if (!message.accepted) {
    state.robotState = message.robotState;
    state.status = message.reason || `Robot rejected ${message.action.toUpperCase()}.`;
    return true;
  }
  state.robotState = message.robotState;
  state.status = message.action === "arm"
    ? "Robot armed. Browser heartbeat watchdog is active."
    : message.action === "estop"
      ? "Emergency stop acknowledged and latched by the robot."
      : message.action === "reset-estop"
        ? "Robot confirmed the physical E-stop latch is reset."
        : `Robot confirmed ${message.robotState.toUpperCase()}.`;
  if (message.robotState === "armed") {
    state.lastHeartbeatAckAt = receivedAt;
    state.watchdogRemainingMs = LIVE_SAFETY_WATCHDOG_MS;
    state.watchdogTripped = false;
  } else {
    state.lastHeartbeatAckAt = 0;
    state.watchdogRemainingMs = 0;
  }
  return true;
}

export function createLiveSafetyHeartbeat(state, connection, workspaceActive, now = Date.now()) {
  if (!state || state.robotState !== "armed" || !connection || workspaceActive !== true) return null;
  return {
    type: "live-safety-heartbeat",
    ...connection,
    sequence: state.heartbeatSequence++,
    timestampMs: now,
    workspaceActive: true,
  };
}

export function acceptLiveSafetyHeartbeatAcknowledgement(state, message, connection, receivedAt = Date.now()) {
  if (
    !state ||
    state.watchdogTripped ||
    !validLiveSafetyHeartbeatAcknowledgement(message) ||
    !connection ||
    message.adapterId !== connection.adapterId ||
    message.sessionId !== connection.sessionId ||
    message.sequence <= state.lastHeartbeatSequence
  ) return false;
  state.lastHeartbeatSequence = message.sequence;
  state.lastHeartbeatAckAt = receivedAt;
  state.watchdogRemainingMs = message.watchdogRemainingMs;
  state.robotState = message.robotState;
  return true;
}

export function tickLiveSafetyWatchdog(state, now = Date.now()) {
  if (!state || state.robotState !== "armed" || !state.lastHeartbeatAckAt) return false;
  const age = now - state.lastHeartbeatAckAt;
  state.watchdogRemainingMs = Math.max(0, LIVE_SAFETY_WATCHDOG_MS - age);
  if (age <= LIVE_SAFETY_WATCHDOG_MS) return false;
  state.robotState = "watchdog";
  state.watchdogTripped = true;
  state.status = "Safety heartbeat expired. The robot-side watchdog must disarm outputs.";
  return true;
}

export function failLiveSafetyRequest(state, requestId, reason) {
  if (!state || state.pendingRequestId !== requestId) return false;
  state.pendingRequestId = "";
  state.pendingAction = "";
  state.armHoldStartedAt = 0;
  state.armHoldProgress = 0;
  state.status = reason;
  return true;
}
