const ROUTE_PHASES = Object.freeze(["idle", "running", "paused", "blocked", "complete", "stopped"]);
const NAVIGATION_MODES = Object.freeze(["route", "return-home"]);

export function createNativeNavigationRunnerState() {
  return {
    phase: "idle",
    mode: "route",
    currentIndex: 0,
    waypointCount: 0,
    loopCount: 1,
    completedLoops: 0,
    holdingIndex: -1,
    holdUntilMs: 0,
    holdRemainingMs: 0,
    lastCommand: null,
    stopReason: "",
  };
}

export function startNativeNavigationRunner(state, waypointCount, mode = "route", loopCount = 1) {
  const count = Number(waypointCount);
  const loops = Math.max(1, Math.min(5, Math.round(Number(loopCount) || 1)));
  if (!state || !Number.isSafeInteger(count) || count < 1 || !NAVIGATION_MODES.includes(mode)) return false;
  state.phase = "running";
  state.mode = mode;
  state.currentIndex = 0;
  state.waypointCount = count;
  state.loopCount = mode === "route" ? loops : 1;
  state.completedLoops = 0;
  state.holdingIndex = -1;
  state.holdUntilMs = 0;
  state.holdRemainingMs = 0;
  state.lastCommand = null;
  state.stopReason = "";
  return true;
}

export function pauseNativeNavigationRunner(state, nowMs = Date.now()) {
  if (!state || state.phase !== "running") return false;
  if (state.holdingIndex === state.currentIndex && state.holdUntilMs > 0) {
    state.holdRemainingMs = Math.max(0, state.holdUntilMs - Number(nowMs));
    state.holdUntilMs = 0;
  }
  state.phase = "paused";
  return true;
}

export function resumeNativeNavigationRunner(state, nowMs = Date.now()) {
  if (!state || state.phase !== "paused") return false;
  if (state.holdingIndex === state.currentIndex && state.holdRemainingMs > 0) {
    state.holdUntilMs = Number(nowMs) + state.holdRemainingMs;
    state.holdRemainingMs = 0;
  }
  state.phase = "running";
  return true;
}

export function stopNativeNavigationRunner(state, reason = "Route stopped by operator.") {
  if (!state || state.phase === "idle") return false;
  state.phase = "stopped";
  state.holdingIndex = -1;
  state.holdUntilMs = 0;
  state.holdRemainingMs = 0;
  state.stopReason = String(reason || "Route stopped by operator.").slice(0, 256);
  return true;
}

/**
 * Advance an active Domino route past its current waypoint without changing
 * the guarded control lease. This is an operator recovery action for a route
 * that is still safe to continue; Return Home deliberately cannot be skipped.
 */
export function skipNativeNavigationRunnerWaypoint(state) {
  if (!state || !nativeNavigationRunnerIsActive(state) || state.mode !== "route") return false;
  if (!Number.isSafeInteger(state.currentIndex) || !Number.isSafeInteger(state.waypointCount) || state.waypointCount < 1) return false;
  const skippedIndex = state.currentIndex;
  state.holdingIndex = -1;
  state.holdUntilMs = 0;
  state.holdRemainingMs = 0;
  state.lastCommand = {
    state: "skipped",
    forward: 0,
    turn: 0,
    skippedIndex,
  };
  state.currentIndex += 1;
  if (state.currentIndex >= state.waypointCount) {
    if (state.completedLoops + 1 < state.loopCount) {
      state.completedLoops += 1;
      state.currentIndex = 0;
      state.lastCommand = { state: "loop", forward: 0, turn: 0, completedLoops: state.completedLoops };
    } else {
      state.currentIndex = Math.max(0, state.waypointCount - 1);
      state.phase = "complete";
    }
  }
  return true;
}

export function blockNativeNavigationRunner(state, reason = "Native route safety gate blocked motion.") {
  if (!state || state.phase === "idle") return false;
  state.phase = "blocked";
  state.holdingIndex = -1;
  state.holdUntilMs = 0;
  state.holdRemainingMs = 0;
  state.stopReason = String(reason || "Native route safety gate blocked motion.").slice(0, 256);
  return true;
}

export function acceptNativeNavigationCommand(state, command, nowMs = Date.now()) {
  if (!state || state.phase !== "running" || !command || typeof command !== "object") return false;
  state.lastCommand = {
    state: String(command.state || "blocked"),
    forward: Number(command.forward) || 0,
    turn: Number(command.turn) || 0,
  };
  const now = Number(nowMs);
  if (state.holdingIndex === state.currentIndex && state.holdUntilMs > now) {
    state.lastCommand = { state: "holding", forward: 0, turn: 0, holdRemainingS: (state.holdUntilMs - now) / 1_000 };
    return true;
  }
  if (command.state === "arrived") {
    if (state.holdingIndex === state.currentIndex) {
      state.holdingIndex = -1;
      state.holdUntilMs = 0;
      state.holdRemainingMs = 0;
    } else {
      const holdS = Math.max(0, Number(command.holdS) || 0);
      if (holdS > 0) {
        state.holdingIndex = state.currentIndex;
        state.holdUntilMs = now + holdS * 1_000;
        state.lastCommand = { state: "holding", forward: 0, turn: 0, holdRemainingS: holdS };
        return true;
      }
    }
    state.currentIndex += 1;
    if (state.currentIndex >= state.waypointCount) {
      if (state.completedLoops + 1 < state.loopCount) {
        state.completedLoops += 1;
        state.currentIndex = 0;
        state.lastCommand = { state: "loop", forward: 0, turn: 0, completedLoops: state.completedLoops };
      } else {
        state.phase = "complete";
      }
    }
    return true;
  }
  if (command.state === "complete") {
    state.holdingIndex = -1;
    state.holdUntilMs = 0;
    state.holdRemainingMs = 0;
    state.currentIndex = Math.max(0, state.waypointCount - 1);
    state.phase = "complete";
    return true;
  }
  if (["blocked", "sensor-wait", "obstacle-stop", "geofence-stop"].includes(command.state)) {
    blockNativeNavigationRunner(state, command.reason);
    return true;
  }
  return command.state === "navigating";
}

export function resetNativeNavigationRunner(state) {
  if (!state) return false;
  Object.assign(state, createNativeNavigationRunnerState());
  return true;
}

export function nativeNavigationRunnerIsActive(state) {
  return Boolean(state && (state.phase === "running" || state.phase === "paused"));
}

export { NAVIGATION_MODES, ROUTE_PHASES };
