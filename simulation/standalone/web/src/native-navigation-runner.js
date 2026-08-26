const ROUTE_PHASES = Object.freeze(["idle", "running", "paused", "blocked", "complete", "stopped"]);

export function createNativeNavigationRunnerState() {
  return {
    phase: "idle",
    currentIndex: 0,
    waypointCount: 0,
    lastCommand: null,
    stopReason: "",
  };
}

export function startNativeNavigationRunner(state, waypointCount) {
  const count = Number(waypointCount);
  if (!state || !Number.isSafeInteger(count) || count < 1) return false;
  state.phase = "running";
  state.currentIndex = 0;
  state.waypointCount = count;
  state.lastCommand = null;
  state.stopReason = "";
  return true;
}

export function pauseNativeNavigationRunner(state) {
  if (!state || state.phase !== "running") return false;
  state.phase = "paused";
  return true;
}

export function resumeNativeNavigationRunner(state) {
  if (!state || state.phase !== "paused") return false;
  state.phase = "running";
  return true;
}

export function stopNativeNavigationRunner(state, reason = "Route stopped by operator.") {
  if (!state || state.phase === "idle") return false;
  state.phase = "stopped";
  state.stopReason = String(reason || "Route stopped by operator.").slice(0, 256);
  return true;
}

export function blockNativeNavigationRunner(state, reason = "Native route safety gate blocked motion.") {
  if (!state || state.phase === "idle") return false;
  state.phase = "blocked";
  state.stopReason = String(reason || "Native route safety gate blocked motion.").slice(0, 256);
  return true;
}

export function acceptNativeNavigationCommand(state, command) {
  if (!state || state.phase !== "running" || !command || typeof command !== "object") return false;
  state.lastCommand = {
    state: String(command.state || "blocked"),
    forward: Number(command.forward) || 0,
    turn: Number(command.turn) || 0,
  };
  if (command.state === "arrived") {
    state.currentIndex += 1;
    if (state.currentIndex >= state.waypointCount) state.phase = "complete";
    return true;
  }
  if (command.state === "complete") {
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

export { ROUTE_PHASES };
