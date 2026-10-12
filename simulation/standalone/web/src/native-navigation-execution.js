const clamp = (value, minimum, maximum) => Math.max(minimum, Math.min(maximum, Number(value) || 0));

const totalDuration = (value) => Math.max(0, Number(value) || 0);

export function createNativeNavigationExecutionState() {
  return {
    phase: "idle",
    active: false,
    startedAt: 0,
    elapsedSeconds: 0,
    speedMultiplier: 1,
  };
}

function setPhase(state, phase) {
  state.phase = phase;
  state.active = phase === "running" || phase === "paused";
  if (phase !== "running") state.startedAt = 0;
}

export function startNativeNavigationExecution(state, totalSeconds, now = 0) {
  if (!state || totalDuration(totalSeconds) <= 0) return false;
  state.elapsedSeconds = 0;
  state.startedAt = Number(now) || 0;
  state.speedMultiplier = clamp(state.speedMultiplier || 1, 0.25, 4);
  setPhase(state, "running");
  return true;
}

export function tickNativeNavigationExecution(state, totalSeconds, now = 0) {
  if (!state) return false;
  const total = totalDuration(totalSeconds);
  if (state.phase !== "running") return state.phase === "complete";
  const current = Number.isFinite(Number(now)) ? Number(now) : 0;
  const started = Number.isFinite(Number(state.startedAt)) ? Number(state.startedAt) : current;
  state.elapsedSeconds = clamp(((current - started) / 1000) * state.speedMultiplier, 0, total);
  if (state.elapsedSeconds >= total) {
    state.elapsedSeconds = total;
    setPhase(state, "complete");
  }
  return state.phase === "complete";
}

export function pauseNativeNavigationExecution(state, totalSeconds, now = 0) {
  if (!state || state.phase !== "running") return false;
  tickNativeNavigationExecution(state, totalSeconds, now);
  if (state.phase === "complete") return false;
  setPhase(state, "paused");
  return true;
}

export function resumeNativeNavigationExecution(state, totalSeconds, now = 0) {
  if (!state || state.phase !== "paused" || state.elapsedSeconds >= totalDuration(totalSeconds)) return false;
  const current = Number.isFinite(Number(now)) ? Number(now) : 0;
  state.startedAt = current - (state.elapsedSeconds * 1000) / state.speedMultiplier;
  setPhase(state, "running");
  return true;
}

export function abortNativeNavigationExecution(state) {
  if (!state || state.phase === "idle") return false;
  setPhase(state, "aborted");
  return true;
}

export function resetNativeNavigationExecution(state) {
  if (!state) return false;
  state.phase = "idle";
  state.active = false;
  state.startedAt = 0;
  state.elapsedSeconds = 0;
  return true;
}

export function stepNativeNavigationExecution(state, totalSeconds, stepSeconds = 1) {
  if (!state || state.phase !== "paused") return false;
  const total = totalDuration(totalSeconds);
  state.elapsedSeconds = clamp(state.elapsedSeconds + Math.max(0.1, Number(stepSeconds) || 1), 0, total);
  if (state.elapsedSeconds >= total) setPhase(state, "complete");
  return true;
}

export function setNativeNavigationExecutionSpeed(state, speedMultiplier, now = 0, totalSeconds = Infinity) {
  if (!state) return false;
  if (state.phase === "running" && Number.isFinite(totalSeconds)) tickNativeNavigationExecution(state, totalSeconds, now);
  state.speedMultiplier = clamp(speedMultiplier, 0.25, 4);
  if (state.phase === "running") {
    const current = Number.isFinite(Number(now)) ? Number(now) : 0;
    state.startedAt = current - (state.elapsedSeconds * 1000) / state.speedMultiplier;
  }
  return true;
}
