import assert from "node:assert/strict";
import test from "node:test";

import {
  abortNativeNavigationExecution,
  createNativeNavigationExecutionState,
  pauseNativeNavigationExecution,
  resetNativeNavigationExecution,
  resumeNativeNavigationExecution,
  setNativeNavigationExecutionSpeed,
  startNativeNavigationExecution,
  stepNativeNavigationExecution,
  tickNativeNavigationExecution,
} from "./web/src/native-navigation-execution.js";

test("native preview execution pauses and resumes without losing elapsed time", () => {
  const state = createNativeNavigationExecutionState();
  assert.equal(startNativeNavigationExecution(state, 10, 1_000), true);
  tickNativeNavigationExecution(state, 10, 3_000);
  assert.equal(state.elapsedSeconds, 2);
  assert.equal(pauseNativeNavigationExecution(state, 10, 3_500), true);
  assert.equal(state.phase, "paused");
  assert.equal(state.elapsedSeconds, 2.5);
  assert.equal(resumeNativeNavigationExecution(state, 10, 8_000), true);
  tickNativeNavigationExecution(state, 10, 9_000);
  assert.equal(state.elapsedSeconds, 3.5);
});

test("native preview execution steps while paused and completes at the route end", () => {
  const state = createNativeNavigationExecutionState();
  startNativeNavigationExecution(state, 2, 0);
  pauseNativeNavigationExecution(state, 2, 500);
  assert.equal(stepNativeNavigationExecution(state, 2, 1), true);
  assert.equal(state.elapsedSeconds, 1.5);
  assert.equal(stepNativeNavigationExecution(state, 2, 1), true);
  assert.equal(state.phase, "complete");
  assert.equal(state.elapsedSeconds, 2);
});

test("native preview execution speed changes preserve the current position", () => {
  const state = createNativeNavigationExecutionState();
  startNativeNavigationExecution(state, 10, 0);
  tickNativeNavigationExecution(state, 10, 1_000);
  setNativeNavigationExecutionSpeed(state, 2, 1_000, 10);
  tickNativeNavigationExecution(state, 10, 2_000);
  assert.equal(state.elapsedSeconds, 3);
  assert.equal(state.speedMultiplier, 2);
});

test("native preview execution aborts and can be reset without retaining motion", () => {
  const state = createNativeNavigationExecutionState();
  startNativeNavigationExecution(state, 10, 0);
  assert.equal(abortNativeNavigationExecution(state), true);
  assert.equal(state.phase, "aborted");
  assert.equal(state.active, false);
  assert.equal(resetNativeNavigationExecution(state), true);
  assert.equal(state.phase, "idle");
  assert.equal(state.elapsedSeconds, 0);
});
