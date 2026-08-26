import assert from "node:assert/strict";
import test from "node:test";

import {
  acceptNativeNavigationCommand,
  blockNativeNavigationRunner,
  createNativeNavigationRunnerState,
  nativeNavigationRunnerIsActive,
  pauseNativeNavigationRunner,
  resetNativeNavigationRunner,
  resumeNativeNavigationRunner,
  startNativeNavigationRunner,
  stopNativeNavigationRunner,
} from "./web/src/native-navigation-runner.js";

test("native route runner starts and resets with a bounded waypoint count", () => {
  const state = createNativeNavigationRunnerState();
  assert.equal(startNativeNavigationRunner(state, 2), true);
  assert.equal(nativeNavigationRunnerIsActive(state), true);
  assert.equal(state.currentIndex, 0);
  assert.equal(resetNativeNavigationRunner(state), true);
  assert.equal(state.phase, "idle");
  assert.equal(state.waypointCount, 0);
});

test("native route runner advances and completes at the final waypoint", () => {
  const state = createNativeNavigationRunnerState();
  startNativeNavigationRunner(state, 2);
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived", forward: 0, turn: 0 }), true);
  assert.equal(state.currentIndex, 1);
  assert.equal(state.phase, "running");
  assert.equal(acceptNativeNavigationCommand(state, { state: "complete", forward: 0, turn: 0 }), true);
  assert.equal(state.phase, "complete");
  assert.equal(nativeNavigationRunnerIsActive(state), false);
});

test("native route runner pauses and resumes without losing waypoint progress", () => {
  const state = createNativeNavigationRunnerState();
  startNativeNavigationRunner(state, 3);
  state.currentIndex = 1;
  assert.equal(pauseNativeNavigationRunner(state), true);
  assert.equal(state.phase, "paused");
  assert.equal(state.currentIndex, 1);
  assert.equal(resumeNativeNavigationRunner(state), true);
  assert.equal(state.phase, "running");
  assert.equal(state.currentIndex, 1);
});

test("native route runner blocks on sensor and obstacle safety states", () => {
  const state = createNativeNavigationRunnerState();
  startNativeNavigationRunner(state, 1);
  assert.equal(acceptNativeNavigationCommand(state, { state: "sensor-wait", reason: "No LiDAR" }), true);
  assert.equal(state.phase, "blocked");
  assert.match(state.stopReason, /LiDAR/);
  resetNativeNavigationRunner(state);
  startNativeNavigationRunner(state, 1);
  assert.equal(blockNativeNavigationRunner(state, "Fence breach"), true);
  assert.equal(state.phase, "blocked");
});

test("native route runner rejects invalid commands and records operator stop", () => {
  const state = createNativeNavigationRunnerState();
  assert.equal(startNativeNavigationRunner(state, 0), false);
  assert.equal(acceptNativeNavigationCommand(state, { state: "navigating" }), false);
  startNativeNavigationRunner(state, 1);
  assert.equal(stopNativeNavigationRunner(state, "Operator override"), true);
  assert.equal(state.phase, "stopped");
  assert.equal(stopNativeNavigationRunner(state), true);
});
