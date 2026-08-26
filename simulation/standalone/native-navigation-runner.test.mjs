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
  skipNativeNavigationRunnerWaypoint,
  startNativeNavigationRunner,
  stopNativeNavigationRunner,
} from "./web/src/native-navigation-runner.js";

test("native route runner starts and resets with a bounded waypoint count", () => {
  const state = createNativeNavigationRunnerState();
  assert.equal(startNativeNavigationRunner(state, 2), true);
  assert.equal(nativeNavigationRunnerIsActive(state), true);
  assert.equal(state.mode, "route");
  assert.equal(state.currentIndex, 0);
  assert.equal(resetNativeNavigationRunner(state), true);
  assert.equal(state.phase, "idle");
  assert.equal(state.waypointCount, 0);
});

test("native route runner records the guarded return-home mode", () => {
  const state = createNativeNavigationRunnerState();
  assert.equal(startNativeNavigationRunner(state, 1, "return-home"), true);
  assert.equal(state.mode, "return-home");
  assert.equal(startNativeNavigationRunner(state, 1, "unsafe-mode"), false);
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

test("native route runner starts the next bounded patrol loop at waypoint one", () => {
  const state = createNativeNavigationRunnerState();
  assert.equal(startNativeNavigationRunner(state, 2, "route", 2), true);
  assert.equal(state.loopCount, 2);
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived" }), true);
  assert.equal(state.currentIndex, 1);
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived" }), true);
  assert.equal(state.phase, "running");
  assert.equal(state.completedLoops, 1);
  assert.equal(state.currentIndex, 0);
  assert.equal(state.lastCommand.state, "loop");
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived" }), true);
  assert.equal(state.currentIndex, 1);
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived" }), true);
  assert.equal(state.phase, "complete");
});

test("native route runner preserves a waypoint dwell across pause and resume", () => {
  const state = createNativeNavigationRunnerState();
  startNativeNavigationRunner(state, 2);
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived", holdS: 2, forward: 0, turn: 0 }, 1_000), true);
  assert.equal(state.currentIndex, 0);
  assert.equal(state.lastCommand.state, "holding");
  assert.equal(pauseNativeNavigationRunner(state, 1_500), true);
  assert.equal(state.holdRemainingMs, 1_500);
  assert.equal(resumeNativeNavigationRunner(state, 5_000), true);
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived", holdS: 2, forward: 0, turn: 0 }, 6_400), true);
  assert.equal(state.lastCommand.state, "holding");
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived", holdS: 2, forward: 0, turn: 0 }, 6_600), true);
  assert.equal(state.currentIndex, 1);
});

test("native route runner completes a final waypoint only after its dwell", () => {
  const state = createNativeNavigationRunnerState();
  startNativeNavigationRunner(state, 1);
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived", holdS: 1, forward: 0, turn: 0 }, 2_000), true);
  assert.equal(state.phase, "running");
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived", holdS: 1, forward: 0, turn: 0 }, 3_001), true);
  assert.equal(state.phase, "complete");
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

test("native route runner skips the current waypoint and clears an active hold", () => {
  const state = createNativeNavigationRunnerState();
  startNativeNavigationRunner(state, 3);
  assert.equal(acceptNativeNavigationCommand(state, { state: "arrived", holdS: 2, forward: 0, turn: 0 }, 1_000), true);
  assert.equal(skipNativeNavigationRunnerWaypoint(state), true);
  assert.equal(state.currentIndex, 1);
  assert.equal(state.phase, "running");
  assert.equal(state.holdingIndex, -1);
  assert.equal(state.holdUntilMs, 0);
  assert.equal(state.lastCommand.state, "skipped");
  assert.equal(state.lastCommand.skippedIndex, 0);
});

test("native route runner keeps a paused route paused when skipping a waypoint", () => {
  const state = createNativeNavigationRunnerState();
  startNativeNavigationRunner(state, 2);
  pauseNativeNavigationRunner(state);
  assert.equal(skipNativeNavigationRunnerWaypoint(state), true);
  assert.equal(state.currentIndex, 1);
  assert.equal(state.phase, "paused");
});

test("native route runner completes when its final route waypoint is skipped", () => {
  const state = createNativeNavigationRunnerState();
  startNativeNavigationRunner(state, 1);
  assert.equal(skipNativeNavigationRunnerWaypoint(state), true);
  assert.equal(state.currentIndex, 0);
  assert.equal(state.phase, "complete");
  assert.equal(nativeNavigationRunnerIsActive(state), false);
});

test("native route runner never skips Return Home", () => {
  const state = createNativeNavigationRunnerState();
  startNativeNavigationRunner(state, 1, "return-home");
  assert.equal(skipNativeNavigationRunnerWaypoint(state), false);
  assert.equal(state.currentIndex, 0);
  assert.equal(state.phase, "running");
});
