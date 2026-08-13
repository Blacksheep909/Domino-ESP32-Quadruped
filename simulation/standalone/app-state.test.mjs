import assert from "node:assert/strict";
import test from "node:test";

import {
  createApplicationState,
  EXPERIENCE_EXPERT,
  selectExperience,
  selectWorkspace,
  simulationCanOwnControl,
  WORKSPACE_REAL_ROBOT,
  WORKSPACE_SIMULATION,
} from "./web/src/app-state.js";

test("the application opens in a simple simulation workspace", () => {
  const state = createApplicationState();
  assert.equal(state.workspace, WORKSPACE_SIMULATION);
  assert.equal(state.experience, "simple");
  assert.equal(simulationCanOwnControl(state), true);
});

test("the real robot workspace cannot own simulation controls", () => {
  const state = createApplicationState();
  selectWorkspace(state, WORKSPACE_REAL_ROBOT);
  assert.equal(simulationCanOwnControl(state), false);
});

test("a hidden simulation workspace cannot own controls", () => {
  const state = createApplicationState();
  assert.equal(simulationCanOwnControl(state, "hidden"), false);
});

test("simple and expert presentation state is independent of workspace", () => {
  const state = createApplicationState({ workspace: WORKSPACE_REAL_ROBOT });
  selectExperience(state, EXPERIENCE_EXPERT);
  assert.equal(state.workspace, WORKSPACE_REAL_ROBOT);
  assert.equal(state.experience, EXPERIENCE_EXPERT);
});
