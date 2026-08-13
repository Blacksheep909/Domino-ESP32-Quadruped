import test from "node:test";
import assert from "node:assert/strict";

import {
  createLiveViewState,
  LIVE_VIEW_CALIBRATION,
  LIVE_VIEW_DIAGNOSTICS,
  LIVE_VIEW_GAITS,
  LIVE_VIEW_COMPARE,
  LIVE_VIEW_DATA,
  LIVE_VIEW_SESSIONS,
  selectLiveView,
} from "./web/src/live-view-state.js";

test("LIVE opens on the comparison view", () => {
  assert.equal(createLiveViewState().selected, LIVE_VIEW_COMPARE);
});

test("implemented Data, Calibration, Gaits, Diagnostics and Sessions views are selectable", () => {
  const state = createLiveViewState();
  assert.equal(selectLiveView(state, LIVE_VIEW_DATA), true);
  assert.equal(state.selected, LIVE_VIEW_DATA);
  assert.equal(selectLiveView(state, LIVE_VIEW_CALIBRATION), true);
  assert.equal(state.selected, LIVE_VIEW_CALIBRATION);
  assert.equal(selectLiveView(state, LIVE_VIEW_DIAGNOSTICS), true);
  assert.equal(state.selected, LIVE_VIEW_DIAGNOSTICS);
  assert.equal(selectLiveView(state, LIVE_VIEW_GAITS), true);
  assert.equal(state.selected, LIVE_VIEW_GAITS);
  assert.equal(selectLiveView(state, LIVE_VIEW_SESSIONS), true);
  assert.equal(state.selected, LIVE_VIEW_SESSIONS);
});

test("unimplemented or unknown views cannot replace the active page", () => {
  const state = createLiveViewState();
  selectLiveView(state, LIVE_VIEW_DATA);
  assert.equal(selectLiveView(state, "controls"), false);
  assert.equal(state.selected, LIVE_VIEW_DATA);
});
