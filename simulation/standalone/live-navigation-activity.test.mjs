import assert from "node:assert/strict";
import test from "node:test";

import {
  clearLiveNavigationActivity,
  createLiveNavigationActivityState,
  liveNavigationActivitySnapshot,
  recordLiveNavigationActivity,
} from "./web/src/live-navigation-activity.js";

test("autonomy activity keeps a bounded, newest-first operator timeline", () => {
  const state = createLiveNavigationActivityState(2);
  recordLiveNavigationActivity(state, "route", "Route started.", "success", 100);
  recordLiveNavigationActivity(state, "manual", "Manual override opened.", "warning", 200);
  recordLiveNavigationActivity(state, "safety stop", "LiDAR stop zone.", "fault", 300);

  assert.deepEqual(liveNavigationActivitySnapshot(state).map((event) => event.message), [
    "LiDAR stop zone.",
    "Manual override opened.",
  ]);
  assert.equal(state.events[0].tone, "fault");
  assert.equal(state.events[1].id, "200-2");
});

test("autonomy activity sanitizes unknown tones and clears cleanly", () => {
  const state = createLiveNavigationActivityState();
  assert.equal(recordLiveNavigationActivity(state, "route", "  Ready.  ", "unknown", 400).tone, "info");
  assert.equal(recordLiveNavigationActivity(state, "route", "", "info", 500), null);
  assert.equal(clearLiveNavigationActivity(state), true);
  assert.deepEqual(liveNavigationActivitySnapshot(state), []);
});
