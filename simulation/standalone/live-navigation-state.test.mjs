import assert from "node:assert/strict";
import test from "node:test";

import {
  addNavigationWaypoint,
  createLiveNavigationState,
  liveNavigationSnapshot,
  moveNavigationWaypoint,
  navigationMissionJson,
  parseNavigationMissionJson,
  removeNavigationWaypoint,
  sanitizeLiveNavigation,
} from "./web/src/live-navigation-state.js";

test("navigation state sanitizes GPS, LiDAR, and ArduPilot telemetry", () => {
  const receivedAt = 10_000;
  const navigation = sanitizeLiveNavigation({
    gps: {
      timestampMs: receivedAt,
      fixType: 3,
      satellites: 12,
      hdop: 0.9,
      position: { lat: -36.85, lon: 174.76, altM: 22 },
    },
    lidar: {
      timestampMs: receivedAt,
      online: true,
      rangesM: [0.4, 2.5, null, 4],
      incrementDeg: 90,
      offsetDeg: 0,
    },
    autopilot: {
      timestampMs: receivedAt,
      heartbeat: true,
      mode: "auto",
      modeCode: 10,
      armed: true,
      prearmReady: true,
      ekfHealthy: true,
      mission: { count: 2, current: 1, state: "active" },
    },
  }, receivedAt);
  const snapshot = liveNavigationSnapshot(navigation, createLiveNavigationState(), receivedAt + 100);
  assert.equal(snapshot.hasFix, true);
  assert.equal(snapshot.positionReady, true);
  assert.equal(snapshot.autonomyReady, true);
  assert.equal(snapshot.frontM, 0.4);
  assert.equal(snapshot.obstacleCount, 3);
  assert.equal(snapshot.mission.current, 1);
});

test("mission editing supports reorder, removal, export, and import", () => {
  const state = createLiveNavigationState();
  assert.equal(addNavigationWaypoint(state, { lat: -36.85, lon: 174.76, label: "A" }), true);
  assert.equal(addNavigationWaypoint(state, { lat: -36.86, lon: 174.77, label: "B" }), true);
  assert.equal(moveNavigationWaypoint(state, 1, "up"), true);
  assert.equal(state.missionDraft[0].label, "B");
  const parsed = parseNavigationMissionJson(navigationMissionJson(state));
  assert.equal(parsed.mission.length, 2);
  assert.equal(parsed.mission[0].label, "B");
  assert.equal(removeNavigationWaypoint(state, 0), true);
  assert.equal(state.missionDraft.length, 1);
});
