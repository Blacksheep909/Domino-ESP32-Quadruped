import assert from "node:assert/strict";
import test from "node:test";

import {
  addNavigationWaypoint,
  coordinateToLocalOffset,
  createLiveNavigationState,
  liveNavigationSnapshot,
  localOffsetToCoordinate,
  moveNavigationWaypoint,
  moveNavigationWaypointToLocal,
  navigationMissionMetrics,
  navigationMissionGeofenceStatus,
  navigationMissionJson,
  parseNavigationMissionJson,
  removeNavigationWaypoint,
  sanitizeLiveNavigation,
  missionWaypointHasCoordinate,
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
  state.plannerRangeM = 160;
  state.geofence = { enabled: true, maxRadiusM: 75, polygon: [] };
  state.obstacleBehavior.stopDistanceM = 0.6;
  assert.equal(addNavigationWaypoint(state, { lat: -36.85, lon: 174.76, label: "A" }), true);
  assert.equal(addNavigationWaypoint(state, { lat: -36.86, lon: 174.77, label: "B" }), true);
  assert.equal(moveNavigationWaypoint(state, 1, "up"), true);
  assert.equal(state.missionDraft[0].label, "B");
  const parsed = parseNavigationMissionJson(navigationMissionJson(state));
  assert.equal(parsed.mission.length, 2);
  assert.equal(parsed.mission[0].label, "B");
  assert.equal(parsed.plannerRangeM, 160);
  assert.equal(parsed.geofence.maxRadiusM, 75);
  assert.equal(parsed.obstacleBehavior.stopDistanceM, 0.6);
  assert.equal(removeNavigationWaypoint(state, 0), true);
  assert.equal(state.missionDraft.length, 1);
});

test("route planner preserves local waypoints and converts them around a GPS origin", () => {
  const origin = { lat: -36.85, lon: 174.76, altM: 18 };
  const offset = coordinateToLocalOffset({ lat: -36.8491, lon: 174.7612 }, origin);
  assert.ok(offset.northM > 0);
  assert.ok(offset.eastM > 0);
  const coordinate = localOffsetToCoordinate(offset, origin);
  assert.ok(Math.abs(coordinate.lat - (-36.8491)) < 0.000001);
  assert.ok(Math.abs(coordinate.lon - 174.7612) < 0.000001);

  const state = createLiveNavigationState();
  state.plannerOrigin = origin;
  assert.equal(addNavigationWaypoint(state, { local: { northM: 4, eastM: -2 }, label: "North gate" }), true);
  const parsed = parseNavigationMissionJson(navigationMissionJson(state));
  assert.equal(parsed.plannerOrigin.lat, origin.lat);
  assert.equal(parsed.plannerOrigin.lon, origin.lon);
  assert.deepEqual(parsed.mission[0].local, { northM: 4, eastM: -2 });
  assert.equal(parsed.mission[0].lat, null);
  assert.equal(missionWaypointHasCoordinate(parsed.mission[0]), false);
});

test("route planner reports measurable distance and an honest unresolved state", () => {
  const metrics = navigationMissionMetrics([
    { local: { northM: 0, eastM: 0 }, speedMps: 1, holdS: 2 },
    { local: { northM: 3, eastM: 4 }, speedMps: 2, holdS: 1 },
  ]);
  assert.equal(metrics.waypointCount, 2);
  assert.equal(metrics.totalDistanceM, 5);
  assert.equal(metrics.estimatedSeconds, 5.5);
  assert.equal(metrics.coordinateReady, false);
  assert.equal(metrics.unresolvedCount, 0);

  const unresolved = navigationMissionMetrics([
    { local: { northM: 0, eastM: 0 } },
    { lat: -36.85, lon: 174.76 },
  ]);
  assert.equal(unresolved.totalDistanceM, null);
  assert.equal(unresolved.estimatedSeconds, null);
  assert.equal(unresolved.unresolvedCount, 1);
});

test("route planner reports waypoints outside the active home radius", () => {
  const origin = { lat: -36.85, lon: 174.76 };
  const status = navigationMissionGeofenceStatus([
    { local: { northM: 3, eastM: 4 } },
    { local: { northM: 12, eastM: 5 } },
  ], origin, 5, true);
  assert.equal(status.checked, true);
  assert.equal(status.outsideCount, 1);
  assert.equal(status.unresolvedCount, 0);

  const unreferenced = navigationMissionGeofenceStatus([
    { local: { northM: 3, eastM: 4 } },
  ], null, 5, true);
  assert.equal(unreferenced.checked, false);
  assert.equal(unreferenced.outsideCount, 0);
  assert.equal(unreferenced.unresolvedCount, 0);
});

test("route planner drags a waypoint within the active local frame", () => {
  const origin = { lat: -36.85, lon: 174.76, altM: 18 };
  const state = createLiveNavigationState();
  assert.equal(addNavigationWaypoint(state, { local: { northM: 2, eastM: 1 }, label: "Dock" }), true);
  assert.equal(moveNavigationWaypointToLocal(state, 0, { northM: 8, eastM: -4 }, origin), true);
  assert.deepEqual(state.missionDraft[0].local, { northM: 8, eastM: -4 });
  assert.ok(Number.isFinite(state.missionDraft[0].lat));
  assert.ok(Number.isFinite(state.missionDraft[0].lon));
  assert.equal(moveNavigationWaypointToLocal(state, 0, { northM: 20_001, eastM: 0 }, origin), false);
});
