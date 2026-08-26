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
  navigationMissionPreview,
  navigationMissionRecommendedRange,
  removeNavigationWaypoint,
  reverseNavigationWaypoints,
  gpsTrackToMission,
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

test("recorded GPS tracks become bounded coordinate missions with local offsets", () => {
  const origin = { lat: -36.85, lon: 174.76, altM: 20 };
  const track = Array.from({ length: 7 }, (_, index) => ({
    lat: -36.85 + index * 0.0001,
    lon: 174.76 + index * 0.0001,
    altM: 20 + index,
  }));
  const mission = gpsTrackToMission(track, origin, 4);
  assert.equal(mission.length, 4);
  assert.equal(mission[0].label, "TRACK 01");
  assert.equal(mission.at(-1).label, "TRACK 04");
  assert.equal(mission[0].lat, track[0].lat);
  assert.equal(mission.at(-1).lon, track.at(-1).lon);
  assert.deepEqual(mission[0].local, { northM: 0, eastM: 0 });
  assert.ok(mission[1].local.northM > 0);
  assert.equal(mission[2].speedMps, 0.5);
  assert.equal(gpsTrackToMission(track, origin, 0).length, 7);
  assert.deepEqual(gpsTrackToMission([{ lat: 400, lon: 2 }]), []);
});

test("route planner previews a local route without vehicle execution", () => {
  const state = createLiveNavigationState();
  addNavigationWaypoint(state, { local: { northM: 0, eastM: 0 }, speedMps: 1 });
  addNavigationWaypoint(state, { local: { northM: 0, eastM: 4 }, speedMps: 2, holdS: 2 });
  const preview = navigationMissionPreview(state.missionDraft, null, 1);
  assert.equal(preview.ready, true);
  assert.equal(preview.currentIndex, 1);
  assert.equal(preview.position.eastM, 2);
  assert.equal(preview.complete, false);
  const finished = navigationMissionPreview(state.missionDraft, null, 4);
  assert.equal(finished.complete, true);
  assert.equal(finished.position.eastM, 4);
  assert.equal(navigationMissionPreview([{ local: { northM: 1, eastM: 1 } }], null, 0).ready, true);
  assert.equal(navigationMissionPreview([{ lat: -36.85, lon: 174.76 }], null, 0).ready, false);
});

test("route planner repeats a bounded Domino patrol loop with a closing leg", () => {
  const mission = [
    { local: { northM: 0, eastM: 0 }, speedMps: 1 },
    { local: { northM: 0, eastM: 4 }, speedMps: 1 },
  ];
  const metrics = navigationMissionMetrics(mission, null, 2);
  assert.equal(metrics.loopCount, 2);
  assert.equal(metrics.totalDistanceM, 12);
  assert.equal(metrics.estimatedSeconds, 12);

  const preview = navigationMissionPreview(mission, null, 6, 2);
  assert.equal(preview.ready, true);
  assert.equal(preview.loopIndex, 1);
  assert.equal(preview.currentIndex, 0);
  assert.equal(preview.position.eastM, 2);
  assert.equal(preview.totalSeconds, 12);
  assert.equal(navigationMissionPreview(mission, null, 12, 2).complete, true);

  const state = createLiveNavigationState();
  state.loopCount = 3;
  const parsed = parseNavigationMissionJson(navigationMissionJson(state));
  assert.equal(parsed.loopCount, 3);
});

test("route planner recommends a readable map range for the draft", () => {
  assert.equal(navigationMissionRecommendedRange([
    { local: { northM: 2, eastM: -3 } },
  ]), 40);
  assert.equal(navigationMissionRecommendedRange([
    { local: { northM: 18, eastM: 0 } },
  ]), 80);
  assert.equal(navigationMissionRecommendedRange([
    { local: { northM: 90, eastM: 0 } },
  ]), 160);
  assert.equal(navigationMissionRecommendedRange([
    { lat: -36.85, lon: 174.76 },
  ]), null);
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

test("route planner can reverse a route while preserving waypoint data", () => {
  const state = createLiveNavigationState();
  addNavigationWaypoint(state, { local: { northM: 0, eastM: 0 }, label: "Start", speedMps: 0.4 });
  addNavigationWaypoint(state, { local: { northM: 4, eastM: 2 }, label: "Gate", holdS: 3 });
  assert.equal(reverseNavigationWaypoints(state), true);
  assert.equal(state.missionDraft[0].label, "Gate");
  assert.equal(state.missionDraft[0].holdS, 3);
  assert.equal(state.missionDraft[1].label, "Start");
  assert.equal(state.missionDraft[1].speedMps, 0.4);
  assert.equal(reverseNavigationWaypoints(createLiveNavigationState()), false);
});
