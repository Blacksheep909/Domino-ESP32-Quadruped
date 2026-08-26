import assert from "node:assert/strict";
import test from "node:test";

import { headingErrorDeg, nativeNavigationCommand } from "./web/src/native-navigation-controller.js";

const route = [
  { local: { northM: 0, eastM: 0 }, radiusM: 1.2, speedMps: 0.8 },
  { local: { northM: 0, eastM: 8 }, radiusM: 1.2, speedMps: 0.8 },
];

test("native navigation turns toward a local waypoint with bounded axes", () => {
  const command = nativeNavigationCommand({
    waypoints: route,
    position: { northM: 0, eastM: 2 },
    headingDeg: 90,
    currentIndex: 1,
    obstacle: { enabled: false },
  });
  assert.equal(command.state, "navigating");
  assert.ok(command.forward >= 0 && command.forward <= 0.8);
  assert.ok(Math.abs(command.turn) <= 0.8);
  assert.ok(command.distanceM > 0);
});

test("native navigation can drive one selected waypoint without the rest of the route", () => {
  const directWaypoint = [{ local: { northM: 0, eastM: 8 }, radiusM: 1.2, speedMps: 0.8 }];
  const command = nativeNavigationCommand({
    waypoints: directWaypoint,
    position: { northM: 0, eastM: 2 },
    headingDeg: 90,
    currentIndex: 0,
    loopCount: 1,
    obstacle: { enabled: false },
  });
  assert.equal(command.state, "navigating");
  assert.equal(command.currentIndex, 0);
  assert.equal(command.target.eastM, 8);
  assert.ok(command.forward > 0 && command.forward <= 0.8);
});

test("native navigation zeros the command when a waypoint is reached", () => {
  const command = nativeNavigationCommand({
    waypoints: route,
    position: { northM: 0, eastM: 8.5 },
    headingDeg: 90,
    currentIndex: 1,
    obstacle: { enabled: false },
  });
  assert.equal(command.state, "complete");
  assert.equal(command.forward, 0);
  assert.equal(command.turn, 0);
});

test("native navigation holds the final waypoint until the patrol loop count is met", () => {
  const waypoint = [{ local: { northM: 0, eastM: 0 }, radiusM: 1.2, speedMps: 0.8 }];
  const firstLoop = nativeNavigationCommand({
    waypoints: waypoint,
    position: { northM: 0, eastM: 0 },
    headingDeg: 0,
    loopCount: 2,
    completedLoops: 0,
    obstacle: { enabled: false },
  });
  assert.equal(firstLoop.state, "arrived");
  const finalLoop = nativeNavigationCommand({
    waypoints: waypoint,
    position: { northM: 0, eastM: 0 },
    headingDeg: 0,
    loopCount: 2,
    completedLoops: 1,
    obstacle: { enabled: false },
  });
  assert.equal(finalLoop.state, "complete");
});

test("native navigation carries waypoint dwell time into the runner contract", () => {
  const command = nativeNavigationCommand({
    waypoints: [{ ...route[0], holdS: 2 }],
    position: { northM: 0, eastM: 0 },
    headingDeg: 0,
    obstacle: { enabled: false },
  });
  assert.equal(command.state, "arrived");
  assert.equal(command.holdS, 2);
});

test("native navigation stops for an obstacle and reports the reason", () => {
  const command = nativeNavigationCommand({
    waypoints: route,
    position: { northM: 0, eastM: 2 },
    headingDeg: 90,
    currentIndex: 1,
    obstacle: { enabled: true, frontM: 0.3, stopDistanceM: 0.45, slowDistanceM: 1.2 },
  });
  assert.equal(command.state, "obstacle-stop");
  assert.match(command.reason, /stop distance/);
  assert.equal(command.forward, 0);
});

test("native navigation stops when the target crosses the home-radius fence", () => {
  const command = nativeNavigationCommand({
    waypoints: route,
    position: { northM: 0, eastM: 0 },
    currentIndex: 1,
    obstacle: { enabled: false },
    geofence: { enabled: true, maxRadiusM: 5 },
  });
  assert.equal(command.state, "geofence-stop");
  assert.equal(command.forward, 0);
});

test("native navigation checks the home-radius fence before accepting arrival", () => {
  const command = nativeNavigationCommand({
    waypoints: [{ local: { northM: 0, eastM: 5.5 }, radiusM: 1.2, speedMps: 0.8 }],
    position: { northM: 0, eastM: 5.2 },
    headingDeg: 90,
    obstacle: { enabled: false },
    geofence: { enabled: true, maxRadiusM: 5 },
  });
  assert.equal(command.state, "geofence-stop");
  assert.equal(command.forward, 0);
  assert.match(command.reason, /outside the active home-radius geofence/);
});

test("native navigation refuses to invent control without a fresh obstacle range", () => {
  const command = nativeNavigationCommand({
    waypoints: route,
    position: { northM: 0, eastM: 2 },
    headingDeg: 90,
    currentIndex: 1,
    obstacle: { enabled: true },
  });
  assert.equal(command.state, "sensor-wait");
  assert.equal(command.forward, 0);
  assert.equal(headingErrorDeg(5, 355), 10);
});

test("native navigation refuses to invent a heading when heading telemetry is missing", () => {
  const command = nativeNavigationCommand({
    waypoints: route,
    position: { northM: 0, eastM: 2 },
    obstacle: { enabled: false },
  });
  assert.equal(command.state, "sensor-wait");
  assert.equal(command.forward, 0);
  assert.match(command.reason, /heading/);
});

test("native navigation biases away from a close side obstacle", () => {
  const command = nativeNavigationCommand({
    waypoints: route,
    position: { northM: 0, eastM: 2 },
    headingDeg: 90,
    currentIndex: 1,
    obstacle: { enabled: true, frontM: 5, leftM: 0.2, rightM: 4, stopDistanceM: 0.45, slowDistanceM: 1.2 },
  });
  assert.equal(command.state, "navigating");
  assert.ok(command.turn > 0);
});
