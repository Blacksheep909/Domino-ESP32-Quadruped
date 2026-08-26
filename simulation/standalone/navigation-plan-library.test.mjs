import assert from "node:assert/strict";
import test from "node:test";

import {
  MAX_NAVIGATION_PLAN_LIBRARY_ENTRIES,
  navigationPlanLibraryJson,
  parseNavigationPlanLibraryJson,
  removeNavigationPlanLibraryEntry,
  upsertNavigationPlanLibraryEntry,
} from "./web/src/navigation-plan-library.js";

const plan = (name, northM = 4) => ({
  missionName: name,
  missionDraft: [{ local: { northM, eastM: 2 }, radiusM: 1.5, speedMps: 0.4, holdS: 2, label: "Inspect" }],
  plannerOrigin: { lat: -41.2, lon: 174.8 },
  plannerRangeM: 40,
  geofence: { enabled: true, maxRadiusM: 30 },
  obstacleBehavior: { enabled: true, stopDistanceM: 0.5, slowDistanceM: 1.4, maxSpeedMps: 0.6 },
});

test("route library saves named plans newest first and replaces the same name", () => {
  let entries = upsertNavigationPlanLibraryEntry([], "Morning patrol", plan("Morning patrol"), 100);
  entries = upsertNavigationPlanLibraryEntry(entries, "Evening patrol", plan("Evening patrol", 8), 200);
  entries = upsertNavigationPlanLibraryEntry(entries, "morning patrol", plan("Morning patrol", 12), 300);
  assert.deepEqual(entries.map((entry) => entry.name), ["morning patrol", "Evening patrol"]);
  assert.equal(entries[0].plan.mission[0].local.northM, 12);
});

test("route library round-trips bounded plan settings", () => {
  const entries = upsertNavigationPlanLibraryEntry([], "Bench route", plan("Bench route"), 1234);
  const restored = parseNavigationPlanLibraryJson(navigationPlanLibraryJson(entries), 9999);
  assert.deepEqual(restored, entries);
  assert.equal(restored[0].plan.geofence.maxRadiusM, 30);
  assert.equal(restored[0].plan.mission[0].holdS, 2);
});

test("route library rejects invalid JSON and oversized collections", () => {
  assert.throws(() => parseNavigationPlanLibraryJson("nope"), /not valid JSON/);
  const oversized = Array.from({ length: MAX_NAVIGATION_PLAN_LIBRARY_ENTRIES + 1 }, (_, index) => ({
    name: `route-${index}`,
    plan: plan(`route-${index}`),
  }));
  assert.throws(() => parseNavigationPlanLibraryJson(JSON.stringify({ schemaVersion: 1, plans: oversized })), /up to/);
});

test("route library deletes only the selected name", () => {
  const entries = [
    { name: "Alpha", savedAt: 1, plan: plan("Alpha") },
    { name: "Beta", savedAt: 2, plan: plan("Beta") },
  ];
  const remaining = removeNavigationPlanLibraryEntry(entries, "alpha");
  assert.deepEqual(remaining.map((entry) => entry.name), ["Beta"]);
});

