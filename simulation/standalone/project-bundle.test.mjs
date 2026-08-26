import test from "node:test";
import assert from "node:assert/strict";

import {
  DOMINO_PROJECT_ROBOT,
  PROJECT_BUNDLE_SCHEMA_VERSION,
  PROJECT_BUNDLE_TYPE,
  createDominoProjectBundle,
  parseProjectBundleJson,
  projectBundleFileName,
  projectBundleJson,
  projectBundleSummary,
  sanitizeProjectName,
} from "./web/src/project-bundle.js";

function fixture() {
  return createDominoProjectBundle({
    name: "Bench / Trot: August",
    exportedAt: 123456,
    gaitLabSettings: { cadenceHz: 9, strideMm: 60 },
    gaitProfiles: {
      Test: { cadenceHz: 0.9, strideMm: 55 },
    },
    gamepadMappings: {
      "Xbox Fixture": { deadzone: 0.2, responseCurve: 2.2, forwardAxis: 3 },
    },
    liveGaitLibrary: {
      Balanced: { name: "Balanced", settings: { cadenceHz: 1.2, strideMm: 70 } },
    },
    liveGaitDraft: { name: "Bench Draft", settings: { cadenceHz: 0.7, strideMm: 45 } },
    navigationPlan: {
      missionName: "Bench patrol",
      missionDraft: [
        { local: { northM: 4, eastM: -2 }, radiusM: 1.8, speedMps: 0.4, holdS: 3, label: "Inspect" },
        { lat: -41.2, lon: 174.8, radiusM: 2.2, speedMps: 0.6, label: "Return" },
      ],
      plannerOrigin: { lat: -41.2, lon: 174.8 },
      plannerRangeM: 80,
      geofence: { enabled: true, maxRadiusM: 35 },
      obstacleBehavior: { enabled: true, stopDistanceM: 0.5, slowDistanceM: 1.6, maxSpeedMps: 0.7 },
    },
    calibrationProfile: {
      joints: Array.from({ length: 12 }, (_, channel) => ({
        logicalChannel: channel,
        channel,
        offsetDeg: channel === 0 ? 3.2 : 0,
      })),
    },
  });
}

test("project bundles contain the robot contract and portable configuration", () => {
  const bundle = fixture();
  assert.equal(bundle.type, PROJECT_BUNDLE_TYPE);
  assert.equal(bundle.schemaVersion, PROJECT_BUNDLE_SCHEMA_VERSION);
  assert.deepEqual(bundle.robot, DOMINO_PROJECT_ROBOT);
  assert.equal(bundle.project.name, "Bench / Trot: August");
  assert.equal(bundle.simulation.gaitLabSettings.cadenceHz, 2.5);
  assert.equal(bundle.simulation.gaitProfiles.Test.strideMm, 55);
  assert.equal(bundle.controller.gamepadMappings["Xbox Fixture"].responseCurve, 2.2);
  assert.equal(bundle.live.gaitLibrary.Balanced.settings.strideMm, 70);
  assert.equal(bundle.live.gaitDraft.name, "Bench Draft");
  assert.equal(bundle.live.calibration.joints[0].offsetDeg, 3.2);
  assert.equal(bundle.live.navigation.missionName, "Bench patrol");
  assert.equal(bundle.live.navigation.missionDraft.length, 2);
  assert.equal(bundle.live.navigation.missionDraft[0].holdS, 3);
  assert.equal(bundle.live.navigation.plannerRangeM, 80);
  assert.equal(bundle.live.navigation.geofence.maxRadiusM, 35);
  assert.equal("safety" in bundle, false);
  assert.equal("connection" in bundle, false);
  assert.equal("benchModeAcknowledged" in bundle.live, false);
});

test("project JSON round-trips with bounded summaries", () => {
  const original = fixture();
  const restored = parseProjectBundleJson(projectBundleJson(original));
  assert.deepEqual(restored, original);
  assert.deepEqual(projectBundleSummary(restored), {
    name: "Bench / Trot: August",
    gaitProfileCount: 1,
    liveGaitProfileCount: 1,
    gamepadMappingCount: 1,
    calibratedJointCount: 12,
    routeWaypointCount: 2,
  });
});

test("project bundle keeps older files compatible with an empty local route plan", () => {
  const bundle = fixture();
  delete bundle.live.navigation;
  const restored = parseProjectBundleJson(projectBundleJson(bundle));
  assert.deepEqual(restored.live.navigation.missionDraft, []);
  assert.equal(restored.live.navigation.geofence.enabled, false);
});

test("project parser rejects wrong files and unsafe physical mappings", () => {
  assert.throws(
    () => parseProjectBundleJson("not json"),
    /not valid JSON/,
  );
  assert.throws(
    () => parseProjectBundleJson(JSON.stringify({ type: "other", schemaVersion: 1 })),
    /not a Domino project bundle/,
  );

  const wrongRobot = fixture();
  wrongRobot.robot.id = "other-robot";
  assert.throws(() => parseProjectBundleJson(JSON.stringify(wrongRobot)), /different robot/);

  const duplicateChannel = fixture();
  duplicateChannel.live.calibration.joints[1].channel = 0;
  assert.throws(() => parseProjectBundleJson(JSON.stringify(duplicateChannel)), /assigned more than once/);
});

test("project names are filename-safe without changing the bundle contract", () => {
  assert.equal(sanitizeProjectName("  Dog: V2 / bench?  "), "Dog: V2 / bench?");
  assert.equal(projectBundleFileName("Dog: V2 / bench?"), "dog-v2-bench.qstudio.json");
  assert.equal(projectBundleFileName("***"), "domino-v2.qstudio.json");
});
