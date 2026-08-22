import test from "node:test";
import assert from "node:assert/strict";

import { gaitLabControls, gaitLabPresets } from "./web/src/gait-lab.js";
import { validLiveGaitAcknowledgement, validLiveGaitCommand, validLiveGaitProfile } from "./web/src/live-gait-protocol.js";
import {
  acceptRobotGaitProfile,
  createLiveGaitCommand,
  createLiveGaitProfile,
  createLiveGaitState,
  liveGaitCanApply,
  liveGaitCanApplyDraft,
  liveGaitDiff,
  liveGaitProfileJson,
  liveGaitRiskAssessment,
  parseLiveGaitProfileJson,
  readLiveGaitLibrary,
  selectLiveGaitPreset,
  updateLiveGaitDraft,
  updateLiveGaitPreviewAssessment,
} from "./web/src/live-gait-state.js";

test("simulation settings upgrade into a versioned cross-workspace profile", () => {
  const library = readLiveGaitLibrary({ Track: gaitLabPresets.fast });
  assert.equal(library.Track.schemaVersion, 2);
  assert.equal(library.Track.source, "simulation");
  assert.equal(Object.keys(library.Track.settings).length, gaitLabControls.length + 2);
});

test("draft settings remain bounded and preset selection is explicit", () => {
  const state = createLiveGaitState();
  updateLiveGaitDraft(state, { cadenceHz: 100, bodyHeightMm: 0 });
  assert.equal(state.draft.settings.cadenceHz, 2.5);
  assert.equal(state.draft.settings.bodyHeightMm, 220);
  assert.equal(state.draft.settings.preset, "custom");
  assert.equal(selectLiveGaitPreset(state, "stable"), true);
  assert.equal(state.draft.settings.preset, "stable");
});

test("robot comparison reports changed parameters and risk findings", () => {
  const state = createLiveGaitState(createLiveGaitProfile(gaitLabPresets.fast, "Fast"));
  acceptRobotGaitProfile(state, createLiveGaitProfile(gaitLabPresets.stable, "Robot safe"));
  assert.ok(liveGaitDiff(state).filter((entry) => entry.changed).length >= 5);
  assert.ok(liveGaitRiskAssessment(state.draft).some((finding) => finding.severity === "warning"));
});

test("repeated robot telemetry does not churn the profile or revert history", () => {
  const state = createLiveGaitState();
  const stable = createLiveGaitProfile(gaitLabPresets.stable, "Robot safe");
  assert.equal(acceptRobotGaitProfile(state, stable), true);
  assert.equal(acceptRobotGaitProfile(state, stable), false);
  assert.equal(state.previousRobotProfile, null);
});

test("robot apply requires disarm and advertised persistent apply support", () => {
  const state = createLiveGaitState();
  assert.equal(liveGaitCanApply(state), false);
  state.robotState = "disarmed";
  state.persistentApplySupported = true;
  assert.equal(liveGaitCanApply(state), true);
  const command = createLiveGaitCommand(state, "apply-profile", "gait-1", 123);
  assert.equal(validLiveGaitCommand(command), true);
  assert.equal(command.safety.twoStageApply, true);
  assert.equal(validLiveGaitCommand({ ...command, safety: {} }), false);
  assert.equal(validLiveGaitProfile(command.profile), true);
  assert.equal(validLiveGaitCommand({
    ...command,
    profile: { ...command.profile, settings: { ...command.profile.settings, strideMm: 121 } },
  }), false);
  assert.equal(validLiveGaitCommand({
    ...command,
    profile: { ...command.profile, settings: { ...command.profile.settings, cadenceHz: Number.NaN } },
  }), false);
});

test("draft apply requires a complete reachable and unclipped IK preview", () => {
  const state = createLiveGaitState();
  state.robotState = "disarmed";
  state.persistentApplySupported = true;
  assert.equal(liveGaitCanApplyDraft(state), false);
  const safeLeg = (leg) => ({ leg, reachable: true, limitedJoints: [] });
  assert.equal(updateLiveGaitPreviewAssessment(state, {
    legDetails: ["FL", "FR", "BL", "BR"].map(safeLeg),
  }), true);
  assert.equal(liveGaitCanApplyDraft(state), true);
  assert.equal(updateLiveGaitPreviewAssessment(state, {
    legDetails: [safeLeg("FL"), safeLeg("FR"), safeLeg("BL"), {
      leg: "BR", reachable: true, limitedJoints: ["lower"],
    }],
  }), false);
  assert.equal(liveGaitCanApplyDraft(state), false);
});

test("versioned and raw simulation JSON both import safely", () => {
  const profile = createLiveGaitProfile(gaitLabPresets.balanced, "Balanced test");
  const restored = parseLiveGaitProfileJson(liveGaitProfileJson(profile));
  assert.deepEqual(restored.settings, profile.settings);
  assert.equal(restored.name, profile.name);
  assert.equal(restored.source, "import");
  const raw = parseLiveGaitProfileJson(JSON.stringify(gaitLabPresets.stable));
  assert.equal(raw.name, "Imported simulation gait");
  const migrated = parseLiveGaitProfileJson(JSON.stringify({
    ...profile,
    schemaVersion: 1,
    settings: Object.fromEntries(Object.entries(profile.settings).filter(
      ([key]) => !["touchdownXMm", "maxForwardScale", "maxTurnScale"].includes(key),
    )),
  }));
  assert.equal(migrated.schemaVersion, 2);
  assert.equal(migrated.settings.touchdownXMm, -15.75);
  assert.equal(migrated.settings.maxForwardScale, 0.8);
  assert.throws(() => parseLiveGaitProfileJson('{"schemaVersion":9,"robot":"domino-esp32-quadruped"}'));
});

test("gait acknowledgement validator rejects unknown actions", () => {
  assert.equal(validLiveGaitAcknowledgement({ type: "live-gait-ack", action: "apply-profile", requestId: "1", accepted: true }), true);
  assert.equal(validLiveGaitAcknowledgement({ type: "live-gait-ack", action: "run-fast", requestId: "1", accepted: true }), false);
});
