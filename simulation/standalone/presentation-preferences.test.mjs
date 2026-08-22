import test from "node:test";
import assert from "node:assert/strict";

import {
  parsePresentationPreferencesJson,
  presentationPreferencesJson,
  PRESENTATION_PREFERENCES_SCHEMA_VERSION,
  sanitizePresentationPreferences,
} from "./web/src/presentation-preferences.js";

test("round-trips only safe presentation preferences", () => {
  const restored = parsePresentationPreferencesJson(presentationPreferencesJson({
    experience: "expert",
    liveView: "diagnostics",
    calibrationFloat: false,
    robotState: "armed",
    authorityToken: "must-not-persist",
    benchModeAcknowledged: true,
  }));
  assert.deepEqual(restored, {
    schemaVersion: PRESENTATION_PREFERENCES_SCHEMA_VERSION,
    experience: "expert",
    liveView: "diagnostics",
    calibrationFloat: false,
  });
  assert.equal(Object.hasOwn(restored, "robotState"), false);
  assert.equal(Object.hasOwn(restored, "authorityToken"), false);
  assert.equal(Object.hasOwn(restored, "benchModeAcknowledged"), false);
});

test("malformed and unsupported preferences fail back to safe presentation defaults", () => {
  const expected = {
    schemaVersion: PRESENTATION_PREFERENCES_SCHEMA_VERSION,
    experience: "simple",
    liveView: "compare",
    calibrationFloat: true,
  };
  assert.deepEqual(parsePresentationPreferencesJson("not json"), expected);
  assert.deepEqual(sanitizePresentationPreferences({ schemaVersion: 9, experience: "expert" }), expected);
  assert.deepEqual(sanitizePresentationPreferences({
    schemaVersion: PRESENTATION_PREFERENCES_SCHEMA_VERSION,
    experience: "engineering",
    liveView: "secret",
    calibrationFloat: "yes",
  }), expected);
});
