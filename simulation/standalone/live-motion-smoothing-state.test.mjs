import assert from "node:assert/strict";
import test from "node:test";

import {
  DEFAULT_MOTION_SMOOTHING,
  motionSmoothingDeadbandPreview,
  motionSmoothingDraft,
  motionSmoothingRampPreview,
  validMotionSmoothingDraft,
  validMotionSmoothingSettings,
} from "./web/src/live-motion-smoothing-state.js";

test("ramp preview follows the firmware's 20 ms rate limit", () => {
  const preview = motionSmoothingRampPreview(DEFAULT_MOTION_SMOOTHING, "rollRateDegPerSec", 16);
  assert.equal(preview.step, 3.6);
  assert.equal(preview.settleMs, 100);
  assert.equal(preview.points[1].value, 3.6);
  assert.equal(preview.points[5].value, 16);
  assert.equal(preview.points.at(-1).value, 16);
  const slower = motionSmoothingRampPreview(
    { ...DEFAULT_MOTION_SMOOTHING, rollRateDegPerSec: 40 }, "rollRateDegPerSec", 20,
  );
  assert.equal(slower.settleMs, 500);
  assert.equal(slower.points[1].value, 0.8);
});

test("deadband preview matches center shaping before the ramp", () => {
  const preview = motionSmoothingDeadbandPreview(0.02);
  assert.equal(preview.points[0].output, 0);
  assert.equal(preview.points[4].output, 0);
  assert.ok(Math.abs(preview.tenPercentOutput - (0.08 / 0.98)) < 1e-10);
  assert.equal(motionSmoothingDeadbandPreview(0.09), null);
});

test("turning smoothing off bypasses both the ramp and center deadband", () => {
  const disabled = { ...DEFAULT_MOTION_SMOOTHING, enabled: false };
  const preview = motionSmoothingRampPreview(disabled, "heightRateMmPerSec", 60);
  assert.equal(preview.settleMs, 20);
  assert.equal(preview.points[1].value, 60);
  assert.equal(preview.points.at(-1).value, 60);
  const deadband = motionSmoothingDeadbandPreview(disabled.tiltInputDeadband, disabled.enabled);
  assert.equal(deadband.points[4].output, 0.02);
  assert.equal(deadband.tenPercentOutput, 0.1);
  assert.equal(validMotionSmoothingSettings(disabled), true);
  assert.equal(validMotionSmoothingSettings({ ...disabled, enabled: 0 }), false);
});

test("older browser drafts retain tuned rates and migrate with smoothing on", () => {
  const legacy = { ...DEFAULT_MOTION_SMOOTHING, schemaVersion: 1, rollRateDegPerSec: 140 };
  delete legacy.enabled;
  assert.equal(validMotionSmoothingSettings(legacy), false);
  assert.equal(validMotionSmoothingDraft(legacy), true);
  const migrated = motionSmoothingDraft(legacy);
  assert.equal(migrated.schemaVersion, 2);
  assert.equal(migrated.enabled, true);
  assert.equal(migrated.rollRateDegPerSec, 140);
});
