import assert from "node:assert/strict";
import test from "node:test";

import {
  LIVE_FLOAT_MINIMUM_HEIGHT_MM,
  livePreviewBodyPose,
} from "./web/src/live-float-state.js";

test("live float raises the body while retaining changing attitude telemetry", () => {
  const source = { heightMm: 221, rollDeg: 12, pitchDeg: -4, yawDeg: 37 };
  assert.deepEqual(livePreviewBodyPose(source, true, 380), {
    ...source,
    heightMm: 380,
  });
});

test("live float enforces a visible minimum height", () => {
  assert.equal(
    livePreviewBodyPose({ heightMm: 120 }, true, 200).heightMm,
    LIVE_FLOAT_MINIMUM_HEIGHT_MM,
  );
});

test("floor mode leaves the incoming pose untouched", () => {
  const source = { heightMm: 245, rollDeg: 3 };
  assert.equal(livePreviewBodyPose(source, false), source);
});
