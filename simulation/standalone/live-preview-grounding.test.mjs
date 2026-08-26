import assert from "node:assert/strict";
import test from "node:test";

import { livePreviewGroundCorrection } from "./web/src/live-preview-grounding.js";

test("pairing lifts a live preview whose lowest foot would cross the floor", () => {
  const correction = livePreviewGroundCorrection([-0.12, -0.08, 0.01, 0.03], 0.002);
  assert.equal(correction, 0.122);
});

test("live grounding lowers a floating stance to the same floor reference", () => {
  const correction = livePreviewGroundCorrection([0.04, 0.05, 0.045, 0.052], 0.002);
  assert.equal(correction, -0.038);
});

test("missing foot probes do not move the live robot", () => {
  assert.equal(livePreviewGroundCorrection([null, undefined, Number.NaN], 0.002), 0);
});
