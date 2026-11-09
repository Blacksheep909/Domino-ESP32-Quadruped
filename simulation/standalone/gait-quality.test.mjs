import test from "node:test";
import assert from "node:assert/strict";

import { gaitLabPresets, trotSwingPath } from "./web/src/gait-lab.js";
import { measureGait } from "./gait-quality.mjs";

// The swing foot must leave and land moving with the ground (rearward at the
// stance speed), not stopped relative to the body: that mismatch is what
// skidded the feet at every touchdown.
test("the swing foot lifts off and touches down moving with the ground", () => {
  const halfStride = 40;
  const duty = 0.62;
  const stanceSpeed = -2 * halfStride * (1 - duty) / duty;  // per unit of swing
  const step = 1e-4;
  const startSpeed = (trotSwingPath(step, halfStride, 45, duty, 1.7).x - trotSwingPath(0, halfStride, 45, duty, 1.7).x) / step;
  const endSpeed = (trotSwingPath(1, halfStride, 45, duty, 1.7).x - trotSwingPath(1 - step, halfStride, 45, duty, 1.7).x) / step;
  assert.ok(Math.abs(startSpeed - stanceSpeed) < 0.05 * Math.abs(stanceSpeed), `lift-off speed ${startSpeed} vs ${stanceSpeed}`);
  assert.ok(Math.abs(endSpeed - stanceSpeed) < 0.05 * Math.abs(stanceSpeed), `touchdown speed ${endSpeed} vs ${stanceSpeed}`);
  assert.equal(trotSwingPath(0, halfStride, 45, duty, 1.7).x, -halfStride);
  assert.equal(trotSwingPath(1, halfStride, 45, duty, 1.7).x, halfStride);
  // Clear of the floor quickly: 20% into the swing the foot is above 70% of
  // its lift height (the old sin^1.7 curve was at 40%).
  assert.ok(-trotSwingPath(0.2, halfStride, 45, duty, 1.7).z > 0.7 * 45);
});

// With the display assists off the legs carry the robot and feet can slip.
// The old Balanced preset reached ~30% of its commanded speed and drifted
// ~60 mm sideways per metre.
test("Balanced walks forward at speed, straight, without the feet skating", async () => {
  const result = await measureGait({ settings: gaitLabPresets.balanced, forward: 1, seconds: 6 });
  const metres = result.speedMmPerSec * 6 / 1000;
  assert.equal(result.fell, false);
  assert.ok(result.speedMmPerSec > 90, `speed ${result.speedMmPerSec} mm/s`);
  assert.ok(result.speedMmPerSec > 0.5 * result.commandedMmPerSec, `reached ${result.speedMmPerSec} of ${result.commandedMmPerSec} mm/s`);
  assert.ok(Math.abs(result.sidewaysSignedMm) / metres < 40, `drifted ${result.sidewaysSignedMm} mm over ${metres} m`);
  assert.ok(Math.abs(result.yawSignedDeg) / metres < 6, `turned ${result.yawSignedDeg} deg over ${metres} m`);
  assert.ok(result.slipSidewaysMmPerSec < 15, `planted feet slid sideways ${result.slipSidewaysMmPerSec} mm/s`);
  assert.ok(result.rollSwingDeg < 8, `body rolled ${result.rollSwingDeg} deg`);
});
