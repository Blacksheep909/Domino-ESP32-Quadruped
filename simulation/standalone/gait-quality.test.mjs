import test from "node:test";
import assert from "node:assert/strict";

import { gaitLabPresets, rightStickIntent, splitRightStickX, trotSwingPath } from "./web/src/gait-lab.js";
import { measureGait, measureSpin } from "./gait-quality.mjs";

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

// Left-stick spin on the spot. The old Balanced turned at ~10 deg/s (40% of
// what it commanded) and wandered ~100 mm off its spot.
test("Balanced spins on the spot quickly without wandering off it", async () => {
  for (const turn of [1, -1]) {
    const result = await measureSpin({ settings: gaitLabPresets.balanced, turn, seconds: 6 });
    assert.equal(result.fell, false);
    assert.ok(result.yawDegPerSec > 28, `spun ${result.yawDegPerSec} deg/s (turn ${turn})`);
    assert.ok(result.yawDegPerSec > 0.55 * result.commandedYawDegPerSec,
      `reached ${result.yawDegPerSec} of ${result.commandedYawDegPerSec} deg/s (turn ${turn})`);
    assert.ok(result.wanderMm < 60, `wandered ${result.wanderMm} mm off the spot (turn ${turn})`);
  }
});

// Right stick X: sidestep at a standstill, handing over to steering as the
// forward stick grows (blend 1), or always sidestep (blend 0, Spot-style).
test("right stick X sidesteps when standing and steers when walking", () => {
  const standing = splitRightStickX(0, 1, 1);
  assert.equal(standing.steer, 0);
  assert.equal(standing.strafe, 1);
  const walking = splitRightStickX(1, 1, 1);
  assert.equal(walking.strafe, 0);
  assert.equal(walking.steer, 1);
  const halfway = splitRightStickX(0.2, -1, 1);
  assert.ok(halfway.steer < 0 && halfway.strafe < 0, "both shares follow the stick direction");
  const spot = splitRightStickX(1, 0.7, 0);
  assert.equal(spot.steer, 0);
  assert.equal(spot.strafe, 0.7);
});

test("Balanced sidesteps both ways at a useful speed", async () => {
  const right = await measureGait({ settings: gaitLabPresets.balanced, forward: 0, steer: 1, seconds: 6 });
  const left = await measureGait({ settings: gaitLabPresets.balanced, forward: 0, steer: -1, seconds: 6 });
  for (const [name, result, sign] of [["right", right, 1], ["left", left, -1]]) {
    assert.equal(result.fell, false, name);
    const sideways = sign * result.sidewaysSignedMm / 6;
    assert.ok(sideways > 50, `${name} sidestep ${sideways} mm/s`);
    assert.ok(Math.abs(result.speedMmPerSec) < 30, `${name} sidestep drifted ${result.speedMmPerSec} mm/s fore/aft`);
    assert.ok(Math.sign(result.commandedRightMmPerSec) === sign, `${name} commanded the right way`);
  }
});

// The right stick does not have to be pushed perfectly sideways or forward.
test("a mostly sideways push is a pure sidestep; a mostly forward push does not crab", () => {
  assert.equal(rightStickIntent(0.25, 1).forward, 0, "15 deg off sideways drops the forward part");
  assert.ok(rightStickIntent(0.7, 0.7).forward > 0.69, "a real diagonal keeps its forward part");
  assert.equal(rightStickIntent(1, 0.1).strafeWeight, 0, "6 deg off forward drops the sidestep part");
  assert.ok(rightStickIntent(0.7, 0.7).strafeWeight > 0.99, "a real diagonal keeps its sidestep part");
  const steering = splitRightStickX(1, 0.1, 1, rightStickIntent(1, 0.1).strafeWeight);
  assert.equal(steering.steer, 0.1, "a small nudge while walking still steers");
});

test("an imperfect sideways push still sidesteps without walking forward", async () => {
  const result = await measureGait({ settings: gaitLabPresets.balanced, forward: 0.25, steer: 1, seconds: 6 });
  assert.equal(result.fell, false);
  assert.ok(result.sidewaysSignedMm / 6 > 50, `sidestep ${result.sidewaysSignedMm / 6} mm/s`);
  assert.ok(Math.abs(result.speedMmPerSec) < 15, `walked ${result.speedMmPerSec} mm/s fore/aft`);
});
