import assert from "node:assert/strict";
import test from "node:test";

import { formatAxisValue, niceLinearScale } from "./web/src/chart-axis.js";

test("builds readable ticks that fully contain the visible data range", () => {
  const scale = niceLinearScale(3.17, 16.84, 4);
  assert.deepEqual(scale.ticks, [0, 5, 10, 15, 20]);
  assert.ok(scale.minimum <= 3.17);
  assert.ok(scale.maximum >= 16.84);
});

test("small and negative ranges retain meaningful precision and zero", () => {
  const scale = niceLinearScale(-0.018, 0.034, 4);
  assert.ok(scale.ticks.includes(0));
  assert.equal(formatAxisValue(scale.ticks[0], scale.step), "-0.02");
  assert.equal(formatAxisValue(0, scale.step), "0.00");
});

test("constant ranges expand instead of producing an unusable axis", () => {
  const scale = niceLinearScale(12, 12, 4);
  assert.ok(scale.maximum > scale.minimum);
  assert.ok(scale.ticks.length >= 3);
});
