import assert from "node:assert/strict";
import { test } from "node:test";
import {
  advanceCalibrationSweep,
  calibrationBoxGapMm,
  jointTravelMetrics,
  nudgeCalibrationInspectionPose,
  placeJointInspectionCard,
} from "./web/src/live-calibration-inspection.js";

test("the first preview nudge moves one degree from the visible pose, including a scrubbed pose", () => {
  assert.equal(nudgeCalibrationInspectionPose(-26.9, 1, -45, 45), -25.9);
  assert.equal(nudgeCalibrationInspectionPose(-25.9, 0.1, -45, 45), -25.8);
  assert.equal(nudgeCalibrationInspectionPose(13.4, -1, -20, 40), 12.4);
});

test("preview nudges stop at asymmetric configured limits and ignore invalid bounds", () => {
  assert.equal(nudgeCalibrationInspectionPose(19.8, 1, -7, 20), 20);
  assert.equal(nudgeCalibrationInspectionPose(-6.8, -1, -7, 20), -7);
  assert.equal(nudgeCalibrationInspectionPose(3.5, 1, 20, -7), 3.5);
  assert.equal(nudgeCalibrationInspectionPose(3.5, NaN, -7, 20), 3.5);
});

test("visual sweep reverses at configured limits, even across a long frame", () => {
  assert.deepEqual(advanceCalibrationSweep(8, 1, -10, 10, 0.1, 40), { position: 8, direction: -1 });
  assert.deepEqual(advanceCalibrationSweep(-8, -1, -10, 10, 0.1, 40), { position: -8, direction: 1 });
  const wrapped = advanceCalibrationSweep(0, 1, -10, 10, 1.5, 40);
  assert.equal(wrapped.position, 0);
  assert.equal(wrapped.direction, -1);
});

test("inspection uses asymmetric configured stops, including poses outside the draft", () => {
  const inside = jointTravelMetrics(12, -20, 40, 90);
  assert.equal(inside.minimumMargin, 32);
  assert.equal(inside.maximumMargin, 28);
  assert.equal(inside.state, "clear");
  assert.equal(jointTravelMetrics(38, -20, 40, 90).state, "near");
  assert.equal(jointTravelMetrics(40, -20, 40, 90).state, "stop");
  const outside = jointTravelMetrics(-22, -20, 40, 90);
  assert.equal(outside.state, "outside");
  assert.equal(outside.nearestStop, "MIN");
  assert.equal(outside.margin, -2);
  assert.equal(jointTravelMetrics(0, 5, 5, 30), null);
});

test("inspection card stays readable and inside the viewport when its joint leaves the frame", () => {
  const viewport = { width: 550, height: 400 };
  for (const anchor of [{ x: -1500, y: -2000 }, { x: 3000, y: 1500 }, { x: 270, y: 180 }]) {
    const card = placeJointInspectionCard(anchor, viewport, { width: 300, height: 270 });
    assert.equal(card.width, 300);
    assert.equal(card.height, 270);
    assert.ok(card.x >= 12 && card.x + card.width <= viewport.width - 12);
    assert.ok(card.y >= 12 && card.y + card.height <= viewport.height - 12);
  }
});

test("inspection avoids the camera gizmo and preserves its side near the center", () => {
  const viewport = { width: 800, height: 600 };
  const obstacle = { x: 0, y: 310, width: 330, height: 290 };
  const card = placeJointInspectionCard({ x: 680, y: 100 }, viewport,
    { width: 300, height: 270 }, "left", [obstacle]);
  assert.equal(card.side, "left");
  assert.equal(card.y, 12);
  assert.equal(placeJointInspectionCard({ x: 405, y: 300 }, viewport,
    { width: 300, height: 270 }, "right").side, "right");
});

test("CAD proximity uses three-dimensional separation between bounds", () => {
  const box = (min, max) => ({ min: { x: min[0], y: min[1], z: min[2] }, max: { x: max[0], y: max[1], z: max[2] } });
  assert.equal(calibrationBoxGapMm(box([0, 0, 0], [1, 1, 1]), box([0.5, 0.5, 0.5], [2, 2, 2])), 0);
  assert.equal(Math.round(calibrationBoxGapMm(box([0, 0, 0], [1, 1, 1]), box([1.003, 0, 0], [2, 1, 1]))), 3);
  assert.equal(Math.round(calibrationBoxGapMm(box([0, 0, 0], [1, 1, 1]), box([1.003, 1.004, 0], [2, 2, 1]))), 5);
});
