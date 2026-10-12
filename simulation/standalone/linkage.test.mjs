import assert from "node:assert/strict";
import test from "node:test";

import { legs } from "./web/src/domino-config.js";
import { distance2, point2, solveLinkagePose } from "./web/src/linkage.js";

const sitDeltas = {
  FR: { upper: 20.30, lower: -37.20 },
  FL: { upper: 20.30, lower: -37.20 },
  BL: { upper: 20.30, lower: -37.20 },
  BR: { upper: 20.30, lower: -37.20 },
};

function assertLength(actualA, actualB, originalA, originalB) {
  assert.ok(
    Math.abs(distance2(actualA, actualB) - distance2(point2(originalA), point2(originalB))) < 1e-6,
  );
}

function solveLegPose(leg, upperDeltaDeg, lowerDeltaDeg) {
  const offset = leg.neutralLinkageOffsetDeg || { upper: 0, lower: 0 };
  return solveLinkagePose(
    leg.points,
    (upperDeltaDeg + offset.upper) * Math.PI / 180,
    (lowerDeltaDeg + offset.lower) * Math.PI / 180,
  );
}

test("sit poses keep every closed-linkage pin connected", () => {
  for (const leg of legs) {
    const deltas = sitDeltas[leg.label];
    const solution = solveLegPose(leg, deltas.upper, deltas.lower);

    assertLength(
      solution.lowerPassive,
      solution.upperClosureCoupler,
      leg.points.lower_passive,
      leg.points.upper_closure_coupler,
    );
    assertLength(
      solution.upperClosureCoupler,
      solution.upperClosureDriver,
      leg.points.upper_closure_coupler,
      leg.points.upper_closure_driver,
    );
    assertLength(
      solution.lowerCoupler,
      solution.lowerClosureDiagonal,
      leg.points.lower_coupler,
      leg.points.lower_closure_driver,
    );
    assertLength(
      solution.lowerClosureDriver,
      solution.lowerClosureDiagonal,
      leg.points.lower_closure_diagonal,
      leg.points.lower_closure_driver,
    );
  }
});

function transformedFoot(points, solution) {
  const originalA = point2(points.lower_closure_diagonal);
  const originalB = point2(points.lower_closure_driver);
  const foot = point2(points.foot_tip);
  const originalAngle = Math.atan2(originalB.z - originalA.z, originalB.x - originalA.x);
  const currentAngle = Math.atan2(
    solution.lowerClosureDiagonal.z - solution.lowerClosureDriver.z,
    solution.lowerClosureDiagonal.x - solution.lowerClosureDriver.x,
  );
  const angle = currentAngle - originalAngle;
  const localX = foot.x - originalA.x;
  const localZ = foot.z - originalA.z;
  return {
    x: solution.lowerClosureDriver.x +
      Math.cos(angle) * localX -
      Math.sin(angle) * localZ,
    z: solution.lowerClosureDriver.z +
      Math.sin(angle) * localX +
      Math.cos(angle) * localZ,
  };
}

test("deep sit raises each CAD foot with bounded longitudinal drift", () => {
  for (const leg of legs) {
    const neutral = solveLegPose(leg, 0, 0);
    const deltas = sitDeltas[leg.label];
    const sitting = solveLegPose(leg, deltas.upper, deltas.lower);
    const neutralFoot = transformedFoot(leg.points, neutral);
    const sittingFoot = transformedFoot(leg.points, sitting);

    assert.ok(Math.abs(sittingFoot.x - neutralFoot.x) < 0.017);
    assert.ok(sittingFoot.z - neutralFoot.z > 0.105);
    assert.ok(sitting.commandScale > 0.999);
    assert.equal(sitting.limited, false);
    assert.ok(Math.abs(deltas.upper) < 25);
    assert.ok(Math.abs(deltas.lower) < 40);
  }
});

test("deep sit transition stays on the authored four-bar assembly branch", () => {
  const lowUpperDelta = 20.30 * Math.PI / 180;
  const lowLowerDelta = -37.20 * Math.PI / 180;

  for (const leg of legs) {
    const neutralFoot = transformedFoot(leg.points, solveLegPose(leg, 0, 0));
    let previousFoot = neutralFoot;
    const offset = leg.neutralLinkageOffsetDeg || { upper: 0, lower: 0 };

    for (let step = 1; step <= 20; step += 1) {
      const scale = step / 20;
      const solution = solveLinkagePose(
        leg.points,
        lowUpperDelta * scale + offset.upper * Math.PI / 180,
        lowLowerDelta * scale + offset.lower * Math.PI / 180,
      );
      const foot = transformedFoot(leg.points, solution);

      assert.ok(distance2(previousFoot, foot) < 0.008);
      previousFoot = foot;
    }

    assert.ok(Math.abs(previousFoot.x - neutralFoot.x) < 0.017);
    assert.ok(previousFoot.z - neutralFoot.z > 0.105);
  }
});

test("lower links preserve the exported mesh endpoint order", () => {
  for (const leg of legs) {
    const solution = solveLinkagePose(leg.points, 0, 0);

    assert.ok(
      distance2(solution.lowerClosureDriver, point2(leg.points.lower_closure_diagonal)) < 1e-9,
    );
    assert.ok(
      distance2(solution.lowerClosureDiagonal, point2(leg.points.lower_closure_driver)) < 1e-9,
    );
  }
});

test("front and rear CAD endpoints match on each physical side", () => {
  const pairs = [["FR", "BR"], ["FL", "BL"]];
  for (const [frontLabel, rearLabel] of pairs) {
    const front = legs.find((leg) => leg.label === frontLabel);
    const rear = legs.find((leg) => leg.label === rearLabel);
    const frontLocal = point2(front.points.foot_tip);
    const rearLocal = point2(rear.points.foot_tip);

    assert.ok(Math.abs(frontLocal.z - rearLocal.z) < 1e-9);
    assert.ok(
      Math.abs(
        (frontLocal.x - front.points.hip_origin[0]) -
        (rearLocal.x - rear.points.hip_origin[0]),
      ) < 1e-9,
    );
  }
});

test("calibrated left and right feet are symmetric in stand and sit", () => {
  const pairs = [["FR", "FL"], ["BR", "BL"]];
  for (const [rightLabel, leftLabel] of pairs) {
    const right = legs.find((leg) => leg.label === rightLabel);
    const left = legs.find((leg) => leg.label === leftLabel);

    for (const deltas of [{ upper: 0, lower: 0 }, sitDeltas[rightLabel]]) {
      const rightFoot = transformedFoot(
        right.points,
        solveLegPose(right, deltas.upper, deltas.lower),
      );
      const leftFoot = transformedFoot(
        left.points,
        solveLegPose(left, deltas.upper, deltas.lower),
      );
      const rightLocalX = rightFoot.x - right.points.hip_origin[0];
      const leftLocalX = leftFoot.x - left.points.hip_origin[0];

      assert.ok(Math.abs(rightFoot.z - leftFoot.z) < 0.00025);
      assert.ok(Math.abs(rightLocalX - leftLocalX) < 0.00025);
    }
  }
});

test("all passive pivots use the measured STL hole centres", () => {
  const expectedBySide = {
    right: {
      p1: { x: -0.012, z: -0.0105 },
      p2: { x: 0.012, z: 0.0505 },
      p3: { x: -0.012013, z: 0.0295 },
      p5: { x: -0.032011, z: 0.024135 },
      p4: { x: -0.144696, z: -0.089257 },
      p6: { x: -0.124685, z: -0.123892 },
    },
    left: {
      p1: { x: -0.012, z: -0.0105 },
      p2: { x: 0.012, z: 0.0505 },
      p3: { x: -0.012013, z: 0.0295 },
      p5: { x: -0.032011, z: 0.024135 },
      p4: { x: -0.152668, z: -0.080732 },
      p6: { x: -0.132657, z: -0.115367 },
    },
  };

  for (const leg of legs) {
    const side = leg.shoulderSign < 0 ? expectedBySide.right : expectedBySide.left;
    const translationX = leg.sourceTranslation[0];
    const p1 = point2(leg.points.lower_passive);
    const p2 = point2(leg.points.upper_closure_driver);
    const p3 = point2(leg.points.upper_closure_coupler);
    const p5 = point2(leg.points.lower_coupler);
    const p4 = point2(leg.points.lower_closure_driver);
    const p6 = point2(leg.points.lower_closure_diagonal);

    assert.ok(Math.abs(p1.x - (side.p1.x + translationX)) < 1e-9);
    assert.ok(Math.abs(p1.z - side.p1.z) < 1e-9);
    assert.ok(Math.abs(p2.x - (side.p2.x + translationX)) < 1e-9);
    assert.ok(Math.abs(p2.z - side.p2.z) < 1e-9);
    assert.ok(Math.abs(p3.x - (side.p3.x + translationX)) < 1e-9);
    assert.ok(Math.abs(p3.z - side.p3.z) < 1e-9);
    assert.ok(Math.abs(p5.x - (side.p5.x + translationX)) < 1e-9);
    assert.ok(Math.abs(p5.z - side.p5.z) < 1e-9);
    assert.ok(Math.abs(p4.x - (side.p4.x + translationX)) < 1e-9);
    assert.ok(Math.abs(p4.z - side.p4.z) < 1e-9);
    assert.ok(Math.abs(p6.x - (side.p6.x + translationX)) < 1e-9);
    assert.ok(Math.abs(p6.z - side.p6.z) < 1e-9);
  }
});
