import assert from "node:assert/strict";
import test from "node:test";

import { assemblyOrigin, legs, standServoReference } from "./web/src/domino-config.js";
import { point2, solveLinkagePose } from "./web/src/linkage.js";

const DEG = Math.PI / 180;

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

function rotationMatrix([rollDeg, pitchDeg, yawDeg]) {
  const [r, p, y] = [rollDeg, pitchDeg, yawDeg].map((value) => value * DEG);
  const [cr, sr, cp, sp, cy, sy] = [
    Math.cos(r), Math.sin(r), Math.cos(p), Math.sin(p), Math.cos(y), Math.sin(y),
  ];
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

function multiply(matrix, vector) {
  return matrix.map((row) =>
    row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2]);
}

function cadWorldFoot(leg, servoAngles, bodyPose) {
  const shoulderDelta =
    (servoAngles[leg.channels.shoulder] - standServoReference[leg.channels.shoulder]) /
    leg.directions.shoulder;
  const upperDelta =
    (servoAngles[leg.channels.upper] - standServoReference[leg.channels.upper]) /
    leg.directions.upper;
  const lowerDelta =
    (servoAngles[leg.channels.lower] - standServoReference[leg.channels.lower]) /
    leg.directions.lower;
  const offset = leg.neutralLinkageOffsetDeg || { upper: 0, lower: 0 };
  const solution = solveLinkagePose(
    leg.points,
    (-upperDelta + offset.upper) * DEG,
    (-lowerDelta + offset.lower) * DEG,
  );
  const planar = transformedFoot(leg.points, solution);
  const hip = leg.points.hip_origin;
  const localX = planar.x - hip[0];
  const localY = leg.points.foot_tip[1] - hip[1];
  const localZ = planar.z - hip[2];
  const shoulderAngle = -leg.shoulderSign * shoulderDelta * DEG;
  const rotatedY = Math.cos(shoulderAngle) * localY -
    Math.sin(shoulderAngle) * localZ;
  const rotatedZ = Math.sin(shoulderAngle) * localY +
    Math.cos(shoulderAngle) * localZ;
  const bodyPoint = [
    hip[0] + localX - assemblyOrigin[0],
    hip[1] + rotatedY - assemblyOrigin[1],
    hip[2] + rotatedZ - assemblyOrigin[2],
  ];
  return multiply(rotationMatrix(bodyPose), bodyPoint);
}

const cases = [
  {
    name: "forward pitch",
    pose: [0, 10, 0],
    servoAngles: [
      129.87, 192.51, 111.92, 105.44, 57.38, 0, 0, 155.25,
      116.10, 115.42, 74.52, 118.80, 0, 0, 119.88, 130.00,
    ],
  },
  {
    name: "backward pitch",
    pose: [0, -10, 0],
    servoAngles: [
      129.87, 146.88, 121.09, 105.44, 103.00, 0, 0, 145.53,
      81.41, 115.42, 84.38, 153.36, 0, 0, 119.88, 120.69,
    ],
  },
  {
    name: "positive yaw",
    pose: [0, 0, 25],
    servoAngles: [
      146.61, 178.60, 118.26, 92.88, 102.06, 0, 0, 161.19,
      114.34, 131.76, 86.53, 139.59, 0, 0, 107.73, 126.50,
    ],
  },
  {
    name: "negative yaw",
    pose: [0, 0, -25],
    servoAngles: [
      117.31, 147.69, 115.42, 122.17, 71.28, 0, 0, 143.37,
      95.31, 103.28, 68.72, 120.56, 0, 0, 136.22, 123.53,
    ],
  },
];

const neutralFeet = new Map(
  legs.map((leg) => [leg.label, cadWorldFoot(leg, standServoReference, [0, 0, 0])]),
);

for (const axisCase of cases) {
  test(`${axisCase.name} keeps every CAD foot planted`, () => {
    for (const leg of legs) {
      const neutral = neutralFeet.get(leg.label);
      const actual = cadWorldFoot(leg, axisCase.servoAngles, axisCase.pose);
      const errorMillimeters = Math.hypot(
        actual[0] - neutral[0],
        actual[1] - neutral[1],
        actual[2] - neutral[2],
      ) * 1000;
      assert.ok(
        errorMillimeters < 0.5,
        `${leg.label} moved ${errorMillimeters.toFixed(3)} mm in ${axisCase.name}`,
      );
    }
  });
}
