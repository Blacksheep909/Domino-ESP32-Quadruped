import assert from "node:assert/strict";
import test from "node:test";

import {
  batteryPacks,
  dominoMassModel,
  legs,
  standServoReference,
} from "./web/src/domino-config.js";
import { contactSurfaceError, createDominoPhysics } from "./web/src/physics.js";
import { logSpecs, terrainSpecs } from "./web/src/course-config.js";
import { createVoronoiTerrain } from "./web/src/voronoi-terrain.js";

test("course obstacles fit Domino's nominal gait envelope", () => {
  const ramp = terrainSpecs.find((spec) => spec.id === "ramp");
  const platform = terrainSpecs.find((spec) => spec.id === "ramp-platform");
  const rampHighEdge = ramp.position[1] +
    Math.sin(ramp.slope) * ramp.size[0] / 2 +
    Math.cos(ramp.slope) * ramp.size[1] / 2;
  const platformTop = platform.position[1] + platform.size[1] / 2;
  assert.ok(
    Math.abs(rampHighEdge - platformTop) < 0.004,
    `ramp/platform seam differs by ${Math.abs(rampHighEdge - platformTop) * 1000} mm`,
  );
  const steps = terrainSpecs.filter((spec) => spec.id.startsWith("step-"));
  assert.equal(steps.length, 4);
  const stepTops = steps.map((step) => step.position[1] + step.size[1] / 2);
  stepTops.forEach((height, index) => {
    const previousHeight = index === 0 ? 0 : stepTops[index - 1];
    assert.ok(
      height - previousHeight <= 0.03,
      `step ${index + 1} rises ${(height - previousHeight) * 1000} mm`,
    );
  });
  assert.ok(Math.max(...logSpecs.map((spec) => spec.radius)) <= 0.045);
  assert.ok(
    terrainSpecs
      .filter((spec) => spec.id.startsWith("stepping-block"))
      .every((spec) => spec.position[1] + spec.size[1] / 2 <= 0.10),
  );
});

test("Voronoi terrain has physical relief within the gait envelope", () => {
  const spec = terrainSpecs.find((terrain) => terrain.kind === "voronoi");
  const terrain = createVoronoiTerrain(spec);
  const heights = [];
  for (let index = 1; index < terrain.vertices.length; index += 3) {
    heights.push(terrain.vertices[index]);
  }
  const baseTop = spec.position[1] + spec.size[1] / 2;
  assert.equal(terrain.cellRanges.length, 12);
  assert.ok(Math.max(...heights) - baseTop >= 0.06);
  assert.ok(Math.max(...heights) - baseTop <= 0.075);
  assert.ok(Math.min(...heights) >= baseTop - 1e-6);
  assert.ok(terrain.indices.length >= 72);
});

test("CAD contact error is relative to elevated terrain, not world zero", () => {
  assert.ok(Math.abs(contactSurfaceError(0.142, 0.012, 0.154, 0.024)) < 1e-12);
  assert.ok(
    Math.abs(contactSurfaceError(0.137, 0.012, 0.154, 0.024) + 0.005) < 1e-12,
  );
});

test("mass model includes two CNHL 1500 mAh 4S packs", async () => {
  assert.equal(batteryPacks.length, 2);
  assert.equal(dominoMassModel.packMassKg, 0.183);
  assert.equal(dominoMassModel.batteryMassKg, 0.366);
  assert.ok(Math.abs(dominoMassModel.totalMassKg - 2.966) < 1e-9);

  const physics = await createDominoPhysics();
  const state = physics.update(0, null, legs, standServoReference);
  assert.equal(state.massModel, dominoMassModel);
});

test("dynamic course balls settle and remain stable across robot resets", async () => {
  const physics = await createDominoPhysics();
  const neutralStand = {
    mode: "STAND",
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, 280],
      [-15.75, -38, 280],
      [-15.75, 38, 280],
      [-15.75, -38, 280],
    ],
  };
  const expectedBalls = new Map([
    ["ball-a", { radius: 0.10, x: 0.55, z: 1.12 }],
    ["ball-b", { radius: 0.075, x: -0.45, z: 1.12 }],
  ]);

  let state;
  for (let frame = 0; frame < 600; frame += 1) {
    state = physics.update(1 / 120, neutralStand, legs, standServoReference);
  }

  assert.equal(state.environmentBalls.length, 2);
  const settledPositions = new Map();
  for (const ball of state.environmentBalls) {
    const expected = expectedBalls.get(ball.id);
    assert.ok(expected, `unexpected environment ball ${ball.id}`);
    assert.ok(ball.position.every(Number.isFinite), `${ball.id} position became non-finite`);
    assert.ok(
      Math.abs(ball.position[1] - expected.radius) < 0.01,
      `${ball.id} settled at y=${ball.position[1]} instead of radius ${expected.radius}`,
    );
    assert.ok(
      Math.hypot(ball.position[0] - expected.x, ball.position[2] - expected.z) < 0.03,
      `${ball.id} drifted from its spawn: ${JSON.stringify(ball.position)}`,
    );
    assert.ok(
      Math.abs(ball.linearVelocity[1]) < 0.01,
      `${ball.id} retained ${ball.linearVelocity[1]} m/s vertical velocity`,
    );
    settledPositions.set(ball.id, [...ball.position]);
  }

  for (let resetIndex = 0; resetIndex < 3; resetIndex += 1) {
    physics.reset(`ball-persistence-${resetIndex}`);
    state = physics.update(1 / 120, neutralStand, legs, standServoReference);
    for (const ball of state.environmentBalls) {
      const previous = settledPositions.get(ball.id);
      const displacement = Math.hypot(
        ball.position[0] - previous[0],
        ball.position[1] - previous[1],
        ball.position[2] - previous[2],
      );
      assert.ok(
        displacement < 0.002,
        `${ball.id} jumped ${displacement} m during robot reset`,
      );
      settledPositions.set(ball.id, [...ball.position]);
    }
  }
});

test("neutral stand settles level on four feet", async () => {
  const physics = await createDominoPhysics();
  const firmwareState = {
    mode: "STAND",
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, 280],
      [-15.75, -38, 280],
      [-15.75, 38, 280],
      [-15.75, -38, 280],
    ],
  };

  let state;
  for (let frame = 0; frame < 900; frame += 1) {
    state = physics.update(1 / 120, firmwareState, legs, standServoReference);
  }

  assert.equal(
    state.contactCount,
    4,
    `contacts=${state.contactCount} feet=${JSON.stringify(state.footPositions)}`,
  );
  assert.equal(state.resetCount, 0);
  assert.ok(state.baseTiltDegrees < 0.75, `base tilt was ${state.baseTiltDegrees} deg`);
});

test("tilt entry preserves elevated foot endpoints and support-relative height", async () => {
  const platform = terrainSpecs.find((spec) => spec.id === "ramp-platform");
  const platformTop = platform.position[1] + platform.size[1] / 2;
  const physics = await createDominoPhysics({
    initialBasePosition: [platform.position[0], 0.304 + platformTop, platform.position[2]],
  });
  const firmwareState = {
    mode: "STAND",
    pose_z_mm: 280,
    body_pose_rpy_deg: [0, 0, 0],
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, 280],
      [-15.75, -38, 280],
      [-15.75, 38, 280],
      [-15.75, -38, 280],
    ],
  };

  let state;
  for (let frame = 0; frame < 600; frame += 1) {
    state = physics.update(1 / 120, firmwareState, legs, standServoReference);
  }
  assert.equal(state.contactCount, 4);
  const standBaseHeight = state.basePosition[1];
  const standFeet = state.footPositions.map((position) => [...position]);

  firmwareState.mode = "TILT";
  firmwareState.body_pose_rpy_deg = [0, 6, 0];
  state = physics.update(1 / 120, firmwareState, legs, standServoReference);
  assert.ok(state.tiltFootAnchors, "tilt did not capture foot endpoints");
  state.tiltFootAnchors.forEach((anchor, index) => {
    assert.ok(anchor[1] > platformTop, `foot ${index} anchor fell below platform top`);
    assert.ok(
      Math.abs(anchor[1] - standFeet[index][1]) < 0.003,
      `foot ${index} anchor moved on tilt entry`,
    );
  });
  assert.ok(
    Math.abs(state.commandedBodyHeight - standBaseHeight) < 0.004,
    `tilt rebased body height from ${standBaseHeight} to ${state.commandedBodyHeight}`,
  );

  for (let frame = 0; frame < 360; frame += 1) {
    state = physics.update(1 / 120, firmwareState, legs, standServoReference);
  }
  assert.equal(state.resetCount, 0, `unexpected ${state.lastResetReason} reset`);
  assert.ok(state.basePosition[1] > platformTop + 0.24);
  assert.ok(
    state.bodyClearance > 0.26 && state.bodyClearance < 0.34,
    `support-relative body height was ${state.bodyClearance}`,
  );
  assert.ok(Math.abs(state.supportHeight - platformTop) < 0.01);
  state.footPositions.forEach((position, index) => {
    assert.ok(
      position[1] > platformTop + state.footRadius * 0.65,
      `foot ${index} dropped through the elevated platform`,
    );
  });
});

test("deep sit lowers the chassis onto four stable feet", async () => {
  const physics = await createDominoPhysics();
  const firmwareState = {
    mode: "STOW",
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, 160],
      [-15.75, -38, 160],
      [-15.75, 38, 160],
      [-15.75, -38, 160],
    ],
  };

  let state;
  for (let frame = 0; frame < 900; frame += 1) {
    state = physics.update(1 / 120, firmwareState, legs, standServoReference);
  }

  assert.equal(state.contactCount, 4);
  assert.equal(state.resetCount, 0);
  assert.ok(state.bodyHeight > 0.16 && state.bodyHeight < 0.20);
  assert.ok(state.baseTiltDegrees < 1, `base tilt was ${state.baseTiltDegrees} deg`);
});

test("diagonal sinusoidal gait stays supported and produces planar travel", async () => {
  const physics = await createDominoPhysics();
  const firmwareState = {
    mode: "STAND",
    pose_z_mm: 280,
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, 280],
      [-15.75, -38, 280],
      [-15.75, 38, 280],
      [-15.75, -38, 280],
    ],
  };

  let state;
  for (let frame = 0; frame < 480; frame += 1) {
    state = physics.update(1 / 120, firmwareState, legs, standServoReference);
  }
  const startPosition = [...state.basePosition];
  let minimumContacts = 4;
  let maximumTilt = 0;
  let plantedSlipDistance = 0;
  let plantedContactSeconds = 0;
  let bodyPathDistance = 0;
  let previousGaitState = null;

  firmwareState.mode = "GAIT";
  for (let frame = 0; frame < 1200; frame += 1) {
    const gaitBodyZ = Math.max(265, 280 - 80 * (frame / 120));
    firmwareState.pose_z_mm = gaitBodyZ;
    const gaitCycle = (0.65 * (frame / 120)) % 1;
    firmwareState.leg_command_xyz_mm = [0, 1, 2, 3].map((legIndex) => {
      const diagonalA = legIndex === 0 || legIndex === 3;
      const leftLeg = legIndex === 0 || legIndex === 2;
      const cycle = (gaitCycle + (diagonalA ? 0 : 0.5)) % 1;
      const stanceFraction = 0.68;
      let xOffset;
      let zOffset;
      if (cycle < stanceFraction) {
        const stance = cycle / stanceFraction;
        xOffset = 27 * (1 - 2 * stance);
        zOffset = 0;
      } else {
        const swing = (cycle - stanceFraction) / (1 - stanceFraction);
        const progress = 0.5 - 0.5 * Math.cos(Math.PI * swing);
        xOffset = 27 * (-1 + 2 * progress);
        zOffset = -25 * Math.sin(Math.PI * swing) ** 2;
      }
      return [
        -15.75 + xOffset,
        leftLeg ? 38 : -38,
        gaitBodyZ + zOffset,
      ];
    });
    state = physics.update(1 / 120, firmwareState, legs, standServoReference);
    if (previousGaitState) {
      bodyPathDistance += Math.hypot(
        state.basePosition[0] - previousGaitState.basePosition[0],
        state.basePosition[2] - previousGaitState.basePosition[2],
      );
      state.footPositions.forEach((position, index) => {
        if (!state.footContacts[index] || !previousGaitState.footContacts[index]) return;
        const previousPosition = previousGaitState.footPositions[index];
        plantedSlipDistance += Math.hypot(
          position[0] - previousPosition[0],
          position[2] - previousPosition[2],
        );
        plantedContactSeconds += 1 / 120;
      });
    }
    previousGaitState = state;
    minimumContacts = Math.min(minimumContacts, state.contactCount);
    maximumTilt = Math.max(maximumTilt, state.baseTiltDegrees);
  }

  const planarTravel = Math.hypot(
    state.basePosition[0] - startPosition[0],
    state.basePosition[2] - startPosition[2],
  );
  const forwardTravel = Math.abs(state.basePosition[0] - startPosition[0]);
  const lateralTravel = Math.abs(state.basePosition[2] - startPosition[2]);
  assert.equal(state.resetCount, 0, `unexpected ${state.lastResetReason} reset`);
  assert.ok(minimumContacts >= 2, `gait dropped to ${minimumContacts} contacts`);
  assert.ok(maximumTilt < 25, `gait body tilt reached ${maximumTilt} deg`);
  assert.ok(planarTravel > 0.02, `gait moved only ${planarTravel} m`);
  assert.ok(
    lateralTravel < Math.max(0.012, forwardTravel * 0.20),
    `forward gait slid ${lateralTravel} m sideways over ${forwardTravel} m forward`,
  );
  const meanPlantedSlipSpeed = plantedSlipDistance / plantedContactSeconds;
  const pathEfficiency = planarTravel / Math.max(bodyPathDistance, 1e-6);
  assert.ok(meanPlantedSlipSpeed < 0.04, `planted-foot slip reached ${meanPlantedSlipSpeed} m/s`);
  assert.ok(pathEfficiency > 0.20, `gait path efficiency was only ${pathEfficiency}`);
  assert.ok(state.drivenTargets.every(Number.isFinite));
});

test("careful walk swings one foot while keeping a three-foot support polygon", async () => {
  const physics = await createDominoPhysics();
  const firmwareState = {
    mode: "STAND",
    pose_z_mm: 265,
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, 265], [-15.75, -38, 265],
      [-15.75, 38, 265], [-15.75, -38, 265],
    ],
  };
  let state;
  for (let frame = 0; frame < 480; frame += 1) {
    state = physics.update(1 / 120, firmwareState, legs, standServoReference);
  }

  const swingOrder = [0, 3, 1, 2];
  let minimumCommandedSupport = 4;
  let minimumPhysicalContacts = 4;
  let sawSingleSwing = false;
  firmwareState.mode = "CAREFUL";
  for (let frame = 0; frame < 1440; frame += 1) {
    const cycle = (0.36 * (frame / 120)) % 1;
    firmwareState.leg_command_xyz_mm = [0, 1, 2, 3].map((legIndex) => {
      const orderIndex = swingOrder.indexOf(legIndex);
      const legCycle = (cycle - 0.25 * orderIndex + 1) % 1;
      const swingFraction = 0.19;
      let xOffset;
      let zOffset = 0;
      if (legCycle < swingFraction) {
        const swing = legCycle / swingFraction;
        xOffset = 18 * (-1 + 2 * (0.5 - 0.5 * Math.cos(Math.PI * swing)));
        zOffset = -18 * Math.sin(Math.PI * swing) ** 2;
      } else {
        const stance = (legCycle - swingFraction) / (1 - swingFraction);
        xOffset = 18 * (1 - 2 * stance);
      }
      return [-15.75 + xOffset, legIndex === 0 || legIndex === 2 ? 38 : -38, 265 + zOffset];
    });
    const commandedSupport = firmwareState.leg_command_xyz_mm
      .filter((command) => Math.abs(command[2] - 265) <= 0.75).length;
    minimumCommandedSupport = Math.min(minimumCommandedSupport, commandedSupport);
    sawSingleSwing ||= commandedSupport === 3;
    state = physics.update(1 / 120, firmwareState, legs, standServoReference);
    minimumPhysicalContacts = Math.min(minimumPhysicalContacts, state.contactCount);
  }

  assert.equal(minimumCommandedSupport, 3);
  assert.ok(sawSingleSwing, "careful walk never lifted exactly one foot");
  assert.ok(minimumPhysicalContacts >= 3, `careful walk dropped to ${minimumPhysicalContacts} contacts`);
  assert.equal(state.resetCount, 0, `unexpected ${state.lastResetReason} reset`);
});

test("tilt command remains finite and physically supported", async () => {
  const physics = await createDominoPhysics();
  const firmwareState = {
    mode: "TILT",
    channels_us: [1675, 1375, 1500, 1650, 2000, 1000, 1000, 2000],
    servo_angle_deg: [
      140.81, 168.75, 119.34, 94.5, 75.47, 0, 0, 150.39,
      99.5, 113.67, 71.42, 135.41, 0, 0, 122.17, 118.8,
    ],
    leg_command_xyz_mm: [
      [-16.43, -17.8, 294.37],
      [-42.34, -90.51, 269.21],
      [-13.29, 26.94, 285.27],
      [-39.2, -45.77, 260.12],
    ],
  };

  let state;
  for (let frame = 0; frame < 1200; frame += 1) {
    state = physics.update(1 / 120, firmwareState, legs, standServoReference);
  }

  assert.equal(state.resetCount, 0, `unexpected ${state.lastResetReason} reset`);
  assert.ok(state.bodyHeight > 0.20, `body height was ${state.bodyHeight}`);
  assert.ok(state.baseTiltDegrees > 2 && state.baseTiltDegrees < 30);
  assert.ok(state.drivenTargets.every(Number.isFinite));
});

test("full-stick tilt remains supported without resetting", async () => {
  const physics = await createDominoPhysics();
  const firmwareState = {
    mode: "TILT",
    channels_us: [2000, 1000, 1500, 2000, 2000, 1000, 1000, 2000],
    servo_angle_deg: [
      148.91, 166.05, 129.06, 86.53, 73.71, 0, 0, 149.99,
      99.77, 114.89, 64.80, 132.03, 0, 0, 122.58, 104.89,
    ],
    leg_command_xyz_mm: [
      [-24.63, -63.27, 301.67],
      [-74.23, -128.03, 257.99],
      [-12.45, 24.81, 286.46],
      [-62.05, -39.95, 242.77],
    ],
  };

  let state;
  for (let frame = 0; frame < 1200; frame += 1) {
    state = physics.update(1 / 120, firmwareState, legs, standServoReference);
  }

  assert.equal(state.resetCount, 0, `unexpected ${state.lastResetReason} reset`);
  assert.equal(
    state.contactCount,
    4,
    `contacts=${state.contactCount} feet=${JSON.stringify(state.footPositions)}`,
  );
  assert.ok(state.bodyHeight > 0.20, `body height was ${state.bodyHeight}`);
  assert.ok(state.baseTiltDegrees > 5 && state.baseTiltDegrees < 35);
  assert.ok(state.drivenTargets.every(Number.isFinite));
});

const isolatedAxisCases = [
  {
    name: "positive roll",
    axis: "roll",
    targetDegrees: 20,
    servo_angle_deg: [
      149.18, 166.32, 123.93, 84.51, 74.11, 0, 0, 146.34,
      108.94, 94.50, 74.11, 144.45, 0, 0, 139.19, 136.49,
    ],
    leg_command_xyz_mm: [
      [-15.75, -63.80, 297.32],
      [-15.75, -127.73, 228.91],
      [-15.75, -63.80, 297.32],
      [-15.75, -127.73, 228.91],
    ],
  },
  {
    name: "negative roll",
    axis: "roll",
    targetDegrees: -20,
    servo_angle_deg: [
      109.08, 175.77, 105.44, 124.74, 83.56, 0, 0, 155.79,
      90.45, 134.73, 83.56, 125.95, 0, 0, 99.09, 117.86,
    ],
    leg_command_xyz_mm: [
      [-15.75, 127.73, 228.91],
      [-15.75, 63.80, 297.32],
      [-15.75, 127.73, 228.91],
      [-15.75, 63.80, 297.32],
    ],
  },
  {
    name: "positive pitch",
    axis: "pitch",
    targetDegrees: 10,
    servo_angle_deg: [
      129.87, 192.51, 111.92, 105.44, 57.38, 0, 0, 155.25,
      116.10, 115.42, 74.52, 118.80, 0, 0, 119.88, 130.00,
    ],
    leg_command_xyz_mm: [
      [27.77, 38.00, 246.44],
      [27.77, -38.00, 246.44],
      [32.86, 38.00, 304.61],
      [32.86, -38.00, 304.61],
    ],
  },
  {
    name: "negative pitch",
    axis: "pitch",
    targetDegrees: -10,
    servo_angle_deg: [
      129.87, 146.88, 121.09, 105.44, 103.00, 0, 0, 145.53,
      81.41, 115.42, 84.38, 153.36, 0, 0, 119.88, 120.69,
    ],
    leg_command_xyz_mm: [
      [-64.43, 38.00, 305.49],
      [-64.43, -38.00, 305.49],
      [-59.34, 38.00, 247.32],
      [-59.34, -38.00, 247.32],
    ],
  },
  {
    name: "positive yaw",
    axis: "yaw",
    targetDegrees: 25,
    servo_angle_deg: [
      146.61, 178.60, 118.26, 92.88, 102.06, 0, 0, 161.19,
      114.34, 131.76, 86.53, 139.59, 0, 0, 107.73, 126.50,
    ],
    leg_command_xyz_mm: [
      [10.78, -43.28, 280.00],
      [-74.14, -100.45, 280.00],
      [42.17, 98.30, 280.00],
      [-42.76, 41.13, 280.00],
    ],
  },
  {
    name: "negative yaw",
    axis: "yaw",
    targetDegrees: -25,
    servo_angle_deg: [
      117.31, 147.69, 115.42, 122.17, 71.28, 0, 0, 143.37,
      95.31, 103.28, 68.72, 120.56, 0, 0, 136.22, 123.53,
    ],
    leg_command_xyz_mm: [
      [-74.14, 100.45, 280.00],
      [10.78, 43.28, 280.00],
      [-42.76, -41.13, 280.00],
      [42.17, -98.30, 280.00],
    ],
  },
];

for (const axisCase of isolatedAxisCases) {
  test(`${axisCase.name} rotates about the body centre without planar drift`, async () => {
    const physics = await createDominoPhysics();
    const bodyPose = axisCase.axis === "roll"
      ? [axisCase.targetDegrees, 0, 0]
      : axisCase.axis === "pitch"
        ? [0, axisCase.targetDegrees, 0]
        : [0, 0, axisCase.targetDegrees];
    const firmwareState = {
      mode: "TILT",
      pose_z_mm: 280,
      body_pose_rpy_deg: bodyPose,
      servo_angle_deg: axisCase.servo_angle_deg,
      leg_command_xyz_mm: axisCase.leg_command_xyz_mm,
    };

    let state;
    let anchorFeet;
    for (let frame = 0; frame < 1200; frame += 1) {
      state = physics.update(1 / 120, firmwareState, legs, standServoReference);
      if (frame === 0) anchorFeet = state.footPositions.map((position) => [...position]);
    }

    const [qx, qy, qz, qw] = state.baseQuaternion;
    const dominantComponent = axisCase.axis === "roll"
      ? qx
      : axisCase.axis === "pitch"
        ? -qz
        : qy;
    const measuredDegrees = 2 * Math.atan2(dominantComponent, qw) * 180 / Math.PI;
    const crossAxisComponents = axisCase.axis === "roll"
      ? [qy, qz]
      : axisCase.axis === "pitch"
        ? [qx, qy]
        : [qx, qz];

    assert.equal(state.resetCount, 0, `unexpected ${state.lastResetReason} reset`);
    assert.equal(state.contactCount, 4);
    assert.ok(Math.abs(state.basePosition[0]) < 0.03, `X drift was ${state.basePosition[0]} m`);
    assert.ok(Math.abs(state.basePosition[2]) < 0.03, `Z drift was ${state.basePosition[2]} m`);
    assert.ok(state.bodyHeight > 0.28 && state.bodyHeight < 0.32);
    assert.ok(
      Math.abs(measuredDegrees - axisCase.targetDegrees) < 3.5,
      `${axisCase.name} measured ${measuredDegrees} deg`,
    );
    assert.ok(
      crossAxisComponents.every((component) => Math.abs(component) < 0.035),
      `${axisCase.name} cross-axis quaternion was ${JSON.stringify(state.baseQuaternion)}`,
    );
    const maxFootPlanarDrift = Math.max(
      ...state.footPositions.map((position, index) =>
        Math.hypot(
          position[0] - anchorFeet[index][0],
          position[2] - anchorFeet[index][2],
        ),
      ),
    );
    const footDriftLimit = axisCase.axis === "roll" ? 0.015 : 0.012;
    assert.ok(
      maxFootPlanarDrift < footDriftLimit,
      `${axisCase.name} moved a planted foot ${maxFootPlanarDrift * 1000} mm`,
    );
    if (axisCase.axis === "yaw") {
      assert.ok(state.baseTiltDegrees < 2, `yaw introduced ${state.baseTiltDegrees} deg lean`);
    }
  });
}

test("negative yaw transition keeps the planted feet supported", async () => {
  const physics = await createDominoPhysics();
  const neutralTilt = {
    mode: "TILT",
    pose_z_mm: 280,
    body_pose_rpy_deg: [0, 0, 0],
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, 280],
      [-15.75, -38, 280],
      [-15.75, 38, 280],
      [-15.75, -38, 280],
    ],
  };
  const yawCase = isolatedAxisCases.find(
    (axisCase) => axisCase.axis === "yaw" && axisCase.targetDegrees < 0,
  );
  const negativeYaw = {
    mode: "TILT",
    pose_z_mm: 280,
    body_pose_rpy_deg: [0, 0, yawCase.targetDegrees],
    servo_angle_deg: yawCase.servo_angle_deg,
    leg_command_xyz_mm: yawCase.leg_command_xyz_mm,
  };

  let state;
  for (let frame = 0; frame < 360; frame += 1) {
    state = physics.update(1 / 120, neutralTilt, legs, standServoReference);
  }

  let minimumContacts = 4;
  let maximumBodyHeight = 0;
  for (let frame = 0; frame < 360; frame += 1) {
    state = physics.update(1 / 120, negativeYaw, legs, standServoReference);
    minimumContacts = Math.min(minimumContacts, state.contactCount);
    maximumBodyHeight = Math.max(maximumBodyHeight, state.bodyHeight);
  }

  assert.equal(state.resetCount, 0, `unexpected ${state.lastResetReason} reset`);
  assert.equal(minimumContacts, 4, `negative yaw dropped to ${minimumContacts} contacts`);
  assert.ok(
    maximumBodyHeight < 0.31,
    `negative yaw raised the body to ${maximumBodyHeight} m`,
  );
});

test("tilt re-entry preserves the heading reached before the mode switch", async () => {
  const physics = await createDominoPhysics();
  const yawCase = isolatedAxisCases.find(
    (axisCase) => axisCase.axis === "yaw" && axisCase.targetDegrees > 0,
  );
  const yawTilt = {
    mode: "TILT",
    pose_z_mm: 280,
    body_pose_rpy_deg: [0, 0, yawCase.targetDegrees],
    servo_angle_deg: yawCase.servo_angle_deg,
    leg_command_xyz_mm: yawCase.leg_command_xyz_mm,
  };
  const neutralStand = {
    mode: "STAND",
    pose_z_mm: 280,
    body_pose_rpy_deg: [0, 0, 0],
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, 280],
      [-15.75, -38, 280],
      [-15.75, 38, 280],
      [-15.75, -38, 280],
    ],
  };

  const headingDegrees = (state) => {
    const [x, y, z, w] = state.baseQuaternion;
    const forwardX = 1 - 2 * (y * y + z * z);
    const forwardZ = 2 * (x * z - w * y);
    return Math.atan2(-forwardZ, forwardX) * 180 / Math.PI;
  };

  let state;
  for (let frame = 0; frame < 720; frame += 1) {
    state = physics.update(1 / 120, yawTilt, legs, standServoReference);
  }
  for (let frame = 0; frame < 360; frame += 1) {
    state = physics.update(1 / 120, neutralStand, legs, standServoReference);
  }
  const headingBeforeReentry = headingDegrees(state);
  const positionBeforeReentry = [...state.basePosition];

  const neutralTilt = { ...neutralStand, mode: "TILT" };
  let minimumContacts = 4;
  for (let frame = 0; frame < 480; frame += 1) {
    state = physics.update(1 / 120, neutralTilt, legs, standServoReference);
    minimumContacts = Math.min(minimumContacts, state.contactCount);
  }

  const headingAfterReentry = headingDegrees(state);
  const planarDrift = Math.hypot(
    state.basePosition[0] - positionBeforeReentry[0],
    state.basePosition[2] - positionBeforeReentry[2],
  );
  assert.equal(state.resetCount, 0, `unexpected ${state.lastResetReason} reset`);
  assert.ok(minimumContacts >= 3, `tilt re-entry dropped to ${minimumContacts} contacts`);
  assert.ok(
    Math.abs(headingAfterReentry - headingBeforeReentry) < 3,
    `tilt re-entry changed heading from ${headingBeforeReentry} to ${headingAfterReentry} deg`,
  );
  assert.ok(planarDrift < 0.02, `tilt re-entry moved ${planarDrift} m in the plane`);
});

test("repeated tilt reversals settle without residual jitter", async () => {
  const physics = await createDominoPhysics();
  const neutralTilt = {
    mode: "TILT",
    pose_z_mm: 280,
    body_pose_rpy_deg: [0, 0, 0],
    servo_angle_deg: [...standServoReference],
    leg_command_xyz_mm: [
      [-15.75, 38, 280],
      [-15.75, -38, 280],
      [-15.75, 38, 280],
      [-15.75, -38, 280],
    ],
  };

  let state;
  for (let frame = 0; frame < 240; frame += 1) {
    state = physics.update(1 / 120, neutralTilt, legs, standServoReference);
  }

  let minimumContactsDuringMotion = 4;
  let maximumBodyHeightDuringMotion = 0;
  let minimumContactPose = "none";
  for (let cycle = 0; cycle < 2; cycle += 1) {
    for (const axisCase of isolatedAxisCases) {
      const bodyPose = axisCase.axis === "roll"
        ? [axisCase.targetDegrees, 0, 0]
        : axisCase.axis === "pitch"
          ? [0, axisCase.targetDegrees, 0]
          : [0, 0, axisCase.targetDegrees];
      const command = {
        mode: "TILT",
        pose_z_mm: 280,
        body_pose_rpy_deg: bodyPose,
        servo_angle_deg: axisCase.servo_angle_deg,
        leg_command_xyz_mm: axisCase.leg_command_xyz_mm,
      };
      for (let frame = 0; frame < 90; frame += 1) {
        state = physics.update(1 / 120, command, legs, standServoReference);
        if (state.contactCount < minimumContactsDuringMotion) {
          minimumContactsDuringMotion = state.contactCount;
          minimumContactPose = `cycle ${cycle + 1} ${axisCase.name} frame ${frame}`;
        }
        maximumBodyHeightDuringMotion = Math.max(
          maximumBodyHeightDuringMotion,
          state.bodyHeight,
        );
      }
    }
  }

  let maximumSettledLinearSpeed = 0;
  let maximumSettledAngularSpeed = 0;
  for (let frame = 0; frame < 720; frame += 1) {
    state = physics.update(1 / 120, neutralTilt, legs, standServoReference);
    if (frame >= 600) {
      maximumSettledLinearSpeed = Math.max(
        maximumSettledLinearSpeed,
        Math.hypot(...state.linearVelocity),
      );
      maximumSettledAngularSpeed = Math.max(
        maximumSettledAngularSpeed,
        Math.hypot(...state.angularVelocity),
      );
    }
  }

  assert.equal(state.resetCount, 0, `unexpected ${state.lastResetReason} reset`);
  assert.ok(
    minimumContactsDuringMotion >= 2,
    `tilt reversals dropped to ${minimumContactsDuringMotion} contacts at ${minimumContactPose}`,
  );
  assert.ok(
    maximumBodyHeightDuringMotion < 0.32,
    `tilt reversals raised the body to ${maximumBodyHeightDuringMotion} m`,
  );
  assert.equal(state.contactCount, 4);
  assert.ok(
    maximumSettledLinearSpeed < 0.025,
    `residual linear jitter was ${maximumSettledLinearSpeed} m/s; ` +
      `final=${JSON.stringify(state.linearVelocity)} position=${JSON.stringify(state.basePosition)}`,
  );
  assert.ok(
    maximumSettledAngularSpeed < 0.12,
    `residual angular jitter was ${maximumSettledAngularSpeed} rad/s`,
  );
});
