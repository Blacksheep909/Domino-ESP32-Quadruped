import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";

import { batteryPacks, dominoMassModel } from "./domino-config.js";
import { environmentBallSpecs, logSpecs, terrainSpecs } from "./course-config.js";
import { createVoronoiTerrain } from "./voronoi-terrain.js";

const FIXED_TIMESTEP = 1 / 120;
const ROBOT_MEMBERSHIP = 0x0001;
const WORLD_MEMBERSHIP = 0x0002;
const PROP_MEMBERSHIP = 0x0004;
const ROBOT_GROUP =
  (ROBOT_MEMBERSHIP << 16) | (WORLD_MEMBERSHIP | PROP_MEMBERSHIP);
const WORLD_GROUP =
  (WORLD_MEMBERSHIP << 16) | (ROBOT_MEMBERSHIP | PROP_MEMBERSHIP);
const PROP_GROUP =
  (PROP_MEMBERSHIP << 16) |
  (ROBOT_MEMBERSHIP | WORLD_MEMBERSHIP | PROP_MEMBERSHIP);
const SHOULDER_LIMIT = THREE.MathUtils.degToRad(45);
const SERVO_TRAVEL = THREE.MathUtils.degToRad(45);
const LEG_LENGTHS = { upper: 0.16, lower: 0.153 };
const HIP_LATERAL_OFFSET = 0.038;
const HIP_VERTICAL_OFFSET = 0.021;
const FOOT_RADIUS = 0.024;
const NEUTRAL_HIP_ANGLE = 0.6392845609367196;
const NEUTRAL_KNEE_ANGLE = -1.1872793258308068;
const INITIAL_BASE_HEIGHT = 0.304;
const BODY_HEIGHT_FROM_FOOT_REACH_OFFSET = FOOT_RADIUS;
const BODY_HEIGHT_ASSIST_STIFFNESS = 600;
const BODY_HEIGHT_ASSIST_DAMPING = 35;
const BODY_HEIGHT_ASSIST_MAX_FORCE = 45;
const BODY_UPRIGHT_ASSIST_STIFFNESS = 16;
const BODY_UPRIGHT_ASSIST_DAMPING = 2.4;
const BODY_UPRIGHT_ASSIST_MAX_TORQUE = 3.5;
const BODY_POSE_ASSIST_STIFFNESS = 52;
const BODY_POSE_ASSIST_DAMPING = 5.5;
const BODY_POSE_ASSIST_MAX_TORQUE = 14;
const BODY_POSE_ASSIST_MAX_TORQUE_RATE = 80;
const BODY_POSE_TARGET_SPEED = THREE.MathUtils.degToRad(120);
const BODY_PLANAR_HOLD_STIFFNESS = 420;
const BODY_PLANAR_HOLD_DAMPING = 55;
const BODY_PLANAR_HOLD_MAX_FORCE = 90;
const FOOT_PLANAR_HOLD_STIFFNESS = 480;
const FOOT_PLANAR_HOLD_DAMPING = 34;
const FOOT_PLANAR_HOLD_MAX_FORCE = 55;
const FOOT_PLANAR_HOLD_MAX_FORCE_RATE = 600;
const FOOT_VERTICAL_HOLD_STIFFNESS = 260;
const FOOT_VERTICAL_HOLD_DAMPING = 24;
const FOOT_VERTICAL_HOLD_MAX_FORCE = 32;
const GAIT_FOOT_HOLD_STIFFNESS = 320;
const GAIT_FOOT_HOLD_DAMPING = 28;
const GAIT_FOOT_HOLD_MAX_FORCE = 42;
const GAIT_FOOT_HOLD_MAX_FORCE_RATE = 480;
const GAIT_STANCE_COMMAND_TOLERANCE_MM = 1.5;
const GAIT_HEADING_STIFFNESS = 10;
const GAIT_HEADING_DAMPING = 2.2;
const MAX_SHOULDER_SPEED = THREE.MathUtils.degToRad(240);
const MAX_LEG_SPEED = THREE.MathUtils.degToRad(300);
const SIL_COMMAND_INDEX = {
  dom_p_4_1: 1,
  dom_p_12_1: 0,
  dom_p_25_1: 2,
  dom_p_21_1: 3,
};
const LEG_SPECS = [
  {
    id: "dom_p_4_1",
    shoulderAnchor: [0.1675, 0, 0.062375],
    upperAnchor: [0, -HIP_VERTICAL_OFFSET, HIP_LATERAL_OFFSET],
    shoulderAxis: [1, 0, 0],
    defaults: [0, NEUTRAL_HIP_ANGLE, NEUTRAL_KNEE_ANGLE],
    limits: [[-SHOULDER_LIMIT, SHOULDER_LIMIT], [-0.523599, 1.047198], [-2.094395, 0]],
  },
  {
    id: "dom_p_12_1",
    shoulderAnchor: [0.1675, 0, -0.062375],
    upperAnchor: [0, -HIP_VERTICAL_OFFSET, -HIP_LATERAL_OFFSET],
    shoulderAxis: [-1, 0, 0],
    defaults: [0, NEUTRAL_HIP_ANGLE, NEUTRAL_KNEE_ANGLE],
    limits: [[-SHOULDER_LIMIT, SHOULDER_LIMIT], [-0.523599, 1.047198], [-2.094395, 0]],
  },
  {
    id: "dom_p_25_1",
    shoulderAnchor: [-0.1675, 0, -0.062375],
    upperAnchor: [0, -HIP_VERTICAL_OFFSET, -HIP_LATERAL_OFFSET],
    shoulderAxis: [-1, 0, 0],
    defaults: [0, NEUTRAL_HIP_ANGLE, NEUTRAL_KNEE_ANGLE],
    limits: [[-SHOULDER_LIMIT, SHOULDER_LIMIT], [-0.523599, 1.047198], [-2.094395, 0]],
  },
  {
    id: "dom_p_21_1",
    shoulderAnchor: [-0.1675, 0, 0.062375],
    upperAnchor: [0, -HIP_VERTICAL_OFFSET, HIP_LATERAL_OFFSET],
    shoulderAxis: [1, 0, 0],
    defaults: [0, NEUTRAL_HIP_ANGLE, NEUTRAL_KNEE_ANGLE],
    limits: [[-SHOULDER_LIMIT, SHOULDER_LIMIT], [-0.523599, 1.047198], [-2.094395, 0]],
  },
];

function vector(values) {
  return { x: values[0], y: values[1], z: values[2] };
}

function quaternionToRapier(quaternion) {
  return { x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w };
}

function bodyDescriptor(position, rotation, additionalIterations = 12) {
  return RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(position.x, position.y, position.z)
    .setRotation(quaternionToRapier(rotation))
    .setLinearDamping(0.18)
    .setAngularDamping(0.28)
    .setAdditionalSolverIterations(additionalIterations)
    .setCcdEnabled(true);
}

function robotCollider(description, mass, friction = 0.65) {
  return description
    .setMass(mass)
    .setFriction(friction)
    .setRestitution(0)
    .setCollisionGroups(ROBOT_GROUP);
}

function createRevolute(world, parent, child, parentAnchor, childAnchor, axis, limits) {
  const data = RAPIER.JointData.revolute(vector(parentAnchor), vector(childAnchor), vector(axis));
  const joint = world.createImpulseJoint(data, parent, child, true);
  joint.setContactsEnabled(false);
  joint.setLimits(limits[0], limits[1]);
  joint.configureMotorModel(RAPIER.MotorModel.ForceBased);
  return joint;
}

function rotateVector(vectorValue, quaternion) {
  return vectorValue.clone().applyQuaternion(quaternion);
}

function clamp(value, limits) {
  return THREE.MathUtils.clamp(value, limits[0] + 0.002, limits[1] - 0.002);
}

export function targetBodyQuaternion(firmwareState) {
  const pose = firmwareState?.body_pose_rpy_deg;
  if (!Array.isArray(pose) || pose.length < 3) return null;
  const roll = THREE.MathUtils.degToRad(Number(pose[0]) || 0);
  const pitch = THREE.MathUtils.degToRad(Number(pose[1]) || 0);
  const yaw = THREE.MathUtils.degToRad(Number(pose[2]) || 0);
  // Firmware uses X forward, Y left, Z up. Rapier uses X forward, Y up,
  // and Z right, so pitch maps onto -Z while yaw maps onto +Y.
  return new THREE.Quaternion()
    .setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw)
    .multiply(
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, -1), pitch),
    )
    .multiply(
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), roll),
    );
}

export function contactSurfaceError(
  visualFootCenterY,
  visualFootRadius,
  proxyFootCenterY,
  proxyFootRadius,
) {
  return (Number(visualFootCenterY) - Number(visualFootRadius)) -
    (Number(proxyFootCenterY) - Number(proxyFootRadius));
}

function levelHeadingQuaternion(quaternion) {
  const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(quaternion);
  forward.y = 0;
  if (forward.lengthSq() < 1e-8) return new THREE.Quaternion();
  forward.normalize();
  const yaw = Math.atan2(-forward.z, forward.x);
  return new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
}

function solveSagittalLeg(xMillimeters, yMillimeters, zMillimeters) {
  const x = -xMillimeters / 1000;
  const y = yMillimeters / 1000;
  const z = zMillimeters / 1000;
  // The shoulder joint removes the 38 mm lateral hip offset before the
  // upper/lower links move in their sagittal plane. Ignoring Y made tilt
  // commands shorten or extend the Rapier leg differently from firmware IK.
  const planeZ = Math.max(
    Math.sqrt(Math.max(
    y ** 2 + z ** 2 - HIP_LATERAL_OFFSET ** 2,
      0,
    )) - HIP_VERTICAL_OFFSET,
    0.05,
  );
  const upper = LEG_LENGTHS.upper;
  const lower = LEG_LENGTHS.lower;
  const radiusSquared = x ** 2 + planeZ ** 2;
  const kneeCosine = THREE.MathUtils.clamp(
    (radiusSquared - upper ** 2 - lower ** 2) / (2 * upper * lower),
    -1,
    1,
  );
  const knee = -Math.acos(kneeCosine);
  const hip = Math.atan2(x, planeZ) -
    Math.atan2(lower * Math.sin(knee), upper + lower * Math.cos(knee));
  return [hip, knee];
}

export async function createDominoPhysics(options = {}) {
  await RAPIER.init();

  const initialBasePosition = Array.isArray(options.initialBasePosition)
    ? new THREE.Vector3(...options.initialBasePosition)
    : new THREE.Vector3(0, INITIAL_BASE_HEIGHT, 0);

  let world;
  let base;
  let legRuntimes = [];
  let accumulator = 0;
  let resetCount = 0;
  let lastResetReason = "startup";
  let elapsed = 0;
  let neutralStandTime = 0;
  let latestTargets = LEG_SPECS.flatMap((spec) => spec.defaults);
  let drivenTargets = [...latestTargets];
  let commandedBodyHeight = INITIAL_BASE_HEIGHT;
  let tiltPlanarAnchor = null;
  let tiltFootAnchors = null;
  let tiltHeightReference = null;
  let gaitFootAnchors = Array(LEG_SPECS.length).fill(null);
  let gaitHeadingForward = null;
  let previousMode = null;
  let assistedBodyQuaternion = new THREE.Quaternion();
  let tiltReferenceQuaternion = new THREE.Quaternion();
  let bodyPoseTargetQuaternion = null;
  const bodyPoseTorque = new THREE.Vector3();
  let environmentBalls = [];

  function createStaticBox(size, position, yaw = 0, slope = 0, friction = 0.9) {
    const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, yaw, slope));
    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(position[0], position[1], position[2])
        .setRotation(quaternionToRapier(rotation)),
    );
    return world.createCollider(
      RAPIER.ColliderDesc.cuboid(size[0] / 2, size[1] / 2, size[2] / 2)
        .setFriction(friction)
        .setRestitution(0)
        .setCollisionGroups(WORLD_GROUP),
      body,
    );
  }

  function createStaticVoronoi(spec) {
    const terrain = createVoronoiTerrain(spec);
    createStaticBox(
      spec.size,
      spec.position,
      spec.yaw ?? 0,
      0,
      spec.friction ?? 0.9,
    );
    const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, spec.yaw ?? 0, 0));
    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(spec.position[0], 0, spec.position[2])
        .setRotation(quaternionToRapier(rotation)),
    );
    return world.createCollider(
      RAPIER.ColliderDesc.trimesh(terrain.vertices, terrain.indices)
        .setFriction(spec.friction ?? 0.9)
        .setRestitution(0)
        .setCollisionGroups(WORLD_GROUP),
      body,
    );
  }

  function createStaticCylinder(spec) {
    const rotation = new THREE.Quaternion().setFromEuler(
      spec.axis === "x"
        ? new THREE.Euler(0, spec.yaw ?? 0, Math.PI / 2)
        : new THREE.Euler(Math.PI / 2, spec.yaw ?? 0, 0),
    );
    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(spec.position[0], spec.position[1], spec.position[2])
        .setRotation(quaternionToRapier(rotation)),
    );
    return world.createCollider(
      RAPIER.ColliderDesc.cylinder(spec.length / 2, spec.radius)
        .setFriction(0.94)
        .setRestitution(0)
        .setCollisionGroups(WORLD_GROUP),
      body,
    );
  }

  function createDynamicBall(spec, preservedState = null) {
    const spawnPosition = preservedState?.position ?? spec.position;
    const descriptor = RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(spawnPosition[0], spawnPosition[1], spawnPosition[2])
      .setLinearDamping(0.12)
      .setAngularDamping(0.12)
      .setAdditionalSolverIterations(4)
      .setCcdEnabled(true);
    if (preservedState?.quaternion) {
      descriptor.setRotation({
        x: preservedState.quaternion[0],
        y: preservedState.quaternion[1],
        z: preservedState.quaternion[2],
        w: preservedState.quaternion[3],
      });
    }
    const body = world.createRigidBody(
      descriptor,
    );
    const collider = world.createCollider(
      RAPIER.ColliderDesc.ball(spec.radius)
        .setMass(spec.mass)
        .setFriction(0.62)
        .setRestitution(0)
        .setCollisionGroups(PROP_GROUP),
      body,
    );
    return { ...spec, body, collider };
  }

  function captureEnvironmentBallStates() {
    const states = new Map();
    environmentBalls.forEach((ball) => {
      const position = ball.body.translation();
      const rotation = ball.body.rotation();
      const values = [
        position.x,
        position.y,
        position.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
      ];
      if (
        values.every(Number.isFinite) &&
        position.y >= ball.radius * 0.8 &&
        Math.abs(position.x) <= 5 &&
        Math.abs(position.z) <= 5
      ) {
        states.set(ball.id, {
          position: [position.x, position.y, position.z],
          quaternion: [rotation.x, rotation.y, rotation.z, rotation.w],
        });
      }
    });
    return states;
  }

  function buildLeg(spec) {
    const basePosition = initialBasePosition.clone();
    const shoulderAnchor = new THREE.Vector3(...spec.shoulderAnchor);
    const upperAnchor = new THREE.Vector3(...spec.upperAnchor);
    const shoulderAxis = new THREE.Vector3(...spec.shoulderAxis);
    const pitchAxis = new THREE.Vector3(0, 0, -1);

    const hipRotation = new THREE.Quaternion().setFromAxisAngle(shoulderAxis, spec.defaults[0]);
    const hipPosition = basePosition.clone().add(shoulderAnchor);
    const hip = world.createRigidBody(bodyDescriptor(hipPosition, hipRotation));
    world.createCollider(
      robotCollider(
        RAPIER.ColliderDesc.cuboid(0.03, 0.015, 0.0275),
        dominoMassModel.hipKg,
      ),
      hip,
    );

    const upperPivot = hipPosition.clone().add(rotateVector(upperAnchor, hipRotation));
    const upperRotation = hipRotation.clone().multiply(
      new THREE.Quaternion().setFromAxisAngle(pitchAxis, spec.defaults[1]),
    );
    const upperPosition = upperPivot.clone().add(
      rotateVector(new THREE.Vector3(0, -LEG_LENGTHS.upper / 2, 0), upperRotation),
    );
    const upper = world.createRigidBody(bodyDescriptor(upperPosition, upperRotation));
    world.createCollider(
      robotCollider(
        RAPIER.ColliderDesc.capsule(0.066, 0.014),
        dominoMassModel.upperLinkKg,
      ),
      upper,
    );

    const lowerPivot = upperPivot.clone().add(
      rotateVector(new THREE.Vector3(0, -LEG_LENGTHS.upper, 0), upperRotation),
    );
    const lowerRotation = upperRotation.clone().multiply(
      new THREE.Quaternion().setFromAxisAngle(pitchAxis, spec.defaults[2]),
    );
    const lowerPosition = lowerPivot.clone().add(
      rotateVector(new THREE.Vector3(0, -LEG_LENGTHS.lower / 2, 0), lowerRotation),
    );
    const lower = world.createRigidBody(bodyDescriptor(lowerPosition, lowerRotation));
    world.createCollider(
      robotCollider(
        RAPIER.ColliderDesc.capsule(0.0645, 0.012),
        dominoMassModel.lowerLinkKg,
      ),
      lower,
    );
    const footCollider = world.createCollider(
      robotCollider(
        RAPIER.ColliderDesc.ball(FOOT_RADIUS).setTranslation(0, -LEG_LENGTHS.lower / 2, 0),
        dominoMassModel.footKg,
        1.25,
      ),
      lower,
    );

    const shoulderJoint = createRevolute(
      world,
      base,
      hip,
      spec.shoulderAnchor,
      [0, 0, 0],
      spec.shoulderAxis,
      spec.limits[0],
    );
    const upperJoint = createRevolute(
      world,
      hip,
      upper,
      spec.upperAnchor,
      [0, LEG_LENGTHS.upper / 2, 0],
      [0, 0, -1],
      spec.limits[1],
    );
    const lowerJoint = createRevolute(
      world,
      upper,
      lower,
      [0, -LEG_LENGTHS.upper / 2, 0],
      [0, LEG_LENGTHS.lower / 2, 0],
      [0, 0, -1],
      spec.limits[2],
    );

    return {
      spec,
      joints: [shoulderJoint, upperJoint, lowerJoint],
      lower,
      footCollider,
      planarHoldForce: new THREE.Vector3(),
    };
  }

  function buildWorld({ preserveEnvironment = false } = {}) {
    const preservedBallStates = preserveEnvironment
      ? captureEnvironmentBallStates()
      : new Map();
    if (world) world.free();
    world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    world.timestep = FIXED_TIMESTEP;
    world.numSolverIterations = 12;
    world.numAdditionalFrictionIterations = 8;
    world.numInternalPgsIterations = 2;
    createStaticBox([40, 0.04, 40], [0, -0.02, 0]);
    terrainSpecs.forEach((terrain) =>
      terrain.kind === "voronoi" ? createStaticVoronoi(terrain) : createStaticBox(
        terrain.size,
        terrain.position,
        terrain.yaw ?? 0,
        terrain.slope ?? 0,
        terrain.friction ?? 0.9,
      ),
    );
    logSpecs.forEach(createStaticCylinder);
    environmentBalls = environmentBallSpecs.map((spec) =>
      createDynamicBall(spec, preservedBallStates.get(spec.id)),
    );

    base = world.createRigidBody(
      bodyDescriptor(initialBasePosition, new THREE.Quaternion(), 20),
    );
    world.createCollider(
      robotCollider(
        RAPIER.ColliderDesc.cuboid(0.215, 0.025, 0.1025),
        dominoMassModel.chassisWithoutBatteriesKg,
        0.55,
      ),
      base,
    );
    batteryPacks.forEach((pack) => {
      // CAD uses X forward, Y left, Z up; Rapier uses X forward, Y up, Z right.
      world.createCollider(
        robotCollider(
          RAPIER.ColliderDesc.cuboid(
            pack.size[0] / 2,
            pack.size[2] / 2,
            pack.size[1] / 2,
          ).setTranslation(pack.center[0], pack.center[2], -pack.center[1]),
          pack.massKg,
          0.55,
        ),
        base,
      );
    });
    legRuntimes = LEG_SPECS.map(buildLeg);
    latestTargets = LEG_SPECS.flatMap((spec) => spec.defaults);
    drivenTargets = [...latestTargets];
    accumulator = 0;
    elapsed = 0;
    neutralStandTime = 0;
    tiltPlanarAnchor = null;
    tiltFootAnchors = null;
    tiltHeightReference = null;
    gaitFootAnchors = Array(LEG_SPECS.length).fill(null);
    gaitHeadingForward = null;
    previousMode = null;
    assistedBodyQuaternion.identity();
    tiltReferenceQuaternion.identity();
    bodyPoseTargetQuaternion = null;
    bodyPoseTorque.set(0, 0, 0);
  }

  function setTargets(firmwareState, visualLegs, servoReference) {
    if (!firmwareState?.servo_angle_deg || !Array.isArray(visualLegs)) return;

    latestTargets = LEG_SPECS.flatMap((physicsSpec, index) => {
      const visualSpec = visualLegs[index];
      const channelSet = [
        visualSpec.channels.shoulder,
        visualSpec.channels.upper,
        visualSpec.channels.lower,
      ];
      const directionSet = [
        visualSpec.directions.shoulder,
        visualSpec.directions.upper,
        visualSpec.directions.lower,
      ];
      const servoTargets = channelSet.map((channel, jointIndex) => {
        const servoAngle = Number(firmwareState.servo_angle_deg[channel]);
        const referenceAngle = Number(servoReference[channel]);
        if (!Number.isFinite(servoAngle) || !Number.isFinite(referenceAngle)) {
          return physicsSpec.defaults[jointIndex];
        }
        const servoDelta = THREE.MathUtils.degToRad(
          (servoAngle - referenceAngle) / directionSet[jointIndex],
        );
        const boundedDelta = THREE.MathUtils.clamp(servoDelta, -SERVO_TRAVEL, SERVO_TRAVEL);
        return clamp(
          physicsSpec.defaults[jointIndex] + boundedDelta,
          physicsSpec.limits[jointIndex],
        );
      });

      const commandIndex = SIL_COMMAND_INDEX[physicsSpec.id];
      const command = firmwareState.leg_command_xyz_mm?.[commandIndex];
      if (
        !Array.isArray(command) ||
        command.length < 3 ||
        !Number.isFinite(Number(command[0])) ||
        !Number.isFinite(Number(command[2]))
      ) {
        return servoTargets;
      }

      const sagittalPose = solveSagittalLeg(
        Number(command[0]),
        Number(command[1]),
        Number(command[2]),
      );
      return [
        servoTargets[0],
        clamp(sagittalPose[0], physicsSpec.limits[1]),
        clamp(sagittalPose[1], physicsSpec.limits[2]),
      ];
    });
  }

  function driveMotors() {
    legRuntimes.forEach((runtime, legIndex) => {
      runtime.joints.forEach((joint, jointIndex) => {
        const targetIndex = (legIndex * 3) + jointIndex;
        const requestedTarget = latestTargets[targetIndex];
        const maxStep =
          (jointIndex === 0 ? MAX_SHOULDER_SPEED : MAX_LEG_SPEED) * FIXED_TIMESTEP;
        drivenTargets[targetIndex] = THREE.MathUtils.clamp(
          requestedTarget,
          drivenTargets[targetIndex] - maxStep,
          drivenTargets[targetIndex] + maxStep,
        );
        joint.configureMotorPosition(
          drivenTargets[targetIndex],
          jointIndex === 0 ? 40 : 35,
          2.8,
        );
      });
    });
  }

  function driveBodyHeight(firmwareState) {
    const legCommands = firmwareState?.leg_command_xyz_mm;
    if (!Array.isArray(legCommands) || legCommands.length !== 4) return;
    const averageFootReach = legCommands.reduce(
      (sum, command) => sum + Math.max(0, Number(command?.[2]) || 0),
      0,
    ) / legCommands.length / 1000;
    const firmwarePoseZ = Number(firmwareState?.pose_z_mm);
    const commandedReach = Number.isFinite(firmwarePoseZ)
      ? firmwarePoseZ / 1000
      : averageFootReach;
    const nominalBodyHeight = commandedReach + BODY_HEIGHT_FROM_FOOT_REACH_OFFSET;
    commandedBodyHeight = firmwareState?.mode === "TILT" && tiltHeightReference
      ? tiltHeightReference.baseHeight +
        (commandedReach - tiltHeightReference.commandedReach)
      : nominalBodyHeight;
    const translation = base.translation();
    const velocity = base.linvel();
    const force = THREE.MathUtils.clamp(
      BODY_HEIGHT_ASSIST_STIFFNESS * (commandedBodyHeight - translation.y) -
        BODY_HEIGHT_ASSIST_DAMPING * velocity.y,
      -BODY_HEIGHT_ASSIST_MAX_FORCE,
      BODY_HEIGHT_ASSIST_MAX_FORCE,
    );
    base.resetForces(true);
    base.addForce({ x: 0, y: force, z: 0 }, true);
  }

  function driveBodyPlanarPosition(firmwareState) {
    if (firmwareState?.mode !== "TILT" || !tiltPlanarAnchor) return;
    const translation = base.translation();
    const velocity = base.linvel();
    const correction = new THREE.Vector3(
      BODY_PLANAR_HOLD_STIFFNESS * (tiltPlanarAnchor.x - translation.x) -
        BODY_PLANAR_HOLD_DAMPING * velocity.x,
      0,
      BODY_PLANAR_HOLD_STIFFNESS * (tiltPlanarAnchor.z - translation.z) -
        BODY_PLANAR_HOLD_DAMPING * velocity.z,
    );
    if (correction.length() > BODY_PLANAR_HOLD_MAX_FORCE) {
      correction.setLength(BODY_PLANAR_HOLD_MAX_FORCE);
    }
    base.addForce({ x: correction.x, y: 0, z: correction.z }, true);
  }

  function driveFootPlanarAnchors(firmwareState) {
    if (firmwareState?.mode !== "TILT" || !tiltFootAnchors) return;
    legRuntimes.forEach((runtime, index) => {
      const anchor = tiltFootAnchors[index];
      applyFootPlanarHold(
        runtime,
        anchor,
        FOOT_PLANAR_HOLD_STIFFNESS,
        FOOT_PLANAR_HOLD_DAMPING,
        FOOT_PLANAR_HOLD_MAX_FORCE,
        FOOT_PLANAR_HOLD_MAX_FORCE_RATE,
      );
    });
  }

  function applyFootPlanarHold(
    runtime,
    anchor,
    stiffness,
    damping,
    maximumForce,
    maximumForceRate,
    xScale = 1,
    zScale = 1,
  ) {
      const position = runtime.footCollider.translation();
      const velocity = runtime.lower.linvel();
      const correction = new THREE.Vector3(
        xScale * (stiffness * (anchor.x - position.x) - damping * velocity.x),
        Number.isFinite(anchor.y)
          ? THREE.MathUtils.clamp(
              FOOT_VERTICAL_HOLD_STIFFNESS * (anchor.y - position.y) -
                FOOT_VERTICAL_HOLD_DAMPING * velocity.y,
              -FOOT_VERTICAL_HOLD_MAX_FORCE,
              FOOT_VERTICAL_HOLD_MAX_FORCE,
            )
          : 0,
        zScale * (stiffness * (anchor.z - position.z) - damping * velocity.z),
      );
      if (correction.length() > maximumForce) {
        correction.setLength(maximumForce);
      }
      const forceStep = correction.clone().sub(runtime.planarHoldForce);
      const maximumForceStep = maximumForceRate * FIXED_TIMESTEP;
      if (forceStep.length() > maximumForceStep) {
        forceStep.setLength(maximumForceStep);
      }
      runtime.planarHoldForce.add(forceStep);
      runtime.lower.resetForces(true);
      runtime.lower.addForce(
        {
          x: runtime.planarHoldForce.x,
          y: runtime.planarHoldForce.y,
          z: runtime.planarHoldForce.z,
        },
        true,
      );
  }

  function footHasContact(runtime) {
    let touching = false;
    world.contactPairsWith(runtime.footCollider, (otherCollider) => {
      world.contactPair(runtime.footCollider, otherCollider, (manifold) => {
        if (manifold.numSolverContacts() > 0) touching = true;
      });
    });
    return touching;
  }

  function readFootContact(runtime) {
    let touching = false;
    const supportHeights = [];
    world.contactPairsWith(runtime.footCollider, (otherCollider) => {
      world.contactPair(runtime.footCollider, otherCollider, (manifold) => {
        const solverContacts = manifold.numSolverContacts();
        if (solverContacts <= 0) return;
        touching = true;
        const normal = manifold.normal();
        if (Math.abs(normal.y) < 0.45) return;
        for (let index = 0; index < solverContacts; index += 1) {
          const point = manifold.solverContactPoint(index);
          if (Number.isFinite(point?.y)) supportHeights.push(point.y);
        }
      });
    });
    supportHeights.sort((a, b) => a - b);
    const middle = Math.floor(supportHeights.length / 2);
    const supportHeight = supportHeights.length === 0
      ? null
      : supportHeights.length % 2 === 0
        ? 0.5 * (supportHeights[middle - 1] + supportHeights[middle])
        : supportHeights[middle];
    return { touching, supportHeight };
  }

  function gaitLegIsInStance(firmwareState, runtimeIndex) {
    const commandIndex = SIL_COMMAND_INDEX[LEG_SPECS[runtimeIndex].id];
    const command = firmwareState?.leg_command_xyz_mm?.[commandIndex];
    const poseZ = Number(firmwareState?.pose_z_mm);
    return Array.isArray(command) &&
      Number.isFinite(poseZ) &&
      Math.abs((Number(command[2]) || 0) - poseZ) <= GAIT_STANCE_COMMAND_TOLERANCE_MM;
  }

  function isWalkingMode(mode) {
    return mode === "GAIT" || mode === "CAREFUL";
  }

  function clearPlanarHoldForces() {
    legRuntimes.forEach((runtime) => runtime.planarHoldForce.set(0, 0, 0));
  }

  function driveGaitStanceAnchors(firmwareState) {
    if (!isWalkingMode(firmwareState?.mode)) return;
    legRuntimes.forEach((runtime, index) => {
      if (!gaitLegIsInStance(firmwareState, index)) {
        gaitFootAnchors[index] = null;
        runtime.planarHoldForce.set(0, 0, 0);
        return;
      }

      const position = runtime.footCollider.translation();
      const closeToGround = position.y <= FOOT_RADIUS + 0.008;
      if (!gaitFootAnchors[index] && (footHasContact(runtime) || closeToGround)) {
        gaitFootAnchors[index] = { x: position.x, z: position.z };
      }
      if (!gaitFootAnchors[index]) return;

      applyFootPlanarHold(
        runtime,
        gaitFootAnchors[index],
        GAIT_FOOT_HOLD_STIFFNESS,
        GAIT_FOOT_HOLD_DAMPING,
        GAIT_FOOT_HOLD_MAX_FORCE,
        GAIT_FOOT_HOLD_MAX_FORCE_RATE,
        0,
        1,
      );
    });
  }

  function driveBodyAttitude(firmwareState) {
    base.resetTorques(true);
    const rotation = base.rotation();
    const quaternion = new THREE.Quaternion(
      rotation.x,
      rotation.y,
      rotation.z,
      rotation.w,
    );

    if (firmwareState?.mode === "TILT") {
      const requestedDelta = targetBodyQuaternion(firmwareState);
      if (!requestedDelta) return;
      const requestedTarget = tiltReferenceQuaternion.clone()
        .multiply(requestedDelta)
        .normalize();
      bodyPoseTargetQuaternion = requestedTarget.clone();
      const targetDistance = assistedBodyQuaternion.angleTo(requestedTarget);
      const targetBlend = targetDistance > 1e-6
        ? Math.min(1, (BODY_POSE_TARGET_SPEED * FIXED_TIMESTEP) / targetDistance)
        : 1;
      assistedBodyQuaternion.slerp(requestedTarget, targetBlend).normalize();
      const error = assistedBodyQuaternion.clone()
        .multiply(quaternion.clone().invert())
        .normalize();
      if (error.w < 0) {
        error.x *= -1;
        error.y *= -1;
        error.z *= -1;
        error.w *= -1;
      }
      const halfAngleSin = Math.sqrt(Math.max(1 - error.w ** 2, 0));
      const angle = 2 * Math.acos(THREE.MathUtils.clamp(error.w, -1, 1));
      const axis = halfAngleSin > 1e-6
        ? new THREE.Vector3(error.x, error.y, error.z).divideScalar(halfAngleSin)
        : new THREE.Vector3();
      const angularVelocity = base.angvel();
      const correction = axis.multiplyScalar(angle * BODY_POSE_ASSIST_STIFFNESS);
      correction.x -= angularVelocity.x * BODY_POSE_ASSIST_DAMPING;
      correction.y -= angularVelocity.y * BODY_POSE_ASSIST_DAMPING;
      correction.z -= angularVelocity.z * BODY_POSE_ASSIST_DAMPING;
      if (correction.length() > BODY_POSE_ASSIST_MAX_TORQUE) {
        correction.setLength(BODY_POSE_ASSIST_MAX_TORQUE);
      }
      const torqueStep = correction.clone().sub(bodyPoseTorque);
      const maximumTorqueStep = BODY_POSE_ASSIST_MAX_TORQUE_RATE * FIXED_TIMESTEP;
      if (torqueStep.length() > maximumTorqueStep) {
        torqueStep.setLength(maximumTorqueStep);
      }
      bodyPoseTorque.add(torqueStep);
      base.addTorque(
        { x: bodyPoseTorque.x, y: bodyPoseTorque.y, z: bodyPoseTorque.z },
        true,
      );
      return;
    }
    bodyPoseTorque.set(0, 0, 0);
    if (firmwareState?.mode !== "STAND" && !isWalkingMode(firmwareState?.mode)) return;

    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(quaternion);
    const targetUp = new THREE.Vector3(0, 1, 0);
    const correction = up.cross(targetUp)
      .multiplyScalar(BODY_UPRIGHT_ASSIST_STIFFNESS);
    const angularVelocity = base.angvel();
    correction.x -= angularVelocity.x * BODY_UPRIGHT_ASSIST_DAMPING;
    correction.z -= angularVelocity.z * BODY_UPRIGHT_ASSIST_DAMPING;
    correction.y = 0;
    if (isWalkingMode(firmwareState?.mode) && gaitHeadingForward) {
      const currentForward = new THREE.Vector3(1, 0, 0).applyQuaternion(quaternion);
      currentForward.y = 0;
      if (currentForward.lengthSq() > 1e-8) {
        currentForward.normalize();
        const headingError = Math.atan2(
          currentForward.z * gaitHeadingForward.x -
            currentForward.x * gaitHeadingForward.z,
          currentForward.dot(gaitHeadingForward),
        );
        correction.y = headingError * GAIT_HEADING_STIFFNESS -
          angularVelocity.y * GAIT_HEADING_DAMPING;
      }
    }
    if (correction.length() > BODY_UPRIGHT_ASSIST_MAX_TORQUE) {
      correction.setLength(BODY_UPRIGHT_ASSIST_MAX_TORQUE);
    }
    base.addTorque({ x: correction.x, y: 0, z: correction.z }, true);
  }

  function update(deltaSeconds, firmwareState, visualLegs, servoReference) {
    setTargets(firmwareState, visualLegs, servoReference);
    const mode = firmwareState?.mode || null;
    const modeChanged = mode !== previousMode;
    if (modeChanged) {
      // Foot holds are rate-limited, so carrying the previous mode's force
      // into the next controller produces a visible kick. Every mode owns a
      // fresh set of anchors and force history.
      clearPlanarHoldForces();
    }
    if (mode === "TILT" && previousMode !== "TILT") {
      const translation = base.translation();
      const rotation = base.rotation();
      assistedBodyQuaternion.set(rotation.x, rotation.y, rotation.z, rotation.w).normalize();
      tiltReferenceQuaternion.copy(levelHeadingQuaternion(assistedBodyQuaternion));
      bodyPoseTargetQuaternion = tiltReferenceQuaternion.clone();
      tiltPlanarAnchor = { x: translation.x, z: translation.z };
      tiltFootAnchors = legRuntimes.map((runtime) => {
        const position = runtime.footCollider.translation();
        return { x: position.x, y: position.y, z: position.z };
      });
      const commandedReach = Number(firmwareState?.pose_z_mm) / 1000;
      tiltHeightReference = {
        baseHeight: translation.y,
        commandedReach: Number.isFinite(commandedReach)
          ? commandedReach
          : tiltFootAnchors.reduce(
              (sum, anchor) => sum + Math.max(0, translation.y - anchor.y + FOOT_RADIUS),
              0,
            ) / tiltFootAnchors.length,
      };
    } else if (mode !== "TILT") {
      tiltPlanarAnchor = null;
      tiltFootAnchors = null;
      tiltHeightReference = null;
      bodyPoseTargetQuaternion = null;
    }
    if (isWalkingMode(mode) && (!isWalkingMode(previousMode) || modeChanged)) {
      gaitFootAnchors = Array(LEG_SPECS.length).fill(null);
      if (!isWalkingMode(previousMode) || !gaitHeadingForward) {
        const rotation = base.rotation();
        gaitHeadingForward = new THREE.Vector3(1, 0, 0).applyQuaternion(
          new THREE.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        );
        gaitHeadingForward.y = 0;
        gaitHeadingForward.normalize();
      }
    } else if (!isWalkingMode(mode)) {
      gaitFootAnchors = Array(LEG_SPECS.length).fill(null);
      gaitHeadingForward = null;
    }
    if (mode !== "TILT" && !isWalkingMode(mode)) {
      clearPlanarHoldForces();
    }
    previousMode = mode;
    if (
      firmwareState?.mode === "STAND" &&
      LEG_SPECS.every((spec, legIndex) =>
        spec.defaults.every(
          (neutralTarget, jointIndex) =>
            Math.abs(latestTargets[(legIndex * 3) + jointIndex] - neutralTarget) < 0.03,
        ),
      )
    ) {
      neutralStandTime += deltaSeconds;
    } else {
      neutralStandTime = 0;
    }
    accumulator = Math.min(accumulator + deltaSeconds, FIXED_TIMESTEP * 8);
    while (accumulator >= FIXED_TIMESTEP) {
      driveMotors();
      driveBodyHeight(firmwareState);
      driveBodyPlanarPosition(firmwareState);
      driveFootPlanarAnchors(firmwareState);
      driveGaitStanceAnchors(firmwareState);
      driveBodyAttitude(firmwareState);
      world.step();
      environmentBalls.forEach((ball) => {
        const position = ball.body.translation();
        if (
          position.y < -0.25 ||
          Math.abs(position.x) > 5 ||
          Math.abs(position.z) > 5
        ) {
          ball.body.setTranslation(
            {
              x: ball.position[0],
              y: ball.position[1],
              z: ball.position[2],
            },
            true,
          );
          ball.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
          ball.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
        }
      });
      accumulator -= FIXED_TIMESTEP;
      elapsed += FIXED_TIMESTEP;
    }

    const translation = base.translation();
    const rotation = base.rotation();
    const linearVelocity = base.linvel();
    const angularVelocity = base.angvel();
    const footContactStates = legRuntimes.map(readFootContact);
    const footContacts = footContactStates.map((contact) => contact.touching);
    const footSupportHeights = footContactStates.map((contact) => contact.supportHeight);
    const footPositions = legRuntimes.map((runtime) => {
      const position = runtime.footCollider.translation();
      return [position.x, position.y, position.z];
    });
    const supportSurfaceHeights = footSupportHeights.filter(Number.isFinite);
    supportSurfaceHeights.sort((a, b) => a - b);
    const supportMiddle = Math.floor(supportSurfaceHeights.length / 2);
    const supportHeight = supportSurfaceHeights.length === 0
      ? 0
      : supportSurfaceHeights.length % 2 === 0
        ? 0.5 * (
            supportSurfaceHeights[supportMiddle - 1] +
            supportSurfaceHeights[supportMiddle]
          )
        : supportSurfaceHeights[supportMiddle];
    const bodyClearance = translation.y - supportHeight;
    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(
      new THREE.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    );

    let resetReason = null;
    if (translation.y < -0.15) resetReason = "below-floor";
    else if (!Number.isFinite(translation.y)) resetReason = "non-finite";
    else if (elapsed > 2 && firmwareState?.mode !== "STAND" && up.y < -0.35) {
      resetReason = "inverted";
    } else if (
      elapsed > 2 &&
      firmwareState?.mode === "STAND" &&
      neutralStandTime > 0.5 &&
      (translation.y < 0.24 || up.y < 0.65)
    ) {
      resetReason = "failed-stand";
    } else if (
      elapsed > 2 &&
      firmwareState?.mode === "TILT" &&
      (translation.y < 0.18 || up.y < 0.35)
    ) {
      resetReason = "failed-tilt";
    } else if (
      elapsed > 2 &&
      isWalkingMode(firmwareState?.mode) &&
      (translation.y < 0.15 || up.y < 0.25)
    ) {
      resetReason = "failed-gait";
    }
    if (resetReason) {
      resetCount += 1;
      lastResetReason = resetReason;
      buildWorld({ preserveEnvironment: true });
      return update(0, firmwareState, visualLegs, servoReference);
    }

    return {
      engine: "Rapier 3D",
      proxy: "12-joint Domino URDF geometry",
      massModel: dominoMassModel,
      basePosition: [translation.x, translation.y, translation.z],
      baseQuaternion: [rotation.x, rotation.y, rotation.z, rotation.w],
      bodyPoseTargetQuaternion: bodyPoseTargetQuaternion
        ? [
            bodyPoseTargetQuaternion.x,
            bodyPoseTargetQuaternion.y,
            bodyPoseTargetQuaternion.z,
            bodyPoseTargetQuaternion.w,
          ]
        : null,
      linearVelocity: [linearVelocity.x, linearVelocity.y, linearVelocity.z],
      angularVelocity: [angularVelocity.x, angularVelocity.y, angularVelocity.z],
      footContacts,
      footPositions,
      footSupportHeights,
      footRadius: FOOT_RADIUS,
      tiltFootAnchors: tiltFootAnchors
        ? tiltFootAnchors.map((anchor) => [anchor.x, anchor.y, anchor.z])
        : null,
      contactCount: footContacts.filter(Boolean).length,
      bodyHeight: translation.y,
      bodyClearance,
      bodyAltitude: translation.y,
      supportHeight,
      baseTiltDegrees: THREE.MathUtils.radToDeg(
        Math.acos(THREE.MathUtils.clamp(up.y, -1, 1)),
      ),
      commandedBodyHeight,
      targets: [...latestTargets],
      drivenTargets: [...drivenTargets],
      neutralPoseError: Math.max(
        ...LEG_SPECS.flatMap((spec, legIndex) =>
          spec.defaults.map(
            (neutralTarget, jointIndex) =>
              Math.abs(latestTargets[(legIndex * 3) + jointIndex] - neutralTarget),
          ),
        ),
      ),
      resetCount,
      lastResetReason,
      neutralStandTime,
      simTime: elapsed,
      environmentBalls: environmentBalls.map((ball) => {
        const position = ball.body.translation();
        const rotation = ball.body.rotation();
        const linearVelocity = ball.body.linvel();
        return {
          id: ball.id,
          position: [position.x, position.y, position.z],
          quaternion: [rotation.x, rotation.y, rotation.z, rotation.w],
          linearVelocity: [linearVelocity.x, linearVelocity.y, linearVelocity.z],
          sleeping: ball.body.isSleeping(),
        };
      }),
    };
  }

  function reset(reason = "manual") {
    resetCount += 1;
    lastResetReason = reason;
    buildWorld({ preserveEnvironment: true });
  }

  buildWorld();
  return { update, reset };
}
