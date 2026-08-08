import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";

const FIXED_TIMESTEP = 1 / 120;
const ROBOT_GROUP = (0x0001 << 16) | 0x0002;
const WORLD_GROUP = (0x0002 << 16) | 0x0001;
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

const obstacleSpecs = [
  { size: [1.4, 0.12, 0.8], position: [2.4, 0.06, -1.8], yaw: 0.2 },
  { size: [0.9, 0.24, 0.9], position: [-2.2, 0.12, -2.1], yaw: -0.15 },
  { size: [1.8, 0.07, 0.55], position: [-1.8, 0.035, 1.9], yaw: 0.45 },
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

export async function createDominoPhysics() {
  await RAPIER.init({});

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
  let previousMode = null;
  let assistedBodyQuaternion = new THREE.Quaternion();
  const bodyPoseTorque = new THREE.Vector3();

  function createStaticBox(size, position, yaw = 0) {
    const rotation = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(position[0], position[1], position[2])
        .setRotation(quaternionToRapier(rotation)),
    );
    return world.createCollider(
      RAPIER.ColliderDesc.cuboid(size[0] / 2, size[1] / 2, size[2] / 2)
        .setFriction(0.9)
        .setRestitution(0)
        .setCollisionGroups(WORLD_GROUP),
      body,
    );
  }

  function buildLeg(spec) {
    const basePosition = new THREE.Vector3(0, INITIAL_BASE_HEIGHT, 0);
    const shoulderAnchor = new THREE.Vector3(...spec.shoulderAnchor);
    const upperAnchor = new THREE.Vector3(...spec.upperAnchor);
    const shoulderAxis = new THREE.Vector3(...spec.shoulderAxis);
    const pitchAxis = new THREE.Vector3(0, 0, -1);

    const hipRotation = new THREE.Quaternion().setFromAxisAngle(shoulderAxis, spec.defaults[0]);
    const hipPosition = basePosition.clone().add(shoulderAnchor);
    const hip = world.createRigidBody(bodyDescriptor(hipPosition, hipRotation));
    world.createCollider(
      robotCollider(RAPIER.ColliderDesc.cuboid(0.03, 0.015, 0.0275), 0.1),
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
      robotCollider(RAPIER.ColliderDesc.capsule(0.066, 0.014), 0.12),
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
      robotCollider(RAPIER.ColliderDesc.capsule(0.0645, 0.012), 0.1),
      lower,
    );
    const footCollider = world.createCollider(
      robotCollider(
        RAPIER.ColliderDesc.ball(FOOT_RADIUS).setTranslation(0, -LEG_LENGTHS.lower / 2, 0),
        0.03,
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

  function buildWorld() {
    if (world) world.free();
    world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    world.timestep = FIXED_TIMESTEP;
    world.numSolverIterations = 12;
    world.numAdditionalFrictionIterations = 8;
    world.numInternalPgsIterations = 2;
    createStaticBox([40, 0.04, 40], [0, -0.02, 0]);
    obstacleSpecs.forEach((obstacle) =>
      createStaticBox(obstacle.size, obstacle.position, obstacle.yaw),
    );

    base = world.createRigidBody(
      bodyDescriptor(new THREE.Vector3(0, INITIAL_BASE_HEIGHT, 0), new THREE.Quaternion(), 20),
    );
    world.createCollider(
      robotCollider(RAPIER.ColliderDesc.cuboid(0.215, 0.025, 0.1025), 1.2, 0.55),
      base,
    );
    legRuntimes = LEG_SPECS.map(buildLeg);
    latestTargets = LEG_SPECS.flatMap((spec) => spec.defaults);
    drivenTargets = [...latestTargets];
    accumulator = 0;
    elapsed = 0;
    neutralStandTime = 0;
    tiltPlanarAnchor = null;
    tiltFootAnchors = null;
    previousMode = null;
    assistedBodyQuaternion.identity();
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
    commandedBodyHeight = commandedReach + BODY_HEIGHT_FROM_FOOT_REACH_OFFSET;
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
      const position = runtime.footCollider.translation();
      const velocity = runtime.lower.linvel();
      const correction = new THREE.Vector3(
        FOOT_PLANAR_HOLD_STIFFNESS * (anchor.x - position.x) -
          FOOT_PLANAR_HOLD_DAMPING * velocity.x,
        0,
        FOOT_PLANAR_HOLD_STIFFNESS * (anchor.z - position.z) -
          FOOT_PLANAR_HOLD_DAMPING * velocity.z,
      );
      if (correction.length() > FOOT_PLANAR_HOLD_MAX_FORCE) {
        correction.setLength(FOOT_PLANAR_HOLD_MAX_FORCE);
      }
      const forceStep = correction.clone().sub(runtime.planarHoldForce);
      const maximumForceStep = FOOT_PLANAR_HOLD_MAX_FORCE_RATE * FIXED_TIMESTEP;
      if (forceStep.length() > maximumForceStep) {
        forceStep.setLength(maximumForceStep);
      }
      runtime.planarHoldForce.add(forceStep);
      runtime.lower.resetForces(true);
      runtime.lower.addForce(
        { x: runtime.planarHoldForce.x, y: 0, z: runtime.planarHoldForce.z },
        true,
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
      const requestedTarget = targetBodyQuaternion(firmwareState);
      if (!requestedTarget) return;
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
    if (firmwareState?.mode !== "STAND") return;

    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(quaternion);
    const targetUp = new THREE.Vector3(0, 1, 0);
    const correction = up.cross(targetUp)
      .multiplyScalar(BODY_UPRIGHT_ASSIST_STIFFNESS);
    const angularVelocity = base.angvel();
    correction.x -= angularVelocity.x * BODY_UPRIGHT_ASSIST_DAMPING;
    correction.z -= angularVelocity.z * BODY_UPRIGHT_ASSIST_DAMPING;
    correction.y = 0;
    if (correction.length() > BODY_UPRIGHT_ASSIST_MAX_TORQUE) {
      correction.setLength(BODY_UPRIGHT_ASSIST_MAX_TORQUE);
    }
    base.addTorque({ x: correction.x, y: 0, z: correction.z }, true);
  }

  function update(deltaSeconds, firmwareState, visualLegs, servoReference) {
    setTargets(firmwareState, visualLegs, servoReference);
    const mode = firmwareState?.mode || null;
    if (mode === "TILT" && previousMode !== "TILT") {
      const translation = base.translation();
      const rotation = base.rotation();
      assistedBodyQuaternion.set(rotation.x, rotation.y, rotation.z, rotation.w).normalize();
      tiltPlanarAnchor = { x: translation.x, z: translation.z };
      tiltFootAnchors = legRuntimes.map((runtime) => {
        const position = runtime.footCollider.translation();
        return { x: position.x, z: position.z };
      });
    } else if (mode !== "TILT") {
      tiltPlanarAnchor = null;
      tiltFootAnchors = null;
      legRuntimes.forEach((runtime) => runtime.planarHoldForce.set(0, 0, 0));
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
      driveBodyAttitude(firmwareState);
      world.step();
      accumulator -= FIXED_TIMESTEP;
      elapsed += FIXED_TIMESTEP;
    }

    const translation = base.translation();
    const rotation = base.rotation();
    const linearVelocity = base.linvel();
    const angularVelocity = base.angvel();
    const footContacts = legRuntimes.map((runtime) => {
      let touching = false;
      world.contactPairsWith(runtime.footCollider, (otherCollider) => {
        world.contactPair(runtime.footCollider, otherCollider, (manifold) => {
          if (manifold.numSolverContacts() > 0) touching = true;
        });
      });
      return touching;
    });
    const footPositions = legRuntimes.map((runtime) => {
      const position = runtime.footCollider.translation();
      return [position.x, position.y, position.z];
    });
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
    }
    if (resetReason) {
      resetCount += 1;
      lastResetReason = resetReason;
      buildWorld();
      return update(0, firmwareState, visualLegs, servoReference);
    }

    return {
      engine: "Rapier 3D",
      proxy: "12-joint Domino URDF geometry",
      basePosition: [translation.x, translation.y, translation.z],
      baseQuaternion: [rotation.x, rotation.y, rotation.z, rotation.w],
      linearVelocity: [linearVelocity.x, linearVelocity.y, linearVelocity.z],
      angularVelocity: [angularVelocity.x, angularVelocity.y, angularVelocity.z],
      footContacts,
      footPositions,
      contactCount: footContacts.filter(Boolean).length,
      bodyHeight: translation.y,
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
    };
  }

  function reset(reason = "manual") {
    resetCount += 1;
    lastResetReason = reason;
    buildWorld();
  }

  buildWorld();
  return { update, reset };
}
