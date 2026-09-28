import * as THREE from "three";

// CAD/firmware Rz(yaw) * Ry(pitch) * Rx(roll), with CAD [x,y,z]
// mapped to scene [x,z,-y]. In scene axes this is Ry * Rz * Rx.
export const BODY_EULER_ORDER = "YZX";

export function bodyPoseQuaternion(rollDeg, pitchDeg, yawDeg, target = new THREE.Quaternion()) {
  const radians = (value) => THREE.MathUtils.degToRad(Number.isFinite(Number(value)) ? Number(value) : 0);
  return target.setFromEuler(new THREE.Euler(
    radians(rollDeg), radians(yawDeg), -radians(pitchDeg), BODY_EULER_ORDER,
  ));
}
