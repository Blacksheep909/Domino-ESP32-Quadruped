import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";

import { assemblyOrigin, expectedMeshCount, legs, standServoReference } from "./domino-config.js";
import { point2, solveLinkagePose } from "./linkage.js";
import { createDominoPhysics, targetBodyQuaternion } from "./physics.js";
import "./styles.css";

const canvas = document.querySelector("#scene");
const demoSelection = new URLSearchParams(window.location.search).get("demo");
const demoMode = ["1", "tilt", "roll", "roll-negative"].includes(demoSelection);
const THEME_STORAGE_KEY = "domino-theme";
let currentTheme = localStorage.getItem(THEME_STORAGE_KEY);
if (currentTheme !== "light" && currentTheme !== "dark") {
  currentTheme = window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light";
}
document.documentElement.dataset.theme = currentTheme;
const SERVO_TRAVEL_DEG = 45;
const CAD_FOOT_RADIUS = 0.012;
const VISUAL_GROUND_SYNC_MAX_OFFSET = 0.05;
const VISUAL_GROUND_SYNC_MAX_ERROR_SPREAD = 0.04;
const VISUAL_GROUND_SYNC_RESPONSE = 12;
const VISUAL_GROUND_SYNC_MAX_RATE = 0.08;
const VISUAL_FLOOR_CLEARANCE = 0.0002;
const VISUAL_BASE_POSITION_RESPONSE = 22;
const VISUAL_BASE_ROTATION_RESPONSE = 24;
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xdededa);
scene.fog = new THREE.Fog(0xdededa, 6, 26);

const PERSPECTIVE_FOV_DEG = 48;
const perspectiveCamera = new THREE.PerspectiveCamera(PERSPECTIVE_FOV_DEG, 1, 0.01, 100);
perspectiveCamera.position.set(1.05, 0.74, 1.02);
const orthographicCamera = new THREE.OrthographicCamera(-0.5, 0.5, 0.5, -0.5, 0.01, 100);
let orthographicViewHeight = 1;
let camera = perspectiveCamera;

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0.34, 0);
controls.minPolarAngle = 0.012;
controls.maxPolarAngle = Math.PI - 0.012;
controls.minDistance = 0.38;
controls.maxDistance = 4;
controls.mouseButtons.MIDDLE = THREE.MOUSE.PAN;
const cameraTargetOffset = new THREE.Vector3();
const robotCameraAnchor = new THREE.Vector3();
const defaultCameraOffset = new THREE.Vector3(1.05, 0.40, 1.02);
let middleButtonPanning = false;

const cameraGizmo = document.querySelector(".camera-gizmo");
const cameraGizmoCanvas = cameraGizmo.querySelector("canvas");
const cameraGizmoContext = cameraGizmoCanvas.getContext("2d");
const CAMERA_GIZMO_SIZE = 112;
const CAMERA_GIZMO_RADIUS = 38;
const CAMERA_SNAP_DURATION_MS = 320;
// Present the robot's CAD/firmware coordinate convention. The STL assembly is
// rotated into Three.js, so CAD +Z is world +Y and CAD +Y is world -Z.
const cameraAxes = [
  { name: "+X", vector: new THREE.Vector3(1, 0, 0), color: "#d85b50", positive: true },
  { name: "-X", vector: new THREE.Vector3(-1, 0, 0), color: "#d85b50", positive: false },
  { name: "+Y", vector: new THREE.Vector3(0, 0, -1), color: "#61ad72", positive: true },
  { name: "-Y", vector: new THREE.Vector3(0, 0, 1), color: "#61ad72", positive: false },
  { name: "+Z", vector: new THREE.Vector3(0, 1, 0), color: "#4d91cb", positive: true },
  { name: "-Z", vector: new THREE.Vector3(0, -1, 0), color: "#4d91cb", positive: false },
].map((axis) => ({
  ...axis,
  button: cameraGizmo.querySelector(`[data-axis="${axis.name}"]`),
  projected: new THREE.Vector3(),
}));
const cameraInverseQuaternion = new THREE.Quaternion();
let cameraSnap = null;
let inspectionGrid = null;
let orthographicAxisDirection = null;
cameraGizmo.dataset.projection = "perspective";

function updateOrthographicProjection() {
  const aspect = window.innerWidth / Math.max(1, window.innerHeight);
  orthographicCamera.left = -orthographicViewHeight * aspect / 2;
  orthographicCamera.right = orthographicViewHeight * aspect / 2;
  orthographicCamera.top = orthographicViewHeight / 2;
  orthographicCamera.bottom = -orthographicViewHeight / 2;
  orthographicCamera.updateProjectionMatrix();
}

function usePerspectiveCamera() {
  if (camera === perspectiveCamera) return;
  const visibleHeight = orthographicViewHeight / orthographicCamera.zoom;
  const distance = visibleHeight /
    (2 * Math.tan(THREE.MathUtils.degToRad(PERSPECTIVE_FOV_DEG) / 2));
  const viewDirection = orthographicCamera.position.clone()
    .sub(controls.target)
    .normalize();
  // Free orbit uses world Y-up. Move a vertical inspection view a fraction
  // off-pole before handing it back to OrbitControls.
  if (Math.abs(viewDirection.y) > 0.9) {
    viewDirection.z = 0.012;
    viewDirection.normalize();
  }
  perspectiveCamera.position.copy(controls.target)
    .add(viewDirection.multiplyScalar(distance));
  perspectiveCamera.up.set(0, 1, 0);
  perspectiveCamera.lookAt(controls.target);
  perspectiveCamera.zoom = 1;
  perspectiveCamera.updateProjectionMatrix();
  camera = perspectiveCamera;
  controls.object = camera;
  cameraGizmo.dataset.projection = "perspective";
  orthographicAxisDirection = null;
  if (inspectionGrid) inspectionGrid.visible = false;
}

function configureInspectionGrid(direction) {
  if (!inspectionGrid) return;
  orthographicAxisDirection = direction.clone().normalize();
  inspectionGrid.rotation.set(0, 0, 0);
  if (Math.abs(direction.x) > 0.9) inspectionGrid.rotation.z = Math.PI / 2;
  else if (Math.abs(direction.z) > 0.9) inspectionGrid.rotation.x = Math.PI / 2;
  inspectionGrid.position.copy(controls.target)
    .addScaledVector(orthographicAxisDirection, -0.55);
  inspectionGrid.visible = true;
}

function useOrthographicCamera(direction) {
  if (camera === orthographicCamera) return;
  const distance = perspectiveCamera.position.distanceTo(controls.target);
  orthographicViewHeight =
    2 * distance * Math.tan(THREE.MathUtils.degToRad(PERSPECTIVE_FOV_DEG) / 2);
  orthographicCamera.position.copy(perspectiveCamera.position);
  orthographicCamera.quaternion.copy(perspectiveCamera.quaternion);
  orthographicCamera.up.copy(perspectiveCamera.up);
  orthographicCamera.zoom = 1;
  updateOrthographicProjection();
  camera = orthographicCamera;
  controls.object = camera;
  cameraGizmo.dataset.projection = "orthographic";
  configureInspectionGrid(direction);
}

function cameraUpForDirection(direction) {
  if (Math.abs(direction.y) > 0.9) {
    return new THREE.Vector3(0, 0, direction.y > 0 ? -1 : 1);
  }
  return new THREE.Vector3(0, 1, 0);
}

function snapCameraToAxis(direction, name) {
  usePerspectiveCamera();
  const distance = THREE.MathUtils.clamp(
    camera.position.distanceTo(controls.target),
    controls.minDistance,
    controls.maxDistance,
  );
  cameraSnap = {
    startedAt: performance.now(),
    startOffset: camera.position.clone().sub(controls.target),
    endOffset: direction.clone().normalize().multiplyScalar(distance),
    startUp: camera.up.clone(),
    endUp: cameraUpForDirection(direction),
    axisDirection: direction.clone(),
  };
  controls.enabled = false;
  cameraGizmo.dataset.view = name;
}

cameraAxes.forEach((axis) => {
  axis.button.addEventListener("click", () => {
    const selectedAxis = cameraGizmo.dataset.view === axis.name
      ? cameraAxes.find((candidate) => candidate.name ===
        `${axis.name.startsWith("+") ? "-" : "+"}${axis.name.slice(1)}`)
      : axis;
    snapCameraToAxis(selectedAxis.vector, selectedAxis.name);
  });
});
renderer.domElement.addEventListener("pointerdown", (event) => {
  cameraGizmo.dataset.view = "";
  // Orthographic OrbitControls support panning. Only rotation gestures leave
  // an axis inspection view; middle-button target movement stays orthographic.
  if (!cameraSnap && event.button !== 1) usePerspectiveCamera();
  if (event.button === 1) middleButtonPanning = true;
}, { capture: true });
function finishMiddleButtonPan(event) {
  if (event.button !== 1 || !middleButtonPanning) return;
  middleButtonPanning = false;
  robotCameraAnchor.set(robotWorld.position.x, 0.32, robotWorld.position.z);
  cameraTargetOffset.copy(controls.target).sub(robotCameraAnchor);
}
window.addEventListener("pointerup", finishMiddleButtonPan, { capture: true });
window.addEventListener("pointercancel", () => {
  middleButtonPanning = false;
});

function updateCameraSnap(now) {
  if (!cameraSnap) return false;
  const progress = THREE.MathUtils.clamp(
    (now - cameraSnap.startedAt) / CAMERA_SNAP_DURATION_MS,
    0,
    1,
  );
  const eased = 1 - (1 - progress) ** 3;
  const offset = cameraSnap.startOffset.clone().lerp(cameraSnap.endOffset, eased);
  camera.position.copy(controls.target).add(offset);
  camera.up.copy(cameraSnap.startUp).lerp(cameraSnap.endUp, eased).normalize();
  camera.lookAt(controls.target);

  if (progress >= 1) {
    const completedAxisDirection = cameraSnap.axisDirection;
    cameraSnap = null;
    useOrthographicCamera(completedAxisDirection);
    controls.enabled = true;
    controls.update();
  }
  return true;
}

function updateCameraGizmo() {
  cameraInverseQuaternion.copy(camera.quaternion).invert();
  const center = CAMERA_GIZMO_SIZE / 2;
  cameraGizmoContext.clearRect(0, 0, CAMERA_GIZMO_SIZE, CAMERA_GIZMO_SIZE);

  cameraAxes.forEach((axis) => {
    axis.projected.copy(axis.vector).applyQuaternion(cameraInverseQuaternion);
  });

  [...cameraAxes]
    .sort((a, b) => a.projected.z - b.projected.z)
    .forEach((axis) => {
      const endX = center + axis.projected.x * CAMERA_GIZMO_RADIUS;
      const endY = center - axis.projected.y * CAMERA_GIZMO_RADIUS;
      cameraGizmoContext.beginPath();
      cameraGizmoContext.moveTo(center, center);
      cameraGizmoContext.lineTo(endX, endY);
      cameraGizmoContext.strokeStyle = axis.color;
      cameraGizmoContext.globalAlpha = axis.positive ? 0.82 : 0.32;
      cameraGizmoContext.lineWidth = axis.positive ? 2 : 1;
      cameraGizmoContext.setLineDash(axis.positive ? [] : [3, 3]);
      cameraGizmoContext.stroke();

      const depthScale = 0.86 + (axis.projected.z + 1) * 0.08;
      axis.button.style.left = `${endX}px`;
      axis.button.style.top = `${endY}px`;
      axis.button.style.scale = depthScale.toFixed(3);
      axis.button.style.zIndex = `${Math.round((axis.projected.z + 1) * 10) + 2}`;
      axis.button.style.opacity = `${axis.positive ? 1 : 0.7}`;
    });

  cameraGizmoContext.globalAlpha = 1;
  cameraGizmoContext.setLineDash([]);
}

scene.add(new THREE.HemisphereLight(0xf4f7f8, 0x485159, 2.1));
const sun = new THREE.DirectionalLight(0xffffff, 3.3);
sun.position.set(-3, 6, 4);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = -5;
sun.shadow.camera.right = 5;
sun.shadow.camera.top = 5;
sun.shadow.camera.bottom = -5;
scene.add(sun);

const groundMaterial = new THREE.MeshStandardMaterial({ color: 0xbdbdb8, roughness: 0.9, metalness: 0.02 });
const ground = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), groundMaterial);
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);

const grid = new THREE.GridHelper(40, 80, 0x888a85, 0xa7a8a3);
grid.position.y = 0.002;
grid.material.opacity = 0.32;
grid.material.transparent = true;
scene.add(grid);

inspectionGrid = new THREE.GridHelper(20, 100, 0x777a74, 0xa9aaa5);
inspectionGrid.material.opacity = 0.24;
inspectionGrid.material.transparent = true;
inspectionGrid.material.depthWrite = false;
inspectionGrid.visible = false;
scene.add(inspectionGrid);

function applyTheme(theme, persist = true) {
  currentTheme = theme === "dark" ? "dark" : "light";
  document.documentElement.dataset.theme = currentTheme;
  const dark = currentTheme === "dark";
  const sceneColor = dark ? 0x30312f : 0xdededa;
  scene.background.set(sceneColor);
  scene.fog.color.set(sceneColor);
  groundMaterial.color.set(dark ? 0x656661 : 0xbdbdb8);
  renderer.toneMappingExposure = dark ? 1.18 : 1.05;

  const toggle = document.querySelector("#theme-toggle");
  toggle.setAttribute("aria-pressed", String(dark));
  toggle.setAttribute("aria-label", dark ? "Dark theme" : "Light theme");
  toggle.title = dark ? "Switch to light theme" : "Switch to dark theme";
  document.querySelector("#theme-icon").textContent = dark ? "\u263d" : "\u263c";
  document.querySelector("#theme-label").textContent = dark ? "Dark theme" : "Light theme";
  if (persist) localStorage.setItem(THEME_STORAGE_KEY, currentTheme);
}

document.querySelector("#theme-toggle").addEventListener("click", () => {
  applyTheme(currentTheme === "dark" ? "light" : "dark");
});
applyTheme(currentTheme, false);

function addObstacle(size, position, rotationY = 0) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(...size),
    new THREE.MeshStandardMaterial({ color: 0x59656c, roughness: 0.78 }),
  );
  mesh.position.set(...position);
  mesh.rotation.y = rotationY;
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  scene.add(mesh);
}

addObstacle([1.4, 0.12, 0.8], [2.4, 0.06, -1.8], 0.2);
addObstacle([0.9, 0.24, 0.9], [-2.2, 0.12, -2.1], -0.15);
addObstacle([1.8, 0.07, 0.55], [-1.8, 0.035, 1.9], 0.45);

const frameMaterial = new THREE.MeshStandardMaterial({
  color: 0x252d32,
  roughness: 0.35,
  metalness: 0.58,
});
const legMaterial = new THREE.MeshStandardMaterial({
  color: 0x11171a,
  roughness: 0.48,
  metalness: 0.28,
});
const tpuMaterial = new THREE.MeshStandardMaterial({
  color: 0x090d0f,
  roughness: 0.96,
  metalness: 0.0,
});
const pinMaterial = new THREE.MeshStandardMaterial({
  color: 0x818b90,
  roughness: 0.32,
  metalness: 0.78,
  transparent: true,
});
const ACTIVE_JOINT_COLOR = 0xf07d46;
const PASSIVE_JOINT_COLOR = 0x55c3d7;
let jointOverlayVisible = false;
const jointOverlayOpacity = 0.78;
let selectedJointLeg = "FR";
let bodyReferenceOverlay = null;

const robotWorld = new THREE.Group();
robotWorld.position.y = 0.34;
scene.add(robotWorld);

const cadAlignment = new THREE.Group();
robotWorld.add(cadAlignment);

const cadRoot = new THREE.Group();
cadRoot.rotation.x = -Math.PI / 2;
cadAlignment.add(cadRoot);

const loader = new STLLoader();
const linkageRuntimes = [];
let loadedMeshCount = 0;
let firmwareState = null;
let visualServoAngles = null;
const neutralServoAngles = [...standServoReference];
const SERVO_VISUAL_RESPONSE = 24;
let standRequested = false;
let tiltRequested = false;
let manualStandOverride = null;
let manualTiltOverride = null;
let manualHeightOverride = null;
let observedPhysicalStand = null;
let observedPhysicalTilt = null;
let observedPhysicalHeight = null;
let forwardInput = 0;
let turnInput = 0;
let rollInput = 0;
let robotYaw = 0;
let clientInputSnapshot = { source: "keyboard", name: "KEYBOARD", axes: [] };
let physics = null;
let physicsState = null;
const visualBasePosition = new THREE.Vector3();
const visualBaseQuaternion = new THREE.Quaternion();
let visualBaseInitialized = false;
let visualPhysicsResetCount = -1;
const visualFootPosition = new THREE.Vector3();
const bodyEuler = new THREE.Euler(0, 0, 0, "YXZ");
const PIN_TOLERANCE_MM = 0.5;
const FOOT_SYMMETRY_TOLERANCE_MM = 1.0;
let previousPinClosureHealthy = null;
let previousFootSymmetryHealthy = null;

const loading = document.querySelector("#loading");
const loadingProgress = document.querySelector("#loading-progress");
const loadingDetail = document.querySelector("#loading-detail");

function linkageRuntimesReady() {
  return linkageRuntimes.length === legs.length &&
    legs.every((_, index) => Boolean(linkageRuntimes[index]));
}

function v3(values) {
  return new THREE.Vector3(values[0], values[1], values[2]);
}

function planarTransform(group, originalA3, originalB3, currentA, currentB, hip) {
  const originalA = point2(originalA3);
  const originalB = point2(originalB3);
  const originalAngle = Math.atan2(originalB.z - originalA.z, originalB.x - originalA.x);
  const currentAngle = Math.atan2(currentB.z - currentA.z, currentB.x - currentA.x);
  group.position.set(currentA.x - hip[0], originalA3[1] - hip[1], currentA.z - hip[2]);
  group.rotation.set(0, -(currentAngle - originalAngle), 0);
}

async function loadMesh(meshName, material, sourceTranslation, anchor, parent) {
  const geometry = await loader.loadAsync(`/cad/${meshName}.stl`);
  geometry.scale(0.001, 0.001, 0.001);
  geometry.translate(
    sourceTranslation[0] - anchor[0],
    sourceTranslation[1] - anchor[1],
    sourceTranslation[2] - anchor[2],
  );
  geometry.computeVertexNormals();
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  parent.add(mesh);
  loadedMeshCount += 1;
  loadingProgress.value = loadedMeshCount;
  loadingDetail.textContent = `${meshName}.stl`;
  document.querySelector("#mesh-count").textContent = `${loadedMeshCount} / ${expectedMeshCount}`;
  return mesh;
}

function bodyGroup(legRoot, anchor) {
  const group = new THREE.Group();
  group.position.copy(v3(anchor).sub(v3(legRoot.userData.hip)));
  legRoot.add(group);
  return group;
}

function createJointLabel(text, color, active) {
  const canvas = document.createElement("canvas");
  canvas.width = 64;
  canvas.height = 64;
  const context = canvas.getContext("2d");
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.beginPath();
  context.arc(32, 32, 28, 0, Math.PI * 2);
  context.fillStyle = "rgba(16, 23, 27, 0.86)";
  context.fill();
  context.strokeStyle = color;
  context.lineWidth = 4;
  context.stroke();
  context.fillStyle = "#f2f5f6";
  context.font = "700 21px Cascadia Mono, Consolas, monospace";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(text, 32, 33);

  const material = new THREE.SpriteMaterial({
    map: new THREE.CanvasTexture(canvas),
    transparent: true,
    opacity: jointOverlayOpacity,
    depthTest: false,
    depthWrite: false,
  });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(0.016, 0.016, 1);
  sprite.renderOrder = 20;
  return sprite;
}

function createJointAnnotation(parent, position, text, active, axisDirection) {
  const color = active ? ACTIVE_JOINT_COLOR : PASSIVE_JOINT_COLOR;
  const group = new THREE.Group();
  group.position.copy(position);
  group.visible = jointOverlayVisible;
  parent.add(group);

  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(active ? 0.006 : 0.0048, 14, 10),
    new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: jointOverlayOpacity,
      depthTest: false,
    }),
  );
  marker.renderOrder = 18;
  group.add(marker);

  const axis = new THREE.ArrowHelper(
    axisDirection,
    axisDirection.clone().multiplyScalar(-0.035),
    0.07,
    color,
    0.012,
    0.007,
  );
  axis.line.material.transparent = true;
  axis.cone.material.transparent = true;
  axis.line.material.depthTest = false;
  axis.cone.material.depthTest = false;
  axis.line.material.opacity = jointOverlayOpacity;
  axis.cone.material.opacity = jointOverlayOpacity;
  axis.renderOrder = 19;
  group.add(axis);

  const label = createJointLabel(
    text,
    `#${color.toString(16).padStart(6, "0")}`,
    active,
  );
  label.position.set(0.013, active ? 0.019 : 0.014, 0);
  group.add(label);
  return { group, marker, axis, label };
}

function createBodyReferenceOverlay() {
  const group = new THREE.Group();
  group.position.z = 0.012;
  group.visible = jointOverlayVisible;
  cadRoot.add(group);

  const plane = new THREE.Mesh(
    new THREE.PlaneGeometry(0.38, 0.18),
    new THREE.MeshBasicMaterial({
      color: 0x78a9b8,
      transparent: true,
      opacity: jointOverlayOpacity * 0.16,
      side: THREE.DoubleSide,
      depthWrite: false,
    }),
  );
  plane.renderOrder = 12;
  group.add(plane);

  const axes = [
    { name: "X", direction: new THREE.Vector3(1, 0, 0), color: 0xd85b50 },
    { name: "Y", direction: new THREE.Vector3(0, 1, 0), color: 0x61ad72 },
    { name: "Z", direction: new THREE.Vector3(0, 0, 1), color: 0x4d91cb },
  ].map((definition) => {
    const arrow = new THREE.ArrowHelper(
      definition.direction,
      new THREE.Vector3(),
      0.16,
      definition.color,
      0.025,
      0.014,
    );
    arrow.line.material.transparent = true;
    arrow.cone.material.transparent = true;
    arrow.line.material.depthTest = false;
    arrow.cone.material.depthTest = false;
    arrow.line.material.opacity = jointOverlayOpacity;
    arrow.cone.material.opacity = jointOverlayOpacity;
    group.add(arrow);

    const label = createJointLabel(
      definition.name,
      `#${definition.color.toString(16).padStart(6, "0")}`,
      true,
    );
    label.position.copy(definition.direction).multiplyScalar(0.18);
    label.scale.set(0.020, 0.020, 1);
    group.add(label);
    return { arrow, label };
  });

  return { group, plane, axes };
}

async function buildLeg(spec, runtimeIndex) {
  const hip = spec.points.hip_origin;
  const legRoot = new THREE.Group();
  legRoot.position.copy(v3(hip).sub(v3(assemblyOrigin)));
  legRoot.userData.hip = hip;
  cadRoot.add(legRoot);

  const groups = {
    ground: bodyGroup(legRoot, hip),
    lower_driver: bodyGroup(legRoot, spec.points.lower_drive),
    coupler: bodyGroup(legRoot, spec.points.lower_passive),
    lower_diagonal: bodyGroup(legRoot, spec.points.lower_coupler),
    lower_closure: bodyGroup(legRoot, spec.points.lower_closure_diagonal),
    upper_driver: bodyGroup(legRoot, spec.points.upper_drive),
    upper_closure: bodyGroup(legRoot, spec.points.upper_closure_coupler),
  };

  await Promise.all(Object.entries(groups).map(([role, group]) => {
    const anchorByRole = {
      ground: hip,
      lower_driver: spec.points.lower_drive,
      coupler: spec.points.lower_passive,
      lower_diagonal: spec.points.lower_coupler,
      lower_closure: spec.points.lower_closure_diagonal,
      upper_driver: spec.points.upper_drive,
      upper_closure: spec.points.upper_closure_coupler,
    };
    const meshCorrection = spec.meshCorrections?.[role] || [0, 0, 0];
    const correctedSourceTranslation = spec.sourceTranslation.map(
      (value, axis) => value + meshCorrection[axis],
    );
    const material = role === "lower_closure" ? tpuMaterial : legMaterial;
    return loadMesh(
      spec.meshes[role],
      material,
      correctedSourceTranslation,
      anchorByRole[role],
      group,
    );
  }));

  const footProbe = new THREE.Object3D();
  footProbe.position.copy(v3(spec.points.foot_tip).sub(v3(spec.points.lower_closure_diagonal)));
  groups.lower_closure.add(footProbe);

  const pinNames = [
    "lower_drive",
    "upper_drive",
    "lower_passive",
    "lower_closure_driver",
    "upper_closure_driver",
    "upper_closure_coupler",
    "lower_coupler",
    "lower_closure_diagonal",
  ];
  const pinGeometry = new THREE.CylinderGeometry(0.0032, 0.0032, 0.052, 16);
  const pins = Object.fromEntries(pinNames.map((name) => {
    const pin = new THREE.Mesh(pinGeometry, pinMaterial);
    const point = spec.points[name];
    pin.position.set(point[0] - hip[0], point[1] - hip[1], point[2] - hip[2]);
    pin.visible = false;
    legRoot.add(pin);
    return [name, pin];
  }));

  const annotationDefinitions = {
    hip_origin: { notation: "q1 SHOULDER", active: true },
    upper_drive: { notation: "q2 UPPER", active: true },
    lower_drive: { notation: "q3 LOWER", active: true },
    lower_passive: { notation: "P1 PIN", active: false },
    upper_closure_driver: { notation: "P2 PIN", active: false },
    upper_closure_coupler: { notation: "P3 PIN", active: false },
    lower_closure_driver: { notation: "P4 PIN", active: false },
    lower_coupler: { notation: "P5 PIN", active: false },
    lower_closure_diagonal: { notation: "P6 PIN", active: false },
  };
  const annotations = Object.fromEntries(
    Object.entries(annotationDefinitions).map(([name, definition]) => {
      const point = spec.points[name];
      const position = new THREE.Vector3(
        point[0] - hip[0],
        point[1] - hip[1],
        point[2] - hip[2],
      );
      return [
        name,
        createJointAnnotation(
          legRoot,
          position,
          definition.notation.split(" ")[0],
          definition.active,
          name === "hip_origin"
            ? new THREE.Vector3(1, 0, 0)
            : new THREE.Vector3(0, 1, 0),
        ),
      ];
    }),
  );

  linkageRuntimes[runtimeIndex] = { spec, legRoot, groups, footProbe, pins, annotations };
}

async function buildRobot() {
  bodyReferenceOverlay = createBodyReferenceOverlay();
  const baseGroup = new THREE.Group();
  cadRoot.add(baseGroup);
  await loadMesh("base_link", frameMaterial, [0, 0, 0], assemblyOrigin, baseGroup);
  await Promise.all(legs.map((spec, index) => buildLeg(spec, index)));
  loading.classList.add("hidden");
}

function updateLinkage(runtime, shoulderDeltaDeg, upperDeltaDeg, lowerDeltaDeg) {
  const { spec, legRoot, groups, annotations } = runtime;
  const points = spec.points;
  const hip = points.hip_origin;
  const neutralOffset = spec.neutralLinkageOffsetDeg || { upper: 0, lower: 0 };
  // Firmware theta3 is the coupled parallelogram output. The exported CAD
  // lower-drive axis is opposite to that firmware-positive convention.
  const lowerDelta = THREE.MathUtils.degToRad(
    THREE.MathUtils.clamp(
      -lowerDeltaDeg + neutralOffset.lower,
      -SERVO_TRAVEL_DEG,
      SERVO_TRAVEL_DEG,
    ),
  );
  // Every exported upper-drive revolute axis is -Y, opposite to the
  // mathematical X/Z rotation used by the closure solver.
  const upperDelta = THREE.MathUtils.degToRad(
    THREE.MathUtils.clamp(
      -upperDeltaDeg + neutralOffset.upper,
      -SERVO_TRAVEL_DEG,
      SERVO_TRAVEL_DEG,
    ),
  );
  const linkage = solveLinkagePose(points, upperDelta, lowerDelta);
  const {
    lowerDrive,
    upperDrive,
    lowerPassive,
    lowerClosureDriver,
    upperClosureDriver,
    upperClosureCoupler,
    lowerCoupler,
    lowerClosureDiagonal,
  } = linkage;
  runtime.commandScale = linkage.commandScale;
  runtime.linkage = linkage;

  legRoot.rotation.x = THREE.MathUtils.degToRad(
    -spec.shoulderSign *
      THREE.MathUtils.clamp(shoulderDeltaDeg, -SERVO_TRAVEL_DEG, SERVO_TRAVEL_DEG),
  );
  planarTransform(groups.lower_driver, points.lower_drive, points.lower_closure_diagonal, lowerDrive, lowerClosureDriver, hip);
  planarTransform(groups.upper_driver, points.upper_drive, points.upper_closure_driver, upperDrive, upperClosureDriver, hip);
  planarTransform(groups.coupler, points.lower_passive, points.upper_closure_coupler, lowerPassive, upperClosureCoupler, hip);
  planarTransform(groups.lower_diagonal, points.lower_coupler, points.lower_closure_driver, lowerCoupler, lowerClosureDiagonal, hip);
  planarTransform(groups.lower_closure, points.lower_closure_diagonal, points.lower_closure_driver, lowerClosureDriver, lowerClosureDiagonal, hip);
  planarTransform(groups.upper_closure, points.upper_closure_coupler, points.upper_closure_driver, upperClosureCoupler, upperClosureDriver, hip);

  const pinPositions = {
    lower_drive: lowerDrive,
    upper_drive: upperDrive,
    lower_passive: lowerPassive,
    lower_closure_driver: lowerClosureDiagonal,
    upper_closure_driver: upperClosureDriver,
    upper_closure_coupler: upperClosureCoupler,
    lower_coupler: lowerCoupler,
    lower_closure_diagonal: lowerClosureDriver,
  };
  Object.entries(runtime.pins).forEach(([name, pin]) => {
    const current = pinPositions[name];
    pin.position.x = current.x - hip[0];
    pin.position.z = current.z - hip[2];
  });
  Object.entries(annotations).forEach(([name, annotation]) => {
    if (name === "hip_origin") return;
    const current = pinPositions[name];
    annotation.group.position.x = current.x - hip[0];
    annotation.group.position.z = current.z - hip[2];
  });
}

const channels = Array(16).fill(1500);
const RIDE_HEIGHT_CHANNEL_INDEX = 2;
channels[4] = 1000;
channels[RIDE_HEIGHT_CHANNEL_INDEX] = 1000;
channels[6] = 1000;
channels[7] = 1000;

const channelDefinitions = [
  { name: "ROLL", switch: false },
  { name: "PITCH", switch: false },
  { name: "HEIGHT / LEFT Y", switch: false, height: true },
  { name: "YAW", switch: false },
  { name: "SA / STAND", switch: true },
  { name: "SB / AUX", switch: true },
  { name: "SC / BALANCE", switch: true },
  { name: "SD / TILT", switch: true },
];
const channelBars = channelDefinitions.map((definition, index) => {
  const element = document.createElement("div");
  element.className = `channel${definition.switch ? " switch" : ""}`;
  element.innerHTML = `
    <div class="channel-heading"><strong>CH${index + 1}</strong><span>${definition.name}</span></div>
    <div class="channel-track"><div class="channel-fill"></div></div>
    <div class="channel-reading"><output>1500</output><span class="channel-position">MID</span></div>
  `;
  document.querySelector("#channel-bars").append(element);
  return element;
});

function updateChannelBars() {
  channelBars.forEach((element, index) => {
    const value = Math.max(1000, Math.min(2000, channels[index]));
    const position = value < 1250 ? "LOW" : value > 1750 ? "HIGH" : "MID";
    const positionLabel = channelDefinitions[index].height
      ? position === "LOW"
        ? "HIGH RIDE"
        : position === "HIGH"
          ? "LOW RIDE"
          : "MID RIDE"
      : position;
    element.style.setProperty("--level", `${(value - 1000) / 10}%`);
    element.classList.toggle("low", position === "LOW");
    element.classList.toggle("mid", position === "MID");
    element.classList.toggle("high", position === "HIGH");
    element.querySelector("output").textContent = Math.round(value);
    element.querySelector(".channel-position").textContent =
      channelDefinitions[index].switch ? positionLabel : "";
  });
}

const socketProtocol = location.protocol === "https:" ? "wss" : "ws";
let socket;
let bridgeInput = { connected: false, channels: null };
let controlClaimUntil = performance.now() + 1000;

function claimControl() {
  controlClaimUntil = performance.now() + 2000;
}

window.addEventListener("pointerdown", claimControl, { capture: true });
window.addEventListener("keydown", claimControl, { capture: true });

function connectControlBridge() {
  socket = new WebSocket(`${socketProtocol}://${location.host}/control`);
  socket.addEventListener("open", () => {
    document.querySelector("#firmware-status").dataset.state = "online";
  });
  socket.addEventListener("close", () => {
    bridgeInput = { connected: false, channels: null };
    document.querySelector("#firmware-status").dataset.state = "offline";
    setTimeout(connectControlBridge, 800);
  });
  socket.addEventListener("message", (event) => {
    try {
      const message = JSON.parse(event.data);
      if ((message.type === "ready" || message.type === "input") && message.input) {
        bridgeInput = message.input;
      }
    } catch {
      // Ignore malformed local bridge packets.
    }
  });
}
connectControlBridge();

function sendChannels() {
  if (socket?.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({
      type: "control",
      channels,
      mode: demoMode ? "demo" : "interactive",
      active: document.visibilityState === "visible",
      claimControl: performance.now() < controlClaimUntil,
      manualOverride:
        manualStandOverride !== null ||
        manualTiltOverride !== null ||
        manualHeightOverride !== null,
      clientInput: clientInputSnapshot,
      physics: physicsState,
    }));
  }
}
setInterval(sendChannels, 25);

const keys = new Set();
window.addEventListener("keydown", (event) => {
  keys.add(event.code);
  if (event.repeat) return;
  if (event.code === "Space") {
    event.preventDefault();
    standRequested = !standRequested;
  }
  if (event.code === "KeyT") tiltRequested = !tiltRequested;
  if (event.code === "KeyR") resetRobot();
});
window.addEventListener("keyup", (event) => keys.delete(event.code));

function resetRobot() {
  physics?.reset();
  robotYaw = 0;
  manualStandOverride = null;
  manualTiltOverride = null;
  manualHeightOverride = null;
  channels[RIDE_HEIGHT_CHANNEL_INDEX] = 1000;
  observedPhysicalStand = null;
  observedPhysicalTilt = null;
  observedPhysicalHeight = null;
}

function resetCameraView() {
  cameraSnap = null;
  controls.enabled = true;
  usePerspectiveCamera();
  cameraTargetOffset.set(0, 0, 0);
  robotCameraAnchor.set(robotWorld.position.x, 0.32, robotWorld.position.z);
  controls.target.copy(robotCameraAnchor);
  perspectiveCamera.position.copy(robotCameraAnchor).add(defaultCameraOffset);
  perspectiveCamera.up.set(0, 1, 0);
  perspectiveCamera.lookAt(controls.target);
  perspectiveCamera.zoom = 1;
  perspectiveCamera.updateProjectionMatrix();
  cameraGizmo.dataset.view = "";
  controls.update();
}

function updateJointOverlay() {
  linkageRuntimes.forEach((runtime) => {
    if (!runtime) return;
    Object.values(runtime.annotations).forEach((annotation) => {
      annotation.group.visible = jointOverlayVisible;
      annotation.group.visible =
        jointOverlayVisible &&
        (selectedJointLeg === "ALL" || runtime.spec.label === selectedJointLeg);
      annotation.marker.material.opacity = jointOverlayOpacity;
      annotation.axis.line.material.opacity = jointOverlayOpacity;
      annotation.axis.cone.material.opacity = jointOverlayOpacity;
      annotation.label.material.opacity = jointOverlayOpacity;
    });
  });
  const button = document.querySelector("#joints-button");
  const control = button.closest(".inspection-control");
  button.classList.toggle("active", jointOverlayVisible);
  button.setAttribute("aria-pressed", String(jointOverlayVisible));
  button.textContent = jointOverlayVisible ? "JOINTS ON" : "JOINTS";
  control.classList.toggle("active", jointOverlayVisible);
  document.querySelector("#joint-legend").hidden = !jointOverlayVisible;
  document.querySelector("#joint-legend-leg").textContent = selectedJointLeg;
  if (bodyReferenceOverlay) {
    bodyReferenceOverlay.group.visible = jointOverlayVisible;
    bodyReferenceOverlay.plane.material.opacity = jointOverlayOpacity * 0.16;
    bodyReferenceOverlay.axes.forEach(({ arrow, label }) => {
      arrow.line.material.opacity = jointOverlayOpacity;
      arrow.cone.material.opacity = jointOverlayOpacity;
      label.material.opacity = jointOverlayOpacity;
    });
  }
}

function updateJointLegendValues() {
  const selectedLeg = legs.find((leg) => leg.label === selectedJointLeg);
  const values = ["#joint-q1-value", "#joint-q2-value", "#joint-q3-value"];
  if (!firmwareState || !selectedLeg) {
    values.forEach((selector) => {
      document.querySelector(selector).textContent = selectedJointLeg === "ALL" ? "MULTI" : "--";
    });
    return;
  }
  const channelNames = ["shoulder", "upper", "lower"];
  channelNames.forEach((name, index) => {
    const channel = selectedLeg.channels[name];
    const absolute = firmwareState.servo_angle_deg[channel];
    const delta = (absolute - neutralServoAngles[channel]) / selectedLeg.directions[name];
    const sign = delta >= 0 ? "+" : "";
    document.querySelector(values[index]).textContent =
      `${sign}${delta.toFixed(1)}° / ${absolute.toFixed(1)}°`;
  });
}

document.querySelector("#stand-button").addEventListener("click", () => {
  const nextStand = !standRequested;
  manualStandOverride = nextStand;
  standRequested = nextStand;
  if (!nextStand) {
    manualTiltOverride = false;
    tiltRequested = false;
  }
});
document.querySelector("#tilt-button").addEventListener("click", () => {
  const nextTilt = !tiltRequested;
  manualTiltOverride = nextTilt;
  tiltRequested = nextTilt;
  if (nextTilt) {
    manualStandOverride = true;
    standRequested = true;
  }
});
document.querySelectorAll("[data-height]").forEach((button) => {
  button.addEventListener("click", () => {
    if (button.disabled) return;
    manualHeightOverride = button.dataset.height;
    channels[RIDE_HEIGHT_CHANNEL_INDEX] = manualHeightOverride === "HIGH"
      ? 1000
      : manualHeightOverride === "LOW"
        ? 2000
        : 1500;
  });
});
document.querySelector("#reset-button").addEventListener("click", resetRobot);
document.querySelector("#reset-view-button").addEventListener("click", resetCameraView);
document.querySelector("#joints-button").addEventListener("click", () => {
  jointOverlayVisible = !jointOverlayVisible;
  updateJointOverlay();
});
document.querySelector("#joint-leg").addEventListener("change", (event) => {
  selectedJointLeg = event.currentTarget.value;
  updateJointOverlay();
  updateJointLegendValues();
});

const gamepadButtonState = new Map();
function pressedOnce(gamepad, index, action) {
  const key = `${gamepad.index}:${index}`;
  const pressed = Boolean(gamepad.buttons[index]?.pressed);
  if (pressed && !gamepadButtonState.get(key)) action();
  gamepadButtonState.set(key, pressed);
}

function deadzone(value, threshold = 0.1) {
  if (Math.abs(value) < threshold) return 0;
  return Math.sign(value) * (Math.abs(value) - threshold) / (1 - threshold);
}

function axisToChannel(axis) {
  return 1500 + Math.round(THREE.MathUtils.clamp(Number(axis) || 0, -1, 1) * 500);
}

function applyPhysicalModeChannels() {
  const physicalStand = channels[4] > 1600;
  const physicalTilt = channels[7] > 1600;
  const physicalHeight = channels[RIDE_HEIGHT_CHANNEL_INDEX] < 1250
    ? "HIGH"
    : channels[RIDE_HEIGHT_CHANNEL_INDEX] > 1750
      ? "LOW"
      : "MEDIUM";

  if (observedPhysicalStand !== null && physicalStand !== observedPhysicalStand) {
    manualStandOverride = null;
  }
  if (observedPhysicalTilt !== null && physicalTilt !== observedPhysicalTilt) {
    manualTiltOverride = null;
  }
  if (observedPhysicalHeight !== null && physicalHeight !== observedPhysicalHeight) {
    manualHeightOverride = null;
  }
  observedPhysicalStand = physicalStand;
  observedPhysicalTilt = physicalTilt;
  observedPhysicalHeight = physicalHeight;

  standRequested = manualStandOverride ?? physicalStand;
  tiltRequested = manualTiltOverride ?? physicalTilt;
  if (!standRequested) tiltRequested = false;

  if (manualStandOverride !== null) channels[4] = standRequested ? 2000 : 1000;
  if (manualTiltOverride !== null || !standRequested) channels[7] = tiltRequested ? 2000 : 1000;
  if (manualHeightOverride !== null) {
    channels[RIDE_HEIGHT_CHANNEL_INDEX] = manualHeightOverride === "HIGH"
      ? 1000
      : manualHeightOverride === "LOW"
        ? 2000
        : 1500;
  }
}

function updateInput() {
  let directRadioChannels = false;
  const gamepad = [...(globalThis.navigator?.getGamepads?.() || [])].find(Boolean);

  if (demoMode) {
    const tiltDiagnostic = demoSelection === "tilt";
    const rollDiagnostic = demoSelection === "roll" || demoSelection === "roll-negative";
    document.querySelector("#gamepad-name").textContent =
      rollDiagnostic ? "ROLL DIAGNOSTIC" :
        tiltDiagnostic ? "TILT DIAGNOSTIC" : "AUTOMATIC DEMO";
    standRequested = true;
    tiltRequested = tiltDiagnostic || rollDiagnostic;
    forwardInput = tiltDiagnostic ? -0.25 : 0;
    turnInput = tiltDiagnostic ? 0.30 : 0;
    rollInput = rollDiagnostic
      ? demoSelection === "roll-negative" ? -1 : 1
      : tiltDiagnostic ? 0.35 : 0;
    clientInputSnapshot = {
      source: "diagnostic",
      name: rollDiagnostic ? "ROLL DIAGNOSTIC" :
        tiltDiagnostic ? "TILT DIAGNOSTIC" : "STAND DIAGNOSTIC",
      axes: [],
    };
  } else if (bridgeInput.connected && Array.isArray(bridgeInput.channels)) {
    directRadioChannels = true;
    for (let index = 0; index < 8; index += 1) {
      channels[index] = bridgeInput.channels[index];
    }
    document.querySelector("#gamepad-name").textContent = `USB / ${bridgeInput.name}`;
    rollInput = deadzone((channels[0] - 1500) / 500);
    forwardInput = deadzone((channels[1] - 1500) / 500);
    turnInput = deadzone((channels[3] - 1500) / 500);
    clientInputSnapshot = {
      source: "direct_hid",
      name: bridgeInput.name,
      axes: Array.isArray(bridgeInput.axes) ? bridgeInput.axes.slice(0, 16) : [],
    };
    applyPhysicalModeChannels();
  } else if (gamepad) {
    const axes = [...gamepad.axes];
    const isEdgeTxRadio =
      axes.length >= 8 && /radiomaster|boxer|edgetx|open.?tx/i.test(gamepad.id);
    document.querySelector("#gamepad-name").textContent =
      `${isEdgeTxRadio ? "BROWSER HID" : "GAMEPAD"} / ${gamepad.id}`;
    clientInputSnapshot = {
      source: isEdgeTxRadio ? "browser_hid" : "gamepad",
      name: gamepad.id,
      axes: axes.slice(0, 16),
    };

    if (isEdgeTxRadio) {
      directRadioChannels = true;
      for (let index = 0; index < 8; index += 1) {
        channels[index] = axisToChannel(axes[index]);
      }
      rollInput = deadzone((channels[0] - 1500) / 500);
      forwardInput = deadzone((channels[1] - 1500) / 500);
      turnInput = deadzone((channels[3] - 1500) / 500);
      applyPhysicalModeChannels();
    } else {
      rollInput = deadzone(axes[0] || 0);
      forwardInput = -deadzone(axes[1] || 0);
      turnInput = deadzone(axes[2] ?? axes[0] ?? 0);
      pressedOnce(gamepad, 0, () => { standRequested = !standRequested; });
      pressedOnce(gamepad, 1, () => { tiltRequested = !tiltRequested; });
      pressedOnce(gamepad, 3, resetRobot);
    }
  } else {
    document.querySelector("#gamepad-name").textContent = "KEYBOARD";
    clientInputSnapshot = { source: "keyboard", name: "KEYBOARD", axes: [] };
    forwardInput = Number(keys.has("KeyW")) - Number(keys.has("KeyS"));
    turnInput = Number(keys.has("KeyD")) - Number(keys.has("KeyA"));
    rollInput = Number(keys.has("KeyE")) - Number(keys.has("KeyQ"));
  }

  if (!directRadioChannels) {
    channels[0] = 1500 + Math.round(rollInput * 500);
    channels[1] = 1500 + Math.round(forwardInput * 500);
    channels[3] = 1500 + Math.round(turnInput * 500);
    channels[4] = standRequested ? 2000 : 1000;
    channels[7] = tiltRequested ? 2000 : 1000;
  }

  document.querySelector("#forward-value").textContent = forwardInput.toFixed(2);
  document.querySelector("#turn-value").textContent = turnInput.toFixed(2);
  document.querySelector("#roll-value").textContent = rollInput.toFixed(2);
  const heightChannelValue = channels[RIDE_HEIGHT_CHANNEL_INDEX];
  document.querySelector("#height-input-value").textContent = heightChannelValue < 1250
    ? "HIGH"
    : heightChannelValue > 1750
      ? "LOW"
      : "MID";
  const manualOverrideActive =
    manualStandOverride !== null ||
    manualTiltOverride !== null ||
    manualHeightOverride !== null;
  document.querySelector("#channel-source").textContent = manualOverrideActive ? "MANUAL" : "ELRS";
  document.querySelector("#channel-source-detail").textContent =
    manualOverrideActive ? "SIM OVERRIDE" : "CRSF INPUT";
  updateChannelBars();
}

async function pollFirmware() {
  try {
    const response = await fetch(`/runtime/state.json?t=${Date.now()}`, { cache: "no-store" });
    if (!response.ok) throw new Error("Firmware state unavailable");
    firmwareState = await response.json();
    if (!visualServoAngles && Array.isArray(firmwareState.servo_angle_deg)) {
      visualServoAngles = [...firmwareState.servo_angle_deg];
    }
    document.querySelector("#firmware-status").dataset.state = "online";
    document.querySelector("#link-status").dataset.state = firmwareState.link_alive ? "online" : "offline";
    const stance = firmwareState.tilt_active
      ? "TILTING"
      : firmwareState.mode === "STOW"
        ? "SITTING"
        : firmwareState.mode === "STAND"
          ? "STANDING"
          : firmwareState.mode;
    const sitting = stance === "SITTING";
    const tilting = stance === "TILTING";
    const standButton = document.querySelector("#stand-button");
    const tiltButton = document.querySelector("#tilt-button");
    document.querySelector("#mode-status").textContent = stance;
    document.querySelector("#height-status").textContent =
      `HEIGHT ${firmwareState.ride_height || "UNKNOWN"}`;
    document.querySelector("#ride-height").textContent = firmwareState.ride_height || "UNKNOWN";
    const requestedStanding = manualStandOverride ?? standRequested;
    const poseTransitionPending =
      Math.abs(firmwareState.pose_z_mm - firmwareState.target_z_mm) > 0.5;
    const transitionPending = requestedStanding === sitting || poseTransitionPending;
    standButton.textContent = transitionPending
      ? requestedStanding
        ? "STANDING..."
        : "SITTING..."
      : sitting
        ? "SITTING"
        : "STANDING";
    standButton.title = transitionPending
      ? "Firmware transition in progress"
      : sitting
        ? "Stand Domino"
        : "Sit Domino";
    standButton.dataset.pending = transitionPending ? "true" : "false";
    standButton.classList.toggle("active", !sitting);
    tiltButton.textContent = tilting ? "TILTING" : "LEVEL";
    tiltButton.title = tilting ? "Return Domino to level" : "Tilt Domino";
    tiltButton.classList.toggle("active", tilting);
    const heightControl = document.querySelector(".height-control");
    heightControl.classList.toggle("disabled", sitting);
    heightControl.setAttribute("aria-disabled", String(sitting));
    document.querySelectorAll("[data-height]").forEach((button) => {
      button.disabled = sitting;
      button.classList.toggle("active", button.dataset.height === firmwareState.ride_height);
    });
    document.querySelector("#frame-count").textContent = firmwareState.accepted_frames;
    document.querySelector("#target-z").textContent = `${firmwareState.target_z_mm.toFixed(1)} mm`;
    document.querySelector("#pose-z").textContent = `${firmwareState.pose_z_mm.toFixed(1)} mm`;
    const activeAngles = legs.flatMap((leg) => [
      firmwareState.servo_angle_deg[leg.channels.shoulder],
      firmwareState.servo_angle_deg[leg.channels.upper],
      firmwareState.servo_angle_deg[leg.channels.lower],
    ]);
    document.querySelector("#servo-range").textContent =
      `${(Math.max(...activeAngles) - Math.min(...activeAngles)).toFixed(1)} deg`;
    updateJointLegendValues();
  } catch {
    document.querySelector("#firmware-status").dataset.state = "offline";
  }
}
// Avoid phase-locking Windows file reads to the firmware's 50 ms atomic state
// publication. A relatively-prime cadence spreads the brief read window out.
setInterval(pollFirmware, 53);

function updateRobot(delta) {
  if (!firmwareState || !linkageRuntimesReady()) return;
  const servoReference = neutralServoAngles;

  if (!visualServoAngles) {
    visualServoAngles = [...firmwareState.servo_angle_deg];
  }
  const blend = 1 - Math.exp(-SERVO_VISUAL_RESPONSE * Math.max(delta, 0));
  visualServoAngles = visualServoAngles.map((current, channel) => {
    const target = Number(firmwareState.servo_angle_deg[channel]);
    return Number.isFinite(target)
      ? THREE.MathUtils.lerp(current, target, blend)
      : current;
  });

  linkageRuntimes.forEach((runtime, index) => {
    const { channels: legChannels, directions } = runtime.spec;
    const shoulderDelta =
      (visualServoAngles[legChannels.shoulder] - servoReference[legChannels.shoulder]) /
      directions.shoulder;
    let upperDelta =
      (visualServoAngles[legChannels.upper] - servoReference[legChannels.upper]) /
      directions.upper;
    let lowerDelta =
      (visualServoAngles[legChannels.lower] - servoReference[legChannels.lower]) /
      directions.lower;

    updateLinkage(runtime, shoulderDelta, upperDelta, lowerDelta);
  });

  canvas.dataset.robotX = robotWorld.position.x.toFixed(4);
  canvas.dataset.robotZ = robotWorld.position.z.toFixed(4);
}

function pointOnBody(group, point, anchor) {
  return group.localToWorld(v3(point).sub(v3(anchor)));
}

function updatePinClosureHealth() {
  if (!linkageRuntimesReady()) return;
  scene.updateMatrixWorld(true);

  const residuals = linkageRuntimes.flatMap((runtime) => {
    const { spec, groups } = runtime;
    const p = spec.points;
    const checks = [
      ["q2", groups.ground, p.hip_origin, groups.upper_driver, p.upper_drive, p.upper_drive],
      ["q3", groups.ground, p.hip_origin, groups.lower_driver, p.lower_drive, p.lower_drive],
      ["P1", groups.lower_driver, p.lower_drive, groups.coupler, p.lower_passive, p.lower_passive],
      ["P2", groups.upper_driver, p.upper_drive, groups.upper_closure, p.upper_closure_coupler, p.upper_closure_driver],
      ["P3", groups.coupler, p.lower_passive, groups.upper_closure, p.upper_closure_coupler, p.upper_closure_coupler],
      ["P4", groups.lower_diagonal, p.lower_coupler, groups.lower_closure, p.lower_closure_diagonal, p.lower_closure_driver],
      ["P5", groups.coupler, p.lower_passive, groups.lower_diagonal, p.lower_coupler, p.lower_coupler],
      ["P6", groups.lower_driver, p.lower_drive, groups.lower_closure, p.lower_closure_diagonal, p.lower_closure_diagonal],
    ];

    return checks.map(([joint, groupA, anchorA, groupB, anchorB, point]) => {
      const a = pointOnBody(groupA, point, anchorA);
      const b = pointOnBody(groupB, point, anchorB);
      return {
        leg: spec.label,
        joint,
        errorMm: a.distanceTo(b) * 1000,
      };
    });
  });

  const worst = residuals.reduce(
    (current, candidate) => candidate.errorMm > current.errorMm ? candidate : current,
    residuals[0],
  );
  const healthy = worst.errorMm <= PIN_TOLERANCE_MM;
  const state = {
    healthy,
    toleranceMm: PIN_TOLERANCE_MM,
    worst,
    residuals,
  };
  globalThis.dominoPinClosureState = state;
  canvas.dataset.pinClosure = JSON.stringify(state);

  const linkageHealth = document.querySelector("#linkage-health");
  const pinError = document.querySelector("#pin-error");
  linkageHealth.textContent = healthy ? "4 CLOSED" : "PIN FAULT";
  linkageHealth.dataset.state = healthy ? "ok" : "fault";
  pinError.textContent = `${worst.errorMm.toFixed(2)} mm ${worst.leg}:${worst.joint}`;
  pinError.dataset.state = healthy ? "ok" : "fault";
  pinError.title = `Tolerance ${PIN_TOLERANCE_MM.toFixed(1)} mm`;

  if (previousPinClosureHealthy !== healthy) {
    const log = healthy ? console.info : console.warn;
    log("[Domino pin closure]", state);
    previousPinClosureHealthy = healthy;
  }
}

function updateFootSymmetryHealth() {
  if (!linkageRuntimesReady()) return;
  scene.updateMatrixWorld(true);

  const cadFeet = Object.fromEntries(linkageRuntimes.map((runtime) => {
    const position = new THREE.Vector3();
    runtime.footProbe.getWorldPosition(position);
    cadRoot.worldToLocal(position);
    return [runtime.spec.label, position];
  }));
  const pairs = [
    { name: "FR-FL", errorMm: Math.abs(cadFeet.FR.z - cadFeet.FL.z) * 1000 },
    { name: "BR-BL", errorMm: Math.abs(cadFeet.BR.z - cadFeet.BL.z) * 1000 },
  ];
  const worst = pairs.reduce(
    (current, candidate) => candidate.errorMm > current.errorMm ? candidate : current,
    pairs[0],
  );
  const intentionalAsymmetry =
    firmwareState?.tilt_active || firmwareState?.mode === "BALANCE";
  const healthy = intentionalAsymmetry || worst.errorMm <= FOOT_SYMMETRY_TOLERANCE_MM;
  const state = {
    healthy,
    intentionalAsymmetry,
    toleranceMm: FOOT_SYMMETRY_TOLERANCE_MM,
    worst,
    pairs,
  };
  globalThis.dominoFootSymmetryState = state;
  canvas.dataset.footSymmetry = JSON.stringify(state);

  const output = document.querySelector("#foot-symmetry");
  output.textContent = intentionalAsymmetry
    ? "ACTIVE POSE"
    : `${worst.errorMm.toFixed(2)} mm ${worst.name}`;
  output.dataset.state = healthy ? "ok" : "fault";
  output.title = intentionalAsymmetry
    ? "Symmetry gate paused for tilt or balance"
    : `CAD-frame tolerance ${FOOT_SYMMETRY_TOLERANCE_MM.toFixed(1)} mm`;

  if (!intentionalAsymmetry && previousFootSymmetryHealthy !== healthy) {
    const log = healthy ? console.info : console.warn;
    log("[Domino foot symmetry]", state);
    previousFootSymmetryHealthy = healthy;
  }
}

function updatePhysics(delta) {
  if (!physics || loadedMeshCount !== expectedMeshCount) return;
  physicsState = physics.update(delta, firmwareState, legs, neutralServoAngles);
  globalThis.dominoPhysicsState = physicsState;
  canvas.dataset.physics = JSON.stringify(physicsState);
  const [x, y, z] = physicsState.basePosition;
  const [qx, qy, qz, qw] = physicsState.baseQuaternion;
  const targetPosition = new THREE.Vector3(x, y, z);
  const physicsQuaternion = new THREE.Quaternion(qx, qy, qz, qw).normalize();
  // Firmware servo targets and body pose are one kinematic snapshot. Driving
  // the visible linkages from that snapshot while driving the CAD body from a
  // slower torque response made the feet lift during Q/E roll transitions.
  // Keep the rendered CAD on the firmware clock; Rapier still supplies world
  // translation, contacts, failure detection, and the underlying dynamics.
  const commandedTiltQuaternion = firmwareState?.mode === "TILT"
    ? targetBodyQuaternion(firmwareState)
    : null;
  const targetQuaternion = commandedTiltQuaternion || physicsQuaternion;
  const physicsWasReset = visualPhysicsResetCount !== physicsState.resetCount;
  if (!visualBaseInitialized || physicsWasReset) {
    visualBasePosition.copy(targetPosition);
    visualBaseQuaternion.copy(targetQuaternion);
    visualBaseInitialized = true;
    visualPhysicsResetCount = physicsState.resetCount;
  } else {
    const positionBlend = 1 - Math.exp(-VISUAL_BASE_POSITION_RESPONSE * Math.max(delta, 0));
    const rotationBlend = 1 - Math.exp(-VISUAL_BASE_ROTATION_RESPONSE * Math.max(delta, 0));
    visualBasePosition.lerp(targetPosition, positionBlend);
    if (visualBaseQuaternion.dot(targetQuaternion) < 0) {
      targetQuaternion.set(
        -targetQuaternion.x,
        -targetQuaternion.y,
        -targetQuaternion.z,
        -targetQuaternion.w,
      );
    }
    visualBaseQuaternion.slerp(targetQuaternion, rotationBlend).normalize();
  }
  robotWorld.position.copy(visualBasePosition);
  robotWorld.quaternion.copy(visualBaseQuaternion);
  bodyEuler.setFromQuaternion(robotWorld.quaternion, "YXZ");
  const rollDeg = THREE.MathUtils.radToDeg(bodyEuler.x);
  // Three.js uses world +Z to the viewer's side, while Domino CAD +Y points
  // left, so CAD pitch has the opposite sign of the Three.js Z rotation.
  const pitchDeg = -THREE.MathUtils.radToDeg(bodyEuler.z);
  const yawDeg = THREE.MathUtils.radToDeg(bodyEuler.y);
  const formatSignedAngle = (value) => `${value >= 0 ? "+" : ""}${value.toFixed(2)}°`;

  document.querySelector("#physics-status").dataset.state = "online";
  document.querySelector("#physics-engine").textContent = "RAPIER + CAD";
  document.querySelector("#body-height").textContent = `${(physicsState.bodyHeight * 1000).toFixed(0)} mm`;
  document.querySelector("#foot-contacts").textContent = `${physicsState.contactCount} / 4`;
  document.querySelector("#body-plane-tilt").textContent =
    `${physicsState.baseTiltDegrees.toFixed(2)}°`;
  document.querySelector("#body-roll").textContent = formatSignedAngle(rollDeg);
  document.querySelector("#body-pitch").textContent = formatSignedAngle(pitchDeg);
  document.querySelector("#body-yaw").textContent = formatSignedAngle(yawDeg);
}

function syncVisibleCadToPhysicsFeet(delta) {
  if (!physicsState || !linkageRuntimesReady()) return;
  scene.updateMatrixWorld(true);

  const visualFeet = linkageRuntimes.map((runtime) => {
    runtime.footProbe.getWorldPosition(visualFootPosition);
    return visualFootPosition.clone();
  });
  const contactSamples = visualFeet.flatMap((foot, index) => {
    if (!physicsState.footContacts[index]) return [];
    return [{
      x: foot.x,
      z: foot.z,
      // A contacting TPU foot belongs on the fixed world plane. Following the
      // proxy collider's tiny solver motion made the whole CAD assembly bounce.
      errorY: CAD_FOOT_RADIUS - foot.y,
    }];
  });

  const errorSpread = contactSamples.length > 0
    ? Math.max(...contactSamples.map((sample) => sample.errorY)) -
      Math.min(...contactSamples.map((sample) => sample.errorY))
    : 0;
  const alignmentCoherent =
    contactSamples.length >= 2 && errorSpread <= VISUAL_GROUND_SYNC_MAX_ERROR_SPREAD;

  if (alignmentCoherent) {
    const sortedErrors = contactSamples
      .map((sample) => sample.errorY)
      .sort((a, b) => a - b);
    const middle = Math.floor(sortedErrors.length / 2);
    const medianError = sortedErrors.length % 2 === 0
      ? 0.5 * (sortedErrors[middle - 1] + sortedErrors[middle])
      : sortedErrors[middle];
    const targetAlignmentY = THREE.MathUtils.clamp(
      cadAlignment.position.y + medianError,
      -VISUAL_GROUND_SYNC_MAX_OFFSET,
      VISUAL_GROUND_SYNC_MAX_OFFSET,
    );
    const dampedAlignmentY = THREE.MathUtils.damp(
      cadAlignment.position.y,
      targetAlignmentY,
      VISUAL_GROUND_SYNC_RESPONSE,
      Math.max(delta, 0),
    );
    const maximumStep = VISUAL_GROUND_SYNC_MAX_RATE * Math.max(delta, 0);
    cadAlignment.position.y += THREE.MathUtils.clamp(
      dampedAlignmentY - cadAlignment.position.y,
      -maximumStep,
      maximumStep,
    );
    // robotWorld already carries Rapier's body quaternion. Applying another
    // roll/pitch fit here made a level stand visibly lean as small proxy/CAD
    // foot differences accumulated. Keep only the vertical registration.
    cadAlignment.rotation.x = 0;
    cadAlignment.rotation.z = 0;
    scene.updateMatrixWorld(true);
  }

  // The serial Rapier proxy can briefly lose a diagonal contact while the
  // real closed linkage remains geometrically planted. Project the complete
  // visible CAD foot set above the world plane after smoothing so no mesh can
  // ever be rendered through the floor. Downward correction remains damped;
  // only penetration is resolved immediately.
  const floorProbeFeet = linkageRuntimes.map((runtime) => {
    runtime.footProbe.getWorldPosition(visualFootPosition);
    return visualFootPosition.y - CAD_FOOT_RADIUS;
  });
  const minimumFootSurface = Math.min(...floorProbeFeet);
  if (minimumFootSurface < VISUAL_FLOOR_CLEARANCE) {
    cadAlignment.position.y = THREE.MathUtils.clamp(
      cadAlignment.position.y + VISUAL_FLOOR_CLEARANCE - minimumFootSurface,
      -VISUAL_GROUND_SYNC_MAX_OFFSET,
      VISUAL_GROUND_SYNC_MAX_OFFSET,
    );
    scene.updateMatrixWorld(true);
  }

  const syncedFeet = linkageRuntimes.map((runtime) => {
    runtime.footProbe.getWorldPosition(visualFootPosition);
    return [visualFootPosition.x, visualFootPosition.y, visualFootPosition.z];
  });
  const visualState = {
    alignment: {
      y: cadAlignment.position.y,
      roll: cadAlignment.rotation.x,
      pitch: cadAlignment.rotation.z,
      coherent: alignmentCoherent,
      errorSpread,
    },
    footCenters: syncedFeet,
    footSurfaceHeights: syncedFeet.map((position) => position[1] - CAD_FOOT_RADIUS),
    contactErrors: syncedFeet.map((position, index) => {
      return physicsState.footContacts[index]
        ? position[1] - CAD_FOOT_RADIUS
        : 0;
    }),
  };
  visualState.groundedCount = visualState.contactErrors.filter(
    (error, index) => physicsState.footContacts[index] && Math.abs(error) <= 0.004,
  ).length;
  globalThis.dominoVisualState = visualState;
  canvas.dataset.visual = JSON.stringify(visualState);
  document.querySelector("#foot-contacts").textContent =
    `${Math.min(physicsState.contactCount, visualState.groundedCount)} / 4`;
}

let lastFrame = performance.now();
let fpsSampleTime = lastFrame;
let fpsFrames = 0;
function animate(now) {
  const delta = Math.min(0.05, (now - lastFrame) / 1000);
  lastFrame = now;
  updateInput();
  updateRobot(delta);
  updatePinClosureHealth();
  updateFootSymmetryHealth();
  updatePhysics(delta);
  syncVisibleCadToPhysicsFeet(delta);
  const previousTarget = controls.target.clone();
  robotCameraAnchor.set(robotWorld.position.x, 0.32, robotWorld.position.z);
  if (!middleButtonPanning) {
    controls.target.lerp(robotCameraAnchor.clone().add(cameraTargetOffset), 0.08);
    camera.position.add(controls.target.clone().sub(previousTarget));
  }
  if (orthographicAxisDirection && inspectionGrid.visible) {
    inspectionGrid.position.copy(controls.target)
      .addScaledVector(orthographicAxisDirection, -0.55);
  }
  if (!updateCameraSnap(now)) controls.update();
  const viewingFromBelow = camera.position.y < 0;
  ground.visible = !viewingFromBelow;
  grid.visible = !viewingFromBelow;
  updateCameraGizmo();
  renderer.render(scene, camera);

  fpsFrames += 1;
  if (now - fpsSampleTime > 500) {
    document.querySelector("#fps-status").textContent =
      `${Math.round((fpsFrames * 1000) / (now - fpsSampleTime))} FPS`;
    fpsFrames = 0;
    fpsSampleTime = now;
  }
  requestAnimationFrame(animate);
}

function resize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  renderer.setSize(width, height, false);
  perspectiveCamera.aspect = width / height;
  perspectiveCamera.updateProjectionMatrix();
  updateOrthographicProjection();
  const pixelRatio = Math.min(window.devicePixelRatio, 2);
  cameraGizmoCanvas.width = CAMERA_GIZMO_SIZE * pixelRatio;
  cameraGizmoCanvas.height = CAMERA_GIZMO_SIZE * pixelRatio;
  cameraGizmoContext.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
}

window.addEventListener("resize", resize);
resize();
buildRobot().catch((error) => {
  loadingDetail.textContent = error.message;
  console.error(error);
});
createDominoPhysics()
  .then((runtime) => {
    physics = runtime;
    document.querySelector("#physics-status").dataset.state = "online";
  })
  .catch((error) => {
    document.querySelector("#physics-status").dataset.state = "offline";
    document.querySelector("#physics-status").textContent = "NO PHYSICS";
    document.querySelector("#physics-engine").textContent = "FAILED";
    console.error("Domino physics failed to initialize", error);
  });
pollFirmware();
requestAnimationFrame(animate);
