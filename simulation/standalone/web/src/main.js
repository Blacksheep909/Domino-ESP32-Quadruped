import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";

import {
  assemblyOrigin,
  dominoMassModel,
  expectedMeshCount,
  legs,
  standServoReference,
} from "./domino-config.js";
import { point2, solveLinkagePose } from "./linkage.js";
import {
  createGaitLab,
  defaultGaitLabSettings,
  GAIT_LAB_STORAGE_KEY,
  gaitLabControls,
  gaitLabPresets,
  sanitizeGaitLabSettings,
} from "./gait-lab.js";
import {
  acceptRobotGaitProfile,
  createLiveGaitCommand,
  createLiveGaitProfile,
  createLiveGaitState,
  liveGaitCanApply,
  liveGaitDiff,
  liveGaitProfileJson,
  liveGaitRiskAssessment,
  LIVE_GAIT_LIBRARY_KEY,
  parseLiveGaitProfileJson,
  readLiveGaitLibrary,
  replaceLiveGaitDraft,
  selectLiveGaitPreset,
  updateLiveGaitDraft,
} from "./live-gait-state.js";
import { contactSurfaceError, createDominoPhysics } from "./physics.js";
import { environmentBallSpecs, logSpecs, terrainSpecs } from "./course-config.js";
import { createVoronoiTerrain } from "./voronoi-terrain.js";
import { initializeFirmwareWorkspace } from "./firmware-ui.js";
import {
  createApplicationState,
  selectExperience,
  selectWorkspace,
  simulationCanOwnControl,
  WORKSPACE_REAL_ROBOT,
  WORKSPACE_SIMULATION,
} from "./app-state.js";
import {
  acceptHeartbeatAcknowledgement,
  createHeartbeatMessage,
  createHeartbeatState,
  HEARTBEAT_INTERVAL_MS,
  heartbeatStatus,
  markHeartbeatSocketClosed,
  markHeartbeatSocketOpen,
  packetAgeMs,
} from "./heartbeat-state.js";
import {
  acceptLiveTelemetryPacket,
  createLiveTelemetryState,
  liveComparisonSnapshot,
} from "./live-telemetry-state.js";
import {
  acceptLiveControllerTelemetry,
  createLiveControllerState,
  liveControllerDiagnosticExport,
  liveControllerSnapshot,
} from "./live-controller-state.js";
import {
  acceptLiveAdapterAnnouncement,
  acceptLiveConnectionAcknowledgement,
  createLiveConnectionCommand,
  createLiveConnectionState,
  failLiveConnectionRequest,
  liveConnectionEnvelope,
  liveConnectionIsReady,
  markLiveConnectionPending,
  pruneLiveAdapters,
  removeLiveAdapter,
  selectLiveAdapter,
  setLiveConnectionBridge,
  setLiveConnectionTransport,
  telemetryBelongsToLiveConnection,
  visibleLiveAdapters,
} from "./live-connection-state.js";
import {
  acceptLiveSafetyAcknowledgement,
  acceptLiveSafetyHeartbeatAcknowledgement,
  beginLiveArmHold,
  cancelLiveArmHold,
  createLiveSafetyCommand,
  createLiveSafetyHeartbeat,
  createLiveSafetyState,
  failLiveSafetyRequest,
  liveSafetyCanArm,
  lockLiveSafetyState,
  markLiveSafetyPending,
  setLiveSafetyRobotState,
  tickLiveSafetyWatchdog,
  updateLiveArmHold,
} from "./live-safety-state.js";
import { LIVE_SAFETY_HEARTBEAT_INTERVAL_MS } from "./live-safety-protocol.js";
import {
  acceptLiveManualAuthorityAcknowledgement,
  beginLiveManualDeadman,
  createLiveManualAuthorityCommand,
  createLiveManualControlFrame,
  createLiveManualControlState,
  endLiveManualDeadman,
  failLiveManualRequest,
  liveManualCanRequest,
  markLiveManualPending,
  revokeLiveManualControl,
  setLiveManualSupport,
  tickLiveManualControl,
  updateLiveManualAxes,
} from "./live-manual-control-state.js";
import { LIVE_MANUAL_FRAME_INTERVAL_MS } from "./live-manual-control-protocol.js";
import {
  createLiveDiagnosticsState,
  liveDiagnosticBundle,
  liveDiagnosticsSnapshot,
  observeLiveDiagnosticPacket,
} from "./live-diagnostics-state.js";
import {
  archiveLiveSession,
  createLiveSessionState,
  liveSessionCsv,
  liveSessionSummary,
  recordLiveComparisonSample,
  removeArchivedLiveSession,
  startLiveSession,
  stopLiveSession,
} from "./live-session-state.js";
import {
  calibrationPreviewServoAngles,
  calibrationProfileJson,
  createCalibrationBenchCommand,
  createLiveCalibrationProfile,
  createLiveCalibrationState,
  jogCalibrationJoint,
  LIVE_CALIBRATION_JOINTS,
  LIVE_CALIBRATION_STEPS,
  LIVE_CALIBRATION_STORAGE_KEY,
  parseCalibrationProfileJson,
  selectCalibrationJoint,
  selectCalibrationStep,
  updateCalibrationJoint,
} from "./live-calibration-state.js";
import {
  createLiveViewState,
  LIVE_VIEW_CALIBRATION,
  LIVE_VIEW_COMPARE,
  LIVE_VIEW_DATA,
  LIVE_VIEW_DIAGNOSTICS,
  LIVE_VIEW_GAITS,
  LIVE_VIEW_SESSIONS,
  selectLiveView,
} from "./live-view-state.js";
import "./styles.css";

initializeFirmwareWorkspace();

const canvas = document.querySelector("#scene");
const applicationState = createApplicationState();
const liveTelemetryState = createLiveTelemetryState();
const liveConnectionState = createLiveConnectionState();
const liveSafetyState = createLiveSafetyState();
const liveManualState = createLiveManualControlState();
const liveControllerState = createLiveControllerState();
const liveDiagnosticsState = createLiveDiagnosticsState();
const liveSessionState = createLiveSessionState();
const liveSessionArchive = [];
const liveViewState = createLiveViewState();
const liveGaitState = createLiveGaitState(createLiveGaitProfile(defaultGaitLabSettings, "Balanced"));
const liveGaitPreviewLab = createGaitLab(liveGaitState.draft.settings);
let liveGaitLibrary = {};
try {
  liveGaitLibrary = readLiveGaitLibrary(JSON.parse(localStorage.getItem(LIVE_GAIT_LIBRARY_KEY) || "{}"));
} catch {
  liveGaitLibrary = {};
}
let liveGaitPendingTimeout = null;
let liveGaitPreviewOutput = null;
let storedCalibrationProfile = null;
try {
  const storedCalibrationJson = localStorage.getItem(LIVE_CALIBRATION_STORAGE_KEY);
  if (storedCalibrationJson) storedCalibrationProfile = parseCalibrationProfileJson(storedCalibrationJson);
} catch {
  localStorage.removeItem(LIVE_CALIBRATION_STORAGE_KEY);
}
const liveCalibrationState = createLiveCalibrationState(
  storedCalibrationProfile || createLiveCalibrationProfile(),
);
let calibrationPendingRequestId = "";
let calibrationPendingAction = "";
let calibrationRequestTimeout = null;
let liveConnectionRequestTimeout = null;
let liveSafetyRequestTimeout = null;
let liveManualRequestTimeout = null;
let liveDiagnosticFilter = "all";
const realWorkspace = document.querySelector("#real-workspace");
const workspaceButtons = {
  [WORKSPACE_SIMULATION]: document.querySelector("#workspace-simulation"),
  [WORKSPACE_REAL_ROBOT]: document.querySelector("#workspace-real-robot"),
};

function applyWorkspace(workspace) {
  if (workspace !== WORKSPACE_REAL_ROBOT && liveManualState.authorityToken) {
    releaseLiveManualControl("Leaving LIVE released browser-control authority.");
  }
  selectWorkspace(applicationState, workspace);
  document.body.dataset.workspace = workspace;
  canvas.dataset.workspace = workspace;
  realWorkspace.hidden = workspace !== WORKSPACE_REAL_ROBOT;
  document.querySelector("#workspace-subtitle").textContent = workspace === WORKSPACE_SIMULATION
    ? "VIRTUAL LAB / 3D CODE TESTING"
    : "LIVE / DIGITAL TWIN";
  document.title = workspace === WORKSPACE_SIMULATION
    ? "Domino Virtual Lab"
    : "Domino Live Digital Twin";

  Object.entries(workspaceButtons).forEach(([name, button]) => {
    const active = name === workspace;
    button.setAttribute("aria-selected", String(active));
    button.tabIndex = active ? 0 : -1;
  });

  if (workspace === WORKSPACE_REAL_ROBOT) {
    applyLiveView(liveViewState.selected);
    footTrajectoryGroup.visible = false;
    inspectionGrid.visible = false;
    if (bodyReferenceOverlay) bodyReferenceOverlay.group.visible = false;
    linkageRuntimes.forEach((runtime) => {
      if (!runtime) return;
      Object.values(runtime.annotations).forEach((annotation) => {
        annotation.group.visible = false;
      });
      Object.values(runtime.pins).forEach((pin) => {
        pin.visible = false;
      });
    });
  } else {
    if (measuredRobotWorld) measuredRobotWorld.visible = false;
    updateJointOverlay();
  }
  requestAnimationFrame(resize);
}

function applyLiveView(view) {
  const leavingCalibration = liveViewState.selected === LIVE_VIEW_CALIBRATION && view !== LIVE_VIEW_CALIBRATION;
  const enteringCalibration = liveViewState.selected !== LIVE_VIEW_CALIBRATION && view === LIVE_VIEW_CALIBRATION;
  const leavingGaits = liveViewState.selected === LIVE_VIEW_GAITS && view !== LIVE_VIEW_GAITS;
  const enteringGaits = liveViewState.selected !== LIVE_VIEW_GAITS && view === LIVE_VIEW_GAITS;
  if (!selectLiveView(liveViewState, view)) return false;
  if (leavingCalibration) {
    if (liveCalibrationState.benchModeAcknowledged) sendCalibrationCommand("exit");
    liveCalibrationState.benchModeAcknowledged = false;
    liveCalibrationState.jogOffsetDeg = 0;
    liveExpectedServoAngles = [...standServoReference];
    if (linkageRuntimesReady()) applyServoAnglesToRuntimes(linkageRuntimes, liveExpectedServoAngles);
    resetCameraView();
  }
  if (leavingGaits) {
    liveGaitPreviewLab.reset();
    liveGaitPreviewOutput = null;
    liveExpectedServoAngles = [...standServoReference];
    resetCameraView();
  }
  realWorkspace.dataset.view = liveViewState.selected;
  document.body.dataset.liveView = liveViewState.selected;
  document.querySelectorAll("[data-live-view]").forEach((button) => {
    const active = button.dataset.liveView === liveViewState.selected;
    button.setAttribute("aria-selected", String(active));
    button.tabIndex = active ? 0 : -1;
  });
  document.querySelector("#live-view-compare").hidden = liveViewState.selected !== LIVE_VIEW_COMPARE;
  document.querySelector("#live-view-data").hidden = liveViewState.selected !== LIVE_VIEW_DATA;
  document.querySelector("#live-view-calibration").hidden = liveViewState.selected !== LIVE_VIEW_CALIBRATION;
  document.querySelector("#live-view-gaits").hidden = liveViewState.selected !== LIVE_VIEW_GAITS;
  document.querySelector("#live-view-diagnostics").hidden = liveViewState.selected !== LIVE_VIEW_DIAGNOSTICS;
  document.querySelector("#live-view-sessions").hidden = liveViewState.selected !== LIVE_VIEW_SESSIONS;
  if (liveViewState.selected === LIVE_VIEW_DATA) requestAnimationFrame(renderLiveComparisonChart);
  if (liveViewState.selected === LIVE_VIEW_CALIBRATION) renderLiveCalibrationUi();
  if (liveViewState.selected === LIVE_VIEW_GAITS) renderLiveGaitUi();
  if (liveViewState.selected === LIVE_VIEW_DIAGNOSTICS) updateLiveComparisonUi();
  if (liveViewState.selected === LIVE_VIEW_SESSIONS) renderLiveSessionArchive();
  requestAnimationFrame(() => {
    resize();
    if (enteringCalibration || enteringGaits) {
      resetCameraView();
      perspectiveCamera.position.copy(robotCameraAnchor).addScaledVector(
        defaultCameraOffset,
        enteringCalibration ? 0.58 : 0.72,
      );
      perspectiveCamera.lookAt(controls.target);
      controls.update();
    }
  });
  return true;
}

Object.entries(workspaceButtons).forEach(([workspace, button]) => {
  button.addEventListener("click", () => applyWorkspace(workspace));
});

document.body.dataset.experience = applicationState.experience;

document.querySelectorAll("[data-live-view]").forEach((button) => {
  button.addEventListener("click", () => applyLiveView(button.dataset.liveView));
});

document.querySelectorAll("[data-experience]").forEach((button) => {
  button.addEventListener("click", () => {
    selectExperience(applicationState, button.dataset.experience);
    document.body.dataset.experience = applicationState.experience;
    document.querySelectorAll("[data-experience]").forEach((candidate) => {
      const active = candidate.dataset.experience === applicationState.experience;
      candidate.classList.toggle("active", active);
      candidate.setAttribute("aria-pressed", String(active));
    });
  });
});

const liveConnectionDialog = document.querySelector("#live-connection-dialog");
document.querySelector("#live-connection-open").addEventListener("click", () => {
  renderLiveConnectionUi();
  if (!liveConnectionDialog.open) liveConnectionDialog.showModal();
});
document.querySelector("#live-connection-close").addEventListener("click", () => liveConnectionDialog.close());
document.querySelectorAll("[data-live-transport]").forEach((button) => {
  button.addEventListener("click", () => {
    if (!setLiveConnectionTransport(liveConnectionState, button.dataset.liveTransport)) return;
    renderLiveConnectionUi();
  });
});
document.querySelector("#live-connection-discover").addEventListener("click", () => {
  sendLiveConnectionRequest("discover");
});
document.querySelector("#live-adapter-list").addEventListener("click", (event) => {
  const button = event.target.closest("[data-adapter-id]");
  if (!button || !selectLiveAdapter(liveConnectionState, button.dataset.adapterId)) return;
  renderLiveConnectionUi();
});
document.querySelector("#live-connection-connect").addEventListener("click", () => {
  sendLiveConnectionRequest("connect");
});
document.querySelector("#live-connection-disconnect").addEventListener("click", () => {
  sendLiveConnectionRequest("disconnect");
});

const liveArmButton = document.querySelector("#live-safety-arm");
liveArmButton.addEventListener("pointerdown", (event) => {
  if (!beginLiveArmHold(liveSafetyState, liveSafetyContext())) return;
  liveArmButton.setPointerCapture?.(event.pointerId);
  renderLiveSafetyUi();
});
const cancelArmHold = () => {
  if (!liveSafetyState.armHoldStartedAt) return;
  cancelLiveArmHold(liveSafetyState);
  renderLiveSafetyUi();
};
liveArmButton.addEventListener("pointerup", cancelArmHold);
liveArmButton.addEventListener("pointercancel", cancelArmHold);
liveArmButton.addEventListener("lostpointercapture", cancelArmHold);
liveArmButton.addEventListener("keydown", (event) => {
  if ((event.key === " " || event.key === "Enter") && !event.repeat) {
    event.preventDefault();
    beginLiveArmHold(liveSafetyState, liveSafetyContext());
  }
});
liveArmButton.addEventListener("keyup", (event) => {
  if (event.key === " " || event.key === "Enter") cancelArmHold();
});
document.querySelector("#live-safety-disarm").addEventListener("click", () => sendLiveSafetyCommand("disarm"));
document.querySelector("#live-safety-estop").addEventListener("click", () => sendLiveSafetyCommand("estop"));
document.querySelector("#live-safety-reset-estop").addEventListener("click", () => sendLiveSafetyCommand("reset-estop"));

const liveManualDialog = document.querySelector("#live-manual-dialog");
document.querySelector("#live-manual-open").addEventListener("click", () => {
  renderLiveManualUi();
  if (!liveManualDialog.open) liveManualDialog.showModal();
});
document.querySelector("#live-manual-close").addEventListener("click", () => {
  if (liveManualState.deadmanActive) stopLiveManualDeadman();
});
document.querySelector("#live-manual-consent").addEventListener("change", (event) => {
  liveManualState.safetyConfirmed = event.target.checked;
  renderLiveManualUi();
});
document.querySelector("#live-manual-request").addEventListener("click", () => sendLiveManualAuthority("request-authority"));
document.querySelector("#live-manual-release").addEventListener("click", () => releaseLiveManualControl());
document.querySelectorAll("[data-live-manual-mode]").forEach((button) => {
  button.addEventListener("click", () => {
    updateLiveManualAxes(liveManualState, { mode: button.dataset.liveManualMode });
    renderLiveManualUi();
  });
});
document.querySelectorAll("[data-live-manual-axis]").forEach((input) => {
  input.addEventListener("input", () => {
    updateLiveManualAxes(liveManualState, { [input.dataset.liveManualAxis]: Number(input.value) });
    renderLiveManualUi();
  });
});
const liveManualDeadman = document.querySelector("#live-manual-deadman");
liveManualDeadman.addEventListener("pointerdown", (event) => {
  if (!beginLiveManualDeadman(liveManualState, liveManualContext())) return;
  liveManualDeadman.setPointerCapture?.(event.pointerId);
  renderLiveManualUi();
});
liveManualDeadman.addEventListener("pointerup", () => stopLiveManualDeadman());
liveManualDeadman.addEventListener("pointercancel", () => stopLiveManualDeadman());
liveManualDeadman.addEventListener("lostpointercapture", () => stopLiveManualDeadman());
liveManualDeadman.addEventListener("keydown", (event) => {
  if ((event.key === " " || event.key === "Enter") && !event.repeat) {
    event.preventDefault();
    beginLiveManualDeadman(liveManualState, liveManualContext());
    renderLiveManualUi();
  }
});
liveManualDeadman.addEventListener("keyup", (event) => {
  if (event.key === " " || event.key === "Enter") stopLiveManualDeadman();
});
document.querySelector("#live-manual-estop").addEventListener("click", () => {
  releaseLiveManualControl("E-stop requested. Browser control was neutralized.");
  sendLiveSafetyCommand("estop");
});

const demoSelection = new URLSearchParams(window.location.search).get("demo");
const demoMode = ["1", "tilt", "roll", "roll-negative", "gait", "gait-reverse"].includes(demoSelection);
const THEME_STORAGE_KEY = "domino-theme-v2";
const MAX_RENDER_PIXEL_RATIO = 1.5;
let currentTheme = localStorage.getItem(THEME_STORAGE_KEY);
if (currentTheme !== "light" && currentTheme !== "dark") currentTheme = "dark";
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
const FOOT_TRAIL_DURATION_SECONDS = 2;
const FOOT_TRAIL_SAMPLE_INTERVAL_SECONDS = 1 / 60;
const FOOT_TRAIL_MAX_SAMPLES = 128;
const FOOT_TRAIL_SUBDIVISIONS = 3;
const FOOT_TRAIL_MAX_VERTICES =
  ((FOOT_TRAIL_MAX_SAMPLES - 1) * FOOT_TRAIL_SUBDIVISIONS) + 1;
const FLOAT_FALLBACK_BODY_HEIGHT = 0.32;
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
// A capped pixel ratio keeps high-DPI laptops and mobile browsers from
// multiplying the CAD/shadow workload without changing the scene geometry.
renderer.setPixelRatio(Math.min(window.devicePixelRatio, MAX_RENDER_PIXEL_RATIO));
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
const environmentBallMeshes = new Map();
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
  robotCameraAnchor.set(
    robotWorld.position.x,
    floatModeEnabled ? robotWorld.position.y : 0.32,
    robotWorld.position.z,
  );
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
sun.shadow.mapSize.set(1024, 1024);
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

const courseVisuals = new THREE.Group();
courseVisuals.name = "course-visuals";
scene.add(courseVisuals);

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
  const sceneColor = dark ? 0x0b0b0c : 0xdededa;
  scene.background.set(sceneColor);
  scene.fog.color.set(sceneColor);
  groundMaterial.color.set(dark ? 0x252527 : 0xbdbdb8);
  renderer.toneMappingExposure = dark ? 1.08 : 1.05;

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

function addObstacle(
  size,
  position,
  rotationY = 0,
  rotationZ = 0,
  materialOptions = {},
) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(...size),
    new THREE.MeshStandardMaterial({
      color: materialOptions.color ?? 0x59656c,
      roughness: materialOptions.roughness ?? 0.78,
      metalness: materialOptions.metalness ?? 0.02,
    }),
  );
  mesh.position.set(...position);
  mesh.rotation.set(0, rotationY, rotationZ);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  courseVisuals.add(mesh);
}

// Keep the origin clear for resets. The course is arranged in distinct lanes
// around it so each surface or obstacle can be tested deliberately.
const terrainObjects = terrainSpecs;

terrainObjects.forEach((terrain) => {
  if (terrain.kind === "voronoi") return;
  addObstacle(
    terrain.size,
    terrain.position,
    terrain.yaw ?? 0,
    terrain.slope ?? 0,
    terrain,
  );
});

function addVoronoiSurface() {
  const patch = terrainObjects.find((terrain) => terrain.kind === "voronoi");
  if (!patch) return;
  const terrain = createVoronoiTerrain(patch);
  const palette = [0xc5b9a4, 0xada391, 0xd0c4ae, 0xb7ac99, 0xc0b39d];
  addObstacle(
    patch.size,
    patch.position,
    patch.yaw ?? 0,
    0,
    { ...patch, color: 0x8e8679, roughness: 1 },
  );
  terrain.cellRanges.forEach(({ cellIndex, indexStart, indexCount }) => {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(terrain.vertices, 3));
    geometry.setIndex(new THREE.BufferAttribute(terrain.indices.slice(indexStart, indexStart + indexCount), 1));
    geometry.computeVertexNormals();
    const cell = new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({
        color: palette[cellIndex % palette.length],
        roughness: 1,
        metalness: 0,
        side: THREE.DoubleSide,
      }),
    );
    cell.position.set(patch.position[0], 0, patch.position[2]);
    cell.rotation.y = patch.yaw ?? 0;
    cell.castShadow = true;
    cell.receiveShadow = true;
    courseVisuals.add(cell);
  });
}

addVoronoiSurface();

function addLog(spec) {
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(spec.radius, spec.radius, spec.length, 14),
    new THREE.MeshStandardMaterial({ color: 0x694a34, roughness: 0.96, metalness: 0 }),
  );
  mesh.position.set(...spec.position);
  mesh.quaternion.setFromEuler(
    spec.axis === "x"
      ? new THREE.Euler(0, spec.yaw ?? 0, Math.PI / 2)
      : new THREE.Euler(Math.PI / 2, spec.yaw ?? 0, 0),
  );
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  courseVisuals.add(mesh);
}

logSpecs.forEach(addLog);

environmentBallSpecs.forEach((spec) => {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(spec.radius, 20, 12),
    new THREE.MeshStandardMaterial({
      color: spec.color,
      roughness: 0.42,
      metalness: 0.08,
    }),
  );
  mesh.position.set(...spec.position);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  environmentBallMeshes.set(spec.id, mesh);
  courseVisuals.add(mesh);
});

const frameMaterial = new THREE.MeshStandardMaterial({
  color: 0x20282d,
  roughness: 0.42,
  metalness: 0.3,
});
const legMaterial = new THREE.MeshStandardMaterial({
  color: 0x303a40,
  roughness: 0.55,
  metalness: 0.16,
});
const drivenLinkMaterial = new THREE.MeshStandardMaterial({
  color: 0x414c52,
  roughness: 0.48,
  metalness: 0.22,
});
const passiveLinkMaterial = new THREE.MeshStandardMaterial({
  color: 0x525d62,
  roughness: 0.58,
  metalness: 0.1,
});
const tpuMaterial = new THREE.MeshStandardMaterial({
  color: 0x090d0f,
  roughness: 0.96,
  metalness: 0.0,
});
const legMaterialByRole = {
  ground: legMaterial,
  lower_driver: drivenLinkMaterial,
  upper_driver: drivenLinkMaterial,
  coupler: passiveLinkMaterial,
  lower_diagonal: passiveLinkMaterial,
  upper_closure: passiveLinkMaterial,
  lower_closure: tpuMaterial,
};
const pinMaterial = new THREE.MeshStandardMaterial({
  color: 0x818b90,
  roughness: 0.32,
  metalness: 0.78,
  transparent: true,
});
const ACTIVE_JOINT_COLOR = 0xf4f4f5;
const PASSIVE_JOINT_COLOR = 0x8b8b92;
let jointOverlayVisible = false;
let jointOverlayOpacity = 0.78;
let selectedJointLeg = "FR";
let selectedDriveJoint = "upper";
let bodyReferenceOverlay = null;
let floatModeEnabled = false;
const floatAnchorPosition = new THREE.Vector3(0, FLOAT_FALLBACK_BODY_HEIGHT, 0);
const floatAnchorQuaternion = new THREE.Quaternion();
const cameraRecenterDelta = new THREE.Vector3();
const footTrajectoryGroup = new THREE.Group();
footTrajectoryGroup.name = "foot-trajectories";
footTrajectoryGroup.visible = false;
scene.add(footTrajectoryGroup);
const footTrajectories = new Map();
let footTrailSampleElapsed = FOOT_TRAIL_SAMPLE_INTERVAL_SECONDS;
const LEG_COMMAND_INDEX_BY_LABEL = { FR: 1, FL: 0, BL: 2, BR: 3 };
const ACTIVE_ANNOTATION_BY_CHANNEL = {
  shoulder: "hip_origin",
  upper: "upper_drive",
  lower: "lower_drive",
};
const DRIVE_META = {
  shoulder: { label: "q1", title: "SHOULDER", axis: "X" },
  upper: { label: "q2", title: "UPPER", axis: "Y" },
  lower: { label: "q3", title: "LOWER", axis: "Y" },
};

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
let measuredRobotWorld = null;
let measuredLinkageRuntimes = [];
let liveExpectedServoAngles = null;
let liveMeasuredServoAngles = null;
let loadedMeshCount = 0;
let firmwareState = null;
let effectiveFirmwareState = null;
let visualServoAngles = null;
const neutralServoAngles = [...standServoReference];
const SERVO_VISUAL_RESPONSE = 24;
const RIDE_HEIGHT_MIN_MM = 220;
const RIDE_HEIGHT_MAX_MM = 280;
let standRequested = false;
let tiltRequested = false;
let walkModeRequested = 0;
let manualStandOverride = null;
let manualTiltOverride = null;
let manualWalkModeOverride = null;
let observedPhysicalStand = null;
let observedPhysicalTilt = null;
let observedPhysicalWalkMode = null;
let forwardInput = 0;
let turnInput = 0;
let rollInput = 0;
let robotYaw = 0;
let clientInputSnapshot = { source: "keyboard", name: "KEYBOARD", axes: [] };
let physics = null;
const walkModeFromChannel = (value) => value < 1250 ? 0 : value > 1750 ? 2 : 1;
const channelFromWalkMode = (mode) => [1000, 1500, 2000][THREE.MathUtils.clamp(mode, 0, 2)];
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

function createFootGlowTexture() {
  const textureCanvas = document.createElement("canvas");
  textureCanvas.width = 64;
  textureCanvas.height = 64;
  const context = textureCanvas.getContext("2d");
  const gradient = context.createRadialGradient(32, 32, 2, 32, 32, 30);
  gradient.addColorStop(0, "rgba(255,255,255,1)");
  gradient.addColorStop(0.28, "rgba(255,255,255,0.88)");
  gradient.addColorStop(1, "rgba(255,255,255,0)");
  context.fillStyle = gradient;
  context.fillRect(0, 0, 64, 64);
  const texture = new THREE.CanvasTexture(textureCanvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

const footGlowTexture = createFootGlowTexture();

function ensureFootTrajectory(runtime) {
  const label = runtime.spec.label;
  if (footTrajectories.has(label)) return footTrajectories.get(label);
  const positions = new Float32Array(FOOT_TRAIL_MAX_VERTICES * 3);
  const colors = new Float32Array(FOOT_TRAIL_MAX_VERTICES * 3);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  geometry.setDrawRange(0, 0);
  const line = new THREE.Line(
    geometry,
    new THREE.LineBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.92,
      blending: THREE.AdditiveBlending,
      depthTest: false,
      depthWrite: false,
    }),
  );
  line.frustumCulled = false;
  line.renderOrder = 28;
  const marker = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: footGlowTexture,
      color: 0xffffff,
      transparent: true,
      opacity: 0.95,
      blending: THREE.AdditiveBlending,
      depthTest: false,
      depthWrite: false,
    }),
  );
  marker.scale.setScalar(0.035);
  marker.renderOrder = 29;
  footTrajectoryGroup.add(line, marker);
  const trajectory = {
    label,
    samples: [],
    line,
    marker,
    positions,
    colors,
    curve: new THREE.CatmullRomCurve3([], false, "centripetal"),
    smoothPoint: new THREE.Vector3(),
  };
  footTrajectories.set(label, trajectory);
  return trajectory;
}

function clearFootTrajectories() {
  footTrajectories.forEach((trajectory) => {
    trajectory.samples.length = 0;
    trajectory.line.geometry.setDrawRange(0, 0);
  });
  footTrailSampleElapsed = FOOT_TRAIL_SAMPLE_INTERVAL_SECONDS;
}

function updateFootTrajectories(deltaSeconds) {
  footTrajectoryGroup.visible = jointOverlayVisible;
  if (!jointOverlayVisible || !linkageRuntimesReady()) return;
  scene.updateMatrixWorld(true);
  const nowSeconds = performance.now() / 1000;
  footTrailSampleElapsed += Math.max(0, deltaSeconds);
  const shouldSample = footTrailSampleElapsed >= FOOT_TRAIL_SAMPLE_INTERVAL_SECONDS;
  if (shouldSample) footTrailSampleElapsed %= FOOT_TRAIL_SAMPLE_INTERVAL_SECONDS;

  linkageRuntimes.forEach((runtime) => {
    if (!runtime) return;
    const trajectory = ensureFootTrajectory(runtime);
    const selected = selectedJointLeg === "ALL" || selectedJointLeg === runtime.spec.label;
    trajectory.line.visible = selected;
    trajectory.marker.visible = selected;
    runtime.footProbe.getWorldPosition(visualFootPosition);
    trajectory.marker.position.copy(visualFootPosition);
    if (shouldSample) {
      const previousSample = trajectory.samples.at(-1);
      const previous = previousSample?.position;
      if (previous && previous.distanceTo(visualFootPosition) > 0.25) {
        trajectory.samples.length = 0;
      }
      if (previousSample && previous.distanceToSquared(visualFootPosition) < 4e-8) {
        previousSample.time = nowSeconds;
        previous.copy(visualFootPosition);
      } else {
        trajectory.samples.push({
          time: nowSeconds,
          position: visualFootPosition.clone(),
        });
      }
    }
    trajectory.samples = trajectory.samples
      .filter((sample) => nowSeconds - sample.time <= FOOT_TRAIL_DURATION_SECONDS)
      .slice(-FOOT_TRAIL_MAX_SAMPLES);

    const sampleCount = trajectory.samples.length;
    const vertexCount = sampleCount < 2
      ? sampleCount
      : Math.min(
          FOOT_TRAIL_MAX_VERTICES,
          ((sampleCount - 1) * FOOT_TRAIL_SUBDIVISIONS) + 1,
        );
    trajectory.curve.points.length = sampleCount;
    trajectory.samples.forEach((sample, index) => {
      trajectory.curve.points[index] = sample.position;
    });
    for (let index = 0; index < vertexCount; index += 1) {
      const curveFraction = vertexCount > 1 ? index / (vertexCount - 1) : 0;
      const samplePosition = curveFraction * Math.max(0, sampleCount - 1);
      const lowerSampleIndex = Math.min(sampleCount - 1, Math.floor(samplePosition));
      const upperSampleIndex = Math.min(sampleCount - 1, lowerSampleIndex + 1);
      const sampleBlend = samplePosition - lowerSampleIndex;
      const interpolatedTime = THREE.MathUtils.lerp(
        trajectory.samples[lowerSampleIndex].time,
        trajectory.samples[upperSampleIndex].time,
        sampleBlend,
      );
      const point = sampleCount > 2
        ? trajectory.curve.getPoint(curveFraction, trajectory.smoothPoint)
        : trajectory.smoothPoint.copy(
            trajectory.samples[lowerSampleIndex].position,
          ).lerp(
            trajectory.samples[upperSampleIndex].position,
            sampleBlend,
          );
      const ageFraction = THREE.MathUtils.clamp(
        1 - (nowSeconds - interpolatedTime) / FOOT_TRAIL_DURATION_SECONDS,
        0,
        1,
      );
      const brightness = ageFraction ** 1.8;
      trajectory.positions[(index * 3) + 0] = point.x;
      trajectory.positions[(index * 3) + 1] = point.y;
      trajectory.positions[(index * 3) + 2] = point.z;
      trajectory.colors[(index * 3) + 0] = brightness;
      trajectory.colors[(index * 3) + 1] = brightness;
      trajectory.colors[(index * 3) + 2] = brightness;
    }
    trajectory.line.geometry.attributes.position.needsUpdate = true;
    trajectory.line.geometry.attributes.color.needsUpdate = true;
    trajectory.line.geometry.setDrawRange(0, vertexCount);
  });
}

const GAIT_PROFILE_STORAGE_KEY = "domino-gait-profiles-v1";
const GAIT_PANEL_LAYOUT_KEY = "domino-gait-panel-layout-v3";
const INSPECT_PANEL_LAYOUT_KEY = "domino-inspect-panel-layout-v2";
const PANEL_VIEWPORT_MARGIN = 10;
let gaitPanelController = null;
let inspectorPanelController = null;

function readStoredObject(storageKey, fallback) {
  try {
    const value = JSON.parse(localStorage.getItem(storageKey) || "null");
    return value && typeof value === "object" ? value : fallback;
  } catch {
    return fallback;
  }
}

function panelViewportIsStable() {
  return !document.hidden && window.innerWidth >= 320 && window.innerHeight >= 240;
}

function clampPanelToViewport(panel) {
  if (!panelViewportIsStable()) return false;
  const bounds = panel.getBoundingClientRect();
  const maximumWidth = Math.max(240, window.innerWidth - PANEL_VIEWPORT_MARGIN * 2);
  const maximumHeight = Math.max(180, window.innerHeight - PANEL_VIEWPORT_MARGIN * 2);
  const width = Math.min(bounds.width, maximumWidth);
  const height = Math.min(bounds.height, maximumHeight);
  const left = THREE.MathUtils.clamp(
    bounds.left,
    PANEL_VIEWPORT_MARGIN,
    Math.max(PANEL_VIEWPORT_MARGIN, window.innerWidth - width - PANEL_VIEWPORT_MARGIN),
  );
  const top = THREE.MathUtils.clamp(
    bounds.top,
    PANEL_VIEWPORT_MARGIN,
    Math.max(PANEL_VIEWPORT_MARGIN, window.innerHeight - height - PANEL_VIEWPORT_MARGIN),
  );
  panel.style.width = `${Math.round(width)}px`;
  panel.style.height = `${Math.round(height)}px`;
  panel.style.left = `${Math.round(left)}px`;
  panel.style.top = `${Math.round(top)}px`;
  panel.style.right = "auto";
  panel.style.bottom = "auto";
  return true;
}

function createFloatingPanelController({
  panel,
  handle,
  storageKey,
  activeClass = "floating-panel-active",
  clearWhenInactive = false,
}) {
  let active = false;
  let customized = false;
  let dragging = false;
  let dragStart = null;
  let persistenceTimer = null;
  let savedLayout = readStoredObject(storageKey, null);

  function persist() {
    if (!active || !customized || !panelViewportIsStable()) return;
    if (persistenceTimer !== null) clearTimeout(persistenceTimer);
    persistenceTimer = setTimeout(() => {
      persistenceTimer = null;
      if (!active || !customized || !panelViewportIsStable()) return;
      const bounds = panel.getBoundingClientRect();
      savedLayout = {
        left: Math.round(bounds.left),
        top: Math.round(bounds.top),
        width: Math.round(bounds.width),
        height: Math.round(bounds.height),
      };
      try {
        localStorage.setItem(storageKey, JSON.stringify(savedLayout));
      } catch {
        console.warn(`Panel layout could not be persisted: ${storageKey}`);
      }
    }, 100);
  }

  function applyLayout(layout) {
    if (!layout) return false;
    for (const key of ["left", "top", "width", "height"]) {
      if (!Number.isFinite(Number(layout[key]))) return false;
    }
    panel.style.left = `${layout.left}px`;
    panel.style.top = `${layout.top}px`;
    panel.style.width = `${layout.width}px`;
    panel.style.height = `${layout.height}px`;
    panel.style.right = "auto";
    panel.style.bottom = "auto";
    customized = true;
    clampPanelToViewport(panel);
    return true;
  }

  handle.addEventListener("pointerdown", (event) => {
    if (!active || event.button !== 0) return;
    if (event.target.closest("button, input, select, label")) return;
    if (panel.id === "gait-lab-panel") closeGaitSettingInfo();
    const bounds = panel.getBoundingClientRect();
    customized = true;
    dragging = true;
    dragStart = {
      pointerX: event.clientX,
      pointerY: event.clientY,
      left: bounds.left,
      top: bounds.top,
    };
    panel.style.width = `${Math.round(bounds.width)}px`;
    panel.style.height = `${Math.round(bounds.height)}px`;
    panel.style.left = `${Math.round(bounds.left)}px`;
    panel.style.top = `${Math.round(bounds.top)}px`;
    panel.style.right = "auto";
    panel.style.bottom = "auto";
    handle.setPointerCapture?.(event.pointerId);
    event.preventDefault();
  });

  panel.addEventListener("pointerdown", (event) => {
    if (!active || event.button !== 0 || event.target === handle) return;
    const bounds = panel.getBoundingClientRect();
    const onResizeGrip =
      bounds.right - event.clientX <= 18 &&
      bounds.bottom - event.clientY <= 18;
    if (!onResizeGrip) return;
    customized = true;
    panel.style.left = `${Math.round(bounds.left)}px`;
    panel.style.top = `${Math.round(bounds.top)}px`;
    panel.style.right = "auto";
    panel.style.bottom = "auto";
  }, { capture: true });

  handle.addEventListener("pointermove", (event) => {
    if (!dragging || !dragStart) return;
    const width = panel.getBoundingClientRect().width;
    const height = panel.getBoundingClientRect().height;
    const left = THREE.MathUtils.clamp(
      dragStart.left + event.clientX - dragStart.pointerX,
      PANEL_VIEWPORT_MARGIN,
      Math.max(PANEL_VIEWPORT_MARGIN, window.innerWidth - width - PANEL_VIEWPORT_MARGIN),
    );
    const top = THREE.MathUtils.clamp(
      dragStart.top + event.clientY - dragStart.pointerY,
      PANEL_VIEWPORT_MARGIN,
      Math.max(PANEL_VIEWPORT_MARGIN, window.innerHeight - height - PANEL_VIEWPORT_MARGIN),
    );
    panel.style.left = `${Math.round(left)}px`;
    panel.style.top = `${Math.round(top)}px`;
  });

  function finishDrag(event) {
    if (!dragging) return;
    dragging = false;
    dragStart = null;
    if (event?.pointerId !== undefined && handle.hasPointerCapture?.(event.pointerId)) {
      handle.releasePointerCapture(event.pointerId);
    }
    persist();
  }
  handle.addEventListener("pointerup", finishDrag);
  handle.addEventListener("pointercancel", finishDrag);

  if (typeof ResizeObserver === "function") {
    new ResizeObserver(() => {
      if (!active || dragging || !customized || panel.hidden || !panelViewportIsStable()) return;
      if (clampPanelToViewport(panel)) persist();
    }).observe(panel);
  }

  return {
    setActive(nextActive) {
      active = Boolean(nextActive);
      panel.classList.toggle(activeClass, active);
      if (active) {
        if (!customized) applyLayout(savedLayout);
      } else if (clearWhenInactive) {
        panel.style.removeProperty("left");
        panel.style.removeProperty("top");
        panel.style.removeProperty("width");
        panel.style.removeProperty("height");
        panel.style.removeProperty("right");
        panel.style.removeProperty("bottom");
        customized = false;
      }
    },
    clamp() {
      if (active && customized && !panel.hidden && panelViewportIsStable()) {
        clampPanelToViewport(panel);
      }
    },
    isCustomized: () => customized,
  };
}

function loadGaitLabSettings() {
  try {
    return sanitizeGaitLabSettings(JSON.parse(localStorage.getItem(GAIT_LAB_STORAGE_KEY) || "{}"));
  } catch {
    return { ...defaultGaitLabSettings };
  }
}

function loadGaitProfiles() {
  const storedProfiles = readStoredObject(GAIT_PROFILE_STORAGE_KEY, {});
  return Object.fromEntries(
    Object.entries(storedProfiles)
      .filter(([name, settings]) =>
        typeof name === "string" && name.trim() && settings && typeof settings === "object")
      .slice(0, 40)
      .map(([name, settings]) => [
        name.trim().slice(0, 32),
        sanitizeGaitLabSettings(settings.schemaVersion === 1 && settings.settings ? settings.settings : settings),
      ]),
  );
}

let gaitLabSettings = loadGaitLabSettings();
const gaitLab = createGaitLab(gaitLabSettings);
let gaitLabPersistenceTimer = null;
let gaitProfiles = loadGaitProfiles();

function setGaitProfileStatus(message = "", state = "") {
  const output = document.querySelector("#gait-profile-status");
  output.textContent = message;
  output.dataset.state = state;
}

function persistGaitProfiles() {
  try {
    localStorage.setItem(GAIT_PROFILE_STORAGE_KEY, JSON.stringify(gaitProfiles));
    return true;
  } catch {
    setGaitProfileStatus("PROFILES COULD NOT BE SAVED", "error");
    return false;
  }
}

function syncGaitProfileUi(selectedName = "") {
  const select = document.querySelector("#gait-profile-select");
  const previousSelection = selectedName || select.value;
  select.replaceChildren(new Option("SAVED PROFILES", ""));
  Object.keys(gaitProfiles)
    .sort((a, b) => a.localeCompare(b))
    .forEach((name) => select.add(new Option(name, name)));
  select.value = Object.hasOwn(gaitProfiles, previousSelection) ? previousSelection : "";
  const hasSelection = Boolean(select.value);
  document.querySelector("#gait-profile-load").disabled = !hasSelection;
  document.querySelector("#gait-profile-delete").disabled = !hasSelection;
}

function selectedGaitProfileName() {
  return document.querySelector("#gait-profile-select").value;
}

function formatGaitSetting(control, value) {
  const displayValue = value * (control.scale || 1);
  const suffix = control.unit ? ` ${control.unit}` : "";
  return `${displayValue.toFixed(control.decimals)}${suffix}`;
}

const gaitSettingInfoPopover = document.querySelector("#gait-setting-info");
const gaitSettingInfoButton = document.querySelector("#gait-setting-info-button");

function positionGaitSettingInfo() {
  if (!panelViewportIsStable() || gaitSettingInfoPopover.hidden) return;
  const anchor = gaitSettingInfoButton.getBoundingClientRect();
  const popover = gaitSettingInfoPopover.getBoundingClientRect();
  let left = anchor.left + anchor.width / 2 - popover.width / 2;
  let top = anchor.bottom + 7;
  left = THREE.MathUtils.clamp(
    left,
    PANEL_VIEWPORT_MARGIN,
    Math.max(PANEL_VIEWPORT_MARGIN, window.innerWidth - popover.width - PANEL_VIEWPORT_MARGIN),
  );
  if (top + popover.height > window.innerHeight - PANEL_VIEWPORT_MARGIN) {
    top = anchor.top - popover.height - 7;
  }
  top = THREE.MathUtils.clamp(
    top,
    PANEL_VIEWPORT_MARGIN,
    Math.max(PANEL_VIEWPORT_MARGIN, window.innerHeight - popover.height - PANEL_VIEWPORT_MARGIN),
  );
  gaitSettingInfoPopover.style.left = `${Math.round(left)}px`;
  gaitSettingInfoPopover.style.top = `${Math.round(top)}px`;
}

function closeGaitSettingInfo() {
  gaitSettingInfoButton.classList.remove("active");
  gaitSettingInfoButton.setAttribute("aria-expanded", "false");
  gaitSettingInfoPopover.hidden = true;
}

function toggleGaitSettingInfo() {
  if (!gaitSettingInfoPopover.hidden) {
    closeGaitSettingInfo();
    return;
  }
  gaitSettingInfoButton.classList.add("active");
  gaitSettingInfoButton.setAttribute("aria-expanded", "true");
  gaitSettingInfoPopover.hidden = false;
  positionGaitSettingInfo();
}

gaitLabControls.forEach((control) => {
  const term = document.createElement("dt");
  const description = document.createElement("dd");
  term.textContent = control.label;
  description.textContent = control.description;
  document.querySelector("#gait-setting-info-list").append(term, description);
});

gaitSettingInfoButton.addEventListener("click", toggleGaitSettingInfo);
document.querySelector("#gait-setting-info-close").addEventListener("click", closeGaitSettingInfo);
document.addEventListener("pointerdown", (event) => {
  if (gaitSettingInfoPopover.hidden) return;
  if (gaitSettingInfoPopover.contains(event.target)) return;
  if (gaitSettingInfoButton.contains(event.target)) return;
  closeGaitSettingInfo();
});
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape") closeGaitSettingInfo();
});
document.querySelector("#gait-lab-panel").addEventListener("scroll", positionGaitSettingInfo);

function updateGaitPresetButtons() {
  document.querySelectorAll("[data-gait-preset]").forEach((button) => {
    button.classList.toggle("active", button.dataset.gaitPreset === gaitLabSettings.preset);
  });
}

function syncGaitLabUi() {
  document.querySelector("#gait-lab-enabled").checked = gaitLabSettings.enabled;
  document.querySelector("#gait-lab-button").dataset.enabled = String(gaitLabSettings.enabled);
  document.querySelector("#gait-lab-button").textContent = "TUNE";
  gaitLabControls.forEach((control) => {
    const field = document.querySelector(`[data-gait-setting="${control.key}"]`);
    const input = field.querySelector("input");
    input.min = control.min;
    input.max = control.max;
    input.step = control.step;
    input.value = gaitLabSettings[control.key];
    input.disabled = !gaitLabSettings.enabled;
    field.querySelector("output").textContent = formatGaitSetting(control, gaitLabSettings[control.key]);
  });
  updateGaitPresetButtons();
}

function commitGaitLabSettings(nextSettings, preset = "custom") {
  gaitLabSettings = sanitizeGaitLabSettings({ ...nextSettings, preset });
  gaitLab.setSettings(gaitLabSettings);
  if (gaitLabPersistenceTimer !== null) clearTimeout(gaitLabPersistenceTimer);
  gaitLabPersistenceTimer = setTimeout(() => {
    gaitLabPersistenceTimer = null;
    try {
      localStorage.setItem(GAIT_LAB_STORAGE_KEY, JSON.stringify(gaitLabSettings));
    } catch {
      console.warn("Gait Lab settings could not be persisted in this browser.");
    }
  }, 120);
  syncGaitLabUi();
}

gaitPanelController = createFloatingPanelController({
  panel: document.querySelector("#gait-lab-panel"),
  handle: document.querySelector("#gait-lab-panel .gait-lab-heading"),
  storageKey: GAIT_PANEL_LAYOUT_KEY,
});
inspectorPanelController = createFloatingPanelController({
  panel: document.querySelector("#telemetry-panel"),
  handle: document.querySelector("#joint-legend > h2"),
  storageKey: INSPECT_PANEL_LAYOUT_KEY,
  activeClass: "inspect-floating",
  clearWhenInactive: true,
});

function updateGaitLabMinimumSize() {
  const panel = document.querySelector("#gait-lab-panel");
  if (document.hidden || panel.hidden) return;
  const maximumHeight = Math.max(300, window.innerHeight - PANEL_VIEWPORT_MARGIN * 2);
  panel.style.minHeight = "0";
  const contentHeight = Math.min(panel.scrollHeight, maximumHeight);
  panel.style.minHeight = `${Math.ceil(contentHeight)}px`;
  return contentHeight;
}

function positionGaitLabPanel() {
  const panel = document.querySelector("#gait-lab-panel");
  if (document.hidden || panel.hidden) return;
  const contentHeight = updateGaitLabMinimumSize();
  if (gaitPanelController?.isCustomized()) return;
  const toolbarBounds = document.querySelector("nav").getBoundingClientRect();
  const top = Math.max(
    PANEL_VIEWPORT_MARGIN,
    Math.min(
      Math.ceil(toolbarBounds.bottom + 8),
      window.innerHeight - contentHeight - PANEL_VIEWPORT_MARGIN,
    ),
  );
  panel.style.top = `${top}px`;
  panel.style.left = `${Math.max(10, Math.round(toolbarBounds.left))}px`;
  panel.style.maxHeight = `${Math.max(300, window.innerHeight - top - PANEL_VIEWPORT_MARGIN)}px`;
}

function setGaitLabPanelOpen(open) {
  const panel = document.querySelector("#gait-lab-panel");
  panel.hidden = !open;
  gaitPanelController?.setActive(open);
  const button = document.querySelector("#gait-lab-button");
  button.setAttribute("aria-expanded", String(open));
  button.classList.toggle("open", open);
  if (!open) closeGaitSettingInfo();
  if (open) {
    updateGaitLabMinimumSize();
    if (!gaitPanelController?.isCustomized()) positionGaitLabPanel();
    gaitPanelController?.clamp();
  }
}

if (typeof ResizeObserver === "function") {
  new ResizeObserver(positionGaitLabPanel).observe(document.querySelector("nav"));
}

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

function motionPlanePoint(axisDirection, angle, radius) {
  return Math.abs(axisDirection.x) > 0.5
    ? new THREE.Vector3(0, Math.cos(angle) * radius, Math.sin(angle) * radius)
    : new THREE.Vector3(Math.cos(angle) * radius, 0, -Math.sin(angle) * radius);
}

function createMotionLine(points, color, opacity = jointOverlayOpacity) {
  const line = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({
      color,
      transparent: true,
      opacity,
      depthTest: false,
      depthWrite: false,
    }),
  );
  line.renderOrder = 19;
  return line;
}

function createJointCallout(color) {
  const canvas = document.createElement("canvas");
  canvas.width = 480;
  canvas.height = 128;
  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: false,
      depthWrite: false,
    }),
  );
  sprite.position.set(0.044, 0, 0.040);
  sprite.scale.set(0.105, 0.028, 1);
  sprite.center.set(0, 0.5);
  sprite.renderOrder = 24;
  sprite.visible = false;
  sprite.userData.calloutAspect = canvas.height / canvas.width;
  return { canvas, context: canvas.getContext("2d"), texture, sprite, color, lastText: "" };
}

function drawJointCallout(callout, meta, deltaDegrees, absoluteDegrees) {
  if (!callout) return;
  const deltaSign = deltaDegrees >= 0 ? "+" : "";
  const degree = String.fromCharCode(176);
  const plusMinus = String.fromCharCode(177);
  const textKey = `${meta.label}:${deltaDegrees.toFixed(1)}:${absoluteDegrees.toFixed(1)}`;
  if (callout.lastText === textKey) return;
  callout.lastText = textKey;

  const { canvas, context } = callout;
  context.clearRect(0, 0, canvas.width, canvas.height);
  const accent = `#${callout.color.toString(16).padStart(6, "0")}`;
  context.fillStyle = "rgba(16, 16, 18, 0.94)";
  context.strokeStyle = "rgba(255, 255, 255, 0.20)";
  context.lineWidth = 2;
  context.beginPath();
  if (typeof context.roundRect === "function") {
    context.roundRect(2, 2, canvas.width - 4, canvas.height - 4, 14);
  } else {
    context.rect(2, 2, canvas.width - 4, canvas.height - 4);
  }
  context.fill();
  context.stroke();

  context.fillStyle = accent;
  if (typeof context.roundRect === "function") {
    context.beginPath();
    context.roundRect(15, 14, 4, 20, 2);
    context.fill();
  } else {
    context.fillRect(15, 14, 4, 20);
  }

  context.fillStyle = "#f3f4f1";
  context.font = "750 19px Cascadia Mono, Consolas, monospace";
  context.fillText(meta.label, 31, 29);
  context.fillStyle = "#d0d0d4";
  context.font = "650 17px Segoe UI, Arial, sans-serif";
  context.fillText(meta.title, 72, 29);
  context.fillStyle = "#85858c";
  context.font = "750 12px Cascadia Mono, Consolas, monospace";
  context.fillText(`AXIS ${meta.axis}`, 393, 28);

  context.strokeStyle = "rgba(255, 255, 255, 0.10)";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(20, 42);
  context.lineTo(canvas.width - 20, 42);
  context.stroke();

  context.fillStyle = "#85858c";
  context.font = "750 11px Cascadia Mono, Consolas, monospace";
  context.fillText("OFFSET", 21, 62);
  context.fillStyle = accent;
  context.font = "750 28px Cascadia Mono, Consolas, monospace";
  context.fillText(`${deltaSign}${deltaDegrees.toFixed(1)}${degree}`, 21, 96);

  context.fillStyle = "#b9b9be";
  context.font = "650 13px Cascadia Mono, Consolas, monospace";
  context.fillText(`SERVO ${absoluteDegrees.toFixed(1)}${degree}`, 224, 66);
  context.fillStyle = "#85858c";
  context.font = "650 12px Cascadia Mono, Consolas, monospace";
  context.fillText(`LIMIT ${plusMinus}45${degree}`, 224, 91);
  callout.texture.needsUpdate = true;
}

function updateJointCalloutScale() {
  if (!jointOverlayVisible) return;
  const viewportHeight = Math.max(1, renderer.domElement.clientHeight);
  const worldHeight = camera === perspectiveCamera
    ? 2 * camera.position.distanceTo(controls.target) *
      Math.tan(THREE.MathUtils.degToRad(PERSPECTIVE_FOV_DEG / 2))
    : orthographicViewHeight / orthographicCamera.zoom;
  const targetPixels = 190;
  const targetWidth = THREE.MathUtils.clamp(
    worldHeight * targetPixels / viewportHeight,
    0.075,
    0.160,
  );
  linkageRuntimes.forEach((runtime) => {
    Object.values(runtime?.annotations || {}).forEach((annotation) => {
      if (!annotation.callout) return;
      const { sprite } = annotation.callout;
      const nextScale = targetWidth;
      if (Math.abs(sprite.scale.x - nextScale) < 0.0002) return;
      sprite.scale.set(nextScale, nextScale * sprite.userData.calloutAspect, 1);
    });
  });
}

function createJointAnnotation(parent, position, text, active, axisDirection) {
  const color = active ? ACTIVE_JOINT_COLOR : PASSIVE_JOINT_COLOR;
  const group = new THREE.Group();
  group.position.copy(position);
  group.visible = jointOverlayVisible;
  parent.add(group);

  const marker = new THREE.Mesh(
    new THREE.TorusGeometry(
      active ? 0.010 : 0.0045,
      active ? 0.0015 : 0.00055,
      8,
      32,
    ),
    new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: active ? jointOverlayOpacity : jointOverlayOpacity * 0.58,
      depthTest: false,
      depthWrite: false,
    }),
  );
  marker.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axisDirection);
  marker.renderOrder = 18;
  group.add(marker);

  const sweep = active
    ? createMotionLine(
        Array.from({ length: 25 }, (_, index) =>
          motionPlanePoint(
            axisDirection,
            THREE.MathUtils.lerp(-Math.PI / 4, Math.PI / 4, index / 24),
            0.013,
          ),
        ),
        color,
        jointOverlayOpacity * 0.5,
      )
    : null;
  if (sweep) group.add(sweep);

  const indicator = active
    ? createMotionLine(
        [new THREE.Vector3(), motionPlanePoint(axisDirection, 0, 0.016)],
        color,
      )
    : null;
  if (indicator) group.add(indicator);

  const leader = active
    ? createMotionLine(
        [new THREE.Vector3(0.011, 0, 0.008), new THREE.Vector3(0.044, 0, 0.040)],
        color,
        jointOverlayOpacity * 0.7,
      )
    : null;
  if (leader) group.add(leader);

  const callout = active ? createJointCallout(color) : null;
  if (callout) group.add(callout.sprite);

  return {
    group,
    marker,
    sweep,
    indicator,
    leader,
    callout,
    active,
    axisDirection,
    text,
  };
}

function createBodyReferenceOverlay() {
  const group = new THREE.Group();
  group.position.z = 0.012;
  group.visible = jointOverlayVisible;
  cadRoot.add(group);

  const plane = new THREE.LineLoop(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-0.19, -0.09, 0),
      new THREE.Vector3(0.19, -0.09, 0),
      new THREE.Vector3(0.19, 0.09, 0),
      new THREE.Vector3(-0.19, 0.09, 0),
    ]),
    new THREE.LineBasicMaterial({
      color: 0x8ea4a7,
      transparent: true,
      opacity: jointOverlayOpacity * 0.52,
      depthTest: false,
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
      0.075,
      definition.color,
      0.010,
      0.006,
    );
    arrow.line.material.transparent = true;
    arrow.cone.material.transparent = true;
    arrow.line.material.depthTest = false;
    arrow.cone.material.depthTest = false;
    arrow.line.material.opacity = jointOverlayOpacity;
    arrow.cone.material.opacity = jointOverlayOpacity;
    group.add(arrow);

    return { arrow };
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
  groups.ground.userData.calibrationChannel = spec.channels.shoulder;
  groups.upper_driver.userData.calibrationChannel = spec.channels.upper;
  groups.upper_closure.userData.calibrationChannel = spec.channels.upper;
  ["lower_driver", "coupler", "lower_diagonal", "lower_closure"].forEach((role) => {
    groups[role].userData.calibrationChannel = spec.channels.lower;
  });

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
    return loadMesh(
      spec.meshes[role],
      legMaterialByRole[role],
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
  createMeasuredPoseOverlay();
  loading.classList.add("hidden");
}

function createMeasuredPoseOverlay() {
  const clonedAlignment = cadAlignment.clone(true);
  const sourceNodes = [];
  const clonedNodes = [];
  cadAlignment.traverse((object) => sourceNodes.push(object));
  clonedAlignment.traverse((object) => clonedNodes.push(object));
  const cloneBySource = new Map(
    sourceNodes.map((object, index) => [object, clonedNodes[index]]),
  );
  const measuredMaterial = new THREE.MeshStandardMaterial({
    color: 0x62d18a,
    emissive: 0x143d24,
    emissiveIntensity: 0.45,
    roughness: 0.42,
    metalness: 0.08,
    transparent: true,
    opacity: 0.34,
    depthWrite: false,
    side: THREE.DoubleSide,
  });
  clonedAlignment.traverse((object) => {
    if (object.isMesh) {
      object.material = measuredMaterial;
      object.castShadow = false;
      object.receiveShadow = false;
      object.renderOrder = 6;
    }
    if (object.isLine || object.isSprite) object.visible = false;
  });

  measuredLinkageRuntimes = linkageRuntimes.map((runtime) => {
    const measuredRuntime = {
      spec: runtime.spec,
      legRoot: cloneBySource.get(runtime.legRoot),
      groups: Object.fromEntries(
        Object.entries(runtime.groups).map(([name, group]) => [name, cloneBySource.get(group)]),
      ),
      footProbe: cloneBySource.get(runtime.footProbe),
      pins: {},
      annotations: {},
    };
    Object.values(runtime.pins).forEach((pin) => {
      const clonedPin = cloneBySource.get(pin);
      if (clonedPin) clonedPin.visible = false;
    });
    Object.values(runtime.annotations).forEach((annotation) => {
      const clonedAnnotation = cloneBySource.get(annotation.group);
      if (clonedAnnotation) clonedAnnotation.visible = false;
    });
    return measuredRuntime;
  });

  measuredRobotWorld = new THREE.Group();
  measuredRobotWorld.name = "live-measured-pose";
  measuredRobotWorld.visible = false;
  measuredRobotWorld.add(clonedAlignment);
  scene.add(measuredRobotWorld);
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
channels[RIDE_HEIGHT_CHANNEL_INDEX] = 2000;
channels[6] = 1000;
channels[7] = 1000;

const channelDefinitions = [
  { name: "ROLL / TILT", switch: false },
  { name: "PITCH / GAIT FWD", switch: false },
  { name: "HEIGHT / LEFT Y", switch: false, height: true },
  { name: "YAW / WALK TURN", switch: false },
  { name: "SA / STAND", switch: true },
  { name: "SB / UNBOUND", switch: false, unbound: true },
  { name: "SC / WALK", switch: true, walkMode: true },
  { name: "SD / TILT", switch: true },
];

function heightMillimetersFromChannel(channelValue) {
  const normalized = THREE.MathUtils.clamp((Number(channelValue) - 1000) / 1000, 0, 1);
  return THREE.MathUtils.lerp(RIDE_HEIGHT_MIN_MM, RIDE_HEIGHT_MAX_MM, normalized);
}

function heightFractionFromMillimeters(heightMillimeters) {
  return THREE.MathUtils.clamp(
    (Number(heightMillimeters) - RIDE_HEIGHT_MIN_MM) /
      (RIDE_HEIGHT_MAX_MM - RIDE_HEIGHT_MIN_MM),
    0,
    1,
  );
}

const channelBars = channelDefinitions.map((definition, index) => {
  const element = document.createElement("div");
  element.className = `channel${definition.switch ? " switch" : ""}${definition.unbound ? " unbound" : ""}`;
  element.innerHTML = `
    <div class="channel-heading"><strong>CH${index + 1}</strong><span>${definition.name}</span></div>
    <div class="channel-track"><div class="channel-fill"></div></div>
    <div class="channel-reading"><output>1500</output><span class="channel-position">MID</span></div>
  `;
  document.querySelector("#channel-bars").append(element);
  return {
    element,
    output: element.querySelector("output"),
    position: element.querySelector(".channel-position"),
  };
});
const renderedChannelValues = Array(16).fill(Number.NaN);

function updateChannelBars() {
  channelBars.forEach(({ element, output, position: positionElement }, index) => {
    const value = Math.max(1000, Math.min(2000, channels[index]));
    if (renderedChannelValues[index] === value) return;
    renderedChannelValues[index] = value;
    const position = value < 1250 ? "LOW" : value > 1750 ? "HIGH" : "MID";
    const positionLabel = channelDefinitions[index].walkMode
      ? position === "LOW" ? "STAND" : position === "MID" ? "CAREFUL" : "TROT"
      : channelDefinitions[index].height
      ? `${heightMillimetersFromChannel(value).toFixed(0)} MM`
      : channelDefinitions[index].unbound
        ? "UNBOUND"
        : position;
    element.style.setProperty("--level", `${(value - 1000) / 10}%`);
    element.classList.toggle("low", position === "LOW");
    element.classList.toggle("mid", position === "MID");
    element.classList.toggle("high", position === "HIGH");
    output.textContent = Math.round(value);
    positionElement.textContent =
      channelDefinitions[index].switch || channelDefinitions[index].height || channelDefinitions[index].unbound
        ? positionLabel
        : "";
  });
}

const socketProtocol = location.protocol === "https:" ? "wss" : "ws";
let socket;
let bridgeInput = { connected: false, channels: null };
const bridgeHeartbeat = createHeartbeatState();
let boxerHeartbeat = { connected: false, name: "", updatedAt: 0 };
let controlClaimUntil = performance.now() + 1000;
applyWorkspace(WORKSPACE_SIMULATION);

function claimControl() {
  if (!simulationCanOwnControl(applicationState, document.visibilityState)) return;
  controlClaimUntil = performance.now() + 2000;
}

window.addEventListener("pointerdown", claimControl, { capture: true });
window.addEventListener("keydown", claimControl, { capture: true });

function ingestLiveTelemetry(packet, receivedAt = Date.now()) {
  if (!telemetryBelongsToLiveConnection(liveConnectionState, packet, receivedAt)) {
    observeLiveDiagnosticPacket(liveDiagnosticsState, packet, false, receivedAt);
    return false;
  }
  const accepted = acceptLiveTelemetryPacket(liveTelemetryState, packet, receivedAt);
  observeLiveDiagnosticPacket(liveDiagnosticsState, packet, accepted, receivedAt);
  if (accepted) {
    acceptLiveControllerTelemetry(liveControllerState, packet, receivedAt);
    let gaitStateChanged = false;
    if (packet.gaitProfile) {
      gaitStateChanged = acceptRobotGaitProfile(liveGaitState, packet.gaitProfile);
    }
    if (typeof packet.diagnostics?.robotState === "string") {
      const robotState = packet.diagnostics.robotState.toLowerCase();
      gaitStateChanged ||= robotState !== liveGaitState.robotState;
      liveGaitState.robotState = robotState;
      liveConnectionState.robotState = robotState;
      setLiveSafetyRobotState(liveSafetyState, robotState);
    }
    if (typeof packet.capabilities?.persistentGaitProfiles === "boolean") {
      gaitStateChanged ||= packet.capabilities.persistentGaitProfiles !== liveGaitState.persistentApplySupported;
      liveGaitState.persistentApplySupported = packet.capabilities.persistentGaitProfiles;
    }
    if (gaitStateChanged && liveViewState.selected === LIVE_VIEW_GAITS) renderLiveGaitUi();
  }
  return accepted;
}

function connectControlBridge() {
  socket = new WebSocket(`${socketProtocol}://${location.host}/control`);
  socket.addEventListener("open", () => {
    markHeartbeatSocketOpen(bridgeHeartbeat);
    setLiveConnectionBridge(liveConnectionState, true);
    document.querySelector("#firmware-status").dataset.state = "online";
    sendBridgeHeartbeat();
    renderLiveConnectionUi();
  });
  socket.addEventListener("close", () => {
    markHeartbeatSocketClosed(bridgeHeartbeat);
    setLiveConnectionBridge(liveConnectionState, false);
    lockLiveSafetyState(liveSafetyState, "The local bridge disconnected. Robot-side outputs must fail safe.");
    revokeLiveManualControl(liveManualState, "The local bridge disconnected. Browser authority was revoked.");
    bridgeInput = { connected: false, channels: null };
    liveCalibrationState.benchModeAcknowledged = false;
    calibrationPendingRequestId = "";
    calibrationPendingAction = "";
    clearTimeout(calibrationRequestTimeout);
    calibrationRequestTimeout = null;
    liveGaitState.pendingRequestId = "";
    liveGaitState.pendingAction = "";
    liveGaitState.persistentApplySupported = false;
    clearTimeout(liveGaitPendingTimeout);
    liveGaitPendingTimeout = null;
    document.querySelector("#firmware-status").dataset.state = "offline";
    renderLiveConnectionUi();
    setTimeout(connectControlBridge, 800);
  });
  socket.addEventListener("message", (event) => {
    try {
      const message = JSON.parse(event.data);
      if ((message.type === "ready" || message.type === "input") && message.input) {
        bridgeInput = message.input;
      }
      if (message.type === "heartbeat-ack") {
        acceptHeartbeatAcknowledgement(bridgeHeartbeat, message);
      }
      if (message.type === "live-adapter-list" && Array.isArray(message.adapters)) {
        message.adapters.forEach((adapter) => acceptLiveAdapterAnnouncement(liveConnectionState, adapter));
        renderLiveConnectionUi();
      }
      if (message.type === "live-adapter-announce") {
        acceptLiveAdapterAnnouncement(liveConnectionState, message);
        renderLiveConnectionUi();
      }
      if (message.type === "live-adapter-removed") {
        removeLiveAdapter(liveConnectionState, message.adapterId, message.reason);
        lockLiveSafetyState(liveSafetyState, "The adapter heartbeat was lost. Robot-side outputs must fail safe.");
        liveCalibrationState.benchModeAcknowledged = false;
        liveGaitState.persistentApplySupported = false;
        revokeLiveManualControl(liveManualState, "The robot adapter disappeared. Browser authority was revoked.");
        renderLiveConnectionUi();
      }
      if (message.type === "live-connection-ack") {
        acceptLiveConnectionAck(message);
      }
      if (message.type === "live-safety-ack") {
        acceptLiveSafetyAck(message);
      }
      if (message.type === "live-manual-authority-ack") {
        acceptLiveManualAuthorityAck(message);
      }
      if (message.type === "live-safety-heartbeat-ack") {
        const connection = liveConnectionEnvelope(liveConnectionState);
        if (acceptLiveSafetyHeartbeatAcknowledgement(liveSafetyState, message, connection)) {
          renderLiveSafetyUi();
        }
      }
      if (message.type === "live-telemetry") {
        ingestLiveTelemetry(message);
      }
      if (message.type === "live-calibration-ack") {
        acceptCalibrationAcknowledgement(message);
      }
      if (message.type === "live-gait-ack") {
        acceptLiveGaitAcknowledgement(message);
      }
    } catch {
      // Ignore malformed local bridge packets.
    }
  });
}
connectControlBridge();

function sendBridgeHeartbeat() {
  if (!simulationCanOwnControl(applicationState, document.visibilityState)) return;
  if (socket?.readyState !== WebSocket.OPEN) return;
  socket.send(JSON.stringify(createHeartbeatMessage(bridgeHeartbeat)));
}

setInterval(sendBridgeHeartbeat, HEARTBEAT_INTERVAL_MS);

function formatPacketAge(age) {
  if (age === null) return "--";
  if (age < 1_000) return `${Math.round(age)} ms`;
  return `${(age / 1_000).toFixed(1)} s`;
}

function resetLiveCommandPermissions(reason) {
  liveCalibrationState.benchModeAcknowledged = false;
  liveGaitState.persistentApplySupported = false;
  if (reason) liveGaitState.status = reason;
}

function liveSafetyContext() {
  const snapshot = liveComparisonSnapshot(liveTelemetryState);
  const controller = liveControllerSnapshot(liveControllerState);
  return {
    connectionReady: liveConnectionIsReady(liveConnectionState),
    telemetryFresh: snapshot.expectedFresh && snapshot.measuredFresh,
    driveLinkAlive: controller.linkReady,
  };
}

function liveManualContext() {
  const snapshot = liveComparisonSnapshot(liveTelemetryState);
  const controller = liveControllerSnapshot(liveControllerState);
  return {
    connectionReady: liveConnectionIsReady(liveConnectionState),
    robotState: liveSafetyState.robotState,
    telemetryFresh: snapshot.expectedFresh && snapshot.measuredFresh,
    controllerLinkReady: controller.linkReady,
    workspaceActive: applicationState.workspace === WORKSPACE_REAL_ROBOT && document.visibilityState === "visible",
  };
}

function renderLiveManualUi() {
  const selected = liveConnectionState.adapters[liveConnectionState.selectedAdapterId] || null;
  setLiveManualSupport(liveManualState, selected?.capabilities?.manualControl === true);
  const context = liveManualContext();
  const remainingMs = Math.max(0, liveManualState.authorityExpiresAt - Date.now());
  document.querySelector("#live-manual-phase").textContent = liveManualState.phase.toUpperCase();
  document.querySelector("#live-manual-lease").textContent = liveManualState.authorityToken
    ? `${(remainingMs / 1_000).toFixed(1)} S`
    : "--";
  document.querySelector("#live-manual-status").textContent = liveManualState.status;
  document.querySelector("#live-manual-consent").checked = liveManualState.safetyConfirmed;
  document.querySelector("#live-manual-request").disabled = !liveManualCanRequest(liveManualState, context);
  document.querySelector("#live-manual-release").disabled = !liveManualState.authorityToken || Boolean(liveManualState.pendingRequestId);
  document.querySelector("#live-manual-estop").disabled = !context.connectionReady;
  const deadman = document.querySelector("#live-manual-deadman");
  deadman.disabled = liveManualState.phase !== "ready" && liveManualState.phase !== "controlling";
  deadman.dataset.active = String(liveManualState.deadmanActive);
  deadman.textContent = liveManualState.deadmanActive ? "DRIVING — RELEASE TO NEUTRAL" : "HOLD TO DRIVE";
  document.querySelectorAll("[data-live-manual-mode]").forEach((button) => {
    button.setAttribute("aria-pressed", String(button.dataset.liveManualMode === liveManualState.mode));
    button.disabled = !liveManualState.authorityToken || liveManualState.deadmanActive;
  });
  document.querySelectorAll("[data-live-manual-axis]").forEach((input) => {
    const value = liveManualState.axes[input.dataset.liveManualAxis];
    input.value = String(value);
    input.disabled = !liveManualState.authorityToken;
    document.querySelector(`[data-live-manual-output="${input.dataset.liveManualAxis}"]`).textContent = value.toFixed(2);
  });
}

function sendLiveManualAuthority(action) {
  const connection = liveConnectionEnvelope(liveConnectionState);
  if (socket?.readyState !== WebSocket.OPEN || !connection || !liveConnectionIsReady(liveConnectionState)) return false;
  const requestId = crypto.randomUUID();
  const command = createLiveManualAuthorityCommand(liveManualState, action, connection, requestId);
  if (!command || (action === "request-authority" && !liveManualCanRequest(liveManualState, liveManualContext()))) return false;
  markLiveManualPending(liveManualState, command);
  socket.send(JSON.stringify(command));
  clearTimeout(liveManualRequestTimeout);
  liveManualRequestTimeout = setTimeout(() => {
    if (failLiveManualRequest(liveManualState, requestId, "Manual-control acknowledgement timed out. No authority was assumed.")) {
      renderLiveManualUi();
    }
  }, 2_000);
  renderLiveManualUi();
  return true;
}

function acceptLiveManualAuthorityAck(message) {
  const connection = liveConnectionEnvelope(liveConnectionState);
  if (!acceptLiveManualAuthorityAcknowledgement(liveManualState, message, connection)) return false;
  clearTimeout(liveManualRequestTimeout);
  liveManualRequestTimeout = null;
  renderLiveManualUi();
  return true;
}

function sendLiveManualFrame(forceNeutral = false) {
  const connection = liveConnectionEnvelope(liveConnectionState);
  const frame = createLiveManualControlFrame(liveManualState, connection, Date.now(), forceNeutral);
  if (!frame || socket?.readyState !== WebSocket.OPEN) return false;
  socket.send(JSON.stringify(frame));
  return true;
}

function stopLiveManualDeadman() {
  if (!liveManualState.deadmanActive) return false;
  sendLiveManualFrame(true);
  endLiveManualDeadman(liveManualState);
  renderLiveManualUi();
  return true;
}

function releaseLiveManualControl(reason = "Manual-control authority released. Robot command is neutral.") {
  if (!liveManualState.authorityToken) return false;
  sendLiveManualFrame(true);
  const sent = sendLiveManualAuthority("release-authority");
  if (!sent) revokeLiveManualControl(liveManualState, reason);
  renderLiveManualUi();
  return true;
}

function serviceLiveManualControl() {
  if (liveManualState.authorityToken) {
    const context = liveManualContext();
    const invalid = Date.now() >= liveManualState.authorityExpiresAt ||
      context.robotState !== "armed" || !context.connectionReady || !context.telemetryFresh ||
      !context.controllerLinkReady || !context.workspaceActive;
    if (invalid) {
      sendLiveManualFrame(true);
      tickLiveManualControl(liveManualState, context);
    } else if (liveManualState.deadmanActive) {
      sendLiveManualFrame(false);
    }
  }
  if (liveManualDialog.open) renderLiveManualUi();
}

setInterval(serviceLiveManualControl, LIVE_MANUAL_FRAME_INTERVAL_MS);

function renderLiveSafetyUi() {
  const context = liveSafetyContext();
  const connected = context.connectionReady;
  const canArm = liveSafetyCanArm(liveSafetyState, context);
  const stateLabel = connected ? liveSafetyState.robotState : "disconnected";
  const armButton = document.querySelector("#live-safety-arm");
  armButton.disabled = !canArm || Boolean(liveSafetyState.pendingRequestId);
  armButton.style.setProperty("--arm-progress", String(liveSafetyState.armHoldProgress));
  armButton.querySelector("span").textContent = liveSafetyState.armHoldStartedAt
    ? `KEEP HOLDING ${Math.round(liveSafetyState.armHoldProgress * 100)}%`
    : "HOLD TO ARM";
  document.querySelector("#live-safety-disarm").disabled =
    liveSafetyState.robotState !== "armed" || Boolean(liveSafetyState.pendingRequestId);
  document.querySelector("#live-safety-estop").disabled =
    !connected || Boolean(liveSafetyState.pendingRequestId) || ["estopped", "watchdog"].includes(liveSafetyState.robotState);
  document.querySelector("#live-safety-reset-estop").disabled =
    liveSafetyState.robotState !== "estopped" || Boolean(liveSafetyState.pendingRequestId);
  document.querySelector("#live-safety-status").textContent = liveSafetyState.status;
  const watchdog = document.querySelector("#live-safety-watchdog");
  watchdog.textContent = liveSafetyState.robotState === "armed"
    ? `${Math.round(liveSafetyState.watchdogRemainingMs)} MS`
    : liveSafetyState.watchdogTripped ? "TRIPPED" : "INACTIVE";
  watchdog.dataset.state = liveSafetyState.watchdogTripped
    ? "fault"
    : liveSafetyState.robotState === "armed" ? "active" : "inactive";
  document.querySelector("#live-robot-safety-state").textContent = stateLabel.toUpperCase();
}

function sendLiveSafetyCommand(action) {
  const connection = liveConnectionEnvelope(liveConnectionState);
  if (socket?.readyState !== WebSocket.OPEN || !connection || !liveConnectionIsReady(liveConnectionState)) return false;
  const requestId = crypto.randomUUID();
  const command = createLiveSafetyCommand(liveSafetyState, action, connection, requestId);
  if (!command) return false;
  markLiveSafetyPending(liveSafetyState, command);
  socket.send(JSON.stringify(command));
  clearTimeout(liveSafetyRequestTimeout);
  liveSafetyRequestTimeout = setTimeout(() => {
    if (!failLiveSafetyRequest(
      liveSafetyState,
      requestId,
      `${action.toUpperCase()} acknowledgement timed out. Treat the robot state as unverified.`,
    )) return;
    if (action === "arm") setLiveSafetyRobotState(liveSafetyState, "unknown", "timeout");
    renderLiveSafetyUi();
  }, 2_000);
  renderLiveSafetyUi();
  return true;
}

function acceptLiveSafetyAck(message) {
  const connection = liveConnectionEnvelope(liveConnectionState);
  if (!acceptLiveSafetyAcknowledgement(liveSafetyState, message, connection)) return false;
  clearTimeout(liveSafetyRequestTimeout);
  liveSafetyRequestTimeout = null;
  liveConnectionState.robotState = liveSafetyState.robotState;
  liveGaitState.robotState = liveSafetyState.robotState;
  if (liveSafetyState.robotState !== "disarmed") resetLiveCommandPermissions();
  renderLiveSafetyUi();
  updateLiveComparisonUi();
  return true;
}

function sendLiveSafetyHeartbeat() {
  if (liveSafetyState.armHoldStartedAt) {
    const hold = updateLiveArmHold(liveSafetyState, liveSafetyContext());
    if (hold.complete) sendLiveSafetyCommand("arm");
  }
  const connection = liveConnectionEnvelope(liveConnectionState);
  const heartbeat = createLiveSafetyHeartbeat(
    liveSafetyState,
    connection,
    applicationState.workspace === WORKSPACE_REAL_ROBOT && document.visibilityState === "visible",
  );
  if (heartbeat && socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(heartbeat));
  if (tickLiveSafetyWatchdog(liveSafetyState)) {
    liveConnectionState.robotState = "watchdog";
    liveGaitState.robotState = "watchdog";
    resetLiveCommandPermissions("Safety watchdog tripped. Profile changes remain blocked until the robot reports disarmed.");
  }
  renderLiveSafetyUi();
}

setInterval(sendLiveSafetyHeartbeat, LIVE_SAFETY_HEARTBEAT_INTERVAL_MS);

function renderLiveConnectionUi() {
  const now = Date.now();
  const connected = liveConnectionIsReady(liveConnectionState, now);
  const adapters = visibleLiveAdapters(liveConnectionState, now);
  const selected = liveConnectionState.adapters[liveConnectionState.selectedAdapterId] || null;
  const phase = document.querySelector("#live-connection-phase");
  phase.textContent = liveConnectionState.phase.toUpperCase();
  phase.dataset.state = connected ? "connected" : liveConnectionState.phase;
  document.querySelector("#live-connection-status").textContent = liveConnectionState.status;

  document.querySelectorAll("[data-live-transport]").forEach((button) => {
    button.setAttribute("aria-pressed", String(button.dataset.liveTransport === liveConnectionState.transportFilter));
    button.disabled = Boolean(liveConnectionState.sessionId || liveConnectionState.pendingRequestId);
  });

  const list = document.querySelector("#live-adapter-list");
  list.replaceChildren();
  adapters.forEach((adapter) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "live-adapter-card";
    button.dataset.adapterId = adapter.adapterId;
    button.setAttribute("role", "option");
    button.setAttribute("aria-selected", String(adapter.adapterId === liveConnectionState.selectedAdapterId));
    button.disabled = Boolean(liveConnectionState.sessionId || liveConnectionState.pendingRequestId);
    const title = document.createElement("strong");
    title.textContent = adapter.name;
    const transport = document.createElement("span");
    transport.textContent = adapter.transport.toUpperCase();
    const detail = document.createElement("em");
    detail.textContent = `${adapter.robot.name || "Domino"} / ${adapter.robot.id || "identity not reported"}`;
    button.append(title, transport, detail);
    list.append(button);
  });
  document.querySelector("#live-adapter-empty").hidden = adapters.length > 0;

  document.querySelector("#live-adapter-transport").textContent = selected
    ? `${selected.transport.toUpperCase()} / ${selected.state.toUpperCase()}`
    : "NO ADAPTER";
  document.querySelector("#live-adapter-robot-name").textContent = selected?.robot.name || "Not selected";
  document.querySelector("#live-adapter-robot-id").textContent = selected?.robot.id || "--";
  document.querySelector("#live-adapter-firmware").textContent = selected?.robot.firmwareVersion || "--";
  document.querySelector("#live-adapter-signal").textContent = Number.isFinite(selected?.signalPercent)
    ? `${Math.round(selected.signalPercent)}%`
    : "--";
  document.querySelector("#live-adapter-endpoint").textContent = selected?.endpoint || "--";
  document.querySelector("#live-adapter-session").textContent = liveConnectionState.sessionId
    ? liveConnectionState.sessionId.slice(0, 12)
    : "--";
  document.querySelectorAll("[data-live-capability]").forEach((badge) => {
    badge.dataset.supported = String(selected?.capabilities?.[badge.dataset.liveCapability] === true);
  });

  document.querySelector("#live-connection-discover").disabled =
    !liveConnectionState.bridgeConnected || Boolean(liveConnectionState.pendingRequestId || liveConnectionState.sessionId);
  document.querySelector("#live-connection-connect").disabled =
    !liveConnectionState.bridgeConnected || !selected || Boolean(liveConnectionState.pendingRequestId || liveConnectionState.sessionId);
  document.querySelector("#live-connection-disconnect").disabled =
    !connected || liveSafetyState.robotState === "armed" || Boolean(liveConnectionState.pendingRequestId);

  const openButton = document.querySelector("#live-connection-open");
  openButton.textContent = connected ? "MANAGE" : "CONNECT";
  openButton.dataset.state = connected ? "connected" : "offline";
  renderLiveManualUi();
}

function sendLiveConnectionRequest(action) {
  if (socket?.readyState !== WebSocket.OPEN) return false;
  const requestId = crypto.randomUUID();
  const command = createLiveConnectionCommand(liveConnectionState, action, requestId);
  if (!command) return false;
  markLiveConnectionPending(liveConnectionState, command);
  socket.send(JSON.stringify(command));
  clearTimeout(liveConnectionRequestTimeout);
  liveConnectionRequestTimeout = setTimeout(() => {
    if (!failLiveConnectionRequest(
      liveConnectionState,
      requestId,
      "The connection request timed out. Robot commands remain blocked.",
    )) return;
    resetLiveCommandPermissions("The engineering session is unavailable. Robot profile changes remain blocked.");
    renderLiveConnectionUi();
    updateLiveComparisonUi();
  }, 4_000);
  renderLiveConnectionUi();
  return true;
}

function acceptLiveConnectionAck(message) {
  if (!acceptLiveConnectionAcknowledgement(liveConnectionState, message)) return false;
  clearTimeout(liveConnectionRequestTimeout);
  liveConnectionRequestTimeout = null;
  if (!liveConnectionIsReady(liveConnectionState)) {
    lockLiveSafetyState(liveSafetyState, "No engineering session is active. Robot-side outputs must fail safe.");
    revokeLiveManualControl(liveManualState, "No engineering session is active. Browser authority was revoked.");
    resetLiveCommandPermissions("No engineering session is active. Robot profile changes remain blocked.");
  } else {
    setLiveSafetyRobotState(liveSafetyState, liveConnectionState.robotState);
    liveSafetyState.status = "Engineering session connected. Verify telemetry and drive link before arming.";
  }
  renderLiveConnectionUi();
  renderLiveSafetyUi();
  updateLiveComparisonUi();
  return true;
}

function updateSimulationLinkHealth() {
  const now = Date.now();
  const bridgeState = heartbeatStatus(bridgeHeartbeat, now);
  const bridgeStateOutput = document.querySelector("#sim-bridge-state");
  bridgeStateOutput.textContent = bridgeState.toUpperCase();
  bridgeStateOutput.dataset.state = bridgeState;
  document.querySelector("#sim-heartbeat-rtt").textContent = bridgeHeartbeat.roundTripMs === null
    ? "-- ms"
    : `${Math.round(bridgeHeartbeat.roundTripMs)} ms`;
  document.querySelector("#sim-heartbeat-age").textContent = formatPacketAge(
    packetAgeMs(bridgeHeartbeat.lastAckAt, now),
  );

  const boxerAge = packetAgeMs(boxerHeartbeat.updatedAt, now);
  const boxerConnected = boxerHeartbeat.connected && boxerAge !== null && boxerAge < 1_000;
  const boxerStateOutput = document.querySelector("#sim-boxer-state");
  boxerStateOutput.textContent = boxerConnected ? "CONNECTED" : "OFFLINE";
  boxerStateOutput.dataset.state = boxerConnected ? "connected" : "disconnected";
  boxerStateOutput.title = boxerHeartbeat.name || "No RadioMaster Boxer detected";
  document.querySelector("#sim-boxer-age").textContent = formatPacketAge(boxerAge);
  const boxerBadge = document.querySelector("#link-status");
  boxerBadge.dataset.state = boxerConnected ? "online" : "offline";
  boxerBadge.title = boxerConnected
    ? `${boxerHeartbeat.name} / packet ${formatPacketAge(boxerAge)} old`
    : "RadioMaster Boxer not connected";

  const crsfConnected = firmwareState?.link_alive === true;
  const crsfStateOutput = document.querySelector("#sim-crsf-state");
  crsfStateOutput.textContent = crsfConnected ? "CONNECTED" : "OFFLINE";
  crsfStateOutput.dataset.state = crsfConnected ? "connected" : "disconnected";
  document.querySelector("#sim-crsf-frames").textContent = Number(
    firmwareState?.accepted_frames || 0,
  ).toLocaleString();
}

setInterval(updateSimulationLinkHealth, 100);

function formatLiveAngle(value, fallback = "--.-°") {
  if (!Number.isFinite(value)) return fallback;
  return `${value >= 0 ? "+" : ""}${value.toFixed(1)}°`;
}

function formatLiveHeight(value, fallback = "--- mm") {
  if (!Number.isFinite(value)) return fallback;
  return `${Math.round(value)} mm`;
}

function setLiveStreamState(selector, connected) {
  const output = document.querySelector(selector);
  output.textContent = connected ? "STREAMING" : "NO STREAM";
  output.dataset.state = connected ? "connected" : "disconnected";
}

const liveChartCanvas = document.querySelector("#live-comparison-chart");
const liveChartSignal = document.querySelector("#live-chart-signal");
const liveDataChartCanvas = document.querySelector("#live-data-chart");
const liveDataChartSignal = document.querySelector("#live-data-chart-signal");
const liveChartDefinitions = {
  pitchDeg: { title: "Body pitch / degrees", field: "pitchDeg" },
  rollDeg: { title: "Body roll / degrees", field: "rollDeg" },
  yawDeg: { title: "Body yaw / degrees", field: "yawDeg" },
  heightMm: { title: "Body height / millimetres", field: "heightMm" },
};

function formatSessionDuration(durationMs) {
  if (durationMs < 10_000) return `${(durationMs / 1_000).toFixed(1)} S`;
  return `${Math.floor(durationMs / 60_000)}:${String(Math.floor(durationMs / 1_000) % 60).padStart(2, "0")}`;
}

function drawLiveSeries(context, points, color, width, height, xAt, yAt) {
  if (points.length < 2) return;
  context.beginPath();
  points.forEach((point, index) => {
    const x = xAt(point.sample.elapsedMs);
    const y = yAt(point.value);
    if (index === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  });
  context.strokeStyle = color;
  context.lineWidth = 1.6;
  context.lineJoin = "round";
  context.lineCap = "round";
  context.stroke();
}

function renderLiveChart(canvas, signal, emptySelector) {
  if (applicationState.workspace !== WORKSPACE_REAL_ROBOT) return;
  const bounds = canvas.getBoundingClientRect();
  if (bounds.width < 10 || bounds.height < 10) return;
  const context = canvas.getContext("2d");
  const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.round(bounds.width);
  const height = Math.round(bounds.height);
  const renderWidth = Math.round(width * pixelRatio);
  const renderHeight = Math.round(height * pixelRatio);
  if (canvas.width !== renderWidth || canvas.height !== renderHeight) {
    canvas.width = renderWidth;
    canvas.height = renderHeight;
  }
  context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  context.clearRect(0, 0, width, height);

  const samples = liveSessionState.samples.slice(-300);
  const emptyState = document.querySelector(emptySelector);
  emptyState.hidden = samples.length > 1;
  if (samples.length < 2) return;

  const definition = liveChartDefinitions[signal.value] || liveChartDefinitions.pitchDeg;
  const expected = samples.map((sample) => ({ sample, value: sample.expectedBody[definition.field] }));
  const measured = samples.map((sample) => ({ sample, value: sample.measuredBody[definition.field] }));
  const error = samples.map((sample) => ({ sample, value: sample.bodyError[definition.field] }));
  const values = [...expected, ...measured, ...error].map((point) => point.value).filter(Number.isFinite);
  let minimum = Math.min(0, ...values);
  let maximum = Math.max(0, ...values);
  if (Math.abs(maximum - minimum) < 1e-6) {
    minimum -= 1;
    maximum += 1;
  }
  const verticalPadding = (maximum - minimum) * 0.12;
  minimum -= verticalPadding;
  maximum += verticalPadding;
  const firstTime = samples[0].elapsedMs;
  const lastTime = Math.max(firstTime + 1, samples.at(-1).elapsedMs);
  const plot = { left: 10, right: width - 10, top: 30, bottom: height - 10 };
  const xAt = (time) => plot.left + ((time - firstTime) / (lastTime - firstTime)) * (plot.right - plot.left);
  const yAt = (value) => plot.bottom - ((value - minimum) / (maximum - minimum)) * (plot.bottom - plot.top);
  const dark = document.documentElement.dataset.theme === "dark";

  context.strokeStyle = dark ? "rgba(255,255,255,.08)" : "rgba(40,45,40,.1)";
  context.lineWidth = 1;
  for (let index = 0; index <= 4; index += 1) {
    const y = plot.top + ((plot.bottom - plot.top) * index) / 4;
    context.beginPath();
    context.moveTo(plot.left, y);
    context.lineTo(plot.right, y);
    context.stroke();
  }
  if (minimum <= 0 && maximum >= 0) {
    context.strokeStyle = dark ? "rgba(255,255,255,.2)" : "rgba(40,45,40,.22)";
    context.beginPath();
    context.moveTo(plot.left, yAt(0));
    context.lineTo(plot.right, yAt(0));
    context.stroke();
  }
  drawLiveSeries(context, expected, dark ? "#f0f0f2" : "#343733", width, height, xAt, yAt);
  drawLiveSeries(context, measured, "#63c383", width, height, xAt, yAt);
  drawLiveSeries(context, error, "#e16d5a", width, height, xAt, yAt);
}

function renderLiveComparisonChart() {
  renderLiveChart(liveChartCanvas, liveChartSignal, "#live-chart-empty");
  renderLiveChart(liveDataChartCanvas, liveDataChartSignal, "#live-data-chart-empty");
}

function downloadLiveSessionCsv(session, name = "domino-live-session") {
  if (!session?.samples?.length) return;
  const blob = new Blob([liveSessionCsv(session)], { type: "text/csv;charset=utf-8" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `${name}-${new Date().toISOString().replace(/[:.]/g, "-")}.csv`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 0);
}

function archivedSessionMetrics(session) {
  const durationMs = session.startedAt && session.stoppedAt
    ? Math.max(0, session.stoppedAt - session.startedAt)
    : 0;
  const errors = session.samples.map((sample) => sample.worstJointErrorDeg).filter(Number.isFinite);
  const power = session.samples.map((sample) => sample.power?.powerW).filter(Number.isFinite);
  return {
    durationMs,
    peakErrorDeg: errors.length ? Math.max(...errors) : null,
    averagePowerW: power.length ? power.reduce((total, value) => total + value, 0) / power.length : null,
  };
}

function createSessionMetric(label, value) {
  const metric = document.createElement("div");
  const caption = document.createElement("span");
  const output = document.createElement("strong");
  caption.textContent = label;
  output.textContent = value;
  metric.append(caption, output);
  return metric;
}

function renderLiveSessionArchive() {
  const summary = liveSessionSummary(liveSessionState);
  document.querySelector("#live-sessions-current-state").textContent = summary.status.toUpperCase();
  document.querySelector("#live-sessions-current-samples").textContent = summary.sampleCount.toLocaleString();
  document.querySelector("#live-sessions-current-duration").textContent = formatSessionDuration(summary.durationMs);
  document.querySelector("#live-session-archive-count").textContent = `${liveSessionArchive.length} SAVED`;
  const list = document.querySelector("#live-session-list");
  list.replaceChildren();
  document.querySelector("#live-session-list-empty").hidden = liveSessionArchive.length > 0;

  liveSessionArchive.forEach((session) => {
    const metrics = archivedSessionMetrics(session);
    const entry = document.createElement("article");
    entry.className = "live-session-entry";
    entry.append(
      createSessionMetric("RECORDED", new Date(session.startedAt).toLocaleString()),
      createSessionMetric("DURATION", formatSessionDuration(metrics.durationMs)),
      createSessionMetric("SAMPLES", session.samples.length.toLocaleString()),
      createSessionMetric(
        "PEAK / AVG POWER",
        `${formatLiveAngle(metrics.peakErrorDeg, "--°")} / ${Number.isFinite(metrics.averagePowerW) ? `${metrics.averagePowerW.toFixed(1)} W` : "-- W"}`,
      ),
    );
    const exportButton = document.createElement("button");
    exportButton.type = "button";
    exportButton.textContent = "EXPORT CSV";
    exportButton.addEventListener("click", () => downloadLiveSessionCsv(session, "domino-live-archive"));
    const deleteButton = document.createElement("button");
    deleteButton.type = "button";
    deleteButton.className = "delete-session";
    deleteButton.textContent = "DELETE";
    deleteButton.addEventListener("click", () => {
      removeArchivedLiveSession(liveSessionArchive, session.id);
      renderLiveSessionArchive();
    });
    entry.append(exportButton, deleteButton);
    list.append(entry);
  });
}

function renderLiveDataTable() {
  const body = document.querySelector("#live-data-table-body");
  const samples = liveSessionState.samples.slice(-20).reverse();
  body.replaceChildren();
  if (samples.length === 0) {
    const row = body.insertRow();
    const cell = row.insertCell();
    cell.colSpan = 7;
    cell.textContent = "No recorded samples.";
    return;
  }
  samples.forEach((sample) => {
    const row = body.insertRow();
    [
      `${(sample.elapsedMs / 1_000).toFixed(2)} s`,
      `${Math.round(sample.alignmentMs)} ms`,
      formatLiveAngle(sample.expectedBody.pitchDeg),
      formatLiveAngle(sample.measuredBody.pitchDeg),
      formatLiveAngle(sample.bodyError.pitchDeg),
      Number.isFinite(sample.power?.voltageV) ? `${sample.power.voltageV.toFixed(2)} V` : "--",
      Number.isFinite(sample.power?.powerW) ? `${sample.power.powerW.toFixed(1)} W` : "--",
    ].forEach((value) => {
      const cell = row.insertCell();
      cell.textContent = value;
    });
  });
}

function selectedCalibrationDefinition() {
  return LIVE_CALIBRATION_JOINTS.find(
    (definition) => definition.channel === liveCalibrationState.selectedChannel,
  );
}

function selectedCalibrationJoint() {
  return liveCalibrationState.profile.joints.find(
    (joint) => joint.channel === liveCalibrationState.selectedChannel,
  );
}

function formatCalibrationDegrees(value, digits = 1) {
  if (!Number.isFinite(value)) return "--°";
  return `${value >= 0 ? "+" : ""}${value.toFixed(digits)}°`;
}

function renderCalibrationJointMap() {
  const list = document.querySelector("#live-calibration-joint-list");
  list.replaceChildren();
  ["FL", "FR", "BL", "BR"].forEach((leg) => {
    const group = document.createElement("section");
    const heading = document.createElement("header");
    const legName = document.createElement("strong");
    const location = document.createElement("span");
    legName.textContent = leg;
    location.textContent = leg.startsWith("F") ? "FRONT LEG" : "REAR LEG";
    heading.append(legName, location);
    group.append(heading);
    LIVE_CALIBRATION_JOINTS.filter((joint) => joint.leg === leg).forEach((joint) => {
      const button = document.createElement("button");
      button.type = "button";
      button.dataset.calibrationChannel = String(joint.channel);
      button.setAttribute("aria-pressed", String(joint.channel === liveCalibrationState.selectedChannel));
      const label = document.createElement("strong");
      const channel = document.createElement("span");
      label.textContent = joint.label.replace(`${leg} `, "").toUpperCase();
      channel.textContent = `CH ${joint.channel}`;
      button.append(label, channel);
      group.append(button);
    });
    list.append(group);
  });
}

function renderCalibrationReview() {
  const body = document.querySelector("#live-calibration-review-body");
  body.replaceChildren();
  LIVE_CALIBRATION_JOINTS.forEach((definition) => {
    const joint = liveCalibrationState.profile.joints.find(
      (candidate) => candidate.channel === definition.channel,
    );
    const row = body.insertRow();
    [
      definition.label.toUpperCase(),
      String(definition.channel),
      formatCalibrationDegrees(joint.offsetDeg),
      joint.direction === -1 ? "INVERTED" : "NORMAL",
      formatCalibrationDegrees(joint.minimumDeg, 0),
      formatCalibrationDegrees(joint.maximumDeg, 0),
    ].forEach((value) => {
      const cell = row.insertCell();
      cell.textContent = value;
    });
  });
}

function renderLiveCalibrationUi() {
  const definition = selectedCalibrationDefinition();
  const joint = selectedCalibrationJoint();
  if (!definition || !joint) return;
  document.querySelectorAll("[data-calibration-step]").forEach((button) => {
    const active = Number(button.dataset.calibrationStep) === liveCalibrationState.step;
    button.setAttribute("aria-current", active ? "step" : "false");
  });
  document.querySelectorAll("[data-calibration-panel]").forEach((panel) => {
    panel.hidden = Number(panel.dataset.calibrationPanel) !== liveCalibrationState.step;
  });
  const progress = ((liveCalibrationState.step + 1) / LIVE_CALIBRATION_STEPS.length) * 100;
  document.querySelector("#live-calibration-progress").style.width = `${progress}%`;
  document.querySelector("#live-calibration-progress-label").textContent =
    `STEP ${liveCalibrationState.step + 1} / ${LIVE_CALIBRATION_STEPS.length}`;
  const back = document.querySelector("#live-calibration-back");
  back.disabled = liveCalibrationState.step === 0;
  const next = document.querySelector("#live-calibration-next");
  const nextStep = LIVE_CALIBRATION_STEPS[liveCalibrationState.step + 1];
  next.textContent = nextStep ? `NEXT: ${nextStep.label.toUpperCase()}` : "RETURN TO COMPARE";
  next.disabled = liveCalibrationState.step === 0 && !liveCalibrationState.safetyConfirmed;

  document.querySelector("#live-calibration-safety-confirm").checked =
    liveCalibrationState.safetyConfirmed;
  document.querySelector("#live-calibration-selected-summary").textContent =
    `${definition.label.toUpperCase()} / CHANNEL ${definition.channel}`;
  document.querySelector("#live-calibration-neutral-title").textContent = definition.label.toUpperCase();
  document.querySelector("#live-calibration-channel").textContent = definition.channel;
  document.querySelector("#live-calibration-reference").textContent = `${definition.neutralServoDeg.toFixed(2)}°`;
  document.querySelector("#live-calibration-preview-caption").textContent =
    `${definition.label.toUpperCase()} / ${formatCalibrationDegrees(joint.offsetDeg + liveCalibrationState.jogOffsetDeg)}`;
  document.querySelector("#live-calibration-offset").value = joint.offsetDeg.toFixed(1);
  document.querySelector("#live-calibration-direction").value = String(joint.direction);
  document.querySelector("#live-calibration-preview").checked = liveCalibrationState.previewEnabled;
  document.querySelector("#live-calibration-jog-value").textContent =
    formatCalibrationDegrees(liveCalibrationState.jogOffsetDeg);
  document.querySelector("#live-calibration-minimum").value = String(joint.minimumDeg);
  document.querySelector("#live-calibration-maximum").value = String(joint.maximumDeg);
  const logicalOutputs = [joint.minimumDeg, joint.maximumDeg].map(
    (limit) => definition.neutralServoDeg + joint.offsetDeg + joint.direction * limit,
  ).sort((a, b) => a - b);
  document.querySelector("#live-calibration-output-range").textContent =
    `${logicalOutputs[0].toFixed(2)}° - ${logicalOutputs[1].toFixed(2)}°`;
  const savedAt = liveCalibrationState.profile.savedAt;
  document.querySelector("#live-calibration-save-state").textContent = liveCalibrationState.dirty
    ? "UNSAVED CHANGES"
    : savedAt
      ? `SAVED ${new Date(savedAt).toLocaleTimeString()}`
      : "UNSAVED DRAFT";
  renderCalibrationJointMap();
  renderCalibrationReview();
}

function downloadCalibrationJson() {
  const blob = new Blob([calibrationProfileJson(liveCalibrationState.profile)], {
    type: "application/json;charset=utf-8",
  });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `domino-calibration-${new Date().toISOString().slice(0, 10)}.json`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 0);
}

function sendCalibrationCommand(action) {
  const connection = liveConnectionEnvelope(liveConnectionState);
  if (socket?.readyState !== WebSocket.OPEN || !connection || !liveConnectionIsReady(liveConnectionState)) return false;
  const requestId = crypto.randomUUID();
  const command = createCalibrationBenchCommand(
    liveCalibrationState,
    action,
    requestId,
  );
  if (!command) return false;
  Object.assign(command, connection);
  calibrationPendingRequestId = requestId;
  calibrationPendingAction = action;
  socket.send(JSON.stringify(command));
  clearTimeout(calibrationRequestTimeout);
  calibrationRequestTimeout = setTimeout(() => {
    if (calibrationPendingRequestId !== requestId) return;
    calibrationPendingRequestId = "";
    calibrationPendingAction = "";
    liveCalibrationState.benchModeAcknowledged = false;
    document.querySelector("#live-calibration-review-status").textContent =
      "The robot did not acknowledge the calibration request. Physical movement remains locked.";
    updateLiveComparisonUi();
  }, 3_000);
  return true;
}

function acceptCalibrationAcknowledgement(message) {
  const connection = liveConnectionEnvelope(liveConnectionState);
  if (
    !message ||
    message.type !== "live-calibration-ack" ||
    !connection ||
    message.adapterId !== connection.adapterId ||
    message.sessionId !== connection.sessionId ||
    message.requestId !== calibrationPendingRequestId ||
    message.action !== calibrationPendingAction
  ) return false;
  const accepted = message.accepted === true;
  if (message.action === "enter") {
    liveCalibrationState.benchModeAcknowledged = Boolean(
      accepted &&
      message.benchMode === true &&
      message.supportsSafeJog === true &&
      Number(message.maxSpeedDegPerSec) <= 5,
    );
  } else if (message.action === "exit") {
    liveCalibrationState.benchModeAcknowledged = false;
  } else if (message.action === "save-profile") {
    const status = document.querySelector("#live-calibration-review-status");
    status.textContent = accepted && message.persisted === true
      ? "Robot confirmed that the calibration profile was written to persistent storage."
      : `Robot rejected calibration save${message.reason ? `: ${message.reason}` : "."}`;
  }
  calibrationPendingRequestId = "";
  calibrationPendingAction = "";
  clearTimeout(calibrationRequestTimeout);
  calibrationRequestTimeout = null;
  renderLiveCalibrationUi();
  return true;
}

function formatDiagnosticUptime(value) {
  if (!Number.isFinite(value) || value < 0) return "--:--:--";
  const seconds = Math.floor(value / 1_000);
  return [
    Math.floor(seconds / 3600),
    Math.floor(seconds / 60) % 60,
    seconds % 60,
  ].map((part) => String(part).padStart(2, "0")).join(":");
}

function formatLiveGaitValue(entry, value) {
  if (!Number.isFinite(value)) return "--";
  const suffix = entry.unit ? ` ${entry.unit}` : "";
  return `${(value * (entry.scale || 1)).toFixed(entry.decimals)}${suffix}`;
}

function persistLiveGaitLibrary(selectedName = "") {
  gaitProfiles = Object.fromEntries(Object.entries(liveGaitLibrary).map(([name, profile]) => [
    name,
    sanitizeGaitLabSettings(profile.settings),
  ]));
  persistGaitProfiles();
  syncGaitProfileUi(selectedName);
}

function syncLiveGaitLibrary(selectedName = "") {
  const select = document.querySelector("#live-gait-library");
  const selection = selectedName || select.value;
  select.replaceChildren(new Option("SELECT A SAVED PROFILE", ""));
  Object.keys(liveGaitLibrary).sort((a, b) => a.localeCompare(b)).forEach((name) => {
    const source = liveGaitLibrary[name].source === "simulation" ? "SIM" : "LIVE";
    select.add(new Option(`${name} / ${source}`, name));
  });
  select.value = Object.hasOwn(liveGaitLibrary, selection) ? selection : "";
  document.querySelector("#live-gait-load").disabled = !select.value;
}

function rebuildLiveGaitSettings() {
  const container = document.querySelector("#live-gait-settings");
  container.replaceChildren();
  gaitLabControls.forEach((control) => {
    const label = document.createElement("label");
    label.dataset.gaitKey = control.key;
    const heading = document.createElement("span");
    const name = document.createElement("strong");
    const output = document.createElement("output");
    name.textContent = control.label.toUpperCase();
    output.textContent = formatGaitSetting(control, liveGaitState.draft.settings[control.key]);
    heading.append(name, output);
    const input = document.createElement("input");
    input.type = "range";
    input.min = control.min;
    input.max = control.max;
    input.step = control.step;
    input.value = liveGaitState.draft.settings[control.key];
    input.setAttribute("aria-label", control.label);
    const description = document.createElement("small");
    description.textContent = control.description;
    label.append(heading, input, description);
    container.append(label);
  });
}

function renderLiveGaitComparison() {
  const difference = liveGaitDiff(liveGaitState);
  const body = document.querySelector("#live-gait-compare-body");
  body.replaceChildren();
  difference.forEach((entry) => {
    const row = body.insertRow();
    row.dataset.changed = String(entry.changed === true);
    [
      entry.label.toUpperCase(),
      formatLiveGaitValue(entry, entry.draft),
      formatLiveGaitValue(entry, entry.robot),
      entry.delta === null ? "--" : formatLiveGaitValue(entry, entry.delta),
    ].forEach((value) => {
      const cell = row.insertCell();
      cell.textContent = value;
    });
  });
  const changes = difference.filter((entry) => entry.changed).length;
  document.querySelector("#live-gait-compare-summary").textContent = liveGaitState.robotProfile
    ? `${changes} PARAMETER${changes === 1 ? "" : "S"} DIFFER`
    : "ROBOT PROFILE NOT REPORTED";
  const risk = document.querySelector("#live-gait-risk");
  risk.replaceChildren();
  liveGaitRiskAssessment(liveGaitState.draft).forEach((finding) => {
    const row = document.createElement("p");
    row.dataset.severity = finding.severity;
    row.textContent = finding.message;
    risk.append(row);
  });
}

function renderLiveGaitUi() {
  const adapter = liveConnectionState.adapters[liveConnectionState.selectedAdapterId];
  const gaitLinkReady = liveConnectionIsReady(liveConnectionState) && adapter?.capabilities.gaitProfiles === true;
  liveGaitPreviewLab.setSettings(liveGaitState.draft.settings);
  document.querySelector("#live-gait-name").value = liveGaitState.draft.name;
  document.querySelector("#live-gait-profile-title").textContent = liveGaitState.draft.name.toUpperCase();
  document.querySelector("#live-gait-draft-state").textContent = liveGaitState.dirty
    ? "UNSAVED DRAFT"
    : `${liveGaitState.draft.source.toUpperCase()} PROFILE`;
  document.querySelector("#live-gait-preview-enabled").checked = liveGaitState.previewEnabled;
  document.querySelector("#live-gait-preview-forward").value = String(liveGaitState.previewForward);
  document.querySelector("#live-gait-preview-turn").value = String(liveGaitState.previewTurn);
  document.querySelector("#live-gait-preview-forward-value").textContent =
    `${liveGaitState.previewForward >= 0 ? "+" : ""}${liveGaitState.previewForward.toFixed(2)}`;
  document.querySelector("#live-gait-preview-turn-value").textContent =
    `${liveGaitState.previewTurn >= 0 ? "+" : ""}${liveGaitState.previewTurn.toFixed(2)}`;
  document.querySelector("#live-gait-preview-mode-label").textContent =
    `${liveGaitState.previewMode.toUpperCase()} / ${Math.round(Math.abs(liveGaitState.previewForward) * 100)}% ${liveGaitState.previewForward < 0 ? "REVERSE" : "FORWARD"}`;
  document.querySelectorAll("[data-live-gait-mode]").forEach((button) => {
    button.setAttribute("aria-pressed", String(button.dataset.liveGaitMode === liveGaitState.previewMode));
  });
  document.querySelectorAll("[data-live-gait-preset]").forEach((button) => {
    button.setAttribute("aria-pressed", String(button.dataset.liveGaitPreset === liveGaitState.draft.settings.preset));
  });
  const link = document.querySelector("#live-gait-link-state");
  link.textContent = !gaitLinkReady
    ? "GAIT LINK OFFLINE"
    : liveGaitState.robotProfile
    ? liveGaitState.persistentApplySupported ? "ROBOT PROFILE READY" : "ROBOT PROFILE READ ONLY"
    : "ROBOT PROFILE UNKNOWN";
  link.dataset.state = gaitLinkReady && liveGaitState.robotProfile ? "online" : "offline";
  document.querySelector("#live-gait-apply-state").textContent = liveGaitState.pendingRequestId
    ? `WAITING / ${liveGaitState.pendingAction.toUpperCase()}`
    : liveGaitCanApply(liveGaitState) ? "DISARMED / READY TO APPLY" : "PREVIEW ONLY";
  document.querySelector("#live-gait-status").textContent = liveGaitState.status;
  document.querySelector("#live-gait-apply").disabled = !gaitLinkReady || !liveGaitCanApply(liveGaitState);
  document.querySelector("#live-gait-use-robot").disabled = !gaitLinkReady || !liveGaitState.robotProfile || Boolean(liveGaitState.pendingRequestId);
  document.querySelector("#live-gait-revert").disabled = !gaitLinkReady || !liveGaitState.previousRobotProfile || !liveGaitCanApply(liveGaitState);
  document.querySelector("#live-gait-request-profile").disabled = !gaitLinkReady || Boolean(liveGaitState.pendingRequestId);
  syncLiveGaitLibrary(liveGaitState.selectedLibraryName);
  rebuildLiveGaitSettings();
  renderLiveGaitComparison();
}

function sendLiveGaitCommand(action, profileOverride = null) {
  const connection = liveConnectionEnvelope(liveConnectionState);
  if (
    socket?.readyState !== WebSocket.OPEN ||
    !connection ||
    !liveConnectionIsReady(liveConnectionState) ||
    liveGaitState.pendingRequestId
  ) return false;
  const requestId = crypto.randomUUID();
  const originalDraft = liveGaitState.draft;
  if (profileOverride) liveGaitState.draft = profileOverride;
  const command = createLiveGaitCommand(liveGaitState, action, requestId);
  liveGaitState.draft = originalDraft;
  if (!command) return false;
  Object.assign(command, connection);
  liveGaitState.pendingRequestId = requestId;
  liveGaitState.pendingAction = action;
  liveGaitState.status = "Waiting for the robot adapter acknowledgement...";
  socket.send(JSON.stringify(command));
  clearTimeout(liveGaitPendingTimeout);
  liveGaitPendingTimeout = setTimeout(() => {
    if (liveGaitState.pendingRequestId !== requestId) return;
    liveGaitState.pendingRequestId = "";
    liveGaitState.pendingAction = "";
    liveGaitState.status = "Robot acknowledgement timed out. No profile change was assumed.";
    renderLiveGaitUi();
  }, 3_000);
  renderLiveGaitUi();
  return true;
}

function acceptLiveGaitAcknowledgement(message) {
  const connection = liveConnectionEnvelope(liveConnectionState);
  if (
    !message ||
    message.type !== "live-gait-ack" ||
    !connection ||
    message.adapterId !== connection.adapterId ||
    message.sessionId !== connection.sessionId ||
    message.requestId !== liveGaitState.pendingRequestId ||
    message.action !== liveGaitState.pendingAction
  ) return false;
  clearTimeout(liveGaitPendingTimeout);
  liveGaitPendingTimeout = null;
  const action = liveGaitState.pendingAction;
  liveGaitState.pendingRequestId = "";
  liveGaitState.pendingAction = "";
  liveGaitState.robotState = String(message.robotState || liveGaitState.robotState).toLowerCase();
  liveGaitState.persistentApplySupported = message.supportsPersistentProfiles === true;
  if (message.profile) acceptRobotGaitProfile(liveGaitState, message.profile);
  if (message.accepted) {
    liveGaitState.status = action === "apply-profile"
      ? message.persisted === true
        ? "Robot confirmed the gait profile was validated and saved persistently."
        : "Robot accepted the profile but did not confirm persistent storage."
      : action === "revert-profile"
        ? "Robot confirmed the previous gait profile was restored."
        : "Robot profile and apply capabilities received.";
  } else {
    liveGaitState.status = `Robot rejected ${action}${message.reason ? `: ${message.reason}` : "."}`;
  }
  renderLiveGaitUi();
  return true;
}

function renderDiagnosticPipeline(snapshot) {
  const pipeline = document.querySelector("#live-diagnostics-pipeline");
  pipeline.replaceChildren();
  snapshot.stages.forEach((stage, index) => {
    const row = document.createElement("article");
    row.dataset.state = stage.status;
    const number = document.createElement("span");
    number.textContent = String(index + 1).padStart(2, "0");
    const status = document.createElement("i");
    status.setAttribute("aria-label", stage.status);
    const body = document.createElement("div");
    const label = document.createElement("strong");
    const detail = document.createElement("small");
    label.textContent = stage.label.toUpperCase();
    detail.textContent = stage.detail;
    body.append(label, detail);
    const rate = document.createElement("b");
    rate.textContent = Number.isFinite(stage.rateHz) ? `${stage.rateHz.toFixed(1)} Hz` : stage.status.toUpperCase();
    row.append(number, status, body, rate);
    pipeline.append(row);
  });
}

function renderDiagnosticEvents(snapshot) {
  const events = liveDiagnosticFilter === "all"
    ? snapshot.events
    : snapshot.events.filter((event) => event.severity === liveDiagnosticFilter);
  const list = document.querySelector("#live-diagnostics-events");
  list.replaceChildren();
  document.querySelector("#live-diagnostics-events-empty").hidden = events.length > 0;
  events.forEach((event) => {
    const row = document.createElement("article");
    row.dataset.severity = event.severity;
    const time = document.createElement("time");
    time.dateTime = new Date(event.timestampMs).toISOString();
    time.textContent = new Date(event.timestampMs).toLocaleTimeString();
    const severity = document.createElement("strong");
    severity.textContent = event.severity.toUpperCase();
    const source = document.createElement("span");
    source.textContent = event.source.toUpperCase();
    const message = document.createElement("p");
    message.textContent = event.message;
    row.append(time, severity, source, message);
    list.append(row);
  });
}

const liveControllerChannelNames = Object.freeze([
  "ROLL", "FORWARD", "HEIGHT", "TURN", "STAND", "AUX 1", "GAIT", "TILT",
  "CH 9", "CH 10", "CH 11", "CH 12", "CH 13", "CH 14", "CH 15", "CH 16",
]);

function renderLiveControllerDiagnostics(snapshot) {
  const telemetry = snapshot.telemetry;
  const quality = document.querySelector("#live-controller-quality");
  quality.textContent = snapshot.quality.toUpperCase();
  quality.dataset.state = snapshot.quality;
  document.querySelector("#live-controller-summary").textContent = !snapshot.fresh
    ? "NO FRESH RECEIVER TELEMETRY"
    : `${telemetry.transmitterName.toUpperCase()} / ${telemetry.receiverName.toUpperCase()}`;
  document.querySelector("#live-controller-frame-age").textContent = Number.isFinite(snapshot.frameAgeMs)
    ? `${Math.round(snapshot.frameAgeMs)} ms`
    : "-- ms";
  document.querySelector("#live-controller-rate").textContent = telemetry ? `${telemetry.packetRateHz.toFixed(1)} Hz` : "-- Hz";
  document.querySelector("#live-controller-lq").textContent = telemetry ? `${telemetry.linkQualityPercent.toFixed(0)} %` : "-- %";
  document.querySelector("#live-controller-rssi").textContent = telemetry
    ? `${telemetry.rssi1Dbm.toFixed(0)} / ${Number.isFinite(telemetry.rssi2Dbm) ? telemetry.rssi2Dbm.toFixed(0) : "--"} dBm`
    : "-- / -- dBm";
  document.querySelector("#live-controller-voltage").textContent = Number.isFinite(telemetry?.receiverVoltageV)
    ? `${telemetry.receiverVoltageV.toFixed(2)} V`
    : "-- V";
  document.querySelector("#live-controller-failsafe").textContent = telemetry ? telemetry.failsafe ? "ACTIVE" : "CLEAR" : "UNKNOWN";
  document.querySelector("#live-controller-snr").textContent = Number.isFinite(telemetry?.snrDb) ? `${telemetry.snrDb.toFixed(1)} dB` : "-- dB";
  document.querySelector("#live-controller-rf-mode").textContent = telemetry?.rfMode || "--";
  document.querySelector("#live-controller-tx-power").textContent = Number.isFinite(telemetry?.txPowerMw) ? `${telemetry.txPowerMw.toFixed(0)} mW` : "-- mW";
  document.querySelector("#live-controller-frame-loss").textContent = telemetry ? telemetry.frameLossCount.toLocaleString() : "--";
  document.querySelector("#live-controller-failsafe-count").textContent = telemetry ? telemetry.failsafeCount.toLocaleString() : "--";
  document.querySelector("#live-controller-antenna").textContent = telemetry?.activeAntenna ? `ANT ${telemetry.activeAntenna}` : "--";

  const channels = document.querySelector("#live-controller-channels");
  channels.replaceChildren();
  (telemetry?.channelsUs || Array(16).fill(null)).forEach((value, index) => {
    const row = document.createElement("div");
    row.className = "controller-channel";
    if (index >= 8) row.dataset.expertOnly = "";
    const label = document.createElement("span");
    label.textContent = liveControllerChannelNames[index];
    const bar = document.createElement("i");
    const level = Number.isFinite(value) ? Math.max(0, Math.min(100, (value - 1_000) / 10)) : 0;
    bar.style.setProperty("--channel-level", `${level}%`);
    const output = document.createElement("strong");
    output.textContent = Number.isFinite(value) ? value.toFixed(0) : "--";
    row.append(label, bar, output);
    channels.append(row);
  });

  const events = document.querySelector("#live-controller-events");
  events.replaceChildren();
  document.querySelector("#live-controller-events-empty").hidden = snapshot.events.length > 0;
  snapshot.events.slice(0, 12).forEach((event) => {
    const row = document.createElement("article");
    row.dataset.severity = event.severity;
    const time = document.createElement("time");
    time.textContent = new Date(event.timestampMs).toLocaleTimeString();
    const severity = document.createElement("strong");
    severity.textContent = event.severity.toUpperCase();
    const message = document.createElement("p");
    message.textContent = event.message;
    row.append(time, severity, message);
    events.append(row);
  });
}

function updateLiveDiagnosticsUi(snapshot, liveSnapshot, controllerSnapshot) {
  const health = document.querySelector("#live-diagnostics-health");
  const hasFault = snapshot.stages.some((stage) => stage.status === "fault");
  const controllerDegraded = snapshot.linkFresh && !controllerSnapshot.linkReady;
  const hasWarning = snapshot.stages.some((stage) => stage.status === "warning") || controllerDegraded;
  health.textContent = !snapshot.linkFresh ? "NO TELEMETRY" : hasFault ? "PIPELINE FAULT" : hasWarning ? "PIPELINE DEGRADED" : "PIPELINE HEALTHY";
  health.dataset.state = snapshot.linkFresh && !hasFault ? "online" : "offline";
  document.querySelector("#live-diagnostics-event-count").textContent = `${snapshot.events.length + controllerSnapshot.events.length} EVENTS`;
  document.querySelector("#live-diagnostics-pipeline-summary").textContent = !snapshot.linkFresh
    ? "WAITING FOR ROBOT DATA"
    : hasFault
      ? "FAULT FOUND"
      : hasWarning
        ? controllerDegraded ? "CONTROLLER LINK NOT READY" : "DEGRADED"
        : "ALL REPORTED STAGES HEALTHY";
  renderDiagnosticPipeline(snapshot);
  const firstBreak = document.querySelector("#live-diagnostics-first-break");
  firstBreak.dataset.state = snapshot.firstBrokenStage?.status || "ok";
  firstBreak.querySelector("strong").textContent = snapshot.firstBrokenStage
    ? snapshot.firstBrokenStage.label.toUpperCase()
    : snapshot.linkFresh ? "NO BROKEN STAGE" : "NOT ENOUGH DATA";
  firstBreak.querySelector("p").textContent = snapshot.firstBrokenStage
    ? snapshot.firstBrokenStage.detail
    : snapshot.linkFresh
      ? "Every stage reported by the robot is currently healthy."
      : "Connect the robot engineering stream to trace the control chain.";
  document.querySelector("#live-diagnostics-rate").textContent = `${snapshot.packetRateHz.toFixed(1)} Hz`;
  document.querySelector("#live-diagnostics-latency").textContent = Number.isFinite(snapshot.telemetry?.commandLatencyMs)
    ? `${snapshot.telemetry.commandLatencyMs.toFixed(1)} ms`
    : "-- ms";
  document.querySelector("#live-diagnostics-dropped").textContent = `${snapshot.droppedPackets} / ${snapshot.rejectedPackets}`;
  document.querySelector("#live-diagnostics-stale").textContent = String(snapshot.staleTransitions);
  document.querySelector("#live-diagnostics-loop").textContent = Number.isFinite(snapshot.telemetry?.esp32LoopHz)
    ? `${snapshot.telemetry.esp32LoopHz.toFixed(1)} Hz`
    : "-- Hz";
  document.querySelector("#live-diagnostics-uptime").textContent = formatDiagnosticUptime(snapshot.telemetry?.uptimeMs);
  document.querySelector("#live-diagnostics-robot-state").textContent = snapshot.telemetry?.robotState?.toUpperCase() || "UNKNOWN";
  document.querySelector("#live-diagnostics-sequence").textContent = snapshot.lastPacket
    ? String(snapshot.lastPacket.sequence)
    : "--";
  renderDiagnosticEvents(snapshot);
  document.querySelector("#live-diagnostics-last-packet").textContent = snapshot.lastPacket
    ? JSON.stringify(snapshot.lastPacket, null, 2)
    : "No packet received.";
  document.querySelector("#live-diagnostics-last-command").textContent = liveSnapshot.expected
    ? JSON.stringify({
        timestampMs: liveSnapshot.expected.timestampMs,
        body: liveSnapshot.expected.body,
        servoAngleDeg: liveSnapshot.expected.servoAngleDeg,
      }, null, 2)
    : "No expected command received.";
  renderLiveControllerDiagnostics(controllerSnapshot);
}

function updateLiveSessionUi(snapshot) {
  if (liveSessionState.status === "recording") {
    recordLiveComparisonSample(liveSessionState, snapshot);
  }
  const summary = liveSessionSummary(liveSessionState);
  const recording = summary.status === "recording";
  const recordingLabel = recording
    ? "STOP RECORDING"
    : summary.sampleCount > 0
      ? "NEW RECORDING"
      : "START RECORDING";
  ["#live-recording-toggle", "#live-data-recording-toggle"].forEach((selector) => {
    const recordingButton = document.querySelector(selector);
    recordingButton.disabled = !recording && !snapshot.paired;
    recordingButton.textContent = recordingLabel;
    recordingButton.dataset.state = recording ? "recording" : "idle";
  });
  ["#live-export-csv", "#live-data-export-csv"].forEach((selector) => {
    document.querySelector(selector).disabled = recording || summary.sampleCount === 0;
  });
  document.querySelector("#live-session-state").textContent = recording
    ? snapshot.paired
      ? `RECORDING / ${summary.sampleCount} / ${formatSessionDuration(summary.durationMs)}`
      : `RECORDING / SIGNAL LOST / ${summary.sampleCount}`
    : summary.sampleCount > 0
      ? `STOPPED / ${summary.sampleCount} / ${formatSessionDuration(summary.durationMs)}`
      : snapshot.paired
        ? "READY TO RECORD"
        : "NO ACTIVE SESSION";
  document.querySelector("#live-data-recorder-state").textContent = recording
    ? snapshot.paired
      ? `RECORDING / ${formatSessionDuration(summary.durationMs)}`
      : "RECORDING / WAITING FOR SIGNAL"
    : summary.sampleCount > 0
      ? `STOPPED / ${formatSessionDuration(summary.durationMs)}`
      : snapshot.paired
        ? "READY TO RECORD"
        : "WAITING FOR BOTH STREAMS";
  document.querySelector("#live-data-sample-count").textContent = `${summary.sampleCount.toLocaleString()} SAMPLES`;
  document.querySelector("#live-chart-empty").textContent = recording && !snapshot.paired
    ? "Telemetry interrupted. Recording will resume when both streams return."
    : summary.sampleCount > 0
      ? "Waiting for another synchronized sample."
      : "Connect the engineering link, then start recording.";
  document.querySelector("#live-data-chart-empty").textContent = recording && !snapshot.paired
    ? "Telemetry interrupted. Recording will resume automatically."
    : summary.sampleCount > 0
      ? "Waiting for another synchronized sample."
      : "Start recording to populate this graph.";
  renderLiveDataTable();
  renderLiveSessionArchive();
  renderLiveComparisonChart();
}

function updateLiveComparisonUi() {
  if (pruneLiveAdapters(liveConnectionState)) {
    lockLiveSafetyState(liveSafetyState, "The adapter heartbeat was lost. Robot-side outputs must fail safe.");
    resetLiveCommandPermissions("The adapter heartbeat was lost. Robot profile changes remain blocked.");
    renderLiveConnectionUi();
  }
  const snapshot = liveComparisonSnapshot(liveTelemetryState);
  const sessionConnected = liveConnectionIsReady(liveConnectionState);
  const connectionAdapter = liveConnectionState.adapters[liveConnectionState.selectedAdapterId];
  const calibrationSupported = sessionConnected && connectionAdapter?.capabilities.calibration === true;
  const engineeringConnected = sessionConnected && snapshot.lastRobotPacketAgeMs !== null &&
    snapshot.lastRobotPacketAgeMs <= 1_000;
  const engineeringOutput = document.querySelector("#live-engineering-link");
  engineeringOutput.textContent = engineeringConnected ? "CONNECTED" : "OFFLINE";
  engineeringOutput.dataset.state = engineeringConnected ? "connected" : "disconnected";
  const dataLinkOutput = document.querySelector("#live-data-link-state");
  dataLinkOutput.textContent = engineeringConnected ? "ENGINEERING CONNECTED" : "ENGINEERING OFFLINE";
  dataLinkOutput.dataset.state = engineeringConnected ? "online" : "offline";
  const calibrationLinkOutput = document.querySelector("#live-calibration-link-state");
  calibrationLinkOutput.textContent = liveCalibrationState.benchModeAcknowledged
    ? "BENCH MODE READY"
    : engineeringConnected
      ? "LINKED / BENCH LOCKED"
      : "ROBOT NOT READY";
  calibrationLinkOutput.dataset.state = liveCalibrationState.benchModeAcknowledged ? "online" : "offline";
  document.querySelector("#live-calibration-apply-robot").disabled =
    !engineeringConnected || !calibrationSupported || !liveCalibrationState.benchModeAcknowledged || Boolean(calibrationPendingRequestId);
  const benchRequest = document.querySelector("#live-calibration-request-bench");
  benchRequest.disabled =
    !engineeringConnected ||
    !calibrationSupported ||
    !liveCalibrationState.safetyConfirmed ||
    liveCalibrationState.benchModeAcknowledged ||
    Boolean(calibrationPendingRequestId);
  benchRequest.textContent = calibrationPendingAction === "enter"
    ? "WAITING FOR ROBOT"
    : liveCalibrationState.benchModeAcknowledged
      ? "BENCH MODE READY"
      : "REQUEST BENCH MODE";
  document.querySelector("#live-calibration-bench-state").textContent =
    liveCalibrationState.benchModeAcknowledged
      ? "UNLOCKED - ROBOT SAFETY ACKNOWLEDGED"
      : calibrationPendingAction === "enter"
        ? "LOCKED - WAITING FOR ROBOT ACKNOWLEDGEMENT"
        : "LOCKED - BENCH MODE NOT ACKNOWLEDGED";
  document.querySelector("#live-calibration-jog-status").textContent =
    liveCalibrationState.benchModeAcknowledged
      ? "Robot bench mode is active. Jog requests remain limited to ±10° and 5°/s."
      : "Preview only. Connect a compatible robot adapter and receive a bench-mode acknowledgement to unlock physical movement.";
  document.querySelector("#live-data-current-time").textContent = engineeringConnected
    ? `PACKET ${formatPacketAge(snapshot.lastRobotPacketAgeMs)} AGO`
    : "NO TELEMETRY";
  document.querySelector("#live-packet-age").textContent = formatPacketAge(
    snapshot.lastRobotPacketAgeMs,
  );
  const engineeringBadge = document.querySelector("#real-engineering-status");
  engineeringBadge.dataset.state = engineeringConnected ? "online" : "offline";
  engineeringBadge.textContent = engineeringConnected ? "ENGINEERING" : "ENGINEERING";
  engineeringBadge.title = !sessionConnected
    ? "No engineering session"
    : engineeringConnected
      ? `Session ${liveConnectionState.sessionId.slice(0, 12)} / telemetry ${formatPacketAge(snapshot.lastRobotPacketAgeMs)} old`
      : "Engineering session connected but telemetry is stale";

  const controllerSnapshot = liveControllerSnapshot(liveControllerState);
  const driveConnected = engineeringConnected && controllerSnapshot.linkReady;
  const driveOutput = document.querySelector("#live-drive-link");
  driveOutput.textContent = driveConnected ? "CONNECTED" : "OFFLINE";
  driveOutput.dataset.state = driveConnected ? "connected" : "disconnected";
  const driveBadge = document.querySelector("#real-drive-status");
  driveBadge.dataset.state = driveConnected ? "online" : "offline";
  driveBadge.title = driveConnected
    ? `Robot reports ${controllerSnapshot.telemetry.packetRateHz.toFixed(0)} Hz Boxer / ELRS frames, ${controllerSnapshot.telemetry.linkQualityPercent.toFixed(0)}% link quality`
    : "Robot has not reported a fresh, failsafe-clear Boxer / ELRS control frame";

  const robotState = sessionConnected ? liveSafetyState.robotState : "disconnected";
  document.querySelector("#live-robot-safety-state").textContent = robotState.toUpperCase();
  document.querySelector("#live-robot-safety-copy").textContent = !sessionConnected
    ? "No physical engineering session is active. Commands are blocked and the CAD is a reference model only."
    : robotState === "disarmed"
      ? "The robot reports disarmed. Telemetry is available; motion remains blocked until separately armed."
      : robotState === "armed"
        ? liveManualState.authorityToken
          ? "The robot reports armed. Guarded browser authority is active; motion streams only while its deadman is held."
          : "The robot reports armed. Guarded browser control still requires an explicit, time-limited authority lease."
        : `The robot reports ${robotState}. Physical commands remain blocked.`;
  const safetyBadge = document.querySelector("#real-safety-status");
  safetyBadge.textContent = robotState.toUpperCase();
  safetyBadge.dataset.state = robotState === "disarmed" || robotState === "disconnected" ? "safe" : "offline";
  renderLiveSafetyUi();

  setLiveStreamState("#live-expected-state", snapshot.expectedFresh);
  setLiveStreamState("#live-measured-state", snapshot.measuredFresh);
  document.querySelector("#live-time-alignment").textContent = snapshot.alignmentMs === null
    ? "-- ms"
    : `${snapshot.alignmentMs >= 0 ? "+" : ""}${Math.round(snapshot.alignmentMs)} ms`;
  document.querySelector("#live-data-alignment").textContent = snapshot.alignmentMs === null
    ? "-- ms"
    : `${snapshot.alignmentMs >= 0 ? "+" : ""}${Math.round(snapshot.alignmentMs)} ms`;
  document.querySelector("#live-worst-joint-error").textContent =
    formatLiveAngle(snapshot.worstJointErrorDeg, "--°");

  document.querySelector("#live-data-joint-error").textContent =
    formatLiveAngle(snapshot.worstJointErrorDeg, "--°");

  document.querySelectorAll("[data-live-body]").forEach((output) => {
    const stream = snapshot[output.dataset.stream];
    const field = output.dataset.liveBody;
    output.textContent = field === "heightMm"
      ? formatLiveHeight(stream?.body?.[field])
      : formatLiveAngle(stream?.body?.[field]);
  });
  document.querySelectorAll("[data-live-body-error]").forEach((output) => {
    const field = output.dataset.liveBodyError;
    output.textContent = field === "heightMm"
      ? Number.isFinite(snapshot.bodyError?.[field])
        ? `${snapshot.bodyError[field] >= 0 ? "+" : ""}${Math.round(snapshot.bodyError[field])} mm`
        : "-- mm"
      : formatLiveAngle(snapshot.bodyError?.[field]);
  });
  document.querySelectorAll("[data-live-joint-error]").forEach((output) => {
    const channel = Number(output.dataset.liveJointError);
    output.textContent = formatLiveAngle(snapshot.jointErrorsDeg[channel]);
    const magnitude = Math.abs(snapshot.jointErrorsDeg[channel]);
    output.dataset.state = !Number.isFinite(magnitude)
      ? "unavailable"
      : magnitude >= 8
        ? "fault"
        : magnitude >= 3
          ? "warning"
          : "ok";
  });

  document.querySelector("#live-voltage").textContent = snapshot.power
    ? `${snapshot.power.voltageV.toFixed(2)} V`
    : "--.- V";
  document.querySelector("#live-data-voltage").textContent = snapshot.power
    ? `${snapshot.power.voltageV.toFixed(2)} V`
    : "--.- V";
  document.querySelector("#live-current").textContent = snapshot.power
    ? `${snapshot.power.currentA.toFixed(2)} A`
    : "--.- A";
  document.querySelector("#live-power").textContent = snapshot.power
    ? `${snapshot.power.powerW.toFixed(1)} W`
    : "--- W";
  document.querySelector("#live-data-power").textContent = snapshot.power
    ? `${snapshot.power.powerW.toFixed(1)} W`
    : "--- W";
  const diagnosticSnapshot = liveDiagnosticsSnapshot(liveDiagnosticsState, snapshot);
  if (liveViewState.selected === LIVE_VIEW_DIAGNOSTICS) {
    updateLiveDiagnosticsUi(diagnosticSnapshot, snapshot, liveControllerSnapshot(liveControllerState));
  }
  updateLiveSessionUi(snapshot);
}

function toggleLiveRecording() {
  const snapshot = liveComparisonSnapshot(liveTelemetryState);
  if (liveSessionState.status === "recording") {
    stopLiveSession(liveSessionState);
    archiveLiveSession(liveSessionArchive, liveSessionState, `live-${liveSessionState.stoppedAt}`);
  } else if (snapshot.paired) {
    startLiveSession(liveSessionState);
  }
  updateLiveComparisonUi();
}

document.querySelector("#live-recording-toggle").addEventListener("click", toggleLiveRecording);
document.querySelector("#live-data-recording-toggle").addEventListener("click", toggleLiveRecording);

liveChartSignal.addEventListener("change", () => {
  const definition = liveChartDefinitions[liveChartSignal.value] || liveChartDefinitions.pitchDeg;
  document.querySelector("#live-chart-title").textContent = definition.title;
  renderLiveComparisonChart();
});

liveDataChartSignal.addEventListener("change", () => {
  const definition = liveChartDefinitions[liveDataChartSignal.value] || liveChartDefinitions.pitchDeg;
  document.querySelector("#live-data-chart-title").textContent = definition.title;
  renderLiveComparisonChart();
});

document.querySelector("#live-export-csv").addEventListener("click", () => {
  downloadLiveSessionCsv(liveSessionState);
});
document.querySelector("#live-data-export-csv").addEventListener("click", () => {
  downloadLiveSessionCsv(liveSessionState);
});
document.querySelector("#live-sessions-open-data").addEventListener("click", () => {
  applyLiveView(LIVE_VIEW_DATA);
});

document.querySelectorAll("[data-calibration-step]").forEach((button) => {
  button.addEventListener("click", () => {
    selectCalibrationStep(liveCalibrationState, button.dataset.calibrationStep);
    renderLiveCalibrationUi();
  });
});
document.querySelector("#live-calibration-safety-confirm").addEventListener("change", (event) => {
  liveCalibrationState.safetyConfirmed = event.target.checked;
  renderLiveCalibrationUi();
});
document.querySelector("#live-calibration-request-bench").addEventListener("click", () => {
  if (!liveCalibrationState.safetyConfirmed || !sendCalibrationCommand("enter")) return;
  updateLiveComparisonUi();
});
document.querySelector("#live-calibration-joint-list").addEventListener("click", (event) => {
  const button = event.target.closest("[data-calibration-channel]");
  if (!button) return;
  selectCalibrationJoint(liveCalibrationState, button.dataset.calibrationChannel);
  renderLiveCalibrationUi();
});
document.querySelector("#live-calibration-offset").addEventListener("change", (event) => {
  updateCalibrationJoint(liveCalibrationState, { offsetDeg: event.target.value });
  renderLiveCalibrationUi();
});
document.querySelector("#live-calibration-direction").addEventListener("change", (event) => {
  updateCalibrationJoint(liveCalibrationState, { direction: event.target.value });
  renderLiveCalibrationUi();
});
document.querySelector("#live-calibration-preview").addEventListener("change", (event) => {
  liveCalibrationState.previewEnabled = event.target.checked;
  renderLiveCalibrationUi();
});
document.querySelectorAll("[data-calibration-jog]").forEach((button) => {
  button.addEventListener("click", () => {
    jogCalibrationJoint(liveCalibrationState, button.dataset.calibrationJog);
    if (liveCalibrationState.benchModeAcknowledged) sendCalibrationCommand("jog");
    renderLiveCalibrationUi();
  });
});
document.querySelector("[data-calibration-jog-reset]").addEventListener("click", () => {
  liveCalibrationState.jogOffsetDeg = 0;
  if (liveCalibrationState.benchModeAcknowledged) sendCalibrationCommand("jog");
  renderLiveCalibrationUi();
});
document.querySelector("#live-calibration-minimum").addEventListener("change", (event) => {
  updateCalibrationJoint(liveCalibrationState, { minimumDeg: event.target.value });
  renderLiveCalibrationUi();
});
document.querySelector("#live-calibration-maximum").addEventListener("change", (event) => {
  updateCalibrationJoint(liveCalibrationState, { maximumDeg: event.target.value });
  renderLiveCalibrationUi();
});
document.querySelector("#live-calibration-back").addEventListener("click", () => {
  selectCalibrationStep(liveCalibrationState, liveCalibrationState.step - 1);
  renderLiveCalibrationUi();
});
document.querySelector("#live-calibration-next").addEventListener("click", () => {
  if (liveCalibrationState.step === LIVE_CALIBRATION_STEPS.length - 1) {
    applyLiveView(LIVE_VIEW_COMPARE);
    return;
  }
  selectCalibrationStep(liveCalibrationState, liveCalibrationState.step + 1);
  renderLiveCalibrationUi();
});
document.querySelector("#live-calibration-save-browser").addEventListener("click", () => {
  liveCalibrationState.profile.savedAt = Date.now();
  localStorage.setItem(
    LIVE_CALIBRATION_STORAGE_KEY,
    calibrationProfileJson(liveCalibrationState.profile),
  );
  liveCalibrationState.dirty = false;
  document.querySelector("#live-calibration-review-status").textContent =
    "Browser copy saved. Export JSON as the portable backup for this physical robot.";
  renderLiveCalibrationUi();
});
document.querySelector("#live-calibration-export").addEventListener("click", downloadCalibrationJson);
document.querySelector("#live-calibration-apply-robot").addEventListener("click", () => {
  if (!liveCalibrationState.benchModeAcknowledged || !sendCalibrationCommand("save-profile")) return;
  document.querySelector("#live-calibration-review-status").textContent =
    "Waiting for the robot to confirm persistent calibration storage...";
  updateLiveComparisonUi();
});
document.querySelector("#live-calibration-import").addEventListener("click", () => {
  document.querySelector("#live-calibration-import-file").click();
});
document.querySelector("#live-calibration-import-file").addEventListener("change", async (event) => {
  const [file] = event.target.files || [];
  if (!file) return;
  const status = document.querySelector("#live-calibration-review-status");
  try {
    liveCalibrationState.profile = parseCalibrationProfileJson(await file.text());
    liveCalibrationState.dirty = true;
    liveCalibrationState.jogOffsetDeg = 0;
    status.textContent = `Imported ${file.name}. Review every joint before saving or applying.`;
  } catch (error) {
    status.textContent = error instanceof Error ? error.message : "Calibration import failed.";
  }
  event.target.value = "";
  renderLiveCalibrationUi();
});

document.querySelector("#live-gait-library").addEventListener("change", (event) => {
  liveGaitState.selectedLibraryName = event.target.value;
  document.querySelector("#live-gait-load").disabled = !event.target.value;
});
document.querySelector("#live-gait-load").addEventListener("click", () => {
  const profile = liveGaitLibrary[liveGaitState.selectedLibraryName];
  if (!profile) return;
  replaceLiveGaitDraft(liveGaitState, profile, profile.source);
  liveGaitPreviewLab.reset();
  liveGaitState.status = `Opened ${profile.name} from the shared Simulation/LIVE library.`;
  renderLiveGaitUi();
});
document.querySelector("#live-gait-name").addEventListener("change", (event) => {
  const name = event.target.value.trim().slice(0, 32) || "Untitled gait";
  liveGaitState.draft.name = name;
  liveGaitState.dirty = true;
  renderLiveGaitUi();
});
document.querySelector("#live-gait-save").addEventListener("click", () => {
  const name = document.querySelector("#live-gait-name").value.trim().slice(0, 32) || "Untitled gait";
  liveGaitState.draft = createLiveGaitProfile({
    ...liveGaitState.draft,
    name,
    updatedAt: Date.now(),
    source: "live",
  }, name);
  liveGaitState.dirty = false;
  liveGaitLibrary[name] = liveGaitState.draft;
  liveGaitState.selectedLibraryName = name;
  persistLiveGaitLibrary(name);
  liveGaitState.status = `${name} saved to the shared Simulation/LIVE profile library.`;
  renderLiveGaitUi();
});
document.querySelector("#live-gait-import").addEventListener("click", () => {
  document.querySelector("#live-gait-import-file").click();
});
document.querySelector("#live-gait-import-file").addEventListener("change", async (event) => {
  const [file] = event.target.files || [];
  if (!file) return;
  try {
    const profile = parseLiveGaitProfileJson(await file.text());
    replaceLiveGaitDraft(liveGaitState, profile, "import");
    liveGaitPreviewLab.reset();
    liveGaitState.status = `Imported ${file.name}. Preview and review the profile before applying.`;
  } catch (error) {
    liveGaitState.status = error instanceof Error ? error.message : "Gait import failed.";
  }
  event.target.value = "";
  renderLiveGaitUi();
});
document.querySelector("#live-gait-export").addEventListener("click", () => {
  const blob = new Blob([liveGaitProfileJson(liveGaitState.draft)], { type: "application/json;charset=utf-8" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `domino-gait-${liveGaitState.draft.name.toLowerCase().replace(/[^a-z0-9]+/g, "-")}.json`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 0);
});
document.querySelector("#live-gait-settings").addEventListener("input", (event) => {
  const input = event.target.closest("input[type=range]");
  const setting = input?.closest("[data-gait-key]");
  if (!input || !setting) return;
  updateLiveGaitDraft(liveGaitState, { [setting.dataset.gaitKey]: input.value });
  liveGaitPreviewLab.setSettings(liveGaitState.draft.settings);
  const control = gaitLabControls.find((candidate) => candidate.key === setting.dataset.gaitKey);
  setting.querySelector("output").textContent = formatGaitSetting(control, liveGaitState.draft.settings[control.key]);
  document.querySelector("#live-gait-draft-state").textContent = "UNSAVED DRAFT";
  renderLiveGaitComparison();
});
document.querySelectorAll("[data-live-gait-preset]").forEach((button) => {
  button.addEventListener("click", () => {
    selectLiveGaitPreset(liveGaitState, button.dataset.liveGaitPreset);
    liveGaitState.draft.name = button.dataset.liveGaitPreset[0].toUpperCase() + button.dataset.liveGaitPreset.slice(1);
    liveGaitPreviewLab.reset();
    renderLiveGaitUi();
  });
});
document.querySelector("#live-gait-preview-enabled").addEventListener("change", (event) => {
  liveGaitState.previewEnabled = event.target.checked;
  liveGaitPreviewLab.reset();
});
document.querySelectorAll("[data-live-gait-mode]").forEach((button) => {
  button.addEventListener("click", () => {
    liveGaitState.previewMode = button.dataset.liveGaitMode;
    liveGaitPreviewLab.reset();
    renderLiveGaitUi();
  });
});
document.querySelector("#live-gait-preview-forward").addEventListener("input", (event) => {
  liveGaitState.previewForward = Number(event.target.value);
  document.querySelector("#live-gait-preview-forward-value").textContent =
    `${liveGaitState.previewForward >= 0 ? "+" : ""}${liveGaitState.previewForward.toFixed(2)}`;
});
document.querySelector("#live-gait-preview-turn").addEventListener("input", (event) => {
  liveGaitState.previewTurn = Number(event.target.value);
  document.querySelector("#live-gait-preview-turn-value").textContent =
    `${liveGaitState.previewTurn >= 0 ? "+" : ""}${liveGaitState.previewTurn.toFixed(2)}`;
});
document.querySelector("#live-gait-request-profile").addEventListener("click", () => sendLiveGaitCommand("request-profile"));
document.querySelector("#live-gait-use-robot").addEventListener("click", () => {
  if (!liveGaitState.robotProfile) return;
  replaceLiveGaitDraft(liveGaitState, liveGaitState.robotProfile, "robot");
  liveGaitState.status = "Robot profile loaded into the local draft for preview. Nothing was sent.";
  liveGaitPreviewLab.reset();
  renderLiveGaitUi();
});
document.querySelector("#live-gait-apply").addEventListener("click", () => {
  if (liveGaitCanApply(liveGaitState)) sendLiveGaitCommand("apply-profile");
});
document.querySelector("#live-gait-revert").addEventListener("click", () => {
  if (liveGaitCanApply(liveGaitState) && liveGaitState.previousRobotProfile) sendLiveGaitCommand("revert-profile");
});

document.querySelectorAll("[data-diagnostic-filter]").forEach((button) => {
  button.addEventListener("click", () => {
    liveDiagnosticFilter = button.dataset.diagnosticFilter;
    document.querySelectorAll("[data-diagnostic-filter]").forEach((candidate) => {
      candidate.setAttribute("aria-pressed", String(candidate === button));
    });
    updateLiveComparisonUi();
  });
});
document.querySelector("#live-diagnostics-clear").addEventListener("click", () => {
  liveDiagnosticsState.events = [];
  liveControllerState.events = [];
  updateLiveComparisonUi();
});
document.querySelector("#live-diagnostics-export").addEventListener("click", () => {
  const liveSnapshot = liveComparisonSnapshot(liveTelemetryState);
  const bundle = liveDiagnosticBundle(liveDiagnosticsState, liveSnapshot, {
    workspace: applicationState.workspace,
    experience: applicationState.experience,
    location: window.location.href,
    connection: {
      adapterId: liveConnectionState.selectedAdapterId,
      sessionId: liveConnectionState.sessionId,
      phase: liveConnectionState.phase,
    },
    safety: {
      robotState: liveSafetyState.robotState,
      watchdogTripped: liveSafetyState.watchdogTripped,
      watchdogRemainingMs: liveSafetyState.watchdogRemainingMs,
    },
    controller: liveControllerDiagnosticExport(liveControllerState),
    calibration: createLiveCalibrationProfile(liveCalibrationState.profile),
    activeSession: liveSessionSummary(liveSessionState),
  });
  const blob = new Blob([`${JSON.stringify(bundle, null, 2)}\n`], { type: "application/json;charset=utf-8" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `domino-diagnostic-bundle-${new Date().toISOString().replace(/[:.]/g, "-")}.json`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 0);
});

const calibrationRaycaster = new THREE.Raycaster();
const calibrationPointer = new THREE.Vector2();
canvas.addEventListener("dblclick", (event) => {
  if (liveViewState.selected !== LIVE_VIEW_CALIBRATION || event.button !== 0) return;
  const bounds = canvas.getBoundingClientRect();
  calibrationPointer.set(
    ((event.clientX - bounds.left) / bounds.width) * 2 - 1,
    -((event.clientY - bounds.top) / bounds.height) * 2 + 1,
  );
  calibrationRaycaster.setFromCamera(calibrationPointer, camera);
  const selected = calibrationRaycaster.intersectObject(cadRoot, true).find((intersection) => {
    let object = intersection.object;
    while (object && object !== cadRoot) {
      if (Number.isInteger(object.userData.calibrationChannel)) return true;
      object = object.parent;
    }
    return false;
  });
  if (!selected) return;
  let object = selected.object;
  while (object && object !== cadRoot && !Number.isInteger(object.userData.calibrationChannel)) {
    object = object.parent;
  }
  if (!Number.isInteger(object?.userData.calibrationChannel)) return;
  selectCalibrationJoint(liveCalibrationState, object.userData.calibrationChannel);
  selectCalibrationStep(liveCalibrationState, "neutral");
  renderLiveCalibrationUi();
});

window.addEventListener("resize", renderLiveComparisonChart);

window.dominoLiveTelemetry = Object.freeze({
  ingest(packet) {
    const accepted = ingestLiveTelemetry(packet);
    updateLiveComparisonUi();
    return accepted;
  },
  snapshot() {
    return liveComparisonSnapshot(liveTelemetryState);
  },
});
window.addEventListener("domino-live-telemetry", (event) => {
  if (event instanceof CustomEvent) {
    ingestLiveTelemetry(event.detail);
    updateLiveComparisonUi();
  }
});

setInterval(updateLiveComparisonUi, 100);
updateLiveComparisonUi();

function sendChannels() {
  if (socket?.readyState === WebSocket.OPEN) {
    const physicsSummary = physicsState
      ? {
          engine: physicsState.engine,
          bodyHeight: physicsState.bodyClearance ?? physicsState.bodyHeight,
          contactCount: physicsState.contactCount,
          footContacts: physicsState.footContacts,
          resetCount: physicsState.resetCount,
        }
      : null;
    socket.send(JSON.stringify({
      type: "control",
      channels,
      mode: demoMode ? "demo" : "interactive",
      active: simulationCanOwnControl(applicationState, document.visibilityState),
      claimControl: performance.now() < controlClaimUntil,
      manualOverride:
        manualStandOverride !== null ||
        manualTiltOverride !== null ||
        manualWalkModeOverride !== null,
      clientInput: clientInputSnapshot,
      physics: physicsSummary,
    }));
  }
}
setInterval(sendChannels, 25);

function requestStand(nextStand) {
  manualStandOverride = Boolean(nextStand);
  standRequested = Boolean(nextStand);
  if (!standRequested) requestTilt(false);
}

function requestTilt(nextTilt) {
  manualTiltOverride = Boolean(nextTilt);
  tiltRequested = Boolean(nextTilt);
  if (tiltRequested) {
    manualStandOverride = true;
    standRequested = true;
  }
}

function requestWalkMode(nextMode) {
  walkModeRequested = THREE.MathUtils.clamp(Number(nextMode) || 0, 0, 2);
  manualWalkModeOverride = walkModeRequested;
}

const keys = new Set();
window.addEventListener("keydown", (event) => {
  keys.add(event.code);
  if (event.repeat) return;
  if (event.code === "Space") {
    event.preventDefault();
    requestStand(!standRequested);
  }
  if (event.code === "KeyT") requestTilt(!tiltRequested);
  if (event.code === "KeyG") {
    requestWalkMode((walkModeRequested + 1) % 3);
  }
  if (event.code === "KeyR") resetRobot();
});
window.addEventListener("keyup", (event) => keys.delete(event.code));

function resetRobot() {
  physics?.reset();
  gaitLab.reset();
  clearFootTrajectories();
  cadAlignment.position.set(0, 0, 0);
  cadAlignment.rotation.set(0, 0, 0);
  robotYaw = 0;
  standRequested = false;
  tiltRequested = false;
  manualStandOverride = null;
  manualTiltOverride = null;
  manualWalkModeOverride = null;
  walkModeRequested = 0;
  channels[RIDE_HEIGHT_CHANNEL_INDEX] = 2000;
  channels[4] = 1000;
  channels[6] = 1000;
  channels[7] = 1000;
  observedPhysicalStand = null;
  observedPhysicalTilt = null;
  observedPhysicalWalkMode = null;
  if (floatModeEnabled) {
    floatAnchorPosition.set(0, floatAnchorPosition.y, 0);
    floatAnchorQuaternion.identity();
    visualBasePosition.copy(floatAnchorPosition);
    visualBaseQuaternion.copy(floatAnchorQuaternion);
    centerCameraOnRobot(floatAnchorPosition);
  }
}

function centerCameraOnRobot(position) {
  cameraSnap = null;
  cameraTargetOffset.set(0, 0, 0);
  cameraRecenterDelta.copy(position).sub(controls.target);
  controls.target.copy(position);
  camera.position.add(cameraRecenterDelta);
  camera.lookAt(controls.target);
  controls.update();
}

function setFloatMode(enabled) {
  const nextEnabled = Boolean(enabled);
  if (floatModeEnabled === nextEnabled) return;
  floatModeEnabled = nextEnabled;
  clearFootTrajectories();
  if (floatModeEnabled) {
    // Capture the exact visible pose. Float mode is an inspection constraint,
    // not a reset, so entering it must not lift or level the robot.
    floatAnchorPosition.copy(robotWorld.position);
    floatAnchorQuaternion.copy(robotWorld.quaternion);
    visualBasePosition.copy(floatAnchorPosition);
    visualBaseQuaternion.copy(floatAnchorQuaternion);
    visualBaseInitialized = true;
    centerCameraOnRobot(floatAnchorPosition);
  } else {
    cadAlignment.position.set(0, 0, 0);
    cadAlignment.rotation.set(0, 0, 0);
    physics?.reset();
    visualBaseInitialized = false;
    visualPhysicsResetCount = -1;
  }
  courseVisuals.visible = !floatModeEnabled;
  const button = document.querySelector("#float-button");
  button.classList.toggle("active", floatModeEnabled);
  button.setAttribute("aria-pressed", String(floatModeEnabled));
  button.textContent = floatModeEnabled ? "FLOAT ON" : "FLOAT";
  button.title = floatModeEnabled
    ? "Restore ground physics"
    : "Suspend Domino and disable ground physics";
  canvas.dataset.floatMode = String(floatModeEnabled);
}

function resetCameraView() {
  cameraSnap = null;
  controls.enabled = true;
  usePerspectiveCamera();
  cameraTargetOffset.set(0, 0, 0);
  robotCameraAnchor.set(
    robotWorld.position.x,
    floatModeEnabled ? robotWorld.position.y : 0.32,
    robotWorld.position.z,
  );
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
    Object.entries(runtime.annotations).forEach(([annotationName, annotation]) => {
      annotation.group.visible =
        jointOverlayVisible &&
        (selectedJointLeg === "ALL" || runtime.spec.label === selectedJointLeg);
      const selectedAnnotation = ACTIVE_ANNOTATION_BY_CHANNEL[selectedDriveJoint];
      const isSelected =
        annotation.active &&
        selectedJointLeg !== "ALL" &&
        runtime.spec.label === selectedJointLeg &&
        annotationName === selectedAnnotation;
      annotation.marker.material.opacity = annotation.active
        ? isSelected ? jointOverlayOpacity : jointOverlayOpacity * 0.28
        : jointOverlayOpacity * 0.42;
      if (annotation.sweep) {
        annotation.sweep.visible = isSelected;
        annotation.sweep.material.opacity = jointOverlayOpacity * 0.72;
      }
      if (annotation.indicator) {
        annotation.indicator.visible = isSelected;
        annotation.indicator.material.opacity = jointOverlayOpacity;
      }
      if (annotation.leader) {
        annotation.leader.visible = isSelected;
      }
      if (annotation.callout) {
        annotation.callout.sprite.visible = isSelected;
        annotation.callout.sprite.material.opacity = Math.max(0.42, jointOverlayOpacity);
      }
    });
  });
  const button = document.querySelector("#joints-button");
  const control = button.closest(".inspection-control");
  button.classList.toggle("active", jointOverlayVisible);
  button.setAttribute("aria-pressed", String(jointOverlayVisible));
  button.textContent = jointOverlayVisible ? "INSPECT ON" : "INSPECT";
  control.classList.toggle("active", jointOverlayVisible);
  document.querySelector("#joint-legend").hidden = !jointOverlayVisible;
  inspectorPanelController?.setActive(jointOverlayVisible);
  if (jointOverlayVisible) inspectorPanelController?.clamp();
  document.querySelector("#joint-legend-leg").textContent = selectedJointLeg;
  document.querySelectorAll("[data-drive-joint]").forEach((driveButton) => {
    const selected = driveButton.dataset.driveJoint === selectedDriveJoint;
    driveButton.classList.toggle("active", selected);
    driveButton.setAttribute("aria-pressed", String(selected));
  });
  if (bodyReferenceOverlay) {
    bodyReferenceOverlay.group.visible = jointOverlayVisible;
    bodyReferenceOverlay.plane.material.opacity = jointOverlayOpacity * 0.52;
    bodyReferenceOverlay.axes.forEach(({ arrow }) => {
      arrow.line.material.opacity = jointOverlayOpacity;
      arrow.cone.material.opacity = jointOverlayOpacity;
    });
  }
}

function updateMotionIndicator(annotation, deltaDegrees) {
  if (!annotation?.indicator) return;
  const endpoint = motionPlanePoint(
    annotation.axisDirection,
    THREE.MathUtils.degToRad(THREE.MathUtils.clamp(deltaDegrees, -45, 45)),
    0.016,
  );
  const positions = annotation.indicator.geometry.attributes.position;
  positions.setXYZ(0, 0, 0, 0);
  positions.setXYZ(1, endpoint.x, endpoint.y, endpoint.z);
  positions.needsUpdate = true;
}

function updateJointLegendValues() {
  if (!jointOverlayVisible) return;
  const displayState = effectiveFirmwareState || firmwareState;
  const selectedLeg = legs.find((leg) => leg.label === selectedJointLeg);
  const values = ["#joint-q1-value", "#joint-q2-value", "#joint-q3-value"];
  const footTarget = document.querySelector("#joint-foot-target");
  const footState = document.querySelector("#joint-foot-state");
  if (!displayState || !selectedLeg) {
    values.forEach((selector) => {
      document.querySelector(selector).textContent = selectedJointLeg === "ALL" ? "MULTI" : "--";
    });
    footTarget.textContent = selectedJointLeg === "ALL" ? "MULTI" : "--";
    footState.textContent = selectedJointLeg === "ALL" ? "MULTI" : "--";
    footState.dataset.state = "unknown";
    return;
  }
  const selectedRuntime = linkageRuntimes.find(
    (runtime) => runtime?.spec.label === selectedJointLeg,
  );
  const channelNames = ["shoulder", "upper", "lower"];
  channelNames.forEach((name, index) => {
    const channel = selectedLeg.channels[name];
    const absolute = displayState.servo_angle_deg[channel];
    const delta = (absolute - neutralServoAngles[channel]) / selectedLeg.directions[name];
    const sign = delta >= 0 ? "+" : "";
    document.querySelector(values[index]).textContent =
      `${sign}${delta.toFixed(1)}° / ${absolute.toFixed(1)}°`;
  });

  channelNames.forEach((name) => {
    const channel = selectedLeg.channels[name];
    const absolute = displayState.servo_angle_deg[channel];
    const delta = (absolute - neutralServoAngles[channel]) / selectedLeg.directions[name];
    updateMotionIndicator(
      selectedRuntime?.annotations[ACTIVE_ANNOTATION_BY_CHANNEL[name]],
      delta,
    );
    drawJointCallout(
      selectedRuntime?.annotations[ACTIVE_ANNOTATION_BY_CHANNEL[name]]?.callout,
      DRIVE_META[name],
      delta,
      absolute,
    );
  });

  const commandIndex = LEG_COMMAND_INDEX_BY_LABEL[selectedJointLeg];
  const command = displayState.leg_command_xyz_mm?.[commandIndex];
  footTarget.textContent = Array.isArray(command)
    ? `${command.map((value) => Number(value).toFixed(0)).join(" / ")} mm`
    : "--";
  const legIndex = legs.indexOf(selectedLeg);
  const planted = Boolean(physicsState?.footContacts?.[legIndex]);
  footState.textContent = planted ? "PLANTED" : "AIR";
  footState.dataset.state = planted ? "ok" : "air";

  const closure = document.querySelector("#joint-closure-value");
  const pinHealth = document.querySelector("#pin-error");
  const closureHealthy = pinHealth.dataset.state === "ok";
  closure.textContent = closureHealthy ? "6 / 6 LOCKED" : "CHECK PINS";
  closure.dataset.state = closureHealthy ? "ok" : "fault";
}

document.querySelector("#stand-button").addEventListener("click", () => {
  requestStand(!standRequested);
});
document.querySelector("#tilt-button").addEventListener("click", () => {
  requestTilt(!tiltRequested);
});
document.querySelectorAll("[data-walk-mode]").forEach((button) => {
  button.addEventListener("click", () => {
    requestWalkMode(Number(button.dataset.walkMode));
  });
});
document.querySelector("#gait-lab-button").addEventListener("click", (event) => {
  setGaitLabPanelOpen(event.currentTarget.getAttribute("aria-expanded") !== "true");
});
document.querySelector("#gait-lab-close").addEventListener("click", () => {
  setGaitLabPanelOpen(false);
});
document.querySelector("#gait-lab-enabled").addEventListener("change", (event) => {
  commitGaitLabSettings({ ...gaitLabSettings, enabled: event.currentTarget.checked }, gaitLabSettings.preset);
});
document.querySelectorAll("[data-gait-preset]").forEach((button) => {
  button.addEventListener("click", () => {
    const preset = button.dataset.gaitPreset;
    setGaitProfileStatus();
    commitGaitLabSettings(
      { ...gaitLabSettings, ...gaitLabPresets[preset], enabled: true },
      preset,
    );
  });
});
gaitLabControls.forEach((control) => {
  const input = document.querySelector(`[data-gait-setting="${control.key}"] input`);
  input.addEventListener("input", (event) => {
    commitGaitLabSettings({ ...gaitLabSettings, [control.key]: Number(event.currentTarget.value) });
  });
});
document.querySelector("#gait-profile-select").addEventListener("change", (event) => {
  const name = event.currentTarget.value;
  document.querySelector("#gait-profile-name").value = name;
  syncGaitProfileUi(name);
  setGaitProfileStatus();
});
document.querySelector("#gait-profile-save").addEventListener("click", () => {
  const input = document.querySelector("#gait-profile-name");
  const name = (input.value.trim() || selectedGaitProfileName()).slice(0, 32);
  if (!name) {
    setGaitProfileStatus("ENTER A PROFILE NAME", "error");
    input.focus();
    return;
  }
  gaitProfiles[name] = sanitizeGaitLabSettings({ ...gaitLabSettings, preset: "custom" });
  if (!persistGaitProfiles()) return;
  input.value = name;
  syncGaitProfileUi(name);
  setGaitProfileStatus(`SAVED ${name}`, "ok");
});
document.querySelector("#gait-profile-load").addEventListener("click", () => {
  const name = selectedGaitProfileName();
  const profile = gaitProfiles[name];
  if (!profile) {
    setGaitProfileStatus("SELECT A SAVED PROFILE", "error");
    return;
  }
  commitGaitLabSettings({ ...profile }, "custom");
  document.querySelector("#gait-profile-name").value = name;
  setGaitProfileStatus(`LOADED ${name}`, "ok");
});
document.querySelector("#gait-profile-delete").addEventListener("click", () => {
  const name = selectedGaitProfileName();
  if (!name || !Object.hasOwn(gaitProfiles, name)) return;
  delete gaitProfiles[name];
  if (!persistGaitProfiles()) return;
  document.querySelector("#gait-profile-name").value = "";
  syncGaitProfileUi();
  setGaitProfileStatus(`DELETED ${name}`, "ok");
});
document.querySelector("#gait-lab-reset").addEventListener("click", () => {
  gaitLab.reset();
  commitGaitLabSettings({ ...defaultGaitLabSettings }, "balanced");
  setGaitProfileStatus();
});
syncGaitLabUi();
syncGaitProfileUi();
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
document.querySelectorAll("[data-drive-joint]").forEach((button) => {
  button.addEventListener("click", () => {
    selectedDriveJoint = button.dataset.driveJoint;
    updateJointOverlay();
    updateJointLegendValues();
  });
});
document.querySelector("#joint-opacity").addEventListener("input", (event) => {
  jointOverlayOpacity = Number(event.currentTarget.value);
  updateJointOverlay();
});
document.querySelector("#float-button").addEventListener("click", () => {
  setFloatMode(!floatModeEnabled);
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
  const physicalWalkMode = walkModeFromChannel(channels[6]);

  if (observedPhysicalStand !== null && physicalStand !== observedPhysicalStand) {
    manualStandOverride = null;
  }
  if (observedPhysicalTilt !== null && physicalTilt !== observedPhysicalTilt) {
    manualTiltOverride = null;
  }
  if (observedPhysicalWalkMode !== null && physicalWalkMode !== observedPhysicalWalkMode) {
    manualWalkModeOverride = null;
  }
  observedPhysicalStand = physicalStand;
  observedPhysicalTilt = physicalTilt;
  observedPhysicalWalkMode = physicalWalkMode;

  standRequested = manualStandOverride ?? physicalStand;
  tiltRequested = manualTiltOverride ?? physicalTilt;
  walkModeRequested = manualWalkModeOverride ?? physicalWalkMode;
  if (!standRequested) tiltRequested = false;

  if (manualStandOverride !== null) channels[4] = standRequested ? 2000 : 1000;
  if (manualTiltOverride !== null || !standRequested) channels[7] = tiltRequested ? 2000 : 1000;
  if (manualWalkModeOverride !== null) channels[6] = channelFromWalkMode(walkModeRequested);
}

function updateInput() {
  let directRadioChannels = false;
  const gamepad = [...(globalThis.navigator?.getGamepads?.() || [])].find(Boolean);

  if (demoMode) {
    boxerHeartbeat = { connected: false, name: "", updatedAt: 0 };
    const tiltDiagnostic = demoSelection === "tilt";
    const rollDiagnostic = demoSelection === "roll" || demoSelection === "roll-negative";
    const gaitDiagnostic = demoSelection === "gait" || demoSelection === "gait-reverse";
    const diagnosticModeReady = gaitDiagnostic
      ? firmwareState?.mode === "GAIT" && firmwareState?.motion_input_armed !== false
      : tiltDiagnostic || rollDiagnostic
        ? firmwareState?.mode === "TILT" && firmwareState?.motion_input_armed !== false
        : true;
    document.querySelector("#gamepad-name").textContent =
      gaitDiagnostic ? "GAIT DIAGNOSTIC" :
        rollDiagnostic ? "ROLL DIAGNOSTIC" :
          tiltDiagnostic ? "TILT DIAGNOSTIC" : "AUTOMATIC DEMO";
    standRequested = true;
    tiltRequested = tiltDiagnostic || rollDiagnostic;
    walkModeRequested = gaitDiagnostic ? 2 : 0;
    forwardInput = diagnosticModeReady && gaitDiagnostic
      ? demoSelection === "gait-reverse" ? -0.55 : 0.55
      : diagnosticModeReady && tiltDiagnostic ? -0.25 : 0;
    turnInput = diagnosticModeReady && tiltDiagnostic ? 0.30 : 0;
    rollInput = diagnosticModeReady && rollDiagnostic
      ? demoSelection === "roll-negative" ? -1 : 1
      : diagnosticModeReady && tiltDiagnostic ? 0.35 : 0;
    clientInputSnapshot = {
      source: "diagnostic",
      name: gaitDiagnostic ? "GAIT DIAGNOSTIC" :
        rollDiagnostic ? "ROLL DIAGNOSTIC" :
          tiltDiagnostic ? "TILT DIAGNOSTIC" : "STAND DIAGNOSTIC",
      axes: [],
    };
  } else if (bridgeInput.connected && Array.isArray(bridgeInput.channels)) {
    directRadioChannels = true;
    for (let index = 0; index < 8; index += 1) {
      channels[index] = bridgeInput.channels[index];
    }
    document.querySelector("#gamepad-name").textContent = `USB / ${bridgeInput.name}`;
    boxerHeartbeat = {
      connected: true,
      name: bridgeInput.name || "RadioMaster Boxer",
      updatedAt: Number(bridgeInput.updatedAt) || Date.now(),
    };
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
    boxerHeartbeat = isEdgeTxRadio
      ? { connected: true, name: gamepad.id, updatedAt: Date.now() }
      : { connected: false, name: "", updatedAt: 0 };
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
      pressedOnce(gamepad, 0, () => requestStand(!standRequested));
      pressedOnce(gamepad, 1, () => requestTilt(!tiltRequested));
      pressedOnce(gamepad, 3, resetRobot);
    }
  } else {
    boxerHeartbeat = { connected: false, name: "", updatedAt: 0 };
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
    channels[6] = channelFromWalkMode(walkModeRequested);
    channels[7] = tiltRequested ? 2000 : 1000;
  }

  const manualOverrideActive =
    manualStandOverride !== null ||
    manualTiltOverride !== null ||
    manualWalkModeOverride !== null;
  const inputUiNow = performance.now();
  const inputUiKey = [
    forwardInput.toFixed(2),
    turnInput.toFixed(2),
    rollInput.toFixed(2),
    ...channels.slice(0, 8).map((value) => Math.round(value / 2) * 2),
    manualOverrideActive ? "manual" : "radio",
  ].join("|");
  if (inputUiNow - lastInputUiUpdate < 40 && inputUiKey === lastInputUiKey) return;
  lastInputUiUpdate = inputUiNow;
  lastInputUiKey = inputUiKey;
  document.querySelector("#forward-value").textContent = forwardInput.toFixed(2);
  document.querySelector("#turn-value").textContent = turnInput.toFixed(2);
  document.querySelector("#roll-value").textContent = rollInput.toFixed(2);
  const heightMillimeters = heightMillimetersFromChannel(channels[RIDE_HEIGHT_CHANNEL_INDEX]);
  document.querySelector("#height-input-value").textContent =
    `${heightMillimeters.toFixed(0)} mm`;
  document.querySelector("#height-control-value").textContent =
    `${heightMillimeters.toFixed(0)} MM`;
  document.querySelector("#height-meter-fill").style.width =
    `${heightFractionFromMillimeters(heightMillimeters) * 100}%`;
  document.querySelector("#channel-source").textContent = manualOverrideActive ? "MANUAL" : "ELRS";
  document.querySelector("#channel-source-detail").textContent =
    manualOverrideActive ? "SIM OVERRIDE" : "CRSF INPUT";
  updateChannelBars();
}

let firmwarePollInFlight = false;

async function pollFirmware() {
  if (firmwarePollInFlight) return;
  firmwarePollInFlight = true;
  try {
    const response = await fetch(`/runtime/state.json?t=${Date.now()}`, { cache: "no-store" });
    if (!response.ok) throw new Error("Firmware state unavailable");
    firmwareState = await response.json();
    if (!visualServoAngles && Array.isArray(firmwareState.servo_angle_deg)) {
      visualServoAngles = [...firmwareState.servo_angle_deg];
    }
    document.querySelector("#firmware-status").dataset.state = "online";
    const stance = firmwareState.tilt_active
      ? "TILTING"
      : firmwareState.mode === "STOW"
        ? "SITTING"
        : firmwareState.mode === "STAND"
          ? "STANDING"
          : firmwareState.mode;
    const sitting = stance === "SITTING";
    const tilting = stance === "TILTING";
    const motionInputArmed = firmwareState.motion_input_armed !== false;
    const activeWalkMode = firmwareState.mode === "CAREFUL" ? 1 : firmwareState.mode === "GAIT" ? 2 : 0;
    const selectedWalkMode = walkModeFromChannel(channels[6]);
    const standButton = document.querySelector("#stand-button");
    const tiltButton = document.querySelector("#tilt-button");
    const gaitState = document.querySelector("#gait-state");
    const rideHeightMm = Number.isFinite(Number(firmwareState.ride_height_mm))
      ? Number(firmwareState.ride_height_mm)
      : heightMillimetersFromChannel(channels[RIDE_HEIGHT_CHANNEL_INDEX]);
    const commandedHeightMm = Number.isFinite(Number(firmwareState.target_z_mm))
      ? Number(firmwareState.target_z_mm)
      : rideHeightMm;
    const effectiveCommandedHeightMm = gaitLab.getTelemetry().active &&
      Number.isFinite(Number(effectiveFirmwareState?.target_z_mm))
      ? Number(effectiveFirmwareState.target_z_mm)
      : commandedHeightMm;
    document.querySelector("#ride-height").textContent =
      `${effectiveCommandedHeightMm.toFixed(0)} mm`;
    const requestedStanding = manualStandOverride ?? standRequested;
    const poseTransitionPending =
      Math.abs(firmwareState.pose_z_mm - firmwareState.target_z_mm) > 0.5;
    const transitionPending = requestedStanding === sitting || poseTransitionPending;
    const walkBlockedByTilt =
      selectedWalkMode > 0 && (channels[7] > 1600 || tilting);
    const walkArmedByPose =
      selectedWalkMode > 0 && (sitting || transitionPending);
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
    document.querySelectorAll("[data-walk-mode]").forEach((button) => {
      const buttonMode = Number(button.dataset.walkMode);
      button.classList.toggle("active", buttonMode === selectedWalkMode);
      button.dataset.state = buttonMode !== selectedWalkMode
        ? "idle"
        : walkBlockedByTilt
          ? "blocked"
          : walkArmedByPose
            ? "armed"
            : buttonMode === activeWalkMode && firmwareState.mode !== "STOW"
              ? "active"
              : "ready";
    });
    const walkLabels = ["STAND", "CAREFUL", "TROT"];
    gaitState.textContent = walkBlockedByTilt
      ? `${walkLabels[selectedWalkMode]} BLOCKED BY TILT`
      : walkArmedByPose
        ? `${walkLabels[selectedWalkMode]} ARMED`
        : activeWalkMode === selectedWalkMode
          ? walkLabels[activeWalkMode]
          : `${walkLabels[selectedWalkMode]} READY`;
    gaitState.dataset.state = walkBlockedByTilt
      ? "blocked"
      : walkArmedByPose
        ? "armed"
        : activeWalkMode > 0
          ? "active"
          : "idle";
    gaitState.title = walkArmedByPose
      ? "Walk switch selected; gait starts after Domino reaches the standing pose"
      : "";
    updateJointLegendValues();
  } catch {
    document.querySelector("#firmware-status").dataset.state = "offline";
  } finally {
    firmwarePollInFlight = false;
  }
}
// Follow the firmware's 20 ms control clock without phase-locking reads to its
// alternating state-file writes.
setInterval(pollFirmware, 23);

function updateRobot(delta) {
  const displayState = effectiveFirmwareState || firmwareState;
  if (!displayState || !linkageRuntimesReady()) return;

  if (!visualServoAngles) {
    visualServoAngles = [...displayState.servo_angle_deg];
  }
  const blend = 1 - Math.exp(-SERVO_VISUAL_RESPONSE * Math.max(delta, 0));
  visualServoAngles.forEach((current, channel) => {
    const target = Number(displayState.servo_angle_deg[channel]);
    if (Number.isFinite(target)) {
      visualServoAngles[channel] = THREE.MathUtils.lerp(current, target, blend);
    }
  });

  applyServoAnglesToRuntimes(linkageRuntimes, visualServoAngles);

  if (telemetryDatasetDue) {
    canvas.dataset.robotX = robotWorld.position.x.toFixed(4);
    canvas.dataset.robotZ = robotWorld.position.z.toFixed(4);
  }
}

function applyServoAnglesToRuntimes(runtimes, servoAngles) {
  const servoReference = neutralServoAngles;

  runtimes.forEach((runtime) => {
    const { channels: legChannels, directions } = runtime.spec;
    const shoulderDelta =
      (servoAngles[legChannels.shoulder] - servoReference[legChannels.shoulder]) /
      directions.shoulder;
    const upperDelta =
      (servoAngles[legChannels.upper] - servoReference[legChannels.upper]) /
      directions.upper;
    const lowerDelta =
      (servoAngles[legChannels.lower] - servoReference[legChannels.lower]) /
      directions.lower;

    updateLinkage(runtime, shoulderDelta, upperDelta, lowerDelta);
  });
}

function smoothLiveServoAngles(current, target, delta) {
  if (!current) return [...target];
  const blend = 1 - Math.exp(-SERVO_VISUAL_RESPONSE * Math.max(delta, 0));
  current.forEach((angle, channel) => {
    const next = Number(target[channel]);
    if (Number.isFinite(next)) current[channel] = THREE.MathUtils.lerp(angle, next, blend);
  });
  return current;
}

function applyLiveBodyPose(group, body) {
  if (!group || !body) return;
  group.position.y = body.heightMm / 1_000;
  group.quaternion.setFromEuler(new THREE.Euler(
    THREE.MathUtils.degToRad(body.rollDeg),
    THREE.MathUtils.degToRad(body.yawDeg),
    THREE.MathUtils.degToRad(-body.pitchDeg),
    "YXZ",
  ));
}

function updateLiveTwinPose(delta) {
  if (liveViewState.selected === LIVE_VIEW_CALIBRATION && linkageRuntimesReady()) {
    const previewAngles = liveCalibrationState.previewEnabled
      ? calibrationPreviewServoAngles(liveCalibrationState)
      : standServoReference;
    liveExpectedServoAngles = smoothLiveServoAngles(
      liveExpectedServoAngles,
      previewAngles,
      delta,
    );
    applyServoAnglesToRuntimes(linkageRuntimes, liveExpectedServoAngles);
    applyLiveBodyPose(robotWorld, { rollDeg: 0, pitchDeg: 0, yawDeg: 0, heightMm: 280 });
    if (measuredRobotWorld) measuredRobotWorld.visible = false;
    canvas.dataset.liveExpectedPose = "calibration-preview";
    canvas.dataset.liveMeasuredPose = "unavailable";
    return;
  }
  if (liveViewState.selected === LIVE_VIEW_GAITS && linkageRuntimesReady()) {
    const height = liveGaitState.draft.settings.bodyHeightMm;
    const source = {
      mode: liveGaitState.previewMode === "careful" ? "CAREFUL" : "GAIT",
      tilt_active: false,
      body_pose_rpy_deg: [0, 0, 0],
      pose_z_mm: height,
      target_z_mm: height,
      ride_height_mm: height,
      servo_angle_deg: [...standServoReference],
      leg_command_xyz_mm: [
        [-15.75, 38, height],
        [-15.75, -38, height],
        [-15.75, 38, height],
        [-15.75, -38, height],
      ],
    };
    liveGaitPreviewOutput = liveGaitState.previewEnabled
      ? liveGaitPreviewLab.update(delta, source, {
          forward: liveGaitState.previewForward,
          turn: liveGaitState.previewTurn,
        })
      : source;
    liveExpectedServoAngles = smoothLiveServoAngles(
      liveExpectedServoAngles,
      liveGaitPreviewOutput.servo_angle_deg,
      delta,
    );
    applyServoAnglesToRuntimes(linkageRuntimes, liveExpectedServoAngles);
    applyLiveBodyPose(robotWorld, {
      rollDeg: 0,
      pitchDeg: 0,
      yawDeg: 0,
      heightMm: liveGaitPreviewOutput.pose_z_mm,
    });
    if (measuredRobotWorld) measuredRobotWorld.visible = false;
    canvas.dataset.liveExpectedPose = "gait-preview";
    canvas.dataset.liveMeasuredPose = "unavailable";
    if (telemetryDatasetDue) {
      const preview = liveGaitPreviewLab.getTelemetry();
      document.querySelector("#live-gait-preview-speed").textContent = `${Math.round(preview.speedMmPerSec || 0)} mm/s`;
      document.querySelector("#live-gait-preview-support").textContent = `${preview.stanceCount ?? 4} / 4`;
      const previewHealth = document.querySelector("#live-gait-preview-health");
      previewHealth.textContent = `${preview.reachableCount ?? 4} / 4 REACHABLE`;
      previewHealth.dataset.state = preview.reachableCount === 4 ? "ok" : "fault";
    }
    return;
  }
  const snapshot = liveComparisonSnapshot(liveTelemetryState);
  canvas.dataset.liveExpectedPose = snapshot.expected ? "fresh" : "unavailable";
  if (snapshot.expected && linkageRuntimesReady()) {
    liveExpectedServoAngles = smoothLiveServoAngles(
      liveExpectedServoAngles,
      snapshot.expected.servoAngleDeg,
      delta,
    );
    applyServoAnglesToRuntimes(linkageRuntimes, liveExpectedServoAngles);
    applyLiveBodyPose(robotWorld, snapshot.expected.body);
  }

  if (measuredRobotWorld) {
    measuredRobotWorld.visible = Boolean(
      snapshot.measured && measuredLinkageRuntimes.length === linkageRuntimes.length,
    );
    if (measuredRobotWorld.visible) {
      liveMeasuredServoAngles = smoothLiveServoAngles(
        liveMeasuredServoAngles,
        snapshot.measured.servoAngleDeg,
        delta,
      );
      applyServoAnglesToRuntimes(measuredLinkageRuntimes, liveMeasuredServoAngles);
      measuredRobotWorld.position.x = robotWorld.position.x;
      measuredRobotWorld.position.z = robotWorld.position.z;
      applyLiveBodyPose(measuredRobotWorld, snapshot.measured.body);
    }
    canvas.dataset.liveMeasuredPose = measuredRobotWorld.visible ? "fresh" : "unavailable";
  }
}

function pointOnBody(group, point, anchor) {
  return group.localToWorld(v3(point).sub(v3(anchor)));
}

function updatePinClosureHealth() {
  if (!telemetryDatasetDue || !linkageRuntimesReady()) return;

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
  if (telemetryDatasetDue) canvas.dataset.pinClosure = JSON.stringify(state);

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
  if (!telemetryDatasetDue || !linkageRuntimesReady()) return;

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
    effectiveFirmwareState?.tilt_active ||
    effectiveFirmwareState?.mode === "BALANCE" ||
    effectiveFirmwareState?.mode === "GAIT" ||
    effectiveFirmwareState?.mode === "CAREFUL";
  const healthy = intentionalAsymmetry || worst.errorMm <= FOOT_SYMMETRY_TOLERANCE_MM;
  const state = {
    healthy,
    intentionalAsymmetry,
    toleranceMm: FOOT_SYMMETRY_TOLERANCE_MM,
    worst,
    pairs,
  };
  globalThis.dominoFootSymmetryState = state;
  if (telemetryDatasetDue) canvas.dataset.footSymmetry = JSON.stringify(state);

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
  const displayState = effectiveFirmwareState || firmwareState;
  if (floatModeEnabled) {
    robotWorld.position.copy(floatAnchorPosition);
    robotWorld.quaternion.copy(floatAnchorQuaternion);
    visualBasePosition.copy(floatAnchorPosition);
    visualBaseQuaternion.copy(floatAnchorQuaternion);
    physicsState = {
      engine: "Kinematic float",
      proxy: "Suspended CAD inspection",
      massModel: dominoMassModel,
      basePosition: floatAnchorPosition.toArray(),
      baseQuaternion: floatAnchorQuaternion.toArray(),
      linearVelocity: [0, 0, 0],
      angularVelocity: [0, 0, 0],
      footContacts: [false, false, false, false],
      footPositions: [],
      footSupportHeights: [null, null, null, null],
      footRadius: CAD_FOOT_RADIUS,
      contactCount: 0,
      bodyHeight: floatAnchorPosition.y,
      bodyClearance: floatAnchorPosition.y,
      bodyAltitude: floatAnchorPosition.y,
      supportHeight: 0,
      baseTiltDegrees: 0,
      commandedBodyHeight: floatAnchorPosition.y,
      resetCount: visualPhysicsResetCount,
      lastResetReason: "float-mode",
      floatMode: true,
      environmentBalls: [],
    };
    globalThis.dominoPhysicsState = physicsState;
    if (telemetryDatasetDue) {
      canvas.dataset.physics = JSON.stringify(physicsState);
      document.querySelector("#physics-status").dataset.state = "online";
      document.querySelector("#physics-engine").textContent = "FLOAT / NO GROUND";
      document.querySelector("#body-height").textContent = "SUSPENDED";
      document.querySelector("#foot-contacts").textContent = "0 / 4";
      document.querySelector("#body-plane-tilt").textContent = "0.00Â°";
      document.querySelector("#body-roll").textContent = "+0.00Â°";
      document.querySelector("#body-pitch").textContent = "+0.00Â°";
      document.querySelector("#body-yaw").textContent = "+0.00Â°";
    }
    return;
  }
  physicsState = physics.update(delta, displayState, legs, neutralServoAngles);
  globalThis.dominoPhysicsState = physicsState;
  if (telemetryDatasetDue) canvas.dataset.physics = JSON.stringify(physicsState);
  const [x, y, z] = physicsState.basePosition;
  const [qx, qy, qz, qw] = physicsState.baseQuaternion;
  const targetPosition = new THREE.Vector3(x, y, z);
  const physicsQuaternion = new THREE.Quaternion(qx, qy, qz, qw).normalize();
  // Firmware servo targets and body pose are one kinematic snapshot. Driving
  // the visible linkages from that snapshot while driving the CAD body from a
  // slower torque response made the feet lift during Q/E roll transitions.
  // Keep the rendered CAD on the firmware clock; Rapier still supplies world
  // translation, contacts, failure detection, and the underlying dynamics.
  const commandedTiltQuaternion =
    displayState?.mode === "TILT" &&
    Array.isArray(physicsState.bodyPoseTargetQuaternion)
      ? new THREE.Quaternion(...physicsState.bodyPoseTargetQuaternion).normalize()
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
  physicsState.environmentBalls?.forEach((ball) => {
    const mesh = environmentBallMeshes.get(ball.id);
    if (!mesh) return;
    mesh.position.set(...ball.position);
    mesh.quaternion.set(...ball.quaternion);
  });
  if (!telemetryDatasetDue) return;
  bodyEuler.setFromQuaternion(robotWorld.quaternion, "YXZ");
  const rollDeg = THREE.MathUtils.radToDeg(bodyEuler.x);
  // Three.js uses world +Z to the viewer's side, while Domino CAD +Y points
  // left, so CAD pitch has the opposite sign of the Three.js Z rotation.
  const pitchDeg = -THREE.MathUtils.radToDeg(bodyEuler.z);
  const yawDeg = THREE.MathUtils.radToDeg(bodyEuler.y);
  const formatSignedAngle = (value) => `${value >= 0 ? "+" : ""}${value.toFixed(2)}°`;

  document.querySelector("#physics-status").dataset.state = "online";
  document.querySelector("#physics-engine").textContent = "RAPIER + CAD";
  const massModel = physicsState.massModel || dominoMassModel;
  const massReadout = document.querySelector("#mass-model");
  massReadout.textContent = `${massModel.totalMassKg.toFixed(2)} kg / 2x 4S`;
  massReadout.title =
    `Two CNHL 1500 mAh 4S packs at ${(massModel.packMassKg * 1000).toFixed(0)} g each`;
  const bodyClearance = physicsState.bodyClearance ?? physicsState.bodyHeight;
  document.querySelector("#body-height").textContent = `${(bodyClearance * 1000).toFixed(0)} mm`;
  document.querySelector("#foot-contacts").textContent = `${physicsState.contactCount} / 4`;
  document.querySelector("#body-plane-tilt").textContent =
    `${physicsState.baseTiltDegrees.toFixed(2)}°`;
  document.querySelector("#body-roll").textContent = formatSignedAngle(rollDeg);
  document.querySelector("#body-pitch").textContent = formatSignedAngle(pitchDeg);
  document.querySelector("#body-yaw").textContent = formatSignedAngle(yawDeg);
}

function syncVisibleCadToPhysicsFeet(delta) {
  if (floatModeEnabled || !physicsState || !linkageRuntimesReady()) return;
  scene.updateMatrixWorld(true);

  const visualFeet = linkageRuntimes.map((runtime) => {
    runtime.footProbe.getWorldPosition(visualFootPosition);
    return visualFootPosition.clone();
  });
  const contactSamples = visualFeet.flatMap((foot, index) => {
    if (!physicsState.footContacts[index]) return [];
    const proxyFoot = physicsState.footPositions[index];
    if (!Array.isArray(proxyFoot)) return [];
    const proxyRadius = Number(physicsState.footRadius) || CAD_FOOT_RADIUS;
    const supportHeight = Number.isFinite(physicsState.footSupportHeights?.[index])
      ? physicsState.footSupportHeights[index]
      : proxyFoot[1] - proxyRadius;
    return [{
      x: foot.x,
      z: foot.z,
      // Register against the proxy's contacted surface, not world zero. This
      // keeps CAD feet planted on stairs, ramps, and movable props in tilt.
      errorY: -contactSurfaceError(
        foot.y,
        CAD_FOOT_RADIUS,
        supportHeight + proxyRadius,
        proxyRadius,
      ),
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
      const proxyFoot = physicsState.footPositions[index];
      const proxyRadius = Number(physicsState.footRadius) || CAD_FOOT_RADIUS;
      const supportHeight = Number.isFinite(physicsState.footSupportHeights?.[index])
        ? physicsState.footSupportHeights[index]
        : proxyFoot?.[1] - proxyRadius;
      return physicsState.footContacts[index] && Array.isArray(proxyFoot)
        ? contactSurfaceError(
            position[1],
            CAD_FOOT_RADIUS,
            supportHeight + proxyRadius,
            proxyRadius,
          )
        : null;
    }),
  };
  visualState.groundedCount = visualState.contactErrors.filter(
    (error) => Number.isFinite(error) && Math.abs(error) <= 0.004,
  ).length;
  globalThis.dominoVisualState = visualState;
  if (telemetryDatasetDue) {
    canvas.dataset.visual = JSON.stringify(visualState);
    document.querySelector("#foot-contacts").textContent =
      `${Math.min(physicsState.contactCount, visualState.groundedCount)} / 4`;
  }
}

let lastFrame = performance.now();
let lastInputUiUpdate = 0;
let lastInputUiKey = "";
let telemetryDatasetElapsed = 0;
let telemetryDatasetDue = true;
let cameraGizmoElapsed = 0;
const previousControlsTarget = new THREE.Vector3();
const desiredControlsTarget = new THREE.Vector3();

function updateGaitLabState(delta) {
  effectiveFirmwareState = gaitLab.update(delta, firmwareState, {
    forward: forwardInput,
    turn: turnInput,
  });
  const telemetry = gaitLab.getTelemetry();
  if (!telemetryDatasetDue) return;
  const runtimeStatus = document.querySelector("#gait-lab-runtime-status");
  const walkingMode = firmwareState?.mode === "GAIT" || firmwareState?.mode === "CAREFUL";
  if (!gaitLabSettings.enabled) {
    runtimeStatus.textContent = "SIM ONLY / OVERRIDE OFF";
    runtimeStatus.dataset.state = "idle";
  } else if (firmwareState?.tilt_active) {
    runtimeStatus.textContent = "BLOCKED / TILT ACTIVE";
    runtimeStatus.dataset.state = "blocked";
  } else if (!walkingMode) {
    runtimeStatus.textContent = firmwareState?.mode === "STOW"
      ? "ARMED / STAND DOMINO FIRST"
      : "ARMED / SELECT WALK MODE";
    runtimeStatus.dataset.state = "armed";
  } else {
    runtimeStatus.textContent = `LIVE / ${firmwareState.mode === "GAIT" ? "TROT" : "CAREFUL"}`;
    runtimeStatus.dataset.state = "live";
  }
  document.querySelector("#gait-lab-speed").textContent =
    `${Math.round(telemetry.speedMmPerSec || 0)} mm/s`;
  document.querySelector("#gait-lab-support").textContent =
    `${telemetry.stanceCount ?? 4} / 4`;
  document.querySelector("#gait-lab-reach").textContent =
    `${telemetry.reachableCount ?? 4} / 4`;
  document.querySelector("#gait-lab-reach").dataset.state =
    telemetry.reachableCount === 4 ? "ok" : "fault";
  document.querySelector("#gait-lab-button").dataset.running = telemetry.active ? "true" : "false";
  canvas.dataset.gaitLab = JSON.stringify({ ...telemetry, settings: gaitLabSettings });
}

function animate(now) {
  const delta = Math.min(0.05, (now - lastFrame) / 1000);
  lastFrame = now;
  telemetryDatasetElapsed += delta;
  telemetryDatasetDue = telemetryDatasetElapsed >= 0.1;
  if (telemetryDatasetDue) telemetryDatasetElapsed = 0;
  if (applicationState.workspace !== WORKSPACE_SIMULATION) {
    updateLiveTwinPose(delta);
    ground.visible = true;
    grid.visible = true;
    courseVisuals.visible = false;
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
    return;
  }
  updateInput();
  updateGaitLabState(delta);
  updateRobot(delta);
  if (linkageRuntimesReady()) scene.updateMatrixWorld(true);
  updatePinClosureHealth();
  updateFootSymmetryHealth();
  updatePhysics(delta);
  syncVisibleCadToPhysicsFeet(delta);
  updateFootTrajectories(delta);
  previousControlsTarget.copy(controls.target);
  robotCameraAnchor.set(
    robotWorld.position.x,
    floatModeEnabled ? robotWorld.position.y : 0.32,
    robotWorld.position.z,
  );
  if (!middleButtonPanning) {
    desiredControlsTarget.copy(robotCameraAnchor).add(cameraTargetOffset);
    controls.target.lerp(desiredControlsTarget, 0.08);
    camera.position.add(controls.target).sub(previousControlsTarget);
  }
  if (orthographicAxisDirection && inspectionGrid.visible) {
    inspectionGrid.position.copy(controls.target)
      .addScaledVector(orthographicAxisDirection, -0.55);
  }
  if (!updateCameraSnap(now)) controls.update();
  const viewingFromBelow = camera.position.y < 0;
  ground.visible = !floatModeEnabled && !viewingFromBelow;
  grid.visible = !floatModeEnabled && !viewingFromBelow;
  courseVisuals.visible = !floatModeEnabled;
  updateJointCalloutScale();
  cameraGizmoElapsed += delta;
  if (cameraGizmoElapsed >= 1 / 30) {
    cameraGizmoElapsed = 0;
    updateCameraGizmo();
  }
  renderer.render(scene, camera);

  requestAnimationFrame(animate);
}

function resize() {
  const bounds = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(bounds.width));
  const height = Math.max(1, Math.round(bounds.height));
  renderer.setSize(width, height, false);
  perspectiveCamera.aspect = width / height;
  perspectiveCamera.updateProjectionMatrix();
  updateOrthographicProjection();
  const pixelRatio = Math.min(window.devicePixelRatio, MAX_RENDER_PIXEL_RATIO);
  cameraGizmoCanvas.width = CAMERA_GIZMO_SIZE * pixelRatio;
  cameraGizmoCanvas.height = CAMERA_GIZMO_SIZE * pixelRatio;
  cameraGizmoContext.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  updateGaitLabMinimumSize();
  positionGaitLabPanel();
  gaitPanelController?.clamp();
  inspectorPanelController?.clamp();
  positionGaitSettingInfo();
}

window.addEventListener("resize", resize);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    releaseLiveManualControl("Browser control was released because this tab is no longer visible.");
    return;
  }
  requestAnimationFrame(() => {
    requestAnimationFrame(() => {
      positionGaitLabPanel();
      gaitPanelController?.clamp();
      inspectorPanelController?.clamp();
      positionGaitSettingInfo();
    });
  });
});
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
