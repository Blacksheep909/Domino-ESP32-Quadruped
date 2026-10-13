export const DEFAULT_CAMERA_PRESET = "studio";

export const CAMERA_CONTROL_PRESETS = Object.freeze([
  { id: "studio", label: "Quadruped Studio", orbit: "Left drag", pan: "Middle / right drag, or Shift + left drag", zoom: "Scroll wheel" },
  { id: "blender", label: "Blender", orbit: "Middle drag", pan: "Shift + middle drag", zoom: "Scroll wheel, or Ctrl + middle drag",
    source: "https://docs.blender.org/manual/en/latest/editors/3dview/navigate/navigation.html" },
  { id: "solidworks", label: "SolidWorks", orbit: "Middle drag", pan: "Ctrl + middle drag", zoom: "Scroll wheel, or Shift + middle drag",
    source: "https://help.solidworks.com/2025/english/SolidWorks/sldworks/r_Middle_Mouse_Button.htm" },
  { id: "fusion", label: "Fusion", orbit: "Shift + middle drag", pan: "Middle drag", zoom: "Scroll wheel, or Ctrl + Shift + middle drag",
    source: "https://help.autodesk.com/view/fusion360/ENU/index.html?guid=GUID-878489CD-3A23-4303-8450-C2F4F8E410B1" },
  { id: "onshape", label: "Onshape", orbit: "Right drag", pan: "Middle drag, or Ctrl + right drag", zoom: "Scroll wheel",
    source: "https://cad.onshape.com/help/Content/View/view_navigation_and_the_view_cube.htm" },
].map((preset) => Object.freeze(preset)));

export function sanitizeCameraPreset(value) {
  return CAMERA_CONTROL_PRESETS.some(({ id }) => id === value) ? value : DEFAULT_CAMERA_PRESET;
}

export function cameraGestureAction(preset, event) {
  const button = event?.button;
  const control = Boolean(event?.ctrlKey || event?.metaKey);
  const shift = Boolean(event?.shiftKey);
  switch (sanitizeCameraPreset(preset)) {
    case "blender":
      return button !== 1 ? "none" : control ? "zoom" : shift ? "pan" : "orbit";
    case "solidworks":
      return button !== 1 ? "none" : control ? "pan" : shift ? "zoom" : "orbit";
    case "fusion":
      return button !== 1 ? "none" : control && shift ? "zoom" : shift ? "orbit" : "pan";
    case "onshape":
      return button === 1 ? "pan" : button === 2 ? control ? "pan" : "orbit" : "none";
    default:
      return button === 0 ? control || shift ? "pan" : "orbit"
        : button === 1 || button === 2 ? "pan" : "none";
  }
}

// OrbitControls swaps ROTATE/PAN when a modifier is held. Account for that
// when selecting its action; Fusion's Shift+middle must still orbit.
export function orbitMouseAction(action, event) {
  const modified = Boolean(event?.ctrlKey || event?.metaKey || event?.shiftKey);
  if (action === "zoom") return "DOLLY";
  if (action === "orbit") return modified ? "PAN" : "ROTATE";
  if (action === "pan") return modified ? "ROTATE" : "PAN";
  return null;
}
