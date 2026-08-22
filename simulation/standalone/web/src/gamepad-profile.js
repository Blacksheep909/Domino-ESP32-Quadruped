const RADIO_PATTERN = /radiomaster|edge\s*tx|open\s*tx|tx16s|boxer|zorro|pocket|jumper|t-?lite|t-?pro/i;
const PLAYSTATION_PATTERN = /playstation|dualshock|dualsense|sony|054c/i;
const XBOX_PATTERN = /xbox|xinput|045e/i;

export const GAMEPAD_MAPPING_STORAGE_KEY = "domino.sim.gamepad-mappings.v1";
export const DEFAULT_GAMEPAD_MAPPING = Object.freeze({
  rollAxis: 0,
  forwardAxis: 1,
  turnAxis: 2,
  invertRoll: false,
  invertForward: true,
  invertTurn: false,
  standButton: 0,
  tiltButton: 1,
  resetButton: 3,
});

const boundedIndex = (value, fallback, maximum = 31) => {
  const parsed = Number(value);
  return Number.isInteger(parsed) && parsed >= 0 && parsed <= maximum ? parsed : fallback;
};

export function sanitizeGamepadMapping(candidate = {}) {
  return {
    rollAxis: boundedIndex(candidate.rollAxis, DEFAULT_GAMEPAD_MAPPING.rollAxis, 15),
    forwardAxis: boundedIndex(candidate.forwardAxis, DEFAULT_GAMEPAD_MAPPING.forwardAxis, 15),
    turnAxis: boundedIndex(candidate.turnAxis, DEFAULT_GAMEPAD_MAPPING.turnAxis, 15),
    invertRoll: candidate.invertRoll === true,
    invertForward: candidate.invertForward !== false,
    invertTurn: candidate.invertTurn === true,
    standButton: boundedIndex(candidate.standButton, DEFAULT_GAMEPAD_MAPPING.standButton),
    tiltButton: boundedIndex(candidate.tiltButton, DEFAULT_GAMEPAD_MAPPING.tiltButton),
    resetButton: boundedIndex(candidate.resetButton, DEFAULT_GAMEPAD_MAPPING.resetButton),
  };
}

export function readGamepadMappings(value) {
  if (!value || typeof value !== "object" || Array.isArray(value)) return {};
  return Object.fromEntries(Object.entries(value).slice(0, 32).map(([id, mapping]) => [String(id).slice(0, 160), sanitizeGamepadMapping(mapping)]));
}

export function identifyInputDevice(gamepad) {
  const id = String(gamepad?.id || "Unknown controller");
  const axes = Array.from(gamepad?.axes || []);
  if (RADIO_PATTERN.test(id) && axes.length >= 8) return { family: "crsf-radio", label: "CRSF RADIO", id };
  if (PLAYSTATION_PATTERN.test(id)) return { family: "playstation", label: "PLAYSTATION", id };
  if (XBOX_PATTERN.test(id)) return { family: "xbox", label: "XBOX", id };
  return { family: "generic", label: "GAMEPAD", id };
}

export function normalizedGamepadControls(gamepad, customMapping = null) {
  const profile = identifyInputDevice(gamepad);
  const axes = Array.from(gamepad?.axes || []);
  if (profile.family === "crsf-radio") {
    return { profile, radioAxes: axes.slice(0, 8) };
  }

  // Xbox, DualShock 4, DualSense, and standards-compliant generic pads expose
  // left X/Y and right X/Y at axes 0..3. The fallback supports older HID pads.
  const mapping = sanitizeGamepadMapping(customMapping || DEFAULT_GAMEPAD_MAPPING);
  const mappedAxis = (index, inverted) => (Number(axes[index]) || 0) * (inverted ? -1 : 1);
  return {
    profile,
    mapping,
    roll: mappedAxis(mapping.rollAxis, mapping.invertRoll),
    forward: mappedAxis(mapping.forwardAxis, mapping.invertForward),
    turn: mappedAxis(mapping.turnAxis, mapping.invertTurn),
    buttons: { stand: mapping.standButton, tilt: mapping.tiltButton, reset: mapping.resetButton },
  };
}
