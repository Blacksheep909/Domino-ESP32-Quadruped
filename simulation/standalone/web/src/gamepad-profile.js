const RADIO_PATTERN = /radiomaster|edge\s*tx|open\s*tx|tx16s|boxer|zorro|pocket|jumper|t-?lite|t-?pro/i;
const PLAYSTATION_PATTERN = /playstation|dualshock|dualsense|sony|054c/i;
const XBOX_PATTERN = /xbox|xinput|045e/i;

export function identifyInputDevice(gamepad) {
  const id = String(gamepad?.id || "Unknown controller");
  const axes = Array.from(gamepad?.axes || []);
  if (RADIO_PATTERN.test(id) && axes.length >= 8) return { family: "crsf-radio", label: "CRSF RADIO", id };
  if (PLAYSTATION_PATTERN.test(id)) return { family: "playstation", label: "PLAYSTATION", id };
  if (XBOX_PATTERN.test(id)) return { family: "xbox", label: "XBOX", id };
  return { family: "generic", label: "GAMEPAD", id };
}

export function normalizedGamepadControls(gamepad) {
  const profile = identifyInputDevice(gamepad);
  const axes = Array.from(gamepad?.axes || []);
  if (profile.family === "crsf-radio") {
    return { profile, radioAxes: axes.slice(0, 8) };
  }

  // Xbox, DualShock 4, DualSense, and standards-compliant generic pads expose
  // left X/Y and right X/Y at axes 0..3. The fallback supports older HID pads.
  return {
    profile,
    roll: axes[0] || 0,
    forward: -(axes[1] || 0),
    turn: axes[2] ?? axes[3] ?? axes[0] ?? 0,
    buttons: { stand: 0, tilt: 1, reset: 3 },
  };
}
