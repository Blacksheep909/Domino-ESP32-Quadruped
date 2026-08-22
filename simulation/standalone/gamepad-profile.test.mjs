import assert from "node:assert/strict";
import test from "node:test";

import { identifyInputDevice, normalizedGamepadControls, readGamepadMappings, sanitizeGamepadMapping } from "./web/src/gamepad-profile.js";

const pad = (id, axes = [0.25, -0.5, 0.75, -0.25]) => ({ id, axes });

test("identifies common controller families while preserving device identity", () => {
  assert.equal(identifyInputDevice(pad("Xbox Wireless Controller")).family, "xbox");
  assert.equal(identifyInputDevice(pad("Wireless Controller (STANDARD GAMEPAD Vendor: 054c Product: 09cc)")).family, "playstation");
  assert.equal(identifyInputDevice(pad("DualSense Wireless Controller")).family, "playstation");
  assert.equal(identifyInputDevice(pad("8BitDo Ultimate")).family, "generic");
  assert.equal(identifyInputDevice(pad("RadioMaster TX16S", Array(8).fill(0))).family, "crsf-radio");
});

test("normalizes standard Xbox, PlayStation, and generic gamepad axes", () => {
  for (const id of ["Xbox Controller", "DualShock 4", "Logitech Gamepad"]) {
    const controls = normalizedGamepadControls(pad(id));
    assert.equal(controls.roll, 0.25);
    assert.equal(controls.forward, 0.5);
    assert.equal(controls.turn, 0.75);
    assert.deepEqual(controls.buttons, { stand: 0, tilt: 1, reset: 3 });
  }
});

test("recognizes representative EdgeTX and OpenTX CRSF radios", () => {
  for (const id of ["RadioMaster Boxer", "RadioMaster Zorro", "TX16S Joystick", "EdgeTX Pocket", "Jumper T-Pro OpenTX"]) {
    const controls = normalizedGamepadControls(pad(id, Array.from({ length: 8 }, (_, index) => index / 10)));
    assert.equal(controls.profile.family, "crsf-radio");
    assert.equal(controls.radioAxes.length, 8);
  }
});

test("custom mappings support non-standard axes, inversion, and buttons", () => {
  const controls = normalizedGamepadControls(pad("Odd USB Pad", [0.1, 0.2, 0.3, 0.4]), {
    rollAxis: 3, forwardAxis: 2, turnAxis: 1,
    invertRoll: true, invertForward: false, invertTurn: true,
    standButton: 5, tiltButton: 6, resetButton: 7,
  });
  assert.equal(controls.roll, -0.4);
  assert.equal(controls.forward, 0.3);
  assert.equal(controls.turn, -0.2);
  assert.deepEqual(controls.buttons, { stand: 5, tilt: 6, reset: 7 });
});

test("persisted mappings are bounded and malformed entries fall back safely", () => {
  const mapping = sanitizeGamepadMapping({ rollAxis: 999, forwardAxis: 4, standButton: -1, invertForward: false });
  assert.equal(mapping.rollAxis, 0);
  assert.equal(mapping.forwardAxis, 4);
  assert.equal(mapping.standButton, 0);
  assert.equal(mapping.invertForward, false);
  assert.deepEqual(readGamepadMappings(null), {});
  assert.equal(Object.keys(readGamepadMappings({ "Test Pad": mapping })).length, 1);
});
