import assert from "node:assert/strict";
import test from "node:test";

import { decodeBoxerReport, isCrsfTransmitterDevice, rawAxisToMicroseconds } from "./boxer-hid.mjs";

test("decodes the live EdgeTX Boxer report layout", () => {
  const report = Buffer.from("0000000004000455020004000000040000000021", "hex");
  const decoded = decodeBoxerReport(report);

  assert.deepEqual(decoded.axes, [1024, 1024, 597, 1024, 0, 1024, 0, 0]);
  assert.deepEqual(decoded.channels.slice(0, 4), [1500, 1500, 1292, 1500]);
  assert.equal(decoded.battery, 33);
  assert.equal(decoded.buttonBits, 0);
  assert.equal(decoded.layout, "buttons-first");
});

test("maps the EdgeTX axis range onto CRSF-style microseconds", () => {
  assert.equal(rawAxisToMicroseconds(0), 1000);
  assert.equal(rawAxisToMicroseconds(1024), 1500);
  assert.equal(rawAxisToMicroseconds(2048), 2000);
});

test("detects supported RadioMaster, EdgeTX, OpenTX, and Jumper joystick devices", () => {
  const joystick = { usagePage: 1, usage: 4 };
  assert.equal(isCrsfTransmitterDevice({ ...joystick, product: "RadioMaster TX16S" }), true);
  assert.equal(isCrsfTransmitterDevice({ ...joystick, product: "EdgeTX Joystick" }), true);
  assert.equal(isCrsfTransmitterDevice({ ...joystick, manufacturer: "Jumper", product: "T-Pro" }), true);
  assert.equal(isCrsfTransmitterDevice({ ...joystick, product: "Xbox Wireless Controller" }), false);
});
