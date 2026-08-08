import assert from "node:assert/strict";
import test from "node:test";

import { decodeBoxerReport, rawAxisToMicroseconds } from "./boxer-hid.mjs";

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
