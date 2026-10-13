import test from "node:test";
import assert from "node:assert/strict";
import { FirmwareLinkCoordinator } from "./firmware-link.mjs";

function harness(paired) {
  const calls = [];
  const firmware = { validateUpload: () => {}, startUpload: options => { calls.push("upload"); return options; } };
  const companion = { status: () => ({ running: true, transport: "usb", endpoint: "COM9" }),
    stop: async () => calls.push("stop"), start: async () => calls.push("start") };
  return { calls, coordinator: new FirmwareLinkCoordinator({ firmware, companion, pairedAdapter: () => paired }) };
}

test("firmware cannot interrupt calibration persistence or readback", async () => {
  const { calls, coordinator } = harness({ robotState: "disarmed", calibrationPending: true });
  await assert.rejects(coordinator.startUpload({}), /calibration.*read-back/);
  assert.deepEqual(calls, []);
  assert.equal(coordinator.busy, false);
});

test("firmware releases and restores the USB link once calibration verification is finished", async () => {
  const { calls, coordinator } = harness({ robotState: "disarmed", calibrationPending: false });
  assert.equal((await coordinator.startUpload({})).port, "COM9");
  assert.deepEqual(calls, ["stop", "upload"]);
  await coordinator.finish({ type: "upload" });
  assert.deepEqual(calls, ["stop", "upload", "start"]);
  assert.equal(coordinator.busy, false);
});
