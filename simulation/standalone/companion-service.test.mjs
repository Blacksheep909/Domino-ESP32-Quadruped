import assert from "node:assert/strict";
import test from "node:test";

import { CompanionService, selectCompanionPort } from "./companion-service.mjs";

test("automatic companion discovery prefers an ESP32 serial adapter", () => {
  const selected = selectCompanionPort([
    { port: "COM8", description: "RadioMaster Boxer" },
    { port: "COM4", description: "Silicon Labs CP210x USB to UART Bridge" },
    { port: "COM7", description: "Standard Serial over Bluetooth link" },
  ]);
  assert.equal(selected.port, "COM4");
});

test("Bluetooth discovery selects a Bluetooth serial endpoint", () => {
  const selected = selectCompanionPort([
    { port: "COM4", description: "CP210x USB UART" },
    { port: "COM7", description: "Standard Serial over Bluetooth link" },
  ], "bluetooth");
  assert.equal(selected.port, "COM7");
});

test("explicit robot port is honored without falling back to another device", () => {
  const ports = [{ port: "COM4" }, { port: "COM7" }];
  assert.equal(selectCompanionPort(ports, "usb", "com7").port, "COM7");
  assert.equal(selectCompanionPort(ports, "usb", "COM9"), null);
});

test("ArduPilot companion mode advertises MAVLink UDP and rejects serial discovery", async () => {
  const service = new CompanionService({
    firmwareService: { ports: async () => [] },
    relayUrl: "ws://127.0.0.1:8770/control",
    adapterEntry: "C:/companion/live-companion-adapter.mjs",
    protocol: "ardupilot",
    logger: {},
  });
  assert.equal(service.status().protocol, "ardupilot");
  const result = await service.discover("usb");
  assert.equal(result.started, false);
  assert.match(result.reason, /MAVLink UDP/i);
});
