import test from "node:test";
import assert from "node:assert/strict";

import {
  acceptLiveControllerTelemetry,
  createLiveControllerState,
  liveControllerDiagnosticExport,
  liveControllerSnapshot,
  sanitizeLiveControllerTelemetry,
} from "./web/src/live-controller-state.js";

const controller = (overrides = {}) => ({
  source: "boxer-elrs",
  transmitterName: "RadioMaster Boxer",
  receiverName: "ExpressLRS EP1",
  frameTimestampMs: 1_000,
  channelsUs: Array(16).fill(1_500),
  packetRateHz: 150,
  frameLossCount: 2,
  failsafe: false,
  failsafeCount: 0,
  linkQualityPercent: 96,
  rssi1Dbm: -61,
  rssi2Dbm: -65,
  activeAntenna: 1,
  snrDb: 8,
  rfMode: "250Hz",
  txPowerMw: 250,
  receiverVoltageV: 5.02,
  ...overrides,
});
const packet = (payload) => ({ type: "live-telemetry", controller: payload });

test("controller telemetry requires all 16 bounded channels and core RF fields", () => {
  assert.ok(sanitizeLiveControllerTelemetry(controller()));
  assert.equal(sanitizeLiveControllerTelemetry(controller({ channelsUs: Array(8).fill(1_500) })), null);
  assert.equal(sanitizeLiveControllerTelemetry(controller({ channelsUs: [...Array(15).fill(1_500), 3_000] })), null);
  assert.equal(sanitizeLiveControllerTelemetry(controller({ linkQualityPercent: 101 })), null);
});

test("fresh Boxer/ELRS frames produce a ready link and stale frames do not", () => {
  const state = createLiveControllerState();
  assert.equal(acceptLiveControllerTelemetry(state, packet(controller()), 1_020), true);
  assert.equal(liveControllerSnapshot(state, 1_030).linkReady, true);
  assert.equal(liveControllerSnapshot(state, 1_600).linkReady, false);
  assert.equal(liveControllerSnapshot(state, 1_600).quality, "offline");
});

test("failsafe and loss counter transitions create bounded evidence", () => {
  const state = createLiveControllerState();
  acceptLiveControllerTelemetry(state, packet(controller()), 1_020);
  acceptLiveControllerTelemetry(state, packet(controller({ frameTimestampMs: 1_100, failsafe: true, failsafeCount: 1, frameLossCount: 5 })), 1_120);
  assert.equal(liveControllerSnapshot(state, 1_130).quality, "failsafe");
  assert.ok(state.events.some((event) => event.message.includes("failsafe asserted")));
  assert.ok(state.events.some((event) => event.message.includes("additional CRSF")));
});

test("RF thresholds distinguish good, degraded, and poor links", () => {
  const state = createLiveControllerState();
  acceptLiveControllerTelemetry(state, packet(controller({ linkQualityPercent: 78 })), 1_020);
  assert.equal(liveControllerSnapshot(state, 1_030).quality, "degraded");
  acceptLiveControllerTelemetry(state, packet(controller({ frameTimestampMs: 1_100, linkQualityPercent: 45 })), 1_120);
  assert.equal(liveControllerSnapshot(state, 1_130).quality, "poor");
  assert.equal(liveControllerSnapshot(state, 1_130).linkReady, false);
});

test("diagnostic export preserves raw telemetry and controller transition history", () => {
  const state = createLiveControllerState();
  acceptLiveControllerTelemetry(state, packet(controller()), 1_020);
  const exported = liveControllerDiagnosticExport(state, 1_030);
  assert.equal(exported.linkReady, true);
  assert.equal(exported.telemetry.channelsUs.length, 16);
  assert.ok(exported.events.length > 0);
});
