import test from "node:test";
import assert from "node:assert/strict";

import {
  createLiveDiagnosticsState,
  liveDiagnosticBundle,
  liveDiagnosticsSnapshot,
  observeLiveDiagnosticPacket,
} from "./web/src/live-diagnostics-state.js";

const liveSnapshot = (fresh = true) => ({ expectedFresh: fresh, measuredFresh: fresh });
const packet = (sequence, diagnostics = {}) => ({
  type: "live-telemetry",
  sequence,
  expected: { timestampMs: 1000 + sequence },
  measured: { timestampMs: 1005 + sequence },
  power: { voltageV: 15.8, currentA: 2.1 },
  diagnostics,
});

test("tracks accepted, rejected, missing and rate-window packets", () => {
  const state = createLiveDiagnosticsState();
  observeLiveDiagnosticPacket(state, packet(10), true, 1_000);
  observeLiveDiagnosticPacket(state, packet(12), true, 1_100);
  observeLiveDiagnosticPacket(state, packet(11), false, 1_150);
  const snapshot = liveDiagnosticsSnapshot(state, liveSnapshot(), 1_200);
  assert.equal(snapshot.acceptedPackets, 2);
  assert.equal(snapshot.droppedPackets, 1);
  assert.equal(snapshot.rejectedPackets, 1);
  assert.equal(snapshot.packetRateHz, 10);
});

test("rejected packet bursts increment counters without flooding the event log", () => {
  const state = createLiveDiagnosticsState();
  observeLiveDiagnosticPacket(state, packet(1), false, 1_000);
  observeLiveDiagnosticPacket(state, packet(2), false, 1_020);
  observeLiveDiagnosticPacket(state, packet(3), false, 1_040);
  assert.equal(state.rejectedPackets, 3);
  assert.equal(state.events.filter((event) => event.message.startsWith("Rejected one or more")).length, 1);
  observeLiveDiagnosticPacket(state, packet(4), false, 2_001);
  assert.equal(state.events.filter((event) => event.message.startsWith("Rejected one or more")).length, 2);
});

test("identifies the first broken command-chain stage", () => {
  const state = createLiveDiagnosticsState();
  observeLiveDiagnosticPacket(state, packet(1, {
    acknowledgementHz: 49.9,
    gaitTargetValid: true,
    ikValid: false,
    jointLimitClips: 2,
    servoOutputChannels: 12,
  }), true, 1_000);
  const snapshot = liveDiagnosticsSnapshot(state, liveSnapshot(), 1_050);
  assert.equal(snapshot.firstBrokenStage.id, "ik");
  assert.equal(snapshot.stages.find((stage) => stage.id === "limits").status, "warning");
});

test("a stale transition is logged once and recovers cleanly", () => {
  const state = createLiveDiagnosticsState();
  observeLiveDiagnosticPacket(state, packet(1), true, 1_000);
  liveDiagnosticsSnapshot(state, liveSnapshot(), 1_100);
  liveDiagnosticsSnapshot(state, liveSnapshot(false), 2_100);
  liveDiagnosticsSnapshot(state, liveSnapshot(false), 2_200);
  assert.equal(state.staleTransitions, 1);
  assert.equal(state.events.filter((event) => event.message.includes("became stale")).length, 1);
});

test("diagnostic bundle includes context, current stages, events and packet summary", () => {
  const state = createLiveDiagnosticsState();
  observeLiveDiagnosticPacket(state, packet(7, { esp32LoopHz: 500, commandLatencyMs: 8, uptimeMs: 86_400_000 }), true, 1_000);
  const bundle = liveDiagnosticBundle(state, liveSnapshot(), { calibration: { schemaVersion: 1 } }, 1_100);
  assert.equal(bundle.schemaVersion, 1);
  assert.equal(bundle.context.calibration.schemaVersion, 1);
  assert.equal(bundle.diagnostics.lastPacket.sequence, 7);
  assert.equal(bundle.diagnostics.telemetry.esp32LoopHz, 500);
  assert.equal(bundle.diagnostics.telemetry.commandLatencyMs, 8);
  assert.equal(bundle.diagnostics.telemetry.uptimeMs, 86_400_000);
  assert.ok(bundle.diagnostics.events.length > 0);
});
