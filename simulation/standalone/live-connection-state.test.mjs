import test from "node:test";
import assert from "node:assert/strict";

import {
  acceptLiveAdapterAnnouncement,
  acceptLiveConnectionAcknowledgement,
  createLiveConnectionCommand,
  createLiveConnectionState,
  liveConnectionEnvelope,
  liveConnectionIsReady,
  markLiveConnectionPending,
  pruneLiveAdapters,
  setLiveConnectionBridge,
  telemetryBelongsToLiveConnection,
  visibleLiveAdapters,
} from "./web/src/live-connection-state.js";
import {
  validLiveAdapterAnnouncement,
  validLiveConnectionAcknowledgement,
  validLiveConnectionCommand,
} from "./web/src/live-connection-protocol.js";

const announcement = (timestampMs = 1_000) => ({
  type: "live-adapter-announce",
  adapterId: "domino-adapter-a",
  name: "Domino workshop adapter",
  transport: "wifi",
  state: "available",
  timestampMs,
  signalPercent: 82,
  robot: { id: "domino-01", name: "Domino", firmwareVersion: "0.4.0" },
  capabilities: { telemetry: true, calibration: true, gaitProfiles: true, persistentProfiles: true },
});

test("adapter announcements are bounded, retained and transport-filtered", () => {
  const state = createLiveConnectionState();
  assert.equal(validLiveAdapterAnnouncement(announcement()), true);
  assert.equal(acceptLiveAdapterAnnouncement(state, announcement(), 1_010), true);
  assert.equal(state.selectedAdapterId, "domino-adapter-a");
  assert.equal(visibleLiveAdapters(state, 1_020).length, 1);
  state.transportFilter = "bluetooth";
  assert.equal(visibleLiveAdapters(state, 1_020).length, 0);
});

test("connect uses a read-only handshake and yields a session-bound link", () => {
  const state = createLiveConnectionState();
  setLiveConnectionBridge(state, true);
  acceptLiveAdapterAnnouncement(state, announcement(), 1_010);
  const command = createLiveConnectionCommand(state, "connect", "request-1", 1_020);
  assert.equal(validLiveConnectionCommand(command), true);
  assert.equal(command.safety.commandsBlockedUntilStateKnown, true);
  markLiveConnectionPending(state, command);
  const acknowledgement = {
    type: "live-connection-ack",
    action: "connect",
    requestId: "request-1",
    accepted: true,
    adapterId: "domino-adapter-a",
    sessionId: "session-a",
    robotState: "disarmed",
  };
  assert.equal(validLiveConnectionAcknowledgement(acknowledgement), true);
  assert.equal(acceptLiveConnectionAcknowledgement(state, acknowledgement, 1_030), true);
  assert.equal(liveConnectionIsReady(state, 1_040), true);
  assert.deepEqual(liveConnectionEnvelope(state), { adapterId: "domino-adapter-a", sessionId: "session-a" });
});

test("telemetry must match the selected adapter and negotiated session", () => {
  const state = createLiveConnectionState();
  setLiveConnectionBridge(state, true);
  acceptLiveAdapterAnnouncement(state, announcement(), 1_010);
  const command = createLiveConnectionCommand(state, "connect", "request-1", 1_020);
  markLiveConnectionPending(state, command);
  acceptLiveConnectionAcknowledgement(state, {
    type: "live-connection-ack", action: "connect", requestId: "request-1", accepted: true,
    adapterId: "domino-adapter-a", sessionId: "session-a", robotState: "disarmed",
  }, 1_030);
  assert.equal(telemetryBelongsToLiveConnection(state, { adapterId: "domino-adapter-a", sessionId: "session-a" }, 1_040), true);
  assert.equal(telemetryBelongsToLiveConnection(state, { adapterId: "domino-adapter-a", sessionId: "other" }, 1_040), false);
  assert.equal(telemetryBelongsToLiveConnection(state, { adapterId: "other", sessionId: "session-a" }, 1_040), false);
});

test("a stale adapter tears down the engineering session", () => {
  const state = createLiveConnectionState();
  setLiveConnectionBridge(state, true);
  acceptLiveAdapterAnnouncement(state, announcement(), 1_000);
  const command = createLiveConnectionCommand(state, "connect", "request-1", 1_010);
  markLiveConnectionPending(state, command);
  acceptLiveConnectionAcknowledgement(state, {
    type: "live-connection-ack", action: "connect", requestId: "request-1", accepted: true,
    adapterId: "domino-adapter-a", sessionId: "session-a", robotState: "disarmed",
  }, 1_020);
  assert.equal(pruneLiveAdapters(state, 5_100), true);
  assert.equal(state.phase, "lost");
  assert.equal(state.sessionId, "");
  assert.equal(liveConnectionIsReady(state, 5_100), false);
});

test("rejected connection acknowledgements leave commands locked", () => {
  const state = createLiveConnectionState();
  setLiveConnectionBridge(state, true);
  acceptLiveAdapterAnnouncement(state, announcement(), 1_000);
  const command = createLiveConnectionCommand(state, "connect", "request-1", 1_010);
  markLiveConnectionPending(state, command);
  assert.equal(acceptLiveConnectionAcknowledgement(state, {
    type: "live-connection-ack", action: "connect", requestId: "request-1", accepted: false,
    adapterId: "domino-adapter-a", reason: "Robot identity mismatch",
  }, 1_020), true);
  assert.equal(state.phase, "disconnected");
  assert.equal(state.sessionId, "");
});
