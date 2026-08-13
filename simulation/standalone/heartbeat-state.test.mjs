import assert from "node:assert/strict";
import test from "node:test";

import {
  acceptHeartbeatAcknowledgement,
  createHeartbeatMessage,
  createHeartbeatState,
  heartbeatStatus,
  markHeartbeatSocketClosed,
  markHeartbeatSocketOpen,
  packetAgeMs,
} from "./web/src/heartbeat-state.js";

test("a new heartbeat progresses from connecting to degraded and fault", () => {
  const state = createHeartbeatState();
  markHeartbeatSocketOpen(state, 1_000);
  assert.equal(heartbeatStatus(state, 1_100), "connecting");
  assert.equal(heartbeatStatus(state, 2_600), "degraded");
  assert.equal(heartbeatStatus(state, 4_000), "fault");
});

test("a valid acknowledgement records smoothed round-trip time", () => {
  const state = createHeartbeatState();
  markHeartbeatSocketOpen(state, 1_000);
  const first = createHeartbeatMessage(state, 1_100);
  assert.equal(acceptHeartbeatAcknowledgement(state, first, 1_120), true);
  assert.equal(state.roundTripMs, 20);
  const second = createHeartbeatMessage(state, 1_500);
  assert.equal(acceptHeartbeatAcknowledgement(state, second, 1_510), true);
  assert.equal(state.roundTripMs, 17);
  assert.equal(heartbeatStatus(state, 1_510), "connected");
});

test("stale and malformed acknowledgements are ignored", () => {
  const state = createHeartbeatState();
  markHeartbeatSocketOpen(state, 1_000);
  const message = createHeartbeatMessage(state, 1_100);
  assert.equal(acceptHeartbeatAcknowledgement(state, message, 1_120), true);
  assert.equal(acceptHeartbeatAcknowledgement(state, message, 1_130), false);
  assert.equal(acceptHeartbeatAcknowledgement(state, { sequence: 2 }, 1_140), false);
});

test("closing the socket is immediately disconnected", () => {
  const state = createHeartbeatState();
  markHeartbeatSocketOpen(state, 1_000);
  markHeartbeatSocketClosed(state);
  assert.equal(heartbeatStatus(state, 1_001), "disconnected");
});

test("packet age rejects missing timestamps", () => {
  assert.equal(packetAgeMs(null, 2_000), null);
  assert.equal(packetAgeMs(1_750, 2_000), 250);
});
