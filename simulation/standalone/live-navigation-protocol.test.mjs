import assert from "node:assert/strict";
import test from "node:test";

import {
  createLiveNavigationCommand,
  validLiveNavigationAcknowledgement,
  validLiveNavigationCommand,
} from "./web/src/live-navigation-protocol.js";

const envelope = (action, payload = {}) => ({
  type: "live-navigation-command",
  action,
  requestId: `request-${action}`,
  timestampMs: 1_000,
  adapterId: "ardupilot-mavlink-1",
  sessionId: "session-1",
  payload,
});

test("navigation protocol accepts Rover modes, missions, and safety policies", () => {
  assert.equal(validLiveNavigationCommand(envelope("set-mode", { mode: "auto", modeCode: 10 })), true);
  assert.equal(validLiveNavigationCommand(envelope("upload-mission", {
    mission: [{ lat: -36.85, lon: 174.76, altM: 0, radiusM: 2, speedMps: 0.5, holdS: 1, label: "Start" }],
  })), true);
  assert.equal(validLiveNavigationCommand(envelope("set-obstacle-behavior", {
    enabled: true,
    stopDistanceM: 0.45,
    slowDistanceM: 1.2,
    maxSpeedMps: 0.5,
  })), true);
});

test("navigation protocol rejects unsafe or malformed payloads", () => {
  assert.equal(validLiveNavigationCommand(envelope("set-mode", { mode: "teleport" })), false);
  assert.equal(validLiveNavigationCommand(envelope("goto", { target: { lat: 91, lon: 174 } })), false);
  assert.equal(validLiveNavigationCommand(envelope("set-obstacle-behavior", {
    enabled: true,
    stopDistanceM: 2,
    slowDistanceM: 1,
    maxSpeedMps: 0.5,
  })), false);
});

test("navigation commands and acknowledgements retain request identity", () => {
  const command = createLiveNavigationCommand("set-home", { useCurrent: true }, "home-1", 2_000);
  assert.ok(command);
  assert.equal(command.requestId, "home-1");
  assert.equal(validLiveNavigationAcknowledgement({
    type: "live-navigation-ack",
    action: "set-home",
    requestId: "home-1",
    accepted: true,
    adapterId: "ardupilot-mavlink-1",
    sessionId: "session-1",
    state: { home: { lat: -36.85, lon: 174.76 } },
  }), true);
});
