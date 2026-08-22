import test from "node:test";
import assert from "node:assert/strict";

import {
  acceptLiveManualAuthorityAcknowledgement,
  beginLiveManualDeadman,
  createLiveManualAuthorityCommand,
  createLiveManualControlFrame,
  createLiveManualControlState,
  endLiveManualDeadman,
  liveManualCanRequest,
  markLiveManualPending,
  setLiveManualSupport,
  tickLiveManualControl,
  updateLiveManualAxes,
} from "./web/src/live-manual-control-state.js";
import {
  validLiveManualAuthorityCommand,
  validLiveManualControlFrame,
} from "./web/src/live-manual-control-protocol.js";

const connection = { adapterId: "adapter-a", sessionId: "session-a" };
const safeContext = {
  connectionReady: true,
  robotState: "armed",
  telemetryFresh: true,
  controllerLinkReady: true,
  workspaceActive: true,
};

function authorizedState() {
  const state = createLiveManualControlState();
  setLiveManualSupport(state, true);
  state.safetyConfirmed = true;
  const command = createLiveManualAuthorityCommand(state, "request-authority", connection, "request-1", 1_000);
  markLiveManualPending(state, command);
  acceptLiveManualAuthorityAcknowledgement(state, {
    type: "live-manual-authority-ack", action: "request-authority", requestId: "request-1",
    accepted: true, ...connection, authorityToken: "authority-a", leaseMs: 30_000, robotState: "armed",
  }, connection, 1_010);
  return state;
}

test("authority request requires explicit consent and every safety prerequisite", () => {
  const state = createLiveManualControlState();
  setLiveManualSupport(state, true);
  assert.equal(liveManualCanRequest(state, safeContext), false);
  state.safetyConfirmed = true;
  assert.equal(liveManualCanRequest(state, { ...safeContext, controllerLinkReady: false }), false);
  assert.equal(liveManualCanRequest(state, safeContext), true);
  const command = createLiveManualAuthorityCommand(state, "request-authority", connection, "request-1", 1_000);
  assert.equal(validLiveManualAuthorityCommand(command), true);
  assert.equal(command.safety.neutralOnRelease, true);
});

test("control frames only stream while deadman is held", () => {
  const state = authorizedState();
  updateLiveManualAxes(state, { forward: 0.7, turn: -0.25, mode: "careful" });
  assert.equal(beginLiveManualDeadman(state, safeContext, 1_020), true);
  const active = createLiveManualControlFrame(state, connection, 1_030);
  assert.equal(validLiveManualControlFrame(active), true);
  assert.equal(active.deadman, true);
  assert.equal(active.axes.forward, 0.7);
  endLiveManualDeadman(state);
  const neutral = createLiveManualControlFrame(state, connection, 1_040, true);
  assert.equal(validLiveManualControlFrame(neutral), true);
  assert.deepEqual(neutral.axes, { forward: 0, turn: 0, roll: 0, pitch: 0, yaw: 0, bodyX: 0, bodyY: 0, height: 0 });
});

test("axes are bounded and neutral frames always command stand", () => {
  const state = authorizedState();
  updateLiveManualAxes(state, { forward: 4, turn: -4, roll: 2, pitch: -2, yaw: 3, bodyX: -3, bodyY: 4, height: -2, mode: "trot" });
  assert.deepEqual(state.axes, { forward: 1, turn: -1, roll: 1, pitch: -1, yaw: 1, bodyX: -1, bodyY: 1, height: -1 });
  const neutral = createLiveManualControlFrame(state, connection, 1_050, true);
  assert.equal(neutral.mode, "stand");
  assert.equal(neutral.neutral, true);
});

test("authority is revoked when the lease or a safety prerequisite expires", () => {
  const state = authorizedState();
  assert.equal(tickLiveManualControl(state, safeContext, 1_020), false);
  assert.equal(tickLiveManualControl(state, { ...safeContext, workspaceActive: false }, 1_030), true);
  assert.equal(state.authorityToken, "");
  assert.equal(state.deadmanActive, false);
});

test("release authority command carries the granted token", () => {
  const state = authorizedState();
  const command = createLiveManualAuthorityCommand(state, "release-authority", connection, "release-1", 1_100);
  assert.equal(validLiveManualAuthorityCommand(command), true);
  assert.equal(command.authorityToken, "authority-a");
});
