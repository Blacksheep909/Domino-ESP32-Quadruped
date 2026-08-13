import test from "node:test";
import assert from "node:assert/strict";

import {
  acceptLiveSafetyAcknowledgement,
  acceptLiveSafetyHeartbeatAcknowledgement,
  beginLiveArmHold,
  createLiveSafetyCommand,
  createLiveSafetyHeartbeat,
  createLiveSafetyState,
  liveSafetyCanArm,
  markLiveSafetyPending,
  setLiveSafetyRobotState,
  tickLiveSafetyWatchdog,
  updateLiveArmHold,
} from "./web/src/live-safety-state.js";
import {
  LIVE_ARM_HOLD_MS,
  validLiveSafetyCommand,
  validLiveSafetyHeartbeat,
} from "./web/src/live-safety-protocol.js";

const connection = { adapterId: "adapter-a", sessionId: "session-a" };
const safeContext = { connectionReady: true, telemetryFresh: true, driveLinkAlive: true };

test("arming requires disarmed state, telemetry, drive link and full hold duration", () => {
  const state = createLiveSafetyState();
  setLiveSafetyRobotState(state, "disarmed");
  assert.equal(liveSafetyCanArm(state, { ...safeContext, driveLinkAlive: false }), false);
  assert.equal(beginLiveArmHold(state, safeContext, 1_000), true);
  assert.equal(updateLiveArmHold(state, safeContext, 1_000 + LIVE_ARM_HOLD_MS - 1).complete, false);
  assert.equal(updateLiveArmHold(state, safeContext, 1_000 + LIVE_ARM_HOLD_MS).complete, true);
  const command = createLiveSafetyCommand(state, "arm", connection, "arm-1", 2_500);
  assert.equal(validLiveSafetyCommand(command), true);
  assert.equal(command.safety.requiresDriveLink, true);
});

test("a changing prerequisite cancels an in-progress hold", () => {
  const state = createLiveSafetyState();
  setLiveSafetyRobotState(state, "disarmed");
  beginLiveArmHold(state, safeContext, 1_000);
  const result = updateLiveArmHold(state, { ...safeContext, telemetryFresh: false }, 1_500);
  assert.equal(result.active, false);
  assert.equal(state.armHoldStartedAt, 0);
});

test("arm acknowledgement starts a session-bound watchdog heartbeat", () => {
  const state = createLiveSafetyState();
  setLiveSafetyRobotState(state, "disarmed");
  beginLiveArmHold(state, safeContext, 1_000);
  updateLiveArmHold(state, safeContext, 2_500);
  const command = createLiveSafetyCommand(state, "arm", connection, "arm-1", 2_500);
  markLiveSafetyPending(state, command);
  assert.equal(acceptLiveSafetyAcknowledgement(state, {
    type: "live-safety-ack", action: "arm", requestId: "arm-1", accepted: true,
    ...connection, robotState: "armed",
  }, connection, 2_510), true);
  const heartbeat = createLiveSafetyHeartbeat(state, connection, true, 2_600);
  assert.equal(validLiveSafetyHeartbeat(heartbeat), true);
  assert.equal(createLiveSafetyHeartbeat(state, connection, false, 2_600), null);
});

test("heartbeat acknowledgements refresh the watchdog and stale ones trip it", () => {
  const state = createLiveSafetyState();
  state.robotState = "armed";
  assert.equal(acceptLiveSafetyHeartbeatAcknowledgement(state, {
    type: "live-safety-heartbeat-ack", ...connection, sequence: 3,
    robotState: "armed", watchdogRemainingMs: 400,
  }, connection, 1_000), true);
  assert.equal(tickLiveSafetyWatchdog(state, 1_399), false);
  assert.equal(tickLiveSafetyWatchdog(state, 1_401), true);
  assert.equal(state.robotState, "watchdog");
  assert.equal(setLiveSafetyRobotState(state, "armed"), false);
  assert.equal(state.robotState, "watchdog");
  assert.equal(setLiveSafetyRobotState(state, "disarmed"), true);
  assert.equal(state.watchdogTripped, false);
});

test("E-stop is valid without arm prerequisites and rejected replies preserve reported state", () => {
  const state = createLiveSafetyState();
  state.robotState = "disarmed";
  const command = createLiveSafetyCommand(state, "estop", connection, "stop-1", 2_000);
  assert.equal(validLiveSafetyCommand(command), true);
  markLiveSafetyPending(state, command);
  acceptLiveSafetyAcknowledgement(state, {
    type: "live-safety-ack", action: "estop", requestId: "stop-1", accepted: false,
    ...connection, robotState: "fault", reason: "Hardware line unavailable",
  }, connection, 2_010);
  assert.equal(state.robotState, "fault");
  assert.equal(state.status, "Hardware line unavailable");
});
