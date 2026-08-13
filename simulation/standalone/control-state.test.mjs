import assert from "node:assert/strict";
import test from "node:test";

import {
  clientControlIsFresh,
  hasActiveClient,
  radioInputIsFresh,
  releaseClientControl,
} from "./control-state.mjs";

test("only visible control owners block radio input", () => {
  const hiddenManualTab = {
    controlActive: true,
    controlMode: "interactive",
    manualOverride: true,
    lastControlAt: 9_900,
  };
  releaseClientControl(hiddenManualTab);
  assert.equal(hasActiveClient([hiddenManualTab], (socket) => socket.manualOverride), false);
  assert.equal(hiddenManualTab.controlMode, null);
  assert.equal(hiddenManualTab.manualOverride, false);
});

test("a tab that stops sending controls loses ownership", () => {
  const socket = { controlActive: true, manualOverride: true, lastControlAt: 9_600 };
  assert.equal(clientControlIsFresh(socket, 10_000), false);
  assert.equal(hasActiveClient([socket], (client) => client.manualOverride, 10_000), false);
});

test("stale or incomplete Boxer reports do not own controls", () => {
  const now = 10_000;
  assert.equal(radioInputIsFresh({ connected: true, channels: Array(16), updatedAt: now - 100 }, now), true);
  assert.equal(radioInputIsFresh({ connected: true, channels: Array(16), updatedAt: now - 300 }, now), false);
  assert.equal(radioInputIsFresh({ connected: true, channels: null, updatedAt: now }, now), false);
  assert.equal(radioInputIsFresh({ connected: false, channels: Array(16), updatedAt: now }, now), false);
});
