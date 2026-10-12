import assert from "node:assert/strict";
import test from "node:test";

import {
  createLiveBatteryAlertState,
  deriveLiveBatteryState,
  dismissLiveBatteryAlert,
  DOMINO_BATTERY_CELL_COUNT,
  LIVE_BATTERY_MIN_DETECTABLE_CELL_VOLTAGE_V,
  LIVE_BATTERY_ALERT_LEVELS,
  liveBatteryCommandsBlocked,
  observeLiveBatteryAlert,
} from "./web/src/live-battery-state.js";

test("derives an honest average cell voltage from the measured 4S pack", () => {
  const battery = deriveLiveBatteryState(15.2);
  assert.equal(DOMINO_BATTERY_CELL_COUNT, 4);
  assert.equal(battery.cellCount, 4);
  assert.equal(battery.averageCellVoltageV, 3.8);
  assert.equal(battery.estimatedChargePercent, 40);
});

test("bounds the approximate LiPo charge estimate", () => {
  assert.equal(deriveLiveBatteryState(13.2).estimatedChargePercent, 0);
  assert.equal(deriveLiveBatteryState(16.8).estimatedChargePercent, 100);
  assert.equal(deriveLiveBatteryState(16.0).estimatedChargePercent, 80);
});

test("does not invent battery state without a valid pack sample", () => {
  assert.equal(deriveLiveBatteryState(null).averageCellVoltageV, null);
  assert.equal(deriveLiveBatteryState(Number.NaN).estimatedChargePercent, null);
});

test("does not mistake a USB-only divider reading for a depleted 4S pack", () => {
  const battery = deriveLiveBatteryState(2.5);
  assert.equal(LIVE_BATTERY_MIN_DETECTABLE_CELL_VOLTAGE_V, 1.5);
  assert.equal(battery.packDetected, false);
  assert.equal(battery.averageCellVoltageV, null);
  assert.equal(battery.estimatedChargePercent, null);
  assert.equal(liveBatteryCommandsBlocked(battery.estimatedChargePercent), false);
  assert.equal(observeLiveBatteryAlert(createLiveBatteryAlertState(), battery.estimatedChargePercent), null);
  assert.equal(deriveLiveBatteryState(6).packDetected, true);
});

test("retains a critical low-voltage indication if the robot is armed", () => {
  const battery = deriveLiveBatteryState(2.5, 4, { armed: true });
  assert.equal(battery.packDetected, true);
  assert.equal(battery.estimatedChargePercent, 0);
  assert.equal(liveBatteryCommandsBlocked(battery.estimatedChargePercent), true);
});

test("raises 10 and 5 percent warnings once per downward crossing", () => {
  const state = createLiveBatteryAlertState();
  assert.equal(observeLiveBatteryAlert(state, 11), null);
  assert.equal(observeLiveBatteryAlert(state, 10), LIVE_BATTERY_ALERT_LEVELS.LOW);
  assert.equal(dismissLiveBatteryAlert(state), true);
  assert.equal(observeLiveBatteryAlert(state, 9), null);
  assert.equal(observeLiveBatteryAlert(state, 5), LIVE_BATTERY_ALERT_LEVELS.CRITICAL);
  assert.equal(dismissLiveBatteryAlert(state), true);
  assert.equal(observeLiveBatteryAlert(state, 4), null);
});

test("uses hysteresis before allowing a warning to be shown again", () => {
  const state = createLiveBatteryAlertState();
  observeLiveBatteryAlert(state, 10);
  dismissLiveBatteryAlert(state);
  assert.equal(observeLiveBatteryAlert(state, 11), null);
  assert.equal(observeLiveBatteryAlert(state, 13), null);
  assert.equal(observeLiveBatteryAlert(state, 10), LIVE_BATTERY_ALERT_LEVELS.LOW);
});

test("zero percent is blocking and cannot be dismissed", () => {
  const state = createLiveBatteryAlertState();
  assert.equal(observeLiveBatteryAlert(state, 0), LIVE_BATTERY_ALERT_LEVELS.DEPLETED);
  assert.equal(liveBatteryCommandsBlocked(0), true);
  assert.equal(dismissLiveBatteryAlert(state), false);
  assert.equal(observeLiveBatteryAlert(state, 1), LIVE_BATTERY_ALERT_LEVELS.CRITICAL);
  assert.equal(liveBatteryCommandsBlocked(1), false);
  assert.equal(dismissLiveBatteryAlert(state), true);
});
