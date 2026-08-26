export const DOMINO_BATTERY_CELL_COUNT = 4;
export const LIVE_BATTERY_ALERT_LEVELS = Object.freeze({
  LOW: "low",
  CRITICAL: "critical",
  DEPLETED: "depleted",
});

// Approximate rested LiPo state-of-charge curve. Voltage sag under servo load,
// temperature and pack age make this an estimate rather than a fuel gauge.
const LIPO_CHARGE_CURVE = [
  [3.30, 0],
  [3.50, 5],
  [3.60, 10],
  [3.70, 20],
  [3.75, 30],
  [3.80, 40],
  [3.85, 50],
  [3.90, 60],
  [3.95, 70],
  [4.00, 80],
  [4.10, 90],
  [4.20, 100],
];

function estimatedLipoChargePercent(cellVoltageV) {
  if (!Number.isFinite(cellVoltageV)) return null;
  if (cellVoltageV <= LIPO_CHARGE_CURVE[0][0]) return 0;
  if (cellVoltageV >= LIPO_CHARGE_CURVE.at(-1)[0]) return 100;
  for (let index = 1; index < LIPO_CHARGE_CURVE.length; index += 1) {
    const [upperVoltage, upperPercent] = LIPO_CHARGE_CURVE[index];
    if (cellVoltageV > upperVoltage) continue;
    const [lowerVoltage, lowerPercent] = LIPO_CHARGE_CURVE[index - 1];
    const fraction = (cellVoltageV - lowerVoltage) / (upperVoltage - lowerVoltage);
    return lowerPercent + fraction * (upperPercent - lowerPercent);
  }
  return null;
}

export function deriveLiveBatteryState(packVoltageV, cellCount = DOMINO_BATTERY_CELL_COUNT) {
  const validCellCount = Number.isInteger(cellCount) && cellCount > 0 ? cellCount : null;
  if (!Number.isFinite(packVoltageV) || packVoltageV <= 0 || !validCellCount) {
    return {
      cellCount: validCellCount,
      averageCellVoltageV: null,
      estimatedChargePercent: null,
    };
  }
  const averageCellVoltageV = packVoltageV / validCellCount;
  return {
    cellCount: validCellCount,
    averageCellVoltageV,
    estimatedChargePercent: estimatedLipoChargePercent(averageCellVoltageV),
  };
}

export function createLiveBatteryAlertState() {
  return {
    activeLevel: null,
    lowShown: false,
    criticalShown: false,
  };
}

export function liveBatteryCommandsBlocked(estimatedChargePercent) {
  return Number.isFinite(estimatedChargePercent) && estimatedChargePercent <= 0;
}

export function observeLiveBatteryAlert(state, estimatedChargePercent) {
  if (!state || typeof state !== "object") return null;
  if (!Number.isFinite(estimatedChargePercent)) {
    state.activeLevel = null;
    return null;
  }

  // Reset only above a small hysteresis margin so servo-load voltage sag does
  // not repeatedly reopen a warning around the same threshold.
  if (estimatedChargePercent > 12) state.lowShown = false;
  if (estimatedChargePercent > 7) state.criticalShown = false;

  if (estimatedChargePercent <= 0) {
    state.activeLevel = LIVE_BATTERY_ALERT_LEVELS.DEPLETED;
  } else if (estimatedChargePercent <= 5 && !state.criticalShown) {
    state.criticalShown = true;
    state.lowShown = true;
    state.activeLevel = LIVE_BATTERY_ALERT_LEVELS.CRITICAL;
  } else if (estimatedChargePercent <= 10 && !state.lowShown) {
    state.lowShown = true;
    state.activeLevel = LIVE_BATTERY_ALERT_LEVELS.LOW;
  } else if (state.activeLevel === LIVE_BATTERY_ALERT_LEVELS.DEPLETED) {
    state.activeLevel = null;
  }
  return state.activeLevel;
}

export function dismissLiveBatteryAlert(state) {
  if (!state || state.activeLevel === LIVE_BATTERY_ALERT_LEVELS.DEPLETED) return false;
  state.activeLevel = null;
  return true;
}
