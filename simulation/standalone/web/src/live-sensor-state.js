const finite = (value) => {
  if (value === null || value === undefined || value === "") return null;
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : null;
};

const wrapAngle = (degrees) => {
  if (!Number.isFinite(degrees)) return null;
  return ((degrees + 180) % 360 + 360) % 360 - 180;
};

export function createLiveSensorCalibrationState() {
  return { rollZeroDeg: 0, pitchZeroDeg: 0, yawZeroDeg: 0, capturedAt: null };
}

export function createLiveSensorAttitudeFilterState() {
  return { initialized: false, rollDeg: 0, pitchDeg: 0, yawDeg: 0, updatedAt: 0 };
}

function shortestAngleDelta(fromDeg, toDeg) {
  return wrapAngle(toDeg - wrapAngle(fromDeg));
}

export function filterLiveSensorAttitude(filter, snapshot, now = Date.now()) {
  if (!filter || !snapshot?.online) {
    if (filter) filter.initialized = false;
    return snapshot;
  }
  if (!filter.initialized) {
    filter.initialized = true;
    filter.rollDeg = snapshot.rollDeg;
    filter.pitchDeg = snapshot.pitchDeg;
    filter.yawDeg = snapshot.yawDeg ?? 0;
    filter.updatedAt = now;
  } else {
    const elapsedMs = Math.max(1, Math.min(250, now - filter.updatedAt));
    const blend = 1 - Math.exp(-elapsedMs / 110);
    const maximumStep = 240 * elapsedMs / 1000;
    const rollStep = Math.max(-maximumStep, Math.min(maximumStep,
      shortestAngleDelta(filter.rollDeg, snapshot.rollDeg) * blend));
    const pitchStep = Math.max(-maximumStep, Math.min(maximumStep,
      shortestAngleDelta(filter.pitchDeg, snapshot.pitchDeg) * blend));
    const yawStep = snapshot.yawDeg === null ? 0 : Math.max(-maximumStep, Math.min(maximumStep,
      shortestAngleDelta(filter.yawDeg, snapshot.yawDeg) * blend));
    filter.rollDeg += rollStep;
    filter.pitchDeg += pitchStep;
    filter.yawDeg += yawStep;
    filter.updatedAt = now;
  }
  return { ...snapshot, displayRollDeg: filter.rollDeg, displayPitchDeg: filter.pitchDeg, displayYawDeg: filter.yawDeg };
}

export function liveSensorSnapshot(comparison, diagnostics, calibration) {
  const body = comparison?.measured?.body;
  const rollRawDeg = finite(body?.rollDeg);
  const pitchRawDeg = finite(body?.pitchDeg);
  const yawRawDeg = finite(body?.yawDeg);
  const axG = finite(diagnostics?.imuAxG);
  const ayG = finite(diagnostics?.imuAyG);
  const azG = finite(diagnostics?.imuAzG);
  const online = comparison?.measuredFresh === true && diagnostics?.imuOnline === true &&
    rollRawDeg !== null && pitchRawDeg !== null;
  const gravityMagnitudeG = [axG, ayG, azG].every((value) => value !== null)
    ? Math.sqrt(axG * axG + ayG * ayG + azG * azG)
    : null;
  return {
    online,
    rollRawDeg,
    pitchRawDeg,
    yawRawDeg,
    rollDeg: online ? wrapAngle(rollRawDeg - finite(calibration?.rollZeroDeg)) : null,
    pitchDeg: online ? wrapAngle(pitchRawDeg - finite(calibration?.pitchZeroDeg)) : null,
    yawDeg: online && yawRawDeg !== null ? wrapAngle(yawRawDeg - finite(calibration?.yawZeroDeg)) : null,
    axG,
    ayG,
    azG,
    gravityMagnitudeG,
    levelCaptured: Number.isFinite(calibration?.capturedAt),
    capturedAt: calibration?.capturedAt ?? null,
  };
}

export function captureLiveSensorLevel(calibration, snapshot, capturedAt = Date.now()) {
  if (!calibration || !snapshot?.online) return false;
  calibration.rollZeroDeg = snapshot.rollRawDeg;
  calibration.pitchZeroDeg = snapshot.pitchRawDeg;
  calibration.yawZeroDeg = snapshot.yawRawDeg ?? 0;
  calibration.capturedAt = capturedAt;
  return true;
}

export function resetLiveSensorLevel(calibration) {
  if (!calibration) return false;
  calibration.rollZeroDeg = 0;
  calibration.pitchZeroDeg = 0;
  calibration.yawZeroDeg = 0;
  calibration.capturedAt = null;
  return true;
}
