export const LIVE_SERVO_CHANNELS = Object.freeze([0, 1, 2, 3, 4, 7, 8, 9, 10, 11, 14, 15]);
export const LIVE_STREAM_FRESH_MS = 1_000;

const finiteNumber = (value) => {
  if (value === null || value === undefined || value === "") return null;
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : null;
};

function sanitizeBody(body, allowPartial = false) {
  if (!body || typeof body !== "object") return null;
  const rollDeg = finiteNumber(body.rollDeg);
  const pitchDeg = finiteNumber(body.pitchDeg);
  const yawDeg = finiteNumber(body.yawDeg);
  const heightMm = finiteNumber(body.heightMm);
  const values = [rollDeg, pitchDeg, yawDeg, heightMm];
  if (allowPartial ? values.every((value) => value === null) : values.some((value) => value === null)) return null;
  return { rollDeg, pitchDeg, yawDeg, heightMm };
}

function sanitizePose(pose, receivedAt, allowPartial = false) {
  if (!pose || typeof pose !== "object") return null;
  const timestampMs = finiteNumber(pose.timestampMs);
  if (timestampMs === null || timestampMs <= 0) return null;
  let servoAngleDeg = null;
  if (Array.isArray(pose.servoAngleDeg) && pose.servoAngleDeg.length >= 16) {
    servoAngleDeg = pose.servoAngleDeg.slice(0, 16).map(finiteNumber);
    if (LIVE_SERVO_CHANNELS.some((channel) => servoAngleDeg[channel] === null)) servoAngleDeg = null;
  }
  if (allowPartial && Object.hasOwn(pose, "servoAngleDeg") && !servoAngleDeg) return null;
  if (!allowPartial && !servoAngleDeg) return null;

  let servoPulseUs = null;
  if (Array.isArray(pose.servoPulseUs) && pose.servoPulseUs.length >= 16) {
    servoPulseUs = pose.servoPulseUs.slice(0, 16).map(finiteNumber);
    if (LIVE_SERVO_CHANNELS.some((channel) => servoPulseUs[channel] === null)) servoPulseUs = null;
  }
  if (Object.hasOwn(pose, "servoPulseUs") && !servoPulseUs) return null;

  let servoPhysicalChannel = null;
  if (Array.isArray(pose.servoPhysicalChannel) && pose.servoPhysicalChannel.length >= 16) {
    servoPhysicalChannel = pose.servoPhysicalChannel.slice(0, 16).map((value) => {
      const channel = finiteNumber(value);
      return Number.isInteger(channel) && channel >= 0 && channel < 16 ? channel : null;
    });
    if (LIVE_SERVO_CHANNELS.some((channel) => servoPhysicalChannel[channel] === null)) servoPhysicalChannel = null;
  }
  if (Object.hasOwn(pose, "servoPhysicalChannel") && !servoPhysicalChannel) return null;
  const body = sanitizeBody(pose.body, allowPartial);
  if (!body) return null;
  let footTargetMm = null;
  if (Array.isArray(pose.footTargetMm) && pose.footTargetMm.length === 4) {
    footTargetMm = pose.footTargetMm.map((target) => Array.isArray(target) && target.length === 3
      ? target.map(finiteNumber)
      : null);
    if (footTargetMm.some((target) => !target || target.some((value) => value === null))) return null;
  } else if (Object.hasOwn(pose, "footTargetMm")) {
    return null;
  }
  return { timestampMs, receivedAt, servoAngleDeg, servoPulseUs, servoPhysicalChannel, footTargetMm, body };
}

function sanitizePower(power, receivedAt) {
  if (!power || typeof power !== "object") return null;
  const voltageV = finiteNumber(power.voltageV);
  const currentA = finiteNumber(power.currentA);
  if (voltageV === null || currentA === null || voltageV < 0 || currentA < 0) return null;
  const suppliedPower = finiteNumber(power.powerW);
  return {
    receivedAt,
    voltageV,
    currentA,
    powerW: suppliedPower === null ? voltageV * currentA : suppliedPower,
  };
}

export function createLiveTelemetryState() {
  return {
    sequence: -1,
    expected: null,
    measured: null,
    power: null,
    lastRobotPacketAt: 0,
  };
}

export function acceptLiveTelemetryPacket(state, packet, receivedAt = Date.now()) {
  if (!state || !packet || packet.type !== "live-telemetry") return false;
  const sequence = Number(packet.sequence);
  if (!Number.isSafeInteger(sequence) || sequence <= state.sequence) return false;
  const expected = sanitizePose(packet.expected, receivedAt);
  const measured = sanitizePose(packet.measured, receivedAt, true);
  const power = sanitizePower(packet.power, receivedAt);
  if (!expected && !measured && !power) return false;

  state.sequence = sequence;
  state.lastRobotPacketAt = receivedAt;
  if (expected) state.expected = expected;
  if (measured) state.measured = measured;
  if (power) state.power = power;
  return true;
}

export function streamIsFresh(stream, now = Date.now(), maximumAgeMs = LIVE_STREAM_FRESH_MS) {
  return Boolean(
    stream &&
    Number.isFinite(stream.receivedAt) &&
    now >= stream.receivedAt &&
    now - stream.receivedAt <= maximumAgeMs,
  );
}

export function signedAngleErrorDeg(measuredDeg, expectedDeg) {
  const measured = finiteNumber(measuredDeg);
  const expected = finiteNumber(expectedDeg);
  if (measured === null || expected === null) return null;
  return ((measured - expected + 540) % 360) - 180;
}

export function liveComparisonSnapshot(state, now = Date.now()) {
  const expectedFresh = streamIsFresh(state?.expected, now);
  const measuredFresh = streamIsFresh(state?.measured, now);
  const powerFresh = streamIsFresh(state?.power, now);
  const paired = expectedFresh && measuredFresh;
  const jointErrorsDeg = Array(16).fill(null);
  if (paired && Array.isArray(state.measured.servoAngleDeg)) {
    LIVE_SERVO_CHANNELS.forEach((channel) => {
      jointErrorsDeg[channel] = signedAngleErrorDeg(
        state.measured.servoAngleDeg[channel],
        state.expected.servoAngleDeg[channel],
      );
    });
  }
  const finiteJointErrors = LIVE_SERVO_CHANNELS
    .map((channel) => jointErrorsDeg[channel])
    .filter(Number.isFinite);
  const bodyError = paired
    ? {
        rollDeg: signedAngleErrorDeg(state.measured.body.rollDeg, state.expected.body.rollDeg),
        pitchDeg: signedAngleErrorDeg(state.measured.body.pitchDeg, state.expected.body.pitchDeg),
        yawDeg: signedAngleErrorDeg(state.measured.body.yawDeg, state.expected.body.yawDeg),
        heightMm: Number.isFinite(state.measured.body.heightMm)
          ? state.measured.body.heightMm - state.expected.body.heightMm
          : null,
      }
    : null;

  return {
    expectedFresh,
    measuredFresh,
    powerFresh,
    paired,
    expected: expectedFresh ? state.expected : null,
    measured: measuredFresh ? state.measured : null,
    power: powerFresh ? state.power : null,
    alignmentMs: paired ? state.measured.timestampMs - state.expected.timestampMs : null,
    jointErrorsDeg,
    worstJointErrorDeg: finiteJointErrors.length
      ? Math.max(...finiteJointErrors.map(Math.abs))
      : null,
    bodyError,
    lastRobotPacketAgeMs: state?.lastRobotPacketAt > 0 && now >= state.lastRobotPacketAt
      ? now - state.lastRobotPacketAt
      : null,
  };
}
