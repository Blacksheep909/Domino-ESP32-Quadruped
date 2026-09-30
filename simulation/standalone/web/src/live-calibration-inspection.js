export function nudgeCalibrationInspectionPose(poseDeg, incrementDeg, minimumDeg, maximumDeg) {
  if (![poseDeg, incrementDeg, minimumDeg, maximumDeg].every(Number.isFinite) || maximumDeg < minimumDeg) {
    return poseDeg;
  }
  return Math.min(maximumDeg, Math.max(minimumDeg,
    Math.round((poseDeg + incrementDeg) * 1000) / 1000));
}

export function advanceCalibrationSweep(position, direction, minimum, maximum, elapsedSeconds, speedDegPerSec = 42) {
  if (![position, direction, minimum, maximum, elapsedSeconds, speedDegPerSec].every(Number.isFinite) ||
      maximum <= minimum || elapsedSeconds <= 0 || speedDegPerSec <= 0) {
    return { position, direction };
  }
  const span = maximum - minimum;
  const phase = direction >= 0 ? position - minimum : span + maximum - position;
  const wrapped = ((phase + speedDegPerSec * elapsedSeconds) % (2 * span) + 2 * span) % (2 * span);
  return wrapped <= span
    ? { position: minimum + wrapped, direction: 1 }
    : { position: maximum - (wrapped - span), direction: -1 };
}

// World-space axis-aligned boxes provide a broad-phase warning, not a mesh
// collision proof. They intentionally err on the side of reporting proximity.
export function calibrationBoxGapMm(first, second) {
  const axisGap = (axis) => Math.max(0,
    second.min[axis] - first.max[axis],
    first.min[axis] - second.max[axis],
  );
  return Math.hypot(axisGap("x"), axisGap("y"), axisGap("z")) * 1_000;
}

export function jointTravelMetrics(poseDeg, minimumDeg, maximumDeg, mechanicalTravelDeg) {
  if (![poseDeg, minimumDeg, maximumDeg, mechanicalTravelDeg].every(Number.isFinite) ||
      maximumDeg <= minimumDeg || mechanicalTravelDeg <= 0) return null;
  const minimumMargin = poseDeg - minimumDeg;
  const maximumMargin = maximumDeg - poseDeg;
  const nearestStop = minimumMargin <= maximumMargin ? "MIN" : "MAX";
  const margin = Math.min(minimumMargin, maximumMargin);
  const nearThreshold = Math.min(5, (maximumDeg - minimumDeg) * 0.1);
  const state = margin < -0.05 ? "outside" : margin <= 0.05 ? "stop"
    : margin <= nearThreshold ? "near" : "clear";
  const percent = (angle) => Math.max(0, Math.min(100,
    ((angle + mechanicalTravelDeg) / (mechanicalTravelDeg * 2)) * 100));
  return {
    minimumMargin, maximumMargin, nearestStop, margin, state,
    minimumPercent: percent(minimumDeg), maximumPercent: percent(maximumDeg),
    posePercent: percent(poseDeg), neutralPercent: 50,
  };
}

// The card uses CSS pixels, independently of camera projection or model scale.
// Keep it within the viewport and away from the anchor; hysteresis prevents
// the card swapping sides every time an orbit crosses the viewport center.
export function placeJointInspectionCard(anchor, viewport, card, previousSide = "left", obstacles = []) {
  const padding = 12;
  const width = Math.min(card.width, Math.max(1, viewport.width - padding * 2));
  const height = Math.min(card.height, Math.max(1, viewport.height - padding * 2));
  let side = previousSide;
  if (anchor.x < viewport.width * 0.35) side = "right";
  if (anchor.x > viewport.width * 0.65) side = "left";
  const bottomY = Math.max(padding, viewport.height - height - padding);
  const candidates = ["left", "right"].flatMap((candidateSide) => {
    const x = candidateSide === "left" ? padding : Math.max(padding, viewport.width - width - padding);
    const topY = candidateSide === "right" && 82 + height <= viewport.height - padding ? 82 : padding;
    return [topY, bottomY].map((y) => {
      const overlap = obstacles.reduce((total, obstacle) => total +
        Math.max(0, Math.min(x + width, obstacle.x + obstacle.width) - Math.max(x, obstacle.x)) *
        Math.max(0, Math.min(y + height, obstacle.y + obstacle.height) - Math.max(y, obstacle.y)), 0);
      const distance = Math.hypot(anchor.x - (x + width / 2), anchor.y - (y + height / 2));
      return { x, y, side: candidateSide, score: distance + (candidateSide === side ? 100 : 0) - overlap * 20 };
    });
  });
  const best = candidates.sort((a, b) => b.score - a.score)[0];
  const { x, y } = best;
  side = best.side;
  const edgeX = Math.max(x, Math.min(x + width, anchor.x));
  const edgeY = Math.max(y + 12, Math.min(y + height - 12, anchor.y));
  return { x, y, width, height, side, edgeX, edgeY };
}
