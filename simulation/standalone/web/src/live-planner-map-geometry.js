// The map uses metres on both axes. Its SVG viewBox grows horizontally with
// the viewport, so a wide map never stretches the tiles or letterboxes them.
export function plannerViewport(width, height) {
  const aspect = Number.isFinite(width) && Number.isFinite(height) && width > 0 && height > 0
    ? Math.max(0.5, Math.min(4, width / height))
    : 1;
  const widthUnits = 100 * aspect;
  return { aspect, widthUnits, minX: 50 - widthUnits / 2, maxX: 50 + widthUnits / 2 };
}

export function plannerLocalAtFraction(fractionX, fractionY, center, rangeM, aspect) {
  return {
    eastM: center.eastM + (fractionX - 0.5) * rangeM * aspect,
    northM: center.northM + (0.5 - fractionY) * rangeM,
  };
}

export function plannerCenterAfterPan(center, deltaX, deltaY, viewportHeight, rangeM) {
  if (!viewportHeight) return { ...center };
  return {
    eastM: center.eastM - deltaX * rangeM / viewportHeight,
    northM: center.northM + deltaY * rangeM / viewportHeight,
  };
}

export function plannerCenterAfterZoom(center, currentRangeM, nextRangeM, fractionX, fractionY, aspect) {
  return {
    eastM: center.eastM + (fractionX - 0.5) * (currentRangeM - nextRangeM) * aspect,
    northM: center.northM + (0.5 - fractionY) * (currentRangeM - nextRangeM),
  };
}
