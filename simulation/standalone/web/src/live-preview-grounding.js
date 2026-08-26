export function livePreviewGroundCorrection(footSurfaceHeights, floorClearance = 0) {
  const surfaces = Array.isArray(footSurfaceHeights)
    ? footSurfaceHeights.filter(Number.isFinite)
    : [];
  if (surfaces.length === 0) return 0;
  const clearance = Number.isFinite(floorClearance) ? floorClearance : 0;
  return clearance - Math.min(...surfaces);
}
