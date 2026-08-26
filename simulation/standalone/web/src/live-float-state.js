export const LIVE_FLOAT_MINIMUM_HEIGHT_MM = 360;

export function livePreviewBodyPose(body = {}, floating = false, anchorHeightMm = LIVE_FLOAT_MINIMUM_HEIGHT_MM) {
  if (!floating) return body;
  const requestedAnchor = Number(anchorHeightMm);
  const heightMm = Number.isFinite(requestedAnchor)
    ? Math.max(LIVE_FLOAT_MINIMUM_HEIGHT_MM, requestedAnchor)
    : LIVE_FLOAT_MINIMUM_HEIGHT_MM;
  return { ...body, heightMm };
}
