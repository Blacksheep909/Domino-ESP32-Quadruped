export const LIVE_VIEW_COMPARE = "compare";
export const LIVE_VIEW_DATA = "data";
export const LIVE_VIEW_CALIBRATION = "calibration";
export const LIVE_VIEW_DIAGNOSTICS = "diagnostics";
export const LIVE_VIEW_SESSIONS = "sessions";
export const ENABLED_LIVE_VIEWS = Object.freeze([
  LIVE_VIEW_COMPARE,
  LIVE_VIEW_DATA,
  LIVE_VIEW_CALIBRATION,
  LIVE_VIEW_DIAGNOSTICS,
  LIVE_VIEW_SESSIONS,
]);

export function createLiveViewState() {
  return { selected: LIVE_VIEW_COMPARE };
}

export function selectLiveView(state, view) {
  if (!state || !ENABLED_LIVE_VIEWS.includes(view)) return false;
  state.selected = view;
  return true;
}
