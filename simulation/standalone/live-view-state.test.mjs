import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

import {
  createLiveViewState,
  LIVE_VIEW_CALIBRATION,
  LIVE_VIEW_DIAGNOSTICS,
  LIVE_VIEW_GAITS,
  LIVE_VIEW_SENSORS,
  LIVE_VIEW_COMPARE,
  LIVE_VIEW_DATA,
  LIVE_VIEW_SESSIONS,
  selectLiveView,
} from "./web/src/live-view-state.js";

test("LIVE opens on the comparison view", () => {
  assert.equal(createLiveViewState().selected, LIVE_VIEW_COMPARE);
});

test("implemented Data, Sensors, Calibration, Gaits, Diagnostics and Sessions views are selectable", () => {
  const state = createLiveViewState();
  assert.equal(selectLiveView(state, LIVE_VIEW_DATA), true);
  assert.equal(state.selected, LIVE_VIEW_DATA);
  assert.equal(selectLiveView(state, LIVE_VIEW_SENSORS), true);
  assert.equal(state.selected, LIVE_VIEW_SENSORS);
  assert.equal(selectLiveView(state, LIVE_VIEW_CALIBRATION), true);
  assert.equal(state.selected, LIVE_VIEW_CALIBRATION);
  assert.equal(selectLiveView(state, LIVE_VIEW_DIAGNOSTICS), true);
  assert.equal(state.selected, LIVE_VIEW_DIAGNOSTICS);
  assert.equal(selectLiveView(state, LIVE_VIEW_GAITS), true);
  assert.equal(state.selected, LIVE_VIEW_GAITS);
  assert.equal(selectLiveView(state, LIVE_VIEW_SESSIONS), true);
  assert.equal(state.selected, LIVE_VIEW_SESSIONS);
});

test("unimplemented or unknown views cannot replace the active page", () => {
  const state = createLiveViewState();
  selectLiveView(state, LIVE_VIEW_DATA);
  assert.equal(selectLiveView(state, "controls"), false);
  assert.equal(state.selected, LIVE_VIEW_DATA);
});

test("switching LIVE tools returns the selected page to its title", () => {
  const main = readFileSync(new URL("./web/src/main.js", import.meta.url), "utf8");
  assert.match(main, /const activeLivePage = document\.querySelector\(`#live-view-\$\{liveViewState\.selected\}`\);/);
  assert.match(main, /activeLivePage\?\.scrollTo\?\.\(0, 0\);/);
});

test("compact LIVE previews collapse while scrolling instead of covering page content", () => {
  const main = readFileSync(new URL("./web/src/main.js", import.meta.url), "utf8");
  const styles = readFileSync(new URL("./web/src/styles.css", import.meta.url), "utf8");
  assert.match(main, /function updateLivePreviewVisibility\(\)/);
  assert.match(main, /livePreviewCollapsed/);
  assert.match(main, /querySelectorAll\("button\[data-live-view\]"\)/);
  assert.match(styles, /@media \(max-width: 1000px\)[\s\S]*data-live-preview-collapsed="true"[\s\S]*#scene/);
  assert.match(styles, /#live-view-sensors \.live-sensor-grid[\s\S]*padding-top: 255px/);
});

test("LIVE camera controls follow light mode while preserving a dark feed surface", () => {
  const styles = readFileSync(new URL("./web/src/styles.css", import.meta.url), "utf8");
  assert.match(styles, /:root:not\(\[data-theme="dark"\]\) \.live-camera-panel[\s\S]*background: #fafaf8/);
  assert.match(styles, /\.live-camera-feed[\s\S]*background: #050506/);
});

test("Compare omits its duplicate title rail and uses the compact content baseline", () => {
  const styles = readFileSync(new URL("./web/src/styles.css", import.meta.url), "utf8");
  assert.match(styles, /Titleless Compare workspace[\s\S]*#real-workspace #live-view-compare \.live-compare-page-heading[\s\S]*display: none !important/);
  assert.match(styles, /data-live-view="compare"\][\s\S]*--live-preview-top: calc\(var\(--live-content-top, 118px\) \+ var\(--app-gutter\)\)/);
  assert.match(styles, /data-live-view="compare"\][\s\S]*top: var\(--live-preview-top\)/);
  assert.match(styles, /--live-editor-width: calc\([\s\S]*var\(--live-preview-width\)[\s\S]*var\(--live-column-gap\)/);
  assert.match(styles, /#live-view-calibration \.live-calibration-layout,[\s\S]*#live-view-gaits \.live-gaits-layout[\s\S]*width: var\(--live-editor-width\)/);
  assert.match(styles, /#live-view-data \.live-data-grid,[\s\S]*#live-view-sessions \.live-sessions-grid[\s\S]*max-width: none/);
  assert.match(styles, /\.calibration-preview-caption,[\s\S]*background: rgba\(247, 247, 244, 0\.94\)/);
  assert.match(styles, /:root\[data-theme="dark"\] \.calibration-preview-caption,[\s\S]*background: rgba\(12, 12, 13, 0\.78\)/);
  assert.match(styles, /#real-workspace #live-view-compare \.live-compare-page-heading[\s\S]*right: var\(--app-gutter\)[\s\S]*width: auto/);
  assert.match(styles, /:root:not\(\[data-theme="dark"\]\) nav button\.active,[\s\S]*background: #347b50/);
  assert.match(styles, /#real-workspace \.live-view-page[\s\S]*overflow-x: hidden/);
});

test("every LIVE 3D view uses a lower shared camera target so the full robot remains framed", () => {
  const main = readFileSync(new URL("./web/src/main.js", import.meta.url), "utf8");
  assert.match(main, /const LIVE_CAMERA_ANCHOR_Y = 0\.26/);
  assert.match(main, /applicationState\.workspace === WORKSPACE_REAL_ROBOT[\s\S]*\? LIVE_CAMERA_ANCHOR_Y/);
  assert.match(main, /robotCameraAnchor\.set\([\s\S]*cameraAnchorY\(\)/);
});

test("right-hand LIVE viewports share the editor card horizontal datum", () => {
  const styles = readFileSync(new URL("./web/src/styles.css", import.meta.url), "utf8");
  assert.match(styles, /data-live-view="sensors"[\s\S]*data-live-view="calibration"[\s\S]*data-live-view="gaits"[\s\S]*--live-preview-top: calc\([\s\S]*var\(--live-title-gap\) \+ 5px/);
});

test("LIVE keeps measured battery state and E-stop in the persistent header", () => {
  const html = readFileSync(new URL("./web/index.html", import.meta.url), "utf8");
  const main = readFileSync(new URL("./web/src/main.js", import.meta.url), "utf8");
  const header = html.match(/<header>[\s\S]*?<\/header>/)?.[0] || "";
  assert.match(header, /id="real-battery-status"[^>]*real-robot-only/);
  assert.match(header, /id="live-global-estop"[^>]*real-robot-only/);
  assert.match(main, /#live-global-estop"\)\.addEventListener\("click", \(\) => sendLiveSafetyCommand\("estop"\)/);
});

test("Data shows a rolling live preview before recording", () => {
  const html = readFileSync(new URL("./web/index.html", import.meta.url), "utf8");
  const main = readFileSync(new URL("./web/src/main.js", import.meta.url), "utf8");
  assert.match(html, /<span>IMU PITCH<\/span><strong id="live-data-pitch"/);
  assert.match(main, /const livePreviewState = createLiveSessionState\(600\)/);
  assert.match(main, /recordLiveComparisonSample\(livePreviewState, snapshot\)/);
  assert.match(main, /liveSessionState\.samples\.length > 0[\s\S]*livePreviewState\.samples/);
});

test("Sensors exposes live IMU alignment and honest future capability modules", () => {
  const html = readFileSync(new URL("./web/index.html", import.meta.url), "utf8");
  assert.match(html, /data-live-view="sensors"/);
  assert.match(html, /id="live-sensor-plane"/);
  assert.match(html, /GUIDED IMU CALIBRATION/);
  assert.match(html, /GNSS \/ GPS[\s\S]*live-gps-module-badge">WAITING/);
  assert.match(html, /LiDAR[\s\S]*live-lidar-module-badge">WAITING/);
});
