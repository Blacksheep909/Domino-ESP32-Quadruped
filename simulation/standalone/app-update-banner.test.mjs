import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

import { appUpdateBannerView, appUpdateChipView } from "./web/src/app-update-banner.js";

test("development builds and the idle state show no banner", () => {
  assert.equal(appUpdateBannerView({ status: "unsupported" }).visible, false);
  assert.equal(appUpdateBannerView({ status: "idle" }).visible, false);
  assert.equal(appUpdateBannerView(null).visible, false);
});

test("every launch shows the check and its result", () => {
  assert.match(appUpdateBannerView({ status: "checking" }).text, /Checking/);
  const upToDate = appUpdateBannerView({ status: "up-to-date", currentVersion: "0.4.5" });
  assert.equal(upToDate.visible, true);
  assert.match(upToDate.text, /0\.4\.5 is up to date/);
  assert.ok(upToDate.autoHideMs > 0);
});

test("a download shows its progress and release notes", () => {
  const view = appUpdateBannerView({ status: "downloading", version: "0.4.6", percent: 41.6 });
  assert.equal(view.percent, 42);
  assert.match(view.text, /0\.4\.6.*42%/);
  assert.match(view.notesUrl, /releases\/tag\/v0\.4\.6$/);
  assert.equal(view.action, undefined);
});

test("a ready update is one click to restart", () => {
  const view = appUpdateBannerView({ status: "ready", version: "0.4.6" });
  assert.equal(view.action.id, "install");
  assert.equal(view.action.label, "RESTART & UPDATE");
  assert.equal(view.action.disabled, false);
  assert.equal(view.dismissLabel, "LATER");
});

test("restarting is blocked, with the reason, while the robot needs the PC link", () => {
  const view = appUpdateBannerView({ status: "ready", version: "0.4.6" }, "Disarm the robot before updating.");
  assert.equal(view.action.disabled, true);
  assert.equal(view.detail, "Disarm the robot before updating.");
});

test("a failed check offers a retry and can be dismissed", () => {
  const view = appUpdateBannerView({ status: "error", message: "No internet connection." });
  assert.equal(view.action.id, "check");
  assert.equal(view.detail, "No internet connection.");
  assert.equal(appUpdateBannerView({ status: "error" }, "", true).visible, false);
});

test("the desktop bridge exposes the update calls and the updater installs silently and relaunches", () => {
  const preload = readFileSync(new URL("./desktop/preload.mjs", import.meta.url), "utf8");
  for (const channel of ["domino:update:get-state", "domino:update:check", "domino:update:install", "domino:update:state"]) {
    assert.ok(preload.includes(channel), channel);
  }
  const updater = readFileSync(new URL("./desktop/updater.mjs", import.meta.url), "utf8");
  assert.match(updater, /quitAndInstall\(true, true\)/);
  assert.match(updater, /INITIAL_CHECK_DELAY_MS = 3_000/);
});

test("on macOS a new version is offered as a download, since it cannot install itself", () => {
  const view = appUpdateBannerView({ status: "manual", version: "0.5.1" });
  assert.equal(view.visible, true);
  assert.match(view.text, /0\.5\.1 is available/);
  assert.equal(view.action.id, "download");
  assert.equal(view.action.disabled, false);
  assert.match(view.downloadUrl, /releases\/tag\/v0\.5\.1$/);
  assert.equal(view.dismissLabel, "LATER");
});

test("the Mac updater only checks for updates and never downloads or installs them", () => {
  const updater = readFileSync(new URL("./desktop/updater.mjs", import.meta.url), "utf8");
  assert.match(updater, /const manualInstall = process\.platform === "darwin"/);
  assert.match(updater, /autoDownload = !manualInstall/);
  assert.match(updater, /autoInstallOnAppQuit = !manualInstall/);
  assert.match(updater, /\["win32", "darwin"\]\.includes\(process\.platform\)/);
});

test("an update that didn't install says so and offers the visible installer", () => {
  const view = appUpdateBannerView({ status: "install-failed", version: "0.5.5", currentVersion: "0.5.2" });
  assert.equal(view.visible, true);
  assert.equal(view.tone, "warning");
  assert.match(view.text, /0\.5\.5 didn't install/);
  assert.match(view.detail, /still 0\.5\.2/);
  assert.equal(view.action.id, "run-installer");
  assert.equal(view.action.disabled, false);
  assert.equal(appUpdateBannerView({ status: "install-failed", version: "0.5.5" }, "Disarm the robot before updating.").action.disabled, true);
  assert.equal(appUpdateChipView({ status: "install-failed", version: "0.5.5" }).label, "UPDATE FAILED");
  const preload = readFileSync(new URL("./desktop/preload.mjs", import.meta.url), "utf8");
  assert.ok(preload.includes("domino:update:run-installer"));
  const main = readFileSync(new URL("./desktop/main.mjs", import.meta.url), "utf8");
  assert.ok(main.includes("domino:update:run-installer"));
});

test("the updater starts before the window loads and keeps a log file", () => {
  const main = readFileSync(new URL("./desktop/main.mjs", import.meta.url), "utf8");
  const ready = main.indexOf("app.whenReady()");
  assert.ok(ready > 0);
  // setupAutoUpdater runs inside whenReady, before createApplication is awaited.
  assert.ok(main.indexOf("setupAutoUpdater(", ready) < main.indexOf("createApplication()", ready));
  assert.match(main, /noteAppQuitting\(\)/);
  const updater = readFileSync(new URL("./desktop/updater.mjs", import.meta.url), "utf8");
  assert.match(updater, /"updater\.log"/);
});

test("a waiting update keeps a header button, so LATER never loses it", () => {
  assert.equal(appUpdateChipView({ status: "ready", version: "0.5.3" }).label, "UPDATE READY");
  assert.equal(appUpdateChipView({ status: "manual", version: "0.5.3" }).label, "UPDATE 0.5.3");
  assert.equal(appUpdateChipView({ status: "downloading", version: "0.5.3", percent: 12.4 }).label, "UPDATING 12%");
  for (const status of ["idle", "checking", "up-to-date", "error", "unsupported"]) {
    assert.equal(appUpdateChipView({ status }).visible, false, status);
  }
  const html = readFileSync(new URL("./web/index.html", import.meta.url), "utf8");
  assert.match(html, /id="app-update-chip"[^>]*hidden/);
});
