import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

import { appUpdateBannerView } from "./web/src/app-update-banner.js";

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
