import { createRequire } from "node:module";

import { app } from "electron";

const require = createRequire(import.meta.url);

const APP_NAME = "Domino Quadruped Studio";
// Check as soon as the window is up, so every launch shows whether an update exists.
const INITIAL_CHECK_DELAY_MS = 3_000;
const PERIODIC_CHECK_INTERVAL_MS = 6 * 60 * 60 * 1_000;

let checkTimer = null;
let periodicCheckTimer = null;
let checkInFlight = false;
let updater = null;
let notifyState = () => {};

// What Studio shows in its update banner. `status` is one of:
// unsupported (development build, or updates disabled), idle, checking,
// downloading, ready, up-to-date, error.
let state = {
  status: "idle",
  currentVersion: app.getVersion(),
  version: "",
  percent: 0,
  message: "",
};

function log(level, ...args) {
  const logger = console[level] || console.log;
  logger(`[${APP_NAME} updater]`, ...args);
}

function setState(patch) {
  state = { ...state, ...patch, currentVersion: app.getVersion() };
  try {
    notifyState(state);
  } catch (error) {
    log("warn", "Could not publish update state:", error instanceof Error ? error.message : error);
  }
}

// macOS builds are not signed with an Apple Developer ID, and macOS refuses to
// install an unsigned update in place. There, Studio still checks for updates
// but only offers the download page (status "manual").
const manualInstall = process.platform === "darwin";

function updatesSupported() {
  return app.isPackaged && ["win32", "darwin"].includes(process.platform) &&
    process.env.DOMINO_DISABLE_AUTO_UPDATE !== "1";
}

function loadUpdater() {
  if (updater) return updater;
  try {
    // electron-updater is CommonJS. Using require keeps this ESM entry point
    // compatible with packaged Electron, where named ESM imports are not
    // available for CommonJS exports.
    const module = require("electron-updater");
    updater = module.autoUpdater || module.default?.autoUpdater || null;
  } catch (error) {
    // Updating is optional. A missing or damaged updater dependency must never
    // prevent the offline application from opening.
    log("warn", "Updater unavailable:", error instanceof Error ? error.message : error);
  }
  return updater;
}

function errorText(error) {
  const text = error instanceof Error ? error.message : String(error ?? "");
  if (/ENOTFOUND|ECONNREFUSED|ETIMEDOUT|EAI_AGAIN|net::ERR_|getaddrinfo|network/i.test(text)) {
    return "No internet connection.";
  }
  return text.split("\n")[0].slice(0, 160) || "Unknown error.";
}

async function checkForUpdates(activeUpdater, reason = "scheduled") {
  if (checkInFlight || ["ready", "downloading", "manual"].includes(state.status)) return;
  checkInFlight = true;
  setState({ status: "checking", message: "" });
  try {
    log("info", `Checking for updates (${reason}).`);
    await activeUpdater.checkForUpdates();
  } catch (error) {
    // Offline use is expected. A failed check must never prevent the app from working.
    log("warn", "Update check failed:", error instanceof Error ? error.message : error);
    setState({ status: "error", message: errorText(error) });
  } finally {
    checkInFlight = false;
  }
}

export function getUpdateState() {
  return state;
}

export function requestUpdateCheck(reason = "manual") {
  if (!updatesSupported()) return false;
  const activeUpdater = updater || loadUpdater();
  if (!activeUpdater) return false;
  void checkForUpdates(activeUpdater, reason);
  return true;
}

// Closes Studio, installs the downloaded version silently in place, and opens
// the new version. Settings and runtime data are kept.
export function installDownloadedUpdate() {
  if (state.status !== "ready" || !updater) return false;
  log("info", `Installing ${state.version} and restarting.`);
  setImmediate(() => updater.quitAndInstall(true, true));
  return true;
}

export function setupAutoUpdater({ onStateChange } = {}) {
  if (typeof onStateChange === "function") notifyState = onStateChange;
  if (!updatesSupported()) {
    setState({ status: "unsupported" });
    return;
  }

  const activeUpdater = loadUpdater();
  if (!activeUpdater) {
    setState({ status: "unsupported" });
    return;
  }

  activeUpdater.autoDownload = !manualInstall;
  activeUpdater.autoInstallOnAppQuit = !manualInstall;
  activeUpdater.allowDowngrade = false;
  activeUpdater.allowPrerelease = false;
  activeUpdater.logger = {
    info: (...args) => log("info", ...args),
    warn: (...args) => log("warn", ...args),
    error: (...args) => log("error", ...args),
    debug: (...args) => log("debug", ...args),
  };

  activeUpdater.on("update-available", (info) => {
    if (manualInstall) {
      log("info", `Update ${info.version} is available to download.`);
      setState({ status: "manual", version: info.version, percent: 0, message: "" });
      return;
    }
    log("info", `Update ${info.version} is available; downloading in the background.`);
    setState({ status: "downloading", version: info.version, percent: 0, message: "" });
  });
  activeUpdater.on("update-not-available", (info) => {
    log("info", `No update available; current version is ${info.version}.`);
    setState({ status: "up-to-date", version: "", percent: 0, message: "" });
  });
  activeUpdater.on("download-progress", (progress) => {
    setState({ status: "downloading", percent: Math.max(0, Math.min(100, Math.round(progress.percent || 0))) });
  });
  activeUpdater.on("update-downloaded", (info) => {
    log("info", `Update ${info.version} downloaded and ready to install.`);
    setState({ status: "ready", version: info.version, percent: 100, message: "" });
  });
  activeUpdater.on("error", (error) => {
    // Keep this quiet for users without network access; the app is designed to work offline.
    log("warn", "Updater error:", error instanceof Error ? error.message : error);
    if (state.status !== "ready") setState({ status: "error", message: errorText(error) });
  });

  checkTimer = setTimeout(() => void checkForUpdates(activeUpdater, "startup"), INITIAL_CHECK_DELAY_MS);
  periodicCheckTimer = setInterval(() => void checkForUpdates(activeUpdater, "periodic"), PERIODIC_CHECK_INTERVAL_MS);
  checkTimer.unref?.();
  periodicCheckTimer.unref?.();
}

export function stopAutoUpdater() {
  if (checkTimer) clearTimeout(checkTimer);
  if (periodicCheckTimer) clearInterval(periodicCheckTimer);
  checkTimer = null;
  periodicCheckTimer = null;
}
