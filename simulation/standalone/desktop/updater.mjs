import { createRequire } from "node:module";

import { app, dialog } from "electron";

const require = createRequire(import.meta.url);

const APP_NAME = "Domino Quadruped Studio";
const INITIAL_CHECK_DELAY_MS = 15_000;
const PERIODIC_CHECK_INTERVAL_MS = 6 * 60 * 60 * 1_000;

let checkTimer = null;
let periodicCheckTimer = null;
let checkInFlight = false;
let updateReady = false;
let updatePromptOpen = false;
let updater = null;

function log(level, ...args) {
  const logger = console[level] || console.log;
  logger(`[${APP_NAME} updater]`, ...args);
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

function updateDialogOptions(version) {
  return {
    type: "info",
    title: `${APP_NAME} update ready`,
    message: `Version ${version} is ready to install.`,
    detail: "Restart now to update the existing installation in place. Your settings and runtime data will be kept.",
    buttons: ["Restart and install", "Later"],
    defaultId: 0,
    cancelId: 1,
    noLink: true,
  };
}

async function promptToInstall(getMainWindow, version, activeUpdater) {
  if (updatePromptOpen) return;
  updatePromptOpen = true;
  try {
    const options = updateDialogOptions(version);
    const window = getMainWindow?.() || null;
    const result = window
      ? await dialog.showMessageBox(window, options)
      : await dialog.showMessageBox(options);
    if (result.response === 0) {
      activeUpdater.quitAndInstall(false, true);
    }
  } finally {
    updatePromptOpen = false;
  }
}

async function checkForUpdates(activeUpdater, reason = "scheduled") {
  if (checkInFlight || updateReady) return;
  checkInFlight = true;
  try {
    log("info", `Checking for updates (${reason}).`);
    await activeUpdater.checkForUpdates();
  } catch (error) {
    // Offline use is expected. A failed check must never prevent the app from starting.
    log("warn", "Update check failed:", error instanceof Error ? error.message : error);
  } finally {
    checkInFlight = false;
  }
}

export function requestUpdateCheck(reason = "manual") {
  if (!app.isPackaged || process.platform !== "win32" || process.env.DOMINO_DISABLE_AUTO_UPDATE === "1") {
    return false;
  }
  const activeUpdater = updater || loadUpdater();
  if (!activeUpdater) return false;
  void checkForUpdates(activeUpdater, reason);
  return true;
}

export function setupAutoUpdater({ getMainWindow } = {}) {
  if (!app.isPackaged || process.platform !== "win32" || process.env.DOMINO_DISABLE_AUTO_UPDATE === "1") {
    return;
  }

  const activeUpdater = loadUpdater();
  if (!activeUpdater) return;

  activeUpdater.autoDownload = true;
  activeUpdater.autoInstallOnAppQuit = true;
  activeUpdater.allowDowngrade = false;
  activeUpdater.allowPrerelease = false;
  activeUpdater.logger = {
    info: (...args) => log("info", ...args),
    warn: (...args) => log("warn", ...args),
    error: (...args) => log("error", ...args),
    debug: (...args) => log("debug", ...args),
  };

  activeUpdater.on("checking-for-update", () => log("info", "Checking for updates."));
  activeUpdater.on("update-available", (info) => {
    log("info", `Update ${info.version} is available; downloading in the background.`);
  });
  activeUpdater.on("update-not-available", (info) => {
    log("info", `No update available; current version is ${info.version}.`);
  });
  activeUpdater.on("download-progress", (progress) => {
    log("info", `Downloading update: ${Math.round(progress.percent)}%.`);
  });
  activeUpdater.on("update-downloaded", (info) => {
    updateReady = true;
    log("info", `Update ${info.version} downloaded and ready to install.`);
    void promptToInstall(getMainWindow, info.version, activeUpdater);
  });
  activeUpdater.on("error", (error) => {
    // Keep this quiet for users without network access; the app is designed to work offline.
    log("warn", "Updater error:", error instanceof Error ? error.message : error);
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
