import { spawn } from "node:child_process";
import { appendFileSync, existsSync, mkdirSync, renameSync, statSync } from "node:fs";
import { createRequire } from "node:module";
import path from "node:path";

import { app, shell } from "electron";

import {
  clearUpdateAttempt,
  readUpdateAttempt,
  updateAttemptOutcome,
  writeUpdateAttempt,
} from "./update-attempt.mjs";

const require = createRequire(import.meta.url);

const APP_NAME = "Domino Quadruped Studio";
// Check as soon as the window is up, so every launch shows whether an update exists.
const INITIAL_CHECK_DELAY_MS = 3_000;
const PERIODIC_CHECK_INTERVAL_MS = 6 * 60 * 60 * 1_000;
const RELEASES_URL = "https://github.com/Blacksheep909/Domino-ESP32-Quadruped/releases/tag/v";
const LOG_LIMIT_BYTES = 512 * 1024;

let checkTimer = null;
let periodicCheckTimer = null;
let checkInFlight = false;
let updater = null;
let notifyState = () => {};
let downloadedInstallerPath = "";

// What Studio shows in its update banner. `status` is one of:
// unsupported (development build, or updates disabled), idle, checking,
// downloading, ready, manual (macOS download), install-failed (the last
// install did not take; offer the visible installer), up-to-date, error.
let state = {
  status: "idle",
  currentVersion: app.getVersion(),
  version: "",
  percent: 0,
  message: "",
};

function logFilePath() {
  try {
    return path.join(app.getPath("userData"), "logs", "updater.log");
  } catch {
    return "";
  }
}

// Updater messages go to <userData>/logs/updater.log as well as the console,
// so a failed check or install can be diagnosed after the fact.
function log(level, ...args) {
  const logger = console[level] || console.log;
  logger(`[${APP_NAME} updater]`, ...args);
  const file = logFilePath();
  if (!file) return;
  try {
    mkdirSync(path.dirname(file), { recursive: true });
    if (existsSync(file) && statSync(file).size > LOG_LIMIT_BYTES) renameSync(file, `${file}.old`);
    const text = args.map((value) => (value instanceof Error ? value.message : String(value))).join(" ");
    appendFileSync(file, `${new Date().toISOString()} ${level.toUpperCase()} ${text}\n`, "utf8");
  } catch {
    // Logging must never stop Studio or the updater.
  }
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
  // After a failed install keep that message up; only an explicit check
  // (TRY AGAIN / focus refresh from the banner) replaces it.
  if (state.status === "install-failed" && reason !== "manual") return;
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
function recordInstallAttempt(how) {
  try {
    writeUpdateAttempt(app.getPath("userData"), {
      version: state.version,
      fromVersion: app.getVersion(),
      installerPath: downloadedInstallerPath,
      how,
      attemptedAt: new Date().toISOString(),
    });
  } catch (error) {
    log("warn", "Could not record the update attempt:", error);
  }
}

export function installDownloadedUpdate() {
  if (state.status !== "ready" || !updater) return false;
  log("info", `Installing ${state.version} and restarting (installer ${downloadedInstallerPath || "unknown"}).`);
  recordInstallAttempt("restart");
  setImmediate(() => updater.quitAndInstall(true, true));
  return true;
}

// Called as Studio quits. A downloaded update is installed silently on quit
// (autoInstallOnAppQuit); remember it so the next launch can check it worked.
export function noteAppQuitting() {
  if (state.status === "ready" && !manualInstall && updater) {
    log("info", `Studio is quitting; installing ${state.version} silently.`);
    recordInstallAttempt("quit");
  }
}

// After a silent install failed: run the installer with its window showing,
// so it either completes or shows why it cannot. Falls back to the release page.
export function runInstallerVisibly() {
  if (state.status !== "install-failed") return false;
  const installer = state.installerPath;
  if (process.platform === "win32" && installer && existsSync(installer)) {
    log("info", `Running the ${state.version} installer visibly: ${installer}`);
    downloadedInstallerPath = installer;
    recordInstallAttempt("visible-installer");
    const child = spawn(installer, [], { detached: true, stdio: "ignore", windowsHide: false });
    child.unref();
    setTimeout(() => app.quit(), 500);
    return true;
  }
  const url = state.version ? `${RELEASES_URL}${state.version}` : "";
  if (!url) return false;
  log("info", `Installer not available; opening ${url}`);
  void shell.openExternal(url);
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
    downloadedInstallerPath = typeof info.downloadedFile === "string" ? info.downloadedFile : "";
    log("info", `Update ${info.version} downloaded and ready to install (${downloadedInstallerPath || "path unknown"}).`);
    setState({ status: "ready", version: info.version, percent: 100, message: "" });
  });
  activeUpdater.on("error", (error) => {
    // Keep this quiet for users without network access; the app is designed to work offline.
    log("warn", "Updater error:", error instanceof Error ? error.message : error);
    if (!["ready", "install-failed"].includes(state.status)) setState({ status: "error", message: errorText(error) });
  });

  // Did the update started last time actually install?
  const userData = app.getPath("userData");
  const outcome = updateAttemptOutcome(readUpdateAttempt(userData), app.getVersion());
  if (outcome.status === "installed") {
    log("info", `Update to ${outcome.version} installed; running ${app.getVersion()}.`);
    clearUpdateAttempt(userData);
  } else if (outcome.status === "failed") {
    log("warn", `The update to ${outcome.version} (started ${outcome.attemptedAt}) did not install; still running ${app.getVersion()}.`);
    setState({
      status: "install-failed",
      version: outcome.version,
      installerPath: outcome.installerPath,
      percent: 0,
      message: "",
    });
  }

  log("info", `Studio ${app.getVersion()} started; checking for updates in ${INITIAL_CHECK_DELAY_MS / 1000} s.`);
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
