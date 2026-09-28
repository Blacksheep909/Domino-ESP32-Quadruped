import { spawn } from "node:child_process";
import { createWriteStream, existsSync, mkdirSync } from "node:fs";
import net from "node:net";
import path from "node:path";
import process from "node:process";

import { app, BrowserWindow, dialog, ipcMain, screen, shell } from "electron";

import { resolveDesktopPaths } from "./paths.mjs";
import {
  projectFileName,
  readProjectFile,
  validateProjectFileContents,
  writeProjectFile,
} from "./project-files.mjs";
import { requestUpdateCheck, setupAutoUpdater, stopAutoUpdater } from "./updater.mjs";
import { desktopWindowTitle } from "./app-title.mjs";

const APP_NAME = "Domino Quadruped Studio";
const APP_TITLE = desktopWindowTitle(APP_NAME, app.getVersion());
const HOST = "127.0.0.1";
const PREFERRED_PORT = 8770;
const children = new Set();
let shuttingDown = false;
let mainWindow = null;
let shutdownLocalService = null;

app.setName(APP_NAME);
app.setAppUserModelId("com.domino.quadruped.studio");

function registerProjectFileHandlers() {
  ipcMain.handle("domino:project:open", async () => {
    const selected = await dialog.showOpenDialog(mainWindow || undefined, {
      title: "Open Domino project",
      defaultPath: app.getPath("documents"),
      properties: ["openFile"],
      filters: [{ name: "Domino project", extensions: ["qstudio.json", "json"] }],
    });
    if (selected.canceled || !selected.filePaths[0]) return { canceled: true };
    const project = await readProjectFile(selected.filePaths[0]);
    return {
      canceled: false,
      contents: project.contents,
      fileName: project.fileName,
    };
  });

  ipcMain.handle("domino:project:save", async (_event, payload) => {
    const contents = payload?.contents;
    const suggestedFileName = projectFileName(payload?.suggestedFileName);
    // Reject malformed data before asking the user to choose an output path.
    validateProjectFileContents(contents);
    const selected = await dialog.showSaveDialog(mainWindow || undefined, {
      title: "Save Domino project",
      defaultPath: path.join(app.getPath("documents"), suggestedFileName),
      filters: [{ name: "Domino project", extensions: ["qstudio.json", "json"] }],
    });
    if (selected.canceled || !selected.filePath) return { canceled: true };
    const project = await writeProjectFile(selected.filePath, contents);
    return { canceled: false, fileName: project.fileName };
  });
}

registerProjectFileHandlers();

ipcMain.handle("domino:window:is-fullscreen", () => mainWindow?.isFullScreen() ?? false);
ipcMain.handle("domino:window:toggle-fullscreen", () => {
  if (!mainWindow) return false;
  const fullscreen = !mainWindow.isFullScreen();
  mainWindow.setFullScreen(fullscreen);
  return fullscreen;
});

function portIsAvailable(port) {
  return new Promise((resolve) => {
    const probe = net.createServer();
    probe.unref();
    probe.once("error", () => resolve(false));
    probe.listen(port, HOST, () => probe.close(() => resolve(true)));
  });
}

async function availablePort() {
  for (let port = PREFERRED_PORT; port < PREFERRED_PORT + 20; port += 1) {
    if (await portIsAvailable(port)) return port;
  }
  throw new Error("No local Domino service port is available.");
}

function attachProcessLogging(child, name, logRoot) {
  const output = createWriteStream(path.join(logRoot, `${name}.log`), { flags: "a" });
  child.stdout?.pipe(output);
  child.stderr?.pipe(output);
  child.once("exit", () => {
    children.delete(child);
    output.end();
  });
  children.add(child);
  return child;
}

function spawnService(executable, args, options, name, logRoot) {
  const child = spawn(executable, args, {
    windowsHide: true,
    stdio: ["ignore", "pipe", "pipe"],
    ...options,
  });
  child.once("error", (error) => {
    if (!shuttingDown) console.error(`${name} failed:`, error);
  });
  return attachProcessLogging(child, name, logRoot);
}

function stopChildren() {
  shuttingDown = true;
  shutdownLocalService?.();
  shutdownLocalService = null;
  for (const child of children) {
    if (child.exitCode === null) child.kill("SIGTERM");
  }
  children.clear();
}

async function createApplication() {
  const paths = resolveDesktopPaths({
    appPath: app.getAppPath(),
    resourcesPath: process.resourcesPath,
    userDataPath: app.getPath("userData"),
    packaged: app.isPackaged,
  });
  mkdirSync(paths.runtimeRoot, { recursive: true });
  mkdirSync(paths.logRoot, { recursive: true });

  for (const requiredPath of [
    paths.serverEntry,
    paths.distRoot,
    paths.cadRoot,
    paths.silExecutable,
    paths.companionEntry,
  ]) {
    if (!existsSync(requiredPath)) throw new Error(`Required desktop resource is missing: ${requiredPath}`);
  }

  const port = await availablePort();
  const serviceUrl = `http://${HOST}:${port}/`;
  const childEnvironment = {
    ...process.env,
    DOMINO_STANDALONE_PORT: String(port),
    DOMINO_PROJECT_ROOT: paths.projectRoot,
    DOMINO_DIST_ROOT: paths.distRoot,
    DOMINO_RUNTIME_ROOT: paths.runtimeRoot,
    DOMINO_FIRMWARE_BUILD_ROOT: app.isPackaged
      ? path.join(app.getPath("home"), ".domino-firmware")
      : paths.projectRoot,
    DOMINO_CAD_ROOT: paths.cadRoot,
    DOMINO_COMPANION_ENTRY: paths.companionEntry,
    DOMINO_DISABLE_RAW_HID: app.isPackaged ? "1" : (process.env.DOMINO_DISABLE_RAW_HID || "0"),
    DOMINO_EMBEDDED_DESKTOP: "1",
  };
  Object.assign(process.env, childEnvironment);
  const localService = await import("../server.mjs");
  localService.setAppUpdateCheck(requestUpdateCheck);
  await localService.serverReady;
  shutdownLocalService = localService.shutdown;

  spawnService(
    paths.silExecutable,
    [
      "--realtime",
      "--loop",
      "--control-file", path.join(paths.runtimeRoot, "controls.txt"),
      "--state-file", path.join(paths.runtimeRoot, "state.json"),
    ],
    { cwd: path.dirname(paths.silExecutable), env: process.env },
    "simulation-firmware",
    paths.logRoot,
  );

  const { workArea } = screen.getPrimaryDisplay();

  mainWindow = new BrowserWindow({
    title: APP_TITLE,
    icon: path.join(paths.appPath, "desktop", "assets", "domino-studio-icon-smooth.png"),
    x: workArea.x,
    y: workArea.y,
    width: workArea.width,
    height: workArea.height,
    minWidth: Math.min(1024, workArea.width),
    minHeight: Math.min(640, workArea.height),
    resizable: true,
    maximizable: true,
    fullscreen: true,
    fullscreenable: true,
    kiosk: false,
    backgroundColor: "#090b0b",
    autoHideMenuBar: true,
    show: false,
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true,
      webSecurity: true,
      preload: path.join(paths.appPath, "desktop", "preload.mjs"),
    },
  });

  mainWindow.webContents.setWindowOpenHandler(({ url }) => {
    if (url.startsWith("https://") || url.startsWith("http://")) void shell.openExternal(url);
    return { action: "deny" };
  });
  mainWindow.webContents.on("will-navigate", (event, url) => {
    if (!url.startsWith(serviceUrl)) event.preventDefault();
  });
  mainWindow.on("page-title-updated", (event) => {
    event.preventDefault();
    mainWindow?.setTitle(APP_TITLE);
  });
  mainWindow.on("enter-full-screen", () => {
    mainWindow?.webContents.send("domino:window:fullscreen-changed", true);
  });
  mainWindow.on("leave-full-screen", () => {
    mainWindow?.webContents.send("domino:window:fullscreen-changed", false);
  });
  mainWindow.once("ready-to-show", () => {
    mainWindow?.show();
  });
  mainWindow.on("closed", () => { mainWindow = null; });
  await mainWindow.loadURL(serviceUrl);
}

const hasSingleInstanceLock = app.requestSingleInstanceLock();
if (!hasSingleInstanceLock) app.quit();
else {
  app.on("second-instance", () => {
    if (!mainWindow) return;
    if (mainWindow.isMinimized()) mainWindow.restore();
    mainWindow.focus();
  });
  app.whenReady().then(createApplication).then(() => {
    setupAutoUpdater({ getMainWindow: () => mainWindow });
  }).catch(async (error) => {
    console.error(error);
    await dialog.showMessageBox({
      type: "error",
      title: `${APP_NAME} could not start`,
      message: "Domino Quadruped Studio could not start its offline services.",
      detail: `${error instanceof Error ? error.message : error}\n\nLogs: ${app.getPath("userData")}`,
    });
    app.quit();
  });
}

app.on("before-quit", () => {
  stopAutoUpdater();
  stopChildren();
});
app.on("window-all-closed", () => app.quit());
