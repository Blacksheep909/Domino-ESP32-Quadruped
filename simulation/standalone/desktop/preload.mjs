import { contextBridge, ipcRenderer } from "electron";

const desktopProjectFiles = Object.freeze({
  openProjectFile: () => ipcRenderer.invoke("domino:project:open"),
  saveProjectFile: ({ contents, suggestedFileName } = {}) => ipcRenderer.invoke(
    "domino:project:save",
    { contents, suggestedFileName },
  ),
});

const desktopWindow = Object.freeze({
  isFullscreen: () => ipcRenderer.invoke("domino:window:is-fullscreen"),
  toggleFullscreen: () => ipcRenderer.invoke("domino:window:toggle-fullscreen"),
  onFullscreenChange: (callback) => {
    const listener = (_event, fullscreen) => callback(Boolean(fullscreen));
    ipcRenderer.on("domino:window:fullscreen-changed", listener);
    return () => ipcRenderer.removeListener("domino:window:fullscreen-changed", listener);
  },
});

const desktopUpdates = Object.freeze({
  getState: () => ipcRenderer.invoke("domino:update:get-state"),
  check: () => ipcRenderer.invoke("domino:update:check"),
  install: () => ipcRenderer.invoke("domino:update:install"),
  runInstaller: () => ipcRenderer.invoke("domino:update:run-installer"),
  onState: (callback) => {
    const listener = (_event, state) => callback(state);
    ipcRenderer.on("domino:update:state", listener);
    return () => ipcRenderer.removeListener("domino:update:state", listener);
  },
});

contextBridge.exposeInMainWorld("dominoDesktop", Object.freeze({
  projectFiles: desktopProjectFiles,
  window: desktopWindow,
  updates: desktopUpdates,
}));
