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

contextBridge.exposeInMainWorld("dominoDesktop", Object.freeze({
  projectFiles: desktopProjectFiles,
  window: desktopWindow,
}));
