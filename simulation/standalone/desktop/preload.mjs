import { contextBridge, ipcRenderer } from "electron";

const desktopProjectFiles = Object.freeze({
  openProjectFile: () => ipcRenderer.invoke("domino:project:open"),
  saveProjectFile: ({ contents, suggestedFileName } = {}) => ipcRenderer.invoke(
    "domino:project:save",
    { contents, suggestedFileName },
  ),
});

contextBridge.exposeInMainWorld("dominoDesktop", Object.freeze({
  projectFiles: desktopProjectFiles,
}));
