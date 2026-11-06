import path from "node:path";

// build.ps1 produces domino_sil.exe on Windows; build.sh produces domino_sil elsewhere.
export const SIL_EXECUTABLE_NAME = process.platform === "win32" ? "domino_sil.exe" : "domino_sil";

export function resolveDesktopPaths({
  appPath,
  resourcesPath,
  userDataPath,
  packaged,
}) {
  const standaloneRoot = path.resolve(appPath);
  const repoRoot = packaged
    ? path.join(resourcesPath, "domino", "firmware")
    : path.resolve(standaloneRoot, "../..");
  const bundledRoot = packaged ? path.join(resourcesPath, "domino") : null;

  return {
    appPath: standaloneRoot,
    serverEntry: path.join(standaloneRoot, "server.mjs"),
    distRoot: path.join(standaloneRoot, "dist"),
    runtimeRoot: path.join(userDataPath, "runtime"),
    logRoot: path.join(userDataPath, "logs"),
    projectRoot: repoRoot,
    cadRoot: packaged
      ? path.join(bundledRoot, "cad")
      : path.join(
          repoRoot,
          "simulation",
          "urdf",
          "generated",
          "Domino_URDF_Parts_Combined_Final_description",
          "meshes",
        ),
    silExecutable: packaged
      ? path.join(bundledRoot, "sil", SIL_EXECUTABLE_NAME)
      : path.join(repoRoot, "simulation", "sil", "bin", SIL_EXECUTABLE_NAME),
    companionEntry: packaged
      ? path.join(bundledRoot, "companion", "live-companion-adapter.mjs")
      : path.join(standaloneRoot, "live-companion-adapter.mjs"),
  };
}
