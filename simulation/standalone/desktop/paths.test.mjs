import assert from "node:assert/strict";
import path from "node:path";
import test from "node:test";

import { resolveDesktopPaths } from "./paths.mjs";

test("development paths use the checked-out repository and private user runtime", () => {
  const paths = resolveDesktopPaths({
    appPath: path.join("C:", "repo", "simulation", "standalone"),
    resourcesPath: path.join("C:", "electron", "resources"),
    userDataPath: path.join("C:", "users", "tester", "Domino"),
    packaged: false,
  });

  assert.equal(paths.projectRoot, path.resolve("C:\\repo"));
  assert.equal(paths.runtimeRoot, path.resolve("C:\\users\\tester\\Domino\\runtime"));
  assert.equal(paths.silExecutable, path.resolve("C:\\repo\\simulation\\sil\\bin\\domino_sil.exe"));
});

test("packaged paths keep mutable data outside installed application resources", () => {
  const paths = resolveDesktopPaths({
    appPath: path.join("C:", "Program Files", "Domino", "resources", "app"),
    resourcesPath: path.join("C:", "Program Files", "Domino", "resources"),
    userDataPath: path.join("C:", "users", "tester", "AppData", "Roaming", "Domino"),
    packaged: true,
  });

  assert.equal(
    paths.projectRoot,
    path.resolve("C:\\Program Files\\Domino\\resources\\domino\\firmware"),
  );
  assert.equal(
    paths.cadRoot,
    path.resolve("C:\\Program Files\\Domino\\resources\\domino\\cad"),
  );
  assert.equal(
    paths.runtimeRoot,
    path.resolve("C:\\users\\tester\\AppData\\Roaming\\Domino\\runtime"),
  );
  assert.equal(
    paths.companionEntry,
    path.resolve("C:\\Program Files\\Domino\\resources\\domino\\companion\\live-companion-adapter.mjs"),
  );
});
