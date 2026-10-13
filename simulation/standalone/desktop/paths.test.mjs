import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

import { resolveDesktopPaths } from "./paths.mjs";

test("desktop package includes the service dependency modules", () => {
  const manifest = JSON.parse(readFileSync(new URL("../package.json", import.meta.url), "utf8"));
  for (const module of ["companion-process.mjs", "firmware-link.mjs"]) {
    assert.ok(manifest.build.files.includes(module), `${module} must ship with the desktop app`);
  }
  const prepareSource = readFileSync(new URL("./prepare-resources.mjs", import.meta.url), "utf8");
  assert.ok(prepareSource.includes('"companion-process.mjs"'), "offline companion must include its process helper");
});

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
