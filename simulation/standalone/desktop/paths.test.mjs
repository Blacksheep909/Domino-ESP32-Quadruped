import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import test from "node:test";

import { resolveDesktopPaths, SIL_EXECUTABLE_NAME } from "./paths.mjs";

// One absolute root on every OS ("C:\\domino-test" on Windows, "/domino-test"
// elsewhere), so the inputs and expected paths compare the same way.
const ROOT = path.resolve(path.sep, "domino-test");
const at = (...parts) => path.join(ROOT, ...parts);
const expected = at;

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
    appPath: at("repo", "simulation", "standalone"),
    resourcesPath: at("electron", "resources"),
    userDataPath: at("users", "tester", "Domino"),
    packaged: false,
  });

  assert.equal(paths.projectRoot, expected("repo"));
  assert.equal(paths.runtimeRoot, expected("users", "tester", "Domino", "runtime"));
  assert.equal(paths.silExecutable, expected("repo", "simulation", "sil", "bin", SIL_EXECUTABLE_NAME));
});

test("packaged paths keep mutable data outside installed application resources", () => {
  const paths = resolveDesktopPaths({
    appPath: at("Program Files", "Domino", "resources", "app"),
    resourcesPath: at("Program Files", "Domino", "resources"),
    userDataPath: at("users", "tester", "AppData", "Roaming", "Domino"),
    packaged: true,
  });

  assert.equal(
    paths.projectRoot,
    expected("Program Files", "Domino", "resources", "domino", "firmware"),
  );
  assert.equal(
    paths.cadRoot,
    expected("Program Files", "Domino", "resources", "domino", "cad"),
  );
  assert.equal(
    paths.runtimeRoot,
    expected("users", "tester", "AppData", "Roaming", "Domino", "runtime"),
  );
  assert.equal(
    paths.companionEntry,
    expected("Program Files", "Domino", "resources", "domino", "companion", "live-companion-adapter.mjs"),
  );
});
