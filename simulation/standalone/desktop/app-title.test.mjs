import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

import { desktopWindowTitle } from "./app-title.mjs";

test("desktop title includes one normalized application version", () => {
  assert.equal(desktopWindowTitle("Domino Quadruped Studio", "0.2.6"), "Domino Quadruped Studio — v0.2.6");
  assert.equal(desktopWindowTitle("Domino Quadruped Studio", "v0.2.6"), "Domino Quadruped Studio — v0.2.6");
});

test("the title helper is included in packaged desktop builds", async () => {
  const packageJson = JSON.parse(await readFile(new URL("../package.json", import.meta.url), "utf8"));
  assert.ok(packageJson.build.files.includes("desktop/app-title.mjs"));
});
