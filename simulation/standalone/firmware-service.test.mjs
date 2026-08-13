import assert from "node:assert/strict";
import { mkdtempSync, mkdirSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";
import test from "node:test";

import { collectFirmwarePackage, resolveReviewFile } from "./firmware-service.mjs";

function fixture() {
  const root = mkdtempSync(path.join(tmpdir(), "domino-firmware-"));
  mkdirSync(path.join(root, "src"));
  mkdirSync(path.join(root, "private"));
  writeFileSync(path.join(root, "platformio.ini"), "[env:esp32dev]\nboard = esp32dev\n");
  writeFileSync(path.join(root, "src", "main.cpp"), "void setup() {}\nvoid loop() {}\n");
  writeFileSync(path.join(root, "private", "secret.txt"), "not deployable\n");
  return root;
}

test("firmware package includes only approved project roots", () => {
  const root = fixture();
  const firmwarePackage = collectFirmwarePackage(root);
  assert.deepEqual(firmwarePackage.files.map((file) => file.path), ["platformio.ini", "src/main.cpp"]);
  assert.equal(resolveReviewFile(root, "private/secret.txt"), null);
  assert.match(resolveReviewFile(root, "src/main.cpp").contents, /void setup/);
});

test("firmware package hash changes when deployable source changes", () => {
  const root = fixture();
  const before = collectFirmwarePackage(root).hash;
  writeFileSync(path.join(root, "src", "main.cpp"), "void setup() {}\nvoid loop() { delay(1); }\n");
  const after = collectFirmwarePackage(root).hash;
  assert.notEqual(after, before);
});
