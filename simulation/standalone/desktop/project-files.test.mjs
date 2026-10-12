import assert from "node:assert/strict";
import { mkdtemp, readFile, rm } from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import test from "node:test";

import {
  PROJECT_FILE_MAX_BYTES,
  projectFileName,
  readProjectFile,
  resolveProjectFilePath,
  resolveProjectSaveFilePath,
  validateProjectFileContents,
  writeProjectFile,
} from "./project-files.mjs";

function bundle(name = "Dog V2") {
  return JSON.stringify({
    type: "domino-quadruped-project",
    schemaVersion: 1,
    project: { name },
  }, null, 2);
}

test("native project files keep a safe default filename", () => {
  assert.equal(projectFileName("Bench / Trot.json"), "Bench - Trot.qstudio.json");
  assert.equal(projectFileName("  "), "Domino V2.qstudio.json");
  assert.equal(path.basename(resolveProjectSaveFilePath("Dog V2")), "Dog V2.qstudio.json");
  assert.throws(() => resolveProjectFilePath("Dog V2.txt"), /extension/);
});

test("project file content rejects malformed, wrong-type, and oversized data", () => {
  assert.throws(() => validateProjectFileContents("not json"), /valid JSON/);
  assert.throws(() => validateProjectFileContents(JSON.stringify({ type: "other" })), /not a Domino/);
  assert.throws(
    () => validateProjectFileContents(JSON.stringify({ type: "domino-quadruped-project", note: "x".repeat(PROJECT_FILE_MAX_BYTES) })),
    /too large/,
  );
});

test("project files write atomically and read back as validated text", async () => {
  const root = await mkdtemp(path.join(os.tmpdir(), "domino-project-files-"));
  const target = path.join(root, "Dog V2.qstudio.json");
  try {
    const first = await writeProjectFile(target, bundle("First project"));
    assert.equal(first.fileName, "Dog V2.qstudio.json");

    const secondContents = bundle("Second project");
    await writeProjectFile(target, secondContents);
    assert.equal(await readFile(target, "utf8"), secondContents);

    const restored = await readProjectFile(target);
    assert.equal(restored.fileName, "Dog V2.qstudio.json");
    assert.equal(JSON.parse(restored.contents).project.name, "Second project");
  } finally {
    await rm(root, { recursive: true, force: true });
  }
});
