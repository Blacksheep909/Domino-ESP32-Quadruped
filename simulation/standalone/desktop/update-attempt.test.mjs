import test from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";

import {
  clearUpdateAttempt,
  compareVersions,
  readUpdateAttempt,
  updateAttemptOutcome,
  writeUpdateAttempt,
} from "./update-attempt.mjs";

test("versions compare numerically, not as text", () => {
  assert.ok(compareVersions("0.5.10", "0.5.9") > 0);
  assert.ok(compareVersions("0.5.2", "0.5.4") < 0);
  assert.equal(compareVersions("0.5.4", "0.5.4"), 0);
  assert.ok(compareVersions("1.0.0", "0.9.9") > 0);
});

test("an install that left Studio on the old version counts as failed", () => {
  const attempt = { version: "0.5.5", installerPath: "C:/x/installer.exe", attemptedAt: "2026-11-09T01:00:00Z" };
  assert.deepEqual(updateAttemptOutcome(attempt, "0.5.2"), {
    status: "failed", version: "0.5.5", installerPath: "C:/x/installer.exe", attemptedAt: "2026-11-09T01:00:00Z",
  });
  assert.equal(updateAttemptOutcome(attempt, "0.5.5").status, "installed");
  assert.equal(updateAttemptOutcome(attempt, "0.5.6").status, "installed");
  assert.equal(updateAttemptOutcome(null, "0.5.2").status, "none");
  assert.equal(updateAttemptOutcome({ version: "" }, "0.5.2").status, "none");
});

test("the attempt record survives a restart and can be cleared", () => {
  const root = mkdtempSync(path.join(tmpdir(), "domino-update-attempt-"));
  try {
    assert.equal(readUpdateAttempt(root), null);
    writeUpdateAttempt(root, { version: "0.5.5", installerPath: "" });
    assert.equal(readUpdateAttempt(root).version, "0.5.5");
    clearUpdateAttempt(root);
    assert.equal(readUpdateAttempt(root), null);
  } finally {
    rmSync(root, { recursive: true, force: true });
  }
});
