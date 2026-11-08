import assert from "node:assert/strict";
import { mkdtempSync, mkdirSync, readFileSync, writeFileSync, existsSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";
import test from "node:test";

import { collectFirmwarePackage, syncFirmwareWorkspace } from "./firmware-service.mjs";
import {
  PolicyLabService,
  parsePolicyHeader,
  readMetrics,
  validateTrainingConfig,
} from "./policy-lab-service.mjs";

const here = path.dirname(new URL(import.meta.url).pathname.replace(/^\/([A-Za-z]:)/, "$1"));
const shippedConfig = JSON.parse(readFileSync(path.join(here, "..", "isaac_lab", "config", "training.json"), "utf8"));

function fakeLab() {
  const root = mkdtempSync(path.join(tmpdir(), "domino-lab-"));
  const lab = path.join(root, "simulation", "isaac_lab");
  mkdirSync(path.join(lab, "config"), { recursive: true });
  mkdirSync(path.join(lab, "scripts"), { recursive: true });
  writeFileSync(path.join(lab, "run-train.ps1"), "# stub\n");
  writeFileSync(path.join(lab, "scripts", "export_policy.py"), "# stub\n");
  writeFileSync(path.join(lab, "config", "training.json"), JSON.stringify(shippedConfig));
  mkdirSync(path.join(root, "src"), { recursive: true });
  writeFileSync(path.join(root, "src", "policy_weights.h"), "#pragma once\n#define DOMINO_POLICY_AVAILABLE 0\n");
  writeFileSync(path.join(root, "platformio.ini"), "[env:esp32dev]\n");
  return { root, lab };
}

const HEADER = `#pragma once
#define DOMINO_POLICY_AVAILABLE 1
#define DOMINO_POLICY_ID "ab12cd34"
#define DOMINO_POLICY_RUN "2026-11-08_00-00-00_walk"
#define DOMINO_POLICY_ITERATION 2999
constexpr int kPolicyObsDim = 60;
constexpr int kPolicyLayerDims[5] = {60, 128, 64, 32, 12};
`;

test("parsePolicyHeader reads the exporter's defines and the placeholder", () => {
  assert.deepEqual(parsePolicyHeader(HEADER), {
    available: true, id: "ab12cd34", run: "2026-11-08_00-00-00_walk", iteration: 2999,
    obsDim: 60, layerDims: [60, 128, 64, 32, 12],
  });
  assert.equal(parsePolicyHeader("#define DOMINO_POLICY_AVAILABLE 0\n").available, false);
});

test("validateTrainingConfig accepts the shipped config and rejects bad edits", () => {
  assert.deepEqual(validateTrainingConfig(structuredClone(shippedConfig), shippedConfig), []);
  const bad = structuredClone(shippedConfig);
  bad.commands.lin_vel_x = [0.4, -0.2];
  bad.domain_randomization.foot_friction = "high";
  bad.rewards.made_up = 1;
  bad.training.num_envs = 0;
  const errors = validateTrainingConfig(bad, shippedConfig);
  assert.ok(errors.some((e) => e.includes("lin_vel_x must be [low, high]")));
  assert.ok(errors.some((e) => e.includes("foot_friction must be a list")));
  assert.ok(errors.some((e) => e.includes("made_up is not a known setting")));
});

test("readMetrics pages through JSONL and stops at a half-written line", () => {
  const dir = mkdtempSync(path.join(tmpdir(), "domino-metrics-"));
  const file = path.join(dir, "metrics.jsonl");
  writeFileSync(file, '{"it":0,"mean_reward":1}\n{"it":1,"mean_reward":2}\n{"it":2,"mean');
  const first = readMetrics(file, 0);
  assert.equal(first.records.length, 2);
  assert.equal(first.next, 2);
  writeFileSync(file, '{"it":0}\n{"it":1}\n{"it":2}\n');
  assert.deepEqual(readMetrics(file, first.next).records, [{ it: 2 }]);
  assert.equal(readMetrics(path.join(dir, "missing.jsonl")).records.length, 0);
});

test("runs, install and the firmware overlay work together", () => {
  const { root, lab } = fakeLab();
  const runtime = mkdtempSync(path.join(tmpdir(), "domino-rt-"));
  const overlay = path.join(runtime, "policy-overlay");
  process.env.DOMINO_LAB_ROOT = lab;
  try {
    const service = new PolicyLabService({ repoRoot: root, runtimeRoot: runtime, overlayRoot: overlay });
    const run = "2026-11-08_00-00-00_walk";
    const runDir = path.join(lab, "runs", run);
    mkdirSync(path.join(runDir, "policy"), { recursive: true });
    writeFileSync(path.join(runDir, "run.json"), JSON.stringify({ status: "finished", started: 1, iteration: 2999, max_iterations: 3000, pid: 1 }));
    writeFileSync(path.join(runDir, "model_100.pt"), "");
    writeFileSync(path.join(runDir, "model_2999.pt"), "");
    writeFileSync(path.join(runDir, "policy", "policy_weights.h"), HEADER);

    const [summary] = service.runs();
    assert.equal(summary.name, run);
    assert.deepEqual(summary.checkpoints, ["model_100.pt", "model_2999.pt"]);
    assert.equal(service.installedPolicy().available, false);
    assert.throws(() => service.runDir("../escape"), /Unknown run/);
    assert.throws(() => service.startPlayback({ run, checkpoint: "..\\x.pt" }), /Unknown checkpoint/);

    const installed = service.install({ run });
    assert.equal(installed.available, true);
    assert.equal(installed.source, "studio");

    // The overlay replaces the bundled placeholder in the firmware package...
    const pkg = collectFirmwarePackage(root, overlay);
    const entry = pkg.files.find((file) => file.path === "src/policy_weights.h");
    assert.equal(entry.overlay, true);
    assert.notEqual(pkg.hash, collectFirmwarePackage(root).hash);
    // ...and lands in a separate build workspace.
    const build = path.join(runtime, "build");
    syncFirmwareWorkspace(root, build, pkg, overlay);
    assert.match(readFileSync(path.join(build, "src", "policy_weights.h"), "utf8"), /ab12cd34/);

    assert.equal(service.uninstall().available, false);
    assert.ok(existsSync(path.join(overlay, "src", "policy_weights.h")));
  } finally {
    delete process.env.DOMINO_LAB_ROOT;
  }
});
