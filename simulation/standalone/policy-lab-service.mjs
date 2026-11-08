// Policy Lab: drives Isaac Lab training of Domino's walking policy from Studio
// and moves trained networks into the firmware build.
//
//   train  -> simulation/isaac_lab/run-train.ps1 (headless or visible)
//   watch  -> runs/<run>/run.json + metrics.jsonl (written by scripts/train.py)
//   export -> scripts/export_policy.py <checkpoint>   (C header + policy.json)
//   install-> copies policy_weights.h into the firmware overlay so the next
//             Studio firmware build flashes it (SB fully down = policy mode).
import { spawn, spawnSync } from "node:child_process";
import {
  copyFileSync,
  existsSync,
  mkdirSync,
  openSync,
  readFileSync,
  readdirSync,
  statSync,
  writeFileSync,
} from "node:fs";
import { homedir } from "node:os";
import path from "node:path";

const RUN_NAME_PATTERN = /^[A-Za-z0-9][A-Za-z0-9_.-]{0,80}$/;
const CHECKPOINT_PATTERN = /^model_\d+\.pt$/;
const MAX_METRIC_LINES = 4000;

export function defaultLabCandidates(repoRoot) {
  const home = homedir();
  return [
    path.join(repoRoot, "simulation", "isaac_lab"),
    path.join(home, "Documents", "DominoQuadruped", "simulation", "isaac_lab"),
    path.join(home, "Documents", "Domino-ESP32-Quadruped", "simulation", "isaac_lab"),
  ];
}

export function isLabRoot(candidate) {
  return Boolean(candidate) &&
    existsSync(path.join(candidate, "run-train.ps1")) &&
    existsSync(path.join(candidate, "config", "training.json")) &&
    existsSync(path.join(candidate, "scripts", "export_policy.py"));
}

function readJson(file, fallback = null) {
  try {
    return JSON.parse(readFileSync(file, "utf8"));
  } catch {
    return fallback;
  }
}

function pidAlive(pid) {
  if (!Number.isInteger(pid) || pid <= 0) return false;
  try {
    process.kill(pid, 0);
    return true;
  } catch (error) {
    return error.code === "EPERM";
  }
}

// Parses the defines at the top of a generated (or placeholder) policy header.
export function parsePolicyHeader(text) {
  const define = (name) => {
    const match = new RegExp(`#define\\s+${name}\\s+("?)([^"\\n]*)\\1`).exec(text);
    return match ? match[2].trim() : null;
  };
  const available = define("DOMINO_POLICY_AVAILABLE") === "1";
  const obsDim = /kPolicyObsDim\s*=\s*(\d+)/.exec(text);
  const dims = /kPolicyLayerDims\[\d+\]\s*=\s*\{([^}]*)\}/.exec(text);
  return {
    available,
    id: available ? define("DOMINO_POLICY_ID") : null,
    run: available ? define("DOMINO_POLICY_RUN") : null,
    iteration: available ? Number(define("DOMINO_POLICY_ITERATION")) : null,
    obsDim: obsDim ? Number(obsDim[1]) : null,
    layerDims: dims ? dims[1].split(",").map((value) => Number(value.trim())) : null,
  };
}

// Training config validation: same shape and value types as the shipped
// default, numbers finite, [lo, hi] ranges ordered. Unknown keys rejected.
export function validateTrainingConfig(candidate, reference) {
  const errors = [];
  const walk = (value, ref, where) => {
    if (Array.isArray(ref)) {
      if (!Array.isArray(value) || value.length !== ref.length) {
        errors.push(`${where} must be a list of ${ref.length}`);
        return;
      }
      value.forEach((item, index) => walk(item, ref[index], `${where}[${index}]`));
      if (ref.length === 2 && typeof ref[0] === "number" && value[0] > value[1]) {
        errors.push(`${where} must be [low, high]`);
      }
      return;
    }
    if (ref !== null && typeof ref === "object") {
      if (value === null || typeof value !== "object" || Array.isArray(value)) {
        errors.push(`${where} must be an object`);
        return;
      }
      for (const key of Object.keys(value)) {
        if (!(key in ref)) errors.push(`${where}.${key} is not a known setting`);
      }
      for (const key of Object.keys(ref)) {
        if (!(key in value)) errors.push(`${where}.${key} is missing`);
        else walk(value[key], ref[key], `${where}.${key}`);
      }
      return;
    }
    if (typeof ref === "number") {
      if (typeof value !== "number" || !Number.isFinite(value)) errors.push(`${where} must be a number`);
      return;
    }
    if (typeof value !== typeof ref) errors.push(`${where} must be a ${typeof ref}`);
  };
  walk(candidate, reference, "config");
  if (!errors.length) {
    const t = candidate.training;
    if (t.num_envs < 1 || t.num_envs > 16384) errors.push("training.num_envs must be 1-16384");
    if (t.max_iterations < 1) errors.push("training.max_iterations must be positive");
    const h = candidate.commands.height_mm;
    if (h[0] < 160 || h[1] > 300) errors.push("commands.height_mm must stay within 160-300 mm");
    if (!RUN_NAME_PATTERN.test(candidate.run_name)) errors.push("run_name may use letters, digits, '-', '_' and '.'");
  }
  return errors;
}

export function readMetrics(file, from = 0, limit = MAX_METRIC_LINES) {
  if (!existsSync(file)) return { from, next: from, records: [] };
  const lines = readFileSync(file, "utf8").split("\n").filter(Boolean);
  const start = Math.max(0, Math.min(from, lines.length));
  const records = [];
  for (const line of lines.slice(start, start + limit)) {
    try {
      records.push(JSON.parse(line));
    } catch {
      // A line being written right now; it is read on the next poll.
      break;
    }
  }
  return { from: start, next: start + records.length, total: lines.length, records };
}

export class PolicyLabService {
  constructor({ repoRoot, runtimeRoot, overlayRoot, firmwareService = null, powershell = "powershell.exe" }) {
    this.repoRoot = repoRoot;
    this.runtimeRoot = runtimeRoot;
    this.overlayRoot = overlayRoot;
    this.firmwareService = firmwareService;
    this.powershell = powershell;
    this.settingsPath = path.join(runtimeRoot, "policy-lab.json");
    this.child = null;
    this.job = null;
    mkdirSync(runtimeRoot, { recursive: true });
  }

  settings() {
    const stored = readJson(this.settingsPath, {}) || {};
    const labRoot = [process.env.DOMINO_LAB_ROOT, stored.labRoot, ...defaultLabCandidates(this.repoRoot)]
      .find((candidate) => isLabRoot(candidate)) || null;
    const python = process.env.DOMINO_PYTHON || stored.python ||
      "C:\\isaac-projects\\sim6\\Scripts\\python.exe";
    return { labRoot, python, pythonFound: existsSync(python) };
  }

  saveSettings({ labRoot, python } = {}) {
    const next = { ...(readJson(this.settingsPath, {}) || {}) };
    if (labRoot !== undefined) {
      if (!isLabRoot(labRoot)) throw new Error("That folder is not simulation/isaac_lab (run-train.ps1 missing).");
      next.labRoot = labRoot;
    }
    if (python !== undefined) {
      if (!existsSync(python)) throw new Error("Python executable not found.");
      next.python = python;
    }
    writeFileSync(this.settingsPath, JSON.stringify(next, null, 2), "utf8");
    return this.settings();
  }

  requireLab() {
    const { labRoot } = this.settings();
    if (!labRoot) throw new Error("Isaac Lab project folder not found. Set it in Policy Lab settings.");
    return labRoot;
  }

  config() {
    const labRoot = this.requireLab();
    return readJson(path.join(labRoot, "config", "training.json"));
  }

  saveConfig(candidate) {
    const labRoot = this.requireLab();
    if (this.trainingActive()) throw new Error("Stop training before changing its settings.");
    const reference = this.config();
    const errors = validateTrainingConfig(candidate, reference);
    if (errors.length) {
      const error = new Error(errors.slice(0, 6).join("; "));
      error.statusCode = 400;
      throw error;
    }
    writeFileSync(path.join(labRoot, "config", "training.json"), `${JSON.stringify(candidate, null, 2)}\n`, "utf8");
    return candidate;
  }

  runsRoot() {
    return path.join(this.requireLab(), "runs");
  }

  runDir(name) {
    if (!RUN_NAME_PATTERN.test(String(name || ""))) throw new Error("Unknown run.");
    const dir = path.join(this.runsRoot(), name);
    if (!existsSync(path.join(dir, "run.json"))) throw new Error("Unknown run.");
    return dir;
  }

  runs(limit = 40) {
    const root = this.runsRoot();
    if (!existsSync(root)) return [];
    return readdirSync(root, { withFileTypes: true })
      .filter((entry) => entry.isDirectory() && existsSync(path.join(root, entry.name, "run.json")))
      .map((entry) => this.runSummary(entry.name))
      .sort((a, b) => (b.started || 0) - (a.started || 0))
      .slice(0, limit);
  }

  runSummary(name) {
    const dir = path.join(this.runsRoot(), name);
    const run = readJson(path.join(dir, "run.json"), {}) || {};
    const checkpoints = readdirSync(dir)
      .filter((file) => CHECKPOINT_PATTERN.test(file))
      .sort((a, b) => Number(a.match(/\d+/)[0]) - Number(b.match(/\d+/)[0]));
    let status = run.status || "unknown";
    if ((status === "training" || status === "starting") && !pidAlive(run.pid)) status = "interrupted";
    const exportInfo = readJson(path.join(dir, "policy", "policy.json"));
    return {
      name,
      status,
      started: run.started || null,
      finished: run.finished || null,
      updated: run.updated || null,
      iteration: run.iteration ?? 0,
      maxIterations: run.max_iterations ?? null,
      // A resumed run continues the checkpoint's iteration count; progress is
      // measured from where it started (model_<n>.pt -> n).
      startIteration: Number(String(run.resume || "").match(/model_(\d+)\.pt$/)?.[1] ?? 0),
      numEnvs: run.num_envs ?? null,
      meanReward: run.mean_reward ?? null,
      error: run.error || null,
      checkpoints,
      export: exportInfo,
    };
  }

  metrics(name, from = 0) {
    return readMetrics(path.join(this.runDir(name), "metrics.jsonl"), Number(from) || 0);
  }

  runConfig(name) {
    return (readJson(path.join(this.runDir(name), "run.json"), {}) || {}).config || null;
  }

  installedPolicy() {
    const overlay = path.join(this.overlayRoot, "src", "policy_weights.h");
    const bundled = path.join(this.repoRoot, "src", "policy_weights.h");
    const source = existsSync(overlay) ? overlay : bundled;
    if (!existsSync(source)) return { available: false, source: null };
    const info = parsePolicyHeader(readFileSync(source, "utf8"));
    return { ...info, source: source === overlay ? "studio" : "firmware-source", installedAt: statSync(source).mtimeMs };
  }

  trainingActive() {
    return Boolean(this.child && this.job && ["starting", "running", "stopping"].includes(this.job.status));
  }

  status() {
    const settings = this.settings();
    let runs = [];
    let configError = null;
    try {
      runs = settings.labRoot ? this.runs() : [];
    } catch (error) {
      configError = error.message;
    }
    return {
      settings,
      ready: Boolean(settings.labRoot && settings.pythonFound),
      job: this.job,
      runs,
      installed: this.installedPolicy(),
      error: configError,
    };
  }

  launch(kind, scriptArgs) {
    const labRoot = this.requireLab();
    const { python, pythonFound } = this.settings();
    if (!pythonFound) throw new Error(`Isaac Lab Python not found at ${python}.`);
    if (this.trainingActive()) throw new Error("A Policy Lab job is already running.");
    mkdirSync(path.join(labRoot, "runs"), { recursive: true });
    const stamp = new Date().toISOString().replaceAll(":", "-").replace(/\..*/, "");
    const logPath = path.join(labRoot, "runs", `_studio-${kind}-${stamp}.log`);
    const log = openSync(logPath, "a");
    const args = ["-NoProfile", "-ExecutionPolicy", "Bypass", "-File", path.join(labRoot, "run-train.ps1"),
      "-Python", python, ...scriptArgs];
    const child = spawn(this.powershell, args, {
      cwd: labRoot,
      windowsHide: true,
      stdio: ["ignore", log, log],
      env: { ...process.env, PYTHONUNBUFFERED: "1" },
    });
    this.child = child;
    this.job = { kind, status: "running", pid: child.pid, startedAt: Date.now(), logPath, exitCode: null, args: scriptArgs };
    const job = this.job;
    child.on("error", (error) => {
      job.status = "failed";
      job.error = error.message;
    });
    child.on("exit", (code) => {
      job.exitCode = code;
      job.finishedAt = Date.now();
      job.status = job.status === "stopping" ? "stopped" : code === 0 ? "finished" : "failed";
      if (this.child === child) this.child = null;
    });
    return this.job;
  }

  startTraining({ numEnvs, iterations, runName, resume, visible } = {}) {
    const args = [];
    if (numEnvs !== undefined) {
      const value = Number(numEnvs);
      if (!Number.isInteger(value) || value < 1 || value > 16384) throw new Error("Robots must be 1-16384.");
      args.push("-NumEnvs", String(value));
    }
    if (iterations !== undefined) {
      const value = Number(iterations);
      if (!Number.isInteger(value) || value < 1 || value > 100000) throw new Error("Iterations must be 1-100000.");
      args.push("-Iterations", String(value));
    }
    if (runName) {
      if (!RUN_NAME_PATTERN.test(runName)) throw new Error("Run names may use letters, digits, '-', '_' and '.'.");
      args.push("-RunName", runName);
    }
    if (resume) {
      const { run, checkpoint } = resume;
      if (!CHECKPOINT_PATTERN.test(String(checkpoint))) throw new Error("Unknown checkpoint.");
      const file = path.join(this.runDir(run), checkpoint);
      if (!existsSync(file)) throw new Error("Unknown checkpoint.");
      args.push("-Resume", file);
    }
    if (visible) args.push("-Visible");
    return this.launch("train", args);
  }

  startPlayback({ run, checkpoint, numEnvs, drive = false } = {}) {
    if (!CHECKPOINT_PATTERN.test(String(checkpoint))) throw new Error("Unknown checkpoint.");
    const file = path.join(this.runDir(run), checkpoint);
    if (!existsSync(file)) throw new Error("Unknown checkpoint.");
    const robots = Math.max(1, Math.min(64, Number(numEnvs) || (drive ? 1 : 16)));
    const args = ["-Play", "-Visible", "-Checkpoint", file, "-NumEnvs", String(robots)];
    if (drive) args.push("-Drive");
    return this.launch(drive ? "drive" : "play", args);
  }

  stop() {
    if (!this.trainingActive()) return false;
    this.job.status = "stopping";
    // Ends the whole tree Studio started (PowerShell -> Python -> Kit).
    spawnSync("taskkill", ["/PID", String(this.child.pid), "/T", "/F"], { windowsHide: true });
    return true;
  }

  exportPolicy({ run, checkpoint } = {}) {
    const dir = this.runDir(run);
    const checkpoints = this.runSummary(run).checkpoints;
    const chosen = checkpoint || checkpoints[checkpoints.length - 1];
    if (!chosen || !CHECKPOINT_PATTERN.test(chosen) || !existsSync(path.join(dir, chosen))) {
      throw new Error("This run has no checkpoint to export yet.");
    }
    const { python, pythonFound } = this.settings();
    if (!pythonFound) throw new Error(`Isaac Lab Python not found at ${python}.`);
    const labRoot = this.requireLab();
    const result = spawnSync(python, [path.join(labRoot, "scripts", "export_policy.py"), path.join(dir, chosen), path.join(dir, "policy")], {
      cwd: labRoot,
      windowsHide: true,
      encoding: "utf8",
      timeout: 120000,
      env: { ...process.env, PYTHONPATH: labRoot },
    });
    if (result.status !== 0) {
      throw new Error(`Export failed: ${(result.stderr || result.stdout || "").trim().split("\n").slice(-3).join(" ")}`);
    }
    return readJson(path.join(dir, "policy", "policy.json"));
  }

  install({ run } = {}) {
    const dir = this.runDir(run);
    const header = path.join(dir, "policy", "policy_weights.h");
    if (!existsSync(header)) throw new Error("Export this run before installing it.");
    const info = parsePolicyHeader(readFileSync(header, "utf8"));
    if (!info.available || info.obsDim !== 60) throw new Error("This export does not match firmware's 60-value observation contract.");
    const destination = path.join(this.overlayRoot, "src", "policy_weights.h");
    mkdirSync(path.dirname(destination), { recursive: true });
    copyFileSync(header, destination);
    return this.installedPolicy();
  }

  uninstall() {
    const destination = path.join(this.overlayRoot, "src", "policy_weights.h");
    if (existsSync(destination)) {
      writeFileSync(destination, "// Removed in Studio Policy Lab.\n#pragma once\n#define DOMINO_POLICY_AVAILABLE 0\n", "utf8");
    }
    return this.installedPolicy();
  }

  log(lines = 200) {
    if (!this.job?.logPath || !existsSync(this.job.logPath)) return [];
    const text = readFileSync(this.job.logPath, "utf8");
    return text.split(/\r?\n/)
      .filter((line) => line && !/\[(Warning|Info)\]|omni\.usd|Unresolved reference/.test(line))
      .slice(-lines);
  }
}
