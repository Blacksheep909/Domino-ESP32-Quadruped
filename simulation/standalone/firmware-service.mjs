import { createHash } from "node:crypto";
import { spawn } from "node:child_process";
import {
  appendFileSync,
  existsSync,
  mkdirSync,
  readFileSync,
  readdirSync,
  statSync,
} from "node:fs";
import { homedir } from "node:os";
import path from "node:path";

const PACKAGE_ROOTS = ["platformio.ini", "src", "include", "lib"];
const REVIEWABLE_EXTENSIONS = new Set([".ini", ".cpp", ".c", ".h", ".hpp", ".md", ".txt"]);
const MAX_REVIEW_BYTES = 512 * 1024;

function normalizedRelative(root, filePath) {
  return path.relative(root, filePath).split(path.sep).join("/");
}

function walkFiles(root, entry) {
  const absolute = path.join(root, entry);
  if (!existsSync(absolute)) return [];
  if (statSync(absolute).isFile()) return [absolute];
  return readdirSync(absolute, { withFileTypes: true })
    .sort((a, b) => a.name.localeCompare(b.name))
    .flatMap((item) => {
      if (item.name.startsWith(".")) return [];
      const child = path.join(absolute, item.name);
      return item.isDirectory() ? walkFiles(root, normalizedRelative(root, child)) : [child];
    });
}

export function collectFirmwarePackage(projectRoot) {
  const files = PACKAGE_ROOTS.flatMap((entry) => walkFiles(projectRoot, entry))
    .filter((filePath) => REVIEWABLE_EXTENSIONS.has(path.extname(filePath).toLowerCase()))
    .map((filePath) => {
      const contents = readFileSync(filePath);
      return {
        path: normalizedRelative(projectRoot, filePath),
        size: contents.byteLength,
        hash: createHash("sha256").update(contents).digest("hex"),
      };
    });
  const digest = createHash("sha256");
  files.forEach((file) => digest.update(`${file.path}\0${file.hash}\0`));
  return {
    environment: "esp32dev",
    board: "Espressif ESP32 Dev Module",
    framework: "Arduino",
    hash: digest.digest("hex"),
    files,
    bytes: files.reduce((total, file) => total + file.size, 0),
  };
}

export function resolveReviewFile(projectRoot, requestedPath) {
  const manifest = collectFirmwarePackage(projectRoot);
  const file = manifest.files.find((candidate) => candidate.path === requestedPath);
  if (!file) return null;
  const absolute = path.resolve(projectRoot, ...file.path.split("/"));
  if (statSync(absolute).size > MAX_REVIEW_BYTES) return null;
  return { ...file, contents: readFileSync(absolute, "utf8") };
}

function findPlatformio() {
  const executable = process.platform === "win32" ? "platformio.exe" : "platformio";
  const candidates = [
    process.env.PLATFORMIO_CMD,
    path.join(homedir(), ".platformio", "penv", process.platform === "win32" ? "Scripts" : "bin", executable),
    executable,
  ].filter(Boolean);
  return candidates.find((candidate) => candidate === executable || existsSync(candidate)) || null;
}

function progressFor(type, text, current) {
  const percent = text.match(/(?:Writing at|Progress:).*?\(?([0-9]{1,3})\s*%/i);
  if (percent) return Math.max(current, type === "upload" ? 35 + Number(percent[1]) * 0.62 : Number(percent[1]));
  if (/Compiling|Building/i.test(text)) return Math.max(current, 38);
  if (/Linking/i.test(text)) return Math.max(current, 68);
  if (/Checking size|RAM:|Flash:/i.test(text)) return Math.max(current, 82);
  if (/Connecting/i.test(text)) return Math.max(current, 24);
  if (/Chip is|Uploading stub/i.test(text)) return Math.max(current, 32);
  if (/Hash of data verified|Leaving/i.test(text)) return Math.max(current, 97);
  return current;
}

export class FirmwareService {
  constructor({ projectRoot, runtimeRoot }) {
    this.projectRoot = projectRoot;
    this.runtimeRoot = path.join(runtimeRoot, "firmware-jobs");
    this.platformio = findPlatformio();
    this.job = null;
    this.child = null;
    this.lastBuild = null;
    mkdirSync(this.runtimeRoot, { recursive: true });
  }

  package() {
    return collectFirmwarePackage(this.projectRoot);
  }

  file(requestedPath) {
    return resolveReviewFile(this.projectRoot, requestedPath);
  }

  publicJob() {
    if (!this.job) return null;
    return { ...this.job, logs: this.job.logs.slice(-500) };
  }

  async ports() {
    if (!this.platformio) return [];
    return new Promise((resolve) => {
      const child = spawn(this.platformio, ["device", "list", "--json-output"], {
        cwd: this.projectRoot,
        windowsHide: true,
      });
      let output = "";
      child.stdout.on("data", (chunk) => { output += chunk; });
      child.on("error", () => resolve([]));
      child.on("close", () => {
        try {
          const devices = JSON.parse(output);
          resolve(devices.map((device) => ({
            port: device.port,
            description: device.description || "Serial device",
            hwid: device.hwid || "",
          })));
        } catch {
          resolve([]);
        }
      });
    });
  }

  status() {
    return {
      toolchain: { available: Boolean(this.platformio), command: this.platformio || "Not found" },
      package: this.package(),
      lastBuild: this.lastBuild,
      job: this.publicJob(),
    };
  }

  startBuild() {
    return this.startJob("build", ["run", "-e", "esp32dev"]);
  }

  startUpload({ confirmation, port } = {}) {
    if (confirmation !== "BOOT HELD") throw new Error("Confirm that BOOT is held before uploading.");
    const currentPackage = this.package();
    if (!this.lastBuild?.ok || this.lastBuild.packageHash !== currentPackage.hash) {
      throw new Error("Build this exact firmware package before uploading it.");
    }
    const args = ["run", "-e", "esp32dev", "-t", "upload"];
    if (port) args.push("--upload-port", String(port));
    return this.startJob("upload", args, currentPackage);
  }

  cancel() {
    if (!this.child || !this.job || this.job.status !== "running") return false;
    this.job.status = "cancelling";
    this.child.kill("SIGTERM");
    return true;
  }

  startJob(type, args, firmwarePackage = this.package()) {
    if (!this.platformio) throw new Error("PlatformIO is not installed or could not be located.");
    if (this.job?.status === "running" || this.job?.status === "cancelling") {
      throw new Error("Another firmware job is already running.");
    }
    const id = `${Date.now()}-${type}`;
    const logPath = path.join(this.runtimeRoot, `${id}.jsonl`);
    this.job = {
      id,
      type,
      status: "running",
      stage: type === "build" ? "Validating package" : "Waiting for ESP32 bootloader",
      progress: 4,
      packageHash: firmwarePackage.hash,
      startedAt: new Date().toISOString(),
      finishedAt: null,
      exitCode: null,
      logs: [],
    };
    const record = (stream, text) => {
      const clean = String(text).replace(/\x1b\[[0-9;]*m/g, "");
      if (!clean) return;
      const entry = { at: new Date().toISOString(), stream, text: clean };
      this.job.logs.push(entry);
      if (this.job.logs.length > 1000) this.job.logs.shift();
      this.job.progress = Math.min(99, progressFor(type, clean, this.job.progress));
      if (/Compiling|Building/i.test(clean)) this.job.stage = "Compiling firmware";
      if (/Linking/i.test(clean)) this.job.stage = "Linking application";
      if (/Checking size|RAM:|Flash:/i.test(clean)) this.job.stage = "Checking flash and memory";
      if (/Connecting/i.test(clean)) this.job.stage = "Connecting to ESP32";
      if (/Writing at/i.test(clean)) this.job.stage = "Writing flash";
      if (/Hash of data verified/i.test(clean)) this.job.stage = "Verifying flash";
      appendFileSync(logPath, `${JSON.stringify(entry)}\n`, "utf8");
    };
    this.child = spawn(this.platformio, args, {
      cwd: this.projectRoot,
      windowsHide: true,
      env: { ...process.env, PLATFORMIO_CORE_DIR: path.join(homedir(), ".platformio") },
    });
    this.child.stdout.on("data", (chunk) => record("stdout", chunk));
    this.child.stderr.on("data", (chunk) => record("stderr", chunk));
    this.child.on("error", (error) => record("stderr", error.message));
    this.child.on("close", (code) => {
      const ok = code === 0;
      this.job.status = ok ? "success" : "failed";
      this.job.stage = ok ? (type === "build" ? "Package ready" : "Upload complete") : "Job failed";
      this.job.progress = ok ? 100 : this.job.progress;
      this.job.exitCode = code;
      this.job.finishedAt = new Date().toISOString();
      if (type === "build") {
        this.lastBuild = {
          ok,
          packageHash: firmwarePackage.hash,
          completedAt: this.job.finishedAt,
          jobId: id,
        };
      }
      this.child = null;
    });
    return this.publicJob();
  }
}
