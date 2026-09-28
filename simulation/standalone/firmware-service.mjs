import { createHash } from "node:crypto";
import { spawn } from "node:child_process";
import {
  appendFileSync,
  copyFileSync,
  existsSync,
  mkdirSync,
  readFileSync,
  readdirSync,
  statSync,
  symlinkSync,
  unlinkSync,
  writeFileSync,
} from "node:fs";
import { homedir } from "node:os";
import path from "node:path";

const PACKAGE_ROOTS = ["platformio.ini", "src", "include", "lib"];
const REVIEWABLE_EXTENSIONS = new Set([".ini", ".cpp", ".c", ".h", ".hpp", ".md", ".txt"]);
const MAX_REVIEW_BYTES = 512 * 1024;
const WORKSPACE_MARKER = ".domino-firmware-workspace.json";

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

export function syncFirmwareWorkspace(projectRoot, buildRoot, firmwarePackage) {
  const sourceRoot = path.resolve(projectRoot);
  const destinationRoot = path.resolve(buildRoot);
  if (sourceRoot === destinationRoot) return destinationRoot;
  if (path.relative(sourceRoot, destinationRoot) === "" ||
      !path.relative(sourceRoot, destinationRoot).startsWith("..") &&
      !path.isAbsolute(path.relative(sourceRoot, destinationRoot))) {
    throw new Error("Firmware build workspace must be outside the bundled source directory.");
  }
  mkdirSync(destinationRoot, { recursive: true });
  const markerPath = path.join(destinationRoot, WORKSPACE_MARKER);
  const prior = existsSync(markerPath) ? JSON.parse(readFileSync(markerPath, "utf8")) : null;
  if (!prior && readdirSync(destinationRoot).length !== 0) {
    throw new Error("Firmware build workspace contains unrecognized files.");
  }
  if (prior && (prior.format !== 1 || prior.sourceRoot !== sourceRoot ||
      !Array.isArray(prior.files))) {
    throw new Error("Firmware build workspace belongs to a different source package.");
  }
  const nextFiles = new Set(firmwarePackage.files.map((file) => file.path));
  for (const oldFile of prior?.files || []) {
    if (!nextFiles.has(oldFile) && /^(platformio\.ini|(?:src|include|lib)\/[\w./-]+)$/.test(oldFile)) {
      const obsolete = path.resolve(destinationRoot, ...oldFile.split("/"));
      if (path.relative(destinationRoot, obsolete).startsWith("..")) continue;
      if (existsSync(obsolete)) unlinkSync(obsolete);
    }
  }
  for (const file of firmwarePackage.files) {
    const components = file.path.split("/");
    const source = path.resolve(sourceRoot, ...components);
    const destination = path.resolve(destinationRoot, ...components);
    if (path.relative(destinationRoot, destination).startsWith("..")) {
      throw new Error("Firmware package contains an unsafe path.");
    }
    mkdirSync(path.dirname(destination), { recursive: true });
    copyFileSync(source, destination);
  }
  writeFileSync(markerPath, JSON.stringify({
    format: 1, sourceRoot, packageHash: firmwarePackage.hash,
    files: [...nextFiles],
  }), "utf8");
  return destinationRoot;
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

export function preparePlatformioEnvironment(
  runtimeRoot,
  installedCore = path.join(homedir(), ".platformio"),
  coreDir = process.platform === "win32"
    ? path.join(homedir(), ".domino-platformio")
    : path.join(runtimeRoot, "platformio-core"),
) {
  // ESP32's GCC receives hundreds of framework include paths. Repeating the
  // long Studio userData path can exceed Windows' process command-line limit
  // when GCC launches cc1plus, reported only as "CreateProcess" failure.
  mkdirSync(coreDir, { recursive: true });

  // PlatformIO places its package-manager lock files beside `platforms` and
  // `packages`. The desktop companion can read the user's installed toolchain,
  // but its restricted process may not write to ~/.platformio. Keep locks and
  // mutable state in the app runtime while reusing the large installed folders.
  for (const directory of ["platforms", "packages"]) {
    const source = path.join(installedCore, directory);
    const destination = path.join(coreDir, directory);
    if (existsSync(source) && !existsSync(destination)) {
      symlinkSync(source, destination, process.platform === "win32" ? "junction" : "dir");
    }
  }

  return {
    ...process.env,
    PLATFORMIO_CORE_DIR: coreDir,
    PLATFORMIO_SETTING_ENABLE_TELEMETRY: "No",
  };
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
  constructor({ projectRoot, runtimeRoot, buildRoot = projectRoot, onJobFinished = null }) {
    this.projectRoot = projectRoot;
    this.buildRoot = buildRoot;
    this.runtimeRoot = path.join(runtimeRoot, "firmware-jobs");
    this.platformioEnvironment = preparePlatformioEnvironment(runtimeRoot);
    this.platformio = findPlatformio();
    this.job = null;
    this.child = null;
    this.cancelRequested = false;
    this.lastBuild = null;
    this.onJobFinished = typeof onJobFinished === "function" ? onJobFinished : null;
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
        env: this.platformioEnvironment,
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
    this.cancelRequested = true;
    this.job.status = "cancelling";
    this.job.stage = "Cancelling job";
    this.child.kill("SIGTERM");
    return true;
  }

  startJob(type, args, firmwarePackage = this.package()) {
    if (!this.platformio) throw new Error("PlatformIO is not installed or could not be located.");
    if (this.job?.status === "running" || this.job?.status === "cancelling") {
      throw new Error("Another firmware job is already running.");
    }
    syncFirmwareWorkspace(this.projectRoot, this.buildRoot, firmwarePackage);
    this.cancelRequested = false;
    const id = `${Date.now()}-${type}`;
    const logPath = path.join(this.runtimeRoot, `${id}.jsonl`);
    const job = {
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
    this.job = job;
    const record = (stream, text) => {
      const clean = String(text).replace(/\x1b\[[0-9;]*m/g, "");
      if (!clean) return;
      const entry = { at: new Date().toISOString(), stream, text: clean };
      job.logs.push(entry);
      if (job.logs.length > 1000) job.logs.shift();
      job.progress = Math.min(99, progressFor(type, clean, job.progress));
      if (/Compiling|Building/i.test(clean)) job.stage = "Compiling firmware";
      if (/Linking/i.test(clean)) job.stage = "Linking application";
      if (/Checking size|RAM:|Flash:/i.test(clean)) job.stage = "Checking flash and memory";
      if (/Connecting/i.test(clean)) job.stage = "Connecting to ESP32";
      if (/Writing at/i.test(clean)) job.stage = "Writing flash";
      if (/Hash of data verified/i.test(clean)) job.stage = "Verifying flash";
      appendFileSync(logPath, `${JSON.stringify(entry)}\n`, "utf8");
    };
    this.child = spawn(this.platformio, args, {
      cwd: this.buildRoot,
      windowsHide: true,
      env: this.platformioEnvironment,
    });
    this.child.stdout.on("data", (chunk) => record("stdout", chunk));
    this.child.stderr.on("data", (chunk) => record("stderr", chunk));
    this.child.on("error", (error) => record("stderr", error.message));
    this.child.on("close", (code) => {
      const cancelled = this.cancelRequested || job.status === "cancelling";
      const ok = !cancelled && code === 0;
      job.status = cancelled ? "cancelled" : ok ? "success" : "failed";
      job.stage = cancelled
        ? "Job cancelled"
        : ok
          ? (type === "build" ? "Package ready" : "Upload complete")
          : "Job failed";
      job.progress = ok ? 100 : job.progress;
      job.exitCode = code;
      job.finishedAt = new Date().toISOString();
      if (type === "build" && !cancelled) {
        this.lastBuild = {
          ok,
          packageHash: firmwarePackage.hash,
          completedAt: this.job.finishedAt,
          jobId: id,
        };
      }
      if (this.job === job) {
        this.child = null;
        this.cancelRequested = false;
      }
      if (this.onJobFinished) {
        try {
          const result = this.onJobFinished({ ...job, logs: job.logs.slice(-500) });
          result?.catch?.((error) => console.error("Firmware completion hook failed:", error));
        } catch (error) {
          console.error("Firmware completion hook failed:", error);
        }
      }
    });
    return this.publicJob();
  }
}
