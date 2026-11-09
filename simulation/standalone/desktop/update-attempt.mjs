// Remembers an update install that Studio started, so the next launch can tell
// whether it worked. The silent Windows installer gives no feedback: when it
// fails (for example, the old version's uninstaller refuses), Studio simply
// reopens at the old version. Comparing the recorded target with the running
// version is the only way to notice.
import { existsSync, mkdirSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import path from "node:path";

const ATTEMPT_FILE = "update-attempt.json";

function versionParts(version) {
  return String(version || "").split(/[.+-]/).slice(0, 3).map((part) => Number.parseInt(part, 10) || 0);
}

// Negative when a < b, zero when equal, positive when a > b (major.minor.patch).
export function compareVersions(a, b) {
  const left = versionParts(a);
  const right = versionParts(b);
  for (let index = 0; index < 3; index += 1) {
    if (left[index] !== right[index]) return left[index] - right[index];
  }
  return 0;
}

// The outcome of the last recorded install attempt for the running version.
export function updateAttemptOutcome(attempt, currentVersion) {
  if (!attempt || typeof attempt.version !== "string" || !attempt.version) return { status: "none" };
  if (compareVersions(currentVersion, attempt.version) >= 0) return { status: "installed", version: attempt.version };
  return {
    status: "failed",
    version: attempt.version,
    installerPath: typeof attempt.installerPath === "string" ? attempt.installerPath : "",
    attemptedAt: typeof attempt.attemptedAt === "string" ? attempt.attemptedAt : "",
  };
}

export function readUpdateAttempt(userDataPath) {
  try {
    const file = path.join(userDataPath, ATTEMPT_FILE);
    if (!existsSync(file)) return null;
    return JSON.parse(readFileSync(file, "utf8"));
  } catch {
    return null;
  }
}

export function writeUpdateAttempt(userDataPath, attempt) {
  mkdirSync(userDataPath, { recursive: true });
  writeFileSync(path.join(userDataPath, ATTEMPT_FILE), `${JSON.stringify(attempt, null, 2)}\n`, "utf8");
}

export function clearUpdateAttempt(userDataPath) {
  rmSync(path.join(userDataPath, ATTEMPT_FILE), { force: true });
}
