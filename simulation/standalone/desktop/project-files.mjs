import { randomUUID } from "node:crypto";
import { lstat, readFile, rename, rm, writeFile } from "node:fs/promises";
import path from "node:path";

export const PROJECT_FILE_MAX_BYTES = 2 * 1024 * 1024;
export const PROJECT_FILE_TYPE = "domino-quadruped-project";

function isSupportedProjectFileName(fileName) {
  const normalized = String(fileName || "").toLowerCase();
  return normalized.endsWith(".qstudio.json") || normalized.endsWith(".json");
}

export function projectFileName(suggestedFileName = "Domino V2.qstudio.json") {
  const fileName = String(suggestedFileName || "")
    .replace(/[\u0000-\u001f\u007f]/g, " ")
    .replace(/[\\/]+/g, "-")
    .replace(/(?:\.qstudio)?\.json$/i, "")
    .replace(/[<>:"/\\|?*]/g, "-")
    .replace(/\s+/g, " ")
    .trim()
    .slice(0, 80);
  return `${fileName || "Domino V2"}.qstudio.json`;
}

export function resolveProjectFilePath(filePath) {
  const candidate = String(filePath || "").trim();
  if (!candidate) throw new Error("Choose a project file first.");
  const resolved = path.resolve(candidate);
  if (!isSupportedProjectFileName(path.basename(resolved))) {
    throw new Error("Project files must use a .qstudio.json or .json extension.");
  }
  return resolved;
}

export function resolveProjectSaveFilePath(filePath) {
  const candidate = String(filePath || "").trim();
  if (!candidate) throw new Error("Choose a project file first.");
  const resolved = path.resolve(candidate);
  if (!path.extname(path.basename(resolved))) return `${resolved}.qstudio.json`;
  return resolveProjectFilePath(resolved);
}

export function validateProjectFileContents(contents) {
  if (typeof contents !== "string") throw new Error("Project data must be text.");
  const byteLength = Buffer.byteLength(contents, "utf8");
  if (!byteLength) throw new Error("Project file is empty.");
  if (byteLength > PROJECT_FILE_MAX_BYTES) {
    throw new Error("Project file is too large to open safely.");
  }

  let project;
  try {
    project = JSON.parse(contents);
  } catch {
    throw new Error("Project file is not valid JSON.");
  }
  if (!project || typeof project !== "object" || Array.isArray(project)) {
    throw new Error("Project file must contain a JSON object.");
  }
  if (project.type !== PROJECT_FILE_TYPE) {
    throw new Error("This file is not a Domino project bundle.");
  }
  return { contents, byteLength };
}

export async function readProjectFile(filePath) {
  const target = resolveProjectFilePath(filePath);
  const stats = await lstat(target);
  if (!stats.isFile()) throw new Error("Selected project path is not a regular file.");
  if (stats.size > PROJECT_FILE_MAX_BYTES) {
    throw new Error("Project file is too large to open safely.");
  }
  const contents = await readFile(target, "utf8");
  const validated = validateProjectFileContents(contents);
  return {
    ...validated,
    fileName: path.basename(target),
  };
}

export async function writeProjectFile(filePath, contents) {
  const target = resolveProjectSaveFilePath(filePath);
  const validated = validateProjectFileContents(contents);
  const directory = path.dirname(target);
  const directoryStats = await lstat(directory);
  if (!directoryStats.isDirectory()) throw new Error("Project destination folder is unavailable.");

  const temporary = path.join(directory, `.${path.basename(target)}.${randomUUID()}.tmp`);
  try {
    await writeFile(temporary, validated.contents, { encoding: "utf8", flag: "wx" });
    await rename(temporary, target);
  } catch (error) {
    await rm(temporary, { force: true }).catch(() => {});
    throw error;
  }

  return {
    fileName: path.basename(target),
    byteLength: validated.byteLength,
  };
}
