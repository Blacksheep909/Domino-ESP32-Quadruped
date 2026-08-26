import { readFileSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const standaloneRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const repoRoot = path.resolve(standaloneRoot, "../..");
const packagePath = path.join(standaloneRoot, "package.json");
const firmwareSourcePath = path.join(repoRoot, "src", "live_robot_endpoint.cpp");

function parseVersion(version) {
  const match = String(version).match(/^(\d+)\.(\d+)\.(\d+)$/);
  if (!match) throw new Error(`Expected a semantic version such as 0.2.1, received: ${version}`);
  return match.slice(1).map(Number);
}

function nextPatch(version) {
  const [major, minor, patch] = parseVersion(version);
  return `${major}.${minor}.${patch + 1}`;
}

function firmwareVersion() {
  const source = readFileSync(firmwareSourcePath, "utf8");
  return source.match(/document\["firmwareVersion"\]\s*=\s*"([^"]+)"/)?.[1] || "unknown";
}

const requestedVersion = process.argv[2];
const packageJson = JSON.parse(readFileSync(packagePath, "utf8"));
const nextVersion = requestedVersion ? parseVersion(requestedVersion).join(".") : nextPatch(packageJson.version);
packageJson.version = nextVersion;
writeFileSync(packagePath, `${JSON.stringify(packageJson, null, 2)}\n`, "utf8");

console.log(`Prepared Domino Quadruped Studio ${nextVersion} for firmware ${firmwareVersion()}.`);
console.log(`Commit the version change, tag it as v${nextVersion}, and push that tag to publish an updater release.`);
