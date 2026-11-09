import test from "node:test";
import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

// Studio 0.5.5 shipped without policy-lab-service.mjs: server.mjs imported it,
// but it was not in package.json "build.files", so the installed app could not
// start. Every local module the desktop app loads must be packaged.
const root = path.dirname(fileURLToPath(import.meta.url));
const files = JSON.parse(readFileSync(path.join(root, "package.json"), "utf8")).build.files;

function packaged(relative) {
  return files.some((pattern) => pattern === relative ||
    (pattern.endsWith("/**/*") && relative.startsWith(pattern.slice(0, -"**/*".length))));
}

function localImports(relative) {
  const source = readFileSync(path.join(root, relative), "utf8");
  const pattern = /(?:import\s[^"']*?from\s*|import\s*\(\s*|import\s+)["'](\.{1,2}\/[^"']+)["']/g;
  return [...source.matchAll(pattern)]
    .map((match) => path.posix.normalize(path.posix.join(path.posix.dirname(relative), match[1])))
    .filter((target) => existsSync(path.join(root, target)));
}

test("every local module the desktop app imports is packaged", () => {
  const seen = new Set();
  const missing = [];
  const queue = ["desktop/main.mjs", "server.mjs"];
  while (queue.length) {
    const current = queue.shift();
    if (seen.has(current)) continue;
    seen.add(current);
    for (const target of localImports(current)) {
      if (!packaged(target)) missing.push(`${target} (imported by ${current})`);
      if (/\.m?js$/.test(target)) queue.push(target);
    }
  }
  assert.ok(seen.size > 5, "walked the import graph");
  assert.deepEqual(missing, []);
});
