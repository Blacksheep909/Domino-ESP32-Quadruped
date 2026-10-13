// Runs every *.test.mjs under this package. The old `npm test` script listed
// files by hand, so new tests were silently skipped (three never ran).
import { readdirSync } from "node:fs";
import { spawnSync } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";

const root = path.dirname(fileURLToPath(import.meta.url));
const skipped = new Set([
  "node_modules", "out", "dist", "runtime", "tmp", ".pnpm-store", ".git",
]);

function collect(directory) {
  return readdirSync(directory, { withFileTypes: true }).flatMap((entry) => {
    const full = path.join(directory, entry.name);
    if (entry.isDirectory()) {
      const relative = path.relative(root, full).split(path.sep).join("/");
      return skipped.has(entry.name) || relative === "desktop/resources" ? [] : collect(full);
    }
    return entry.name.endsWith(".test.mjs") ? [path.relative(root, full)] : [];
  });
}

const files = collect(root).sort();
if (files.length === 0) {
  console.error("No test files found.");
  process.exit(1);
}
console.log(`Running ${files.length} test files.`);
const result = spawnSync(process.execPath, ["--test", ...files], { cwd: root, stdio: "inherit" });
process.exit(result.status ?? 1);
