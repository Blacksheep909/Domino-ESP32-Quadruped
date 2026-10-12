import { cpSync, existsSync, mkdirSync, readFileSync, realpathSync, rmSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const desktopRoot = path.dirname(fileURLToPath(import.meta.url));
const standaloneRoot = path.resolve(desktopRoot, "..");
const repoRoot = path.resolve(standaloneRoot, "../..");
const outputRoot = path.resolve(desktopRoot, "resources", "domino");
const expectedOutputRoot = path.resolve(standaloneRoot, "desktop", "resources", "domino");

if (outputRoot !== expectedOutputRoot) throw new Error(`Refusing to replace unexpected path: ${outputRoot}`);

const silSource = path.join(repoRoot, "simulation", "sil", "bin", "domino_sil.exe");
const firmwareVersionSource = path.join(repoRoot, "src", "live_robot_endpoint.cpp");
const cadSource = path.join(
  repoRoot,
  "simulation",
  "urdf",
  "generated",
  "Domino_URDF_Parts_Combined_Final_description",
  "meshes",
);
for (const requiredPath of [silSource, cadSource, path.join(repoRoot, "platformio.ini")]) {
  if (!existsSync(requiredPath)) throw new Error(`Cannot prepare desktop resources; missing ${requiredPath}`);
}

const firmwareVersion = readFileSync(firmwareVersionSource, "utf8")
  .match(/document\["firmwareVersion"\]\s*=\s*"([^"]+)"/)?.[1] || "unknown";

rmSync(outputRoot, { recursive: true, force: true });
mkdirSync(path.join(outputRoot, "sil"), { recursive: true });
mkdirSync(path.join(outputRoot, "firmware"), { recursive: true });
mkdirSync(path.join(outputRoot, "companion", "node_modules"), { recursive: true });
cpSync(silSource, path.join(outputRoot, "sil", "domino_sil.exe"));
cpSync(cadSource, path.join(outputRoot, "cad"), { recursive: true });
for (const entry of ["platformio.ini", "src", "include", "lib"]) {
  const source = path.join(repoRoot, entry);
  if (existsSync(source)) cpSync(source, path.join(outputRoot, "firmware", entry), { recursive: true });
}
for (const entry of [
  "live-companion-adapter.mjs",
  "ardupilot-companion-adapter.mjs",
  "live-companion-core.mjs",
  "windows-serial-bridge.ps1",
]) {
  cpSync(path.join(standaloneRoot, entry), path.join(outputRoot, "companion", entry));
}
cpSync(
  path.join(standaloneRoot, "web", "src"),
  path.join(outputRoot, "companion", "web", "src"),
  { recursive: true },
);
cpSync(
  realpathSync(path.join(standaloneRoot, "node_modules", "ws")),
  path.join(outputRoot, "companion", "node_modules", "ws"),
  { recursive: true },
);
writeFileSync(
  path.join(outputRoot, "companion", "package.json"),
  `${JSON.stringify({ private: true, type: "module" }, null, 2)}\n`,
  "utf8",
);
writeFileSync(
  path.join(outputRoot, "manifest.json"),
  `${JSON.stringify({ format: 2, preparedAt: new Date().toISOString(), firmwareVersion }, null, 2)}\n`,
  "utf8",
);

console.log(`Prepared offline desktop resources in ${outputRoot}`);
