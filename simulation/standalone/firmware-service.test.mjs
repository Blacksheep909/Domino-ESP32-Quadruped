import assert from "node:assert/strict";
import { mkdtempSync, mkdirSync, readFileSync, realpathSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";
import test from "node:test";
import { fileURLToPath } from "node:url";

import {
  collectFirmwarePackage,
  preparePlatformioEnvironment,
  resolveReviewFile,
} from "./firmware-service.mjs";

function fixture() {
  const root = mkdtempSync(path.join(tmpdir(), "domino-firmware-"));
  mkdirSync(path.join(root, "src"));
  mkdirSync(path.join(root, "private"));
  writeFileSync(path.join(root, "platformio.ini"), "[env:esp32dev]\nboard = esp32dev\n");
  writeFileSync(path.join(root, "src", "main.cpp"), "void setup() {}\nvoid loop() {}\n");
  writeFileSync(path.join(root, "private", "secret.txt"), "not deployable\n");
  return root;
}

test("firmware package includes only approved project roots", () => {
  const root = fixture();
  const firmwarePackage = collectFirmwarePackage(root);
  assert.deepEqual(firmwarePackage.files.map((file) => file.path), ["platformio.ini", "src/main.cpp"]);
  assert.equal(resolveReviewFile(root, "private/secret.txt"), null);
  assert.match(resolveReviewFile(root, "src/main.cpp").contents, /void setup/);
});

test("firmware package hash changes when deployable source changes", () => {
  const root = fixture();
  const before = collectFirmwarePackage(root).hash;
  writeFileSync(path.join(root, "src", "main.cpp"), "void setup() {}\nvoid loop() { delay(1); }\n");
  const after = collectFirmwarePackage(root).hash;
  assert.notEqual(after, before);
});

test("firmware jobs keep writable PlatformIO state in the app runtime", () => {
  const root = mkdtempSync(path.join(tmpdir(), "domino-platformio-"));
  const installedCore = path.join(root, "installed");
  const runtimeRoot = path.join(root, "runtime");
  mkdirSync(path.join(installedCore, "platforms"), { recursive: true });
  mkdirSync(path.join(installedCore, "packages"), { recursive: true });

  const environment = preparePlatformioEnvironment(runtimeRoot, installedCore);
  assert.equal(environment.PLATFORMIO_CORE_DIR, path.join(runtimeRoot, "platformio-core"));
  assert.equal(environment.PLATFORMIO_SETTING_ENABLE_TELEMETRY, "No");
  assert.equal(
    realpathSync(path.join(environment.PLATFORMIO_CORE_DIR, "platforms")),
    realpathSync(path.join(installedCore, "platforms")),
  );
  assert.equal(
    realpathSync(path.join(environment.PLATFORMIO_CORE_DIR, "packages")),
    realpathSync(path.join(installedCore, "packages")),
  );
});

test("physical CRSF control does not require a LIVE browser arm", () => {
  const standaloneRoot = path.dirname(fileURLToPath(import.meta.url));
  const repoRoot = path.resolve(standaloneRoot, "..", "..");
  const mainSource = readFileSync(path.join(repoRoot, "src", "main.cpp"), "utf8");
  const endpointSource = readFileSync(path.join(repoRoot, "src", "live_robot_endpoint.cpp"), "utf8");

  assert.match(mainSource, /liveRobotEndpointAllowsRadioControl\(\)/);
  assert.match(mainSource, /if \(!servoOutputsEnabled\(\)\) setServoOutputsEnabled\(pca, true\)/);
  assert.doesNotMatch(
    mainSource,
    /servoOutputsEnabled\(\)\s*&&\s*!liveRobotEndpointAllowsLocomotion\(\)/,
  );
  assert.match(endpointSource, /bool radioControlEnabled = true;/);
  assert.match(endpointSource, /radioControlEnabled = false;\s*\n\s*benchMode = true;/);
});

test("physical telemetry and profile commands use bounded UART rings", () => {
  const standaloneRoot = path.dirname(fileURLToPath(import.meta.url));
  const repoRoot = path.resolve(standaloneRoot, "..", "..");
  const mainSource = readFileSync(path.join(repoRoot, "src", "main.cpp"), "utf8");
  const endpointSource = readFileSync(path.join(repoRoot, "src", "live_robot_endpoint.cpp"), "utf8");

  assert.match(
    mainSource,
    /#ifndef DOMINO_SIL\s*\n\s*Serial\.setTxBufferSize\(4096\);\s*\n\s*Serial\.setRxBufferSize\(4096\);\s*\n\s*#endif/,
  );
  assert.match(mainSource, /Serial\.begin\(460800\);/);
  assert.match(endpointSource, /static char output\[kDocumentBufferBytes\]/);
  assert.match(endpointSource, /Serial\.write\([^;]+length \+ 1\);/s);
  assert.doesNotMatch(endpointSource, /serializeJson\(document, Serial\)/);
});

test("firmware keeps all identically oriented hip servos in the same direction", () => {
  const standaloneRoot = path.dirname(fileURLToPath(import.meta.url));
  const repoRoot = path.resolve(standaloneRoot, "..", "..");
  const controllerSource = readFileSync(path.join(repoRoot, "src", "leg_controller.cpp"), "utf8");
  const calibrationSource = readFileSync(path.join(repoRoot, "src", "servo_calibration.cpp"), "utf8");

  for (const leg of ["FR", "BR", "FL", "BL"]) {
    assert.match(controllerSource, new RegExp(String.raw`const LegConfig ${leg}_leg\{[\s\S]*?\+1,\s*// hipDir`));
  }
  const directions = calibrationSource
    .match(/constexpr int8_t kDefaultDirection[^=]*=\s*\{([^}]+)\}/s)?.[1]
    .match(/-?\d+/g)
    .map(Number) || [];
  for (const channel of [0, 3, 9, 14]) {
    assert.equal(directions[channel], 1, `hip channel ${channel} must use normal direction`);
  }
  assert.match(controllerSource, /constexpr float kMaxHipMechanicalDeltaDeg = 30\.0f;/);
  assert.match(calibrationSource, /if \(hipChannel\(logicalChannel\)\)[\s\S]*?logicalDelta < -30\.0f[\s\S]*?logicalDelta > 30\.0f/);
  assert.match(calibrationSource, /hipChannel\(channel\) \? 30\.0f : 45\.0f/);
});

test("firmware roll and pitch preserve stance position and change leg height instead", () => {
  const standaloneRoot = path.dirname(fileURLToPath(import.meta.url));
  const repoRoot = path.resolve(standaloneRoot, "..", "..");
  const mainSource = readFileSync(path.join(repoRoot, "src", "main.cpp"), "utf8");

  assert.match(
    mainSource,
    /bodyKinematicsSimple\(static_cast<LegIndex>\(i\),[\s\S]*?bodyZ,\s*0\.0f,\s*0\.0f,\s*yawDeg/,
  );
  assert.match(
    mainSource,
    /zLeg \+= side \* sinf\(rollDeg \* kDegToRad\) \* FOOT_OUT_OFFSET_Y;/,
  );
  assert.match(
    mainSource,
    /zLeg \+= foreAft \* sinf\(pitchDeg \* kDegToRad\) \* BODY_HALF_LENGTH_X;/,
  );
});

test("Domino PCB voltage telemetry uses the routed GPIO36 divider", () => {
  const standaloneRoot = path.dirname(fileURLToPath(import.meta.url));
  const repoRoot = path.resolve(standaloneRoot, "..", "..");
  const projectConfig = readFileSync(path.join(repoRoot, "platformio.ini"), "utf8");
  const monitorSource = readFileSync(path.join(repoRoot, "src", "power_monitor.cpp"), "utf8");
  const endpointSource = readFileSync(path.join(repoRoot, "src", "live_robot_endpoint.cpp"), "utf8");

  assert.match(projectConfig, /DOMINO_VOLTAGE_ADC_PIN=36/);
  assert.match(projectConfig, /DOMINO_VOLTAGE_DIVIDER_RATIO_MILLI=5000/);
  assert.match(monitorSource, /analogReadMilliVolts\(DOMINO_VOLTAGE_ADC_PIN\)/);
  assert.match(endpointSource, /if \(power\.voltageValid\) powerJson\["voltageV"\]/);
});
