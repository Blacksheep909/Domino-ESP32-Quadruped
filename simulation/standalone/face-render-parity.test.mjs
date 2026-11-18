import assert from "node:assert/strict";
import { existsSync, readFileSync, mkdirSync } from "node:fs";
import { spawnSync } from "node:child_process";
import os from "node:os";
import path from "node:path";
import test from "node:test";
import { fileURLToPath } from "node:url";

import { createFaceEngine, parseFaceEmoteTracks, sanitizeFaceSettings, validFaceSettings } from "./web/src/face-engine.js";
import { createFaceRenderer, parseFaceFont } from "./web/src/face-renderer.js";

const here = path.dirname(fileURLToPath(import.meta.url));
const repo = path.resolve(here, "..", "..");
const font = parseFaceFont(readFileSync(path.join(repo, "src", "face_font.h"), "utf8"));
const tracks = parseFaceEmoteTracks(readFileSync(path.join(repo, "src", "face_emotes.h"), "utf8"));
const renderer = createFaceRenderer(font);
const compiler = path.join(os.homedir(), ".platformio", "packages", "toolchain-gccmingw32", "bin", "g++.exe");

test("Studio's face renderer draws the same bytes as the firmware renderer", { skip: !existsSync(compiler) && "native g++ not installed" }, () => {
  const out = path.join(repo, "simulation", "sil", "bin");
  mkdirSync(out, { recursive: true });
  const exe = path.join(out, "face_render_dump_test.exe");
  const build = spawnSync(compiler, ["-std=c++11", "-O1", "-static", `-I${path.join(repo, "src")}`,
    path.join(repo, "src", "face_renderer.cpp"), path.join(repo, "simulation", "sil", "src", "face_render_dump.cpp"),
    "-o", exe], { env: { ...process.env, PATH: `${path.dirname(compiler)};${process.env.PATH}` }, encoding: "utf8" });
  assert.equal(build.status, 0, build.stderr);
  const run = spawnSync(exe, { encoding: "utf8", maxBuffer: 16 * 1024 * 1024 });
  assert.equal(run.status, 0, run.stderr);
  const frames = JSON.parse(run.stdout);
  assert.ok(frames.length >= 10);
  const layers = ["blank", "eye", "battery", "text"];
  for (const f of frames) {
    const buf = renderer.render({ layer: layers[f.layer], eye: f.eye, battery: f.battery, text: f.text });
    assert.equal(Buffer.from(buf).toString("hex"), f.hex, f.name);
  }
});

test("font and emote eye tracks parse from the firmware headers", () => {
  assert.equal(font.length, 64);
  assert.deepEqual(tracks.map((t) => t.name), ["kFaceCurious", "kFaceHappy", "kFaceBow", "kFaceLook", "kFaceShy"]);
});

test("eye tracks last exactly as long as the personality emotes", () => {
  const cpp = readFileSync(path.join(repo, "src", "personality.cpp"), "utf8");
  const names = ["kCurious", "kHappy", "kBow", "kLook", "kShy"];
  names.forEach((name, i) => {
    const body = cpp.match(new RegExp(String.raw`constexpr Key ${name}\[\] = \{([\s\S]*?)\};`))[1];
    const last = [...body.matchAll(/\{\s*(\d+),/g)].map((m) => Number(m[1])).at(-1);
    assert.equal(tracks[i].keys.at(-1)[0], last, name);
  });
});

test("face engine port: sleeps when lying, flashes low battery on one eye, reminds after idle", () => {
  const engine = createFaceEngine(tracks, 5);
  let out;
  const run = (inputs, until) => { while (inputs.nowMs < until) { inputs.nowMs += 20; out = engine.step(inputs); } };
  const inputs = { nowMs: 0, lying: false, sticksIdle: true, batteryPercent: 64, emote: -1 };
  run(inputs, 1000);
  assert.ok(out[0].eye.h >= 30);
  inputs.lying = true;
  run(inputs, 4000);
  assert.ok(out[0].eye.h <= 5 && out[1].eye.h <= 5);
  inputs.lying = false;
  run(inputs, 16500);
  assert.equal(out[1].layer, "battery");
  assert.equal(out[1].battery.percent, 64);
  run(inputs, 22000);
  assert.equal(out[1].layer, "eye");

  const low = createFaceEngine(tracks, 7);
  const li = { nowMs: 0, sticksIdle: true, batteryPercent: 12, emote: -1 };
  let battery = 0;
  for (let i = 0; i < 300; i += 1) {
    li.nowMs += 20;
    const o = low.step(li);
    if (o[1].layer === "battery") battery += 1;
    assert.notEqual(o[0].layer, "battery");
  }
  assert.ok(battery > 100 && battery < 200);
});

test("face settings sanitise and validate", () => {
  assert.equal(validFaceSettings(sanitizeFaceSettings({})), true);
  const s = sanitizeFaceSettings({ mode: "loud", brightness: 400, batteryEye: "left", reminderMinutes: -3 });
  assert.deepEqual([s.mode, s.brightness, s.batteryEye, s.reminderMinutes], ["eyes", 100, "left", 0]);
  assert.equal(validFaceSettings({ ...s, glow: "yes" }), false);
});

test("eye style sizes match the firmware table", async () => {
  const { FACE_STYLES } = await import("./web/src/face-engine.js");
  const cpp = readFileSync(path.join(repo, "src", "face_engine.cpp"), "utf8");
  const firmware = [...cpp.matchAll(/\{"(\w+)", (\d+), (\d+), (\d+)\}/g)].map((m) => [m[1], +m[2], +m[3], +m[4]]);
  assert.deepEqual(FACE_STYLES.map((s) => [s.id, s.w, s.h, s.r]), firmware);
});

test("ear spring constants match the firmware and the ears overshoot then settle", async () => {
  const { EAR, earTargets, createEarSpring, stepEarSpring } = await import("./web/src/ear-dynamics.js");
  const h = readFileSync(path.join(repo, "src", "personality.h"), "utf8");
  const k = (name) => Number(h.match(new RegExp(String.raw`constexpr float ${name} = (-?[\d.]+)f;`))[1]);
  assert.deepEqual(
    [EAR.tiltBaseDeg, EAR.pitchGain, EAR.rollGain, EAR.yawGain, EAR.lyingDeg, EAR.springHz, EAR.damping],
    [k("kEarTiltBaseDeg"), k("kEarPitchGain"), k("kEarRollGain"), k("kEarYawGain"), k("kEarLyingDeg"), k("kEarSpringHz"), k("kEarDamping")],
  );
  const [l, r] = earTargets({ tilt: true, rollDeg: 10, pitchDeg: 5 });
  assert.ok(l > r, "roll lifts one ear and drops the other");
  const ear = createEarSpring(-45);
  let peak = -45;
  for (let i = 0; i < 100; i += 1) { stepEarSpring(ear, -15, 0.01); peak = Math.max(peak, ear.angleDeg); }
  assert.ok(peak > -15 + 2, "overshoots the target like a floppy ear");
  for (let i = 0; i < 300; i += 1) stepEarSpring(ear, -15, 0.01);
  assert.ok(Math.abs(ear.angleDeg + 15) < 0.5, "then settles");
});
