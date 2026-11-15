// One source, three outputs, from droid_presets.json:
//   sounds/wav/<version>/NN_<name>.wav   numbered files to double-click and give feedback on
//   sounds/preview.html            a page with a play button per emote
//   src/droid_voice_presets.h      the table the firmware player (src/droid_voice.cpp) plays
// Usage: node sounds/build.mjs
import { mkdirSync, readFileSync, readdirSync, rmSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { WAVES, hopRandom, normaliseFx, normaliseSegment, renderEmote } from "./droid-synth.mjs";

const here = dirname(fileURLToPath(import.meta.url));
const presets = JSON.parse(readFileSync(join(here, "droid_presets.json"), "utf8"));
const rate = presets.sampleRate;

// ---- long sequences: expand phrases and babble into ordinary emotes ----
const shifted = (segs, pitch = 1, tempo = 1) => segs.map((s) => ({
  ...s,
  f0: s.f0 * pitch,
  f1: (s.f1 ?? s.f0) * pitch,
  ms: s.ms / tempo,
  gap: (s.gap ?? 0) / tempo,
  step: s.step === undefined ? undefined : s.step / tempo,
}));
for (const [name, items] of Object.entries(presets.phrases ?? {})) {
  const out = [];
  for (const item of items) {
    if (typeof item === "number") {
      if (out.length) out[out.length - 1] = { ...out.at(-1), gap: (out.at(-1).gap ?? 0) + item };
      continue;
    }
    const ref = typeof item === "string" ? { emote: item } : item;
    if (!presets.emotes[ref.emote]) throw new Error(`phrase ${name}: unknown emote ${ref.emote}`);
    out.push(...shifted(presets.emotes[ref.emote], ref.pitch, ref.tempo));
  }
  presets.emotes[name] = out;
}
// A seeded sentence that mixes spoken-sounding words with whistles. "notes" counts words.
for (const [name, { seed, notes }] of Object.entries(presets.babble ?? {})) {
  let n = 0;
  const rnd = () => hopRandom(seed, n++);
  const pick = (lo, hi) => lo + (hi - lo) * rnd();
  const vowels = ["oo", "oh", "ah", "uh", "eh", "ee"];
  const vowel = () => vowels[Math.floor(rnd() * vowels.length)];
  const out = [];
  let base = pick(600, 800);
  for (let i = 0; i < notes; i++) {
    if (rnd() < 0.35) {
      // a whistle-shaped word
      const lo = pick(1200, 1700);
      const top = lo * pick(1.5, 1.9);
      out.push({ wave: "sine", f0: lo, f1: top, ms: pick(110, 180), gap: 0, curve: "exp", sub: 0.2 });
      out.push({ wave: "sine", f0: top, f1: lo * pick(1.1, 1.4), ms: pick(140, 220), gap: 0, vib: 9, vibDepth: 0.015, sub: 0.2 });
    } else {
      // a spoken word of one to three syllables, each gliding between two vowels
      base = Math.min(1000, Math.max(520, base * pick(0.8, 1.25)));
      const syllables = 1 + Math.floor(rnd() * 3);
      for (let k = 0; k < syllables; k++) {
        const up = rnd() < 0.5;
        const f0 = base * pick(0.9, 1.1);
        out.push({ wave: "vox", vowel: `${vowel()}>${vowel()}`, f0, f1: f0 * (up ? pick(1.1, 1.45) : pick(0.7, 0.92)),
          ms: pick(110, 210), gap: k < syllables - 1 ? pick(20, 50) : 0, sub: 0.3, subRatio: 2, atk: 10, rel: 25 });
      }
    }
    out[out.length - 1].gap = rnd() < 0.25 ? pick(200, 320) : pick(60, 130);
  }
  out[out.length - 1].gap = 0;
  presets.emotes[name] = out;
}
delete presets.phrases;
delete presets.babble;

function wav(samples) {
  const buf = Buffer.alloc(44 + samples.length * 2);
  buf.write("RIFF", 0);
  buf.writeUInt32LE(36 + samples.length * 2, 4);
  buf.write("WAVEfmt ", 8);
  buf.writeUInt32LE(16, 16);
  buf.writeUInt16LE(1, 20);
  buf.writeUInt16LE(1, 22);
  buf.writeUInt32LE(rate, 24);
  buf.writeUInt32LE(rate * 2, 28);
  buf.writeUInt16LE(2, 32);
  buf.writeUInt16LE(16, 34);
  buf.write("data", 36);
  buf.writeUInt32LE(samples.length * 2, 40);
  samples.forEach((v, i) => buf.writeInt16LE(Math.round(Math.max(-1, Math.min(1, v)) * 0.8 * 32767), 44 + i * 2));
  return buf;
}

const wavDir = join(here, "wav", presets.version ?? "current");
mkdirSync(wavDir, { recursive: true });
for (const f of readdirSync(wavDir)) if (f.endsWith(".wav")) rmSync(join(wavDir, f));
const names = Object.keys(presets.emotes);
names.forEach((name, i) => {
  const samples = renderEmote(presets.emotes[name], rate, { fx: presets.fx });
  const file = `${String(i + 1).padStart(2, "0")}_${name}.wav`;
  writeFileSync(join(wavDir, file), wav(samples));
  console.log(`${file.padEnd(24)} ${(samples.length / rate).toFixed(2)} s`);
});

// ---- firmware table ----
const id = (name) => `DROID_EMOTE_${name.toUpperCase()}`;
const segLine = (seg) => {
  const s = normaliseSegment(seg);
  return `  {${WAVES.indexOf(s.wave)}, ${s.curve === "exp" ? 1 : 0}, ${s.f0}, ${s.f1}, ${s.ms}, ${s.gap}, ` +
    `${Math.round(s.amp * 255)}, ${s.vib}, ${Math.round(s.vibDepth * 1000)}, ${s.step}, ${s.seed}u, ` +
    `${Math.round(s.fmRatio * 100)}, ${Math.round(s.fmIndex * 100)}, ${Math.round(s.sub * 255)}, ${s.atk}, ${s.rel}, ${Math.round(s.subRatio * 100)}, ${s.fa0}, ${s.fb0}, ${s.fa1}, ${s.fb1}},`;
};
const fx = Object.fromEntries(Object.entries(normaliseFx(presets.fx ?? {})).map(([k, v]) => [k, Number.isInteger(v) ? v : v.toFixed(2)]));
const header = `// Generated by sounds/build.mjs from sounds/droid_presets.json. Do not edit by hand:
// change the JSON, run "node sounds/build.mjs", and listen in sounds/preview.html first.
#pragma once

#include <stdint.h>

constexpr uint32_t kDroidSampleRate = ${rate};

// Effects chain ("fx" in the presets file): soft distortion, two echoes, treble roll-off.
constexpr float kDroidFxDrive = ${fx.drive}f;
constexpr uint16_t kDroidFxEchoMs = ${fx.echoMs};
constexpr float kDroidFxEchoGain = ${fx.echoGain}f;
constexpr uint16_t kDroidFxEcho2Ms = ${fx.echo2Ms};
constexpr float kDroidFxEcho2Gain = ${fx.echo2Gain}f;
constexpr uint16_t kDroidFxLowpassHz = ${fx.lowpassHz};

enum DroidWave : uint8_t { ${WAVES.map((w) => `DROID_WAVE_${w.toUpperCase()}`).join(", ")} };

struct DroidSegment {
  uint8_t wave;
  uint8_t expCurve;
  uint16_t f0;
  uint16_t f1;
  uint16_t ms;
  uint16_t gapMs;
  uint8_t amp;               // 0..255
  uint8_t vibHz;
  uint16_t vibDepthPermille;
  uint16_t stepMs;           // hop interval for DROID_WAVE_HOP
  uint32_t seed;
  uint16_t fmRatioCenti;
  uint16_t fmIndexCenti;
  uint8_t sub;               // 0..255 mix of the octave-down body voice
  uint8_t attackMs;
  uint8_t releaseMs;
  uint16_t subRatioCenti;    // second voice pitch as a ratio of the main voice (50 = octave down)
  uint16_t formantA0;        // DROID_WAVE_VOX: the two vowel resonances, start -> end of the segment
  uint16_t formantB0;
  uint16_t formantA1;
  uint16_t formantB1;
};

struct DroidEmoteDef {
  const char *name;
  const DroidSegment *segments;
  uint8_t count;
};

enum DroidEmote : uint8_t {
${names.map((n) => `  ${id(n)},`).join("\n")}
  DROID_EMOTE_COUNT
};

${names.map((n) => `static const DroidSegment kDroidSeg_${n}[] = {\n${presets.emotes[n].map(segLine).join("\n")}\n};`).join("\n\n")}

static const DroidEmoteDef kDroidEmotes[DROID_EMOTE_COUNT] = {
${names.map((n) => `  {"${n}", kDroidSeg_${n}, ${presets.emotes[n].length}},`).join("\n")}
};
`;
writeFileSync(join(here, "..", "src", "droid_voice_presets.h"), header);
console.log("wrote src/droid_voice_presets.h");

const synth = readFileSync(join(here, "droid-synth.mjs"), "utf8").replace(/^export /gm, "");
const html = `<!doctype html>
<meta charset="utf-8">
<title>Domino droid voice</title>
<style>
  body { font: 15px system-ui, sans-serif; margin: 24px; background: #14161a; color: #e8e8e8; }
  button { font: inherit; padding: 10px 14px; margin: 4px; border: 1px solid #444; border-radius: 8px; background: #23262d; color: inherit; cursor: pointer; }
  button:hover { background: #2f343d; }
  label { display: inline-block; margin: 0 18px 14px 0; }
</style>
<h2>Domino droid voice</h2>
<label>Volume <input id="vol" type="range" min="0" max="1" step="0.01" value="0.5"></label>
<label>Pitch <input id="pitch" type="range" min="0.6" max="1.6" step="0.01" value="1"> <span id="pv">1.00</span>x</label>
<label><input id="fx" type="checkbox" checked> Effects</label>
<label>Tempo <input id="tempo" type="range" min="0.6" max="1.6" step="0.01" value="1"> <span id="tv">1.00</span>x</label>
<div id="emotes"></div>
<script>
${synth}
const PRESETS = ${JSON.stringify(presets)};
let ctx;
const $ = (id) => document.getElementById(id);
function play(name) {
  ctx ??= new AudioContext();
  const s = renderEmote(PRESETS.emotes[name], PRESETS.sampleRate, { pitch: +$("pitch").value, tempo: +$("tempo").value, fx: $("fx").checked ? PRESETS.fx : null });
  const b = ctx.createBuffer(1, s.length, PRESETS.sampleRate);
  b.copyToChannel(s, 0);
  const src = ctx.createBufferSource();
  const gain = ctx.createGain();
  gain.gain.value = +$("vol").value;
  src.buffer = b;
  src.connect(gain).connect(ctx.destination);
  src.start();
}
for (const name of Object.keys(PRESETS.emotes)) {
  const b = document.createElement("button");
  b.textContent = name;
  b.onclick = () => play(name);
  $("emotes").append(b);
}
$("pitch").oninput = (e) => ($("pv").textContent = (+e.target.value).toFixed(2));
$("tempo").oninput = (e) => ($("tv").textContent = (+e.target.value).toFixed(2));
</script>
`;
writeFileSync(join(here, "preview.html"), html);
console.log("wrote preview.html");
