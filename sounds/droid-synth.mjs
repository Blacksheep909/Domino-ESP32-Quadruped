// Domino droid voice synth. The firmware (src/droid_voice.cpp) runs the same
// maths, so what you hear in the preview page is what the robot plays.
// Keep this file free of imports: build.mjs inlines it into preview.html.

export const WAVES = ["sine", "tri", "sq", "fm", "hop", "vox"];

// Vowel shapes for the "vox" wave: the two resonances (Hz) that make a mouth
// sound like that vowel, pitched up a little for a small creature.
export const VOWELS = {
  oo: [380, 950],
  oh: [560, 1050],
  ah: [900, 1500],
  uh: [650, 1300],
  eh: [650, 2100],
  ee: [380, 2700],
  mm: [300, 1100],
};

export function hopRandom(seed, index) {
  let x = (seed + Math.imul(index, 2654435761)) >>> 0;
  x ^= x >>> 15;
  x = Math.imul(x, 2246822519) >>> 0;
  x ^= x >>> 13;
  return (x >>> 0) / 4294967296;
}

// "oo>ah" glides from one vowel to another across the segment; "ah" holds one.
function formants(vowel) {
  const [a, b] = String(vowel ?? "ah").split(">");
  const from = VOWELS[a] ?? VOWELS.ah;
  const to = VOWELS[b ?? a] ?? from;
  return { fa0: from[0], fb0: from[1], fa1: to[0], fb1: to[1] };
}

// Fill in defaults and round to what the firmware table can store.
export function normaliseSegment(seg) {
  return {
    wave: WAVES.includes(seg.wave) ? seg.wave : "sine",
    f0: Math.round(seg.f0),
    f1: Math.round(seg.f1 ?? seg.f0),
    ms: Math.round(seg.ms),
    gap: Math.round(seg.gap ?? 0),
    curve: seg.curve === "exp" ? "exp" : "lin",
    amp: Math.round(Math.min(1, Math.max(0, seg.amp ?? 1)) * 255) / 255,
    vib: Math.round(seg.vib ?? 0),
    vibDepth: Math.round((seg.vibDepth ?? 0) * 1000) / 1000,
    step: Math.round(seg.step ?? 40),
    seed: (seg.seed ?? 1) >>> 0,
    fmRatio: Math.round((seg.fmRatio ?? 0.5) * 100) / 100,
    fmIndex: Math.round((seg.fmIndex ?? 1) * 100) / 100,
    sub: Math.round(Math.min(1, Math.max(0, seg.sub ?? 0)) * 255) / 255,
    subRatio: Math.round((seg.subRatio ?? 0.5) * 100) / 100,
    atk: Math.min(255, Math.max(1, Math.round(seg.atk ?? 4))),
    rel: Math.min(255, Math.max(1, Math.round(seg.rel ?? 8))),
    ...formants(seg.vowel),
  };
}

// Fill in defaults for the effects chain. Returns null when there is none.
export function normaliseFx(fx) {
  if (!fx) return null;
  return {
    drive: Math.round((fx.drive ?? 1) * 100) / 100,
    echoMs: Math.round(fx.echoMs ?? 0),
    echoGain: Math.round((fx.echoGain ?? 0) * 100) / 100,
    echo2Ms: Math.round(fx.echo2Ms ?? 0),
    echo2Gain: Math.round((fx.echo2Gain ?? 0) * 100) / 100,
    lowpassHz: Math.round(fx.lowpassHz ?? 0),
  };
}

// Soft clipper (a cheap tanh). The firmware uses the same formula.
function saturate(x) {
  if (x > 3) return 1;
  if (x < -3) return -1;
  return (x * (27 + x * x)) / (27 + 9 * x * x);
}

// The "post" pass: a little distortion, two short echoes (a small metal body),
// and a gentle treble roll-off. Adds a tail so the echoes can ring out.
export function applyFx(dry, sampleRate, fxIn) {
  const fx = normaliseFx(fxIn);
  if (!fx) return dry;
  const d1 = Math.round((fx.echoMs * sampleRate) / 1000);
  const d2 = Math.round((fx.echo2Ms * sampleRate) / 1000);
  const tail = Math.max(d1, d2) + Math.round(0.04 * sampleRate);
  const out = new Float32Array(dry.length + tail);
  const sat = new Float32Array(out.length);
  const norm = 1 / saturate(fx.drive);
  const mix = 1 / (1 + fx.echoGain + fx.echo2Gain);
  const lp = fx.lowpassHz > 0 ? 1 - Math.exp((-2 * Math.PI * fx.lowpassHz) / sampleRate) : 1;
  let y = 0;
  for (let i = 0; i < out.length; i++) {
    sat[i] = i < dry.length ? saturate(dry[i] * fx.drive) * norm : 0;
    let v = sat[i];
    if (d1 > 0 && i >= d1) v += fx.echoGain * sat[i - d1];
    if (d2 > 0 && i >= d2) v += fx.echo2Gain * sat[i - d2];
    y += lp * (v * mix - y);
    out[i] = y;
  }
  return out;
}

// Returns Float32Array samples in -1..1. pitch and tempo are global multipliers
// (pitch 1.2 = 20 % higher, tempo 1.2 = 20 % faster). fx is the optional
// effects chain from the presets file.
export function renderEmote(segments, sampleRate, { pitch = 1, tempo = 1, fx = null } = {}) {
  const segs = segments.map(normaliseSegment);
  const lengths = segs.map((s) => [
    Math.max(1, Math.round((s.ms / tempo) * sampleRate / 1000)),
    Math.round((s.gap / tempo) * sampleRate / 1000),
  ]);
  const total = lengths.reduce((n, [a, b]) => n + a + b, 0);
  const out = new Float32Array(total);
  let pos = 0;
  let phase = 0;
  let subPhase = 0;
  segs.forEach((s, k) => {
    const [n, gap] = lengths[k];
    const f0 = s.f0 * pitch;
    const f1 = s.f1 * pitch;
    const stepSamples = Math.max(1, Math.round((s.step / tempo) * sampleRate / 1000));
    const attack = Math.max(1, Math.round((s.atk * sampleRate) / 1000));
    const release = Math.max(1, Math.round((s.rel * sampleRate) / 1000));
    for (let i = 0; i < n; i++) {
      const u = i / n;
      let f;
      if (s.wave === "hop") f = f0 + (f1 - f0) * hopRandom(s.seed, Math.floor(i / stepSamples));
      else if (s.curve === "exp") f = f0 * Math.pow(f1 / f0, u);
      else f = f0 + (f1 - f0) * u;
      if (s.vib > 0) f *= 1 + s.vibDepth * Math.sin((2 * Math.PI * s.vib * i) / sampleRate);
      phase += f / sampleRate;
      phase -= Math.floor(phase);
      let v;
      if (s.wave === "tri") v = 4 * Math.abs(phase - 0.5) - 1;
      else if (s.wave === "sq") v = phase < 0.5 ? 0.6 : -0.6;
      else if (s.wave === "fm") v = Math.sin(2 * Math.PI * phase + s.fmIndex * Math.sin(2 * Math.PI * phase * s.fmRatio));
      else if (s.wave === "vox") {
        // A voice: once per pitch period, a short burst that rings at the two vowel resonances.
        const fa = s.fa0 + (s.fa1 - s.fa0) * u;
        const fb = s.fb0 + (s.fb1 - s.fb0) * u;
        const window = 0.5 - 0.5 * Math.cos(2 * Math.PI * phase);
        v = 2.0 * window * (0.65 * Math.sin((2 * Math.PI * phase * fa) / f) + 0.35 * Math.sin((2 * Math.PI * phase * fb) / f));
      } else v = Math.sin(2 * Math.PI * phase);
      // second voice: a sine at subRatio times the pitch, mixed in and renormalised
      subPhase += (f * s.subRatio) / sampleRate;
      subPhase -= Math.floor(subPhase);
      if (s.sub > 0) v = (v + s.sub * Math.sin(2 * Math.PI * subPhase)) / (1 + s.sub);
      const env = Math.min(1, (i + 1) / attack, (n - i) / release);
      out[pos++] = v * env * s.amp;
    }
    pos += gap;
  });
  return applyFx(out, sampleRate, fx);
}
