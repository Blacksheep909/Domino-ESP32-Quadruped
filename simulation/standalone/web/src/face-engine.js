// Port of src/face_engine.cpp for Studio's SCREENS / EMOTES previews: decides
// what each OLED shows from the robot's state. Emote eye tracks are parsed
// from src/face_emotes.h (parseFaceEmoteTracks) so the preview matches the
// robot. Screens are robot-relative: 0 = left eye, 1 = right eye.

export const FACE_MODES = Object.freeze(["eyes", "telemetry", "off"]);
export const FACE_STYLES = Object.freeze([
  { id: "burne", label: "BURN-E (rounded squares)", w: 48, h: 36, r: 9 },
  { id: "round", label: "Round", w: 40, h: 40, r: 20 },
  { id: "tall", label: "Tall", w: 30, h: 46, r: 12 },
  { id: "wide", label: "Wide", w: 54, h: 28, r: 8 },
]);

export const DEFAULT_FACE_SETTINGS = Object.freeze({
  mode: "eyes",
  style: "burne",
  glow: true,
  brightness: 70,
  batteryEye: "right",
  lowBatteryPercent: 20,
  reminderMinutes: 5,
  blink: true,
  sleepWhenLying: true,
  followSticks: true,
  emoteEyes: true,
});

export function sanitizeFaceSettings(candidate = {}) {
  const m = { ...DEFAULT_FACE_SETTINGS, ...(candidate || {}) };
  const int = (v, lo, hi, d) => {
    const n = Math.round(Number(v));
    return Number.isFinite(n) ? Math.min(hi, Math.max(lo, n)) : d;
  };
  return {
    mode: FACE_MODES.includes(m.mode) ? m.mode : "eyes",
    style: FACE_STYLES.some((s) => s.id === m.style) ? m.style : "burne",
    glow: m.glow !== false,
    brightness: int(m.brightness, 0, 100, 70),
    batteryEye: m.batteryEye === "left" ? "left" : "right",
    lowBatteryPercent: int(m.lowBatteryPercent, 0, 50, 20),
    reminderMinutes: int(m.reminderMinutes, 0, 60, 5),
    blink: m.blink !== false,
    sleepWhenLying: m.sleepWhenLying !== false,
    followSticks: m.followSticks !== false,
    emoteEyes: m.emoteEyes !== false,
  };
}

export function validFaceSettings(s) {
  if (!s || typeof s !== "object") return false;
  const clean = sanitizeFaceSettings(s);
  return Object.keys(clean).every((key) => clean[key] === s[key]);
}

export function parseFaceEmoteTracks(headerText) {
  const block = headerText.split("FACE_EMOTES_BEGIN")[1].split("FACE_EMOTES_END")[0];
  const tracks = [...block.matchAll(/static const FaceEmoteKey (\w+)\[\] = \{([\s\S]*?)\};/g)].map((m) => ({
    name: m[1],
    keys: [...m[2].matchAll(/\{([^{}]+)\}/g)].map((k) => k[1].split(",").map((v) => Number(v.trim()))),
  }));
  if (!tracks.length || tracks.some((t) => t.keys.some((k) => k.length !== 8 || k.some(Number.isNaN)))) {
    throw new Error("face_emotes.h: unexpected eye track table");
  }
  return tracks;
}

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
const approach = (cur, target, rate, dt) => cur + (target - cur) * (1 - Math.exp(-rate * dt));
const smooth = (u) => u * u * (3 - 2 * u);
const lround = (v) => (v < 0 ? -Math.round(-v) : Math.round(v));

function trackSample(tracks, emote, t) {
  const track = tracks[emote];
  if (!track) return null;
  const keys = track.keys;
  if (t >= keys[keys.length - 1][0]) return null;
  let k = 0;
  while (k + 1 < keys.length && keys[k + 1][0] <= t) k += 1;
  const a = keys[k];
  const b = keys[k + 1];
  const s = smooth((t - a[0]) / (b[0] - a[0]));
  const mix = (i) => a[i] + (b[i] - a[i]) * s;
  return [0, 1].map((eye) => ({
    open: mix(eye === 0 ? 1 : 2) / 100,
    lookX: mix(3),
    lookY: mix(4),
    smile: mix(5) / 100,
    mood: mix(6) / 100,
    scale: mix(7) / 100,
  }));
}

const basePose = () => ({ open: 1, lookX: 0, lookY: 0, scale: 1, smile: 0, mood: 0 });

export function createFaceEngine(tracks, seed = 1) {
  let settings = sanitizeFaceSettings();
  let rng = seed >>> 0 || 1;
  const random = () => {
    rng ^= rng << 13; rng >>>= 0;
    rng ^= rng >>> 17;
    rng ^= rng << 5; rng >>>= 0;
    return rng;
  };
  const randomRange = (lo, hi) => lo + (hi - lo) * (random() % 10000) / 10000;
  const st = {
    started: false, lastMs: 0, sleep: 0, nextBlinkMs: 0, blinkStartMs: 0, blinking: false, doubleBlink: false,
    nextGlanceMs: 0, glanceX: 0, glanceY: 0, lookX: 0, lookY: 0, reaction: "none", reactionStartMs: 0,
    idleSinceMs: 0, wasIdle: false, lastReminderMs: 0, reminderShownThisIdle: false, remindedOnce: false,
    batteryShowUntilMs: 0, pose: [basePose(), basePose()],
  };

  function eyeFrame(eye) {
    const p = st.pose[eye];
    const style = FACE_STYLES.find((s) => s.id === settings.style) || FACE_STYLES[0];
    const e = { glow: settings.glow, lidTop: 0, slant: 0, smile: 0 };
    e.w = lround(style.w * p.scale);
    e.h = Math.max(2, lround(style.h * p.scale * p.open));
    e.r = Math.min(Math.trunc(style.r * p.scale), Math.trunc(e.h / 2));
    e.cx = lround(64 + p.lookX);
    e.cy = lround(32 + p.lookY);
    e.smile = lround(e.h * 0.8 * clamp(p.smile, 0, 1));
    const k = lround(e.h * 0.4 * Math.abs(clamp(p.mood, -1, 1)));
    const angry = p.mood > 0;
    const innerLow = eye === 0;
    if (k > 0) {
      if (angry === innerLow) { e.lidTop = k; e.slant = -k; } else { e.lidTop = 0; e.slant = k; }
    }
    return { layer: "eye", eye: e };
  }

  // inputs: { nowMs, lying, sticksIdle, turn, strafe, forward, batteryPercent, emote, emoteElapsedMs,
  //           reaction: "none" | "pushed" | "fallen", telemetry: [lines[], lines[]] }
  function step(inputs) {
    const now = inputs.nowMs;
    if (!st.started) {
      st.started = true;
      st.lastMs = now;
      st.nextBlinkMs = now + 1500;
      st.nextGlanceMs = now + 2000;
      st.idleSinceMs = now;
      st.lastReminderMs = now;
      st.sleep = settings.sleepWhenLying && inputs.lying ? 1 : 0;
    }
    const dt = clamp((now - st.lastMs) / 1000, 0, 0.2);
    st.lastMs = now;
    if (settings.mode === "off") return [{ layer: "blank" }, { layer: "blank" }];
    if (settings.mode === "telemetry") {
      return [0, 1].map((e) => ({ layer: "text", text: { bigFirstLine: true, lines: inputs.telemetry?.[e] || [] } }));
    }
    if (inputs.sticksIdle && !st.wasIdle) {
      st.idleSinceMs = now;
      st.reminderShownThisIdle = false;
    }
    st.wasIdle = inputs.sticksIdle;
    st.sleep = approach(st.sleep, settings.sleepWhenLying && inputs.lying ? 1 : 0, inputs.lying ? 2.5 : 4, dt);

    let blinkOpen = 1;
    if (settings.blink && st.sleep < 0.5) {
      if (!st.blinking && now >= st.nextBlinkMs) {
        st.blinking = true;
        st.blinkStartMs = now;
        st.doubleBlink = random() % 5 === 0;
      }
      if (st.blinking) {
        const t = now - st.blinkStartMs;
        const phase = t % 220;
        if (t >= (st.doubleBlink ? 440 : 220)) {
          st.blinking = false;
          st.nextBlinkMs = now + Math.trunc(randomRange(2500, 6500));
        } else if (phase < 70) blinkOpen = 1 - phase / 70;
        else if (phase < 100) blinkOpen = 0;
        else blinkOpen = (phase - 100) / 120;
      }
    }

    let targetX = st.glanceX;
    let targetY = st.glanceY;
    if (settings.followSticks && !inputs.sticksIdle) {
      targetX = clamp((inputs.turn || 0) * 16 + (inputs.strafe || 0) * 10, -18, 18);
      targetY = clamp(-(inputs.forward || 0) * 4, -6, 6);
      st.nextGlanceMs = now + 1200;
    } else if (now >= st.nextGlanceMs) {
      const centre = random() % 3 === 0;
      st.glanceX = centre ? 0 : randomRange(-14, 14);
      st.glanceY = centre ? 0 : randomRange(-5, 5);
      targetX = st.glanceX;
      targetY = st.glanceY;
      st.nextGlanceMs = now + Math.trunc(randomRange(1500, 4500));
    }
    if (st.sleep > 0.5) { targetX = 0; targetY = 4; }
    st.lookX = approach(st.lookX, targetX, 14, dt);
    st.lookY = approach(st.lookY, targetY, 14, dt);

    if (inputs.reaction && inputs.reaction !== "none") {
      st.reaction = inputs.reaction;
      st.reactionStartMs = now;
    }
    st.pose = [0, 1].map(() => ({ ...basePose(), lookX: st.lookX, lookY: st.lookY }));
    const track = settings.emoteEyes && inputs.emote >= 0 && st.sleep < 0.5
      ? trackSample(tracks, inputs.emote, inputs.emoteElapsedMs) : null;
    if (track) st.pose = track;
    if (st.reaction !== "none") {
      const t = now - st.reactionStartMs;
      if (st.reaction === "pushed" && t < 700) {
        st.pose.forEach((p) => { p.scale = 1.22; });
        blinkOpen = 1;
      } else if (st.reaction === "fallen" && t < 2500) {
        st.pose.forEach((p) => { p.open = 0.45; p.mood = -0.7; });
      } else {
        st.reaction = "none";
      }
    }
    const sleepOpen = 1 - 0.92 * st.sleep;
    st.pose.forEach((p) => { p.open = clamp(p.open * sleepOpen * blinkOpen, 0, 1); });
    const out = [eyeFrame(0), eyeFrame(1)];

    // Battery overlay (same rules as the firmware).
    const pct = Number.isFinite(inputs.batteryPercent) ? Math.round(inputs.batteryPercent) : -1;
    let show = now < st.batteryShowUntilMs;
    let low = false;
    if (pct >= 0 && settings.lowBatteryPercent > 0 && pct <= settings.lowBatteryPercent) {
      low = true;
      const critical = pct <= Math.trunc(settings.lowBatteryPercent / 2);
      if (critical || now % 20000 < 6000) show = Math.trunc(now / 500) % 2 === 0;
    }
    if (!low && settings.reminderMinutes > 0 && inputs.sticksIdle && !st.reminderShownThisIdle &&
        st.sleep < 0.5 && now - st.idleSinceMs >= 15000 &&
        (!st.remindedOnce || now - st.lastReminderMs >= settings.reminderMinutes * 60000)) {
      st.remindedOnce = true;
      st.reminderShownThisIdle = true;
      st.lastReminderMs = now;
      st.batteryShowUntilMs = now + 5000;
      show = true;
    }
    if (show) out[settings.batteryEye === "left" ? 0 : 1] = { layer: "battery", battery: { percent: pct, low } };
    return out;
  }

  return {
    step,
    setSettings(next) { settings = sanitizeFaceSettings(next); },
    get settings() { return settings; },
    showBatteryFor(nowMs, ms) { st.batteryShowUntilMs = nowMs + ms; },
    get sleep() { return st.sleep; },
  };
}
