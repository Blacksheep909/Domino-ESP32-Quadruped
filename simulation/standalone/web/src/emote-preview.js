// Sound for the LIVE / EMOTES 3D preview: an emote's droid voice rendered in
// the browser by the same synthesiser the firmware uses
// (sounds/droid-synth.mjs + sounds/droid_presets.json). No robot needed.
// Timelines: ./emote-timelines.js (kept in sync with the firmware by a test).

import { renderEmote } from "../../../../sounds/droid-synth.mjs";
import DROID_PRESETS from "../../../../sounds/droid_presets.json";
import { ANTENNA_REST_DEG, EMOTE_TIMELINES, emoteDurationMs, emotePose } from "./emote-timelines.js";

export { ANTENNA_REST_DEG, EMOTE_TIMELINES, emoteDurationMs, emotePose };

let audioContext = null;
let audioSource = null;

// Plays an emote's droid voice in the browser (same synthesis as the robot).
export function playEmoteSound(id, volumePercent = 45) {
  try { audioSource?.stop(); } catch { /* already stopped */ }
  audioSource = null;
  const voice = EMOTE_TIMELINES[id]?.voice;
  const segments = voice ? DROID_PRESETS.emotes?.[voice] : null;
  if (!segments || volumePercent <= 0) return false;
  audioContext ??= new AudioContext();
  const samples = renderEmote(segments, DROID_PRESETS.sampleRate, { fx: DROID_PRESETS.fx });
  const buffer = audioContext.createBuffer(1, samples.length, DROID_PRESETS.sampleRate);
  buffer.copyToChannel(samples, 0);
  audioSource = audioContext.createBufferSource();
  const gain = audioContext.createGain();
  gain.gain.value = Math.min(1, volumePercent / 100);
  audioSource.buffer = buffer;
  audioSource.connect(gain).connect(audioContext.destination);
  audioSource.start();
  return true;
}
