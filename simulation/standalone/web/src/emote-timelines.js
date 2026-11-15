// Emote timelines for Studio's local preview. Mirrors the keyframe tables in
// src/personality.cpp; live-personality-protocol.test.mjs parses the C++ and
// fails if the two drift apart.

export const ANTENNA_REST_DEG = -45;
const R = ANTENNA_REST_DEG;

// [timeMs, leftAntennaDeg, rightAntennaDeg, heightOffsetMm]; + antenna = forward.
export const EMOTE_TIMELINES = Object.freeze({
  curious: { voice: "curious", keys: [[0, R, R, 0], [350, 20, 20, 4], [900, 30, -5, 8], [1500, 30, -5, 8], [2000, 10, 10, 4], [2600, R, R, 0]] },
  happy: { voice: "happy", keys: [[0, R, R, 0], [200, 15, -15, -6], [400, -15, 15, 0], [600, 15, -15, -6], [800, -15, 15, 0], [1000, 15, -15, -6], [1200, -15, 15, 0], [1500, 0, 0, 0], [2100, R, R, 0]] },
  bow: { voice: "excited", keys: [[0, R, R, 0], [750, 30, 30, -22], [1400, 35, 35, -24], [2100, 10, 10, -6], [2700, R, R, 0]] },
  look: { voice: "thinking", keys: [[0, R, R, 0], [400, 25, -30, 0], [1000, 25, -30, 0], [1400, -30, 25, 0], [2000, -30, 25, 0], [2500, 0, 0, 0], [3000, R, R, 0]] },
  shy: { voice: "coo", keys: [[0, R, R, 0], [650, -80, -80, -18], [1800, -80, -80, -20], [2700, -60, -60, -8], [3300, R, R, 0]] },
});

export function emoteDurationMs(id) {
  const keys = EMOTE_TIMELINES[id]?.keys;
  return keys ? keys[keys.length - 1][0] : 0;
}

// Same smoothstep interpolation as personalityEmotePose() in the firmware.
export function emotePose(id, tMs) {
  const keys = EMOTE_TIMELINES[id]?.keys;
  if (!keys || tMs >= keys[keys.length - 1][0] || tMs < 0) {
    return { left: R, right: R, height: 0, playing: false };
  }
  let k = 0;
  while (k + 1 < keys.length && keys[k + 1][0] <= tMs) k += 1;
  const [ta, la, ra, ha] = keys[k];
  const [tb, lb, rb, hb] = keys[k + 1];
  const u = (tMs - ta) / (tb - ta);
  const s = u * u * (3 - 2 * u);
  return { left: la + (lb - la) * s, right: ra + (rb - ra) * s, height: ha + (hb - ha) * s, playing: true };
}
