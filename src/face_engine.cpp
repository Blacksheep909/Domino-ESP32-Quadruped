#include "face_engine.h"

#include <math.h>
#include <string.h>

#include "face_emotes.h"

namespace {

constexpr uint32_t kRecordMagic = 0x31534644u;  // "DFS1"

struct StyleDims {
  const char *name;
  int16_t w, h, r;
};
// BURN-E: small rounded squares. The others are variations to taste.
constexpr StyleDims kStyles[kFaceStyleCount] = {
    {"burne", 48, 36, 9},
    {"round", 40, 40, 20},
    {"tall", 30, 46, 12},
    {"wide", 54, 28, 8},
};

uint32_t checksum(const uint8_t *b, size_t n) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; ++i) {
    h ^= b[i];
    h *= 16777619u;
  }
  return h;
}

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float approach(float cur, float target, float rate, float dt) {
  // Exponential ease: covers ~63% of the gap every 1/rate seconds.
  const float k = 1.0f - expf(-rate * dt);
  return cur + (target - cur) * k;
}
float smooth(float u) { return u * u * (3.0f - 2.0f * u); }

// Emote eye track sample (smoothstep between keys, like the antenna tracks).
bool trackSample(int emote, uint32_t t, FaceEyePose pose[2]) {
  if (emote < 0 || emote >= kFaceEmoteTrackCount) return false;
  const FaceEmoteTrack &tr = kFaceEmoteTracks[emote];
  if (t >= tr.keys[tr.count - 1].tMs) return false;
  uint8_t k = 0;
  while (k + 1 < tr.count && tr.keys[k + 1].tMs <= t) ++k;
  const FaceEmoteKey &a = tr.keys[k];
  const FaceEmoteKey &b = tr.keys[k + 1];
  const float s = smooth(static_cast<float>(t - a.tMs) / static_cast<float>(b.tMs - a.tMs));
  auto mix = [s](float x, float y) { return x + (y - x) * s; };
  pose[0].open = mix(a.openL, b.openL) / 100.0f;
  pose[1].open = mix(a.openR, b.openR) / 100.0f;
  for (int e = 0; e < 2; ++e) {
    pose[e].lookX = mix(a.lookX, b.lookX);
    pose[e].lookY = mix(a.lookY, b.lookY);
    pose[e].smile = mix(a.smile, b.smile) / 100.0f;
    pose[e].mood = mix(a.mood, b.mood) / 100.0f;
    pose[e].scale = mix(a.scale, b.scale) / 100.0f;
  }
  return true;
}

}  // namespace

FaceSettings defaultFaceSettings() { return FaceSettings{}; }

bool validateFaceSettings(const FaceSettings &s) {
  return static_cast<uint8_t>(s.mode) <= 2 && static_cast<uint8_t>(s.style) < kFaceStyleCount &&
         s.brightness <= 100 && s.batteryEye <= 1 && s.lowBatteryPercent <= 50 && s.reminderMinutes <= 60;
}

const char *faceStyleName(FaceStyle style) {
  const uint8_t i = static_cast<uint8_t>(style);
  return i < kFaceStyleCount ? kStyles[i].name : "";
}

int faceStyleIndex(const char *name) {
  if (!name) return -1;
  for (int i = 0; i < kFaceStyleCount; ++i) {
    if (!strcmp(kStyles[i].name, name)) return i;
  }
  return -1;
}

size_t encodeFaceRecord(const FaceSettings &s, uint8_t *out, size_t capacity) {
  if (!out || capacity < kFaceRecordSize || !validateFaceSettings(s)) return 0;
  memset(out, 0, kFaceRecordSize);
  for (int i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(kRecordMagic >> (8 * i));
  out[4] = 1;
  out[5] = static_cast<uint8_t>(s.mode);
  out[6] = static_cast<uint8_t>(s.style);
  out[7] = s.brightness;
  out[8] = s.batteryEye;
  out[9] = s.lowBatteryPercent;
  out[10] = s.reminderMinutes;
  out[11] = static_cast<uint8_t>((s.glow ? 1 : 0) | (s.blink ? 2 : 0) | (s.sleepWhenLying ? 4 : 0) |
                                 (s.followSticks ? 8 : 0) | (s.emoteEyes ? 16 : 0));
  const uint32_t sum = checksum(out, kFaceRecordSize - 4);
  for (int i = 0; i < 4; ++i) out[kFaceRecordSize - 4 + i] = static_cast<uint8_t>(sum >> (8 * i));
  return kFaceRecordSize;
}

bool decodeFaceRecord(const uint8_t *b, size_t length, FaceSettings *s) {
  if (!b || !s || length != kFaceRecordSize) return false;
  uint32_t magic = 0, sum = 0;
  for (int i = 0; i < 4; ++i) {
    magic |= static_cast<uint32_t>(b[i]) << (8 * i);
    sum |= static_cast<uint32_t>(b[kFaceRecordSize - 4 + i]) << (8 * i);
  }
  if (magic != kRecordMagic || b[4] != 1 || sum != checksum(b, kFaceRecordSize - 4)) return false;
  FaceSettings out;
  out.mode = static_cast<FaceMode>(b[5]);
  out.style = static_cast<FaceStyle>(b[6]);
  out.brightness = b[7];
  out.batteryEye = b[8];
  out.lowBatteryPercent = b[9];
  out.reminderMinutes = b[10];
  out.glow = b[11] & 1;
  out.blink = b[11] & 2;
  out.sleepWhenLying = b[11] & 4;
  out.followSticks = b[11] & 8;
  out.emoteEyes = b[11] & 16;
  if (!validateFaceSettings(out)) return false;
  *s = out;
  return true;
}

FaceEngine::FaceEngine(uint32_t seed) : rng_(seed ? seed : 1) {}

uint32_t FaceEngine::random() {
  rng_ ^= rng_ << 13;
  rng_ ^= rng_ >> 17;
  rng_ ^= rng_ << 5;
  return rng_;
}

float FaceEngine::randomRange(float lo, float hi) {
  return lo + (hi - lo) * static_cast<float>(random() % 10000u) / 10000.0f;
}

void FaceEngine::showBatteryFor(uint32_t nowMs, uint32_t ms) { batteryShowUntilMs_ = nowMs + ms; }

void FaceEngine::step(const FaceInputs &in, FaceFrame out[2]) {
  const uint32_t now = in.nowMs;
  if (!started_) {
    started_ = true;
    lastMs_ = now;
    nextBlinkMs_ = now + 1500;
    nextGlanceMs_ = now + 2000;
    idleSinceMs_ = now;
    lastReminderMs_ = now;
    sleep_ = (settings_.sleepWhenLying && in.lying) ? 1.0f : 0.0f;
  }
  const float dt = clampf(static_cast<float>(now - lastMs_) / 1000.0f, 0.0f, 0.2f);
  lastMs_ = now;

  if (settings_.mode == FaceMode::Off) {
    out[0] = FaceFrame{};
    out[1] = FaceFrame{};
    return;
  }
  if (settings_.mode == FaceMode::Telemetry) {
    for (int e = 0; e < 2; ++e) {
      out[e] = FaceFrame{};
      out[e].layer = FaceLayer::Text;
      out[e].text.bigFirstLine = true;
      if (in.telemetry[e]) memcpy(out[e].text.lines, in.telemetry[e], sizeof(out[e].text.lines));
    }
    return;
  }

  // Idle tracking (for glances and the battery reminder).
  if (in.sticksIdle && !wasIdle_) {
    idleSinceMs_ = now;
    reminderShownThisIdle_ = false;
  }
  wasIdle_ = in.sticksIdle;

  // Sleep: slim lines when lying down, eyes open as it stands up.
  sleep_ = approach(sleep_, (settings_.sleepWhenLying && in.lying) ? 1.0f : 0.0f, in.lying ? 2.5f : 4.0f, dt);

  // Blink: quick close, short hold, slower open; sometimes twice.
  float blinkOpen = 1.0f;
  if (settings_.blink && sleep_ < 0.5f) {
    if (!blinking_ && now >= nextBlinkMs_) {
      blinking_ = true;
      blinkStartMs_ = now;
      doubleBlink_ = random() % 5 == 0;
    }
    if (blinking_) {
      const uint32_t t = now - blinkStartMs_;
      const uint32_t one = 220;
      const uint32_t phase = t % one;
      if (t >= (doubleBlink_ ? 2 * one : one)) {
        blinking_ = false;
        nextBlinkMs_ = now + static_cast<uint32_t>(randomRange(2500.0f, 6500.0f));
      } else if (phase < 70) {
        blinkOpen = 1.0f - phase / 70.0f;
      } else if (phase < 100) {
        blinkOpen = 0.0f;
      } else {
        blinkOpen = (phase - 100) / 120.0f;
      }
    }
  }

  // Where to look: the sticks when steering, otherwise the odd idle glance.
  float targetX = glanceX_, targetY = glanceY_;
  if (settings_.followSticks && !in.sticksIdle) {
    targetX = clampf(in.turn * 16.0f + in.strafe * 10.0f, -18.0f, 18.0f);
    targetY = clampf(-in.forward * 4.0f, -6.0f, 6.0f);
    nextGlanceMs_ = now + 1200;
  } else if (now >= nextGlanceMs_) {
    const bool centre = random() % 3 == 0;
    glanceX_ = centre ? 0.0f : randomRange(-14.0f, 14.0f);
    glanceY_ = centre ? 0.0f : randomRange(-5.0f, 5.0f);
    targetX = glanceX_;
    targetY = glanceY_;
    nextGlanceMs_ = now + static_cast<uint32_t>(randomRange(1500.0f, 4500.0f));
  }
  if (sleep_ > 0.5f) {
    targetX = 0.0f;
    targetY = 4.0f;  // droopy
  }
  lookX_ = approach(lookX_, targetX, 14.0f, dt);
  lookY_ = approach(lookY_, targetY, 14.0f, dt);

  if (in.reaction != FaceReaction::None) {
    reaction_ = in.reaction;
    reactionStartMs_ = now;
  }

  for (int e = 0; e < 2; ++e) {
    FaceEyePose &p = pose_[e];
    p = FaceEyePose{};
    p.lookX = lookX_;
    p.lookY = lookY_;
  }
  FaceEyePose track[2];
  const bool tracking = settings_.emoteEyes && in.emote >= 0 && sleep_ < 0.5f &&
                        trackSample(in.emote, in.emoteElapsedMs, track);
  if (tracking) {
    for (int e = 0; e < 2; ++e) pose_[e] = track[e];
  }
  // Reactions: wide eyes when shoved, squeezed sad eyes after a fall.
  if (reaction_ != FaceReaction::None) {
    const uint32_t t = now - reactionStartMs_;
    if (reaction_ == FaceReaction::Pushed && t < 700) {
      for (int e = 0; e < 2; ++e) pose_[e].scale = 1.22f;
      blinkOpen = 1.0f;
    } else if (reaction_ == FaceReaction::Fallen && t < 2500) {
      for (int e = 0; e < 2; ++e) {
        pose_[e].open = 0.45f;
        pose_[e].mood = -0.7f;
      }
    } else {
      reaction_ = FaceReaction::None;
    }
  }
  const float sleepOpen = 1.0f - 0.92f * sleep_;
  for (int e = 0; e < 2; ++e) {
    pose_[e].open = clampf(pose_[e].open * sleepOpen * blinkOpen, 0.0f, 1.0f);
    eyeFrame(e, out[e]);
  }
  batteryOverlay(in, out);
}

void FaceEngine::eyeFrame(int eye, FaceFrame &f) const {
  const FaceEyePose &p = pose_[eye];
  const StyleDims &st = kStyles[static_cast<uint8_t>(settings_.style)];
  f = FaceFrame{};
  f.layer = FaceLayer::Eye;
  FaceEye &e = f.eye;
  e.w = static_cast<int16_t>(lroundf(st.w * p.scale));
  const float fullH = st.h * p.scale;
  e.h = static_cast<int16_t>(lroundf(fullH * p.open));
  if (e.h < 2) e.h = 2;
  e.r = static_cast<int16_t>(st.r * p.scale);
  if (e.r > e.h / 2) e.r = static_cast<int16_t>(e.h / 2);
  e.cx = static_cast<int16_t>(lroundf(kFaceWidth / 2 + p.lookX));
  e.cy = static_cast<int16_t>(lroundf(kFaceHeight / 2 + p.lookY));
  e.glow = settings_.glow;
  e.smile = static_cast<int16_t>(lroundf(e.h * 0.8f * clampf(p.smile, 0.0f, 1.0f)));
  // Mood brows: the inner edge (toward the other eye) drops for angry and
  // lifts for sad. Left eye's inner edge is its low-x side.
  const int16_t k = static_cast<int16_t>(lroundf(e.h * 0.4f * fabsf(clampf(p.mood, -1.0f, 1.0f))));
  const bool angry = p.mood > 0.0f;
  const bool innerLow = (eye == 0);  // left eye: inner edge at low x
  if (k > 0) {
    if (angry == innerLow) {
      e.lidTop = k;
      e.slant = static_cast<int16_t>(-k);
    } else {
      e.lidTop = 0;
      e.slant = k;
    }
  }
}

void FaceEngine::batteryOverlay(const FaceInputs &in, FaceFrame out[2]) {
  const uint32_t now = in.nowMs;
  const int pct = in.batteryPercent;
  bool show = now < batteryShowUntilMs_;
  bool low = false;
  // Low battery: flash for 6 s every 20 s; below half the threshold, all the time.
  if (pct >= 0 && settings_.lowBatteryPercent > 0 && pct <= settings_.lowBatteryPercent) {
    low = true;
    const bool critical = pct <= settings_.lowBatteryPercent / 2;
    const bool window = critical || (now % 20000u) < 6000u;
    if (window) show = ((now / 500u) & 1u) == 0;  // 1 Hz flash
  }
  // Reminder: after 15 s of standing idle, show the battery for 5 s, at most
  // once every `reminderMinutes`.
  if (!low && settings_.reminderMinutes > 0 && in.sticksIdle && !reminderShownThisIdle_ &&
      sleep_ < 0.5f && now - idleSinceMs_ >= 15000u &&
      (!remindedOnce_ || now - lastReminderMs_ >= settings_.reminderMinutes * 60000u)) {
    remindedOnce_ = true;
    reminderShownThisIdle_ = true;
    lastReminderMs_ = now;
    batteryShowUntilMs_ = now + 5000u;
    show = true;
  }
  if (!show) return;
  FaceFrame &f = out[settings_.batteryEye ? 1 : 0];
  f = FaceFrame{};
  f.layer = FaceLayer::Battery;
  f.battery.percent = static_cast<int8_t>(pct);
  f.battery.low = low;
}
