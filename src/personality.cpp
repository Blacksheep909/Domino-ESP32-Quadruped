#include "personality.h"

#include <math.h>
#include <string.h>

#ifndef DOMINO_PERSONALITY_PURE
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

#include "crsf.h"
#include "droid_voice.h"
#ifndef DOMINO_SIL
#include <Preferences.h>
#endif
#endif

namespace {

// ---------------------------------------------------------------- emotes
// Keyframes: time (ms), left / right antenna (deg from upright, + forward),
// ride-height offset (mm). Linear in between; the last frame returns to rest.
struct Key {
  uint16_t tMs;
  int8_t left;
  int8_t right;
  int8_t heightMm;
};

constexpr int8_t R = static_cast<int8_t>(kAntennaRestDeg);

// curious: both perk forward, then a lopsided "head tilt", a small rise.
constexpr Key kCurious[] = {
    {0, R, R, 0}, {350, 20, 20, 4}, {900, 30, -5, 8}, {1500, 30, -5, 8}, {2000, 10, 10, 4}, {2600, R, R, 0}};
// happy: quick alternating flutter with two small bobs.
constexpr Key kHappy[] = {
    {0, R, R, 0},     {200, 15, -15, -6}, {400, -15, 15, 0}, {600, 15, -15, -6}, {800, -15, 15, 0},
    {1000, 15, -15, -6}, {1200, -15, 15, 0}, {1500, 0, 0, 0},  {2100, R, R, 0}};
// bow: play-bow dip, antennas forward, hold, back up.
constexpr Key kBow[] = {{0, R, R, 0}, {750, 30, 30, -22}, {1400, 35, 35, -24}, {2100, 10, 10, -6}, {2700, R, R, 0}};
// look: antennas scan left and right like ears tracking a sound.
constexpr Key kLook[] = {
    {0, R, R, 0}, {400, 25, -30, 0}, {1000, 25, -30, 0}, {1400, -30, 25, 0}, {2000, -30, 25, 0}, {2500, 0, 0, 0}, {3000, R, R, 0}};
// shy: ears flat back, sink low, slowly recover.
constexpr Key kShy[] = {{0, R, R, 0}, {650, -80, -80, -18}, {1800, -80, -80, -20}, {2700, -60, -60, -8}, {3300, R, R, 0}};

struct EmoteDef {
  const char *name;
  const char *voice;  // droid_voice preset
  const Key *keys;
  uint8_t keyCount;
};

constexpr EmoteDef kEmotes[kPersonalityEmoteCount] = {
    {"curious", "curious", kCurious, sizeof(kCurious) / sizeof(Key)},
    {"happy", "happy", kHappy, sizeof(kHappy) / sizeof(Key)},
    {"bow", "excited", kBow, sizeof(kBow) / sizeof(Key)},
    {"look", "thinking", kLook, sizeof(kLook) / sizeof(Key)},
    {"shy", "coo", kShy, sizeof(kShy) / sizeof(Key)},
};

// Small, mouth-like sounds for the idle state (never the busy chat presets).
constexpr const char *kIdleVoices[] = {"purr", "coo", "whistle", "peep", "trill"};

constexpr uint32_t kRecordMagic = 0x31535044u;  // "DPS1"

uint32_t checksum(const uint8_t *bytes, size_t length) {
  uint32_t h = 2166136261u;  // FNV-1a
  for (size_t i = 0; i < length; ++i) {
    h ^= bytes[i];
    h *= 16777619u;
  }
  return h;
}

}  // namespace

const char *personalityEmoteName(uint8_t index) {
  return index < kPersonalityEmoteCount ? kEmotes[index].name : "";
}

int personalityEmoteIndex(const char *name) {
  if (!name) return -1;
  for (uint8_t i = 0; i < kPersonalityEmoteCount; ++i) {
    if (!strcmp(kEmotes[i].name, name)) return i;
  }
  return -1;
}

PersonalitySettings defaultPersonalitySettings() { return PersonalitySettings{}; }

bool validatePersonalitySettings(const PersonalitySettings &s) {
  return s.volumePercent <= 100 && static_cast<uint8_t>(s.idleChatter) <= 2 && s.emoteChannel <= 16 &&
         s.fixedEmote >= -1 && s.fixedEmote < static_cast<int8_t>(kPersonalityEmoteCount);
}

size_t encodePersonalityRecord(const PersonalitySettings &s, uint8_t *out, size_t capacity) {
  if (!out || capacity < kPersonalityRecordSize || !validatePersonalitySettings(s)) return 0;
  memset(out, 0, kPersonalityRecordSize);
  for (int i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(kRecordMagic >> (8 * i));
  out[4] = 1;  // record version
  out[5] = static_cast<uint8_t>((s.enabled ? 1 : 0) | (s.soundEnabled ? 2 : 0));
  out[6] = s.volumePercent;
  out[7] = static_cast<uint8_t>(s.idleChatter);
  out[8] = s.emoteChannel;
  out[9] = static_cast<uint8_t>(s.fixedEmote);
  const uint32_t sum = checksum(out, kPersonalityRecordSize - 4);
  for (int i = 0; i < 4; ++i) out[kPersonalityRecordSize - 4 + i] = static_cast<uint8_t>(sum >> (8 * i));
  return kPersonalityRecordSize;
}

bool decodePersonalityRecord(const uint8_t *b, size_t length, PersonalitySettings *s) {
  if (!b || !s || length != kPersonalityRecordSize) return false;
  uint32_t magic = 0, sum = 0;
  for (int i = 0; i < 4; ++i) {
    magic |= static_cast<uint32_t>(b[i]) << (8 * i);
    sum |= static_cast<uint32_t>(b[kPersonalityRecordSize - 4 + i]) << (8 * i);
  }
  if (magic != kRecordMagic || b[4] != 1 || sum != checksum(b, kPersonalityRecordSize - 4)) return false;
  PersonalitySettings out;
  out.enabled = (b[5] & 1) != 0;
  out.soundEnabled = (b[5] & 2) != 0;
  out.volumePercent = b[6];
  out.idleChatter = static_cast<PersonalityChatter>(b[7]);
  out.emoteChannel = b[8];
  out.fixedEmote = static_cast<int8_t>(b[9]);
  if (!validatePersonalitySettings(out)) return false;
  *s = out;
  return true;
}

void personalityEarTargets(bool tilt, bool lying, float rollDeg, float pitchDeg, float yawDeg,
                           float *leftDeg, float *rightDeg) {
  if (lying) {
    *leftDeg = *rightDeg = kEarLyingDeg;
    return;
  }
  if (!tilt) {
    *leftDeg = *rightDeg = kAntennaRestDeg;
    return;
  }
  const float common = kEarTiltBaseDeg + kEarPitchGain * pitchDeg;
  const float split = kEarRollGain * rollDeg + kEarYawGain * yawDeg;
  *leftDeg = fminf(40.0f, fmaxf(-85.0f, common + split));
  *rightDeg = fminf(40.0f, fmaxf(-85.0f, common - split));
}

void personalityEarStep(EarSpring &ear, float targetDeg, float dt) {
  const float w = 2.0f * 3.14159265f * kEarSpringHz;
  // Small fixed substeps keep the spring stable at any loop rate.
  while (dt > 0.0f) {
    const float h = dt > 0.005f ? 0.005f : dt;
    dt -= h;
    const float accel = w * w * (targetDeg - ear.angleDeg) - 2.0f * kEarDamping * w * ear.velocityDegPerSec;
    ear.velocityDegPerSec += accel * h;
    ear.angleDeg += ear.velocityDegPerSec * h;
    if (ear.angleDeg > 40.0f) { ear.angleDeg = 40.0f; ear.velocityDegPerSec = 0.0f; }
    if (ear.angleDeg < -85.0f) { ear.angleDeg = -85.0f; ear.velocityDegPerSec = 0.0f; }
  }
}

uint32_t personalityEmoteDurationMs(uint8_t index) {
  if (index >= kPersonalityEmoteCount) return 0;
  const EmoteDef &e = kEmotes[index];
  return e.keys[e.keyCount - 1].tMs;
}

bool personalityEmotePose(uint8_t index, uint32_t t, PersonalityPose *pose) {
  if (index >= kPersonalityEmoteCount || !pose) return false;
  const EmoteDef &e = kEmotes[index];
  if (t >= e.keys[e.keyCount - 1].tMs) {
    *pose = {kAntennaRestDeg, kAntennaRestDeg, 0.0f};
    return false;
  }
  uint8_t k = 0;
  while (k + 1 < e.keyCount && e.keys[k + 1].tMs <= t) ++k;
  const Key &a = e.keys[k];
  const Key &b = e.keys[k + 1];
  const float u = static_cast<float>(t - a.tMs) / static_cast<float>(b.tMs - a.tMs);
  // Smoothstep between keys: no velocity jumps on the servos or the body.
  const float s = u * u * (3.0f - 2.0f * u);
  pose->leftAntennaDeg = a.left + (b.left - a.left) * s;
  pose->rightAntennaDeg = a.right + (b.right - a.right) * s;
  pose->heightOffsetMm = a.heightMm + (b.heightMm - a.heightMm) * s;
  return true;
}

#ifndef DOMINO_PERSONALITY_PURE

namespace {

PersonalitySettings gSettings;
bool gPersisted = false;
int gActive = -1;
uint32_t gActiveStartMs = 0;
uint32_t gCooldownUntilMs = 0;
int gLastEmote = -1;
bool gSwitchHigh = false;
bool gSwitchArmed = false;  // switch must be seen low once after boot / link loss
uint32_t gIdleSinceMs = 0;
bool gIdle = false;
uint32_t gNextIdleSoundMs = 0;
float gHeightOffsetMm = 0.0f;
bool gUpright = false;  // from the last personalityUpdate(); false until the first one
EarSpring gEarL, gEarR;
uint32_t gEarLastMs = 0;
[[maybe_unused]] float gLastLeftDeg = 1000.0f;
[[maybe_unused]] float gLastRightDeg = 1000.0f;
[[maybe_unused]] uint32_t gLastAntennaWriteMs = 0;

uint32_t gRng = 0x9E3779B9u;
uint32_t nextRandom() {  // xorshift32, seeded from the clock at boot (also runs in SIL)
  gRng ^= gRng << 13;
  gRng ^= gRng >> 17;
  gRng ^= gRng << 5;
  return gRng;
}

constexpr int kSwitchOnUs = 1700;
constexpr int kSwitchOffUs = 1300;
constexpr uint32_t kEmoteCooldownMs = 1200;
constexpr uint32_t kIdleBeforeChatterMs = 20000;
constexpr uint32_t kAntennaWritePeriodMs = 40;

uint32_t idleIntervalMs() {
  // Long, uneven gaps so it never feels like a timer.
  if (gSettings.idleChatter == PersonalityChatter::Normal) return 45000 + nextRandom() % 65000;
  return 90000 + nextRandom() % 110000;
}

void writeAntennas(Adafruit_PWMServoDriver &driver, float leftDeg, float rightDeg, uint32_t now, bool force) {
#if DOMINO_ANTENNAS_ENABLED
  if (!force && now - gLastAntennaWriteMs < kAntennaWritePeriodMs) return;
  if (!force && fabsf(leftDeg - gLastLeftDeg) < 0.5f && fabsf(rightDeg - gLastRightDeg) < 0.5f) return;
  // MG90S: about 11.1 us per degree around 1500 us (upright). The right
  // antenna is mirrored on the face, so its direction flips.
  auto us = [](float deg) {
    const float clamped = fminf(40.0f, fmaxf(-85.0f, deg));
    return static_cast<uint16_t>(1500.0f + clamped * 11.1f);
  };
  driver.writeMicroseconds(DOMINO_ANTENNA_LEFT_CHANNEL, us(leftDeg));
  driver.writeMicroseconds(DOMINO_ANTENNA_RIGHT_CHANNEL, static_cast<uint16_t>(3000 - us(rightDeg)));
  gLastLeftDeg = leftDeg;
  gLastRightDeg = rightDeg;
  gLastAntennaWriteMs = now;
#else
  (void)driver;
  (void)leftDeg;
  (void)rightDeg;
  (void)now;
  (void)force;
#endif
}

void applyVolume() { droidVoiceSetVolume(gSettings.soundEnabled ? gSettings.volumePercent : 0); }

bool loadSettings(PersonalitySettings *out) {
#ifndef DOMINO_SIL
  Preferences prefs;
  if (!prefs.begin("personality", true)) return false;
  uint8_t buf[kPersonalityRecordSize];
  const size_t n = prefs.getBytes("v1", buf, sizeof(buf));
  prefs.end();
  return n == sizeof(buf) && decodePersonalityRecord(buf, n, out);
#else
  (void)out;
  return false;
#endif
}

bool storeSettings(const PersonalitySettings &s) {
#ifndef DOMINO_SIL
  uint8_t buf[kPersonalityRecordSize];
  if (!encodePersonalityRecord(s, buf, sizeof(buf))) return false;
  Preferences prefs;
  if (!prefs.begin("personality", false)) return false;
  const bool wrote = prefs.putBytes("v1", buf, sizeof(buf)) == sizeof(buf);
  prefs.end();
  PersonalitySettings check;
  return wrote && loadSettings(&check) && memcmp(&check, &s, sizeof(s)) == 0;
#else
  (void)s;
  return false;
#endif
}

}  // namespace

const PersonalitySettings &personalitySettings() { return gSettings; }
bool personalityPersisted() { return gPersisted; }

bool setPersonalitySettings(const PersonalitySettings &s, bool persist, bool *persisted) {
  if (!validatePersonalitySettings(s)) return false;
  gSettings = s;
  applyVolume();
  if (!s.enabled) gActive = -1;
  bool ok = true;
  if (persist) {
    ok = storeSettings(s);
    gPersisted = ok;
  }
  if (persisted) *persisted = persist && ok;
  return !persist || ok;
}

void personalityBegin(Adafruit_PWMServoDriver &driver) {
  gRng ^= micros() | 1u;
  PersonalitySettings loaded;
  gPersisted = loadSettings(&loaded);
  gSettings = gPersisted ? loaded : defaultPersonalitySettings();
  applyVolume();
  gNextIdleSoundMs = millis() + idleIntervalMs();
  writeAntennas(driver, kAntennaRestDeg, kAntennaRestDeg, millis(), true);
}

bool personalitySoundAllowed() { return gSettings.enabled && gSettings.soundEnabled; }
int personalityActiveEmote() { return gActive; }
bool personalityRobotUpright() { return gUpright; }
uint32_t personalityActiveEmoteElapsedMs(uint32_t nowMs) { return gActive >= 0 ? nowMs - gActiveStartMs : 0; }
float personalityHeightOffsetMm() { return gHeightOffsetMm; }

bool personalityPlayEmote(int index, uint32_t now) {
  // Emotes are for a standing robot: never while stowed / lying down, from
  // the switch or from Studio.
  if (!gSettings.enabled || !gUpright || gActive >= 0 || now < gCooldownUntilMs) return false;
  if (index < 0) {
    // Random, but never the same one twice in a row.
    index = static_cast<int>(nextRandom() % kPersonalityEmoteCount);
    if (index == gLastEmote) index = (index + 1 + nextRandom() % (kPersonalityEmoteCount - 1)) % kPersonalityEmoteCount;
  }
  if (index >= kPersonalityEmoteCount) return false;
  gActive = index;
  gLastEmote = index;
  gActiveStartMs = now;
  if (personalitySoundAllowed()) droidVoicePlayByName(kEmotes[index].voice);
  gNextIdleSoundMs = now + idleIntervalMs();  // no idle sound right after an emote
  return true;
}

void personalityUpdate(Adafruit_PWMServoDriver &driver, uint32_t now, const PersonalityContext &ctx) {
  gUpright = ctx.robotUpright;
  // Emote switch: momentary, act on the press (rising edge) only.
  if (gSettings.emoteChannel >= 1 && ctx.linkFresh) {
    const int us = ch_us[gSettings.emoteChannel - 1];
    if (!gSwitchHigh && us > kSwitchOnUs) {
      gSwitchHigh = true;
      if (gSwitchArmed && ctx.robotUpright) personalityPlayEmote(gSettings.fixedEmote, now);
    } else if (gSwitchHigh && us < kSwitchOffUs) {
      gSwitchHigh = false;
    }
    if (us < kSwitchOffUs) gSwitchArmed = true;
  } else {
    gSwitchArmed = false;  // a switch held through link loss must not fire on reconnect
  }

  // Emote timeline.
  PersonalityPose pose{kAntennaRestDeg, kAntennaRestDeg, 0.0f};
  if (gActive >= 0) {
    if (!gSettings.enabled || !ctx.robotUpright ||
        !personalityEmotePose(static_cast<uint8_t>(gActive), now - gActiveStartMs, &pose)) {
      gActive = -1;
      gCooldownUntilMs = now + kEmoteCooldownMs;
      pose = {kAntennaRestDeg, kAntennaRestDeg, 0.0f};
    }
  }
  gHeightOffsetMm = pose.heightOffsetMm;
  // Between emotes the ears run on springs (perk up and follow the body in
  // tilt mode, fold back lying down); an emote drives them directly and the
  // springs pick up from wherever it left them.
  const float dt = gEarLastMs ? fminf(0.1f, (now - gEarLastMs) / 1000.0f) : 0.0f;
  gEarLastMs = now;
  if (gActive >= 0) {
    gEarL.angleDeg = pose.leftAntennaDeg;
    gEarL.velocityDegPerSec = 0.0f;
    gEarR.angleDeg = pose.rightAntennaDeg;
    gEarR.velocityDegPerSec = 0.0f;
  } else if (gSettings.enabled) {
    float tl, tr;
    personalityEarTargets(ctx.tiltActive, !ctx.robotUpright, ctx.rollDeg, ctx.pitchDeg, ctx.yawDeg, &tl, &tr);
    personalityEarStep(gEarL, tl, dt);
    personalityEarStep(gEarR, tr, dt);
    pose.leftAntennaDeg = gEarL.angleDeg;
    pose.rightAntennaDeg = gEarR.angleDeg;
  }
  writeAntennas(driver, pose.leftAntennaDeg, pose.rightAntennaDeg, now, false);

  // Idle sounds: only after a long, genuinely quiet spell, and rarely.
  const bool idleNow = ctx.linkFresh && ctx.robotUpright && ctx.sticksIdle && !ctx.walking && gActive < 0;
  if (idleNow && !gIdle) gIdleSinceMs = now;
  gIdle = idleNow;
  if (!idleNow || !personalitySoundAllowed() || gSettings.idleChatter == PersonalityChatter::Off) {
    if (now > gNextIdleSoundMs) gNextIdleSoundMs = now + idleIntervalMs();
    return;
  }
  if (now - gIdleSinceMs >= kIdleBeforeChatterMs && now >= gNextIdleSoundMs && !droidVoiceBusy()) {
    droidVoicePlayByName(kIdleVoices[nextRandom() % (sizeof(kIdleVoices) / sizeof(kIdleVoices[0]))]);
    gNextIdleSoundMs = now + idleIntervalMs();
  }
}

#endif  // DOMINO_PERSONALITY_PURE
