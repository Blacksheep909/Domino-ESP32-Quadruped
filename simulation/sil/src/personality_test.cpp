// Native checks for the pure parts of src/personality.cpp: the NVS record,
// the emote table and the emote timelines. Run: simulation/sil/test-personality.ps1
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "personality.h"

static int failures = 0;
#define CHECK(cond)                                              \
  do {                                                           \
    if (!(cond)) {                                               \
      printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond);      \
      ++failures;                                                \
    }                                                            \
  } while (0)

int main() {
  // Defaults: on, sound on, rare idle sounds, emote switch on CH8 (Boxer SH), random emote.
  const PersonalitySettings d = defaultPersonalitySettings();
  CHECK(d.enabled && d.soundEnabled && d.idleChatter == PersonalityChatter::Rare);
  CHECK(d.emoteChannel == 8 && d.fixedEmote == -1 && validatePersonalitySettings(d));

  // Record round trip, including "everything off".
  PersonalitySettings off = d;
  off.enabled = false;
  off.soundEnabled = false;
  off.volumePercent = 0;
  off.idleChatter = PersonalityChatter::Off;
  off.emoteChannel = 0;
  off.fixedEmote = 3;
  uint8_t buf[kPersonalityRecordSize];
  CHECK(encodePersonalityRecord(off, buf, sizeof(buf)) == kPersonalityRecordSize);
  PersonalitySettings back;
  CHECK(decodePersonalityRecord(buf, sizeof(buf), &back));
  CHECK(!back.enabled && !back.soundEnabled && back.volumePercent == 0 &&
        back.idleChatter == PersonalityChatter::Off && back.emoteChannel == 0 && back.fixedEmote == 3);

  // A flipped bit, a wrong size or out-of-range values are rejected.
  buf[6] ^= 1;
  CHECK(!decodePersonalityRecord(buf, sizeof(buf), &back));
  CHECK(!decodePersonalityRecord(buf, sizeof(buf) - 1, &back));
  PersonalitySettings bad = d;
  bad.volumePercent = 101;
  CHECK(!validatePersonalitySettings(bad) && encodePersonalityRecord(bad, buf, sizeof(buf)) == 0);
  bad = d;
  bad.emoteChannel = 17;
  CHECK(!validatePersonalitySettings(bad));
  bad = d;
  bad.fixedEmote = kPersonalityEmoteCount;
  CHECK(!validatePersonalitySettings(bad));

  // Emote names are stable (shared with Studio) and every timeline starts and
  // ends at rest, stays in the antenna travel and nudges height gently.
  const char *names[] = {"curious", "happy", "bow", "look", "shy"};
  for (uint8_t i = 0; i < kPersonalityEmoteCount; ++i) {
    CHECK(!strcmp(personalityEmoteName(i), names[i]) && personalityEmoteIndex(names[i]) == i);
    const uint32_t duration = personalityEmoteDurationMs(i);
    CHECK(duration >= 1500 && duration <= 3500);
    PersonalityPose p;
    CHECK(personalityEmotePose(i, 0, &p));
    CHECK(fabsf(p.leftAntennaDeg - kAntennaRestDeg) < 0.01f && fabsf(p.heightOffsetMm) < 0.01f);
    float prevHeight = 0.0f;
    for (uint32_t t = 0; t < duration; t += 10) {
      CHECK(personalityEmotePose(i, t, &p));
      CHECK(p.leftAntennaDeg >= -85.0f && p.leftAntennaDeg <= 40.0f);
      CHECK(p.rightAntennaDeg >= -85.0f && p.rightAntennaDeg <= 40.0f);
      CHECK(p.heightOffsetMm >= -25.0f && p.heightOffsetMm <= 10.0f);
      // Height moves at most ~60 mm/s (well inside the stand ramp).
      if (t > 0) CHECK(fabsf(p.heightOffsetMm - prevHeight) <= 0.6f);
      prevHeight = p.heightOffsetMm;
    }
    CHECK(!personalityEmotePose(i, duration, &p));
    CHECK(fabsf(p.rightAntennaDeg - kAntennaRestDeg) < 0.01f && fabsf(p.heightOffsetMm) < 0.01f);
  }
  CHECK(personalityEmoteIndex("dance") == -1 && personalityEmoteIndex(nullptr) == -1);

  if (failures) {
    printf("%d personality check(s) failed\n", failures);
    return 1;
  }
  printf("personality ok: %u emotes\n", static_cast<unsigned>(kPersonalityEmoteCount));
  return 0;
}
