#pragma once

// Personality: emotes, reaction sounds and idle sounds, and the owner's
// on/off switch for all of it. FIRMWARE STEP "face & voice 6 of 6" (first
// part; see docs/face-voice-firmware-steps.md).
//
// An emote is a short timeline (about 2-3 s) that plays a droid voice preset,
// moves the two antenna servos and nudges the ride height. It is layered on
// top of whatever the legs are doing (stand, Studio gait or the learned
// policy), so it never takes the legs away from the walking controller. Body
// pitch/roll emotes need a policy trained with body-pose commands; until then
// the only body motion is the ride-height nudge, which the policy already
// accepts as its height command.
//
// Settings live in their own NVS record ("personality"), separate from the
// servo calibration. Turning `enabled` off makes the robot silent and still:
// no startup sound, no reactions, no idle sounds, the emote switch does
// nothing and the antennas hold their rest pose.

#include <stddef.h>
#include <stdint.h>

class Adafruit_PWMServoDriver;

#ifndef DOMINO_ANTENNAS_ENABLED
#define DOMINO_ANTENNAS_ENABLED 0
#endif
// PCA9685 channels for the MG90S antennas (5, 6, 12 and 13 are free).
#ifndef DOMINO_ANTENNA_LEFT_CHANNEL
#define DOMINO_ANTENNA_LEFT_CHANNEL 12
#endif
#ifndef DOMINO_ANTENNA_RIGHT_CHANNEL
#define DOMINO_ANTENNA_RIGHT_CHANNEL 13
#endif

enum class PersonalityChatter : uint8_t { Off = 0, Rare = 1, Normal = 2 };

struct PersonalitySettings {
  bool enabled = true;          // master switch for everything below
  bool soundEnabled = true;     // emotes still move when false
  uint8_t volumePercent = 45;   // 0..100
  PersonalityChatter idleChatter = PersonalityChatter::Rare;
  uint8_t emoteChannel = 8;     // 1-based CRSF channel of the momentary emote switch (Boxer SH); 0 = none
  int8_t fixedEmote = -1;       // -1 = a random one each press, else that emote's index
};

constexpr uint8_t kPersonalityEmoteCount = 5;
// Stable names shared with Studio: curious, happy, bow, look, shy.
const char *personalityEmoteName(uint8_t index);
int personalityEmoteIndex(const char *name);  // -1 if unknown

PersonalitySettings defaultPersonalitySettings();
bool validatePersonalitySettings(const PersonalitySettings &settings);

// NVS record (pure, testable on a PC).
constexpr size_t kPersonalityRecordSize = 16;
size_t encodePersonalityRecord(const PersonalitySettings &settings, uint8_t *out, size_t capacity);
bool decodePersonalityRecord(const uint8_t *bytes, size_t length, PersonalitySettings *settings);

// Emote timeline sample (pure): antenna angles in degrees from upright
// (negative = leaning back) and the ride-height offset in mm, `elapsedMs` into
// emote `index`. Returns false once the emote has finished.
struct PersonalityPose {
  float leftAntennaDeg;
  float rightAntennaDeg;
  float heightOffsetMm;
};
constexpr float kAntennaRestDeg = -45.0f;
bool personalityEmotePose(uint8_t index, uint32_t elapsedMs, PersonalityPose *pose);
uint32_t personalityEmoteDurationMs(uint8_t index);

struct PersonalityContext {
  bool linkFresh;      // radio frames are current
  bool robotUpright;   // not fallen / not stowed
  bool sticksIdle;     // no walking command
  bool walking;        // legs stepping (any walking mode)
};

const PersonalitySettings &personalitySettings();
// Applies at once; persist=true also writes NVS and reads it back.
bool setPersonalitySettings(const PersonalitySettings &settings, bool persist, bool *persisted);
bool personalityPersisted();

void personalityBegin(Adafruit_PWMServoDriver &driver);
void personalityUpdate(Adafruit_PWMServoDriver &driver, uint32_t nowMs, const PersonalityContext &context);
// Studio preview / switch press: false while disabled or another emote plays.
bool personalityPlayEmote(int index, uint32_t nowMs);
int personalityActiveEmote();  // -1 when none
// False while stowed / lying down (emotes are refused then).
bool personalityRobotUpright();
uint32_t personalityActiveEmoteElapsedMs(uint32_t nowMs);
// Ride-height nudge from the playing emote (0 when none or disabled).
float personalityHeightOffsetMm();
// Whether reaction / startup / idle sounds may play right now.
bool personalitySoundAllowed();
