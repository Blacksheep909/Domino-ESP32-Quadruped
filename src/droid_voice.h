#pragma once

// Droid voice: plays the emotes in droid_voice_presets.h through a MAX98357A
// I2S amplifier. FIRMWARE STEP "face & voice 1 of 6" (see
// docs/face-voice-firmware-steps.md for the whole sequence and what is next).
//
// Off by default. Build with -DDOMINO_AUDIO_ENABLED=1 once the amplifier is
// wired; with it off every call below is a no-op, so callers need no #if.
//
// Playing is asynchronous: droidVoicePlay() queues the emote and returns at
// once, and a low-priority task on the other core synthesises it, so the
// control loop never waits on audio.

#include <stdint.h>

#include "droid_voice_presets.h"

#ifndef DOMINO_AUDIO_ENABLED
#define DOMINO_AUDIO_ENABLED 0
#endif

#if DOMINO_AUDIO_ENABLED && !defined(DOMINO_SIL)

void droidVoiceBegin();
// False if the emote is unknown or four are already waiting.
bool droidVoicePlay(DroidEmote emote);
// Same, by the name used in sounds/droid_presets.json ("hello", "sad", ...).
bool droidVoicePlayByName(const char *name);
// 0..100. The MAX98357A has no volume input, so this scales the samples.
void droidVoiceSetVolume(uint8_t percent);
uint8_t droidVoiceVolume();
bool droidVoiceBusy();

#else

inline void droidVoiceBegin() {}
inline bool droidVoicePlay(DroidEmote) { return false; }
inline bool droidVoicePlayByName(const char *) { return false; }
inline void droidVoiceSetVolume(uint8_t) {}
inline uint8_t droidVoiceVolume() { return 0; }
inline bool droidVoiceBusy() { return false; }

#endif
