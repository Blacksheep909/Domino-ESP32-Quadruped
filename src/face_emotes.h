#pragma once

// Eye tracks for the personality emotes (src/personality.cpp), same order and
// same durations. Studio's SCREENS / EMOTES previews parse this table
// (face-engine.js), so robot and preview move the eyes identically.
//
// Each key: time ms, open left %, open right %, look x px (+ = robot's left,
// i.e. the viewer's right), look y px (+ = down), smile % (lower lid rises:
// happy ^ eyes), mood % (+ angry brows, - sad brows), scale %.

#include <stdint.h>

struct FaceEmoteKey {
  uint16_t tMs;
  uint8_t openL;
  uint8_t openR;
  int8_t lookX;
  int8_t lookY;
  uint8_t smile;
  int8_t mood;
  uint8_t scale;
};

// FACE_EMOTES_BEGIN
// curious: one eye opens wide, the other squints, looking up to one side.
static const FaceEmoteKey kFaceCurious[] = {
    {0, 100, 100, 0, 0, 0, 0, 100}, {350, 100, 70, 6, -6, 0, 0, 108}, {900, 100, 55, 10, -8, 0, 0, 112},
    {1500, 100, 55, 10, -8, 0, 0, 112}, {2000, 100, 100, 4, -2, 0, 0, 104}, {2600, 100, 100, 0, 0, 0, 0, 100}};
// happy: ^ ^ eyes bobbing with the body.
static const FaceEmoteKey kFaceHappy[] = {
    {0, 100, 100, 0, 0, 0, 0, 100}, {200, 100, 100, 0, -3, 65, 0, 100}, {400, 100, 100, 0, 0, 65, 0, 100},
    {600, 100, 100, 0, -3, 65, 0, 100}, {800, 100, 100, 0, 0, 65, 0, 100}, {1500, 100, 100, 0, 0, 65, 0, 100},
    {2100, 100, 100, 0, 0, 0, 0, 100}};
// bow: look down into the bow, come up happy.
static const FaceEmoteKey kFaceBow[] = {
    {0, 100, 100, 0, 0, 0, 0, 100}, {750, 100, 100, 0, 10, 0, 0, 100}, {1400, 100, 100, 0, 10, 55, 0, 100},
    {2100, 100, 100, 0, 4, 40, 0, 100}, {2700, 100, 100, 0, 0, 0, 0, 100}};
// look: eyes scan with the antennas.
static const FaceEmoteKey kFaceLook[] = {
    {0, 100, 100, 0, 0, 0, 0, 100}, {400, 100, 100, -16, 0, 0, 0, 100}, {1000, 100, 100, -16, 0, 0, 0, 100},
    {1400, 100, 100, 16, 0, 0, 0, 100}, {2000, 100, 100, 16, 0, 0, 0, 100}, {2500, 100, 100, 0, 0, 0, 0, 100},
    {3000, 100, 100, 0, 0, 0, 0, 100}};
// shy: look down and away, half closed, a little smaller.
static const FaceEmoteKey kFaceShy[] = {
    {0, 100, 100, 0, 0, 0, 0, 100}, {650, 55, 55, -8, 8, 0, -40, 85}, {1800, 50, 50, -8, 8, 0, -40, 85},
    {2700, 80, 80, -3, 3, 0, -15, 95}, {3300, 100, 100, 0, 0, 0, 0, 100}};
// FACE_EMOTES_END

struct FaceEmoteTrack {
  const FaceEmoteKey *keys;
  uint8_t count;
};

static const FaceEmoteTrack kFaceEmoteTracks[] = {
    {kFaceCurious, sizeof(kFaceCurious) / sizeof(FaceEmoteKey)},
    {kFaceHappy, sizeof(kFaceHappy) / sizeof(FaceEmoteKey)},
    {kFaceBow, sizeof(kFaceBow) / sizeof(FaceEmoteKey)},
    {kFaceLook, sizeof(kFaceLook) / sizeof(FaceEmoteKey)},
    {kFaceShy, sizeof(kFaceShy) / sizeof(FaceEmoteKey)},
};
constexpr uint8_t kFaceEmoteTrackCount = sizeof(kFaceEmoteTracks) / sizeof(FaceEmoteTrack);
