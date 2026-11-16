#pragma once

// Draws one 128x64 monochrome OLED frame (an eye, the battery, or text) into
// an SSD1306/SSD1309 page buffer: byte (y / 8) * 128 + x, bit y % 8.
//
// Pure integer maths with no Arduino dependency, so it runs on a PC too.
// Studio's SCREENS preview (web/src/face-renderer.js) is a line-for-line port
// and a test checks both produce the same bytes for the same inputs.

#include <stdint.h>

constexpr int kFaceWidth = 128;
constexpr int kFaceHeight = 64;
constexpr int kFaceBufferBytes = kFaceWidth * kFaceHeight / 8;

enum class FaceLayer : uint8_t { Blank = 0, Eye = 1, Battery = 2, Text = 3 };

// One eye: a rounded rectangle (BURN-E style) with lids.
//  lidTop     rows hidden from the top at the eye's left edge
//  slant      extra rows hidden at the right edge (+) or removed (-): mood brows
//  smile      rows the lower lid rises at the centre (a happy ^ eye)
//  glow       a dithered halo around a fully open eye
struct FaceEye {
  int16_t cx = 64;
  int16_t cy = 32;
  int16_t w = 40;
  int16_t h = 38;
  int16_t r = 9;
  int16_t lidTop = 0;
  int16_t slant = 0;
  int16_t smile = 0;
  bool glow = true;
};

struct FaceBattery {
  int8_t percent = -1;  // -1 = unknown
  bool low = false;     // draws "LOW" with the percent
};

constexpr int kFaceTextLines = 7;
constexpr int kFaceTextChars = 21;
struct FaceText {
  char lines[kFaceTextLines][kFaceTextChars + 1] = {};
  bool bigFirstLine = false;  // first line at 2x
};

struct FaceFrame {
  FaceLayer layer = FaceLayer::Blank;
  FaceEye eye;
  FaceBattery battery;
  FaceText text;
};

void faceRender(const FaceFrame &frame, uint8_t *buffer);

// Building blocks (exposed for tests and the battery / text layers).
void faceClear(uint8_t *buffer);
void faceSetPixel(uint8_t *buffer, int x, int y, bool on);
bool faceGetPixel(const uint8_t *buffer, int x, int y);
bool faceInRoundRect(int x, int y, int left, int top, int w, int h, int r);
void faceDrawText(uint8_t *buffer, int x, int y, const char *text, int scale);
int faceTextWidth(const char *text, int scale);
