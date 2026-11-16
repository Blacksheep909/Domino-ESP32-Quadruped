#include "face_renderer.h"

#include <string.h>

#include "face_font.h"

namespace {

int imin(int a, int b) { return a < b ? a : b; }
int imax(int a, int b) { return a > b ? a : b; }
// C++ integer division truncates toward zero; face-renderer.js uses Math.trunc.
int idiv(int a, int b) { return a / b; }

void drawEye(const FaceEye &e, uint8_t *buf) {
  const int w = imax(1, e.w);
  const int h = imax(1, e.h);
  const int left = e.cx - idiv(w, 2);
  const int top = e.cy - idiv(h, 2);
  const int span = imax(1, w - 1);
  const bool neutral = e.lidTop <= 0 && e.slant == 0 && e.smile <= 0;
  for (int y = imax(0, top - 2); y < imin(kFaceHeight, top + h + 2); ++y) {
    for (int x = imax(0, left - 2); x < imin(kFaceWidth, left + w + 2); ++x) {
      const bool inEye = faceInRoundRect(x, y, left, top, w, h, e.r);
      if (inEye) {
        // Upper lid: a straight edge, tilted by `slant` (brows / moods).
        const int lidRow = top + e.lidTop + idiv(e.slant * (x - left), span);
        if (y < lidRow) continue;
        // Lower lid: an ellipse rising from below the eye (happy ^ eyes), a
        // little wider than the eye so the arch is smooth. Doubled coordinates
        // keep it in integers.
        if (e.smile > 0) {
          const int ax = w + 2;           // 2 x semi-axis
          const int by = 2 * e.smile;     // 2 x semi-axis
          const int dx = 2 * (x - left) - (w - 1);
          const int dy = 2 * (y - (top + h)) + 1;
          if (dx * dx * by * by + dy * dy * ax * ax <= ax * ax * by * by) continue;
        }
        faceSetPixel(buf, x, y, true);
      } else if (e.glow && neutral && ((x + y) & 1) == 0 &&
                 faceInRoundRect(x, y, left - 2, top - 2, w + 4, h + 4, e.r + 2)) {
        faceSetPixel(buf, x, y, true);  // dithered halo, like the eyes' glow
      }
    }
  }
}

void drawBattery(const FaceBattery &b, uint8_t *buf) {
  // Battery outline 56 x 26 with a nub, filled in proportion, percent below.
  const int bx = 33, by = 6, bw = 56, bh = 26;
  for (int x = bx; x < bx + bw; ++x) {
    for (int t = 0; t < 2; ++t) {
      faceSetPixel(buf, x, by + t, true);
      faceSetPixel(buf, x, by + bh - 1 - t, true);
    }
  }
  for (int y = by; y < by + bh; ++y) {
    for (int t = 0; t < 2; ++t) {
      faceSetPixel(buf, bx + t, y, true);
      faceSetPixel(buf, bx + bw - 1 - t, y, true);
    }
  }
  for (int y = by + 8; y < by + bh - 8; ++y) {
    for (int x = bx + bw; x < bx + bw + 4; ++x) faceSetPixel(buf, x, y, true);
  }
  const int pct = b.percent < 0 ? 0 : (b.percent > 100 ? 100 : b.percent);
  const int inner = bw - 8;
  const int fill = idiv(inner * pct + 50, 100);
  for (int y = by + 4; y < by + bh - 4; ++y) {
    for (int x = bx + 4; x < bx + 4 + fill; ++x) faceSetPixel(buf, x, y, true);
  }
  char text[12];
  if (b.percent < 0) {
    strcpy(text, "--%");
  } else {
    // "LOW 12%" or "57%"
    int n = 0;
    if (b.low) { memcpy(text, "LOW ", 4); n = 4; }
    if (pct >= 100) text[n++] = '1';
    if (pct >= 10) text[n++] = static_cast<char>('0' + (pct / 10) % 10);
    text[n++] = static_cast<char>('0' + pct % 10);
    text[n++] = '%';
    text[n] = 0;
  }
  const int scale = 2;
  faceDrawText(buf, (kFaceWidth - faceTextWidth(text, scale)) / 2, 40, text, scale);
}

void drawText(const FaceText &t, uint8_t *buf) {
  int y = 0;
  for (int i = 0; i < kFaceTextLines; ++i) {
    const int scale = (i == 0 && t.bigFirstLine) ? 2 : 1;
    faceDrawText(buf, 1, y, t.lines[i], scale);
    y += 8 * scale + (scale == 2 ? 1 : 0);
    if (y >= kFaceHeight) break;
  }
}

}  // namespace

void faceClear(uint8_t *buffer) { memset(buffer, 0, kFaceBufferBytes); }

void faceSetPixel(uint8_t *buffer, int x, int y, bool on) {
  if (x < 0 || x >= kFaceWidth || y < 0 || y >= kFaceHeight) return;
  uint8_t &b = buffer[(y >> 3) * kFaceWidth + x];
  const uint8_t bit = static_cast<uint8_t>(1u << (y & 7));
  if (on) b |= bit; else b &= static_cast<uint8_t>(~bit);
}

bool faceGetPixel(const uint8_t *buffer, int x, int y) {
  if (x < 0 || x >= kFaceWidth || y < 0 || y >= kFaceHeight) return false;
  return (buffer[(y >> 3) * kFaceWidth + x] >> (y & 7)) & 1;
}

bool faceInRoundRect(int x, int y, int left, int top, int w, int h, int r) {
  if (x < left || x >= left + w || y < top || y >= top + h) return false;
  r = imin(r, imin(w / 2, h / 2));
  if (r <= 0) return true;
  int dx = -1, dy = -1;
  if (x < left + r) dx = left + r - 1 - x;
  else if (x >= left + w - r) dx = x - (left + w - r);
  if (y < top + r) dy = top + r - 1 - y;
  else if (y >= top + h - r) dy = y - (top + h - r);
  if (dx < 0 || dy < 0) return true;
  // Circle of radius r through the corner pixels' centres (doubled to stay integer).
  const int a = 2 * dx + 1, b = 2 * dy + 1;
  return a * a + b * b <= 4 * r * r;
}

int faceTextWidth(const char *text, int scale) {
  const int n = static_cast<int>(strlen(text));
  return n > 0 ? (n * 6 - 1) * scale : 0;
}

void faceDrawText(uint8_t *buffer, int x, int y, const char *text, int scale) {
  for (int i = 0; text[i]; ++i) {
    int c = static_cast<unsigned char>(text[i]);
    if (c >= 'a' && c <= 'z') c -= 32;
    if (c < kFaceFontFirst || c > kFaceFontLast) c = '?';
    const uint8_t *glyph = kFaceFont[c - kFaceFontFirst];
    for (int col = 0; col < 5; ++col) {
      for (int row = 0; row < 7; ++row) {
        if (!((glyph[col] >> row) & 1)) continue;
        for (int sy = 0; sy < scale; ++sy) {
          for (int sx = 0; sx < scale; ++sx) {
            faceSetPixel(buffer, x + (i * 6 + col) * scale + sx, y + row * scale + sy, true);
          }
        }
      }
    }
  }
}

void faceRender(const FaceFrame &frame, uint8_t *buffer) {
  faceClear(buffer);
  switch (frame.layer) {
    case FaceLayer::Eye: drawEye(frame.eye, buffer); break;
    case FaceLayer::Battery: drawBattery(frame.battery, buffer); break;
    case FaceLayer::Text: drawText(frame.text, buffer); break;
    case FaceLayer::Blank: break;
  }
}
