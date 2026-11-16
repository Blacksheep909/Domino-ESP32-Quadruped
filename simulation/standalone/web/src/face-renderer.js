// Line-for-line port of src/face_renderer.cpp: draws one 128x64 OLED frame into
// an SSD1306/SSD1309 page buffer (byte (y >> 3) * 128 + x, bit y & 7). The
// font comes from src/face_font.h itself (parseFaceFont), and a test checks
// this file and the firmware produce identical bytes.

export const FACE_WIDTH = 128;
export const FACE_HEIGHT = 64;
export const FACE_BUFFER_BYTES = (FACE_WIDTH * FACE_HEIGHT) / 8;
export const FACE_TEXT_LINES = 7;

const idiv = (a, b) => Math.trunc(a / b);

export function parseFaceFont(headerText) {
  const block = headerText.split("FACE_FONT_BEGIN")[1].split("FACE_FONT_END")[0];
  const glyphs = [...block.matchAll(/\{([^{}]*)\}/g)].map((m) =>
    m[1].split(",").map((v) => Number.parseInt(v.trim(), 16)));
  if (glyphs.length !== 64 || glyphs.some((g) => g.length !== 5 || g.some(Number.isNaN))) {
    throw new Error("face_font.h: expected 64 five-column glyphs");
  }
  return glyphs;
}

export function createFaceBuffer() {
  return new Uint8Array(FACE_BUFFER_BYTES);
}

export function faceSetPixel(buffer, x, y, on) {
  if (x < 0 || x >= FACE_WIDTH || y < 0 || y >= FACE_HEIGHT) return;
  const i = (y >> 3) * FACE_WIDTH + x;
  const bit = 1 << (y & 7);
  buffer[i] = on ? buffer[i] | bit : buffer[i] & ~bit & 0xff;
}

export function faceGetPixel(buffer, x, y) {
  if (x < 0 || x >= FACE_WIDTH || y < 0 || y >= FACE_HEIGHT) return false;
  return ((buffer[(y >> 3) * FACE_WIDTH + x] >> (y & 7)) & 1) === 1;
}

export function faceInRoundRect(x, y, left, top, w, h, r) {
  if (x < left || x >= left + w || y < top || y >= top + h) return false;
  r = Math.min(r, Math.min(idiv(w, 2), idiv(h, 2)));
  if (r <= 0) return true;
  let dx = -1;
  let dy = -1;
  if (x < left + r) dx = left + r - 1 - x;
  else if (x >= left + w - r) dx = x - (left + w - r);
  if (y < top + r) dy = top + r - 1 - y;
  else if (y >= top + h - r) dy = y - (top + h - r);
  if (dx < 0 || dy < 0) return true;
  const a = 2 * dx + 1;
  const b = 2 * dy + 1;
  return a * a + b * b <= 4 * r * r;
}

export function faceTextWidth(text, scale) {
  return text.length > 0 ? (text.length * 6 - 1) * scale : 0;
}

export function createFaceRenderer(font) {
  function drawText(buffer, x, y, text, scale) {
    for (let i = 0; i < text.length; i += 1) {
      let c = text.charCodeAt(i);
      if (c >= 97 && c <= 122) c -= 32;
      if (c < 32 || c > 95) c = 63;
      const glyph = font[c - 32];
      for (let col = 0; col < 5; col += 1) {
        for (let row = 0; row < 7; row += 1) {
          if (!((glyph[col] >> row) & 1)) continue;
          for (let sy = 0; sy < scale; sy += 1) {
            for (let sx = 0; sx < scale; sx += 1) {
              faceSetPixel(buffer, x + (i * 6 + col) * scale + sx, y + row * scale + sy, true);
            }
          }
        }
      }
    }
  }

  function drawEye(e, buf) {
    const w = Math.max(1, e.w);
    const h = Math.max(1, e.h);
    const left = e.cx - idiv(w, 2);
    const top = e.cy - idiv(h, 2);
    const span = Math.max(1, w - 1);
    const neutral = e.lidTop <= 0 && e.slant === 0 && e.smile <= 0;
    for (let y = Math.max(0, top - 2); y < Math.min(FACE_HEIGHT, top + h + 2); y += 1) {
      for (let x = Math.max(0, left - 2); x < Math.min(FACE_WIDTH, left + w + 2); x += 1) {
        const inEye = faceInRoundRect(x, y, left, top, w, h, e.r);
        if (inEye) {
          const lidRow = top + e.lidTop + idiv(e.slant * (x - left), span);
          if (y < lidRow) continue;
          if (e.smile > 0) {
            const ax = w + 2;
            const by = 2 * e.smile;
            const dx = 2 * (x - left) - (w - 1);
            const dy = 2 * (y - (top + h)) + 1;
            if (dx * dx * by * by + dy * dy * ax * ax <= ax * ax * by * by) continue;
          }
          faceSetPixel(buf, x, y, true);
        } else if (e.glow && neutral && ((x + y) & 1) === 0 &&
                   faceInRoundRect(x, y, left - 2, top - 2, w + 4, h + 4, e.r + 2)) {
          faceSetPixel(buf, x, y, true);
        }
      }
    }
  }

  function drawBattery(b, buf) {
    const bx = 33, by = 6, bw = 56, bh = 26;
    for (let x = bx; x < bx + bw; x += 1) {
      for (let t = 0; t < 2; t += 1) {
        faceSetPixel(buf, x, by + t, true);
        faceSetPixel(buf, x, by + bh - 1 - t, true);
      }
    }
    for (let y = by; y < by + bh; y += 1) {
      for (let t = 0; t < 2; t += 1) {
        faceSetPixel(buf, bx + t, y, true);
        faceSetPixel(buf, bx + bw - 1 - t, y, true);
      }
    }
    for (let y = by + 8; y < by + bh - 8; y += 1) {
      for (let x = bx + bw; x < bx + bw + 4; x += 1) faceSetPixel(buf, x, y, true);
    }
    const pct = b.percent < 0 ? 0 : (b.percent > 100 ? 100 : b.percent);
    const inner = bw - 8;
    const fill = idiv(inner * pct + 50, 100);
    for (let y = by + 4; y < by + bh - 4; y += 1) {
      for (let x = bx + 4; x < bx + 4 + fill; x += 1) faceSetPixel(buf, x, y, true);
    }
    let text;
    if (b.percent < 0) {
      text = "--%";
    } else {
      text = `${b.low ? "LOW " : ""}${pct}%`;
    }
    const scale = 2;
    drawText(buf, idiv(FACE_WIDTH - faceTextWidth(text, scale), 2), 40, text, scale);
  }

  function drawTextLayer(t, buf) {
    let y = 0;
    for (let i = 0; i < FACE_TEXT_LINES; i += 1) {
      const scale = i === 0 && t.bigFirstLine ? 2 : 1;
      drawText(buf, 1, y, (t.lines[i] || "").slice(0, 21), scale);
      y += 8 * scale + (scale === 2 ? 1 : 0);
      if (y >= FACE_HEIGHT) break;
    }
  }

  // frame: { layer: "blank" | "eye" | "battery" | "text", eye, battery, text }
  function render(frame, buffer = createFaceBuffer()) {
    buffer.fill(0);
    if (frame.layer === "eye") drawEye({ glow: true, lidTop: 0, slant: 0, smile: 0, ...frame.eye }, buffer);
    else if (frame.layer === "battery") drawBattery({ percent: -1, low: false, ...frame.battery }, buffer);
    else if (frame.layer === "text") drawTextLayer({ lines: [], bigFirstLine: false, ...frame.text }, buffer);
    return buffer;
  }

  return { render, drawText };
}
