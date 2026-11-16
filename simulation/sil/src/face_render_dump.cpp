// Renders a fixed set of face frames with the firmware renderer and prints
// them as JSON (parameters + hex buffer). live-face-protocol.test.mjs renders
// the same parameters with web/src/face-renderer.js and requires identical
// bytes. Build: see simulation/sil/test-face.ps1 or the Studio test.
#include <stdio.h>
#include <string.h>

#include "face_renderer.h"

static void dump(const char *name, const FaceFrame &f, bool &first) {
  uint8_t buf[kFaceBufferBytes];
  faceRender(f, buf);
  printf("%s{\"name\":\"%s\",\"layer\":%d,", first ? "" : ",\n", name, static_cast<int>(f.layer));
  first = false;
  printf("\"eye\":{\"cx\":%d,\"cy\":%d,\"w\":%d,\"h\":%d,\"r\":%d,\"lidTop\":%d,\"slant\":%d,\"smile\":%d,\"glow\":%s},",
         f.eye.cx, f.eye.cy, f.eye.w, f.eye.h, f.eye.r, f.eye.lidTop, f.eye.slant, f.eye.smile,
         f.eye.glow ? "true" : "false");
  printf("\"battery\":{\"percent\":%d,\"low\":%s},", f.battery.percent, f.battery.low ? "true" : "false");
  printf("\"text\":{\"bigFirstLine\":%s,\"lines\":[", f.text.bigFirstLine ? "true" : "false");
  for (int i = 0; i < kFaceTextLines; ++i) printf("%s\"%s\"", i ? "," : "", f.text.lines[i]);
  printf("]},\"hex\":\"");
  for (int i = 0; i < kFaceBufferBytes; ++i) printf("%02x", buf[i]);
  printf("\"}");
}

int main() {
  bool first = true;
  printf("[\n");
  FaceFrame f;
  f.layer = FaceLayer::Eye;
  dump("eye_default", f, first);
  f.eye.glow = false; f.eye.w = 52; f.eye.h = 30; f.eye.r = 8; f.eye.cx = 70; f.eye.cy = 30;
  dump("eye_wide_no_glow", f, first);
  f.eye = FaceEye{}; f.eye.lidTop = 9; f.eye.slant = 12;
  dump("eye_angry_left", f, first);
  f.eye = FaceEye{}; f.eye.lidTop = 12; f.eye.slant = -12;
  dump("eye_sad_left", f, first);
  f.eye = FaceEye{}; f.eye.smile = 18;
  dump("eye_happy", f, first);
  f.eye = FaceEye{}; f.eye.h = 3; f.eye.r = 1;
  dump("eye_asleep_line", f, first);
  f.eye = FaceEye{}; f.eye.cx = 120; f.eye.cy = 2; f.eye.w = 44; f.eye.h = 44; f.eye.r = 22;
  dump("eye_round_clipped", f, first);
  f = FaceFrame{}; f.layer = FaceLayer::Battery; f.battery.percent = 57;
  dump("battery_57", f, first);
  f.battery.percent = 8; f.battery.low = true;
  dump("battery_low_8", f, first);
  f.battery.percent = 100; f.battery.low = false;
  dump("battery_100", f, first);
  f.battery.percent = -1;
  dump("battery_unknown", f, first);
  f = FaceFrame{}; f.layer = FaceLayer::Text; f.text.bigFirstLine = true;
  strcpy(f.text.lines[0], "15.6V 72%");
  strcpy(f.text.lines[1], "LINK OK  LQ 98%");
  strcpy(f.text.lines[2], "MODE POLICY");
  strcpy(f.text.lines[3], "ROLL -1.2 PITCH 0.4");
  strcpy(f.text.lines[4], "abc: [x_y] #3 +5 (?)");
  dump("text_telemetry", f, first);
  printf("\n]\n");
  return 0;
}
