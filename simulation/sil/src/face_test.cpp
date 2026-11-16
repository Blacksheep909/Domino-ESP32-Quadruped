// Native checks for the OLED face: settings record and the engine's behaviour.
// Run: simulation/sil/test-face.ps1
#include <stdio.h>
#include <string.h>

#include "face_engine.h"

static int failures = 0;
#define CHECK(cond)                                         \
  do {                                                      \
    if (!(cond)) {                                          \
      printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); \
      ++failures;                                           \
    }                                                       \
  } while (0)

static void run(FaceEngine &f, FaceInputs &in, uint32_t untilMs, FaceFrame out[2]) {
  while (in.nowMs < untilMs) {
    in.nowMs += 20;
    f.step(in, out);
  }
}

int main() {
  // Settings record.
  FaceSettings s = defaultFaceSettings();
  CHECK(validateFaceSettings(s) && s.mode == FaceMode::Eyes && s.style == FaceStyle::Burne);
  s.mode = FaceMode::Telemetry; s.style = FaceStyle::Wide; s.glow = false; s.brightness = 33;
  s.batteryEye = 0; s.lowBatteryPercent = 15; s.reminderMinutes = 0; s.blink = false; s.emoteEyes = false;
  uint8_t rec[kFaceRecordSize];
  CHECK(encodeFaceRecord(s, rec, sizeof(rec)) == kFaceRecordSize);
  FaceSettings back;
  CHECK(decodeFaceRecord(rec, sizeof(rec), &back));
  CHECK(back.mode == FaceMode::Telemetry && back.style == FaceStyle::Wide && !back.glow && back.brightness == 33 &&
        back.batteryEye == 0 && back.lowBatteryPercent == 15 && back.reminderMinutes == 0 && !back.blink &&
        back.sleepWhenLying && back.followSticks && !back.emoteEyes);
  rec[7] ^= 1;
  CHECK(!decodeFaceRecord(rec, sizeof(rec), &back));
  CHECK(faceStyleIndex("burne") == 0 && faceStyleIndex("wide") == 3 && faceStyleIndex("x") == -1);

  FaceFrame out[2];
  // Standing, awake: two open eyes.
  {
    FaceEngine f(7);
    FaceInputs in;
    in.batteryPercent = 80;
    run(f, in, 1000, out);
    CHECK(out[0].layer == FaceLayer::Eye && out[1].layer == FaceLayer::Eye);
    CHECK(out[0].eye.h >= 30);
    // Lying down: eyes narrow to slim lines and stay that way.
    in.lying = true;
    run(f, in, 4000, out);
    CHECK(out[0].eye.h <= 5 && out[1].eye.h <= 5);
    // Standing up: eyes open again.
    in.lying = false;
    run(f, in, 6000, out);
    CHECK(out[0].eye.h >= 25);
  }
  // Steering: eyes look toward the turn.
  {
    FaceEngine f(3);
    FaceInputs in;
    in.sticksIdle = false;
    in.turn = 1.0f;
    run(f, in, 1500, out);
    CHECK(out[0].eye.cx > 64 + 10 && out[1].eye.cx > 64 + 10);
  }
  // Low battery: flashes on the battery eye only (right by default).
  {
    FaceEngine f(5);
    FaceInputs in;
    in.batteryPercent = 12;
    int batteryFrames = 0, leftBattery = 0, eyeFramesOnRight = 0;
    for (int i = 0; i < 300; ++i) {  // 6 s from t = 0: inside the flash window
      in.nowMs += 20;
      f.step(in, out);
      if (out[1].layer == FaceLayer::Battery) {
        ++batteryFrames;
        CHECK(out[1].battery.low && out[1].battery.percent == 12);
      } else {
        ++eyeFramesOnRight;
      }
      if (out[0].layer == FaceLayer::Battery) ++leftBattery;
    }
    CHECK(batteryFrames > 100 && eyeFramesOnRight > 100 && leftBattery == 0);  // flashing, not solid
  }
  // Idle reminder: after 15 s idle the battery shows for 5 s, then eyes again.
  {
    FaceEngine f(9);
    FaceInputs in;
    in.batteryPercent = 64;
    run(f, in, 14000, out);
    CHECK(out[1].layer == FaceLayer::Eye);
    run(f, in, 16000, out);
    CHECK(out[1].layer == FaceLayer::Battery && !out[1].battery.low && out[1].battery.percent == 64);
    run(f, in, 21500, out);
    CHECK(out[1].layer == FaceLayer::Eye);
    // Not again within reminderMinutes, even if idle continues.
    run(f, in, 60000, out);
    CHECK(out[1].layer == FaceLayer::Eye);
  }
  // Emote: happy raises the lower lids (^ ^).
  {
    FaceEngine f(11);
    FaceSettings st = defaultFaceSettings();
    st.blink = false;
    f.setSettings(st);
    FaceInputs in;
    in.emote = 1;  // happy
    for (uint32_t t = 0; t <= 1000; t += 20) {
      in.nowMs = t + 20;
      in.emoteElapsedMs = t;
      f.step(in, out);
    }
    CHECK(out[0].eye.smile > 8 && out[1].eye.smile > 8);
  }
  // Telemetry and off modes.
  {
    FaceEngine f(1);
    FaceSettings st = defaultFaceSettings();
    st.mode = FaceMode::Telemetry;
    f.setSettings(st);
    static const char left[kFaceTextLines][kFaceTextChars + 1] = {"15.6V 72%", "LINK OK"};
    static const char right[kFaceTextLines][kFaceTextChars + 1] = {"POLICY", "ROLL 1.0"};
    FaceInputs in;
    in.telemetry[0] = left;
    in.telemetry[1] = right;
    run(f, in, 100, out);
    CHECK(out[0].layer == FaceLayer::Text && !strcmp(out[0].text.lines[0], "15.6V 72%"));
    CHECK(out[1].layer == FaceLayer::Text && !strcmp(out[1].text.lines[1], "ROLL 1.0"));
    st.mode = FaceMode::Off;
    f.setSettings(st);
    run(f, in, 200, out);
    CHECK(out[0].layer == FaceLayer::Blank && out[1].layer == FaceLayer::Blank);
  }

  if (failures) {
    printf("%d face check(s) failed\n", failures);
    return 1;
  }
  printf("face ok\n");
  return 0;
}
