#pragma once

// Face behaviour for the two OLED eyes (FIRMWARE STEP "face & voice 3 of 6").
// Pure logic, no hardware: given the robot's state each frame it decides what
// each screen shows (an eye, the battery, telemetry text or nothing) and hands
// FaceFrames to face_renderer. src/face.cpp drives the real screens; Studio's
// SCREENS tab runs a port of this engine (web/src/face-engine.js) for its live
// preview.
//
// Screens are indexed from the robot's point of view: 0 = left eye, 1 = right
// eye. Pixel x on each screen increases toward the robot's left (the viewer's
// right when facing it).

#include <stddef.h>
#include <stdint.h>

#include "face_renderer.h"

enum class FaceMode : uint8_t { Eyes = 0, Telemetry = 1, Off = 2 };
enum class FaceStyle : uint8_t { Burne = 0, Round = 1, Tall = 2, Wide = 3 };
constexpr uint8_t kFaceStyleCount = 4;

struct FaceSettings {
  FaceMode mode = FaceMode::Eyes;
  FaceStyle style = FaceStyle::Burne;
  bool glow = true;
  uint8_t brightness = 70;          // 0..100
  uint8_t batteryEye = 1;           // screen that shows battery alerts: 0 left, 1 right
  uint8_t lowBatteryPercent = 20;   // flash a warning at or below this (0 = never)
  uint8_t reminderMinutes = 5;      // show the battery for 5 s when idle, at most this often (0 = never)
  bool blink = true;
  bool sleepWhenLying = true;       // eyes narrow to slim lines when the robot lies down
  bool followSticks = true;         // eyes look where the sticks steer
  bool emoteEyes = true;            // eyes play the emote tracks
};

FaceSettings defaultFaceSettings();
bool validateFaceSettings(const FaceSettings &settings);
const char *faceStyleName(FaceStyle style);  // burne, round, tall, wide
int faceStyleIndex(const char *name);        // -1 if unknown

constexpr size_t kFaceRecordSize = 16;
size_t encodeFaceRecord(const FaceSettings &settings, uint8_t *out, size_t capacity);
bool decodeFaceRecord(const uint8_t *bytes, size_t length, FaceSettings *settings);

enum class FaceReaction : uint8_t { None = 0, Pushed = 1, Fallen = 2 };

struct FaceInputs {
  uint32_t nowMs = 0;
  bool lying = false;          // stowed / lying down
  bool sticksIdle = true;      // no stick input
  float turn = 0.0f;           // -1..1, + = turning to the robot's left
  float strafe = 0.0f;         // -1..1, + = to the robot's left
  float forward = 0.0f;        // -1..1
  bool tilt = false;           // tilt mode: eyes follow the body pose below
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  int8_t batteryPercent = -1;  // -1 unknown
  int8_t emote = -1;           // personality emote playing, -1 none
  uint32_t emoteElapsedMs = 0;
  FaceReaction reaction = FaceReaction::None;  // event this frame
  // Telemetry mode text for each screen.
  const char (*telemetry[2])[kFaceTextChars + 1] = {nullptr, nullptr};
};

// Pose of one eye before it is turned into pixels.
struct FaceEyePose {
  float open = 1.0f;   // 0..1 height
  float lookX = 0.0f;  // px
  float lookY = 0.0f;  // px
  float scale = 1.0f;
  float smile = 0.0f;  // 0..1
  float mood = 0.0f;   // -1 sad .. +1 angry
};

class FaceEngine {
 public:
  explicit FaceEngine(uint32_t seed = 1);
  void setSettings(const FaceSettings &settings) { settings_ = settings; }
  const FaceSettings &settings() const { return settings_; }
  // Advance to inputs.nowMs and fill both screens.
  void step(const FaceInputs &inputs, FaceFrame out[2]);
  // Show the battery on the battery eye for `ms` (Studio "show battery now").
  void showBatteryFor(uint32_t nowMs, uint32_t ms);
  FaceEyePose eyePose(int eye) const { return pose_[eye]; }

 private:
  uint32_t random();
  float randomRange(float lo, float hi);
  void eyeFrame(int eye, FaceFrame &frame) const;
  void batteryOverlay(const FaceInputs &in, FaceFrame out[2]);

  FaceSettings settings_;
  uint32_t rng_;
  bool started_ = false;
  uint32_t lastMs_ = 0;
  float sleep_ = 0.0f;  // 0 awake .. 1 asleep
  uint32_t nextBlinkMs_ = 0;
  uint32_t blinkStartMs_ = 0;
  bool blinking_ = false;
  bool doubleBlink_ = false;
  uint32_t nextGlanceMs_ = 0;
  float glanceX_ = 0.0f, glanceY_ = 0.0f;
  float lookX_ = 0.0f, lookY_ = 0.0f;
  uint32_t reactionStartMs_ = 0;
  FaceReaction reaction_ = FaceReaction::None;
  uint32_t idleSinceMs_ = 0;
  bool wasIdle_ = false;
  uint32_t lastReminderMs_ = 0;
  bool reminderShownThisIdle_ = false;
  bool remindedOnce_ = false;
  uint32_t batteryShowUntilMs_ = 0;
  FaceEyePose pose_[2];
};
