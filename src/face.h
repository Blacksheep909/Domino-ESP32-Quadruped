#pragma once

// OLED face: two 2.42" SSD1309 128x64 screens as Domino's eyes. FIRMWARE STEP
// "face & voice 3 of 6" (see docs/face-voice-firmware-steps.md).
//
// What the screens show is decided by FaceEngine (face_engine.*) and drawn by
// face_renderer; this module owns the settings (NVS record "face"), the
// battery estimate, and the screens themselves.
//
// Bus: a full frame is ~1 KB per screen, far too much for the shared 100 kHz
// servo bus. The screens share the IMU's own bus (Wire1, GPIO 32/33, 400 kHz)
// and are sent one 128-byte page at a time, only for pages that changed, by
// the IMU's background task on core 0 (faceServiceBus) between IMU reads. If
// the IMU is not on its own bus, the face starts its own core-0 task on Wire1.
//
// Off by default: build with -DDOMINO_FACE_ENABLED=1 once the screens are
// wired. Settings can still be read and saved from Studio without them.

#include <stdint.h>

#include "face_engine.h"

#ifndef DOMINO_FACE_ENABLED
#define DOMINO_FACE_ENABLED 0
#endif
#ifndef DOMINO_FACE_LEFT_ADDRESS
#define DOMINO_FACE_LEFT_ADDRESS 0x3C
#endif
#ifndef DOMINO_FACE_RIGHT_ADDRESS
#define DOMINO_FACE_RIGHT_ADDRESS 0x3D
#endif
// The IMU's own bus pins (src/imu.cpp DOMINO_IMU_SDA_PIN / SCL_PIN).
#ifndef DOMINO_FACE_SDA_PIN
#define DOMINO_FACE_SDA_PIN 32
#endif
#ifndef DOMINO_FACE_SCL_PIN
#define DOMINO_FACE_SCL_PIN 33
#endif
// Rotate a screen 180 degrees if it is mounted upside down.
#ifndef DOMINO_FACE_LEFT_FLIP
#define DOMINO_FACE_LEFT_FLIP 0
#endif
#ifndef DOMINO_FACE_RIGHT_FLIP
#define DOMINO_FACE_RIGHT_FLIP 0
#endif

void faceBegin();  // after imuInit(): loads settings, starts the screens
// Main loop, any rate: renders at most every 40 ms (25 fps).
void faceUpdate(uint32_t nowMs, const FaceInputs &inputs);
// Send at most one changed page. Called from the IMU's own-bus task.
void faceServiceBus();

const FaceSettings &faceSettings();
bool setFaceSettings(const FaceSettings &settings, bool persist, bool *persisted);
bool facePersisted();
void faceShowBattery(uint32_t nowMs, uint32_t ms);
bool faceScreenOnline(int screen);

// 4S LiPo estimate from the pack voltage (same curve as Studio), smoothed so
// servo load sag does not make it jump. -1 while unknown.
int8_t faceBatteryPercent(uint32_t nowMs);
// Pure: rested-cell voltage -> percent (Studio live-battery-state.js curve).
int faceLipoPercentFromCell(float cellVoltageV);
