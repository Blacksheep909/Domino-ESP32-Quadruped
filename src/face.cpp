#include "face.h"

#include <math.h>
#include <string.h>

#ifndef DOMINO_FACE_PURE
#include <Arduino.h>
#include <Wire.h>

#include "imu.h"
#include "power_monitor.h"
#ifndef DOMINO_SIL
#include <Preferences.h>
#endif
#endif

namespace {

// Rested 4S LiPo curve, per cell (copy of Studio's LIPO_CHARGE_CURVE).
constexpr float kLipoCurve[][2] = {
    {3.30f, 0}, {3.50f, 5}, {3.60f, 10}, {3.70f, 20}, {3.75f, 30}, {3.80f, 40},
    {3.85f, 50}, {3.90f, 60}, {3.95f, 70}, {4.00f, 80}, {4.10f, 90}, {4.20f, 100},
};
constexpr int kCellCount = 4;

}  // namespace

int faceLipoPercentFromCell(float v) {
  if (!(v > 0.0f)) return -1;
  const int n = sizeof(kLipoCurve) / sizeof(kLipoCurve[0]);
  if (v <= kLipoCurve[0][0]) return 0;
  if (v >= kLipoCurve[n - 1][0]) return 100;
  for (int i = 1; i < n; ++i) {
    if (v > kLipoCurve[i][0]) continue;
    const float f = (v - kLipoCurve[i - 1][0]) / (kLipoCurve[i][0] - kLipoCurve[i - 1][0]);
    return static_cast<int>(lroundf(kLipoCurve[i - 1][1] + f * (kLipoCurve[i][1] - kLipoCurve[i - 1][1])));
  }
  return 100;
}

#ifndef DOMINO_FACE_PURE

namespace {

FaceSettings gSettings;
bool gPersisted = false;
FaceEngine gEngine(0xD0A1u);
uint32_t gLastRenderMs = 0;
float gBatteryPercent = -1.0f;
uint32_t gBatteryAtMs = 0;

#if DOMINO_FACE_ENABLED && !defined(DOMINO_SIL)
// Frames shared between the main loop (core 1) and the bus service (core 0).
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t gFront[2][kFaceBufferBytes];
uint8_t gDirty[2] = {0xFF, 0xFF};  // one bit per 8-row page
bool gScreenOff = false;           // FaceMode::Off: display off
bool gScreenOffApplied[2] = {false, false};
uint8_t gContrast = 0;
bool gContrastPending[2] = {true, true};
bool gOnline[2] = {false, false};
uint8_t gErrors[2] = {0, 0};
uint32_t gRetryAtMs[2] = {0, 0};
bool gOwnTask = false;
constexpr uint8_t kAddress[2] = {DOMINO_FACE_LEFT_ADDRESS, DOMINO_FACE_RIGHT_ADDRESS};
constexpr bool kFlip[2] = {DOMINO_FACE_LEFT_FLIP != 0, DOMINO_FACE_RIGHT_FLIP != 0};

bool sendCommands(int s, const uint8_t *cmds, size_t n) {
  Wire1.beginTransmission(kAddress[s]);
  Wire1.write(0x00);  // control byte: command stream
  for (size_t i = 0; i < n; ++i) Wire1.write(cmds[i]);
  return Wire1.endTransmission() == 0;
}

bool initScreen(int s) {
  // SSD1309 power-on sequence (also fine for SSD1306 modules).
  const uint8_t remap = kFlip[s] ? 0xA0 : 0xA1;
  const uint8_t scan = kFlip[s] ? 0xC0 : 0xC8;
  const uint8_t cmds[] = {0xAE, 0xD5, 0xA0, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x20, 0x00, remap, scan,
                          0xDA, 0x12, 0x81, gContrast, 0xD9, 0x22, 0xDB, 0x34, 0xA4, 0xA6, 0xAF};
  return sendCommands(s, cmds, sizeof(cmds));
}

bool sendPage(int s, int page, const uint8_t *data) {
  const uint8_t addr[] = {0x21, 0, 127, 0x22, static_cast<uint8_t>(page), static_cast<uint8_t>(page)};
  if (!sendCommands(s, addr, sizeof(addr))) return false;
  for (int off = 0; off < kFaceWidth; off += 32) {  // small chunks: Wire buffer and IMU latency
    Wire1.beginTransmission(kAddress[s]);
    Wire1.write(0x40);  // control byte: data
    Wire1.write(data + off, 32);
    if (Wire1.endTransmission() != 0) return false;
  }
  return true;
}

void markOffline(int s, uint32_t now) {
  gOnline[s] = false;
  gRetryAtMs[s] = now + 5000;
  portENTER_CRITICAL(&gMux);
  gDirty[s] = 0xFF;
  gContrastPending[s] = true;
  gScreenOffApplied[s] = false;
  portEXIT_CRITICAL(&gMux);
}

void ownTask(void *) {
  for (;;) {
    faceServiceBus();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
#endif

bool loadSettings(FaceSettings *out) {
#ifndef DOMINO_SIL
  Preferences prefs;
  if (!prefs.begin("face", true)) return false;
  uint8_t buf[kFaceRecordSize];
  const size_t n = prefs.getBytes("v1", buf, sizeof(buf));
  prefs.end();
  return n == sizeof(buf) && decodeFaceRecord(buf, n, out);
#else
  (void)out;
  return false;
#endif
}

bool storeSettings(const FaceSettings &s) {
#ifndef DOMINO_SIL
  uint8_t buf[kFaceRecordSize];
  if (!encodeFaceRecord(s, buf, sizeof(buf))) return false;
  Preferences prefs;
  if (!prefs.begin("face", false)) return false;
  const bool wrote = prefs.putBytes("v1", buf, sizeof(buf)) == sizeof(buf);
  prefs.end();
  uint8_t check[kFaceRecordSize];
  Preferences reader;
  if (!reader.begin("face", true)) return false;
  const size_t n = reader.getBytes("v1", check, sizeof(check));
  reader.end();
  return wrote && n == sizeof(check) && memcmp(buf, check, sizeof(buf)) == 0;
#else
  (void)s;
  return false;
#endif
}

void applySettings() {
  gEngine.setSettings(gSettings);
#if DOMINO_FACE_ENABLED && !defined(DOMINO_SIL)
  portENTER_CRITICAL(&gMux);
  gContrast = static_cast<uint8_t>(lroundf(gSettings.brightness * 2.55f));
  gContrastPending[0] = gContrastPending[1] = true;
  gScreenOff = gSettings.mode == FaceMode::Off;
  gScreenOffApplied[0] = gScreenOffApplied[1] = false;
  portEXIT_CRITICAL(&gMux);
#endif
}

}  // namespace

const FaceSettings &faceSettings() { return gSettings; }
bool facePersisted() { return gPersisted; }

bool setFaceSettings(const FaceSettings &s, bool persist, bool *persisted) {
  if (!validateFaceSettings(s)) return false;
  gSettings = s;
  applySettings();
  bool ok = true;
  if (persist) {
    ok = storeSettings(s);
    gPersisted = ok;
  }
  if (persisted) *persisted = persist && ok;
  return !persist || ok;
}

void faceShowBattery(uint32_t nowMs, uint32_t ms) { gEngine.showBatteryFor(nowMs, ms); }

int8_t faceBatteryPercent(uint32_t now) {
#ifdef DOMINO_SIL
  (void)now;
  return -1;  // the simulator has no battery
#else
  const PowerMonitorSample p = powerMonitorSample(now);
  if (!p.voltageValid || !(p.voltageV > 1.0f)) return gBatteryPercent < 0 ? -1 : static_cast<int8_t>(lroundf(gBatteryPercent));
  const int raw = faceLipoPercentFromCell(p.voltageV / kCellCount);
  if (gBatteryPercent < 0) {
    gBatteryPercent = static_cast<float>(raw);
  } else {
    // ~20 s time constant: servo load sag should not make the eye jump.
    const float dt = static_cast<float>(now - gBatteryAtMs) / 1000.0f;
    const float k = 1.0f - expf(-dt / 20.0f);
    gBatteryPercent += (raw - gBatteryPercent) * k;
  }
  gBatteryAtMs = now;
  return static_cast<int8_t>(lroundf(gBatteryPercent));
#endif
}

bool faceScreenOnline(int screen) {
#if DOMINO_FACE_ENABLED && !defined(DOMINO_SIL)
  return screen >= 0 && screen < 2 && gOnline[screen];
#else
  (void)screen;
  return false;
#endif
}

void faceBegin() {
  FaceSettings loaded;
  gPersisted = loadSettings(&loaded);
  gSettings = gPersisted ? loaded : defaultFaceSettings();
  applySettings();
#if DOMINO_FACE_ENABLED && !defined(DOMINO_SIL)
  memset(gFront, 0, sizeof(gFront));
  if (!imuOwnBus()) {
    // No IMU on Wire1: the face gets the bus (same pins) and its own task.
    Wire1.begin(DOMINO_FACE_SDA_PIN, DOMINO_FACE_SCL_PIN, 400000);
    Wire1.setTimeOut(5);
    gOwnTask = true;
    xTaskCreatePinnedToCore(ownTask, "face", 3072, nullptr, 1, nullptr, 0);
  }
#endif
}

void faceUpdate(uint32_t now, const FaceInputs &inputs) {
#if DOMINO_FACE_ENABLED && !defined(DOMINO_SIL)
  if (now - gLastRenderMs < 40) return;
  gLastRenderMs = now;
  FaceFrame frames[2];
  gEngine.step(inputs, frames);
  static uint8_t buf[kFaceBufferBytes];
  for (int s = 0; s < 2; ++s) {
    faceRender(frames[s], buf);
    portENTER_CRITICAL(&gMux);
    for (int p = 0; p < 8; ++p) {
      uint8_t *front = gFront[s] + p * kFaceWidth;
      if (memcmp(front, buf + p * kFaceWidth, kFaceWidth) != 0) {
        memcpy(front, buf + p * kFaceWidth, kFaceWidth);
        gDirty[s] |= static_cast<uint8_t>(1u << p);
      }
    }
    portEXIT_CRITICAL(&gMux);
  }
#else
  (void)now;
  (void)inputs;
#endif
}

void faceServiceBus() {
#if DOMINO_FACE_ENABLED && !defined(DOMINO_SIL)
  const uint32_t now = millis();
  static int first = 0;  // alternate which eye goes first, so both stay smooth
  first ^= 1;
  for (int i = 0; i < 2; ++i) {
    const int s = (first + i) & 1;
    if (!gOnline[s]) {
      if (now < gRetryAtMs[s]) continue;
      if (initScreen(s)) {
        gOnline[s] = true;
        gErrors[s] = 0;
        portENTER_CRITICAL(&gMux);
        gDirty[s] = 0xFF;
        gContrastPending[s] = false;
        portEXIT_CRITICAL(&gMux);
      } else {
        markOffline(s, now);
      }
      return;  // at most one bus job per call
    }
    bool off, offApplied, contrastPending;
    uint8_t contrast;
    portENTER_CRITICAL(&gMux);
    off = gScreenOff;
    offApplied = gScreenOffApplied[s];
    contrastPending = gContrastPending[s];
    contrast = gContrast;
    portEXIT_CRITICAL(&gMux);
    if (off != offApplied || contrastPending) {
      const uint8_t cmds[] = {static_cast<uint8_t>(off ? 0xAE : 0xAF), 0x81, contrast};
      if (sendCommands(s, cmds, sizeof(cmds))) {
        portENTER_CRITICAL(&gMux);
        gScreenOffApplied[s] = off;
        gContrastPending[s] = false;
        portEXIT_CRITICAL(&gMux);
      } else if (++gErrors[s] > 10) {
        markOffline(s, now);
      }
      return;
    }
    if (off) continue;
    int page = -1;
    uint8_t data[kFaceWidth];
    portENTER_CRITICAL(&gMux);
    for (int p = 0; p < 8; ++p) {
      if (gDirty[s] & (1u << p)) {
        page = p;
        memcpy(data, gFront[s] + p * kFaceWidth, kFaceWidth);
        gDirty[s] &= static_cast<uint8_t>(~(1u << p));
        break;
      }
    }
    portEXIT_CRITICAL(&gMux);
    if (page < 0) continue;
    if (sendPage(s, page, data)) {
      gErrors[s] = 0;
    } else {
      portENTER_CRITICAL(&gMux);
      gDirty[s] |= static_cast<uint8_t>(1u << page);
      portEXIT_CRITICAL(&gMux);
      if (++gErrors[s] > 10) markOffline(s, now);
    }
    return;
  }
#endif
}

#endif  // DOMINO_FACE_PURE
