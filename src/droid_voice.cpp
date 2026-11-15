#include "droid_voice.h"

#if DOMINO_AUDIO_ENABLED && !defined(DOMINO_SIL)

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include <string.h>

// MAX98357A wiring (I2S, not I2C):
//   BCLK -> GPIO 26, LRC -> GPIO 25, DIN -> GPIO 27, VIN -> 5 V, GND -> GND.
// Leave SD and GAIN unconnected (mono mix of both channels, 9 dB gain).
// Move the pins with -DDOMINO_AUDIO_BCLK_PIN / _LRC_PIN / _DIN_PIN if PCB
// V1.1B routes any of these GPIOs elsewhere.
#ifndef DOMINO_AUDIO_BCLK_PIN
#define DOMINO_AUDIO_BCLK_PIN 26
#endif
#ifndef DOMINO_AUDIO_LRC_PIN
#define DOMINO_AUDIO_LRC_PIN 25
#endif
#ifndef DOMINO_AUDIO_DIN_PIN
#define DOMINO_AUDIO_DIN_PIN 27
#endif
#ifndef DOMINO_AUDIO_DEFAULT_VOLUME
#define DOMINO_AUDIO_DEFAULT_VOLUME 50
#endif

namespace {

constexpr i2s_port_t kPort = I2S_NUM_0;
constexpr size_t kChunkFrames = 256;
constexpr float kTwoPi = 6.28318530718f;

QueueHandle_t emoteQueue = nullptr;
volatile uint8_t volumePercent = DOMINO_AUDIO_DEFAULT_VOLUME;
volatile bool playing = false;

// Effects state. The echo ring holds up to 200 ms of the distorted signal.
constexpr size_t kEchoFrames = kDroidSampleRate / 5;
float echoRing[kEchoFrames];
size_t echoHead = 0;
float lowpassState = 0.0f;

int16_t chunk[kChunkFrames * 2];
size_t chunkFill = 0;

void flushChunk() {
  if (chunkFill == 0) return;
  size_t written = 0;
  i2s_write(kPort, chunk, chunkFill * 2 * sizeof(int16_t), &written, portMAX_DELAY);
  chunkFill = 0;
}

void pushSample(int16_t value) {
  chunk[chunkFill * 2] = value;
  chunk[chunkFill * 2 + 1] = value;
  if (++chunkFill == kChunkFrames) flushChunk();
}

// Soft clipper (a cheap tanh), identical to saturate() in sounds/droid-synth.mjs.
float saturate(float x) {
  if (x > 3.0f) return 1.0f;
  if (x < -3.0f) return -1.0f;
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

// One dry sample (-1..1) through the effects chain and out to the amplifier.
// Same maths as applyFx() in sounds/droid-synth.mjs.
void pushVoice(float dry, float fullScale) {
  static const size_t d1 = min<size_t>(kEchoFrames - 1, kDroidFxEchoMs * kDroidSampleRate / 1000);
  static const size_t d2 = min<size_t>(kEchoFrames - 1, kDroidFxEcho2Ms * kDroidSampleRate / 1000);
  static const float norm = 1.0f / saturate(kDroidFxDrive);
  static const float mix = 1.0f / (1.0f + kDroidFxEchoGain + kDroidFxEcho2Gain);
  static const float lp = kDroidFxLowpassHz > 0
      ? 1.0f - expf(-kTwoPi * kDroidFxLowpassHz / kDroidSampleRate) : 1.0f;
  const float sat = saturate(dry * kDroidFxDrive) * norm;
  echoRing[echoHead] = sat;
  float v = sat;
  if (d1 > 0) v += kDroidFxEchoGain * echoRing[(echoHead + kEchoFrames - d1) % kEchoFrames];
  if (d2 > 0) v += kDroidFxEcho2Gain * echoRing[(echoHead + kEchoFrames - d2) % kEchoFrames];
  echoHead = (echoHead + 1) % kEchoFrames;
  lowpassState += lp * (v * mix - lowpassState);
  pushSample(static_cast<int16_t>(lowpassState * fullScale));
}

void pushSilence(uint32_t frames, float fullScale) {
  for (uint32_t i = 0; i < frames; i++) pushVoice(0.0f, fullScale);
}

// Must stay identical to hopRandom() in sounds/droid-synth.mjs.
float hopRandom(uint32_t seed, uint32_t index) {
  uint32_t x = seed + index * 2654435761u;
  x ^= x >> 15;
  x *= 2246822519u;
  x ^= x >> 13;
  return static_cast<float>(x) / 4294967296.0f;
}

uint32_t msToFrames(uint16_t ms) {
  return (static_cast<uint32_t>(ms) * kDroidSampleRate + 500) / 1000;
}

// Same maths as renderEmote() in sounds/droid-synth.mjs, so the preview page
// and the robot sound alike.
void renderEmote(const DroidEmoteDef &emote) {
  // Squared so the slider feels even; 0.8 leaves headroom, as the WAVs do.
  const float volume = volumePercent / 100.0f;
  const float fullScale = volume * volume * 0.8f * 32767.0f;
  float phase = 0.0f;
  float subPhase = 0.0f;

  for (uint8_t k = 0; k < emote.count; k++) {
    const DroidSegment &seg = emote.segments[k];
    const uint32_t n = max<uint32_t>(1, msToFrames(seg.ms));
    const uint32_t stepFrames = max<uint32_t>(1, msToFrames(seg.stepMs));
    const float f0 = seg.f0;
    const float f1 = seg.f1;
    const float amp = seg.amp / 255.0f;
    const float vibDepth = seg.vibDepthPermille / 1000.0f;
    const float vibStep = kTwoPi * seg.vibHz / kDroidSampleRate;
    const float fmRatio = seg.fmRatioCenti / 100.0f;
    const float fmIndex = seg.fmIndexCenti / 100.0f;
    const float sub = seg.sub / 255.0f;
    const float subRatio = seg.subRatioCenti / 100.0f;
    const uint32_t attack = max<uint32_t>(1, msToFrames(seg.attackMs));
    const uint32_t release = max<uint32_t>(1, msToFrames(seg.releaseMs));
    // An exponential glide multiplies by the same factor every sample.
    const float glideRatio = (seg.expCurve && f0 > 0.0f) ? powf(f1 / f0, 1.0f / n) : 1.0f;
    float glide = f0;
    float hopFreq = f0;

    for (uint32_t i = 0; i < n; i++) {
      float f;
      if (seg.wave == DROID_WAVE_HOP) {
        if (i % stepFrames == 0) hopFreq = f0 + (f1 - f0) * hopRandom(seg.seed, i / stepFrames);
        f = hopFreq;
      } else if (seg.expCurve) {
        f = glide;
        glide *= glideRatio;
      } else {
        f = f0 + (f1 - f0) * (static_cast<float>(i) / n);
      }
      if (seg.vibHz > 0) f *= 1.0f + vibDepth * sinf(vibStep * i);

      phase += f / kDroidSampleRate;
      phase -= floorf(phase);

      float v;
      switch (seg.wave) {
        case DROID_WAVE_TRI: v = 4.0f * fabsf(phase - 0.5f) - 1.0f; break;
        case DROID_WAVE_SQ: v = phase < 0.5f ? 0.6f : -0.6f; break;
        case DROID_WAVE_FM: v = sinf(kTwoPi * phase + fmIndex * sinf(kTwoPi * phase * fmRatio)); break;
        case DROID_WAVE_VOX: {
          // A voice: once per pitch period, a short burst ringing at the two vowel resonances.
          const float u = static_cast<float>(i) / n;
          const float fa = seg.formantA0 + (seg.formantA1 - seg.formantA0) * u;
          const float fb = seg.formantB0 + (seg.formantB1 - seg.formantB0) * u;
          const float window = 0.5f - 0.5f * cosf(kTwoPi * phase);
          v = 2.0f * window * (0.65f * sinf(kTwoPi * phase * fa / f) + 0.35f * sinf(kTwoPi * phase * fb / f));
          break;
        }
        default: v = sinf(kTwoPi * phase); break;
      }

      // Second voice: a sine at subRatio times the pitch, mixed in and renormalised.
      subPhase += f * subRatio / kDroidSampleRate;
      subPhase -= floorf(subPhase);
      if (seg.sub > 0) v = (v + sub * sinf(kTwoPi * subPhase)) / (1.0f + sub);

      float env = 1.0f;
      env = min(env, static_cast<float>(i + 1) / attack);
      env = min(env, static_cast<float>(n - i) / release);
      pushVoice(v * env * amp, fullScale);
    }
    pushSilence(msToFrames(seg.gapMs), fullScale);
  }
  // Let the echoes ring out, then leave the effects state clean for the next emote.
  pushSilence(msToFrames(max<uint16_t>(kDroidFxEchoMs, kDroidFxEcho2Ms) + 40), fullScale);
  memset(echoRing, 0, sizeof(echoRing));
  lowpassState = 0.0f;
  flushChunk();
}

void voiceTask(void *) {
  for (;;) {
    uint8_t id = 0;
    if (xQueueReceive(emoteQueue, &id, portMAX_DELAY) != pdTRUE) continue;
    if (id >= DROID_EMOTE_COUNT) continue;
    playing = true;
    renderEmote(kDroidEmotes[id]);
    i2s_zero_dma_buffer(kPort);
    playing = uxQueueMessagesWaiting(emoteQueue) > 0;
  }
}

}  // namespace

void droidVoiceBegin() {
  if (emoteQueue) return;

  i2s_config_t config = {};
  config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
  config.sample_rate = kDroidSampleRate;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.dma_buf_count = 6;
  config.dma_buf_len = kChunkFrames;
  config.tx_desc_auto_clear = true;  // silence, not a stuck tone, on underrun

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = DOMINO_AUDIO_BCLK_PIN;
  pins.ws_io_num = DOMINO_AUDIO_LRC_PIN;
  pins.data_out_num = DOMINO_AUDIO_DIN_PIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  if (i2s_driver_install(kPort, &config, 0, nullptr) != ESP_OK || i2s_set_pin(kPort, &pins) != ESP_OK) {
    Serial.println("Droid voice: I2S setup failed, audio disabled.");
    return;
  }
  i2s_zero_dma_buffer(kPort);

  emoteQueue = xQueueCreate(4, sizeof(uint8_t));
  // Core 0, lowest useful priority: the control loop runs on core 1.
  xTaskCreatePinnedToCore(voiceTask, "droidVoice", 4096, nullptr, 1, nullptr, 0);
  Serial.printf("Droid voice: ready on I2S (BCLK %d, LRC %d, DIN %d), %u emotes, volume %u%%.\n",
                DOMINO_AUDIO_BCLK_PIN, DOMINO_AUDIO_LRC_PIN, DOMINO_AUDIO_DIN_PIN,
                static_cast<unsigned>(DROID_EMOTE_COUNT), static_cast<unsigned>(volumePercent));
}

bool droidVoicePlay(DroidEmote emote) {
  if (!emoteQueue || emote >= DROID_EMOTE_COUNT) return false;
  const uint8_t id = emote;
  return xQueueSend(emoteQueue, &id, 0) == pdTRUE;
}

bool droidVoicePlayByName(const char *name) {
  if (!name) return false;
  for (uint8_t i = 0; i < DROID_EMOTE_COUNT; i++) {
    if (!strcmp(kDroidEmotes[i].name, name)) return droidVoicePlay(static_cast<DroidEmote>(i));
  }
  return false;
}

void droidVoiceSetVolume(uint8_t percent) { volumePercent = percent > 100 ? 100 : percent; }

uint8_t droidVoiceVolume() { return volumePercent; }

bool droidVoiceBusy() { return playing; }

#endif
