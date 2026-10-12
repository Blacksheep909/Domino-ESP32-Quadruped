#include "loop_diagnostics.h"

#ifndef DOMINO_SIL
#include <esp_attr.h>
#include <esp_system.h>
#include <string.h>
#include <type_traits>

namespace {
constexpr uint32_t kRtcMagic = 0x444C5047;  // DLPG
constexpr uint32_t kBlackboxMagic = 0x44424258;  // DBBX
constexpr uint32_t kSlowStageUs = 100000;
constexpr uint8_t kStageCount = 6;

struct RtcStageMarker {
  uint32_t magic;
  uint8_t stage;
  uint8_t inverseStage;
};

RTC_NOINIT_ATTR RtcStageMarker rtcStage;
struct RtcBlackbox {
  uint32_t magic;
  uint32_t inverseMagic;
  uint32_t nextSequence;
  uint32_t bootSequence;
  uint8_t count;
  uint8_t nextIndex;
  BlackboxEvent events[kBlackboxCapacity];
};
static_assert(std::is_trivially_default_constructible<RtcBlackbox>::value,
              "RTC blackbox must not be reinitialized by a C++ constructor");
RTC_NOINIT_ATTR RtcBlackbox rtcBlackbox;
LoopStage activeStage = LoopStage::None;
LoopDiagnosticsSnapshot counters{};

void markStage(LoopStage stage) {
  const uint8_t value = static_cast<uint8_t>(stage);
  rtcStage.magic = kRtcMagic;
  rtcStage.stage = value;
  rtcStage.inverseStage = static_cast<uint8_t>(~value);
}

uint32_t &maximumFor(LoopStage stage) {
  switch (stage) {
    case LoopStage::Crsf: return counters.maxCrsfUs;
    case LoopStage::Imu: return counters.maxImuUs;
    case LoopStage::Live: return counters.maxLiveUs;
    case LoopStage::Control: return counters.maxControlUs;
    case LoopStage::ServoWrite: return counters.maxServoWriteUs;
    default: return counters.maxControlUs;
  }
}
}  // namespace

void loopDiagnosticsInit() {
  const bool retained = rtcStage.magic == kRtcMagic &&
      rtcStage.stage < kStageCount &&
      rtcStage.inverseStage == static_cast<uint8_t>(~rtcStage.stage);
  counters = {};
  counters.priorResetStage = retained ? rtcStage.stage : 0;
  if (rtcBlackbox.magic != kBlackboxMagic ||
      rtcBlackbox.inverseMagic != ~kBlackboxMagic ||
      rtcBlackbox.count > kBlackboxCapacity ||
      rtcBlackbox.nextIndex >= kBlackboxCapacity) {
    memset(&rtcBlackbox, 0, sizeof(rtcBlackbox));
    rtcBlackbox.magic = kBlackboxMagic;
    rtcBlackbox.inverseMagic = ~kBlackboxMagic;
    rtcBlackbox.nextSequence = 1;
  }
  ++rtcBlackbox.bootSequence;
  activeStage = LoopStage::None;
  markStage(activeStage);
  loopBlackboxRecord(BlackboxEventKind::Boot, counters.priorResetStage,
                     static_cast<uint32_t>(esp_reset_reason()));
}

LoopDiagnosticsSnapshot loopDiagnosticsSnapshot() { return counters; }

void loopBlackboxRecord(BlackboxEventKind kind, uint8_t stage, uint32_t value,
                        uint16_t aux) {
  BlackboxEvent &event = rtcBlackbox.events[rtcBlackbox.nextIndex];
  event.sequence = rtcBlackbox.nextSequence++;
  event.bootSequence = rtcBlackbox.bootSequence;
  event.atMs = millis();
  event.value = value;
  event.aux = aux;
  event.kind = static_cast<uint8_t>(kind);
  event.stage = stage;
  rtcBlackbox.nextIndex = (rtcBlackbox.nextIndex + 1) % kBlackboxCapacity;
  if (rtcBlackbox.count < kBlackboxCapacity) ++rtcBlackbox.count;
}

BlackboxSnapshot loopBlackboxSnapshot() {
  BlackboxSnapshot result{};
  result.count = rtcBlackbox.count;
  const uint8_t first = (rtcBlackbox.nextIndex + kBlackboxCapacity -
                         rtcBlackbox.count) % kBlackboxCapacity;
  for (uint8_t index = 0; index < result.count; ++index) {
    result.events[index] = rtcBlackbox.events[(first + index) % kBlackboxCapacity];
  }
  return result;
}

ScopedLoopStage::ScopedLoopStage(LoopStage stage)
    : stage_(stage), parent_(activeStage), startedUs_(micros()) {
  activeStage = stage_;
  markStage(stage_);
}

ScopedLoopStage::~ScopedLoopStage() {
  const uint32_t durationUs = micros() - startedUs_;
  uint32_t &maximum = maximumFor(stage_);
  if (durationUs > maximum) {
    maximum = durationUs;
    if (stage_ == LoopStage::Crsf) counters.maxCrsfAtMs = millis();
    if (stage_ == LoopStage::Imu) counters.maxImuAtMs = millis();
  }
  if (durationUs >= kSlowStageUs) {
    ++counters.slowStageCount;
    counters.lastSlowStage = static_cast<uint8_t>(stage_);
    counters.lastSlowStageUs = durationUs;
    counters.lastSlowStageAtMs = millis();
    loopBlackboxRecord(BlackboxEventKind::SlowStage,
                       static_cast<uint8_t>(stage_), durationUs);
  }
  activeStage = parent_;
  markStage(parent_);
}
#endif
