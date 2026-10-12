#pragma once

#include <Arduino.h>

enum class LoopStage : uint8_t {
  None = 0,
  Crsf = 1,
  Imu = 2,
  Live = 3,
  Control = 4,
  ServoWrite = 5,
};

struct LoopDiagnosticsSnapshot {
  uint8_t priorResetStage = 0;
  uint8_t lastSlowStage = 0;
  uint32_t lastSlowStageUs = 0;
  uint32_t lastSlowStageAtMs = 0;
  uint32_t slowStageCount = 0;
  uint32_t maxCrsfUs = 0;
  uint32_t maxCrsfAtMs = 0;
  uint32_t maxImuUs = 0;
  uint32_t maxImuAtMs = 0;
  uint32_t maxLiveUs = 0;
  uint32_t maxControlUs = 0;
  uint32_t maxServoWriteUs = 0;
};

enum class BlackboxEventKind : uint8_t {
  Boot = 1,
  SlowStage = 2,
  ImuError = 3,
  UartOverflow = 4,
  Failsafe = 5,
};

constexpr uint8_t kBlackboxCapacity = 16;
struct BlackboxEvent {
  uint32_t sequence;
  uint32_t bootSequence;
  uint32_t atMs;
  uint32_t value;
  uint16_t aux;
  uint8_t kind;
  uint8_t stage;
};

struct BlackboxSnapshot {
  uint8_t count = 0;
  BlackboxEvent events[kBlackboxCapacity]{};
};

#ifndef DOMINO_SIL
void loopDiagnosticsInit();
LoopDiagnosticsSnapshot loopDiagnosticsSnapshot();
void loopBlackboxRecord(BlackboxEventKind kind, uint8_t stage, uint32_t value,
                        uint16_t aux = 0);
BlackboxSnapshot loopBlackboxSnapshot();

// A stage remains in RTC memory while its scope is active. If a watchdog
// resets the ESP32 inside a stage, the next boot can report that marker.
class ScopedLoopStage {
 public:
  explicit ScopedLoopStage(LoopStage stage);
  ~ScopedLoopStage();
  ScopedLoopStage(const ScopedLoopStage &) = delete;
  ScopedLoopStage &operator=(const ScopedLoopStage &) = delete;

 private:
  LoopStage stage_;
  LoopStage parent_;
  uint32_t startedUs_;
};
#endif
