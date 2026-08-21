#pragma once

#include <stdint.h>

constexpr uint32_t DOMINO_MANUAL_TIMEOUT_MS = 250;
constexpr uint32_t DOMINO_MANUAL_MAX_LEASE_MS = 30000;

enum class LiveManualMode : uint8_t { Stand, Careful, Trot };

struct LiveManualControlSnapshot {
  bool active = false;
  bool deadman = false;
  LiveManualMode mode = LiveManualMode::Stand;
  float forward = 0.0f;
  float turn = 0.0f;
  float roll = 0.0f;
  float height = 0.0f;
};

class ManualControlGuard {
 public:
  bool grant(const char *token, uint32_t leaseMs, uint32_t now);
  bool release(const char *token);
  bool acceptFrame(const char *token, uint32_t sequence, uint32_t now,
                   LiveManualMode mode, float forward, float turn,
                   float roll, float height, uint32_t timeoutMs);
  void acceptNeutral(uint32_t now);
  void tick(uint32_t now, bool prerequisitesValid);
  void revoke();

  bool authorityActive() const { return authorityActive_; }
  bool frameReceived() const { return frameReceived_; }
  uint32_t frameAgeMs(uint32_t now) const;
  uint32_t leaseRemainingMs(uint32_t now) const;
  const LiveManualControlSnapshot &snapshot() const { return snapshot_; }

 private:
  bool tokenMatches(const char *candidate) const;
  void neutralize();
  static bool before(uint32_t now, uint32_t deadline);
  static bool bounded(float value);

  bool authorityActive_ = false;
  char token_[65] = {};
  uint32_t expiresMs_ = 0;
  bool frameReceived_ = false;
  uint32_t lastFrameMs_ = 0;
  bool haveSequence_ = false;
  uint32_t lastSequence_ = 0;
  LiveManualControlSnapshot snapshot_{};
};
