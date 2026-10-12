#include "manual_control_guard.h"

#include <math.h>
#include <string.h>

bool ManualControlGuard::before(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(deadline - now) > 0;
}

bool ManualControlGuard::bounded(float value) {
  return isfinite(value) && value >= -1.0f && value <= 1.0f;
}

bool ManualControlGuard::tokenMatches(const char *candidate) const {
  if (!authorityActive_ || !candidate) return false;
  const size_t expectedLength = strlen(token_);
  const size_t candidateLength = strlen(candidate);
  size_t difference = expectedLength ^ candidateLength;
  const size_t comparedLength = expectedLength > candidateLength ? expectedLength : candidateLength;
  for (size_t index = 0; index < comparedLength; ++index) {
    const uint8_t expected = index < expectedLength ? token_[index] : 0;
    const uint8_t supplied = index < candidateLength ? candidate[index] : 0;
    difference |= expected ^ supplied;
  }
  return difference == 0;
}

void ManualControlGuard::neutralize() {
  snapshot_ = LiveManualControlSnapshot{};
  snapshot_.active = authorityActive_;
  frameReceived_ = authorityActive_;
}

bool ManualControlGuard::grant(const char *token, uint32_t leaseMs, uint32_t now) {
  if (authorityActive_ || !token || token[0] == '\0' || strlen(token) >= sizeof(token_) ||
      leaseMs == 0 || leaseMs > DOMINO_MANUAL_MAX_LEASE_MS) return false;
  strcpy(token_, token);
  authorityActive_ = true;
  expiresMs_ = now + leaseMs;
  lastFrameMs_ = now;
  frameReceived_ = false;
  haveSequence_ = false;
  lastSequence_ = 0;
  snapshot_ = LiveManualControlSnapshot{};
  return true;
}

bool ManualControlGuard::release(const char *token) {
  if (!tokenMatches(token)) return false;
  revoke();
  return true;
}

bool ManualControlGuard::acceptFrame(const char *token, uint32_t sequence, uint32_t now,
                                     LiveManualMode mode, float forward, float turn,
                                     float roll, float pitch, float yaw, float bodyX,
                                     float bodyY, float height, uint32_t timeoutMs) {
  if (!tokenMatches(token) || !before(now, expiresMs_) ||
      timeoutMs != DOMINO_MANUAL_TIMEOUT_MS ||
      (haveSequence_ && sequence <= lastSequence_) ||
      !bounded(forward) || !bounded(turn) || !bounded(roll) || !bounded(pitch) ||
      !bounded(yaw) || !bounded(bodyX) || !bounded(bodyY) || !bounded(height)) return false;
  snapshot_.active = true;
  snapshot_.deadman = true;
  snapshot_.mode = mode;
  snapshot_.forward = forward;
  snapshot_.turn = turn;
  snapshot_.roll = roll;
  snapshot_.pitch = pitch;
  snapshot_.yaw = yaw;
  snapshot_.bodyX = bodyX;
  snapshot_.bodyY = bodyY;
  snapshot_.height = height;
  frameReceived_ = true;
  lastFrameMs_ = now;
  lastSequence_ = sequence;
  haveSequence_ = true;
  return true;
}

void ManualControlGuard::acceptNeutral(uint32_t now) {
  if (!authorityActive_) return;
  neutralize();
  lastFrameMs_ = now;
}

void ManualControlGuard::tick(uint32_t now, bool prerequisitesValid) {
  if (!authorityActive_) return;
  if (!prerequisitesValid || !before(now, expiresMs_)) {
    revoke();
  } else if (frameReceived_ && now - lastFrameMs_ > DOMINO_MANUAL_TIMEOUT_MS) {
    neutralize();
    lastFrameMs_ = now;
  }
}

void ManualControlGuard::revoke() {
  authorityActive_ = false;
  token_[0] = '\0';
  expiresMs_ = 0;
  frameReceived_ = false;
  lastFrameMs_ = 0;
  haveSequence_ = false;
  lastSequence_ = 0;
  snapshot_ = LiveManualControlSnapshot{};
}

uint32_t ManualControlGuard::frameAgeMs(uint32_t now) const {
  return frameReceived_ ? now - lastFrameMs_ : 0;
}

uint32_t ManualControlGuard::leaseRemainingMs(uint32_t now) const {
  return authorityActive_ && before(now, expiresMs_) ? expiresMs_ - now : 0;
}
