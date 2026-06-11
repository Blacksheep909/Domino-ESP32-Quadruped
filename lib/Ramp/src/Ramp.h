#pragma once

#include <Arduino.h>

class rampFloat {
 public:
  rampFloat() = default;

  explicit rampFloat(float speed) : speed_(fabsf(speed)) {}

  void setSpeed(float speed) { speed_ = fabsf(speed); }

  void go(float value) {
    value_ = target_ = value;
    lastMicros_ = micros();
    initialized_ = true;
  }

  void move(float target) { target_ = target; }

  void setTarget(float target) { move(target); }

  float getValue() const { return value_; }

  bool isFinished(float epsilon = 1e-3f) const {
    return fabsf(target_ - value_) <= epsilon;
  }

  float update() {
    if (!initialized_) {
      go(target_);
      return value_;
    }

    uint32_t now = micros();
    float dt = (now - lastMicros_) / 1e6f;
    lastMicros_ = now;

    if (dt <= 0.0f || speed_ <= 0.0f) {
      value_ = target_;
      return value_;
    }

    const float delta = target_ - value_;
    const float maxStep = speed_ * dt;

    if (fabsf(delta) <= maxStep) {
      value_ = target_;
    } else {
      value_ += (delta > 0.0f ? maxStep : -maxStep);
    }
    return value_;
  }

 private:
  float target_ = 0.0f;
  float value_ = 0.0f;
  float speed_ = 100.0f;  // units per second
  uint32_t lastMicros_ = 0;
  bool initialized_ = false;
};
