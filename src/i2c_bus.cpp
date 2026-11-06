#include "i2c_bus.h"

#include <Arduino.h>
#include <Wire.h>

namespace {
// Keep these matched to the original setup(): 100 kHz with a bounded timeout so
// a device that stops responding cannot stall the control loop for long.
constexpr uint32_t kI2cClockHz = 100000;
constexpr uint16_t kI2cTimeoutMs = 20;
constexpr uint8_t kRecoveryClockPulses = 9;
constexpr uint32_t kHalfClockUs = 5;  // 100 kHz

uint32_t recoveryCount = 0;
uint32_t failedRecoveryCount = 0;
uint32_t lastRecoveryMs = 0;
}  // namespace

void i2cBusBegin() {
  Wire.begin();
  Wire.setClock(kI2cClockHz);
  Wire.setTimeOut(kI2cTimeoutMs);
}

bool i2cBusIdle() {
  // The GPIO input stays readable while the I2C peripheral owns the pins.
  return digitalRead(SDA) == HIGH && digitalRead(SCL) == HIGH;
}

bool i2cBusRecover() {
  ++recoveryCount;
  lastRecoveryMs = millis();
  Wire.end();

  // Drive SCL ourselves; let SDA float on its pull-up so we can watch it.
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(kHalfClockUs);
  // Clock until the stuck slave releases SDA. Stop as soon as it does: an extra
  // clock while SDA is low would look like an ACK and start another byte.
  for (uint8_t pulse = 0; pulse < kRecoveryClockPulses && digitalRead(SDA) == LOW; ++pulse) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(kHalfClockUs);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(kHalfClockUs);
  }

  // STOP condition: SDA rises while SCL is high, which resets every slave's
  // state machine.
  pinMode(SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA, LOW);
  delayMicroseconds(kHalfClockUs);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(kHalfClockUs);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(kHalfClockUs);

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  const bool released = digitalRead(SDA) == HIGH && digitalRead(SCL) == HIGH;
  if (!released) ++failedRecoveryCount;
  i2cBusBegin();
  return released;
}

bool i2cBusEnsureIdle() {
  if (i2cBusIdle()) return true;
  // Transactions are synchronous, so between them both lines should be high.
  // Look again briefly before declaring the bus stuck, so a slow rising edge
  // on a long wire is not mistaken for a held line.
  delayMicroseconds(20);
  if (i2cBusIdle()) return true;
  return i2cBusRecover();
}

uint32_t i2cBusRecoveryCount() { return recoveryCount; }
uint32_t i2cBusFailedRecoveryCount() { return failedRecoveryCount; }
uint32_t i2cBusLastRecoveryMs() { return lastRecoveryMs; }
