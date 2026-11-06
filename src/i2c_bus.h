#pragma once

#include <stdint.h>

// Shared I2C bus (PCA9685 servo driver + MPU6050 IMU).
//
// A noise spike on SCL during a read can leave a slave part-way through a byte,
// holding SDA low while it waits for clocks that never come. Every later
// transaction on the bus then fails, which stalls the servo driver too. These
// helpers detect a stuck bus and free it with the standard sequence: clock SCL
// (up to nine pulses, stopping as soon as SDA is released), send a STOP, and
// restart the I2C peripheral.

// Start (or restart) Wire with Domino's bus settings.
void i2cBusBegin();

// True when both lines are high, i.e. no transaction is holding the bus.
bool i2cBusIdle();

// Free a stuck bus. Returns true when SDA and SCL are both high afterwards.
bool i2cBusRecover();

// If the bus is not idle, recover it. Cheap enough to call every control tick.
// Returns false only when a recovery was needed and did not free the bus.
bool i2cBusEnsureIdle();

uint32_t i2cBusRecoveryCount();
uint32_t i2cBusFailedRecoveryCount();
uint32_t i2cBusLastRecoveryMs();
