#pragma once

#include <stdint.h>

void simRecordServoPulse(uint8_t channel, uint16_t microseconds);
uint16_t simServoPulseUs(uint8_t channel);
float simServoAngleDeg(uint8_t channel);
uint32_t simServoWriteCount(uint8_t channel);
void simResetServoOutputs();
