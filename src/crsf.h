#pragma once

#include <Arduino.h>

constexpr int RX_PIN = 16;
constexpr int TX_PIN = 17;
constexpr uint32_t CRSF_BAUD = 420000;
constexpr float CH_FILTER_ALPHA = 0.25f;
constexpr uint8_t CRSF_ADDR_FC = 0xC8;
constexpr uint8_t CRSF_TYPE_RC_CHANNELS = 0x16;
constexpr int CRSF_MAX_LEN = 64;
constexpr uint32_t CRSF_TIMEOUT_MS = 250;

constexpr int SA_CH_INDEX = 4;
constexpr int SA_ON_THRESHOLD_US = 1600;
constexpr int SA_OFF_THRESHOLD_US = 1400;
// SC is used as a 3-position mode switch (balance mode = middle).
// Adjust SC_CH_INDEX if your radio maps SC differently.
constexpr int SC_CH_INDEX = 6;

extern uint16_t ch_raw[16];
extern int ch_us[16];
extern float ch_us_filt[16];
extern float ch_us_filtered_2nd[16];
extern unsigned long lastCrsfMs;

void initCrsfState();
void processCrsfFrames(unsigned long now);
bool crsfLinkAlive(unsigned long now);
