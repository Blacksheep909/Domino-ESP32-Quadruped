#pragma once

#include <Arduino.h>

constexpr int RX_PIN = 16;
constexpr int TX_PIN = 17;
constexpr uint32_t CRSF_BAUD = 420000;
// The default 256-byte UART ring holds less than 20 ms at 500 Hz / 26 bytes.
// Servo I2C work and engineering telemetry can delay the next receiver poll.
constexpr size_t CRSF_RX_BUFFER_BYTES = 4096;
// At 250 Hz the receiver sends about 6.5 kB/s of channel frames. This cap
// bounds one pass while leaving enough headroom for a normal 40 Hz loop.
constexpr size_t CRSF_RX_MAX_BYTES_PER_PASS = 384;
constexpr float CH_FILTER_ALPHA = 0.25f;
constexpr uint8_t CRSF_ADDR_FC = 0xC8;
constexpr uint8_t CRSF_TYPE_RC_CHANNELS = 0x16;
constexpr uint8_t CRSF_TYPE_LINK_STATISTICS = 0x14;
constexpr int CRSF_MAX_LEN = 64;
constexpr uint32_t CRSF_TIMEOUT_MS = 1000;

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
bool crsfHasReceivedFrame();
uint32_t crsfAcceptedFrameCount();
uint32_t crsfCrcErrorCount();
uint32_t crsfUartOverflowCount();
uint32_t crsfReceiveBudgetHitCount();
uint32_t crsfLastBudgetHitAtMs();
uint16_t crsfPendingBytes();
uint16_t crsfMaxPendingBytes();
uint16_t crsfLastPassBytes();

struct CrsfLinkStatistics {
  bool valid;
  uint8_t linkQualityPercent;
  int16_t rssi1Dbm;
  int16_t rssi2Dbm;
  int8_t snrDb;
  uint8_t activeAntenna;
  uint8_t rfMode;
  uint8_t txPowerCode;
  unsigned long timestampMs;
};

CrsfLinkStatistics crsfLinkStatistics();
float crsfPacketRateHz();
