#include "crsf.h"

#include <HardwareSerial.h>
#include <string.h>

uint16_t ch_raw[16] = {0};
int ch_us[16] = {1500};
float ch_us_filt[16] = {0.0f};
float ch_us_filtered_2nd[16] = {0.0f};
unsigned long lastCrsfMs = 0;

namespace {

bool hasReceivedCrsfFrame = false;
uint32_t acceptedCrsfFrameCount = 0;
CrsfLinkStatistics linkStatistics{};
float packetRateHz = 0.0f;
unsigned long packetRateWindowStartedMs = 0;
uint32_t packetRateWindowFrames = 0;

uint8_t crc8_dvb_s2_buf(const uint8_t* buf, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; ++i) {
    crc ^= buf[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

inline int crsfToUs(uint16_t v) {
  if (v < 172) v = 172;
  if (v > 1811) v = 1811;
  return 1000 + ((int32_t)(v - 172) * 1000L) / 1639L;
}

void unpackChannels11(const uint8_t* p, uint8_t payloadLen, uint16_t* out16) {
  for (int i = 0; i < 16; ++i) {
    uint16_t value = 0;
    const uint32_t channelBitStart = static_cast<uint32_t>(i) * 11U;
    for (uint8_t bit = 0; bit < 11; ++bit) {
      const uint32_t srcBit = channelBitStart + bit;
      const uint8_t byteIndex = srcBit >> 3;
      if (byteIndex >= payloadLen) {
        break;
      }
      if ((p[byteIndex] & (1U << (srcBit & 7))) != 0) {
        value |= (1U << bit);
      }
    }
    out16[i] = value;
  }
}

bool readCrsfFrame(uint8_t& type, uint8_t* payload, uint8_t& plen) {
  static enum { WAIT_ADDR, GOT_LEN, GOT_PAYLOAD } state = WAIT_ADDR;
  static uint8_t len = 0;
  static uint8_t pos = 0;
  static uint8_t buf[CRSF_MAX_LEN];

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    switch (state) {
      case WAIT_ADDR:
        if (b == CRSF_ADDR_FC || b == 0xEA || b == 0x00) state = GOT_LEN;
        break;

      case GOT_LEN:
        len = b;
        if (len < 2 || len > (CRSF_MAX_LEN - 2)) {
          state = WAIT_ADDR;
          break;
        }
        pos = 0;
        state = GOT_PAYLOAD;
        break;

      case GOT_PAYLOAD:
        buf[pos++] = b;
        if (pos >= len) {
          uint8_t calc = crc8_dvb_s2_buf(buf, len - 1);
          if (calc == buf[len - 1]) {
            type = buf[0];
            plen = len - 2;
            memcpy(payload, &buf[1], plen);
            state = WAIT_ADDR;
            return true;
          }
          state = WAIT_ADDR;
        }
        break;
    }
  }
  return false;
}

}  // namespace

void initCrsfState() {
  for (int i = 0; i < 16; ++i) {
    ch_us[i] = 1500;
    ch_us_filt[i] = 1500.0f;
    ch_us_filtered_2nd[i] = 1500.0f;
  }
  lastCrsfMs = 0;
  hasReceivedCrsfFrame = false;
  acceptedCrsfFrameCount = 0;
  linkStatistics = {};
  packetRateHz = 0.0f;
  packetRateWindowStartedMs = 0;
  packetRateWindowFrames = 0;
}

void processCrsfFrames(unsigned long now) {
  uint8_t type = 0;
  uint8_t payload[CRSF_MAX_LEN] = {0};
  uint8_t plen = 0;

  while (readCrsfFrame(type, payload, plen)) {
    if (type == CRSF_TYPE_RC_CHANNELS && plen == 22) {
      unpackChannels11(payload, plen, ch_raw);
      for (int i = 0; i < 16; ++i) {
        int sample = crsfToUs(ch_raw[i]);
        float prev1 = ch_us_filt[i];
        float prev2 = ch_us_filtered_2nd[i];
        float next1 = prev1 + (sample - prev1) * CH_FILTER_ALPHA;
        float next2 = prev2 + (next1 - prev2) * (CH_FILTER_ALPHA * 2.0f);
        ch_us_filt[i] = next1;
        ch_us_filtered_2nd[i] = next2;
        ch_us[i] = static_cast<int>(next2 + 0.5f);
      }
      lastCrsfMs = now;
      hasReceivedCrsfFrame = true;
      ++acceptedCrsfFrameCount;
      ++packetRateWindowFrames;
      if (packetRateWindowStartedMs == 0) packetRateWindowStartedMs = now;
      const unsigned long rateWindowMs = now - packetRateWindowStartedMs;
      if (rateWindowMs >= 500) {
        packetRateHz = static_cast<float>(packetRateWindowFrames) * 1000.0f /
                       static_cast<float>(rateWindowMs);
        packetRateWindowFrames = 0;
        packetRateWindowStartedMs = now;
      }
    } else if (type == CRSF_TYPE_LINK_STATISTICS && plen >= 10) {
      linkStatistics.valid = true;
      linkStatistics.rssi1Dbm = -static_cast<int16_t>(payload[0]);
      linkStatistics.rssi2Dbm = -static_cast<int16_t>(payload[1]);
      linkStatistics.linkQualityPercent = payload[2] > 100 ? 100 : payload[2];
      linkStatistics.snrDb = static_cast<int8_t>(payload[3]);
      linkStatistics.activeAntenna = payload[4];
      linkStatistics.rfMode = payload[5];
      linkStatistics.txPowerCode = payload[6];
      linkStatistics.timestampMs = now;
    }
  }
}

bool crsfLinkAlive(unsigned long now) {
  return hasReceivedCrsfFrame && ((now - lastCrsfMs) < CRSF_TIMEOUT_MS);
}

bool crsfHasReceivedFrame() {
  return hasReceivedCrsfFrame;
}

uint32_t crsfAcceptedFrameCount() {
  return acceptedCrsfFrameCount;
}

CrsfLinkStatistics crsfLinkStatistics() { return linkStatistics; }

float crsfPacketRateHz() { return packetRateHz; }
