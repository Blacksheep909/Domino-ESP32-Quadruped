#include "crsf.h"

#include <HardwareSerial.h>
#include <atomic>
#include <string.h>

uint16_t ch_raw[16] = {0};
int ch_us[16] = {1500};
float ch_us_filt[16] = {0.0f};
float ch_us_filtered_2nd[16] = {0.0f};
unsigned long lastCrsfMs = 0;

namespace {

bool hasReceivedCrsfFrame = false;
uint32_t acceptedCrsfFrameCount = 0;
uint32_t crcErrorCount = 0;
std::atomic<uint32_t> uartOverflowCount{0};
uint32_t receiveBudgetHitCount = 0;
uint32_t lastBudgetHitAtMs = 0;
uint16_t pendingBytes = 0;
uint16_t maxPendingBytes = 0;
uint16_t lastPassBytes = 0;
uint16_t pendingChannels[16] = {0};
uint16_t pendingChannelFrames = 0;
bool catchingUp = false;
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

enum class FrameState : uint8_t { WaitAddress, Length, Payload };
FrameState frameState = FrameState::WaitAddress;
uint8_t frameLength = 0;
uint8_t framePosition = 0;
uint8_t frameBuffer[CRSF_MAX_LEN] = {0};

bool pushCrsfByte(uint8_t b, uint8_t& type, uint8_t* payload, uint8_t& plen) {
  switch (frameState) {
    case FrameState::WaitAddress:
      if (b == CRSF_ADDR_FC || b == 0xEA || b == 0x00) frameState = FrameState::Length;
      break;
    case FrameState::Length:
      frameLength = b;
      if (frameLength < 2 || frameLength > (CRSF_MAX_LEN - 2)) {
        frameState = FrameState::WaitAddress;
        break;
      }
      framePosition = 0;
      frameState = FrameState::Payload;
      break;
    case FrameState::Payload:
      frameBuffer[framePosition++] = b;
      if (framePosition >= frameLength) {
        frameState = FrameState::WaitAddress;
        const uint8_t calc = crc8_dvb_s2_buf(frameBuffer, frameLength - 1);
        if (calc != frameBuffer[frameLength - 1]) {
          ++crcErrorCount;
          break;
        }
        type = frameBuffer[0];
        plen = frameLength - 2;
        memcpy(payload, &frameBuffer[1], plen);
        return true;
      }
      break;
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
  crcErrorCount = 0;
  uartOverflowCount.store(0, std::memory_order_relaxed);
  receiveBudgetHitCount = 0;
  lastBudgetHitAtMs = 0;
  pendingBytes = 0;
  maxPendingBytes = 0;
  lastPassBytes = 0;
  pendingChannelFrames = 0;
  catchingUp = false;
  frameState = FrameState::WaitAddress;
  frameLength = 0;
  framePosition = 0;
#ifndef DOMINO_SIL
  Serial2.onReceiveError([](hardwareSerial_error_t error) {
    if (error == UART_BUFFER_FULL_ERROR || error == UART_FIFO_OVF_ERROR) {
      uartOverflowCount.fetch_add(1, std::memory_order_relaxed);
    }
  });
#endif
  linkStatistics = {};
  packetRateHz = 0.0f;
  packetRateWindowStartedMs = 0;
  packetRateWindowFrames = 0;
}

void processCrsfFrames(unsigned long now) {
  uint8_t type = 0;
  uint8_t payload[CRSF_MAX_LEN] = {0};
  uint8_t plen = 0;
  bool channelFrameThisPass = false;
  const int availableAtStart = Serial2.available();
  const size_t passBytes = availableAtStart > 0
      ? (static_cast<size_t>(availableAtStart) < CRSF_RX_MAX_BYTES_PER_PASS
          ? static_cast<size_t>(availableAtStart) : CRSF_RX_MAX_BYTES_PER_PASS) : 0;
  lastPassBytes = 0;
  if (availableAtStart > maxPendingBytes) {
    maxPendingBytes = static_cast<uint16_t>(availableAtStart < 65535 ? availableAtStart : 65535);
  }

  for (size_t index = 0; index < passBytes; ++index) {
    const int nextByte = Serial2.read();
    if (nextByte < 0) break;
    ++lastPassBytes;
    if (!pushCrsfByte(static_cast<uint8_t>(nextByte), type, payload, plen)) continue;
    if (type == CRSF_TYPE_RC_CHANNELS && plen == 22) {
      unpackChannels11(payload, plen, pendingChannels);
      channelFrameThisPass = true;
      if (pendingChannelFrames < UINT16_MAX) ++pendingChannelFrames;
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
  const int remaining = Serial2.available();
  pendingBytes = static_cast<uint16_t>(remaining < 0 ? 0 : remaining < 65535 ? remaining : 65535);
  const bool budgetHit = availableAtStart > static_cast<int>(CRSF_RX_MAX_BYTES_PER_PASS) ||
      lastPassBytes < passBytes;
  if (budgetHit) {
    ++receiveBudgetHitCount;
    lastBudgetHitAtMs = millis();
    catchingUp = true;
  }
  // Never publish an old frame while catching up to a full RX ring. Apply
  // only the newest validated channels after the queued bytes have drained.
  if (catchingUp && !budgetHit && !channelFrameThisPass) {
    // A discarded or partial tail is not proof that the held command is new.
    pendingChannelFrames = 0;
    catchingUp = false;
  }
  if (pendingChannelFrames > 0 && !budgetHit) {
    const uint16_t filterSteps = catchingUp ? 1 :
        (pendingChannelFrames < 8 ? pendingChannelFrames : 8);
    for (uint16_t step = 0; step < filterSteps; ++step) {
      for (int channel = 0; channel < 16; ++channel) {
        const int sample = crsfToUs(pendingChannels[channel]);
        ch_us_filt[channel] += (sample - ch_us_filt[channel]) * CH_FILTER_ALPHA;
        ch_us_filtered_2nd[channel] +=
            (ch_us_filt[channel] - ch_us_filtered_2nd[channel]) * (CH_FILTER_ALPHA * 2.0f);
      }
    }
    for (int channel = 0; channel < 16; ++channel) {
      ch_raw[channel] = pendingChannels[channel];
      ch_us[channel] = static_cast<int>(ch_us_filtered_2nd[channel] + 0.5f);
    }
    lastCrsfMs = now;
    hasReceivedCrsfFrame = true;
    pendingChannelFrames = 0;
    catchingUp = false;
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

uint32_t crsfCrcErrorCount() { return crcErrorCount; }
uint32_t crsfUartOverflowCount() {
  return uartOverflowCount.load(std::memory_order_relaxed);
}
uint32_t crsfReceiveBudgetHitCount() { return receiveBudgetHitCount; }
uint32_t crsfLastBudgetHitAtMs() { return lastBudgetHitAtMs; }
uint16_t crsfPendingBytes() { return pendingBytes; }
uint16_t crsfMaxPendingBytes() { return maxPendingBytes; }
uint16_t crsfLastPassBytes() { return lastPassBytes; }

CrsfLinkStatistics crsfLinkStatistics() { return linkStatistics; }

float crsfPacketRateHz() { return packetRateHz; }
