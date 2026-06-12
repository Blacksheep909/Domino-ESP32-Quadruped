#include <Arduino.h>

constexpr int RX_PIN = 16;
constexpr int TX_PIN = 17;
constexpr uint32_t CRSF_BAUD = 420000;
constexpr uint8_t CRSF_ADDR_FC = 0xC8;
constexpr uint8_t CRSF_TYPE_RC_CHANNELS = 0x16;
constexpr int CRSF_MAX_LEN = 64;
constexpr float CH_FILTER_ALPHA = 0.25f;
constexpr uint32_t CRSF_TIMEOUT_MS = 250;

uint16_t chRaw[16] = {0};
int chUs[16] = {1500};
float chFilt1[16] = {0.0f};
float chFilt2[16] = {0.0f};
uint32_t lastCrsfMs = 0;
uint32_t lastPrintMs = 0;

uint8_t crc8DvbS2(const uint8_t *buf, int len) {
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

int crsfToUs(uint16_t value) {
  value = constrain(value, static_cast<uint16_t>(172), static_cast<uint16_t>(1811));
  return 1000 + ((static_cast<int32_t>(value) - 172) * 1000L) / 1639L;
}

void unpackChannels11(const uint8_t *payload, uint16_t *out) {
  uint32_t bit = 0;
  for (int i = 0; i < 16; ++i) {
    const int byteIndex = bit >> 3;
    const uint32_t window = static_cast<uint32_t>(payload[byteIndex]) |
                            (static_cast<uint32_t>(payload[byteIndex + 1]) << 8) |
                            (static_cast<uint32_t>(payload[byteIndex + 2]) << 16);
    out[i] = (window >> (bit & 7)) & 0x07FF;
    bit += 11;
  }
}

bool readCrsfFrame(uint8_t &type, uint8_t *payload, uint8_t &payloadLen) {
  enum ParseState { WAIT_ADDR, GOT_LEN, GOT_PAYLOAD };
  static ParseState state = WAIT_ADDR;
  static uint8_t len = 0;
  static uint8_t pos = 0;
  static uint8_t buf[CRSF_MAX_LEN];

  while (Serial2.available()) {
    const uint8_t b = Serial2.read();
    switch (state) {
      case WAIT_ADDR:
        if (b == CRSF_ADDR_FC || b == 0xEA || b == 0x00) {
          state = GOT_LEN;
        }
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
          const uint8_t expected = crc8DvbS2(buf, len - 1);
          if (expected == buf[len - 1]) {
            type = buf[0];
            payloadLen = len - 2;
            memcpy(payload, &buf[1], payloadLen);
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

void updateCrsf(uint32_t now) {
  uint8_t type = 0;
  uint8_t payload[CRSF_MAX_LEN] = {0};
  uint8_t payloadLen = 0;

  while (readCrsfFrame(type, payload, payloadLen)) {
    if (type == CRSF_TYPE_RC_CHANNELS && payloadLen == 22) {
      unpackChannels11(payload, chRaw);
      for (int i = 0; i < 16; ++i) {
        const int sample = crsfToUs(chRaw[i]);
        chFilt1[i] += (sample - chFilt1[i]) * CH_FILTER_ALPHA;
        chFilt2[i] += (chFilt1[i] - chFilt2[i]) * (CH_FILTER_ALPHA * 2.0f);
        chUs[i] = static_cast<int>(chFilt2[i] + 0.5f);
      }
      lastCrsfMs = now;
    }
  }
}

bool crsfLinkAlive(uint32_t now) {
  return (now - lastCrsfMs) < CRSF_TIMEOUT_MS;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(CRSF_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  for (int i = 0; i < 16; ++i) {
    chUs[i] = 1500;
    chFilt1[i] = 1500.0f;
    chFilt2[i] = 1500.0f;
  }
  lastCrsfMs = millis();

  Serial.println("ESP32 CRSF reader ready.");
}

void loop() {
  const uint32_t now = millis();
  updateCrsf(now);

  if ((now - lastPrintMs) >= 100) {
    lastPrintMs = now;
    Serial.printf("link=%d", crsfLinkAlive(now) ? 1 : 0);
    for (int i = 0; i < 16; ++i) {
      Serial.printf(" ch%d=%d", i + 1, chUs[i]);
    }
    Serial.println();
  }
}
