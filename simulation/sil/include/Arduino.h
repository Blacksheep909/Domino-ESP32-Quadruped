#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <deque>
#include <string>
#include <vector>

#define SERIAL_8N1 0

template <typename T>
T constrain(T value, T minimum, T maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

unsigned long millis();
unsigned long micros();
void delay(unsigned long milliseconds);

void simSetTimeUs(uint64_t microseconds);
void simAdvanceTimeUs(uint64_t microseconds);

class HardwareSerial {
 public:
  explicit HardwareSerial(bool logOutput = false);

  void begin(uint32_t baud);
  void begin(uint32_t baud, uint32_t config, int8_t rxPin, int8_t txPin);
  int available() const;
  int read();
  void inject(const std::vector<uint8_t>& bytes);

  void print(const char* value);
  void print(const std::string& value);
  void println(const char* value);
  void println(const std::string& value);
  int printf(const char* format, ...);

 private:
  bool logOutput_;
  std::deque<uint8_t> rxBytes_;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial2;
