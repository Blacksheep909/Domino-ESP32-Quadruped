#include <Arduino.h>
#include <Wire.h>

#include <stdarg.h>

namespace {
uint64_t gSimTimeUs = 0;
}

HardwareSerial Serial(true);
HardwareSerial Serial2(false);
TwoWire Wire;

unsigned long millis() {
  return static_cast<unsigned long>(gSimTimeUs / 1000ULL);
}

unsigned long micros() {
  return static_cast<unsigned long>(gSimTimeUs);
}

void delay(unsigned long milliseconds) {
  simAdvanceTimeUs(static_cast<uint64_t>(milliseconds) * 1000ULL);
}

void simSetTimeUs(uint64_t microseconds) {
  gSimTimeUs = microseconds;
}

void simAdvanceTimeUs(uint64_t microseconds) {
  gSimTimeUs += microseconds;
}

HardwareSerial::HardwareSerial(bool logOutput) : logOutput_(logOutput) {}

void HardwareSerial::begin(uint32_t) {}

void HardwareSerial::begin(uint32_t, uint32_t, int8_t, int8_t) {}

int HardwareSerial::available() const {
  return static_cast<int>(rxBytes_.size());
}

int HardwareSerial::read() {
  if (rxBytes_.empty()) {
    return -1;
  }
  const uint8_t value = rxBytes_.front();
  rxBytes_.pop_front();
  return value;
}

void HardwareSerial::inject(const std::vector<uint8_t>& bytes) {
  rxBytes_.insert(rxBytes_.end(), bytes.begin(), bytes.end());
}

void HardwareSerial::print(const char* value) {
  if (logOutput_) {
    fputs(value, stdout);
  }
}

void HardwareSerial::print(const std::string& value) {
  print(value.c_str());
}

void HardwareSerial::println(const char* value) {
  if (logOutput_) {
    fprintf(stdout, "%s\n", value);
  }
}

void HardwareSerial::println(const std::string& value) {
  println(value.c_str());
}

int HardwareSerial::printf(const char* format, ...) {
  if (!logOutput_) {
    return 0;
  }
  va_list args;
  va_start(args, format);
  const int written = vfprintf(stdout, format, args);
  va_end(args);
  return written;
}
