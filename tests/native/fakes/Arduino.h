#pragma once

#include <array>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#define F(text) text
constexpr int OUTPUT = 1;
constexpr int LOW = 0;
constexpr int HIGH = 1;

inline uint32_t testMillis = 0;
inline std::array<int, 40> testPins{};
inline std::array<uint32_t, 16> testDuty{};

inline uint32_t millis() { return testMillis; }
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t pin, int value) { testPins.at(pin) = value; }
inline double ledcSetup(uint8_t, uint32_t frequency, uint8_t) { return frequency; }
inline void ledcAttachPin(uint8_t, uint8_t) {}
inline void ledcWrite(uint8_t channel, uint32_t duty) { testDuty.at(channel) = duty; }

struct TestSerial {
  std::string input;
  std::string output;
  size_t offset = 0;
  uint32_t readTimeMs = 0;

  int available() const { return static_cast<int>(input.size() - offset); }
  int read() {
    testMillis += readTimeMs;
    return offset < input.size() ? static_cast<unsigned char>(input[offset++]) : -1;
  }
  void println(const char* text = "") { output += std::string(text) + '\n'; }
  void printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    va_list copy;
    va_copy(copy, args);
    const int size = vsnprintf(nullptr, 0, format, copy);
    va_end(copy);
    std::vector<char> buffer(static_cast<size_t>(size) + 1);
    vsnprintf(buffer.data(), buffer.size(), format, args);
    va_end(args);
    output.append(buffer.data(), static_cast<size_t>(size));
  }
};

inline TestSerial Serial;
