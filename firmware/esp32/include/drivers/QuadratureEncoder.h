#pragma once

#include <Arduino.h>

#include "control/QuadratureDecoder.h"

namespace roboty {

struct EncoderSnapshot {
  int64_t ticks;
  uint32_t invalidTransitions;
};

class QuadratureEncoder {
 public:
  QuadratureEncoder(
      int8_t pinA, int8_t pinB, bool inverted = false, bool pullup = false);
  QuadratureEncoder(const QuadratureEncoder&) = delete;
  QuadratureEncoder& operator=(const QuadratureEncoder&) = delete;

  bool begin();
  void end();
  EncoderSnapshot snapshot() const;

 private:
  int8_t pinA_;
  int8_t pinB_;
  bool pullup_;
  bool started_;
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  QuadratureDecoder decoder_;

  static void IRAM_ATTR onChange(void* context);
  uint8_t IRAM_ATTR readState() const;
};

}  // namespace roboty
