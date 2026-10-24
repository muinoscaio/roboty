#pragma once

#include <cstdint>

namespace roboty {

// AB is encoded as (A << 1) | B. 00 -> 01 -> 11 -> 10 -> 00 is positive.
// Keep the update path inline and table-free: it also runs with flash unavailable.
class QuadratureDecoder {
 public:
  explicit QuadratureDecoder(bool inverted = false)
      : inverted_(inverted), state_(0), ticks_(0), invalidTransitions_(0) {}

  void reset(uint8_t state, int64_t ticks = 0) {
    state_ = state & 3U;
    ticks_ = ticks;
    invalidTransitions_ = 0;
  }

  inline __attribute__((always_inline)) void update(uint8_t state) {
    state &= 3U;
    const uint8_t changed = state_ ^ state;
    const uint8_t previous = state_;
    state_ = state;

    if (changed == 0U) {
      return;
    }
    if (changed == 3U) {
      if (invalidTransitions_ != UINT32_MAX) {
        ++invalidTransitions_;
      }
      return;
    }

    const bool positive = (((previous >> 1U) ^ state) & 1U) != 0U;
    if (positive != inverted_) {
      if (ticks_ != INT64_MAX) {
        ++ticks_;
      }
    } else if (ticks_ != INT64_MIN) {
      --ticks_;
    }
  }

  int64_t ticks() const { return ticks_; }
  uint32_t invalidTransitions() const { return invalidTransitions_; }

 private:
  bool inverted_;
  uint8_t state_;
  int64_t ticks_;
  uint32_t invalidTransitions_;
};

}  // namespace roboty
