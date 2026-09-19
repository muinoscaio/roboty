#pragma once

#include <Arduino.h>

namespace roboty {

class MotorDriver {
 public:
  MotorDriver(
      uint8_t in1Pin,
      uint8_t in2Pin,
      uint8_t pwmPin,
      uint8_t pwmChannel,
      uint8_t standbyPin,
      bool inverted = false);

  void begin(uint32_t pwmFrequencyHz, uint8_t pwmResolutionBits);
  void setPercent(int percent);
  void stop();

  int percent() const;

 private:
  uint8_t in1Pin_;
  uint8_t in2Pin_;
  uint8_t pwmPin_;
  uint8_t pwmChannel_;
  uint8_t standbyPin_;
  bool inverted_;
  uint8_t pwmResolutionBits_;
  int percent_;

  void writePwm(uint32_t duty);
};

}  // namespace roboty
