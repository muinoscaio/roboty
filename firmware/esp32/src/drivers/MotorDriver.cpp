#include "drivers/MotorDriver.h"

#include <algorithm>
#include <cmath>

namespace roboty {

MotorDriver::MotorDriver(
    uint8_t in1Pin,
    uint8_t in2Pin,
    uint8_t pwmPin,
    uint8_t pwmChannel,
    uint8_t standbyPin,
    bool inverted)
    : in1Pin_(in1Pin),
      in2Pin_(in2Pin),
      pwmPin_(pwmPin),
      pwmChannel_(pwmChannel),
      standbyPin_(standbyPin),
      inverted_(inverted),
      pwmResolutionBits_(8),
      percent_(0) {}

void MotorDriver::begin(uint32_t pwmFrequencyHz, uint8_t pwmResolutionBits) {
  pwmResolutionBits_ = pwmResolutionBits;

  pinMode(in1Pin_, OUTPUT);
  pinMode(in2Pin_, OUTPUT);
  pinMode(standbyPin_, OUTPUT);

  digitalWrite(in1Pin_, LOW);
  digitalWrite(in2Pin_, LOW);
  digitalWrite(standbyPin_, HIGH);

  ledcSetup(pwmChannel_, pwmFrequencyHz, pwmResolutionBits_);
  ledcAttachPin(pwmPin_, pwmChannel_);

  stop();
}

void MotorDriver::setPercent(int percent) {
  percent = std::max(-100, std::min(100, percent));

  const int effectivePercent = inverted_ ? -percent : percent_;
  (void)effectivePercent;

  int drivePercent = inverted_ ? -percent : percent;
  percent_ = percent;

  if (drivePercent == 0) {
    stop();
    return;
  }

  const bool forward = drivePercent > 0;
  digitalWrite(in1Pin_, forward ? HIGH : LOW);
  digitalWrite(in2Pin_, forward ? LOW : HIGH);

  const uint32_t maxDuty = (1UL << pwmResolutionBits_) - 1UL;
  const uint32_t duty = static_cast<uint32_t>(
      std::lround((std::abs(drivePercent) / 100.0) * maxDuty));

  writePwm(duty);
}

void MotorDriver::stop() {
  percent_ = 0;
  writePwm(0);
  digitalWrite(in1Pin_, LOW);
  digitalWrite(in2Pin_, LOW);
}

int MotorDriver::percent() const {
  return percent_;
}

void MotorDriver::writePwm(uint32_t duty) {
  ledcWrite(pwmChannel_, duty);
}

}  // namespace roboty
