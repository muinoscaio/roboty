#pragma once

#include <cmath>
#include <cstdint>

namespace roboty {

class WheelSpeedEstimator {
 public:
  WheelSpeedEstimator(float countsPerWheelRevolution, uint32_t sampleIntervalMs)
      : countsPerWheelRevolution_(countsPerWheelRevolution),
        sampleIntervalMs_(sampleIntervalMs == 0U ? 1U : sampleIntervalMs),
        calibrated_(std::isfinite(countsPerWheelRevolution) &&
                    countsPerWheelRevolution > 0.0F),
        initialized_(false),
        lastTicks_(0),
        lastSampleMs_(0),
        deltaTicks_(0),
        sampleMs_(0),
        rpm_(0.0) {}

  void reset(int64_t ticks, uint32_t nowMs) {
    lastTicks_ = ticks;
    lastSampleMs_ = nowMs;
    deltaTicks_ = 0;
    sampleMs_ = 0;
    rpm_ = 0.0;
    initialized_ = true;
  }

  bool update(int64_t ticks, uint32_t nowMs) {
    if (!initialized_) {
      reset(ticks, nowMs);
      return false;
    }

    const uint32_t elapsedMs = nowMs - lastSampleMs_;
    if (elapsedMs < sampleIntervalMs_) {
      return false;
    }

    deltaTicks_ = saturatedDifference(ticks, lastTicks_);
    sampleMs_ = elapsedMs;
    rpm_ = calibrated_
               ? static_cast<double>(deltaTicks_) * 60000.0 /
                     (countsPerWheelRevolution_ * elapsedMs)
               : 0.0;
    lastTicks_ = ticks;
    lastSampleMs_ = nowMs;
    return true;
  }

  double rpm() const { return rpm_; }
  int64_t deltaTicks() const { return deltaTicks_; }
  uint32_t sampleMs() const { return sampleMs_; }
  bool calibrated() const { return calibrated_; }

 private:
  double countsPerWheelRevolution_;
  uint32_t sampleIntervalMs_;
  bool calibrated_;
  bool initialized_;
  int64_t lastTicks_;
  uint32_t lastSampleMs_;
  int64_t deltaTicks_;
  uint32_t sampleMs_;
  double rpm_;

  static int64_t saturatedDifference(int64_t current, int64_t previous) {
    if (previous > 0 && current < INT64_MIN + previous) {
      return INT64_MIN;
    }
    if (previous < 0 && current > INT64_MAX + previous) {
      return INT64_MAX;
    }
    return current - previous;
  }
};

}  // namespace roboty
