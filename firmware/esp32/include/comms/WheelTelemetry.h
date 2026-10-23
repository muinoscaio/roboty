#pragma once

#include <cstdint>

namespace roboty {

struct WheelTelemetry {
  bool enabled = false;
  bool calibrated = false;
  int64_t leftTicks = 0;
  int64_t rightTicks = 0;
  int64_t leftDelta = 0;
  int64_t rightDelta = 0;
  double leftRpm = 0.0;
  double rightRpm = 0.0;
  uint32_t leftInvalid = 0;
  uint32_t rightInvalid = 0;
  uint32_t sampleMs = 0;
};

}  // namespace roboty
