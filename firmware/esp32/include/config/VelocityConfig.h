#pragma once

#include "control/VelocityControl.h"

namespace roboty::config {

// Hardware-dependent gains and limits must be measured before opting in.
constexpr bool CLOSED_LOOP_ENABLED = false;
constexpr double VELOCITY_MAX_TARGET_RPM = 0.0;
constexpr double VELOCITY_MAX_FEEDBACK_RPM = 0.0;
constexpr double VELOCITY_MAX_PWM_PERCENT = 0.0;
constexpr PidGains LEFT_VELOCITY_PID{0.0, 0.0, 0.0, 0.0, 0.02};
constexpr PidGains RIGHT_VELOCITY_PID{0.0, 0.0, 0.0, 0.0, 0.02};
constexpr uint32_t VELOCITY_FEEDBACK_TIMEOUT_MS = 300;
constexpr uint32_t VELOCITY_NO_MOTION_TIMEOUT_MS = 750;

inline VelocityConfig velocityConfig() {
  VelocityConfig result;
  result.enabled = CLOSED_LOOP_ENABLED;
  result.left = LEFT_VELOCITY_PID;
  result.right = RIGHT_VELOCITY_PID;
  result.maxTargetRpm = VELOCITY_MAX_TARGET_RPM;
  result.maxFeedbackRpm = VELOCITY_MAX_FEEDBACK_RPM;
  result.maxPwmPercent = VELOCITY_MAX_PWM_PERCENT;
  result.feedbackTimeoutMs = VELOCITY_FEEDBACK_TIMEOUT_MS;
  result.noMotionTimeoutMs = VELOCITY_NO_MOTION_TIMEOUT_MS;
  return result;
}

}  // namespace roboty::config
