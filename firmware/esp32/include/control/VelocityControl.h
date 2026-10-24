#pragma once

#include <cmath>
#include <cstdint>

#include "comms/WheelTelemetry.h"
#include "control/WheelPid.h"

namespace roboty {

struct VelocityConfig {
  bool enabled = false;
  PidGains left;
  PidGains right;
  double maxTargetRpm = 0.0;
  double maxFeedbackRpm = 0.0;
  double maxPwmPercent = 0.0;
  uint32_t feedbackTimeoutMs = 300;
  uint32_t noMotionTimeoutMs = 750;

  bool valid() const {
    return left.valid() && right.valid() &&
           std::isfinite(maxTargetRpm) && maxTargetRpm > 0.0 &&
           std::isfinite(maxFeedbackRpm) && maxFeedbackRpm >= maxTargetRpm &&
           std::isfinite(maxPwmPercent) && maxPwmPercent >= 1.0 && maxPwmPercent <= 100.0 &&
           feedbackTimeoutMs > 0 && feedbackTimeoutMs <= 10000 &&
           noMotionTimeoutMs > feedbackTimeoutMs && noMotionTimeoutMs <= 60000;
  }
};

enum class VelocityFault {
  None, Disabled, Configuration, TargetRange, FeedbackUnavailable,
  StaleFeedback, InvalidFeedback, EncoderTransition, NoMotion, SampleTiming, Numeric
};

// This class owns control math and fault latching, but never touches GPIOs.
class VelocityControl {
 public:
  explicit VelocityControl(const VelocityConfig& config = {}) : config_(config) {}

  void configure(const VelocityConfig& config) {
    stop();
    config_ = config;
    // Reconfiguration is not a fault acknowledgement.
  }

  void stop() {
    active_ = false;
    leftTarget_ = rightTarget_ = 0.0;
    leftOutput_ = rightOutput_ = 0.0;
    leftPid_.reset();
    rightPid_.reset();
  }

  bool clearFault() {
    if (active_) return false;
    fault_ = VelocityFault::None;
    return true;
  }

  bool setTargets(double left, double right, const WheelTelemetry& feedback, uint32_t now) {
    if (fault_ != VelocityFault::None) return false;
    if (!std::isfinite(left) || !std::isfinite(right)) return trip(VelocityFault::TargetRange);
    if (left == 0.0 && right == 0.0) {
      stop();
      return true;
    }
    if (!config_.enabled) return trip(VelocityFault::Disabled);
    if (!config_.valid()) return trip(VelocityFault::Configuration);
    if (std::fabs(left) > config_.maxTargetRpm || std::fabs(right) > config_.maxTargetRpm) {
      return trip(VelocityFault::TargetRange);
    }
    const VelocityFault badFeedback = feedbackFault(feedback, now);
    if (badFeedback != VelocityFault::None) return trip(badFeedback);

    if (!active_) {
      lastSequence_ = feedback.sampleSequence;
      lastSampleAt_ = feedback.sampleAtMs;
      leftTicks_ = feedback.leftTicks;
      rightTicks_ = feedback.rightTicks;
      leftInvalid_ = feedback.leftInvalid;
      rightInvalid_ = feedback.rightInvalid;
      leftMotionAt_ = rightMotionAt_ = now;
    }
    changeTarget(left, leftTarget_, leftPid_, leftOutput_, leftMotionAt_, now);
    changeTarget(right, rightTarget_, rightPid_, rightOutput_, rightMotionAt_, now);
    active_ = true;
    return true;
  }

  bool update(const WheelTelemetry& feedback, uint32_t now) {
    if (!active_) return fault_ == VelocityFault::None;
    const VelocityFault badFeedback = feedbackFault(feedback, now);
    if (badFeedback != VelocityFault::None) return trip(badFeedback);
    if (feedback.leftInvalid != leftInvalid_ || feedback.rightInvalid != rightInvalid_) {
      return trip(VelocityFault::EncoderTransition);
    }

    // Observe total counts every loop, not just on PID samples. Repeating or
    // changing the magnitude of an RPM command must not renew these timers.
    if (progressInDirection(feedback.leftTicks, leftTicks_, leftTarget_)) leftMotionAt_ = now;
    if (progressInDirection(feedback.rightTicks, rightTicks_, rightTarget_)) rightMotionAt_ = now;
    leftTicks_ = feedback.leftTicks;
    rightTicks_ = feedback.rightTicks;
    if ((leftTarget_ != 0.0 && static_cast<uint32_t>(now - leftMotionAt_) >= config_.noMotionTimeoutMs) ||
        (rightTarget_ != 0.0 && static_cast<uint32_t>(now - rightMotionAt_) >= config_.noMotionTimeoutMs)) {
      return trip(VelocityFault::NoMotion);
    }
    if (feedback.sampleSequence == lastSequence_) return true;
    if (static_cast<uint32_t>(feedback.sampleSequence - lastSequence_) != 1U ||
        static_cast<uint32_t>(feedback.sampleAtMs - lastSampleAt_) != feedback.sampleMs) {
      return trip(VelocityFault::SampleTiming);
    }
    const double seconds = feedback.sampleMs / 1000.0;
    if (!leftPid_.step(config_.left, leftTarget_, feedback.leftRpm, seconds,
                       config_.maxPwmPercent, leftOutput_) ||
        !rightPid_.step(config_.right, rightTarget_, feedback.rightRpm, seconds,
                        config_.maxPwmPercent, rightOutput_)) {
      return trip(VelocityFault::Numeric);
    }
    lastSequence_ = feedback.sampleSequence;
    lastSampleAt_ = feedback.sampleAtMs;
    return true;
  }

  bool active() const { return active_; }
  bool faulted() const { return fault_ != VelocityFault::None; }
  VelocityFault fault() const { return fault_; }
  double leftTarget() const { return leftTarget_; }
  double rightTarget() const { return rightTarget_; }
  int leftPercent() const { return static_cast<int>(leftOutput_); }
  int rightPercent() const { return static_cast<int>(rightOutput_); }

  const char* faultName() const {
    switch (fault_) {
      case VelocityFault::None: return "none";
      case VelocityFault::Disabled: return "disabled";
      case VelocityFault::Configuration: return "configuration";
      case VelocityFault::TargetRange: return "target_range";
      case VelocityFault::FeedbackUnavailable: return "feedback_unavailable";
      case VelocityFault::StaleFeedback: return "stale_feedback";
      case VelocityFault::InvalidFeedback: return "invalid_feedback";
      case VelocityFault::EncoderTransition: return "encoder_transition";
      case VelocityFault::NoMotion: return "no_motion";
      case VelocityFault::SampleTiming: return "sample_timing";
      case VelocityFault::Numeric: return "numeric";
    }
    return "numeric";
  }

 private:
  VelocityConfig config_;
  WheelPid leftPid_;
  WheelPid rightPid_;
  VelocityFault fault_ = VelocityFault::None;
  bool active_ = false;
  double leftTarget_ = 0.0, rightTarget_ = 0.0;
  double leftOutput_ = 0.0, rightOutput_ = 0.0;
  uint32_t lastSequence_ = 0, lastSampleAt_ = 0;
  uint32_t leftMotionAt_ = 0, rightMotionAt_ = 0;
  uint32_t leftInvalid_ = 0, rightInvalid_ = 0;
  int64_t leftTicks_ = 0, rightTicks_ = 0;

  bool trip(VelocityFault fault) {
    stop();
    fault_ = fault;
    return false;
  }

  VelocityFault feedbackFault(const WheelTelemetry& f, uint32_t now) const {
    if (!f.enabled || !f.calibrated || f.sampleMs == 0) return VelocityFault::FeedbackUnavailable;
    if (static_cast<uint32_t>(now - f.sampleAtMs) >= config_.feedbackTimeoutMs) {
      return VelocityFault::StaleFeedback;
    }
    if (f.sampleMs >= config_.feedbackTimeoutMs) return VelocityFault::SampleTiming;
    if (!std::isfinite(f.leftRpm) || !std::isfinite(f.rightRpm) ||
        std::fabs(f.leftRpm) > config_.maxFeedbackRpm ||
        std::fabs(f.rightRpm) > config_.maxFeedbackRpm) return VelocityFault::InvalidFeedback;
    return VelocityFault::None;
  }

  static bool progressInDirection(int64_t current, int64_t previous, double target) {
    return target == 0.0 || (target > 0.0 ? current > previous : current < previous);
  }

  static void changeTarget(double next, double& previous, WheelPid& pid,
                           double& output, uint32_t& motionAt, uint32_t now) {
    if (next == 0.0 || previous == 0.0 || (next > 0.0) != (previous > 0.0)) {
      pid.reset();
      output = 0.0;
      if (previous == 0.0) motionAt = now;
    }
    previous = next;
  }
};

}  // namespace roboty
