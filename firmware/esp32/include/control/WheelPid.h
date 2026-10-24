#pragma once

#include <algorithm>
#include <cmath>

namespace roboty {

struct PidGains {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double feedforward = 0.0;
  double derivativeFilterSeconds = 0.02;

  bool valid() const {
    return std::isfinite(kp) && kp >= 0.0 &&
           std::isfinite(ki) && ki >= 0.0 &&
           std::isfinite(kd) && kd >= 0.0 &&
           std::isfinite(feedforward) && feedforward >= 0.0 &&
           std::isfinite(derivativeFilterSeconds) && derivativeFilterSeconds > 0.0 &&
           (kp > 0.0 || ki > 0.0);
  }
};

// Output units are signed PWM percent; target and measurement units are wheel RPM.
class WheelPid {
 public:
  void reset() {
    integral_ = 0.0;
    derivative_ = 0.0;
    previousMeasurement_ = 0.0;
    initialized_ = false;
  }

  bool step(const PidGains& gains, double target, double measured,
            double seconds, double limit, double& output) {
    output = 0.0;
    if (!gains.valid() || !std::isfinite(target) || !std::isfinite(measured) ||
        !std::isfinite(seconds) || seconds <= 0.0 ||
        !std::isfinite(limit) || limit <= 0.0 || limit > 100.0) {
      reset();
      return false;
    }
    if (target == 0.0) {
      reset();
      return true;
    }

    // No reverse braking torque for a nonzero target. A new target direction
    // is handled by the supervisor, which first clears the wheel's output.
    const double minimum = target > 0.0 ? 0.0 : -limit;
    const double maximum = target > 0.0 ? limit : 0.0;
    const double error = target - measured;
    double filteredDerivative = 0.0;
    if (initialized_ && gains.kd > 0.0) {
      const double raw = (measured - previousMeasurement_) / seconds;
      const double alpha = seconds / (gains.derivativeFilterSeconds + seconds);
      filteredDerivative = derivative_ + alpha * (raw - derivative_);
    }
    const double base = gains.feedforward * target + gains.kp * error -
                        gains.kd * filteredDerivative;
    const double increment = gains.ki * error * seconds;
    const double proposedIntegral = integral_ + increment;
    if (!std::isfinite(error) || !std::isfinite(base) ||
        !std::isfinite(filteredDerivative) || !std::isfinite(proposedIntegral)) {
      reset();
      return false;
    }
    double candidate = std::max(-limit, std::min(limit, proposedIntegral));
    const double currentOutput = base + integral_;
    const double proposedOutput = base + candidate;
    if (!std::isfinite(currentOutput) || !std::isfinite(proposedOutput)) {
      reset();
      return false;
    }
    // Allow integration up to the saturation boundary, including an I-only
    // first step. Once saturated, only an increment that unwinds is accepted.
    if ((currentOutput >= minimum && currentOutput <= maximum) ||
        (currentOutput > maximum && increment < 0.0) ||
        (currentOutput < minimum && increment > 0.0)) {
      if (increment > 0.0 && proposedOutput > maximum) candidate = maximum - base;
      if (increment < 0.0 && proposedOutput < minimum) candidate = minimum - base;
      integral_ = std::max(-limit, std::min(limit, candidate));
    }
    const double rawOutput = base + integral_;
    if (!std::isfinite(rawOutput)) {
      reset();
      return false;
    }
    output = std::max(minimum, std::min(maximum, rawOutput));
    derivative_ = filteredDerivative;
    previousMeasurement_ = measured;
    initialized_ = true;
    return true;
  }

  double integral() const { return integral_; }

 private:
  double integral_ = 0.0;
  double derivative_ = 0.0;
  double previousMeasurement_ = 0.0;
  bool initialized_ = false;
};

}  // namespace roboty
