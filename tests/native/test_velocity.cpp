#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "control/VelocityControl.h"

namespace {

using roboty::VelocityControl;
using roboty::VelocityFault;
using roboty::WheelTelemetry;

roboty::VelocityConfig configuration() {
  roboty::VelocityConfig c;
  c.enabled = true;
  c.left = {0.5, 0.1, 0.01, 0.1, 0.02};
  c.right = c.left;
  c.maxTargetRpm = 100;
  c.maxFeedbackRpm = 200;
  c.maxPwmPercent = 40;
  c.feedbackTimeoutMs = 300;
  c.noMotionTimeoutMs = 750;
  return c;
}

WheelTelemetry sample(uint32_t at = 100) {
  WheelTelemetry f;
  f.enabled = f.calibrated = true;
  f.sampleMs = 100;
  f.sampleSequence = 1;
  f.sampleAtMs = at;
  return f;
}

void advance(WheelTelemetry& f, int64_t left = 1, int64_t right = 1) {
  ++f.sampleSequence;
  f.sampleAtMs += f.sampleMs;
  f.leftTicks += left;
  f.rightTicks += right;
  f.leftDelta = left;
  f.rightDelta = right;
}

void pid_limits_and_antiwindup() {
  roboty::WheelPid pid;
  roboty::PidGains g{1, 1, 0, 0, .02};
  double out = 0;
  for (int i = 0; i < 1000; ++i) {
    assert(pid.step(g, 100, 0, .1, 25, out));
    assert(out == 25 && pid.integral() == 0);
  }
  assert(pid.step(g, 10, 10, .1, 25, out) && out == 0);
  g = {0, 100, 0, 0, .02};
  assert(pid.step(g, 10, 0, 1, 25, out) && out == 25);
  assert(pid.integral() == 25);
  assert(pid.step(g, 10, 20, 1, 25, out) && out == 0);
  assert(pid.integral() == 0);
}

void pid_direction_zero_and_derivative_on_measurement() {
  roboty::WheelPid pid;
  roboty::PidGains g{1, 0, 1, 0, .02};
  double out;
  assert(pid.step(g, 10, 0, .1, 50, out) && out == 10);
  assert(pid.step(g, 20, 0, .1, 50, out) && out == 20); // No setpoint derivative kick.
  assert(pid.step(g, 20, 21, .1, 50, out) && out == 0); // No reverse braking.
  assert(pid.step(g, 0, 100, .1, 50, out) && out == 0);
  assert(pid.step(g, -20, 0, .1, 50, out) && out == -20);
  assert(pid.step(g, -20, -100, .1, 50, out) && out == 0);
}

void pid_bad_inputs_and_overflow_fail_closed() {
  roboty::WheelPid pid;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  double out;
  auto g = configuration().left;
  for (double dt : {0., -1., nan, inf}) assert(!pid.step(g, 20, 0, dt, 50, out) && out == 0);
  for (double bad : {nan, inf, -inf}) {
    assert(!pid.step(g, bad, 0, .1, 50, out));
    assert(!pid.step(g, 20, bad, .1, 50, out));
  }
  g.kp = std::numeric_limits<double>::max();
  assert(!pid.step(g, 100, -100, .1, 50, out) && out == 0);
  g = {};
  assert(!pid.step(g, 10, 0, .1, 50, out));
}

void default_off_and_configuration_gates() {
  auto f = sample();
  VelocityControl off;
  assert(off.setTargets(0, 0, f, 100));
  assert(!off.setTargets(10, 10, f, 100) && off.fault() == VelocityFault::Disabled);
  for (int variant = 0; variant < 9; ++variant) {
    auto c = configuration();
    switch (variant) {
      case 0: c.maxTargetRpm = 0; break;
      case 1: c.left.kp = -1; break;
      case 2: c.right = {}; break;
      case 3: c.maxFeedbackRpm = 99; break;
      case 4: c.maxPwmPercent = 101; break;
      case 5: c.feedbackTimeoutMs = 0; break;
      case 6: c.noMotionTimeoutMs = 300; break;
      case 7: c.maxTargetRpm = std::numeric_limits<double>::infinity(); break;
      case 8: c.left.derivativeFilterSeconds = 0; break;
    }
    VelocityControl control(c);
    assert(!control.setTargets(10, 10, f, 100));
    assert(control.fault() == VelocityFault::Configuration);
  }
}

void output_waits_for_fresh_sample_and_duplicate_samples_do_not_integrate() {
  auto f = sample();
  VelocityControl c(configuration());
  assert(c.setTargets(50, -50, f, 100));
  assert(c.active() && c.leftPercent() == 0 && c.rightPercent() == 0);
  assert(c.update(f, 110) && c.leftPercent() == 0);
  advance(f, 1, -1);
  assert(c.update(f, 200));
  assert(c.leftPercent() > 0 && c.rightPercent() < 0);
  const int previous = c.leftPercent();
  for (uint32_t t = 201; t < 299; ++t) assert(c.update(f, t) && c.leftPercent() == previous);
}

void zero_reversal_stop_and_restart_clear_controller_state() {
  auto f = sample();
  VelocityControl c(configuration());
  assert(c.setTargets(50, 50, f, 100));
  advance(f); assert(c.update(f, 200));
  assert(c.leftPercent() > 0);
  assert(c.setTargets(-50, 0, f, 210));
  assert(c.leftPercent() == 0 && c.rightPercent() == 0);
  advance(f, -1, 0); assert(c.update(f, 300));
  assert(c.leftPercent() < 0 && c.rightPercent() == 0);
  assert(!c.clearFault()); // Cannot acknowledge while running, even without a fault.
  c.stop(); assert(!c.active() && c.leftTarget() == 0 && c.leftPercent() == 0);
  assert(c.setTargets(20, 20, f, 310) && c.leftPercent() == 0);
  assert(c.setTargets(0, 0, f, 310) && !c.active());
}

void feedback_failures_latch_and_never_auto_resume() {
  for (int variant = 0; variant < 8; ++variant) {
    auto f = sample();
    VelocityControl c(configuration());
    assert(c.setTargets(40, 40, f, 100));
    advance(f); assert(c.update(f, 200));
    auto bad = f;
    uint32_t now = 210;
    switch (variant) {
      case 0: bad.enabled = false; break;
      case 1: bad.calibrated = false; break;
      case 2: bad.leftRpm = std::numeric_limits<double>::quiet_NaN(); break;
      case 3: bad.rightRpm = 201; break;
      case 4: ++bad.leftInvalid; break;
      case 5: ++bad.rightInvalid; break;
      case 6: now = 500; break;
      case 7: bad.sampleMs = 300; break;
    }
    assert(!c.update(bad, now) && c.faulted());
    assert(!c.active() && c.leftPercent() == 0 && c.rightPercent() == 0);
    assert(!c.setTargets(40, 40, f, 210));
    c.stop(); assert(c.faulted());
    c.configure(configuration()); assert(c.faulted());
    assert(c.clearFault());
    assert(c.setTargets(40, 40, f, 210) && c.leftPercent() == 0);
  }
}

void repeated_targets_do_not_mask_no_motion() {
  auto f = sample();
  VelocityControl c(configuration());
  assert(c.setTargets(10, 0, f, 100));
  for (uint32_t t = 200; t <= 800; t += 100) {
    advance(f, 0, 0);
    assert(c.update(f, t));
    assert(c.setTargets(t == 800 ? 20 : 10, 0, f, t));
  }
  assert(!c.update(f, 850) && c.fault() == VelocityFault::NoMotion);
}

void either_wheel_no_motion_and_tick_progress() {
  for (bool frozenLeft : {false, true}) {
    auto f = sample();
    VelocityControl c(configuration());
    assert(c.setTargets(20, 20, f, 100));
    for (uint32_t t = 200; t <= 800; t += 100) {
      advance(f, frozenLeft ? 0 : 1, frozenLeft ? 1 : 0);
      assert(c.update(f, t));
    }
    assert(!c.update(f, 850) && c.fault() == VelocityFault::NoMotion);
  }
  auto f = sample();
  VelocityControl c(configuration());
  assert(c.setTargets(20, 20, f, 100));
  for (uint32_t t = 200; t <= 5000; t += 100) {
    advance(f); assert(c.update(f, t));
  }
}

void skipped_backwards_and_inconsistent_samples_are_faults() {
  for (int variant = 0; variant < 3; ++variant) {
    auto f = sample();
    VelocityControl c(configuration());
    assert(c.setTargets(20, 20, f, 100));
    advance(f);
    if (variant == 0) ++f.sampleSequence;
    if (variant == 1) f.sampleSequence = 0;
    if (variant == 2) f.sampleMs = 99;
    assert(!c.update(f, 200) && c.fault() == VelocityFault::SampleTiming);
  }
}

void millisecond_and_sample_sequence_wrap() {
  auto f = sample(UINT32_MAX - 49U);
  f.sampleSequence = UINT32_MAX;
  VelocityControl c(configuration());
  assert(c.setTargets(20, 20, f, f.sampleAtMs));
  advance(f);
  assert(f.sampleAtMs == 50 && f.sampleSequence == 0);
  assert(c.update(f, 50) && c.leftPercent() > 0);
  assert(c.update(f, 349));
  assert(!c.update(f, 350) && c.fault() == VelocityFault::StaleFeedback);
}

void target_range_and_fractional_pwm_limit() {
  auto f = sample();
  for (double target : {101., -101., std::numeric_limits<double>::quiet_NaN()}) {
    VelocityControl c(configuration());
    assert(!c.setTargets(target, 0, f, 100) && c.fault() == VelocityFault::TargetRange);
  }
  auto cfg = configuration(); cfg.maxPwmPercent = 10.5;
  VelocityControl c(cfg);
  assert(c.setTargets(100, -100, f, 100));
  advance(f);
  assert(c.update(f, 200) && c.leftPercent() == 10 && c.rightPercent() == -10);
}

void independent_wheel_gains() {
  auto cfg = configuration();
  cfg.left = {1, 0, 0, 0, .02};
  cfg.right = {.5, 0, 0, 0, .02};
  VelocityControl c(cfg);
  auto f = sample();
  assert(c.setTargets(20, 20, f, 100));
  advance(f); assert(c.update(f, 200));
  assert(c.leftPercent() == 20 && c.rightPercent() == 10);
}

void deterministic_plant_tracks_target_with_bounded_output() {
  auto cfg = configuration();
  cfg.left = {.7, 1.5, 0, .4, .02}; cfg.right = cfg.left;
  cfg.maxPwmPercent = 60;
  VelocityControl c(cfg);
  auto f = sample();
  assert(c.setTargets(40, -25, f, 100));
  for (int n = 0; n < 200; ++n) {
    // Synthetic first-order plant; this is not a motor calibration.
    f.leftRpm += .2 * (2.0 * c.leftPercent() - f.leftRpm);
    f.rightRpm += .2 * (2.0 * c.rightPercent() - f.rightRpm);
    advance(f, n == 0 ? 0 : 1, n == 0 ? 0 : -1);
    assert(c.update(f, f.sampleAtMs));
    assert(c.leftPercent() >= 0 && c.leftPercent() <= 60);
    assert(c.rightPercent() <= 0 && c.rightPercent() >= -60);
  }
  assert(std::fabs(f.leftRpm - 40) < 2);
  assert(std::fabs(f.rightRpm + 25) < 2);
}

}  // namespace

int main() {
  pid_limits_and_antiwindup();
  pid_direction_zero_and_derivative_on_measurement();
  pid_bad_inputs_and_overflow_fail_closed();
  default_off_and_configuration_gates();
  output_waits_for_fresh_sample_and_duplicate_samples_do_not_integrate();
  zero_reversal_stop_and_restart_clear_controller_state();
  feedback_failures_latch_and_never_auto_resume();
  repeated_targets_do_not_mask_no_motion();
  either_wheel_no_motion_and_tick_progress();
  skipped_backwards_and_inconsistent_samples_are_faults();
  millisecond_and_sample_sequence_wrap();
  target_range_and_fractional_pwm_limit();
  independent_wheel_gains();
  deterministic_plant_tracks_target_with_bounded_output();
  std::cout << "velocity/PID tests passed (14 groups)\n";
}
