#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

#include "control/QuadratureDecoder.h"
#include "control/WheelSpeedEstimator.h"

namespace {

void expectNear(double actual, double expected) {
  assert(std::fabs(actual - expected) < 0.000001);
}

void quadratureCountsEveryEdge() {
  roboty::QuadratureDecoder decoder;
  decoder.reset(0);
  for (const uint8_t state : {1, 3, 2, 0}) {
    decoder.update(state);
  }
  assert(decoder.ticks() == 4);
  assert(decoder.invalidTransitions() == 0);
  for (const uint8_t state : {2, 3, 1, 0}) {
    decoder.update(state);
  }
  assert(decoder.ticks() == 0);
  for (const uint8_t state : {2, 3, 1, 0}) {
    decoder.update(state);
  }
  assert(decoder.ticks() == -4);
}

void quadratureInversionAndInitialPhase() {
  roboty::QuadratureDecoder decoder(true);
  decoder.reset(3);
  for (const uint8_t state : {2, 0, 1, 3}) {
    decoder.update(state);
  }
  assert(decoder.ticks() == -4);
  for (const uint8_t state : {1, 0, 2, 3}) {
    decoder.update(state);
  }
  assert(decoder.ticks() == 0);
}

void quadratureRejectsInvalidTransitionsAndRecovers() {
  roboty::QuadratureDecoder decoder;
  decoder.reset(0);
  decoder.update(0);
  assert(decoder.ticks() == 0);
  assert(decoder.invalidTransitions() == 0);
  decoder.update(3);
  assert(decoder.ticks() == 0);
  assert(decoder.invalidTransitions() == 1);
  decoder.update(2);
  assert(decoder.ticks() == 1);
  decoder.update(3);  // Reverse bounce cancels the preceding edge.
  assert(decoder.ticks() == 0);
  decoder.update(0);
  assert(decoder.invalidTransitions() == 2);
  decoder.reset(2);
  assert(decoder.ticks() == 0);
  assert(decoder.invalidTransitions() == 0);
  decoder.update(0);
  assert(decoder.ticks() == 1);
}

void quadratureDoesNotOverflow() {
  roboty::QuadratureDecoder decoder;
  decoder.reset(0, INT64_MAX);
  decoder.update(1);
  assert(decoder.ticks() == INT64_MAX);
  decoder.update(0);
  assert(decoder.ticks() == INT64_MAX - 1);
  decoder.reset(0, INT64_MIN);
  decoder.update(2);
  assert(decoder.ticks() == INT64_MIN);
  decoder.update(0);
  assert(decoder.ticks() == INT64_MIN + 1);
}

void rpmUsesActualElapsedTimeAndSignedTicks() {
  roboty::WheelSpeedEstimator speed(1200.0F, 100);
  assert(speed.calibrated());
  speed.reset(100, 1000);
  assert(!speed.update(180, 1099));
  assert(speed.update(220, 1100));
  assert(speed.deltaTicks() == 120);
  assert(speed.sampleMs() == 100);
  expectNear(speed.rpm(), 60.0);
  assert(speed.update(520, 1350));
  assert(speed.deltaTicks() == 300);
  assert(speed.sampleMs() == 250);
  expectNear(speed.rpm(), 60.0);
  assert(speed.update(400, 1450));
  expectNear(speed.rpm(), -60.0);
  assert(speed.update(400, 1550));
  expectNear(speed.rpm(), 0.0);
}

void rpmHandlesTimerWrapAndReset() {
  roboty::WheelSpeedEstimator speed(100.0F, 100);
  speed.reset(0, UINT32_MAX - 49U);
  assert(!speed.update(10, 49U));
  assert(speed.update(10, 50U));
  assert(speed.sampleMs() == 100);
  expectNear(speed.rpm(), 60.0);
  speed.reset(-5000, 60);
  assert(speed.deltaTicks() == 0);
  assert(speed.sampleMs() == 0);
  expectNear(speed.rpm(), 0.0);
  assert(speed.update(-5010, 160));
  expectNear(speed.rpm(), -60.0);
}

void rpmDoesNotInventCalibration() {
  const float invalidCpr[] = {0.0F, -1.0F,
                              std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::quiet_NaN()};
  for (const float cpr : invalidCpr) {
    roboty::WheelSpeedEstimator speed(cpr, 100);
    assert(!speed.calibrated());
    speed.reset(0, 0);
    assert(speed.update(120, 100));
    assert(speed.deltaTicks() == 120);
    assert(speed.sampleMs() == 100);
    expectNear(speed.rpm(), 0.0);
  }
}

void rpmInitializesOnFirstUpdateAndAvoidsZeroDuration() {
  roboty::WheelSpeedEstimator speed(100.0F, 0);
  assert(!speed.update(1000, 50));
  assert(!speed.update(1001, 50));
  assert(speed.update(1001, 51));
  expectNear(speed.rpm(), 600.0);
}

void rpmDeltaNeverHasSignedOverflow() {
  roboty::WheelSpeedEstimator speed(1.0F, 1);
  speed.reset(INT64_MIN, 0);
  assert(speed.update(INT64_MAX, 1));
  assert(speed.deltaTicks() == INT64_MAX);
  assert(std::isfinite(speed.rpm()));
  assert(speed.rpm() > 0.0);
  assert(speed.update(INT64_MIN, 2));
  assert(speed.deltaTicks() == INT64_MIN);
  assert(std::isfinite(speed.rpm()));
  assert(speed.rpm() < 0.0);
  speed.reset(INT64_MIN, 2);
  assert(speed.update(INT64_MIN + 10, 3));
  assert(speed.deltaTicks() == 10);
}

}  // namespace

int main() {
  quadratureCountsEveryEdge();
  quadratureInversionAndInitialPhase();
  quadratureRejectsInvalidTransitionsAndRecovers();
  quadratureDoesNotOverflow();
  rpmUsesActualElapsedTimeAndSignedTicks();
  rpmHandlesTimerWrapAndReset();
  rpmDoesNotInventCalibration();
  rpmInitializesOnFirstUpdateAndAvoidsZeroDuration();
  rpmDeltaNeverHasSignedOverflow();
  std::cout << "encoder/RPM tests passed (9 cases)\n";
}
