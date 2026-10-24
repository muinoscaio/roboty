#include "control/DifferentialDrive.h"

#include <algorithm>
#include <cmath>

namespace roboty {

DifferentialDrive::DifferentialDrive(
    MotorDriver& leftMotor,
    MotorDriver& rightMotor)
    : leftMotor_(leftMotor), rightMotor_(rightMotor) {}

void DifferentialDrive::setNormalized(float linear, float angular) {
  if (!std::isfinite(linear) || !std::isfinite(angular)) {
    stop();
    return;
  }
  linear = clampUnit(linear);
  angular = clampUnit(angular);
  float left = linear - angular;
  float right = linear + angular;
  const float maxMagnitude =
      std::max(1.0F, std::max(std::fabs(left), std::fabs(right)));
  setMotorPercents(
      static_cast<int>(std::lround(left / maxMagnitude * 100.0F)),
      static_cast<int>(std::lround(right / maxMagnitude * 100.0F)));
}

void DifferentialDrive::setMotorPercents(int leftPercent, int rightPercent) {
  velocity_.stop();
  // A latched feedback fault also blocks raw commands until acknowledged.
  leftMotor_.setPercent(faulted() ? 0 : leftPercent);
  rightMotor_.setPercent(faulted() ? 0 : rightPercent);
}

void DifferentialDrive::forward(int percent) { setMotorPercents(percent, percent); }
void DifferentialDrive::backward(int percent) { setMotorPercents(-percent, -percent); }
void DifferentialDrive::turnLeft(int percent) { setMotorPercents(-percent, percent); }
void DifferentialDrive::turnRight(int percent) { setMotorPercents(percent, -percent); }

void DifferentialDrive::stop() {
  velocity_.stop();
  leftMotor_.stop();
  rightMotor_.stop();
}

void DifferentialDrive::configureVelocity(const VelocityConfig& config, const WheelTelemetry& telemetry) {
  stop();
  telemetry_ = &telemetry;
  velocity_.configure(config);
}

bool DifferentialDrive::setWheelRpms(double left, double right, uint32_t now) {
  const WheelTelemetry unavailable;
  const bool accepted = velocity_.setTargets(left, right, telemetry_ ? *telemetry_ : unavailable, now);
  applyVelocityOutputs();
  return accepted;
}

void DifferentialDrive::updateVelocity(uint32_t now) {
  if (!velocity_.active()) return;
  const WheelTelemetry unavailable;
  velocity_.update(telemetry_ ? *telemetry_ : unavailable, now);
  applyVelocityOutputs();
}

void DifferentialDrive::applyVelocityOutputs() {
  leftMotor_.setPercent(velocity_.leftPercent());
  rightMotor_.setPercent(velocity_.rightPercent());
}

bool DifferentialDrive::clearFault() {
  if (motionRequested()) return false;
  return velocity_.clearFault();
}

bool DifferentialDrive::faulted() const { return velocity_.faulted(); }
bool DifferentialDrive::motionRequested() const {
  return velocity_.active() || leftPercent() != 0 || rightPercent() != 0;
}
const char* DifferentialDrive::modeName() const {
  return velocity_.active() ? "velocity" : motionRequested() ? "open_loop" : "stopped";
}
const char* DifferentialDrive::faultName() const { return velocity_.faultName(); }
double DifferentialDrive::leftTargetRpm() const { return velocity_.leftTarget(); }
double DifferentialDrive::rightTargetRpm() const { return velocity_.rightTarget(); }
int DifferentialDrive::leftPercent() const { return leftMotor_.percent(); }
int DifferentialDrive::rightPercent() const { return rightMotor_.percent(); }
float DifferentialDrive::clampUnit(float value) { return std::max(-1.0F, std::min(1.0F, value)); }

}  // namespace roboty
