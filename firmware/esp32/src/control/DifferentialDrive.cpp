#include "control/DifferentialDrive.h"

#include <algorithm>
#include <cmath>

namespace roboty {

DifferentialDrive::DifferentialDrive(
    MotorDriver& leftMotor,
    MotorDriver& rightMotor)
    : leftMotor_(leftMotor), rightMotor_(rightMotor) {}

void DifferentialDrive::setNormalized(float linear, float angular) {
  linear = clampUnit(linear);
  angular = clampUnit(angular);

  float left = linear - angular;
  float right = linear + angular;

  const float maxMagnitude =
      std::max(1.0F, std::max(std::fabs(left), std::fabs(right)));

  left /= maxMagnitude;
  right /= maxMagnitude;

  setMotorPercents(
      static_cast<int>(std::lround(left * 100.0F)),
      static_cast<int>(std::lround(right * 100.0F)));
}

void DifferentialDrive::setMotorPercents(int leftPercent, int rightPercent) {
  leftMotor_.setPercent(leftPercent);
  rightMotor_.setPercent(rightPercent);
}

void DifferentialDrive::forward(int percent) {
  setMotorPercents(percent, percent);
}

void DifferentialDrive::backward(int percent) {
  setMotorPercents(-percent, -percent);
}

void DifferentialDrive::turnLeft(int percent) {
  setMotorPercents(-percent, percent);
}

void DifferentialDrive::turnRight(int percent) {
  setMotorPercents(percent, -percent);
}

void DifferentialDrive::stop() {
  leftMotor_.stop();
  rightMotor_.stop();
}

int DifferentialDrive::leftPercent() const {
  return leftMotor_.percent();
}

int DifferentialDrive::rightPercent() const {
  return rightMotor_.percent();
}

float DifferentialDrive::clampUnit(float value) {
  return std::max(-1.0F, std::min(1.0F, value));
}

}  // namespace roboty
