#pragma once

#include "drivers/MotorDriver.h"

namespace roboty {

class DifferentialDrive {
 public:
  DifferentialDrive(MotorDriver& leftMotor, MotorDriver& rightMotor);

  void setNormalized(float linear, float angular);
  void setMotorPercents(int leftPercent, int rightPercent);
  void forward(int percent);
  void backward(int percent);
  void turnLeft(int percent);
  void turnRight(int percent);
  void stop();

  int leftPercent() const;
  int rightPercent() const;

 private:
  MotorDriver& leftMotor_;
  MotorDriver& rightMotor_;

  static float clampUnit(float value);
};

}  // namespace roboty
