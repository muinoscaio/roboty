#pragma once

#include "drivers/MotorDriver.h"
#include "control/VelocityControl.h"

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

  void configureVelocity(const VelocityConfig& config, const WheelTelemetry& telemetry);
  bool setWheelRpms(double left, double right, uint32_t now);
  void updateVelocity(uint32_t now);
  bool clearFault();
  bool faulted() const;
  bool motionRequested() const;
  const char* modeName() const;
  const char* faultName() const;
  double leftTargetRpm() const;
  double rightTargetRpm() const;
  int leftPercent() const;
  int rightPercent() const;

 private:
  MotorDriver& leftMotor_;
  MotorDriver& rightMotor_;
  VelocityControl velocity_;
  const WheelTelemetry* telemetry_ = nullptr;

  void applyVelocityOutputs();
  static float clampUnit(float value);
};

}  // namespace roboty
