#include <Arduino.h>

#include "comms/SerialCommandInterface.h"
#include "config/RobotConfig.h"
#include "control/DifferentialDrive.h"
#include "drivers/MotorDriver.h"

using roboty::DifferentialDrive;
using roboty::MotorDriver;
using roboty::SerialCommandInterface;

namespace {

MotorDriver leftMotor(
    roboty::config::LEFT_IN1_PIN,
    roboty::config::LEFT_IN2_PIN,
    roboty::config::LEFT_PWM_PIN,
    roboty::config::LEFT_PWM_CHANNEL,
    roboty::config::MOTOR_STBY_PIN,
    roboty::config::LEFT_INVERTED);

MotorDriver rightMotor(
    roboty::config::RIGHT_IN1_PIN,
    roboty::config::RIGHT_IN2_PIN,
    roboty::config::RIGHT_PWM_PIN,
    roboty::config::RIGHT_PWM_CHANNEL,
    roboty::config::MOTOR_STBY_PIN,
    roboty::config::RIGHT_INVERTED);

DifferentialDrive drive(leftMotor, rightMotor);

SerialCommandInterface commands(
    drive,
    roboty::config::COMMAND_WATCHDOG_MS,
    roboty::config::DEFAULT_DRIVE_PERCENT);

}  // namespace

void setup() {
  Serial.begin(roboty::config::SERIAL_BAUD);
  delay(250);

  leftMotor.begin(
      roboty::config::PWM_FREQUENCY_HZ,
      roboty::config::PWM_RESOLUTION_BITS);

  rightMotor.begin(
      roboty::config::PWM_FREQUENCY_HZ,
      roboty::config::PWM_RESOLUTION_BITS);

  drive.stop();
  commands.begin();

  Serial.println();
  Serial.println(F("Roboty M0 basic rover ready."));
  Serial.println(F("Fail-safe: motors stop if motion commands are not refreshed."));
  commands.printHelp();
}

void loop() {
  commands.poll();
  delay(1);
}
