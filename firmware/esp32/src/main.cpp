#include <Arduino.h>

#include "comms/SerialCommandInterface.h"
#include "config/RobotConfig.h"
#include "control/DifferentialDrive.h"
#include "control/WheelSpeedEstimator.h"
#include "drivers/MotorDriver.h"
#include "drivers/QuadratureEncoder.h"

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
roboty::WheelTelemetry telemetry;

roboty::QuadratureEncoder leftEncoder(
    roboty::config::LEFT_ENCODER_A_PIN, roboty::config::LEFT_ENCODER_B_PIN,
    roboty::config::LEFT_ENCODER_INVERTED, roboty::config::LEFT_ENCODER_PULLUP);
roboty::QuadratureEncoder rightEncoder(
    roboty::config::RIGHT_ENCODER_A_PIN, roboty::config::RIGHT_ENCODER_B_PIN,
    roboty::config::RIGHT_ENCODER_INVERTED, roboty::config::RIGHT_ENCODER_PULLUP);
roboty::WheelSpeedEstimator leftSpeed(
    roboty::config::LEFT_ENCODER_COUNTS_PER_REVOLUTION,
    roboty::config::ENCODER_SAMPLE_INTERVAL_MS);
roboty::WheelSpeedEstimator rightSpeed(
    roboty::config::RIGHT_ENCODER_COUNTS_PER_REVOLUTION,
    roboty::config::ENCODER_SAMPLE_INTERVAL_MS);

SerialCommandInterface commands(
    drive,
    roboty::config::COMMAND_WATCHDOG_MS,
    roboty::config::DEFAULT_DRIVE_PERCENT,
    telemetry);

void beginEncoders() {
  using namespace roboty::config;
  if (!ENCODERS_ENABLED) {
    Serial.println(F("Encoders disabled: configure GPIOs and enable in RobotConfig.h."));
    return;
  }
  if (!encoderPinsAvailable(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN,
                           RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN) ||
      !leftEncoder.begin() || !rightEncoder.begin()) {
    leftEncoder.end();
    rightEncoder.end();
    Serial.println(F("ERR encoder initialization failed; check GPIOs/pullups; encoders disabled"));
    return;
  }
  const uint32_t now = millis();
  leftSpeed.reset(leftEncoder.snapshot().ticks, now);
  rightSpeed.reset(rightEncoder.snapshot().ticks, now);
  telemetry.enabled = true;
  telemetry.calibrated = leftSpeed.calibrated() && rightSpeed.calibrated();
  if (!telemetry.calibrated) {
    Serial.println(F("Encoders enabled; RPM unavailable until both wheel CPR values are configured."));
  } else {
    Serial.println(F("Encoders enabled; verify direction and calibration on the bench."));
  }
}

void updateEncoders() {
  if (!telemetry.enabled) return;
  const roboty::EncoderSnapshot left = leftEncoder.snapshot();
  const roboty::EncoderSnapshot right = rightEncoder.snapshot();
  const uint32_t now = millis();
  telemetry.leftTicks = left.ticks;
  telemetry.rightTicks = right.ticks;
  telemetry.leftInvalid = left.invalidTransitions;
  telemetry.rightInvalid = right.invalidTransitions;
  leftSpeed.update(left.ticks, now);
  rightSpeed.update(right.ticks, now);
  telemetry.leftDelta = leftSpeed.deltaTicks();
  telemetry.rightDelta = rightSpeed.deltaTicks();
  telemetry.leftRpm = leftSpeed.rpm();
  telemetry.rightRpm = rightSpeed.rpm();
  telemetry.sampleMs = leftSpeed.sampleMs();
}

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
  beginEncoders();

  Serial.println();
  Serial.println(F("Roboty M1 encoder rover ready (open-loop motor control)."));
  Serial.println(F("Fail-safe: motors stop if motion commands are not refreshed."));
  commands.printHelp();
}

void loop() {
  updateEncoders();
  commands.poll();
  delay(1);
}
