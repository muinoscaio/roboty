#pragma once

#include <Arduino.h>

namespace roboty::config {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t COMMAND_WATCHDOG_MS = 750;
constexpr int DEFAULT_DRIVE_PERCENT = 45;

constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;

// TB6612FNG shared standby pin.
constexpr uint8_t MOTOR_STBY_PIN = 33;

// Left motor.
constexpr uint8_t LEFT_IN1_PIN = 26;
constexpr uint8_t LEFT_IN2_PIN = 27;
constexpr uint8_t LEFT_PWM_PIN = 25;
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr bool LEFT_INVERTED = false;

// Right motor. The default inversion assumes a typical mirrored 2WD chassis.
// If your rover moves in circles when asked to go forward, verify wiring first
// and then change this value if required.
constexpr uint8_t RIGHT_IN1_PIN = 18;
constexpr uint8_t RIGHT_IN2_PIN = 19;
constexpr uint8_t RIGHT_PWM_PIN = 23;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;
constexpr bool RIGHT_INVERTED = true;

}  // namespace roboty::config
