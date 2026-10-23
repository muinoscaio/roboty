#pragma once

#include <cstddef>
#include <cstdint>

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

// Configure only after identifying the encoder voltage/output and free GPIOs.
constexpr bool ENCODERS_ENABLED = false;
constexpr int8_t LEFT_ENCODER_A_PIN = -1;
constexpr int8_t LEFT_ENCODER_B_PIN = -1;
constexpr int8_t RIGHT_ENCODER_A_PIN = -1;
constexpr int8_t RIGHT_ENCODER_B_PIN = -1;
constexpr bool LEFT_ENCODER_INVERTED = false;
constexpr bool RIGHT_ENCODER_INVERTED = false;
constexpr bool LEFT_ENCODER_PULLUP = false;
constexpr bool RIGHT_ENCODER_PULLUP = false;

// Quadrature x4 counts per complete wheel revolution, including the gearbox.
// Zero means unknown: tick telemetry works, but RPM is reported as null.
constexpr float LEFT_ENCODER_COUNTS_PER_REVOLUTION = 0.0F;
constexpr float RIGHT_ENCODER_COUNTS_PER_REVOLUTION = 0.0F;
constexpr uint32_t ENCODER_SAMPLE_INTERVAL_MS = 100;

inline bool encoderPinsAvailable(int8_t leftA, int8_t leftB, int8_t rightA, int8_t rightB) {
  const int8_t pins[] = {leftA, leftB, rightA, rightB};
  const uint8_t reserved[] = {
      MOTOR_STBY_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_PIN,
      RIGHT_IN1_PIN, RIGHT_IN2_PIN, RIGHT_PWM_PIN,
      0, 1, 2, 3, 5, 12, 15};  // Boot straps and USB/UART0 remain reserved.
  for (size_t i = 0; i < 4; ++i) {
    if (pins[i] < 0) return false;
    for (const uint8_t pin : reserved) {
      if (pins[i] == pin) return false;
    }
    for (size_t j = 0; j < i; ++j) {
      if (pins[i] == pins[j]) return false;
    }
  }
  return true;
}

static_assert(ENCODER_SAMPLE_INTERVAL_MS > 0, "Encoder sample interval must be positive");

}  // namespace roboty::config
