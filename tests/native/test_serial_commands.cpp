#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "comms/SerialCommandInterface.h"
#include "config/RobotConfig.h"

#define CHECK(condition) do { if (!(condition)) throw std::runtime_error( \
    std::string(__func__) + ":" + std::to_string(__LINE__) + " " #condition); } while (false)

struct Rover {
  roboty::MotorDriver left{26, 27, 25, 0, 33};
  roboty::MotorDriver right{18, 19, 23, 1, 33, true};
  roboty::DifferentialDrive drive{left, right};
  roboty::WheelTelemetry telemetry;
  roboty::SerialCommandInterface commands{drive, 750, 45, telemetry};

  Rover() {
    Serial = TestSerial{};
    testMillis = 0;
    testDuty.fill(0);
    left.begin(20000, 8);
    right.begin(20000, 8);
    commands.begin();
  }
  void send(const std::string& command) {
    Serial.input += command + '\n';
    while (Serial.available() > 0) commands.poll();
  }
  void expect(int leftPercent, int rightPercent) const {
    CHECK(drive.leftPercent() == leftPercent);
    CHECK(drive.rightPercent() == rightPercent);
    if (leftPercent == 0) CHECK(testDuty[0] == 0);
    if (rightPercent == 0) CHECK(testDuty[1] == 0);
  }
};

void commands_and_motor_outputs() {
  Rover rover;
  rover.send("f"); rover.expect(45, 45);
  CHECK(testPins[26] == HIGH && testPins[27] == LOW);
  CHECK(testPins[18] == LOW && testPins[19] == HIGH);
  rover.send("B"); rover.expect(-45, -45);
  rover.send("L"); rover.expect(-45, 45);
  rover.send("R"); rover.expect(45, -45);
  rover.send("  fwd\t20\r"); rover.expect(20, 20);
  rover.send("BACK 30"); rover.expect(-30, -30);
  rover.send("LEFT 25"); rover.expect(-25, 25);
  rover.send("RIGHT 15"); rover.expect(15, -15);
  rover.send("DRIVE 1 1"); rover.expect(0, 100);
  rover.send("DRIVE 1 -1"); rover.expect(100, 0);
  rover.send("MOTOR -100 100"); rover.expect(-100, 100);
  rover.send("S"); rover.expect(0, 0);
  rover.send("FWD 0"); rover.expect(0, 0);
  CHECK(Serial.output.find("active=no") != std::string::npos);
}

void invalid_commands_stop_instead_of_using_default_speed() {
  Rover rover;
  const std::vector<std::string> invalid{
      "FWD abc", "FWD -1", "FWD 101", "FWD 5 6", "BACK 2.5", "LEFT +",
      "RIGHT 2147483648", "MOTOR 4294967297 20", "MOTOR 1 -4294967296",
      "MOTOR 1 2 extra", "MOTOR 1", "MOTOR 1 101", "MOTOR -101 1",
      "DRIVE nan 0", "DRIVE 0 inf", "DRIVE 1.01 0", "DRIVE 0 -1.01",
      "DRIVE .5 .2 extra", "DRIVE 0", "DRIVE 1e999 0", "FWDXYZ 10",
      "STOP extra", "STATUS extra", "HELP extra", "? extra", "UNKNOWN",
      "TELEMETRY -1", "TELEMETRY 49", "TELEMETRY 5001", "TELEMETRY NaN",
      "TELEMETRY 100 extra", "MOTOR 99999999999999999999999999 0"};
  for (const auto& command : invalid) {
    rover.send("FWD 30");
    Serial.output.clear();
    rover.send(command);
    rover.expect(0, 0);
    CHECK(Serial.output.find("ERR ") == 0);
  }
}

void oversized_and_binary_lines_discard_the_entire_tail() {
  Rover rover;
  rover.send("FWD 40");
  Serial.output.clear();
  rover.send(std::string(96, 'x') + "FWD 100");
  rover.expect(0, 0);
  CHECK(Serial.output.find("command too long") != std::string::npos);
  CHECK(Serial.output.find("active=yes") == std::string::npos);
  rover.send("FWD 10"); rover.expect(10, 10);
  rover.send(std::string("FWD 20\0FWD 100", 14)); rover.expect(0, 0);
  rover.send("FWD 10"); rover.expect(10, 10);
  rover.send(std::string(1, static_cast<char>(0xff)) + "FWD 100"); rover.expect(0, 0);
  rover.send("FWD 15"); rover.expect(15, 15);
}

void watchdog_expiry_queries_and_partial_lines() {
  Rover rover;
  rover.send("FWD 40");
  testMillis = 400;
  rover.send("STATUS");
  rover.send("HELP");
  rover.send("TELEMETRY");
  rover.send("TELEMETRY 50");
  testMillis = 749;
  rover.commands.poll(); rover.expect(40, 40);
  Serial.input += "FWD 90";
  testMillis = 750;
  rover.commands.poll(); rover.expect(0, 0);
  CHECK(Serial.output.find("WATCHDOG stop") != std::string::npos);
}

void watchdog_clock_wrap_and_motion_refresh() {
  Rover rover;
  const uint32_t start = std::numeric_limits<uint32_t>::max() - 300;
  testMillis = start;
  rover.send("FWD 35");
  testMillis = start + 749;
  rover.commands.poll(); rover.expect(35, 35);
  testMillis = start + 750;
  rover.commands.poll(); rover.expect(0, 0);
  rover.send("FWD 25");
  testMillis += 700;
  rover.send("FWD 25");
  testMillis += 700;
  rover.commands.poll(); rover.expect(25, 25);
  testMillis += 50;
  rover.commands.poll(); rover.expect(0, 0);
}

void serial_flood_is_bounded_and_does_not_starve_watchdog() {
  Rover rover;
  rover.send("FWD 40");
  testMillis = 749;
  Serial.readTimeMs = 1;
  Serial.input += std::string(10000, ' ');
  const int before = Serial.available();
  rover.commands.poll();
  rover.expect(0, 0);
  CHECK(before - Serial.available() <= 64);
  CHECK(Serial.available() > 0);
}

void telemetry_stream_bounds_and_unavailable_rpm() {
  Rover rover;
  rover.send("TELEMETRY");
  CHECK(Serial.output.find("\"encoders\":false") != std::string::npos);
  CHECK(Serial.output.find("\"left_rpm\":null") != std::string::npos);
  rover.send("TELEMETRY 50");
  Serial.output.clear();
  testMillis = 49;
  rover.commands.poll(); CHECK(Serial.output.empty());
  testMillis = 50;
  rover.commands.poll(); CHECK(Serial.output.find("{\"type\":\"telemetry\"") == 0);
  rover.send("TELEMETRY 0");
  Serial.output.clear();
  testMillis = 10000;
  rover.commands.poll(); CHECK(Serial.output.empty());
  rover.telemetry.enabled = true;
  rover.telemetry.calibrated = true;
  rover.telemetry.leftRpm = 60.0;
  rover.send("TELEMETRY");
  CHECK(Serial.output.find("\"left_rpm\":null") != std::string::npos);
  Serial.output.clear();
  rover.telemetry.sampleMs = 100;
  rover.telemetry.leftRpm = std::numeric_limits<double>::infinity();
  rover.send("TELEMETRY");
  CHECK(Serial.output.find("\"left_rpm\":null") != std::string::npos);
  rover.send("TELEMETRY 5000");
  CHECK(Serial.output.find("OK telemetry_interval_ms=5000") != std::string::npos);
}

void encoder_pins_preserve_motor_uart_and_boot_configuration() {
  using roboty::config::encoderPinsAvailable;
  CHECK(!roboty::config::ENCODERS_ENABLED);
  CHECK(encoderPinsAvailable(21, 22, 34, 35));
  CHECK(!encoderPinsAvailable(-1, 22, 34, 35));
  CHECK(!encoderPinsAvailable(21, 22, 21, 35));
  for (const int8_t pin : {0, 1, 2, 3, 5, 12, 15, 18, 19, 23, 25, 26, 27, 33}) {
    CHECK(!encoderPinsAvailable(pin, 22, 34, 35));
  }
}

void emit_telemetry() {
  Rover rover;
  rover.send("TELEMETRY");
  rover.telemetry.enabled = true;
  rover.telemetry.calibrated = true;
  rover.telemetry.leftTicks = std::numeric_limits<int64_t>::max();
  rover.telemetry.rightTicks = std::numeric_limits<int64_t>::min();
  rover.telemetry.leftDelta = 100;
  rover.telemetry.rightDelta = -100;
  rover.telemetry.sampleMs = 100;
  rover.telemetry.leftRpm = 60.25;
  rover.telemetry.rightRpm = -60.25;
  rover.send("TELEMETRY");
  rover.telemetry.leftRpm = 1e200;
  rover.send("TELEMETRY");
  std::cout << Serial.output;
}

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--emit-telemetry") {
    emit_telemetry();
    return 0;
  }
  const std::vector<std::pair<const char*, void (*)()>> tests{
      {"motion commands and PWM", commands_and_motor_outputs},
      {"invalid commands stop", invalid_commands_stop_instead_of_using_default_speed},
      {"overflow and binary tails", oversized_and_binary_lines_discard_the_entire_tail},
      {"watchdog, queries and partial lines", watchdog_expiry_queries_and_partial_lines},
      {"watchdog wrap and refresh", watchdog_clock_wrap_and_motion_refresh},
      {"bounded input flood", serial_flood_is_bounded_and_does_not_starve_watchdog},
      {"telemetry stream and null RPM", telemetry_stream_bounds_and_unavailable_rpm},
      {"encoder pin conflicts", encoder_pins_preserve_motor_uart_and_boot_configuration}};
  try {
    for (const auto& test : tests) {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  std::cout << tests.size() << " serial/control tests passed\n";
}
