#include <cassert>
#include <iostream>
#include <string>
#include <vector>

#include "comms/SerialCommandInterface.h"
#include "config/VelocityConfig.h"

struct Rover {
  roboty::MotorDriver left{26, 27, 25, 0, 33};
  roboty::MotorDriver right{18, 19, 23, 1, 33, true};
  roboty::DifferentialDrive drive{left, right};
  roboty::WheelTelemetry feedback;
  roboty::SerialCommandInterface commands{drive, 750, 45, feedback};

  explicit Rover(bool enabled = true, uint32_t now = 100) {
    Serial = TestSerial{}; testMillis = now; testDuty.fill(0);
    left.begin(20000, 8); right.begin(20000, 8);
    auto config = roboty::config::velocityConfig();
    assert(!config.enabled && !config.valid());
    if (enabled) {
      config.enabled = true;
      config.left = {1, 0, 0, 0, .02}; config.right = config.left;
      config.maxTargetRpm = 100; config.maxFeedbackRpm = 200; config.maxPwmPercent = 40;
      config.noMotionTimeoutMs = 1500;
    }
    feedback.enabled = feedback.calibrated = true;
    feedback.sampleMs = 100; feedback.sampleSequence = 1; feedback.sampleAtMs = now;
    drive.configureVelocity(config, feedback);
    commands.begin();
  }
  void send(const std::string& text) {
    Serial.input += text + '\n';
    while (Serial.available() > 0) commands.poll();
  }
  void sample(uint32_t now, bool moving = true) {
    feedback.sampleMs = now - feedback.sampleAtMs;
    feedback.sampleAtMs = now; ++feedback.sampleSequence;
    if (moving) { ++feedback.leftTicks; ++feedback.rightTicks; }
    testMillis = now; commands.poll();
  }
  void stopped() const {
    assert(!drive.motionRequested() && drive.leftPercent() == 0 && drive.rightPercent() == 0);
    assert(testDuty[0] == 0 && testDuty[1] == 0);
  }
};

void default_disabled_and_explicit_recovery() {
  Rover r(false);
  r.send("FWD 20"); assert(r.drive.leftPercent() == 20);
  r.send("RPM 20 20"); r.stopped(); assert(r.drive.faulted());
  r.send("FWD 20"); r.stopped();
  r.send("STOP"); assert(r.drive.faulted());
  r.send("CLEARFAULT"); r.stopped(); assert(!r.drive.faulted());
  r.send("FWD 20"); assert(r.drive.leftPercent() == 20);
}

void velocity_lease_starts_even_with_zero_initial_pwm() {
  Rover r;
  r.send("rpm 20 -10");
  assert(r.drive.motionRequested() && r.drive.leftPercent() == 0);
  assert(Serial.output.find("active=yes mode=velocity") != std::string::npos);
  r.sample(200); assert(r.drive.leftPercent() == 20 && r.drive.rightPercent() == -10);
  r.send("TELEMETRY 50"); r.send("STATUS");
  for (uint32_t now = 300; now <= 800; now += 100) r.sample(now);
  testMillis = 849; r.commands.poll(); assert(r.drive.motionRequested());
  testMillis = 850; r.commands.poll(); r.stopped();
  assert(Serial.output.find("WATCHDOG stop") != std::string::npos);
  r.sample(900); r.stopped(); // A later sample must never revive an expired PID.
}

void explicit_stop_and_raw_mode_cancel_pid() {
  Rover r;
  r.send("RPM 20 20"); r.sample(200);
  r.send("STOP"); r.stopped(); r.sample(300); r.stopped();
  r.send("RPM 20 20"); r.sample(400);
  r.send("MOTOR -15 10");
  assert(std::string(r.drive.modeName()) == "open_loop");
  assert(r.drive.leftTargetRpm() == 0 && r.drive.rightTargetRpm() == 0);
  r.sample(500); assert(r.drive.leftPercent() == -15 && r.drive.rightPercent() == 10);
  r.send("CLEARFAULT"); r.stopped(); // Refuse acknowledgement while moving.
  assert(Serial.output.find("requires stopped") != std::string::npos);
}

void invalid_commands_stop_velocity_without_refresh() {
  for (const auto& command : std::vector<std::string>{
      "RPM", "RPM 20", "RPM 20 20 extra", "RPM nan 20", "RPM 20 inf", "RPM 1e999 0",
      "RPM 20_0 0", "FWD garbage", "MOTOR 1 2 3", "DRIVE nan 0", "CLEARFAULT extra",
      std::string(96, 'x') + "RPM 100 100"}) {
    Rover r;
    r.send("RPM 20 20"); r.sample(200);
    r.send(command); r.stopped();
    assert(Serial.output.find("ERR ") != std::string::npos);
    r.sample(300); r.stopped();
  }
}

void feedback_fault_stops_both_wheels_and_blocks_repeated_commands() {
  Rover r;
  r.send("RPM 20 30"); r.sample(200);
  ++r.feedback.leftInvalid;
  r.send("RPM 20 30"); r.stopped();
  assert(r.drive.faulted());
  assert(Serial.output.find("fault=encoder_transition") != std::string::npos);
  r.send("FWD 100"); r.stopped();
  r.send("CLEARFAULT"); r.stopped();
  r.send("RPM 10 10"); assert(r.drive.leftPercent() == 0);
  r.sample(300); assert(r.drive.leftPercent() == 10);
}

void missing_or_stale_feedback_cannot_start_velocity() {
  for (bool missing : {false, true}) {
    Rover r;
    if (missing) r.feedback.sampleMs = 0;
    else testMillis = 400;
    r.send("RPM 20 20"); r.stopped(); assert(r.drive.faulted());
  }
}

void lease_and_feedback_survive_clock_wrap() {
  const uint32_t initial = UINT32_MAX - 150U;
  Rover r(true, initial);
  r.send("RPM 10 10");
  for (uint32_t offset = 100; offset <= 700; offset += 100) r.sample(initial + offset);
  testMillis = initial + 750; r.commands.poll(); r.stopped();
  assert(Serial.output.find("WATCHDOG stop") != std::string::npos);
}

void zero_target_and_fault_ack_never_start_motion() {
  Rover r;
  r.send("RPM 20 20"); r.sample(200);
  r.send("RPM 0 0"); r.stopped();
  r.send("CLEARFAULT"); r.stopped();
  r.send("RPM 101 0"); r.stopped(); assert(r.drive.faulted());
  r.send("CLEARFAULT"); r.stopped();
}

void m0_motion_and_parser_regressions() {
  Rover r;
  r.send("F"); assert(r.drive.leftPercent() == 45 && r.drive.rightPercent() == 45);
  r.send("B"); assert(r.drive.leftPercent() == -45);
  r.send("L"); assert(r.drive.leftPercent() == -45 && r.drive.rightPercent() == 45);
  r.send("R"); assert(r.drive.leftPercent() == 45 && r.drive.rightPercent() == -45);
  r.send("DRIVE 1 1"); assert(r.drive.leftPercent() == 0 && r.drive.rightPercent() == 100);
  r.send("DRIVE 1 -1"); assert(r.drive.leftPercent() == 100 && r.drive.rightPercent() == 0);
  r.send("MOTOR 4294967296 10"); r.stopped();
  r.send("FWD 10"); r.send(std::string("FWD 20\0FWD 100", 14)); r.stopped();
  r.send("FWD 20"); testMillis += 750; r.commands.poll(); r.stopped();
}

void emit_frames() {
  Rover r;
  auto print = [&] { Serial.output.clear(); r.commands.printTelemetry(); std::cout << Serial.output; };
  print();
  r.send("RPM 20 -15"); r.sample(200); print();
  ++r.feedback.rightInvalid; r.commands.poll(); print();
  r.send("CLEARFAULT"); print();
}

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--emit-telemetry") { emit_frames(); return 0; }
  default_disabled_and_explicit_recovery();
  velocity_lease_starts_even_with_zero_initial_pwm();
  explicit_stop_and_raw_mode_cancel_pid();
  invalid_commands_stop_velocity_without_refresh();
  feedback_fault_stops_both_wheels_and_blocks_repeated_commands();
  missing_or_stale_feedback_cannot_start_velocity();
  lease_and_feedback_survive_clock_wrap();
  zero_target_and_fault_ack_never_start_motion();
  m0_motion_and_parser_regressions();
  std::cout << "velocity/serial integration passed (9 groups)\n";
}
