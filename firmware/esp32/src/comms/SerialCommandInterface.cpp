#include "comms/SerialCommandInterface.h"

#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace roboty {

SerialCommandInterface::SerialCommandInterface(
    DifferentialDrive& drive, uint32_t watchdogMs, int defaultDrivePercent,
    const WheelTelemetry& telemetry)
    : drive_(drive), telemetry_(telemetry), watchdogMs_(watchdogMs),
      defaultDrivePercent_(defaultDrivePercent), lastMotionCommandMs_(0),
      motionActive_(false), buffer_{0}, bufferLength_(0), discardLine_(false),
      telemetryIntervalMs_(0), lastTelemetryMs_(0) {}

void SerialCommandInterface::begin() {
  drive_.stop();
  bufferLength_ = 0;
  discardLine_ = false;
  motionActive_ = false;
  telemetryIntervalMs_ = 0;
  lastMotionCommandMs_ = millis();
  lastTelemetryMs_ = millis();
}

void SerialCommandInterface::poll() {
  // Expire the motion lease before applying another PID output.
  enforceWatchdog();
  const bool alreadyFaulted = drive_.faulted();
  drive_.updateVelocity(millis());
  if (!alreadyFaulted && drive_.faulted()) {
    motionActive_ = false;
    Serial.printf("ERR velocity fault=%s; motors stopped; use CLEARFAULT after inspection\n", drive_.faultName());
  }
  size_t consumed = 0;
  while (Serial.available() > 0 && consumed < MAX_BYTES_PER_POLL) {
    consumeChar(static_cast<char>(Serial.read()));
    ++consumed;
    enforceWatchdog();
  }
  const uint32_t now = millis();
  if (telemetryIntervalMs_ > 0 &&
      static_cast<uint32_t>(now - lastTelemetryMs_) >= telemetryIntervalMs_) {
    lastTelemetryMs_ = now;
    printTelemetry();
  }
  enforceWatchdog();
}

void SerialCommandInterface::printHelp() const {
  Serial.println();
  Serial.println(F("Roboty M2 commands:"));
  Serial.println(F("  FWD [0..100]       - drive forward (alias F)"));
  Serial.println(F("  BACK [0..100]      - drive backward (alias B)"));
  Serial.println(F("  LEFT [0..100]      - rotate left (alias L)"));
  Serial.println(F("  RIGHT [0..100]     - rotate right (alias R)"));
  Serial.println(F("  DRIVE L A          - normalized linear/angular [-1..1]"));
  Serial.println(F("  MOTOR L R          - raw motor percentages [-100..100]"));
  Serial.println(F("  RPM L R            - wheel targets; requires configured closed-loop control"));
  Serial.println(F("  STOP               - stop and reset controllers (alias S)"));
  Serial.println(F("  CLEARFAULT         - acknowledge latched fault while stopped; never starts motion"));
  Serial.println(F("  STATUS             - motor state, mode, targets and fault"));
  Serial.println(F("  TELEMETRY          - one JSON encoder/speed/control snapshot"));
  Serial.println(F("  TELEMETRY 50..5000 - stream every N milliseconds"));
  Serial.println(F("  TELEMETRY 0        - disable streaming"));
  Serial.println(F("  HELP               - show this help (alias ?)"));
  Serial.println(F("Only valid motion commands refresh the motion watchdog."));
}

void SerialCommandInterface::printStatus() const {
  Serial.printf(
      "left=%d%% right=%d%% watchdog=%lums active=%s mode=%s fault=%s left_target_rpm=%.6g right_target_rpm=%.6g\n",
      drive_.leftPercent(), drive_.rightPercent(), static_cast<unsigned long>(watchdogMs_),
      motionActive_ ? "yes" : "no", drive_.modeName(), drive_.faultName(),
      drive_.leftTargetRpm(), drive_.rightTargetRpm());
}

void SerialCommandInterface::printTelemetry() const {
  char leftRpm[40] = "null";
  char rightRpm[40] = "null";
  if (telemetry_.enabled && telemetry_.calibrated && telemetry_.sampleMs > 0) {
    if (std::isfinite(telemetry_.leftRpm)) snprintf(leftRpm, sizeof(leftRpm), "%.6g", telemetry_.leftRpm);
    if (std::isfinite(telemetry_.rightRpm)) snprintf(rightRpm, sizeof(rightRpm), "%.6g", telemetry_.rightRpm);
  }
  Serial.printf(
      "{\"type\":\"telemetry\",\"ms\":%lu,\"encoders\":%s,\"calibrated\":%s,"
      "\"left_ticks\":%lld,\"right_ticks\":%lld,\"left_delta\":%lld,\"right_delta\":%lld,"
      "\"left_rpm\":%s,\"right_rpm\":%s,\"left_invalid\":%lu,\"right_invalid\":%lu,"
      "\"sample_ms\":%lu,\"left_pwm\":%d,\"right_pwm\":%d,\"motion_active\":%s,"
      "\"schema_version\":2,\"sample_seq\":%lu,\"sample_at_ms\":%lu,"
      "\"control_mode\":\"%s\",\"control_fault\":\"%s\","
      "\"left_target_rpm\":%.6g,\"right_target_rpm\":%.6g}\n",
      static_cast<unsigned long>(millis()), telemetry_.enabled ? "true" : "false",
      telemetry_.enabled && telemetry_.calibrated ? "true" : "false",
      static_cast<long long>(telemetry_.leftTicks), static_cast<long long>(telemetry_.rightTicks),
      static_cast<long long>(telemetry_.leftDelta), static_cast<long long>(telemetry_.rightDelta),
      leftRpm, rightRpm, static_cast<unsigned long>(telemetry_.leftInvalid),
      static_cast<unsigned long>(telemetry_.rightInvalid), static_cast<unsigned long>(telemetry_.sampleMs),
      drive_.leftPercent(), drive_.rightPercent(), motionActive_ ? "true" : "false",
      static_cast<unsigned long>(telemetry_.sampleSequence), static_cast<unsigned long>(telemetry_.sampleAtMs),
      drive_.modeName(), drive_.faultName(), drive_.leftTargetRpm(), drive_.rightTargetRpm());
}

void SerialCommandInterface::consumeChar(char c) {
  if (c == '\n' || c == '\r') {
    if (!discardLine_ && bufferLength_ > 0) {
      buffer_[bufferLength_] = '\0';
      executeLine(buffer_);
    }
    bufferLength_ = 0;
    discardLine_ = false;
    return;
  }
  if (discardLine_) return;
  if ((c < ' ' && c != '\t') || c > '~') {
    discardLine_ = true;
    bufferLength_ = 0;
    stopWithError("invalid character; line discarded; motors stopped");
    return;
  }
  if (bufferLength_ >= BUFFER_SIZE - 1) {
    discardLine_ = true;
    bufferLength_ = 0;
    stopWithError("command too long; line discarded; motors stopped");
    return;
  }
  buffer_[bufferLength_++] = c;
}

void SerialCommandInterface::executeLine(char* line) {
  char* savePtr = nullptr;
  char* command = strtok_r(line, " \t", &savePtr);
  if (command == nullptr) return;
  char* first = strtok_r(nullptr, " \t", &savePtr);
  char* second = strtok_r(nullptr, " \t", &savePtr);
  char* extra = strtok_r(nullptr, " \t", &savePtr);
  uppercase(command);
  const bool forward = strcmp(command, "FWD") == 0 || strcmp(command, "F") == 0;
  const bool backward = strcmp(command, "BACK") == 0 || strcmp(command, "B") == 0;
  const bool left = strcmp(command, "LEFT") == 0 || strcmp(command, "L") == 0;
  const bool right = strcmp(command, "RIGHT") == 0 || strcmp(command, "R") == 0;
  const bool rpm = strcmp(command, "RPM") == 0;
  const bool normalized = strcmp(command, "DRIVE") == 0;
  const bool raw = strcmp(command, "MOTOR") == 0;
  if ((forward || backward || left || right || rpm || normalized || raw) && drive_.faulted()) {
    stopWithError("latched control fault; inspect STATUS and use CLEARFAULT while stopped");
    return;
  }
  if (forward || backward || left || right) {
    int percent = 0;
    if (second != nullptr || !parsePercentOrDefault(first, percent)) {
      stopWithError("usage: FWD|BACK|LEFT|RIGHT [0..100]; motors stopped");
      return;
    }
    if (forward) drive_.forward(percent);
    if (backward) drive_.backward(percent);
    if (left) drive_.turnLeft(percent);
    if (right) drive_.turnRight(percent);
    registerMotion(); printStatus(); return;
  }
  if (normalized) {
    float linear = 0.0F, angular = 0.0F;
    if (extra != nullptr || !parseFloat(first, linear) || !parseFloat(second, angular)) {
      stopWithError("usage: DRIVE <linear -1..1> <angular -1..1>; motors stopped"); return;
    }
    drive_.setNormalized(linear, angular);
    registerMotion(); printStatus(); return;
  }
  if (raw) {
    int leftPercent = 0, rightPercent = 0;
    if (extra != nullptr || !parseInt(first, leftPercent) || !parseInt(second, rightPercent) ||
        leftPercent < -100 || leftPercent > 100 || rightPercent < -100 || rightPercent > 100) {
      stopWithError("usage: MOTOR <left -100..100> <right -100..100>; motors stopped"); return;
    }
    drive_.setMotorPercents(leftPercent, rightPercent);
    registerMotion(); printStatus(); return;
  }
  if (rpm) {
    double leftRpm = 0, rightRpm = 0;
    if (extra != nullptr || !parseNumber(first, leftRpm) || !parseNumber(second, rightRpm)) {
      stopWithError("usage: RPM <finite left> <finite right>; motors stopped"); return;
    }
    if (!drive_.setWheelRpms(leftRpm, rightRpm, millis())) {
      motionActive_ = false;
      Serial.printf("ERR velocity fault=%s; motors stopped\n", drive_.faultName()); return;
    }
    registerMotion(); printStatus(); return;
  }
  if (strcmp(command, "TELEMETRY") == 0) {
    int interval = 0;
    if (second != nullptr || (first != nullptr &&
        (!parseInt(first, interval) || (interval != 0 && (interval < 50 || interval > 5000))))) {
      stopWithError("usage: TELEMETRY [0|50..5000 ms]; motors stopped"); return;
    }
    if (first != nullptr) {
      telemetryIntervalMs_ = static_cast<uint32_t>(interval);
      lastTelemetryMs_ = millis();
      Serial.printf("OK telemetry_interval_ms=%d\n", interval);
    } else printTelemetry();
    return;
  }
  if (first != nullptr) { stopWithError("unexpected arguments; motors stopped"); return; }
  if (strcmp(command, "STOP") == 0 || strcmp(command, "S") == 0) {
    drive_.stop(); motionActive_ = false; printStatus();
  } else if (strcmp(command, "CLEARFAULT") == 0) {
    if (!drive_.clearFault()) {
      stopWithError("CLEARFAULT requires stopped motors; STOP first and retry");
    } else Serial.println(F("OK control_fault=none; motion remains stopped"));
  } else if (strcmp(command, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(command, "HELP") == 0 || strcmp(command, "?") == 0) {
    printHelp();
  } else stopWithError("unknown command; motors stopped; type HELP");
}

void SerialCommandInterface::registerMotion() {
  lastMotionCommandMs_ = millis();
  motionActive_ = drive_.motionRequested();
}

void SerialCommandInterface::enforceWatchdog() {
  if (!motionActive_ || static_cast<uint32_t>(millis() - lastMotionCommandMs_) < watchdogMs_) return;
  drive_.stop();
  motionActive_ = false;
  Serial.println(F("WATCHDOG stop: motion command expired"));
}

void SerialCommandInterface::stopWithError(const char* message) {
  drive_.stop(); motionActive_ = false; Serial.printf("ERR %s\n", message);
}

bool SerialCommandInterface::parsePercentOrDefault(const char* token, int& value) const {
  if (token == nullptr) { value = defaultDrivePercent_; return value >= 0 && value <= 100; }
  return parseInt(token, value) && value >= 0 && value <= 100;
}

void SerialCommandInterface::uppercase(char* text) {
  while (*text != '\0') {
    if (*text >= 'a' && *text <= 'z') *text = static_cast<char>(*text - ('a' - 'A'));
    ++text;
  }
}

bool SerialCommandInterface::parseFloat(const char* token, float& value) {
  if (token == nullptr) return false;
  errno = 0;
  char* end = nullptr;
  const float parsed = strtof(token, &end);
  if (errno != 0 || end == token || *end != '\0' || !std::isfinite(parsed) ||
      parsed < -1.0F || parsed > 1.0F) return false;
  value = parsed; return true;
}

bool SerialCommandInterface::parseNumber(const char* token, double& value) {
  if (token == nullptr) return false;
  errno = 0;
  char* end = nullptr;
  const double parsed = strtod(token, &end);
  if (errno != 0 || end == token || *end != '\0' || !std::isfinite(parsed)) return false;
  value = parsed; return true;
}

bool SerialCommandInterface::parseInt(const char* token, int& value) {
  if (token == nullptr) return false;
  errno = 0;
  char* end = nullptr;
  const long parsed = strtol(token, &end, 10);
  if (errno != 0 || end == token || *end != '\0' || parsed < INT_MIN || parsed > INT_MAX) return false;
  value = static_cast<int>(parsed); return true;
}

}  // namespace roboty
