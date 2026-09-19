#include "comms/SerialCommandInterface.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace roboty {

SerialCommandInterface::SerialCommandInterface(
    DifferentialDrive& drive,
    uint32_t watchdogMs,
    int defaultDrivePercent)
    : drive_(drive),
      watchdogMs_(watchdogMs),
      defaultDrivePercent_(defaultDrivePercent),
      lastMotionCommandMs_(0),
      motionActive_(false),
      buffer_{0},
      bufferLength_(0) {}

void SerialCommandInterface::begin() {
  bufferLength_ = 0;
  motionActive_ = false;
  lastMotionCommandMs_ = millis();
}

void SerialCommandInterface::poll() {
  while (Serial.available() > 0) {
    consumeChar(static_cast<char>(Serial.read()));
  }

  enforceWatchdog();
}

void SerialCommandInterface::printHelp() const {
  Serial.println();
  Serial.println(F("Roboty M0 commands:"));
  Serial.println(F("  FWD [0..100]       - drive forward"));
  Serial.println(F("  BACK [0..100]      - drive backward"));
  Serial.println(F("  LEFT [0..100]      - rotate left"));
  Serial.println(F("  RIGHT [0..100]     - rotate right"));
  Serial.println(F("  DRIVE L A          - normalized linear/angular [-1..1]"));
  Serial.println(F("  MOTOR L R          - raw motor percentages [-100..100]"));
  Serial.println(F("  STOP               - stop immediately"));
  Serial.println(F("  STATUS             - print commanded motor state"));
  Serial.println(F("  HELP               - show this help"));
  Serial.println();
  Serial.println(F("Motion commands must be refreshed before the watchdog expires."));
}

void SerialCommandInterface::printStatus() const {
  Serial.printf(
      "left=%d%% right=%d%% watchdog=%lums active=%s\n",
      drive_.leftPercent(),
      drive_.rightPercent(),
      static_cast<unsigned long>(watchdogMs_),
      motionActive_ ? "yes" : "no");
}

void SerialCommandInterface::consumeChar(char c) {
  if (c == '\r') {
    return;
  }

  if (c == '\n') {
    buffer_[bufferLength_] = '\0';

    if (bufferLength_ > 0) {
      executeLine(buffer_);
    }

    bufferLength_ = 0;
    buffer_[0] = '\0';
    return;
  }

  if (bufferLength_ >= BUFFER_SIZE - 1) {
    bufferLength_ = 0;
    buffer_[0] = '\0';
    drive_.stop();
    motionActive_ = false;
    Serial.println(F("ERR command too long; motors stopped"));
    return;
  }

  buffer_[bufferLength_++] = c;
}

void SerialCommandInterface::executeLine(char* line) {
  char* savePtr = nullptr;
  char* command = strtok_r(line, " \t", &savePtr);

  if (command == nullptr) {
    return;
  }

  uppercase(command);

  if (strcmp(command, "FWD") == 0 || strcmp(command, "F") == 0) {
    drive_.forward(parsePercentOrDefault(strtok_r(nullptr, " \t", &savePtr)));
    registerMotion();
    printStatus();
    return;
  }

  if (strcmp(command, "BACK") == 0 || strcmp(command, "B") == 0) {
    drive_.backward(parsePercentOrDefault(strtok_r(nullptr, " \t", &savePtr)));
    registerMotion();
    printStatus();
    return;
  }

  if (strcmp(command, "LEFT") == 0 || strcmp(command, "L") == 0) {
    drive_.turnLeft(parsePercentOrDefault(strtok_r(nullptr, " \t", &savePtr)));
    registerMotion();
    printStatus();
    return;
  }

  if (strcmp(command, "RIGHT") == 0 || strcmp(command, "R") == 0) {
    drive_.turnRight(parsePercentOrDefault(strtok_r(nullptr, " \t", &savePtr)));
    registerMotion();
    printStatus();
    return;
  }

  if (strcmp(command, "DRIVE") == 0) {
    float linear = 0.0F;
    float angular = 0.0F;

    if (!parseFloat(strtok_r(nullptr, " \t", &savePtr), linear) ||
        !parseFloat(strtok_r(nullptr, " \t", &savePtr), angular)) {
      Serial.println(F("ERR usage: DRIVE <linear -1..1> <angular -1..1>"));
      return;
    }

    drive_.setNormalized(linear, angular);
    registerMotion();
    printStatus();
    return;
  }

  if (strcmp(command, "MOTOR") == 0) {
    int left = 0;
    int right = 0;

    if (!parseInt(strtok_r(nullptr, " \t", &savePtr), left) ||
        !parseInt(strtok_r(nullptr, " \t", &savePtr), right) ||
        left < -100 || left > 100 || right < -100 || right > 100) {
      Serial.println(F("ERR usage: MOTOR <left -100..100> <right -100..100>"));
      return;
    }

    drive_.setMotorPercents(left, right);
    registerMotion();
    printStatus();
    return;
  }

  if (strcmp(command, "STOP") == 0 || strcmp(command, "S") == 0) {
    drive_.stop();
    motionActive_ = false;
    printStatus();
    return;
  }

  if (strcmp(command, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcmp(command, "HELP") == 0 || strcmp(command, "?") == 0) {
    printHelp();
    return;
  }

  Serial.println(F("ERR unknown command; type HELP"));
}

void SerialCommandInterface::registerMotion() {
  lastMotionCommandMs_ = millis();
  motionActive_ =
      drive_.leftPercent() != 0 || drive_.rightPercent() != 0;
}

void SerialCommandInterface::enforceWatchdog() {
  if (!motionActive_) {
    return;
  }

  if (millis() - lastMotionCommandMs_ <= watchdogMs_) {
    return;
  }

  drive_.stop();
  motionActive_ = false;
  Serial.println(F("WATCHDOG stop: motion command expired"));
}

int SerialCommandInterface::parsePercentOrDefault(const char* token) const {
  if (token == nullptr) {
    return defaultDrivePercent_;
  }

  int value = 0;
  if (!parseInt(token, value) || value < 0 || value > 100) {
    Serial.printf(
        "WARN invalid speed '%s'; using %d%%\n",
        token,
        defaultDrivePercent_);
    return defaultDrivePercent_;
  }

  return value;
}

void SerialCommandInterface::uppercase(char* text) {
  while (*text != '\0') {
    if (*text >= 'a' && *text <= 'z') {
      *text = static_cast<char>(*text - ('a' - 'A'));
    }
    ++text;
  }
}

bool SerialCommandInterface::parseFloat(const char* token, float& value) {
  if (token == nullptr) {
    return false;
  }

  errno = 0;
  char* end = nullptr;
  const float parsed = strtof(token, &end);

  if (errno != 0 || end == token || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }

  if (parsed < -1.0F || parsed > 1.0F) {
    return false;
  }

  value = parsed;
  return true;
}

bool SerialCommandInterface::parseInt(const char* token, int& value) {
  if (token == nullptr) {
    return false;
  }

  errno = 0;
  char* end = nullptr;
  const long parsed = strtol(token, &end, 10);

  if (errno != 0 || end == token || *end != '\0') {
    return false;
  }

  value = static_cast<int>(parsed);
  return true;
}

}  // namespace roboty
