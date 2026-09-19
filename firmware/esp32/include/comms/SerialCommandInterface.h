#pragma once

#include <Arduino.h>

#include "control/DifferentialDrive.h"

namespace roboty {

class SerialCommandInterface {
 public:
  SerialCommandInterface(
      DifferentialDrive& drive,
      uint32_t watchdogMs,
      int defaultDrivePercent);

  void begin();
  void poll();
  void printHelp() const;
  void printStatus() const;

 private:
  static constexpr size_t BUFFER_SIZE = 96;

  DifferentialDrive& drive_;
  uint32_t watchdogMs_;
  int defaultDrivePercent_;
  uint32_t lastMotionCommandMs_;
  bool motionActive_;
  char buffer_[BUFFER_SIZE];
  size_t bufferLength_;

  void consumeChar(char c);
  void executeLine(char* line);
  void registerMotion();
  void enforceWatchdog();
  int parsePercentOrDefault(const char* token) const;

  static void uppercase(char* text);
  static bool parseFloat(const char* token, float& value);
  static bool parseInt(const char* token, int& value);
};

}  // namespace roboty
