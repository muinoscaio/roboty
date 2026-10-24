#pragma once

#include <Arduino.h>

#include "comms/WheelTelemetry.h"
#include "control/DifferentialDrive.h"

namespace roboty {

class SerialCommandInterface {
 public:
  SerialCommandInterface(
      DifferentialDrive& drive,
      uint32_t watchdogMs,
      int defaultDrivePercent,
      const WheelTelemetry& telemetry);

  void begin();
  void poll();
  void printHelp() const;
  void printStatus() const;
  void printTelemetry() const;

 private:
  static constexpr size_t BUFFER_SIZE = 96;
  static constexpr size_t MAX_BYTES_PER_POLL = 64;

  DifferentialDrive& drive_;
  const WheelTelemetry& telemetry_;
  uint32_t watchdogMs_;
  int defaultDrivePercent_;
  uint32_t lastMotionCommandMs_;
  bool motionActive_;
  char buffer_[BUFFER_SIZE];
  size_t bufferLength_;
  bool discardLine_;
  uint32_t telemetryIntervalMs_;
  uint32_t lastTelemetryMs_;

  void consumeChar(char c);
  void executeLine(char* line);
  void registerMotion();
  void enforceWatchdog();
  void stopWithError(const char* message);
  bool parsePercentOrDefault(const char* token, int& value) const;

  static void uppercase(char* text);
  static bool parseFloat(const char* token, float& value);
  static bool parseNumber(const char* token, double& value);
  static bool parseInt(const char* token, int& value);
};

}  // namespace roboty
