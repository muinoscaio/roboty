# Roadmap

## M0 — Basic rover

Status: software implemented and regression-tested; physical acceptance pending.

- [x] PlatformIO ESP32 project and TB6612FNG driver.
- [x] Differential-drive mixing and USB serial protocol.
- [x] Motion watchdog / fail-safe stop.
- [x] Host serial helper and CI firmware build.
- [x] Reject invalid speeds, excess arguments and oversized-line suffixes.
- [x] Bound input processing and preserve the motion watchdog during telemetry.
- [x] Order host motion writes and STOP; handle EOF and connection failures.
- [ ] Validate motor polarity, STOP and watchdog on physical hardware.
- [ ] Record first-drive video.

## M1 — Encoder acquisition

Status: software implemented; configuration and measurement must be verified on the purchased encoders.

- [x] Quadrature x4 driver with independent per-wheel inversion.
- [x] Atomic signed counts and invalid-transition diagnostics.
- [x] Actual-interval wheel RPM calculation, including clock wrap.
- [x] Explicit unavailable RPM while CPR is unknown.
- [x] GPIO validation, opt-in initialization and rollback on failure.
- [x] On-demand and periodic JSON telemetry.
- [x] Passive CSV capture tool.
- [x] Hardware calibration document and protocol reference.
- [x] Native decoder/RPM/parser/watchdog tests and Python host tests in CI.
- [ ] Identify encoder output type/voltage and approve the final GPIO wiring.
- [ ] Measure each wheel's x4 counts per revolution and verify forward polarity.
- [ ] Verify RPM against a physical reference and determine usable pulse throughput/noise limits.
- [ ] Record a bench capture and results in the calibration checklist.

Start with [encoder calibration](hardware/encoder-calibration.md). Current progress, verification commands and handoff notes are in the [M1 checkpoint](checkpoints/m1-encoder-telemetry.md).

## M2 — Closed-loop wheel control

Next software milestone. Physical tuning depends on M1 bench evidence.

- [ ] PID per wheel with output limits, anti-windup and clean disable/reset.
- [ ] Target wheel RPM API and explicit control-mode state.
- [ ] Bound speed targets using the actual motors and power supply.
- [ ] Stop and clear controller state on watchdog expiry or unusable feedback.
- [ ] Step-response telemetry with target/measured RPM and applied PWM.
- [ ] Reproducible tuning procedure and saved calibration values.
- [ ] Preserve raw PWM commands for diagnosis and recovery.
- [ ] Validate tracking, reversals, stopping and stalled/absent feedback on a supported bench.

## M3 — Odometry and IMU

- [ ] Wheel odometry and robot physical-parameter configuration.
- [ ] Timestamped pose/twist telemetry.
- [ ] IMU integration and sensor calibration.

## M4 — ROS 2 integration

- [ ] Companion-computer workspace.
- [ ] URDF and TF tree.
- [ ] `cmd_vel` bridge and odometry publication.
- [ ] RViz bringup.

## M5 — Simulation

- [ ] Gazebo model, differential drive and simulated sensors.
- [ ] Same high-level command contract as the physical rover.

## M6 — Mapping

- [ ] 2D LiDAR and SLAM Toolbox.
- [ ] Save/load occupancy maps.

## M7 — Autonomous navigation

- [ ] Nav2 with global/local costmaps.
- [ ] Goal navigation, obstacle avoidance and recovery.

## M8 — Vision

- [ ] Camera and ROS image pipeline.
- [ ] OpenCV and object detection/tracking.
- [ ] Perception-driven mission.
