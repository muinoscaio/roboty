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

Start with [encoder calibration](hardware/encoder-calibration.md). The [M1 checkpoint](checkpoints/m1-encoder-telemetry.md) records the earlier implementation; PR #2 was integrated before M2.

## M2 — Closed-loop wheel control

Status: software implemented and CI-validated; opt-in remains disabled until real configuration. Hardware tuning and acceptance are not complete.

- [x] Independent wheel PID with output caps, conditional-integration anti-windup, filtered derivative and clean reset.
- [x] Signed wheel RPM targets and explicit stopped/open-loop/velocity modes.
- [x] Configurable finite target/feedback RPM and PWM limits; unknown defaults fail closed.
- [x] STOP/watchdog clear controller state, including while waiting for the first output.
- [x] Latched faults for unavailable/stale/invalid feedback, sample timing and invalid encoder transitions.
- [x] No-progress timeout for either targeted wheel; wrong-direction counts and repeating commands do not keep it alive.
- [x] Explicit stopped-only fault acknowledgement; no automatic restart on feedback recovery.
- [x] Preserve raw PWM diagnostics, blocked by a latched fault until acknowledged.
- [x] Schema 2 telemetry: desired/measured RPM, commanded PWM, mode, fault and completed sample identity.
- [x] Same-port interactive control and CSV logging; stop refresh on logging/I/O errors or observed restart.
- [x] Native PID/synthetic-plant/protocol tests and host regression tests.
- [x] CI compiles both default and enabled synthetic firmware without changing real defaults.
- [x] Reproducible tuning guide and checkpoint with recovery references.
- [ ] Select and record real gains, speed/PWM limits and timeouts for the purchased hardware.
- [ ] Validate tracking, low-speed behavior, saturation recovery, stopping, reversals and feedback faults on a controlled bench.
- [ ] Save measured configuration and captures with acceptance results.

See [velocity tuning](hardware/velocity-tuning.md), [protocol](serial-protocol.md) and [M2 checkpoint](checkpoints/m2-wheel-velocity.md). Compiling or passing simulation tests does not close physical acceptance items.

## M3 — Odometry and IMU

Next implementation milestone. Mathematical tests can proceed independently of hardware; real dimensions and an actual IMU must not be invented.

- [ ] Explicit per-wheel radius, wheel separation and calibrated CPR; disabled/unavailable odometry until configured.
- [ ] Differential-drive pose/twist from cumulative signed count differences, with straight/arc/in-place rotation tests.
- [ ] Timestamped odometry and sample identity without summing repeated streamed delta rows.
- [ ] Reset/session semantics, duplicate/skipped sample handling, wrap and invalid-data tests.
- [ ] Measured geometry and physical distance/rotation acceptance runs.
- [ ] Identify the actual IMU, voltage/interface, mounting and driver requirements.
- [ ] IMU integration, stationary bias calibration and documented fusion assumptions.
- [ ] Preserve M0/M1/M2 stop/fault behavior and publish a continuation checkpoint.

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
