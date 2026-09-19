# Roadmap

## M0 — Basic Rover

Status: implementation started.

Acceptance criteria:

- [x] PlatformIO ESP32 project
- [x] TB6612FNG abstraction
- [x] Differential-drive mixing
- [x] USB serial command protocol
- [x] Motion watchdog / fail-safe stop
- [x] Host serial helper
- [x] CI firmware build
- [ ] Validate on physical hardware
- [ ] Record first-drive video

## M1 — Encoder acquisition

- quadrature encoder driver
- deterministic wheel tick counters
- wheel RPM calculation
- serial telemetry
- hardware calibration document

## M2 — Closed-loop wheel control

- PID per wheel
- target wheel velocity API
- step-response telemetry
- tuning procedure
- raw PWM diagnostic mode retained

## M3 — Odometry and IMU

- wheel odometry
- robot physical-parameter configuration
- IMU integration
- timestamped telemetry

## M4 — ROS 2 integration

- companion-computer workspace
- URDF
- TF tree
- cmd_vel bridge
- odom publication
- RViz bringup

## M5 — Simulation

- Gazebo model
- simulated differential drive
- simulated sensors
- same high-level command contract as real rover

## M6 — Mapping

- 2D LiDAR
- SLAM Toolbox
- saved/loaded occupancy maps

## M7 — Autonomous navigation

- Nav2
- global/local costmaps
- autonomous goal navigation
- obstacle avoidance and recovery

## M8 — Vision

- camera
- ROS image pipeline
- OpenCV
- object detection/tracking
- perception-driven mission
