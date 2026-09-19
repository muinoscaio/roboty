# Roboty

Roboty is a modular autonomous mobile robot project that starts as a simple ESP32-based differential-drive rover and evolves toward closed-loop motor control, odometry, ROS 2, SLAM, navigation and computer vision.

## Current milestone: M0 — Basic Rover

The first milestone targets a physical 2WD robot with:

- ESP32 DevKit (ESP32-WROOM-32)
- TB6612FNG dual H-bridge
- 2 DC geared motors
- USB serial command interface
- independent left/right motor control
- differential-drive commands (linear + angular)
- command watchdog / fail-safe stop
- architecture prepared for encoders, PID and ROS 2

## Planned evolution

```text
M0  Basic rover / open-loop control
 ↓
M1  Encoders + wheel speed measurement
 ↓
M2  Closed-loop wheel PID
 ↓
M3  Odometry + IMU
 ↓
M4  ROS 2 bridge + URDF/TF
 ↓
M5  Simulation + Gazebo
 ↓
M6  LiDAR + SLAM
 ↓
M7  Nav2 autonomous navigation
 ↓
M8  Camera + computer vision
```

The hardware/firmware boundary is intentionally kept small so the higher-level autonomy stack can grow without coupling ROS or perception code to the motor driver.

## Repository layout

```text
roboty/
├── firmware/esp32/       # Real-time rover firmware
├── docs/                 # Architecture, hardware and roadmap
├── tools/                # Host-side utilities
└── .github/workflows/    # CI
```

## License

MIT.
