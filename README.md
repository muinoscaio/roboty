# Roboty

Roboty is a modular autonomous mobile robot project that starts as a simple ESP32-based differential-drive rover and evolves toward closed-loop motor control, odometry, ROS 2, SLAM, navigation and computer vision.

## Current milestone: M0 — Basic Rover

The first milestone targets a physical 2WD robot with:

- ESP32 DevKit (ESP32-WROOM-32)
- TB6612FNG dual H-bridge
- 2 matched 6V geared motors with quadrature encoders
- USB serial command interface
- independent left/right motor control
- differential-drive commands (linear + angular)
- command watchdog / fail-safe stop
- architecture prepared for encoder feedback, PID and ROS 2

## Quick start

### Build

~~~bash
git clone https://github.com/muinoscaio/roboty.git
cd roboty/firmware/esp32
pio run
~~~

### Upload

~~~bash
pio run --target upload
~~~

If the board port is not detected automatically:

~~~bash
pio device list
pio run --target upload --upload-port COM5
~~~

### Manual serial test

~~~bash
pio device monitor --baud 115200
~~~

Available commands:

~~~text
FWD 40
BACK 40
LEFT 35
RIGHT 35
DRIVE 0.5 0.0
DRIVE 0.4 -0.3
MOTOR 40 40
STOP
STATUS
HELP
~~~

A motion command expires after 750 ms unless refreshed. This is an intentional fail-safe.

### Continuous host control

~~~bash
cd tools
python -m venv .venv
# Windows:
.venv\Scripts\activate
pip install -r requirements.txt
python serial_control.py COM5
~~~

The Python controller refreshes active motion commands before the watchdog expires.

## Hardware

- [M0 wiring](docs/hardware/wiring.md)
- [Bill of materials](docs/hardware/bom.md)
- [Current shopping list and purchase order](docs/hardware/shopping-list.md)
- [Architecture](docs/architecture.md)
- [Roadmap](docs/roadmap.md)

For the first powered motor test, keep the wheels off the ground. Do not power the motors from the ESP32 3V3 pin.

## Planned evolution

~~~text
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
~~~

The hardware/firmware boundary is intentionally kept small so the higher-level autonomy stack can grow without coupling ROS or perception code to the motor driver.

## Repository layout

~~~text
roboty/
├── firmware/esp32/       # Real-time rover firmware
├── docs/                 # Architecture, hardware and roadmap
├── tools/                # Host-side utilities
└── .github/workflows/    # CI
~~~

## License

MIT.
