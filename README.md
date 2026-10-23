# Roboty

Roboty is a modular mobile robot project that starts as an ESP32 differential-drive rover and evolves toward closed-loop motor control, odometry, ROS 2, SLAM, navigation and computer vision.

## Current milestone: M1 — Encoders and wheel-speed telemetry

The software now includes:

- ESP32 / TB6612FNG open-loop motor control and differential-drive mixing.
- A 750 ms motion watchdog, strict command validation and bounded serial input processing.
- Quadrature x4 acquisition for two encoders, signed tick counts and invalid-transition diagnostics.
- Wheel RPM calculated using measured elapsed time and each wheel's configured resolution.
- On-demand or periodic JSON telemetry and a Python CSV recorder.
- A serial controller with ordered STOP/shutdown and connection-error handling.
- Native firmware tests, Python tests and an ESP32 build in GitHub Actions.

**Physical validation remains pending.** Encoders are disabled by default, their GPIOs are unassigned and their counts per wheel revolution are unknown. Enable them only after following the [encoder calibration guide](docs/hardware/encoder-calibration.md). Without that configuration, the original motor commands work and telemetry reports encoder/RPM availability explicitly. Motor control remains open-loop; PID is the next milestone.

## Hardware baseline

- ESP32 DevKit / ESP32-WROOM-32.
- TB6612FNG dual H-bridge.
- Two matched 6V geared motors with quadrature encoders.
- A 2WD chassis, wheels and a separate motor supply with common ground.

The exact encoder output levels, resolution and final GPIO assignments must be confirmed for the purchased hardware.

## Quick start

### Install and build

Python 3.10 or newer is required for the host tools; CI uses Python 3.12. Install PlatformIO and the serial dependency in a virtual environment:

~~~bash
git clone https://github.com/muinoscaio/roboty.git
cd roboty
python -m venv .venv
~~~

Activate with `.venv\Scripts\activate` on Windows or `source .venv/bin/activate` on Linux/macOS, then:

~~~bash
python -m pip install platformio==6.1.19 -r tools/requirements.txt
pio run -d firmware/esp32
~~~

The target platform remains `espressif32@6.12.0`, using Arduino ESP32 2.0.17.

### Upload

~~~bash
pio device list
pio run -d firmware/esp32 --target upload --upload-port COM5
~~~

Replace `COM5` with the detected port, for example `/dev/ttyUSB0` on Linux. Keep the wheels lifted during the first powered test.

### Manual commands

~~~bash
pio device monitor --baud 115200
~~~

~~~text
FWD 40
BACK 40
LEFT 35
RIGHT 35
DRIVE 0.5 0.0
MOTOR 40 40
STOP
STATUS
TELEMETRY
TELEMETRY 200
TELEMETRY 0
HELP
~~~

A valid motion command must be refreshed before 750 ms expires. Queries and telemetry do not extend that deadline. Malformed commands stop the motors; an omitted directional speed uses 45%, while an invalid speed is rejected.

### Continuous host control

Close the serial monitor before starting another tool on the same port:

~~~bash
python tools/serial_control.py COM5
~~~

This is a latched command console: a valid motion command repeats every 200 ms until `STOP`, an invalid command, a firmware error/watchdog message, a connection error, or exit. `STATUS`, `HELP` and `TELEMETRY` queries preserve the current repeating command. Use `EXIT`, Ctrl+C or EOF to quit with a final STOP attempt. The firmware watchdog remains responsible for stopping if USB communication is lost.

### Record encoders to CSV

With the motor supply disconnected, use this tool while turning the wheels manually:

~~~bash
python tools/telemetry_capture.py COM5 --output encoder-run.csv --interval 200 --duration 30
~~~

The recorder sends only telemetry commands. It does not drive the robot and does not share a serial port with the interactive console. RPM unavailable in JSON (`null`) is an empty CSV cell, distinct from a measured zero.

## Verify without hardware

Install a C++17 compiler (`g++` by default), then run from the repository root:

~~~bash
python tests/run_native_tests.py
python -m unittest discover -s tests/python -v
pio run -d firmware/esp32
~~~

Set `CXX` to choose another compiler. On supported hosts, `SANITIZE=1 python tests/run_native_tests.py` adds AddressSanitizer and UndefinedBehaviorSanitizer. The tests exercise the real firmware parser, motor mixing and watchdog through fake GPIO/serial, plus the hardware-independent encoder/RPM logic. They do not certify electrical wiring, interrupt throughput or motor performance.

## Documentation

- [Motor wiring](docs/hardware/wiring.md)
- [Encoder wiring checks and calibration](docs/hardware/encoder-calibration.md)
- [Serial protocol and telemetry fields](docs/serial-protocol.md)
- [Bill of materials](docs/hardware/bom.md)
- [Shopping list](docs/hardware/shopping-list.md) — existing planning reference; availability/prices have not been rechecked for M1.
- [Architecture](docs/architecture.md)
- [Roadmap](docs/roadmap.md)
- [M1 checkpoint and continuation notes](docs/checkpoints/m1-encoder-telemetry.md)

## Planned evolution

| Milestone | Scope | State |
|---|---|---|
| M0 | Open-loop rover and serial control | Software implemented; physical first drive pending |
| M1 | Encoders, RPM, telemetry and calibration tools | Software implemented; physical encoder validation pending |
| M2 | Closed-loop wheel PID | Next |
| M3 | Odometry and IMU | Planned |
| M4 | ROS 2 bridge, URDF and TF | Planned |
| M5 | Gazebo simulation | Planned |
| M6 | LiDAR and SLAM | Planned |
| M7 | Nav2 autonomous navigation | Planned |
| M8 | Camera and computer vision | Planned |

## Repository layout

| Directory | Responsibility |
|---|---|
| `firmware/esp32/` | Real-time firmware, configuration and drivers |
| `tools/` | Interactive serial control and telemetry capture |
| `tests/` | Native C++ firmware tests and Python host tests |
| `docs/` | Architecture, hardware, protocol, roadmap and checkpoints |
| `.github/workflows/` | Automated tests and ESP32 compilation |

## License

MIT.
