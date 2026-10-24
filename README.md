# Roboty

Roboty is a modular mobile robot project that starts as an ESP32 differential-drive rover and evolves toward odometry, ROS 2, simulation, mapping, navigation and vision.

## Current milestone: M2 — Closed-loop wheel velocity control

Implemented in software:

- ESP32 / TB6612FNG motor control, quadrature x4 encoders and signed wheel RPM.
- Independent wheel PID with output limits, conditional-integration anti-windup, filtered derivative on measurement and optional feedforward.
- `RPM <left> <right>` targets alongside the original open-loop commands.
- A 750 ms motion watchdog, controller reset on STOP and latched feedback faults.
- Versioned JSON telemetry with desired/measured RPM, commanded PWM and sample identity.
- A Python console that controls and records CSV through one serial connection.
- Native C++ and Python tests, plus default and opt-in ESP32 builds in CI.

**No physical acceptance has been completed.** Encoders remain disabled with unassigned GPIOs and unknown counts per wheel revolution. Closed-loop control also remains disabled, with zero/unset gains and limits. Do not enable it using the synthetic values from tests. Complete [encoder calibration](docs/hardware/encoder-calibration.md), then follow [wheel velocity tuning](docs/hardware/velocity-tuning.md).

## Hardware baseline

ESP32 DevKit / ESP32-WROOM-32, TB6612FNG, two matched 6V geared motors with quadrature encoders, a 2WD chassis and separate motor power with common ground. Confirm the purchased encoder's signal levels, resolution and final GPIO assignment before connecting it.

## Install and build

Host tools require Python 3.10 or newer; CI uses Python 3.12. Create and activate a virtual environment:

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

The firmware pins `espressif32@6.12.0` and selects C++17 explicitly.

## Upload and open-loop control

Keep the wheels lifted for the first powered test. Follow the [wiring procedure](docs/hardware/wiring.md).

~~~bash
pio device list
pio run -d firmware/esp32 --target upload --upload-port COM5
python tools/serial_control.py COM5
~~~

Replace `COM5` with the detected port, such as `/dev/ttyUSB0`. Use only one application per serial port. Example commands:

~~~text
FWD 20
MOTOR 20 20
STOP
STATUS
TELEMETRY
HELP
~~~

This is a latched command console: valid motion repeats every 200 ms until stopped or canceled. `STATUS`, `HELP` and `TELEMETRY` queries preserve the repeat command. Invalid input, firmware errors, an observed restart banner and I/O errors cancel it. EXIT, Ctrl+C and EOF end the session with a final STOP attempt. A disconnected host cannot deliver STOP; the firmware motion watchdog owns the deadline.

## Velocity control and same-port CSV

After physical calibration and deliberate configuration, start a new log:

~~~bash
python tools/serial_control.py COM5 --telemetry-csv pid-run-01.csv --interval 100
~~~

Enter `RPM <left_rpm> <right_rpm>` using conservative targets within the measured limits. `STOP` clears both controllers and outputs. A latched fault also blocks raw commands; inspect `STATUS`, correct the cause, then use `CLEARFAULT` while stopped. Acknowledgement never starts motion. The CSV records targets, measured speed, commanded PWM, mode, fault and completed sample identity. Existing console log files are not overwritten.

The closed-loop PWM and speed caps apply to `RPM`, **not** to the retained `MOTOR`/directional/open-loop commands. These remain diagnostic commands with their original ranges.

## Passive encoder capture

For manual encoder calibration, disconnect motor power and use the standalone recorder:

~~~bash
python tools/telemetry_capture.py COM5 --output encoder-run.csv --interval 200 --duration 30
~~~

It sends only telemetry requests, never motion. Unlike the console's new-file logging, this recorder overwrites its selected output file. Unavailable RPM is a blank CSV cell, not a measured zero. Do not run it concurrently with another serial application.

## Verify without hardware

Install a C++17 compiler (`g++` by default), then:

~~~bash
python tests/run_native_tests.py
python tests/run_m2_tests.py
python -m unittest discover -s tests/python -v
python tests/build_configurations.py
~~~

Set `CXX` to choose another compatible compiler. `SANITIZE=1` enables AddressSanitizer and UndefinedBehaviorSanitizer for native tests on supported hosts. The build script compiles the shipped configuration, then a synthetic opt-in configuration in a temporary copy; it never uploads or changes the real configuration. Software tests do not certify wiring, interrupt throughput, real gains or stopping distance.

## Documentation and continuation

- [Motor wiring](docs/hardware/wiring.md), [encoder calibration](docs/hardware/encoder-calibration.md) and [velocity tuning](docs/hardware/velocity-tuning.md).
- [Serial protocol](docs/serial-protocol.md) and [architecture](docs/architecture.md).
- [Bill of materials](docs/hardware/bom.md) and [shopping list](docs/hardware/shopping-list.md); purchasing information has not been rechecked for M2.
- [Roadmap](docs/roadmap.md), [M1 checkpoint](docs/checkpoints/m1-encoder-telemetry.md) and [M2 checkpoint](docs/checkpoints/m2-wheel-velocity.md).

| Milestone | Scope | State |
|---|---|---|
| M0 | Open-loop rover | Software implemented; physical acceptance pending |
| M1 | Encoders and RPM | Software implemented; physical calibration pending |
| M2 | Closed-loop wheel velocity | Software implemented; disabled until configured; physical tuning pending |
| M3 | Odometry and IMU | Next; dimensions and IMU model not yet confirmed |
| M4 | ROS 2 bridge, URDF and TF | Planned |
| M5 | Gazebo simulation | Planned |
| M6 | LiDAR and SLAM | Planned |
| M7 | Nav2 navigation | Planned |
| M8 | Camera and vision | Planned |

`firmware/esp32/` owns embedded control; `tools/` owns host interaction; `tests/` owns regression tests; `docs/` records design and acceptance evidence. GPIO ownership remains in drivers.

## License

MIT.
