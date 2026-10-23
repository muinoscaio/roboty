# ESP32 firmware

PlatformIO firmware for Roboty M0 motor control and M1 encoder acquisition.

## Build and upload

The platform is pinned to `espressif32@6.12.0` (Arduino ESP32 2.0.17 / ESP-IDF 4.4.7). The supported board is ESP32 DevKit / ESP32-WROOM-32, with a TB6612FNG motor driver.

From the repository root:

~~~bash
python -m pip install platformio==6.1.19
pio run -d firmware/esp32
pio device list
pio run -d firmware/esp32 --target upload --upload-port COM5
~~~

Replace `COM5` with the board port. See the root README for virtual-environment setup.

## Configuration

Edit `include/config/RobotConfig.h` for motor polarity and encoder settings. Motor GPIO assignments are unchanged from M0.

The shipped encoder configuration deliberately has:

- `ENCODERS_ENABLED = false`.
- All four encoder pins set to `-1`.
- Both `*_ENCODER_COUNTS_PER_REVOLUTION` values set to `0.0F`.
- A 100 ms speed-estimation interval.

Follow [encoder calibration](../../docs/hardware/encoder-calibration.md) before assigning pins or changing these defaults. Four distinct GPIOs are required; the firmware rejects conflicts with motor pins, UART0 and boot straps. The driver also validates pin availability and internal pull-up support. A failed initialization removes installed handlers and leaves encoder telemetry disabled.

With valid encoder pins and unknown CPR, signed tick counts and invalid-transition diagnostics are still available. RPM is JSON `null` until both CPR values are positive/finite and a sample window has completed.

## Serial protocol

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

`TELEMETRY` alone emits one JSON record. `TELEMETRY 50..5000` sets the streaming interval in milliseconds; `TELEMETRY 0` disables it. Streaming starts disabled at boot.

All command details and telemetry units are in the [serial protocol](../../docs/serial-protocol.md).

## Motion behavior

Control remains open-loop: percentages command PWM, not measured RPM. Encoder feedback is observational in M1. The command watchdog stops both motors at expiry (750 ms without a valid motion refresh); queries and telemetry do not refresh it. Serial input is bounded per loop iteration, and overflow or malformed commands stop the motors.

For first motor tests, lift the wheels. Do not power the motors from the ESP32 3V3 pin. Hardware verification, including encoder signal levels and motor direction, remains pending.

## Host tests

~~~bash
python tests/run_native_tests.py
python -m unittest discover -s tests/python -v
~~~

Run these from the repository root after installing `tools/requirements.txt`. The native suite needs a C++17 compiler and runs without an ESP32.
