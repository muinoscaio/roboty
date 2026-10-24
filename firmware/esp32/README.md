# ESP32 firmware

Roboty M0 open-loop motor control, M1 encoder acquisition and M2 closed-loop wheel RPM control.

## Build and upload

Target: ESP32 DevKit / ESP32-WROOM-32, TB6612FNG. PlatformIO pins `espressif32@6.12.0`; `platformio.ini` explicitly selects C++17.

From the repository root:

~~~bash
python -m pip install platformio==6.1.19 -r tools/requirements.txt
pio run -d firmware/esp32
pio device list
pio run -d firmware/esp32 --target upload --upload-port COM5
~~~

Replace the port and follow the root README for virtual-environment setup. First powered tests require lifted wheels and verified wiring.

## Configuration defaults

`include/config/RobotConfig.h` preserves the M0 motor GPIOs and polarity. Encoders ship disabled, four GPIOs unassigned (`-1`), both CPR values unknown (`0.0F`) and the speed window set to 100 ms. Follow [encoder calibration](../../docs/hardware/encoder-calibration.md) before enabling.

`include/config/VelocityConfig.h` ships with closed-loop disabled and gains/RPM/PWM limits unset. Real measured parameters are required; see [velocity tuning](../../docs/hardware/velocity-tuning.md). Synthetic CI constants must not be copied as real calibration.

## Commands

M0 commands remain: `FWD`, `BACK`, `LEFT`, `RIGHT`, `DRIVE`, `MOTOR`, `STOP`, `STATUS`, `HELP`. M1 telemetry remains `TELEMETRY`, `TELEMETRY 50..5000`, and `TELEMETRY 0`.

M2 adds `RPM <left> <right>` for wheel speed targets and `CLEARFAULT` to acknowledge a latched fault while stopped. RPM requires opt-in, valid limits/gains and fresh calibrated encoder feedback. STOP resets control state/PWM; it does not clear a latched fault. Faults block raw movement too until acknowledged. Raw commands retain their original ranges and do not use M2 caps.

The 750 ms watchdog owns all nonzero movement requests, including RPM targets waiting for the first output. Queries and encoder samples do not renew it. The [serial protocol](../../docs/serial-protocol.md) defines schema 2 and all command/fault semantics.

## Tests without hardware

~~~bash
python tests/run_native_tests.py
python tests/run_m2_tests.py
python -m unittest discover -s tests/python -v
python tests/build_configurations.py
~~~

Native tests need a C++17 compiler. The build script compiles the default firmware and an enabled synthetic configuration in a temporary copy; it never uploads or modifies repository defaults. GPIO/timing mocks and simulated plant response do not replace real wiring, tuning or acceptance tests.
