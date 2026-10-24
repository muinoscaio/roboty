# Architecture

Roboty separates GPIO, motion semantics, measurement and transport so each layer can evolve without introducing ROS dependencies into the motor driver.

## M1 runtime

| Layer | Components | Responsibility |
|---|---|---|
| Configuration | `RobotConfig.h` | Motor pins/polarity; encoder opt-in, pins, polarity, pull-ups, wheel CPR and sample interval |
| Drivers | `MotorDriver`, `QuadratureEncoder` | TB6612 PWM/direction; two-channel GPIO acquisition and atomic encoder snapshots |
| Control/measurement | `DifferentialDrive`, `QuadratureDecoder`, `WheelSpeedEstimator` | Motion mixing, x4 state transitions and elapsed-time wheel RPM |
| Communication | `SerialCommandInterface`, `WheelTelemetry` | Command validation, motion watchdog, human-readable responses and JSON telemetry |
| Host | `serial_control.py`, `telemetry_capture.py` | Repeating teleoperation commands or passive CSV recording |

At startup, both motor outputs are zeroed. Encoders are initialized only if enabled and configured. Any encoder initialization failure removes handlers already installed by this startup path and reports `encoders=false`; M0 open-loop commands remain available.

Each loop snapshots encoders, updates speed estimates if a window has elapsed, processes bounded serial input, checks the motion deadline and optionally emits telemetry. Only valid motion commands refresh the deadline. RPM measurement does not feed PWM in M1.

## Encoder acquisition

Both A and B channels interrupt on both edges, producing quadrature x4 counts. The Gray-code sequence `00 -> 01 -> 11 -> 10 -> 00` is positive before optional inversion. Repeated states do not count; a two-bit transition is recorded as invalid and resynchronizes to the observed state without inventing motion.

The ISR reads GPIO registers and updates the decoder under an ESP32 `portMUX`. Loop snapshots use the same mux, keeping 64-bit counts coherent. RPM calculation and serial formatting run outside the ISR. Signed counts and invalid-transition counters saturate at their type limits rather than overflowing.

The handler, register reader and inlined decoder use RAM-safe code without a flash lookup table. The driver asks the ESP-IDF GPIO service for `ESP_INTR_FLAG_IRAM`. That service is global: if future code installs it first with different flags, accepting an already-installed service does not guarantee its dispatcher runs during flash operations. Review service ownership before adding other GPIO-interrupt users. See [Espressif interrupt allocation](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/system/intr_alloc.html) and [memory placement](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-guides/memory-types.html).

GPIO acquisition has a finite throughput and cannot reconstruct edges never observed. Test wiring noise and the maximum pulse rate on the actual hardware before relying on it for closed-loop motion. A later PCNT implementation can replace the driver without changing the measured-count contract.

## Measurement and telemetry

CPR means x4 counts per wheel revolution, including gearbox reduction. Each wheel has its own value. `WheelSpeedEstimator` uses the actual elapsed unsigned millisecond interval, including clock wrap, and calculates signed RPM. Unknown/invalid CPR leaves counting active but makes RPM unavailable at the protocol boundary.

`WheelTelemetry` contains measurement state; the command interface adds emission time and current commanded outputs. Tick counts are current snapshots; delta/RPM describe the last completed estimation window. See the [protocol](serial-protocol.md) for timing and null-value semantics.

## Fail-safe ownership

The firmware owns the movement deadline. The host refreshes motion but cannot extend its lifetime with status queries. Parsing failures stop PWM; overflow discards the entire remaining line. Input work is bounded to 64 bytes per poll, with watchdog checks interleaved.

The Python console uses one lock for repeat-state changes and serial writes. Final STOP and thread termination are ordered so an already-selected motion command cannot be written after shutdown's STOP. A timeout/disconnection ends refresh rather than silently continuing. These software mechanisms do not replace a physical motor-power disconnect.

## M2 extension point

Add per-wheel velocity targets and PID controllers between the command/motion layer and raw `MotorDriver` output. Preserve `MOTOR`/open-loop diagnostics and make STOP/watchdog clear PID state as well as PWM. Require verified encoder direction and CPR before enabling closed-loop operation. Firmware status must distinguish commanded PWM, target RPM and measured RPM.

## Design rules

1. High-level code never writes motor GPIO directly.
2. Transport does not own mixing, encoder decoding or PID mathematics.
3. Only motion commands may renew the motion deadline.
4. New modes preserve a raw/open-loop diagnostic path.
5. Physical dimensions and encoder resolution stay in configuration.
6. ROS types remain outside embedded drivers.
7. Software tests and physical validation are recorded separately.
