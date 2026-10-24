# Architecture

Roboty separates GPIO ownership, acquisition, measurement, control and transport. High-level features do not introduce ROS dependencies into embedded drivers.

## Runtime components

| Layer | Components | Responsibility |
|---|---|---|
| Configuration | `RobotConfig.h`, `VelocityConfig.h` | Hardware mapping/CPR, opt-in, gains and limits |
| Drivers | `MotorDriver`, `QuadratureEncoder` | TB6612 PWM/direction and protected encoder snapshots |
| Acquisition/measurement | `QuadratureDecoder`, `WheelSpeedEstimator`, `WheelTelemetry` | x4 transitions, signed counts, elapsed-time RPM and sample identity |
| Control | `WheelPid`, `VelocityControl`, `DifferentialDrive` | Per-wheel PID, feedback supervision, mode arbitration and output dispatch |
| Transport | `SerialCommandInterface` | Strict commands, movement lease, status and JSON |
| Host | `serial_control.py`, `telemetry_capture.py` | Ordered repeated commands, same-port control logging or passive capture |

Startup zeroes both outputs, configures the velocity supervisor without starting it, and initializes encoders only when opted in. An encoder initialization failure rolls back installed handlers and reports unavailable feedback. Raw M0 movement remains available unless a control fault has subsequently latched.

Each loop snapshots counts, updates both speed estimators and publishes a sequence/timestamp when a sample completes. The serial interface checks its movement lease **before** requesting a PID update, then processes bounded input and telemetry. Only accepted motion commands renew the lease. A new velocity target waits for a new sample rather than integrating stale data immediately.

## Encoder boundary

Both A/B channels interrupt on both edges. `00 -> 01 -> 11 -> 10 -> 00` counts positively before inversion. Repeated states contribute nothing; two-bit transitions increment a diagnostic and resynchronize without inventing motion. Signed counts and invalid-transition counters saturate instead of overflowing.

The ISR reads GPIO registers and updates the decoder under an ESP32 portMUX; snapshots use the same lock for coherent 64-bit reads. RPM and serial formatting stay outside the ISR. The handler/reader and inline decoder avoid a flash lookup table. The GPIO service is requested with `ESP_INTR_FLAG_IRAM`, but it is global: a service installed first by another component can have different dispatcher flags. Future interrupt users must review ownership and memory placement.

References: Espressif [interrupt allocation](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/system/intr_alloc.html) and [memory placement](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-guides/memory-types.html). No software test establishes a lossless physical pulse rate or noise immunity; PCNT can replace the acquisition driver later without changing the cumulative-count contract.

## Closed-loop boundary

`WheelPid` is hardware-independent: input is wheel RPM and elapsed seconds; output is bounded signed PWM percent. It implements conditional integration, filtered derivative on measurement and optional feedforward. Output never opposes a nonzero target direction; zero and sign changes reset controller state. This is not an active braking or speed-qualified reversal system.

`VelocityControl` owns two PIDs and a latched fault. It gates configuration and feedback, rejects stale/implausible/nonfinite data, checks sample sequence/duration, observes invalid quadrature transitions and times absence of count progress in the requested direction. Repeated or magnitude-adjusted targets do not renew that progress timer. Fresh sample sequence prevents integrating duplicate windows.

`DifferentialDrive` owns mode arbitration and is the only control layer dispatching to the motor drivers. Open-loop commands disable/reset velocity state. STOP resets state and PWM but not a fault. A fault blocks both velocity and raw commands until explicit acknowledgement while stopped; acknowledgement does not start movement. The RPM caps intentionally do not alter raw diagnostic command ranges.

The serial layer owns the 750 ms movement lease for both raw and RPM commands. A nonzero RPM request is active even before its first nonzero PWM. Watchdog expiry clears target and controller state, so future samples alone cannot restart it.

## Host and telemetry boundary

One host lock orders repeat-state changes, serial writes and final STOP. EOF, serial errors and received firmware faults stop refresh. Optional CSV output uses the same serial reader, avoiding competing applications on one port. Logging failures also abort the session. An observed ready banner cancels repetition, but there is no boot-ID handshake or guarantee of detecting every reset/lost message.

Schema 2 retains original measurements and adds targets, mode, fault, sample sequence and sample timestamp. Current command/PWM state and the preceding completed measurement window are distinct. Old M1 frames remain readable with blank missing control fields. See [protocol](serial-protocol.md) for exact semantics.

## Safety limits and verification

Software tests cover control arithmetic, synthetic plant response, protocol/driver interaction, clock wrap, fault latching, STOP ordering and host error paths. CI also compiles default and synthetic opt-in configurations. Neither is a physical motor test.

Firmware/watchdog scheduling is cooperative; physical shutdown, stopping distance, current limits, noise and actual gains remain bench responsibilities. No-motion monitoring can be fooled by false forward counts. Keep the defaults disabled until calibration and acceptance are recorded. See [velocity tuning](hardware/velocity-tuning.md).

## Next boundary: M3

Build odometry from cumulative wheel-count differences, not repeated telemetry delta rows or integrated streamed RPM. Keep wheel dimensions, separation and CPR explicit and unconfigured until measured. Add independent pose/sample identity, reset semantics and deterministic straight/arc/rotation/wrap tests. Preserve motor/watchdog ownership. Select an actual IMU before implementing its electrical driver; do not fabricate IMU data or calibration.

## Design rules

1. Drivers own GPIO; transport does not own PID/kinematics mathematics.
2. Only accepted motion commands renew the motion lease.
3. Stop, fault, acknowledgement and physical acceptance are different states.
4. Preserve a diagnostic raw mode without silently weakening fault latching.
5. Physical parameters and real calibration evidence stay explicit.
6. ROS types remain outside embedded drivers.
7. Record software and hardware validation separately, with recoverable commits.
