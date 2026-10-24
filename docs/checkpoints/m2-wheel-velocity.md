# M2 checkpoint — wheel velocity control

## Recovery references

M1 PR #2 was merged before this work. Do not restart from the original M0 skeleton.

| Commit | Milestone |
|---|---|
| `4073cf029caf9e78fafc590591ead0144218c9de` | M1 integration into main |
| `5b351ecba1f930997dc480ee062b2c26bc10ee52` | Bounded wheel PID and latched feedback supervisor |
| `82a4492265565454b214b38fcc340bbd0b0f2d4b` | RPM/fault protocol, firmware integration and dual-configuration CI |
| `03b136029e8a757f2abb9daed132734bbf2f714d` | Same-port CSV logging, host compatibility and wrong-direction feedback checks |

Implementation tree at the last code checkpoint: `eb71c1dcaee2cdd9b43c11a4a0c4309f0f2b8e6f`.
Work branch: `feat/m2-wheel-velocity`. This document and the updated guides are a subsequent documentation checkpoint. Use the branch's integrated successor in main after merging, preserving all commits.

## Delivered

Independent wheel PID with conditional integration, filtered derivative on measurement, optional feedforward and signed output caps. `RPM L R` controls speed while old raw commands remain. STOP/watchdog reset controller state, and unusable feedback latches a fault that blocks every movement mode until explicit CLEARFAULT while stopped. Repeated targets do not mask missing progress; wrong-direction counts cannot renew the progress timer.

Telemetry schema 2 retains M1 measurements and adds targets, mode, fault and completed sample sequence/time. Python supports RPM, stopped-only acknowledgement semantics, same-port control/logging and backward-compatible M1 capture. Log/write/read errors cancel refresh, and an observed ready banner or fault telemetry also stops repeating motion. This is not a boot-ID handshake.

Encoder wiring/defaults remain unchanged. Closed-loop defaults are disabled, with gains and limits unset. Test parameters are intentionally not real robot configuration.

## Verification evidence

[GitHub Actions run 40086651552](https://github.com/muinoscaio/roboty/actions/runs/40086651552) completed successfully for code checkpoint `03b136029e8a757f2abb9daed132734bbf2f714d`:

| Check | Result |
|---|---|
| Original encoder/RPM tests | 9 cases passed |
| Original serial/control/watchdog tests | 8 groups passed |
| Original telemetry formatter | 3 real formatter frames parsed as JSON |
| M2 velocity/PID tests | 15 groups passed, including synthetic plant tracking and wrong-direction feedback |
| M2 real parser/drive integration | 9 groups passed |
| M2 telemetry formatter | 4 real formatter frames parsed as JSON |
| Python host/capture tests | All 28 passed with pyserial 3.5 installed in CI |
| Native sanitizers | AddressSanitizer and UndefinedBehaviorSanitizer enabled for both native runners in CI |
| ESP32 builds | Default configuration and isolated synthetic opt-in configuration both compiled |

Build job: `120850863272`. Test job: `120850863401`; its downloaded log explicitly records the test counts above. The preceding integration checkpoint also passed run `40086131227`.

Local verification ran the M2 native suites normally and with sanitizers (leak detection disabled for the sandbox), plus ten new fake-port Python tests with a local exception-only serial shim outside the repository. The complete repository tests and real pyserial dependency were verified in CI, not inferred from that shim. The connected desktop was offline; this work did not inspect or modify a user's local checkout or run a physical robot. A partial local test workspace must not be treated as a complete clean Git checkout.

Reproduce from a full repository checkout:

~~~bash
python -m pip install platformio==6.1.19 -r tools/requirements.txt
python tests/run_native_tests.py
python tests/run_m2_tests.py
python -m unittest discover -s tests/python -v
python tests/build_configurations.py
~~~

On supported native hosts, prefix each native runner with `SANITIZE=1`. The build script uses a temporary firmware copy for enabled encoder/PID compilation. It never uploads, assigns production GPIOs or writes real calibration.

## Hardware and behavior limits

No physical M0/M1/M2 acceptance has been completed. Real motor direction, encoder voltage/CPR, wiring quality, usable pulse rate, gains, speed/output limits, supply behavior and stopping performance remain unknown. Follow [encoder calibration](../hardware/encoder-calibration.md) and [velocity tuning](../hardware/velocity-tuning.md).

The feedback supervisor is not a current-based stall detector and cannot reject all valid-looking noise. A latched fault must be investigated, not automatically cleared by a host. PWM/speed caps apply only to velocity mode; raw diagnostics preserve their old ranges. STOP is a zero-output command, not a guarantee of immediate mechanical standstill. Cooperative software deadlines do not replace a physical motor-power disconnect.

## Next goal: M3

Start with testable differential-drive odometry based on cumulative encoder counts. Keep radii, separation and CPR explicit and unconfigured until measured. Include straight/arc/in-place rotation, zero motion, timestamp/sequence wrap, reset and duplicate/skipped sample tests. Add pose/twist telemetry without changing movement lease ownership or fault recovery.

Identify the actual IMU before selecting a hardware driver or pin assignment. Do not invent readings, bias calibration or physical validation. Preserve raw-control diagnostics and run all existing suites before publishing each checkpoint. The complete [roadmap](../roadmap.md) retains M4–M8 for ROS 2, simulation, mapping, navigation and vision.
