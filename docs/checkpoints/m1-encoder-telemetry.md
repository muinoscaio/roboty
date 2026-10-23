# M1 checkpoint — encoder acquisition and telemetry

## Recovery references

- Starting `main`: [`5994e3e8c7a66b6d0caaecc76a292165f0b9395b`](https://github.com/muinoscaio/roboty/commit/5994e3e8c7a66b6d0caaecc76a292165f0b9395b).
- Implementation checkpoint: [`cb79e8340bf812bbd48160fd8d7e4373d6101e7e`](https://github.com/muinoscaio/roboty/commit/cb79e8340bf812bbd48160fd8d7e4373d6101e7e).
- Implementation tree: `71279ea16819fbe40b524fd6af9acc05cac8ae15`.
- Work branch: `feat/m1-encoder-telemetry`.

The implementation tree was compared with the published GitHub tree, then fetched back and matched to the local index. Documentation is recorded in a subsequent checkpoint on the same branch.

## Delivered behavior

M1 adds quadrature x4 acquisition for both wheels, atomic signed counters, invalid-transition diagnostics, actual-interval RPM estimates, JSON telemetry and passive CSV capture. GPIOs and CPR are intentionally unassigned; encoder acquisition is opt-in. M0 motor control remains open-loop and available with encoders disabled.

This milestone also fixes M0 parser and host-controller defects: invalid directional speeds cannot start default-speed motion, oversized-line suffixes are discarded, extra tokens and integer overflow are rejected, input processing is bounded, and final host STOP cannot be overtaken by a refresh write. Firmware errors, EOF and connection failures cancel host refresh.

## Validation recorded

| Check | Evidence |
|---|---|
| Native encoder/RPM suite | 9 passing cases |
| Native parser/control/watchdog suite | 8 passing groups, including malformed input, overflow, binary input, clock wrap and query floods |
| Firmware JSON output | 3 actual emitted frames parsed by Python; null RPM, signed 64-bit extrema and scientific notation verified |
| Python console/recorder | 18 passing tests, including forced STOP/refresh interleaving and disconnect while terminal input is blocked |
| Native memory/undefined behavior | AddressSanitizer and UndefinedBehaviorSanitizer passed; local leak detection disabled for the sandbox restriction |
| Default ESP32 configuration | PlatformIO build passed with encoders disabled |
| Enabled encoder configuration | Build/link passed in an isolated temporary firmware copy; pins and CPR were synthetic test values, not the rover configuration |
| ISR placement | Enabled-build ELF/objdump confirmed the handler, GPIO reader and critical-section helpers in IRAM |
| GitHub Actions | [`39894443181`](https://github.com/muinoscaio/roboty/actions/runs/39894443181) succeeded for `cb79e8340bf812bbd48160fd8d7e4373d6101e7e` |

The enabled build used GPIOs 21/22 and 34/35, no internal pulls, and CPR 1440 solely to exercise the code path and linker. Those values were never applied to the repository's default configuration. Both builds used about 6.7% RAM; the enabled image used about 22.2% of the configured application flash partition.

Reproduce software checks from the repository root:

~~~bash
python -m pip install platformio==6.1.19 -r tools/requirements.txt
python tests/run_native_tests.py
python -m unittest discover -s tests/python -v
pio run -d firmware/esp32
~~~

On a host supporting the sanitizers:

~~~bash
SANITIZE=1 python tests/run_native_tests.py
~~~

## Physical acceptance remains open

No physical robot was connected for this work. Do not infer tested wiring, correct CPR, wheel polarity, maximum pulse rate or PID readiness from software checks or the `calibrated` telemetry flag.

Follow [encoder calibration](../hardware/encoder-calibration.md), fill in its bench record, and retain a capture from a single uninterrupted serial session. M0 motor-direction/STOP/watchdog checks and the first-drive video are also still pending.

## Continue with M2

Use the current branch or its integrated successor; do not restart from the original M0 skeleton. Read the [roadmap](../roadmap.md), [protocol](../serial-protocol.md) and [architecture](../architecture.md) before changing command semantics.

Next implementation tasks are per-wheel velocity targets and PID with anti-windup/output limits, explicit open-loop/closed-loop modes, controller reset on STOP/watchdog and step-response telemetry. Keep driver GPIO ownership and acquisition separate from control math. Tests can cover PID logic without hardware, but real gain tuning, output limits and feedback-failure behavior must be validated against measured encoder and motor behavior before closed-loop motion is accepted.
