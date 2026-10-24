# M2 wheel velocity setup and tuning

M2 adds software PID control of wheel RPM. **No real gains, motor limits, stopping distance or tracking performance have been validated.** The shipped configuration deliberately cannot start nonzero closed-loop motion. Test constants and simulated plant results are not settings for the purchased robot.

## Prerequisites

Complete the [M0 wiring checks](wiring.md) and [M1 encoder calibration](encoder-calibration.md). Confirm both wheels independently: motor-forward means positive encoder counts and positive measured RPM. Verify CPR at the wheel/output shaft, signal levels, noise and pulse-rate capacity. Retain the measurements and firmware commit in a bench record.

For initial powered tests, support the chassis with wheels clear, use a suitable current-limited motor supply and keep a physical motor-power disconnect accessible. Do not intentionally hold a powered shaft by hand or short encoder signals to test faults. Inject feedback failures in native tests first; any physical fault test requires a controlled bench setup.

The watchdog and feedback supervisor are cooperative software, not a certified emergency stop. Setting PWM to zero is not a guarantee of immediate mechanical stopping. M2 does not implement a current sensor, independent hardware shutdown or a guaranteed obstacle stop.

## Configuration

Encoder GPIOs, polarity and CPR remain in `firmware/esp32/include/config/RobotConfig.h`. Configure M2 in `firmware/esp32/include/config/VelocityConfig.h`:

| Setting | Shipped value | Meaning |
|---|---:|---|
| `CLOSED_LOOP_ENABLED` | `false` | Deliberate opt-in after prerequisite checks |
| `VELOCITY_MAX_TARGET_RPM` | `0.0` | Maximum magnitude of either requested wheel speed |
| `VELOCITY_MAX_FEEDBACK_RPM` | `0.0` | Maximum credible magnitude of measured wheel RPM; at least the target cap |
| `VELOCITY_MAX_PWM_PERCENT` | `0.0` | Closed-loop output cap; configured range 1..100 percent |
| `LEFT_VELOCITY_PID`, `RIGHT_VELOCITY_PID` | Zero gains | Independent wheel gains, described below |
| `VELOCITY_FEEDBACK_TIMEOUT_MS` | `300` | Sample age and duration must each be strictly below this value |
| `VELOCITY_NO_MOTION_TIMEOUT_MS` | `750` | Maximum time without encoder progress in the requested direction |

The no-motion timeout must exceed the feedback timeout. Supported configuration ranges are 1..10000 ms for feedback timeout and at most 60000 ms for no-motion timeout. These are validation bounds, not recommendations to use long timeouts. Select actual limits from measured hardware behavior, safe bench constraints and the minimum speed you need to control. Very slow movement may take too long to produce a tick; that must be accounted for without masking a disconnected encoder.

Gain order is `{kp, ki, kd, feedforward, derivativeFilterSeconds}`. PWM is represented as percent, speed as wheel RPM and time as seconds:

- `kp`: percent per RPM of error.
- `ki`: percent per RPM-second of integrated error.
- `kd`: percent-second per RPM of measured speed change.
- `feedforward`: percent per requested RPM; optional.
- `derivativeFilterSeconds`: positive derivative filter time constant; default 0.02 seconds.

All gains must be finite and nonnegative. At least one of `kp` or `ki` must be positive for each wheel. Configuration is compiled into the firmware and persists through the image; no runtime gain-writing or NVS calibration command is implemented. Save measured values and their evidence in version control when the hardware has been characterized.

Rebuild and upload only after reviewing the real values. `tests/build_configurations.py` uses a temporary synthetic configuration solely to exercise the compiler/linker and never uploads it.

## Control semantics

`RPM L R` uses signed wheel RPM. A newly accepted nonzero target waits for the next completed encoder sample before producing output. Each PID integrates once per sample, using the actual sample duration. Repeated serial commands renew the movement lease without repeatedly integrating the same sample or resetting the no-motion timer.

The controller uses conditional integration to limit windup and a filtered derivative of measurement to avoid a derivative jump caused solely by a target change. PWM is capped independently for each wheel and truncated toward zero when passed to the integer-percent driver. Expect quantization at low outputs.

For a positive target, the output stays between zero and the positive cap; for a negative target it stays between the negative cap and zero. There is no reverse braking torque to slow an overspeed wheel. A zero wheel target clears that wheel's controller/output. A sign reversal clears its controller/output before a later sample can command the opposite direction. This is not a measured-speed reversal interlock: use STOP and allow the wheels to stop before reversing during initial bench work.

`STOP`, a malformed command or motion-watchdog expiry clears both PID states and outputs. Switching to an open-loop command also leaves velocity control. **Open-loop `MOTOR`, `DRIVE` and directional commands do not use the M2 speed/PWM caps.** Their original ranges are retained for diagnosis.

## Faults and recovery

A control fault latches and stops both wheels. It blocks RPM and raw movement until acknowledged. STOP does not erase it, and valid feedback returning does not automatically resume movement.

| Fault | Inspection |
|---|---|
| `disabled`, `configuration` | Check opt-in, gains, speed/PWM limits and timeout validity |
| `target_range` | Check finite targets against the configured magnitude cap |
| `feedback_unavailable` | Both encoders, both CPR values and a completed sample are required |
| `stale_feedback`, `sample_timing` | Inspect sample age, duration and sequence continuity |
| `invalid_feedback` | Inspect nonfinite or implausibly high measured RPM |
| `encoder_transition` | An invalid-transition counter changed while controlling; inspect acquisition/wiring |
| `no_motion` | One targeted wheel showed no progress in the requested direction within the timeout |
| `numeric` | PID arithmetic failed a finite-value check; inspect configuration and input |

Wrong-direction encoder counts do not keep the no-motion timer alive. This is still not a complete stall/current detector: valid-looking electrical noise or other erroneous forward counts can deceive it. Do not treat it as a substitute for physical monitoring.

After identifying and correcting the cause:

~~~text
STOP
STATUS
CLEARFAULT
STATUS
~~~

`CLEARFAULT` requires stopped commanded outputs/controllers and never starts motion. It does not certify that the mechanical wheels are at rest or the fault cause has been fixed. A fresh movement command is required and prerequisites are rechecked.

## Capture and tune on one serial port

Close other serial tools, then create a new CSV:

~~~bash
python tools/serial_control.py COM5 --telemetry-csv pid-run-01.csv --interval 100
~~~

Replace the port as appropriate. The file is opened exclusively, so an existing log is not overwritten. The console starts with STOP and telemetry configuration, not a movement command. Enter `RPM <left> <right>` only after the setup is ready and with targets appropriate to the verified hardware limits. The command is repeated every 200 ms; use STOP before inspecting or handling the robot. Queries do not stop this repeat command.

Suggested measured tuning sequence:

1. Start with a low output cap, one wheel targeted and the other at zero. Begin with proportional control; keep integral, derivative and feedforward zero until their need is established.
2. Apply a small target step above the motor's verified minimum controllable speed. Record target, measured RPM, PWM, fault state and timing. Stop between experiments.
3. Adjust proportional gain in small increments. If response oscillates or the cap is continually reached, stop and review the gain, target, friction, power supply and cap rather than increasing every value together.
4. Add integral gain gradually when a persistent steady-state error remains. Verify recovery from saturation and STOP. Add derivative only when justified by measured dynamics; inspect encoder quantization/noise before increasing it.
5. Repeat for the other wheel and both directions. Then test paired targets, stopping, direction changes after stopping, different supported loads and supply conditions.
6. Retain the configuration, firmware commit, setup, captures and acceptance criteria for each trial. Do not call simulated tracking evidence a physical result.

Use `sample_seq` and `sample_at_ms` to identify completed speed windows within one uninterrupted boot. Streaming faster than acquisition repeats the same window; slower streaming skips windows. Do not sum CSV delta columns. Use cumulative tick differences for displacement. CSV `ms` is emission time; PWM/targets are current commanded state while measured speed describes the preceding completed window, so do not assume they are simultaneous physical measurements.

M1 frames remain readable, but new sample/target/control fields are blank. Missing values must not be interpreted as measured zero. Rebooting resets sequence, counts and time; unsigned counters also wrap. Keep boot/session context rather than concatenating runs blindly.

The host cancels its repeated command on an observed restart banner, firmware error/watchdog message or valid fault telemetry. Serial/log errors stop refresh and end the session with a STOP attempt. These checks do not implement a boot-ID handshake and cannot guarantee detection of every reset or lost message. The firmware remains responsible for the motion lease.

## Physical acceptance — still pending

- [ ] Record actual board, motor/encoder model, supply, wiring and measured CPR/polarity.
- [ ] Select and justify per-wheel gains, RPM/PWM caps and timeouts from bench measurements.
- [ ] Measure speed tracking, steady-state error, oscillation, saturation recovery and minimum reliable speed.
- [ ] Verify STOP and the 750 ms motion watchdog while telemetry is streaming.
- [ ] Verify controlled feedback-loss/wrong-direction/no-motion scenarios without automatic restart.
- [ ] Verify open-loop recovery only after explicit fault acknowledgement.
- [ ] Save captures and real configuration; record limitations, stopping behavior and load range.

## Reference

Conditional-integration anti-windup is also documented in the ROS control toolbox [PID API](https://control.ros.org/rolling/doc/api/classcontrol__toolbox_1_1Pid.html). Roboty's implementation is local C++ with native regression tests; it has no runtime dependency on ROS.
