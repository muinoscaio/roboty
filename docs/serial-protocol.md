# USB serial protocol

Roboty uses 115200 baud, ASCII commands and LF, CR or CRLF line endings. Command names are case-insensitive; spaces and tabs separate tokens. Lines contain at most 95 characters before the terminator. Boot/help/status/errors and JSON telemetry share the port.

## Commands

| Command | Arguments | Behavior |
|---|---|---|
| `FWD` / `F` | Optional integer `0..100` | Open-loop forward |
| `BACK` / `B` | Optional integer `0..100` | Open-loop backward |
| `LEFT` / `L` | Optional integer `0..100` | Left backward, right forward |
| `RIGHT` / `R` | Optional integer `0..100` | Left forward, right backward |
| `DRIVE` | Two finite numbers `-1..1`: linear, angular | Open-loop normalized mixing; positive angular rotates left |
| `MOTOR` | Two integers `-100..100`: left, right | Raw signed wheel PWM percentages |
| `RPM` | Two finite numbers: left, right wheel RPM | Closed-loop targets, within configured magnitude limits |
| `STOP` / `S` | None | Zero both outputs and reset controllers; preserve a latched fault |
| `CLEARFAULT` | None | Acknowledge control fault while stopped; never start movement |
| `STATUS` | None | Commanded PWM, watchdog state, mode, fault and targets |
| `TELEMETRY` | None | Emit one JSON record |
| `TELEMETRY` | Integer `50..5000` | Set streaming interval in milliseconds |
| `TELEMETRY 0` | — | Disable streaming |
| `HELP` / `?` | None | Show help |

Directional commands with no percentage use `DEFAULT_DRIVE_PERCENT`, initially 45. An invalid supplied speed never falls back to that value. Extra arguments, unknown commands, out-of-range/nonfinite numbers and invalid control characters produce `ERR ...` and stop the motors. An overlong or binary-contaminated line is discarded through its terminator so its tail cannot become another movement command.

## Modes, movement lease and faults

Boot starts stopped. `RPM` requires deliberate opt-in, valid gains/limits, initialized encoders, configured CPR and a fresh completed sample. The shipped configuration rejects nonzero RPM targets. The first accepted nonzero target starts the movement lease even while PWM is initially zero awaiting a new sample.

Only accepted motion commands refresh the 750 ms watchdog. Queries, streaming, fault acknowledgement and new encoder samples do not extend it. Zero-output/zero-target commands stop movement; `RPM 0 0` is accepted with closed-loop disabled when no fault is latched. Active expiry emits `WATCHDOG stop: motion command expired` and resets PID state as well as PWM. Later samples cannot revive the expired controller without a new motion command.

`MOTOR`, `DRIVE` and directional commands exit/reset velocity mode and retain their original ranges, outside M2 output/speed caps. A latched control fault blocks these commands too. Diagnose it using STATUS, correct the cause, then acknowledge with CLEARFAULT while stopped. A new command is required to move again. Attempting CLEARFAULT while movement is commanded is rejected and stops the motors; retry while stopped. Fault acknowledgement does not validate the physical cause or prove mechanical standstill.

Mode strings are `stopped`, `open_loop` and `velocity`. Fault strings are `none`, `disabled`, `configuration`, `target_range`, `feedback_unavailable`, `stale_feedback`, `invalid_feedback`, `encoder_transition`, `no_motion`, `sample_timing` and `numeric`. See [velocity tuning](hardware/velocity-tuning.md) for their checks and limits.

The watchdog runs before PID output, during bounded serial parsing and after telemetry. At most 64 input bytes are processed per poll. This remains a cooperative software deadline, not an independent hardware emergency stop.

## Telemetry schema 2

M2 retains M1 field names and adds explicit schema, sample and control fields:

| Field | Meaning |
|---|---|
| `type` | Always `telemetry` |
| `ms` | Unsigned 32-bit firmware uptime at emission |
| `encoders` | Both encoder drivers initialized |
| `calibrated` | Both CPR settings are positive and finite; **not** physical acceptance evidence |
| `left_ticks`, `right_ticks` | Signed 64-bit quadrature x4 total counts |
| `left_delta`, `right_delta` | Signed counts in the last completed speed window |
| `left_rpm`, `right_rpm` | Signed measured wheel RPM for that window, or JSON `null` when unavailable |
| `left_invalid`, `right_invalid` | Invalid quadrature-transition totals |
| `sample_ms` | Duration of the last completed speed window; zero before the first sample |
| `left_pwm`, `right_pwm` | Current commanded signed integer percentages, not current/voltage measurements |
| `motion_active` | Whether a nonzero motion request is subject to the watchdog; can be true with PWM zero |
| `schema_version` | `2` for M2; absent in M1 |
| `sample_seq` | Unsigned 32-bit sequence, incremented once when both wheels complete a sample |
| `sample_at_ms` | Unsigned 32-bit uptime at completion of that sample |
| `control_mode` | `stopped`, `open_loop` or `velocity` |
| `control_fault` | Latched control fault, or `none` |
| `left_target_rpm`, `right_target_rpm` | Requested signed wheel RPM in velocity mode; zero outside it |

The default speed window is 100 ms, independently of streaming interval. Counts and command state are current loop snapshots; RPM/delta describe the preceding completed window. A fast stream can repeat a sample and a slower stream can omit intermediate samples. **Do not sum delta columns.** Use cumulative tick differences for displacement and sample sequence/time for completed measurement windows.

Encoder polarity is independent of motor electrical inversion. Missing CPR still permits counts; RPM stays null until both CPR values and a completed sample are available. Invalid-transition counters diagnose some signal issues, not every missed edge.

`TELEMETRY N` acknowledges with `OK telemetry_interval_ms=N`; setting the interval does not reset ticks. STOP does not disable telemetry. Rebooting resets counts, sequence and uptime; 32-bit time/sequence also wrap. Compare data within a known uninterrupted boot, not blindly across file/session boundaries.

## Python console and capture

The console repeats a valid motion command every 200 ms. STATUS/HELP/TELEMETRY preserve the repeat state. Invalid input, received ERR/WATCHDOG, an observed ready banner or valid fault telemetry cancel it. CLEARFAULT clears host repeat state and is sent once, not repeated. EOF, Ctrl+C, EXIT, serial errors and CSV write errors stop refresh with a final STOP attempt. Lost USB means the firmware must enforce its lease.

For powered bench measurements, use one connection for both control and logging:

~~~bash
python tools/serial_control.py COM5 --telemetry-csv pid-run-01.csv --interval 100
~~~

The log must not already exist. Startup sends STOP and enables telemetry only; movement requires explicit input. Normal shutdown stops movement, disables the requested stream and attempts a final STOP. A connection error can prevent these writes.

For passive/manual encoder work with motor power disconnected:

~~~bash
python tools/telemetry_capture.py COM5 --output encoder-run.csv --interval 200 --duration 30
~~~

The standalone recorder sends only telemetry commands and overwrites its chosen file. It cannot share the port with the console/monitor. Duration is optional; Ctrl+C ends capture otherwise. It attempts to disable streaming on exit.

Both readers accept M1 and M2 JSON, ignore unrelated boot/status text and reject malformed/nonfinite records. CSV contains the telemetry fields except `type`; unavailable RPM is blank. M1 gets `schema_version=1` and blank new control/sample fields rather than invented targets. The console additionally reacts to firmware error lines and control faults as described above.
