# USB serial protocol

Roboty uses 115200 baud, ASCII commands and LF, CR or CRLF line endings. Command names are case-insensitive; spaces and tabs separate tokens. A line can contain at most 95 characters before its terminator. The boot banner, help, status, errors and JSON telemetry share the port.

## Commands

| Command | Arguments | Behavior |
|---|---|---|
| `FWD` / `F` | Optional integer `0..100` | Both wheels forward |
| `BACK` / `B` | Optional integer `0..100` | Both wheels backward |
| `LEFT` / `L` | Optional integer `0..100` | Left backward, right forward |
| `RIGHT` / `R` | Optional integer `0..100` | Left forward, right backward |
| `DRIVE` | Two finite numbers `-1..1`: linear, angular | Mix and normalize wheel outputs; positive angular rotates left |
| `MOTOR` | Two integers `-100..100`: left, right | Raw signed wheel PWM percentages |
| `STOP` / `S` | None | Stop both wheels immediately |
| `STATUS` | None | Print commanded PWM percentages and watchdog state |
| `TELEMETRY` | None | Emit one JSON telemetry record |
| `TELEMETRY` | Integer `50..5000` | Stream every N milliseconds |
| `TELEMETRY 0` | — | Disable streaming |
| `HELP` / `?` | None | Print available commands |

Directional commands without a percentage use `DEFAULT_DRIVE_PERCENT`, initially 45. An explicitly invalid speed never falls back to that value. Extra arguments, unknown commands, out-of-range/nonfinite numbers and invalid control characters produce `ERR ...` and stop the motors. An overlong or binary-contaminated line is discarded through its terminator, so its tail cannot become a second motion command.

Valid motion commands refresh the 750 ms watchdog, including commands that set zero output. `STATUS`, `HELP`, telemetry configuration and telemetry output never extend the motion deadline. An expired active command emits `WATCHDOG stop: motion command expired`. The watchdog is checked before input processing, between received bytes and after telemetry; at most 64 input bytes are consumed per loop iteration. This is a cooperative software deadline, not an independent hardware emergency stop.

The Python console repeats the active motion command every 200 ms. Queries preserve that repeat state. Invalid user input or a received firmware `ERR`/`WATCHDOG` cancels it and sends STOP. EOF, Ctrl+C, EXIT and serial failures end the session with a STOP attempt; if the connection is gone, the firmware watchdog stops the motors.

## Telemetry

Example only, showing a stationary rover with configured encoders:

~~~json
{"type":"telemetry","ms":1200,"encoders":true,"calibrated":true,"left_ticks":1440,"right_ticks":1440,"left_delta":0,"right_delta":0,"left_rpm":0,"right_rpm":0,"left_invalid":0,"right_invalid":0,"sample_ms":100,"left_pwm":0,"right_pwm":0,"motion_active":false}
~~~

| Field | Unit / meaning |
|---|---|
| `type` | Always `telemetry`; use this to distinguish records |
| `ms` | Unsigned 32-bit `millis()` at emission, wrapping at 2^32 |
| `encoders` | Both encoder drivers initialized successfully |
| `calibrated` | Both counts-per-wheel-revolution settings are positive and finite; physical calibration is not certified by this flag |
| `left_ticks`, `right_ticks` | Signed 64-bit x4 counts since encoder initialization |
| `left_delta`, `right_delta` | Signed counts accumulated during the last completed speed-estimation window |
| `left_rpm`, `right_rpm` | Signed wheel revolutions per minute; JSON `null` when unavailable |
| `left_invalid`, `right_invalid` | Number of observed two-bit transitions discarded by each decoder |
| `sample_ms` | Actual duration of the last completed speed window; zero before the first sample |
| `left_pwm`, `right_pwm` | Commanded signed percentages, not measured wheel speed |
| `motion_active` | Whether a nonzero commanded output is subject to the motion watchdog |

The default estimation window is 100 ms; streaming is independently configurable. Current ticks and emission time are sampled each loop, while delta/RPM/window duration describe the last completed speed sample. A 50 ms stream can therefore repeat the same speed sample. **Do not sum the delta columns across CSV rows.** Use differences of cumulative tick counts to measure displacement or calibrate counts per revolution.

Encoder inversion is independent of motor inversion. A physically forward wheel should increase its encoder count after calibration. Missing calibration still permits tick acquisition; both RPM fields stay `null` until both CPR values are configured. Invalid-transition counts diagnose some signal problems but cannot prove that no edges were missed.

`TELEMETRY <interval>` acknowledges with `OK telemetry_interval_ms=N`. Changing the interval starts a new telemetry scheduling period; it does not reset ticks. STOP does not turn off telemetry. Rebooting/reopening a reset-on-connect ESP32 restarts ticks and clock, so compare counts within one uninterrupted session.

## CSV capture

~~~bash
python tools/telemetry_capture.py COM5 --output encoder-run.csv --interval 200 --duration 30
~~~

The recorder validates the JSON fields, ignores boot/status messages and malformed records, and writes a header plus the fields listed above except `type`. Unavailable RPM becomes an empty cell. `--duration` is optional; otherwise Ctrl+C ends capture. The recorder disables telemetry on exit and never sends a motion command.

Use only one application on a serial port at a time. This recorder is intended for passive observation/manual encoder calibration; it cannot run simultaneously with the interactive controller on the same port. Close the controller or monitor before recording. The recorders' argument and I/O errors produce nonzero exit status.
