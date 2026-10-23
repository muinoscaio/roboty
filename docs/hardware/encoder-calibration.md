# M1 encoder setup and calibration

M1 measures signed wheel counts and wheel RPM while retaining the M0 open-loop motor commands. The exact purchased motor/encoder model, electrical output and counts per revolution are still unknown. Encoder acquisition is therefore disabled by default. Physical M0 and M1 validation remains pending; a successful firmware build does not validate the wiring or a real encoder.

## 1. Identify the encoder before connecting it

Record the motor/encoder part number and its datasheet, supply voltage, A/B signal voltage, output type, wire assignment and gearbox ratio. Do not infer wire colors or encoder characteristics from another motor with a similar name.

The ESP32 inputs need signals compatible with 3.3 V logic. An encoder powered from 5 V may also drive its outputs to 5 V. Confirm whether the outputs are push-pull or open-collector/open-drain and whether the module already contains pull-up resistors. Use a suitable level interface when needed; do not connect an unverified 5 V output directly to an ESP32 GPIO. Pull-ups for compatible open-collector outputs must reference the appropriate 3.3 V logic supply, not an unknown encoder supply. The ESP32 input levels and internal pull resistance are specified in the [official datasheet, DC characteristics](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf).

Keep the ESP32 and encoder grounds common. During manual calibration, disconnect motor power at the TB6612 VM supply and keep only the ESP32/encoder logic powered according to their specifications. Support the chassis so the wheels are clear. Turn a wheel by hand only if the gearbox permits it without force.

## 2. Choose four GPIOs for the actual board

No permanent encoder pinout is assigned in this project. Check the exact board/module schematic before selecting A/B for each wheel.

For the ESP32-WROOM-32 target, the current firmware rejects duplicate pins, motor-control pins, UART0 GPIO1/3, boot-strapping GPIO0/2/5/12/15, flash GPIO6–11 and invalid/unexposed GPIOs. Preserve the motor assignments in [M0 wiring](wiring.md). Check for any additional peripherals on your board as well.

GPIO34–39 are input-only and have no internal pull-up or pull-down resistors. The driver rejects enabling its internal pull-up option on these pins. GPIO36/39 also have an interrupt caveat when using ADC or Wi-Fi/Bluetooth with sleep. GPIO16/17 availability depends on the module's flash/PSRAM connections. See the [Espressif GPIO restrictions for ESP-IDF 4.4.7](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/peripherals/gpio.html).

The software checks do not detect incorrect voltages, board-level wiring or a signal connected to the wrong physical header pin.

## 3. Configure and enable acquisition

Edit [`firmware/esp32/include/config/RobotConfig.h`](../../firmware/esp32/include/config/RobotConfig.h).

| Setting | Default | What to configure |
|---|---:|---|
| `ENCODERS_ENABLED` | `false` | Set `true` after verifying the electrical interface and all four pins. |
| `LEFT_ENCODER_A_PIN`, `LEFT_ENCODER_B_PIN` | `-1`, `-1` | Left wheel's A/B GPIO numbers. |
| `RIGHT_ENCODER_A_PIN`, `RIGHT_ENCODER_B_PIN` | `-1`, `-1` | Right wheel's A/B GPIO numbers. |
| `LEFT_ENCODER_PULLUP`, `RIGHT_ENCODER_PULLUP` | `false`, `false` | Enable only if the chosen pins and encoder output circuit support an internal pull-up. |
| `LEFT_ENCODER_INVERTED`, `RIGHT_ENCODER_INVERTED` | `false`, `false` | Reverse the measured count sign for that wheel. |
| `LEFT_ENCODER_COUNTS_PER_REVOLUTION` | `0.0F` | Measured x4 counts per complete left wheel revolution. |
| `RIGHT_ENCODER_COUNTS_PER_REVOLUTION` | `0.0F` | Measured x4 counts per complete right wheel revolution. |
| `ENCODER_SAMPLE_INTERVAL_MS` | `100` | Minimum interval between wheel-speed samples; must be positive. |

Leave both counts-per-revolution values at zero for the first acquisition test. Counts work without known CPR; RPM stays `null`. Rebuild and upload after changing configuration:

~~~bash
pio run -d firmware/esp32
pio run -d firmware/esp32 --target upload
~~~

The firmware reports disabled acquisition, initialization failure, missing CPR or enabled acquisition at startup. `encoders: true` means that both encoder drivers initialized successfully. It does not prove that the signals are wired correctly.

## 4. Read counts and confirm direction

Open a single serial tool at 115200 baud. For example, from the repository root:

~~~bash
pio device monitor --baud 115200
~~~

Use the following commands, terminated with a newline:

~~~text
STOP
TELEMETRY
TELEMETRY 200
TELEMETRY 0
~~~

`TELEMETRY` prints one JSON record. `TELEMETRY 200` starts a record every 200 ms; `TELEMETRY 0` stops streaming. These commands do not request movement or refresh the motion watchdog.

Turn one wheel slowly in the direction that would propel the rover forward. Its tick count should increase while the other wheel's count remains unchanged. Turn backward and confirm that it decreases. If the sign is reversed, change that wheel's `*_ENCODER_INVERTED` setting, rebuild and repeat. Encoder inversion is independent of `LEFT_INVERTED`/`RIGHT_INVERTED`, which control the motors' electrical direction.

Roboty counts all four valid quadrature transitions: `00 -> 01 -> 11 -> 10 -> 00` contributes four counts before inversion. Repeated states do not count. A simultaneous two-bit change is recorded as an invalid transition and contributes no displacement. An increasing `left_invalid` or `right_invalid` value calls for checking signal quality, wiring and acquisition rate; zero invalid transitions alone does not prove that no edges were missed.

## 5. Measure wheel CPR

**CPR here means x4 counts per complete revolution of the wheel/output shaft, including the gearbox.** A manufacturer's PPR may refer to a single encoder channel or the motor shaft. Do not copy a PPR value into the configuration without checking its definition and reduction ratio.

Perform this procedure separately for each wheel with motor power disconnected:

1. Mark the wheel and a fixed reference point on the chassis.
2. Keep the same serial session open and let the wheel rest at the mark.
3. Record the wheel's initial total tick count from `TELEMETRY` or a captured row.
4. Turn the wheel through a known number `N` of complete revolutions in one direction, finishing at the same mark. Several revolutions reduce the effect of a small alignment error.
5. Stop turning, then record the final total tick count in the same session.
6. Calculate the value below and record `N`, both raw counts and the result.
7. Repeat in the opposite direction and repeat the measurement to check consistency.

~~~text
CPR = abs(final_total_ticks - initial_total_ticks) / N
~~~

Use `left_ticks` or `right_ticks` for this calculation. Do not use a single `left_delta`/`right_delta`, since those fields cover only the most recent speed-sampling window.

Opening a serial port can reset some ESP32 development boards; uploading or resetting also restarts the counters. Never subtract an initial reading from one boot/session from a final reading in another. A decreasing `ms` can indicate either reset or its normal 32-bit clock wrap; retain the session/boot context.

Enter each measured CPR into its matching configuration setting and upload again. Both CPR values must be positive and finite before this version publishes RPM for either wheel. `calibrated: true` means that both CPR values are configured; it is not a record of a successful physical calibration.

## 6. Capture a manual calibration run to CSV

Install the host dependency and run the capture tool from the repository root:

~~~bash
python -m pip install -r tools/requirements.txt
python tools/telemetry_capture.py COM5 --output encoder-left-run1.csv --interval 200 --duration 30
~~~

Replace `COM5` with the detected port, such as `/dev/ttyUSB0` on Linux. Choose a fresh output filename for each run; an existing file at that path is overwritten. Increase the duration if needed to complete the chosen number of turns.

The capture tool only requests/disables telemetry and reads records; it sends no motion command. It enables streaming on entry and attempts to disable streaming on exit. Unavailable RPM becomes an empty CSV cell. Keep the wheel at its reference mark for the first recorded row, perform the turns, and hold the final position long enough for another row before capture ends.

**Use one tool per serial port.** Close the PlatformIO monitor or interactive controller before starting capture. Do not run `serial_control.py` and `telemetry_capture.py` on the same port concurrently. Each capture may start a new board session, so use the initial and final rows from that capture only.

### Reading the telemetry fields

| Fields | Meaning |
|---|---|
| `ms` | Firmware uptime in milliseconds at emission; wraps after the 32-bit range. |
| `encoders`, `calibrated` | Both drivers initialized; both CPR values configured. Neither is physical acceptance evidence. |
| `left_ticks`, `right_ticks` | Signed totals from the most recent loop snapshot. |
| `left_invalid`, `right_invalid` | Invalid transition totals from the most recent snapshot. |
| `left_delta`, `right_delta` | Signed counts in the last completed speed-sampling window. |
| `sample_ms` | Actual duration of that last completed window, not the telemetry interval or a timestamp. |
| `left_rpm`, `right_rpm` | Signed speed from that window; `null` until acquisition, both CPR values and a completed sample are available. |
| `left_pwm`, `right_pwm`, `motion_active` | Commanded motor percentages and motion-watchdog state. |

Speed uses the actual elapsed sample time:

~~~text
RPM = delta_ticks * 60000 / (CPR * sample_ms)
~~~

Ticks and emission time can advance while the last completed speed sample remains unchanged. Streaming faster than the sampling interval can therefore repeat the same delta; streaming more slowly can skip intermediate deltas. **Do not sum the CSV delta columns.** Use differences of total ticks for displacement and the provided RPM/sample duration for the last measured speed window.

## 7. Physical acceptance record — still pending

Save a bench record with the firmware commit, board/module model, motor/encoder model, voltage measurements, signal interface, GPIO mapping, inversion settings, CPR measurements, CSV filenames and observed results.

- [ ] M0: confirm wheel direction and independent motor control with the chassis supported.
- [ ] M0: confirm explicit STOP and watchdog stop after motion-command refresh ends.
- [ ] M0: complete and record the first controlled floor test/video.
- [ ] M1: confirm both encoder channels and each wheel's signed counts by hand.
- [ ] M1: measure each CPR repeatedly in both directions in uninterrupted sessions.
- [ ] M1: verify RPM against an independent physical timing/speed reference.
- [ ] M1: inspect counts and invalid-transition totals across the intended speed range.
- [ ] M1: repeat watchdog and STOP checks while telemetry is streaming.

For powered tests, follow the [M0 wiring procedure](wiring.md). Continue using open-loop commands; M1 does not automatically correct wheel speed.

Estimate the interrupt event rate for each wheel as `CPR * abs(RPM) / 60`, then consider both wheels together. GPIO interrupt acquisition has finite service capacity. Native tests validate decoding and arithmetic, but cannot establish a lossless hardware rate or immunity to electrical noise. Validate the actual combination of encoder, wiring, firmware load and speed before accepting the measurement.

The driver uses an IRAM handler and protected counter snapshots. The GPIO ISR service is shared: a service installed earlier by another component can have different interrupt flags, and future handlers must respect the service's requirements. IRAM placement alone is not a guarantee of uninterrupted counting during flash operations. See Espressif's [interrupt allocation](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/system/intr_alloc.html), [memory placement](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-guides/memory-types.html) and [FreeRTOS critical-section guidance](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-guides/freertos-smp.html).

The pinned [PlatformIO Espressif32 6.12.0 release](https://github.com/platformio/platform-espressif32/releases/tag/v6.12.0) uses Arduino ESP32 2.0.17 with ESP-IDF 4.4.7, matching the API references above.
