# M0 wiring — ESP32 + TB6612FNG

## ESP32 to TB6612FNG

| TB6612FNG | ESP32 | Purpose |
|---|---:|---|
| VCC | 3V3 | Logic supply |
| GND | GND | Common ground |
| STBY | GPIO 33 | Driver enable |
| AIN1 | GPIO 26 | Left direction |
| AIN2 | GPIO 27 | Left direction |
| PWMA | GPIO 25 | Left PWM |
| BIN1 | GPIO 18 | Right direction |
| BIN2 | GPIO 19 | Right direction |
| PWMB | GPIO 23 | Right PWM |

## Power side

~~~text
Battery / motor supply +  ---> TB6612 VM
Battery / motor supply -  ---> common GND

ESP32 GND ---------------+
                          +--> TB6612 GND
Battery GND --------------+
~~~

Do **not** power the motors from the ESP32 3V3 pin.

For first bench tests, power the ESP32 over USB and power the motor side separately, with grounds connected together.

## Motors

- Left motor -> AO1 / AO2
- Right motor -> BO1 / BO2

The firmware defaults to `RIGHT_INVERTED = true` because motors mounted on opposite sides of a 2WD chassis are usually mirrored. If FORWARD makes one wheel rotate the wrong way, first verify AO/BO wiring. Direction inversion can then be changed in `RobotConfig.h`.

## First power-on procedure

1. Put the rover on a support so both wheels are clear of the table.
2. Disconnect motor power.
3. Flash firmware and open serial at 115200.
4. Confirm `STATUS` responds.
5. Connect motor power.
6. Send `FWD 20` and verify wheel directions.
7. Send `STOP`.
8. Only then test on the floor.

## M1 encoder wiring

The firmware now supports quadrature encoders, but leaves them disabled with GPIOs unassigned. Confirm the exact motor/encoder model before choosing permanent encoder pins; some encoder boards require pull-ups or voltage adaptation.

Follow the [encoder calibration guide](encoder-calibration.md) to verify signal levels, select four free GPIOs, enable acquisition, measure x4 counts per wheel revolution and check direction. Encoder polarity is configured independently of the motor's electrical inversion. Unknown CPR does not prevent tick telemetry; RPM remains unavailable until configured.

The firmware rejects encoder pins shared between wheels or with the motor connections above, UART0 or boot straps. The GPIO driver also rejects unavailable/flash pins and attempts to use internal pull-ups on input-only pins. This software validation does not replace checking the actual board and encoder wiring.
