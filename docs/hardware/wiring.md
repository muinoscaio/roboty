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

## M1 encoder note

The next milestone adds quadrature encoders. Do not choose permanent encoder GPIO assignments until the exact motor/encoder model is purchased; some encoder boards require pull-ups or voltage adaptation.
