# ESP32 firmware

PlatformIO firmware for the M0 Roboty rover.

## Requirements

- VS Code + PlatformIO extension, or PlatformIO CLI
- ESP32 DevKit / ESP32-WROOM-32
- TB6612FNG dual motor driver

## Build

~~~bash
cd firmware/esp32
pio run
~~~

## Upload

Connect the ESP32 over USB:

~~~bash
pio run --target upload
~~~

If PlatformIO cannot select the port automatically:

~~~bash
pio device list
pio run --target upload --upload-port COM5
~~~

Replace `COM5` with your board port.

## Serial control

~~~bash
pio device monitor --baud 115200
~~~

Commands:

~~~text
FWD 40
BACK 40
LEFT 35
RIGHT 35
DRIVE 0.5 0.0
DRIVE 0.4 -0.3
MOTOR 40 40
STOP
STATUS
HELP
~~~

The M0 firmware is deliberately open-loop. M1 will add encoder acquisition and M2 will close wheel-speed control with PID.

## Safety

Raise the rover so the wheels are off the ground for the first powered test.

The command watchdog stops both motors after 750 ms without a refreshed motion command. This is intentional: host-side teleoperation tools should continuously refresh motion while a key/button is held.
