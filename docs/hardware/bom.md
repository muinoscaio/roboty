# Bill of materials

This is the hardware baseline for Roboty M0-M2. Exact purchase links and observed prices are maintained separately from the firmware because marketplace listings change often.

| Item | Qty | Minimum requirement | Used in |
|---|---:|---|---|
| ESP32 DevKit / WROOM-32 | 2 | USB-programmable, 3.3 V logic | M0+ |
| TB6612FNG motor driver | 1 | Dual H-bridge module | M0+ |
| DC geared motor with quadrature encoder | 2 | 6-12 V class, matched pair | M0-M3 |
| Compatible wheels | 2 | Match selected motor shaft | M0+ |
| 2WD chassis + caster | 1 | Enough space for electronics and battery | M0+ |
| Breadboard 830 points | 1-2 | Full-size | Bench |
| Jumper wires | 1 kit | M-M, M-F and F-F | Bench |
| Multimeter | 1 | DC voltage/current/resistance/continuity | Bench |
| Motor power source | 1 | Sized to chosen motors, with safe current capacity | M0+ |
| Main power switch | 1 | Rated above expected motor current | M0+ |
| 100 nF ceramic capacitors | 2+ | Motor-noise suppression | M0+ |
| Electrolytic capacitor | 1+ | 470-1000 uF near motor driver VM | M0+ |
| HC-SR04 or equivalent | 1-2 | Basic ranging for exercises | Learning |
| SG90 micro servo | 1-2 | Basic actuator exercises | Learning |

## Purchase strategy

### Buy now

ESP32, breadboard/jumpers, multimeter, TB6612FNG and the two matched encoder motors.

### Choose together

Chassis, wheels, battery and power regulation should be selected together after confirming motor shaft type, motor voltage and stall current.

### Buy later

Raspberry Pi/mini-PC, IMU, LiDAR and camera are intentionally deferred until the ROS 2 milestones.
