# Architecture

Roboty is split by responsibility so low-level hardware control can evolve independently from autonomy software.

## M0 runtime

~~~text
USB serial
    |
    v
SerialCommandInterface
    |
    v
DifferentialDrive
   / \
  v   v
MotorDriver  MotorDriver
    \       /
      TB6612FNG
       /   \
      v     v
  left DC  right DC
    motor     motor
~~~

### Layers

**drivers**

Own direct interaction with hardware peripherals. M0 contains only the TB6612 motor abstraction.

**control**

Own robot-motion semantics. The differential-drive class converts linear/angular commands into left/right wheel commands. Future PID controllers belong below this layer and above raw motor PWM.

**comms**

Own transport/protocol parsing. USB serial is the M0 transport. Future Wi-Fi, micro-ROS or a companion-computer transport should feed the same motion/control interfaces instead of controlling GPIO directly.

## Planned control path

~~~text
ROS 2 / Nav2
     |
  cmd_vel
     |
 companion computer
     |
 serial / micro-ROS
     |
    ESP32
     |
wheel velocity targets
     |
   PID L/R
     |
   motor PWM
     |
   motors
     ^
     |
  encoders
~~~

## Design rules

1. High-level code never writes motor GPIO directly.
2. Transport code never owns kinematics.
3. Motor output must fail safe to STOP when command input expires.
4. Encoder/PID work must preserve a raw/open-loop diagnostic mode.
5. Physical dimensions, encoder CPR and gear ratio stay in configuration, not scattered through control code.
6. ROS-specific types remain outside the embedded motor driver.
