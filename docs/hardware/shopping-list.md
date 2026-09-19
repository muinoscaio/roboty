# Shopping list — Roboty M0/M2

Last price check: **2026-09-19 (Brazil)**.

Prices below are snapshots and do not include freight. Marketplace prices can change without notice.

## Core electronics — buy now

| Item | Qty | Target / observed price | Suggested source | Why |
|---|---:|---:|---|---|
| ESP32 DOIT DevKit ESP32-WROOM-32 | 2 | ~R$40.85 each / ~R$86.92 kit 2 | https://lista.mercadolivre.com.br/esp32-doit-devkit | One for the rover + one spare/bench board |
| TB6612FNG dual H-bridge | 1 | R$15.49 Pix | https://www.usinainfo.com.br/driver-para-motor/driver-tb6612fng-duplo-ponte-h-de-motor-dc-ou-passo-4697.html | Current M0 motor driver |
| JGA25-370-CE 6V 100RPM with quadrature encoder | 2 | ~R$70–75 each | https://www.robocore.net/motor-gearmotor/motor-com-caixa-de-reducao-6v-100rpm-com-encoder | Keeps M1 encoder + M2 PID possible without replacing motors |
| Protoboard 830 points | 1 | R$11.30 Pix | https://www.eletrogate.com/protoboard-830-pontos | Bench prototyping |
| Jumper kit M-M + M-F + F-F, 120 pcs | 1 | ~R$28–32 | https://lista.mercadolivre.com.br/kit-jumpers-macho-femea | Avoid buying three separate packs |
| Hikari HM-1001 multimeter | 1 | ~R$32–50 | https://www.lojadomecanico.com.br/busca/multimetro-hikari-hm-1001 | Voltage, continuity and resistance checks |
| 100 nF ceramic capacitor | 2+ | ~R$1–5 total | electronics supplier | One directly across each motor's terminals |
| 470–1000 uF electrolytic capacitor, >= 10V | 1+ | ~R$2–8 | electronics supplier | Bulk decoupling close to TB6612 VM/GND |

### Driver current note

The selected 6V/100RPM motor is specified around **1.30 A at stall**, while common TB6612FNG modules are typically specified around **1 A continuous per channel with short peaks up to ~3 A**.

That is acceptable for an initial lightweight rover when motors run normally, but **do not intentionally hold/jam a wheel under power**. If Roboty later becomes heavier, the motor driver layer is already isolated so a higher-current driver can replace the TB6612 without changing navigation/control architecture.

## Mechanical parts

First lock the exact motor above. Its output shaft is **4 mm**.

| Item | Qty | Budget | Notes |
|---|---:|---:|---|
| 25 mm JGA25/25GA370 motor bracket | 2 | ~R$20–45 each | Must fit 25 mm motor body |
| Wheel/hub for 4 mm output shaft | 2 | ~R$20–60 pair/set | Confirm actual hub-to-wheel compatibility before ordering |
| 2WD acrylic/aluminum base | 1 | ~R$19–50 | Generic plate is fine; drilling/custom holes are acceptable |
| Ball caster / caster wheel | 1 | ~R$6–15 | Third support point |
| M3 screw/nut/standoff assortment | 1 kit | ~R$20–35 | Mount PCB, driver, battery and brackets |

Useful vendor/search references:

- 25 mm motor bracket: https://lista.mercadolivre.com.br/suporte-motor-25ga370
- 4 mm RoboCore hub reference: https://www.robocore.net/item-mecanico/hub-para-roda-neo-eixo-de-4mm
- generic 2WD acrylic base: https://www.rsrobotica.com.br/produto/base-acrilico-chassi-2wd.html
- caster/ball caster: https://lista.mercadolivre.com.br/roda-boba-robotica

**Do not buy a TT-motor-only chassis kit expecting the JGA25 motors to snap into the same mounts.** TT motors are mechanically different.

## Power for M0

For the first physical tests, use two supplies:

1. **ESP32:** USB from the PC or a normal USB power bank.
2. **Motors:** 4 x AA alkaline holder (~6 V nominal) into TB6612 `VM`.
3. **Common ground:** ESP32 GND, TB6612 GND and battery negative must be connected together.

Suggested 4xAA search/source:

https://lista.mercadolivre.com.br/suporte-pilha-arduino

Observed 4xAA holders are around **R$14–22**.

This is intentionally simpler and safer than introducing loose 18650 cells and charging/BMS circuitry in M0. A rechargeable regulated power system can be designed after motor current is measured on the real rover.

## Optional learning parts

These are useful for the electronics-learning track, but they are **not required to make M0 drive**:

| Item | Qty | Typical price |
|---|---:|---:|
| HC-SR04 ultrasonic sensor | 2 | ~R$9–12 each |
| SG90 micro servo | 2 | ~R$14–20 each |
| LEDs + resistor assortment | 1 kit | ~R$15–25 |
| 10k potentiometers | 2 | ~R$2–5 each |
| push buttons | 5+ | ~R$5–10 kit |

## Expected spend

A realistic M0/M2-capable purchase is roughly:

- electronics + two encoder motors: **~R$300–350**
- mechanical parts: **~R$90–160**
- simple motor power: **~R$20–50**

Expected total: **~R$410–560 plus freight**, depending mainly on the wheel/hub/bracket choice and whether you already own a multimeter, USB power bank, fasteners or jumper wires.

## Purchase order

1. ESP32 + TB6612 + protoboard + jumpers + multimeter.
2. Two **identical** JGA25-370-CE 6V/100RPM encoder motors.
3. After confirming their 4 mm shafts and body dimensions, buy brackets + wheels/hubs + chassis.
4. Build M0.
5. Measure real current and only then design the rechargeable battery/power stage.
6. Do **not** buy LiDAR, Raspberry Pi/mini-PC or camera yet; those belong to the ROS 2 milestones.
