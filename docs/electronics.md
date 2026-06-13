# Electronics Notes

Domino's electronics are built around a practical goal: reduce wiring uncertainty in a twelve-servo robot while keeping the control hardware accessible for testing, repair, and iteration.

> Work in progress: this document describes the current electronics architecture and PCB package. It is not yet a full wiring diagram or production assembly guide.

## System Blocks

The current architecture includes:

- ESP32 DevKit controller.
- PCA9685 16-channel PWM servo driver.
- CRSF/ExpressLRS receiver input.
- External high-current servo power path.
- Regulated logic power.
- MPU6050 IMU.
- Servo harnessing for twelve 270-degree servos.
- Utility headers for sensors, receiver wiring, and bring-up experiments.

## Evolution From SpotMicro Nitro

The first Domino prototype reused the PCB direction from my earlier SpotMicro ESP32 Nitro build. That board collected the ESP32, PCA9685, power distribution, receiver wiring, and sensor/utility connections into one serviceable layout.

For the initial Domino prototype, the CRSF/ExpressLRS receiver change was a wiring-level modification rather than a full PCB redesign. The receiver needed appropriate power, common ground, and its TX signal routed to the ESP32 serial RX input used by the firmware.

That approach let the robot validate the CRSF firmware and mechanical packaging before committing to a new PCB revision.

## Electronics Cage

Domino uses a central electronics cage rather than a sealed body shell. The cage is a functional mechanical feature, not decoration.

Design intent:

- Keep the PCB visible and serviceable.
- Protect the electronics inside the body rails.
- Provide predictable cable routing for twelve servo leads.
- Keep receiver and ESP32 access practical during testing.
- Prevent the body from becoming wider than the leg motion envelope.

![Domino electronics cage](images/domino-electronics-cage.jpg)

This packaging choice matters because quadrupeds become wiring projects very quickly. A central cage makes failures easier to inspect and makes the robot's design decisions visible in photos and reviews.

## Power And Servo Control

The ESP32 controls behavior and state. The PCA9685 generates servo PWM. Servo power must be supplied separately from the ESP32 USB/logic power path.

Bring-up checks:

- ESP32, receiver, PCA9685, and servo power must share ground.
- Servo voltage must match the selected servos.
- The regulator/BEC must handle realistic servo current, including stall events.
- One servo channel should be tested before all twelve servos are connected.
- Mechanical horn alignment should be corrected physically before relying on software trims.

## Domino PCB V1.1B

Domino now includes a dedicated PCB manufacturing package:

- [hardware/pcb/domino-quadruped-pcb-v1.1b](../hardware/pcb/domino-quadruped-pcb-v1.1b)

This board is closely related to the SpotMicro ESP32 Nitro quadruped PCB concept, but it is cleaner and more targeted for Domino. It keeps the ESP32/PCA9685 controller direction while adding extra headers, improving connector placement and silkscreen readability, and addressing practical issues found while using the older board.

![Domino PCB V1.1B layout view](images/domino-pcb-1.png)

![Domino PCB V1.1B render](images/domino-pcb-2.png)

The PCB package is included as a prototype manufacturing reference. Before ordering or assembling it, inspect the Gerbers, confirm the connector pinout against the firmware, and bring the board up one subsystem at a time.
