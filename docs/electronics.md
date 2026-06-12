# Electronics Notes

Domino uses the same electronics direction as the earlier SpotMicro ESP32 Nitro project: reduce wiring uncertainty by putting the major control and power interfaces onto a repeatable board, then mount that board inside a serviceable center bay.

## System Blocks

Main components:

- ESP32 DevKit controller.
- PCA9685 PWM servo driver.
- CRSF/ExpressLRS receiver input.
- External high-current servo power path.
- Regulated logic power.
- MPU6050 IMU.
- Servo harnesses for twelve 270-degree servos.

The PCB was originally designed for the earlier SpotMicro build, then reused as the basis for Domino's electronics layout. It keeps the wiring strategy consistent across robot versions.

![PCB render](images/nitro-pcb-render.png)

## Electronics Cage

The Domino body is built around a central electronics cage rather than a fully enclosed decorative shell.

Design intent:

- Keep the PCB visible and serviceable.
- Protect the electronics inside the body rails.
- Provide predictable cable routing for twelve servo leads.
- Leave access to the ESP32 USB port and receiver wiring.
- Keep the robot narrow enough that leg motion remains the dominant mechanical constraint.

This is one of the more useful mechanical lessons from the build: the electronics packaging matters as much as the leg geometry. A quadruped with twelve servos becomes a wiring project very quickly, so the center bay is designed as a functional part of the robot rather than an afterthought.

![Domino electronics cage](images/domino-electronics-cage.jpg)

## Power And Servo Control

The firmware assumes servo position commands go through the PCA9685, while the ESP32 handles RC input, kinematics, state transitions, and IMU sampling.

Practical considerations:

- Servo power must be treated separately from ESP32 logic power.
- Grounds must be common between the ESP32, receiver, servo driver, and external power.
- Large servo loads can cause brownouts if the regulator and wiring are not sized properly.
- Calibration needs both mechanical horn alignment and software trims.

## Reuse From SpotMicro Nitro

The older SpotMicro ESP32 Nitro fork is still useful because it documents the path that led to this electronics layout:

- RC control moved from rough experiments toward a repeatable transmitter setup.
- The PCB gathered the ESP32, PCA9685, power distribution, and sensor/utility connections.
- The build exposed the practical problem of packaging electronics inside a moving robot without making maintenance painful.

Domino keeps that electronics architecture but moves the mechanics to a custom body and leg system.
