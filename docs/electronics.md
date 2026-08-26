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

### Measured voltage and optional current telemetry

Domino PCB V1.1B includes the same common 0-25 V divider module used by the
SpotMicro-derived board. Gerber connectivity routes the module signal to ESP32
`SENSOR_VP` / GPIO36; its input is connected to switched battery positive and
ground. Firmware 0.8.2 samples this ADC at 10 Hz, averages 16 calibrated
millivolt readings, applies the module's nominal 5:1 divider, and publishes
measured battery voltage to LIVE by default.

LIVE treats Domino's two packs as one 4S electrical pack and derives an
**average** cell voltage by dividing the measured pack voltage by four. It also
shows an estimated LiPo charge percentage from a conservative voltage curve.
Neither value is individual-cell telemetry: voltage sag under servo load,
temperature, pack age, and cell imbalance affect the estimate. Inspecting each
cell independently requires a balance-tap monitor or battery-management IC.

The inexpensive divider and ESP32 ADC both have tolerance. Compare LIVE with a
trusted multimeter under battery power, then adjust
`DOMINO_VOLTAGE_CALIBRATION_PPM` in `platformio.ini` if required. A value of
`1000000` is 1.000000x. The 25 V module was designed around a nominal 5 V ADC;
with a 4S pack, confirm its output never exceeds the ESP32 ADC input limit at
full charge before relying on the reading.

Current and watts remain unavailable unless a current monitor is installed and
configured; firmware never estimates them from servo commands. The optional
INA226 interface supports a future high-side monitor and external shunt. It
supports the 4S bus-voltage range, while the shunt, PCB copper, connectors, and
monitor module must all be selected for the robot's real peak and stall current.
Register scaling and the SIL reference case follow the
[Texas Instruments INA226 datasheet](https://www.ti.com/lit/ds/symlink/ina226.pdf).

Enable the commented `DOMINO_POWER_*` flags in `platformio.ini` only after:

- Confirming the device is an INA226 at the configured I2C address.
- Measuring the actual shunt resistance and entering it in micro-ohms.
- Choosing a maximum-current scale that covers the protected electrical path.
- Comparing voltage and current against independent bench instruments.
- Verifying shunt dissipation, creepage, wiring, fuse, and connector ratings.

Firmware 0.8.2 also uses the measured divider voltage with the configured
critical and recovery thresholds for its
latched armed-state power fault. The defaults are 12.8 V sustained for 750 ms
and 13.6 V to recover. Treat these as placeholders until the actual 4S pack,
load sag, wiring loss, and desired cell minimum have been measured. The recovery
threshold must remain higher than the critical threshold to provide hysteresis.

The endpoint publishes each field independently: the built-in divider can
populate voltage while current and watts honestly remain unavailable.

## Domino PCB V1.1B

Domino now includes a dedicated PCB manufacturing package:

- [hardware/pcb/domino-quadruped-pcb-v1.1b](../hardware/pcb/domino-quadruped-pcb-v1.1b)

This board is closely related to the SpotMicro ESP32 Nitro quadruped PCB concept, but it is cleaner and more targeted for Domino. It keeps the ESP32/PCA9685 controller direction while adding extra headers, improving connector placement and silkscreen readability, and addressing practical issues found while using the older board.

![Domino PCB V1.1B layout view](images/domino-pcb-1.png)

![Domino PCB V1.1B render](images/domino-pcb-2.png)

The PCB package is included as a prototype manufacturing reference. Before ordering or assembling it, inspect the Gerbers, confirm the connector pinout against the firmware, and bring the board up one subsystem at a time.
