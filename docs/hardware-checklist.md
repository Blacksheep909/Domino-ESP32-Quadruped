# Hardware Planning Checklist

> Work in progress: this is a planning checklist, not a verified bill of materials. Use the [public build manual](build-guide.md) for the staged process and copy the [as-built record](build-record-template.md) to capture exact parts and measurements.

## Core Electronics

- ESP32 DevKit-style board.
- PCA9685 16-channel PWM servo driver or Domino PCB V1.1B.
- CRSF / ExpressLRS receiver.
- MPU6050 IMU.
- PCB 0-25 V divider module on GPIO36 for measured battery voltage.
- High-current servo regulator or BEC.
- Battery suitable for the selected servos and regulator.
- Common-ground wiring between servo power, ESP32, receiver, and PCA9685.

## Actuation

- Twelve 270-degree servos.
- Servo horns that can be mechanically re-clocked.
- Servo extensions or harnessing for four legs.
- Hardware for pivots, rods, inserts, and printed joints.

## Mechanical Structure

- 3D-printed body plates, clamps, pivot parts, and linkage parts.
- Carbon rods or tubes for the leg links and body rails.
- Feet or end caps for the rods.
- Springs or linkage hardware if using the same assisted linkage direction.

## Tools

- 3D printer or access to printed parts.
- Soldering iron.
- Multimeter.
- Small screwdrivers or hex drivers.
- Calipers.
- Bench power supply, if available.
- USB cable for the ESP32.
- Stand or jig that lets all four legs move without supporting the robot's weight.

## Decisions Still Required For A Rebuild

- Exact ESP32 board pinout.
- Receiver wiring and CRSF serial pin assignment.
- Servo model and safe operating voltage.
- Regulator/BEC current capacity.
- Servo horn orientation.
- Printed-part material and tolerances.
- Fastener, insert, bearing, and rod dimensions.
- Whether the builder is copying Domino directly or adapting the firmware to different geometry.

## Safety Notes

Servos can draw high current and move suddenly. Bring the system up in layers:

1. ESP32 on USB only.
2. CRSF receiver only.
3. One servo.
4. One complete leg.
5. All legs with the robot lifted safely.
6. Floor testing only after stand/stow behavior is predictable.

Do not hold the robot by a moving linkage while testing mode transitions.
