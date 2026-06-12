# Hardware Checklist

This is the practical shopping / preparation list for anyone trying to build something based on this robot.

## Core Electronics

- ESP32 DevKit style board.
- PCA9685 16-channel PWM servo driver.
- CRSF / ExpressLRS receiver.
- MPU6050 IMU.
- High-current servo power regulator or BEC.
- Battery suitable for the selected servos and regulator.
- Common-ground wiring between servo power, ESP32, receiver, and PCA9685.

## Actuation

- 12x 270-degree servos.
- Servo horns that can be re-clocked mechanically.
- Screws, nuts, heat-set inserts, or other hardware matching the printed parts.
- Enough spare servo leads or extensions to route all legs cleanly.

## Mechanical Parts

- 3D-printed body plates, clamps, pivot parts, and leg linkage parts.
- Carbon tube / rod members for the leg links and body rails.
- Feet or end caps for the rods.
- Springs or linkage hardware if using the same assisted linkage style.

## Tools

- 3D printer or access to printed parts.
- Soldering iron.
- Multimeter.
- Small screwdrivers / hex drivers.
- Calipers.
- Bench power supply if available.
- USB cable for the ESP32.
- A stand or jig that lets all four legs move without the robot supporting its own weight.

## Decisions To Make Before Building

- Which exact ESP32 board pinout you are using.
- Which receiver output is wired to ESP32 `Serial2`.
- Which servo channels are assigned to each leg.
- Whether your servo horns can be mounted close to the 135-degree electrical midpoint.
- Whether your regulator can handle stall current from multiple servos.
- Whether you are copying Domino's mechanics directly or adapting the firmware to your own body geometry.

## Safety Notes

Servos can draw high current and move suddenly. Bring the system up in layers:

1. Power the ESP32 by USB only.
2. Confirm serial logs.
3. Confirm CRSF channel reads.
4. Power one servo.
5. Power one full leg.
6. Power all legs with the robot lifted safely.

Do not hold the robot by a moving linkage while testing stand/stow transitions.
