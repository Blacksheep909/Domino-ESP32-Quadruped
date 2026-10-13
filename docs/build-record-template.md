# Domino as-built record

The current state of the robot, with what is known filled in and the few gaps
listed at the end. For the parts themselves see the [bill of materials](bill-of-materials.md).

## Identity

| | |
| --- | --- |
| Robot | Domino, ESP32 composite quadruped |
| Builder | Blacksheep909 |
| Controller board | Domino PCB V1.1B ([Gerbers](../hardware/pcb/domino-quadruped-pcb-v1.1b)) |
| Firmware | 0.8.19 (`src/`, PlatformIO `esp32dev`) |
| Software | Quadruped Studio 0.4.0 |
| Repository | [Blacksheep909/Domino-ESP32-Quadruped](https://github.com/Blacksheep909/Domino-ESP32-Quadruped) |

## Servo routing

PCA9685 channel for each joint. This is the default map; the calibration wizard
can re-route any joint to another output.

| Leg | Hip | Upper | Lower |
| --- | ---: | ---: | ---: |
| Front-left (FL) | 0 | 1 | 2 |
| Front-right (FR) | 3 | 4 | 15 |
| Back-left (BL) | 14 | 7 | 8 |
| Back-right (BR) | 9 | 10 | 11 |

## Wiring

| Signal | Connection |
| --- | --- |
| Servo PWM | PCA9685, 12 of 16 channels (5, 6, 12 and 13 unused) |
| PCA9685 control | ESP32 I2C (`Wire.begin()` defaults) |
| CRSF receiver | Receiver TX to ESP32 UART2 RX (GPIO16), 5 V supply, common ground |
| Battery voltage | 0-25 V sensor module to ESP32 GPIO36 (`SENSOR_VP`) |
| Servo power | Battery, relay module, SZBK07 buck converter, PCA9685 servo rail |
| Logic power | Battery, logic-rail step-down, ESP32 and modules |
| IMU | Not fitted (`DOMINO_IMU_ENABLED=0`) |
| Current monitor | Not fitted (`DOMINO_POWER_*` flags off) |

ESP32, receiver, PCA9685 and the servo supply share one ground.

## Calibration

Each robot keeps its own calibration. It lives in the ESP32's flash and can be
exported as a JSON file from Studio. Export one after every change and keep it
with this record. See the [calibration guide](calibration-guide.md).

## Bring-up

Follow the [LIVE hardware bring-up procedure](live-hardware-bring-up.md) before
any motion: unpowered inspection, USB logic, receiver, one unloaded servo, one
supported leg, a suspended robot, then a restrained floor test.

## Still to record

- Exact servo model numbers.
- Rod diameters and lengths, fastener sizes.
- Fuse rating and wire gauge.
- Measured mass.
- Wiring photos and a connector-level wiring diagram.
