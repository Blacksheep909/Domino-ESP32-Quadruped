# Domino As-Built Record Template

Copy this file for each robot. Replace `MEASURE` and `VERIFY` only with physical
evidence. Attach photos, calibration JSON, LIVE CSV sessions, DMM results, and
manufacturer documents beside the completed record.

## Identity

| Field | Value |
| --- | --- |
| Robot name / serial | MEASURE |
| Builder and date | MEASURE |
| Repository commit | VERIFY |
| Firmware version/hash | VERIFY |
| Calibration JSON | VERIFY |
| PCB/controller revision | MEASURE |

## Exact BOM

| Qty | Subsystem | Manufacturer / part | Revision | Key rating | Source | Installed |
| ---: | --- | --- | --- | --- | --- | --- |
| 1 | ESP32 | MEASURE | MEASURE | Logic/input voltage | MEASURE | [ ] |
| 1 | PCA9685/PCB | MEASURE | MEASURE | Logic/servo separation | MEASURE | [ ] |
| 1 | CRSF receiver | MEASURE | MEASURE | Supply/signal voltage | MEASURE | [ ] |
| 1 | MPU6050 | MEASURE | MEASURE | I2C address | MEASURE | [ ] |
| 12 | Servos | MEASURE | MEASURE | Voltage, no-load/stall current, travel | MEASURE | [ ] |
| 1 | Regulator/BEC | MEASURE | MEASURE | Input/output, continuous/peak current | MEASURE | [ ] |
| 1+ | Battery | MEASURE | MEASURE | Chemistry, cells, capacity, C rating | MEASURE | [ ] |
| 1 | Fuse/current limit | MEASURE | MEASURE | Rating/interrupt capability | MEASURE | [ ] |
| 1 | Physical disconnect | MEASURE | MEASURE | Voltage/current rating | MEASURE | [ ] |
| 1 | Optional INA226/shunt | MEASURE | MEASURE | Address, shunt µΩ, current, dissipation | MEASURE | [ ] / N/A |

Add every printed part, fastener, insert/bearing, rod/tube, connector, wire,
sleeve, foot, and mechanical consumable as additional rows.

## Manufactured mechanics

| Item | Material/process | Print/machining settings | Critical measured dimensions | Pass |
| --- | --- | --- | --- | --- |
| Body/electronics cage | MEASURE | MEASURE | MEASURE | [ ] |
| FR / FL leg parts | MEASURE | MEASURE | MEASURE | [ ] |
| BR / BL leg parts | MEASURE | MEASURE | MEASURE | [ ] |
| Rods/tubes/pivots | MEASURE | MEASURE | Length/diameter/fit | [ ] |

Record servo-pocket fit, pivot play, mirrored orientation, horn length, linkage
branch, foot position, and full unpowered clearance.

## Power and wiring worksheet

| Check | Measured value / evidence | Pass |
| --- | --- | --- |
| Battery open-circuit voltage | MEASURE | [ ] |
| Logic rail voltage | MEASURE | [ ] |
| Servo rail unloaded / loaded minimum | MEASURE | [ ] |
| Regulator temperature after test | MEASURE | [ ] |
| Fuse/current-limit setting | MEASURE | [ ] |
| Power connector and wire gauge | MEASURE | [ ] |
| Common-ground continuity | MEASURE | [ ] |
| Power-to-ground resistance before power | MEASURE | [ ] |
| Disconnect removes servo power | VERIFY | [ ] |
| Receiver supply and TX → ESP32 RX16 | VERIFY | [ ] |
| ESP32/PCA/IMU SDA and SCL pins | MEASURE | [ ] |
| PCA9685 address/oscillator | MEASURE | [ ] |
| Optional INA226 address/shunt | MEASURE | [ ] / N/A |

Attach an as-built diagram with every connector pin, polarity, gauge,
fuse/disconnect, regulator, battery, logic/servo rail, and ground.

## Servo routing and calibration

| Joint | PCA channel | Servo identity | Neutral photo | Direction | Offset | Min | Max | Unloaded | Supported |
| --- | ---: | --- | --- | --- | ---: | ---: | ---: | --- | --- |
| FL Hip | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| FL Upper | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| FL Lower | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| FR Hip | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| FR Upper | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| FR Lower | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| BL Hip | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| BL Upper | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| BL Lower | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| BR Hip | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| BR Upper | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |
| BR Lower | MEASURE | MEASURE | LINK | MEASURE | MEASURE | MEASURE | MEASURE | [ ] | [ ] |

## Bring-up gates

| Gate | Date | Power/current limit | Evidence/log/session | Result | Notes |
| --- | --- | --- | --- | --- | --- |
| A — unpowered inspection | MEASURE | N/A | LINK | PASS / FAIL | MEASURE |
| B — USB logic | MEASURE | MEASURE | LINK | PASS / FAIL | MEASURE |
| C — receiver/IMU | MEASURE | MEASURE | LINK | PASS / FAIL | MEASURE |
| D — one unloaded servo | MEASURE | MEASURE | LINK | PASS / FAIL | MEASURE |
| E — one supported leg | MEASURE | MEASURE | LINK | PASS / FAIL | MEASURE |
| F — suspended robot | MEASURE | MEASURE | LINK | PASS / FAIL | MEASURE |
| G — restrained floor | MEASURE | MEASURE | LINK | PASS / FAIL | MEASURE |

## Safety acceptance

| Test | Expected result | Evidence | Pass |
| --- | --- | --- | --- |
| Physical disconnect | Servo rail de-energizes independently | LINK | [ ] |
| E-stop | Outputs latch off until safe reset | LINK | [ ] |
| CRSF failsafe/link loss | Motion stops in defined safe state | LINK | [ ] |
| PC link loss | Engineering authority revokes | LINK | [ ] |
| Armed watchdog | Missing heartbeat disables outputs | LINK | [ ] |
| Power cycle | Boots disarmed with PWM off | LINK | [ ] |
| Brownout/transient | No uncontrolled output; recovery understood | LINK | [ ] |

## Optional power-monitor calibration

| Condition | DMM V | INA226 V | Independent A | INA226 A | Error | Pass |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Unloaded rail | MEASURE | MEASURE | MEASURE | MEASURE | CALCULATE | [ ] |
| One moving servo | MEASURE | MEASURE | MEASURE | MEASURE | CALCULATE | [ ] |
| Suspended stand | MEASURE | MEASURE | MEASURE | MEASURE | CALCULATE | [ ] |
| Controlled dynamic test | MEASURE | MEASURE | MEASURE | MEASURE | CALCULATE | [ ] |

Record shunt temperature/dissipation, connector temperature, sampling settings,
and exact `DOMINO_POWER_*` flags.

## Final media and sign-off

- [ ] Readable wiring photos and diagram.
- [ ] Twelve neutral-horn photos.
- [ ] Supported calibration video/GIF.
- [ ] Suspended baseline and restrained-floor LIVE CSVs.
- [ ] Baseline/candidate comparison screenshot.
- [ ] Final assembly photos, measured mass, known limitations.

- Builder: `MEASURE`
- Date: `MEASURE`
- Next inspection/service date: `MEASURE`
