# Domino Quadruped PCB V1.1B

This folder contains the Gerber manufacturing package for the Domino Quadruped PCB V1.1B.

> Work in progress: this is a prototype hardware package. Inspect the Gerbers and confirm the connector pinout, power path, and mounting requirements before ordering or assembling the board.

## Files

| Path | Purpose |
| --- | --- |
| `gerbers/Gerber_Domino_Quadruped_PC_V1.1B_2026-06-13.zip` | Gerber/drill package for PCB manufacturing. |

The package contains copper layers, solder mask, silkscreen, board outline, drill files, a document layer, flying probe data, and a short ordering note.

## Design Context

This board builds on the SpotMicro ESP32 Nitro quadruped PCB concept: an ESP32-based controller board that consolidates servo control, receiver wiring, sensor connections, and power/control headers.

Domino V1.1B moves that idea toward a cleaner robot-specific board:

- Extra headers for expansion, bring-up, and receiver wiring.
- Continued ESP32/PCA9685 control architecture.
- SpotMicro-style 0-25 V battery divider routed to ESP32 GPIO36 (`SENSOR_VP`).
- Separate analog current-sensor footprint routed to GPIO39 (`SENSOR_VN`).
- Cleaner connector placement and silkscreen presentation.
- Better fit for the Domino electronics cage.
- Fixes for practical issues found while testing with the older board.

![Domino PCB V1.1B layout view](../../../docs/images/domino-pcb-1.png)

![Domino PCB V1.1B render](../../../docs/images/domino-pcb-2.png)

## Bring-Up Notes

Before installing the PCB in the robot:

1. Inspect the Gerbers in a PCB viewer.
2. Confirm board outline and mounting holes against the electronics cage.
3. Confirm receiver power, ground, and CRSF signal routing.
4. Power logic and servo rails carefully.
5. Compare the GPIO36 voltage reading against a trusted multimeter.
6. Test one servo channel before connecting all twelve servos.

See [../../../docs/electronics.md](../../../docs/electronics.md) for broader electronics notes.
