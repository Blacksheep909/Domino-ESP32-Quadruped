# Domino Quadruped PCB V1.1B

This folder contains the Gerber manufacturing package for the Domino Quadruped PCB V1.1B.

> Work in progress: review the board files and your exact hardware requirements before ordering. This repository documents the prototype hardware path, not a guaranteed production release.

## Files

| Path | Purpose |
| --- | --- |
| `gerbers/Gerber_Domino_Quadruped_PC_V1.1B_2026-06-13.zip` | Gerber/drill package for PCB manufacturing. |

The Gerber package contains copper layers, solder mask, silkscreen, board outline, drill files, a document layer, flying probe data, and a short ordering note.

## Design Notes

This PCB revision is closely related to the SpotMicro ESP32 Nitro quadruped PCB concept: an ESP32-based controller board that gathers the robot's servo control, receiver wiring, sensor wiring, and power/control headers into a repeatable layout.

The Domino V1.1B board moves that idea forward:

- Adds extra headers for cleaner expansion and bring-up wiring.
- Keeps the ESP32/PCA9685 quadruped-control direction from the Nitro board.
- Improves the board presentation with a cleaner layout and silkscreen direction.
- Optimizes the practical connector layout for the Domino electronics cage.
- Resolves several practical board-level issues exposed by the older board during robot testing.

The goal is still serviceability. Domino has twelve servos, a CRSF/ExpressLRS receiver, sensor wiring, and external servo power, so the PCB should reduce wiring uncertainty rather than hide what the robot is doing.

![Domino PCB V1.1B layout view](../../../docs/images/domino-pcb-1.png)

![Domino PCB V1.1B render](../../../docs/images/domino-pcb-2.png)

## Bring-Up Notes

Before installing the PCB in the robot:

1. Inspect the Gerbers in a PCB viewer.
2. Confirm the board outline and mounting holes match the electronics cage.
3. Confirm receiver power, ground, and CRSF signal routing before plugging in a receiver.
4. Power logic and servo rails carefully; do not rely on USB power for servos.
5. Test one servo channel before connecting all twelve servos.

See [../../../docs/electronics.md](../../../docs/electronics.md) for the broader electronics notes.
