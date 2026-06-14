# Domino ESP32 Composite Quadruped

Domino is a prototype ESP32 quadruped robot built around a carbon-and-printed composite chassis, a custom 3-DoF leg mechanism, CRSF/ExpressLRS radio control, PCA9685 servo output, and CAD-to-simulation experiments.

> Work in progress: Domino is an active engineering prototype, not a finished kit. The firmware, CAD exports, PCB package, calibration notes, and simulation files are included as a project record and technical reference. The repo does not yet contain a complete step-by-step build manual, measured BOM, wiring diagram, or validated production assembly process.

![Domino quadruped robot build](docs/images/domino-master.jpg)

## Why This Project Matters

Domino is the successor to my earlier SpotMicro ESP32 Nitro work. The earlier project gave me a working base for servo-driven quadruped control, PCB packaging, and RC-controlled robot bring-up. Domino moves beyond that reference design into a more custom platform:

- A scratch-built mechanical layout using carbon members and 3D-printed structural parts.
- A serviceable electronics cage rather than a sealed body shell.
- Per-leg inverse kinematics for a custom 3-DoF mechanism.
- A move from iBUS-style receiver handling to custom CRSF/ExpressLRS parsing on the ESP32.
- A new Domino PCB manufacturing package based on lessons from the older SpotMicro Nitro board.
- CAD, STEP, USD, and Isaac Sim export work documenting the mechanical and simulation path.

As a portfolio project, Domino demonstrates mechanical design, embedded firmware, RC protocol work, power distribution, servo safety, calibration, and simulation constraints in one prototype hardware stack.

## Current Status

Implemented or documented:

- ESP32 DevKit firmware using PlatformIO and Arduino.
- PCA9685 output for twelve servos across four legs.
- CAD-derived inverse kinematics for hip, upper-leg, and lower-linkage angles.
- Custom CRSF/ExpressLRS receiver parser with packed-channel decoding, filtering, switch debouncing, and link-loss handling.
- Stand, stow, body tilt, ride-height preset, and experimental balance modes.
- MPU6050 IMU sampling and filtered roll/pitch state.
- Per-servo safety limits to reduce the chance of driving the mechanism into hard stops.
- STEP exports for assembly, body, and leg inspection.
- USD/USDZ exports from the Isaac Sim exploration path.
- Domino PCB V1.1B Gerber package and PCB screenshots.

Still in progress:

- Full walking gait generation.
- Dynamic balance tuning.
- A complete measured BOM.
- Wiring diagrams and annotated cable-routing photos.
- Printed-part orientation, fastener, bearing, insert, and rod-length documentation.
- A validated start-to-finish public build manual.

## Where To Start

If you are looking to build the project start here:

1. [docs/control-notes.md](docs/control-notes.md) - coordinate frames, IK constants, servo mapping, and mode flow.
2. [docs/crsf.md](docs/crsf.md) - CRSF/ExpressLRS parser and receiver migration notes.
3. [docs/cad-design.md](docs/cad-design.md) - mechanical structure, STEP exports, composite cage, and simulation limitations.
4. [docs/electronics.md](docs/electronics.md) - electronics architecture, PCB evolution, and power/control notes.
5. [hardware/pcb/domino-quadruped-pcb-v1.1b](hardware/pcb/domino-quadruped-pcb-v1.1b) - Domino PCB V1.1B Gerber package.

## Mechanical Design

Each leg has three actuated degrees of freedom:

- Hip abduction/adduction.
- Upper-leg pitch.
- Lower linkage / knee motion.

The body uses a long central cage instead of a compact square chassis. This keeps wiring, receiver placement, PCB access, and servo harness routing serviceable while giving the leg modules room to move.

The lower leg behaves more like a closed-chain/four-bar mechanism than a simple open-chain robot arm. That creates useful packaging options, but it also makes firmware and simulation more demanding: servo mapping, linkage clearances, safe joint limits, and CAD-derived IK constants all need to agree.

![Domino side linkage view](docs/images/domino-linkage-side.jpg)

![CAD side view](docs/images/cad-full-side.png)

The neutral CAD exports are included under [cad/step](cad/step). The mechanical write-up is in [docs/cad-design.md](docs/cad-design.md).

## Electronics

Domino began by reusing the electronics architecture from my SpotMicro ESP32 Nitro fork: ESP32 control, PCA9685 servo output, external servo power, receiver wiring, sensor headers, and utility connections gathered into a serviceable board layout.

The first Domino prototype reused the older PCB directly. Moving to CRSF/ExpressLRS did not require a full board redesign: the receiver needed suitable power, common ground, and its TX signal routed into the ESP32 serial input used by the firmware.

The newer Domino PCB V1.1B is a dedicated board package for this platform. It follows the same ESP32/PCA9685 quadruped-control concept, but adds extra headers, improves connector layout and silkscreen presentation, and addresses practical issues found while using the older board.

![Domino PCB V1.1B layout view](docs/images/domino-pcb-1.png)

![Domino PCB V1.1B render](docs/images/domino-pcb-2.png)

More detail is in [docs/electronics.md](docs/electronics.md).

## Firmware Architecture

The firmware is split by responsibility:

- `src/crsf.cpp` / `src/crsf.h`: CRSF frame parsing, channel unpacking, filtering, and link-health state.
- `src/ik.cpp` / `src/ik.h`: CAD-derived single-leg inverse kinematics.
- `src/leg_controller.cpp` / `src/leg_controller.h`: PCA9685 channel mapping, direction signs, trims, and servo limit enforcement.
- `src/imu.cpp` / `src/imu.h`: MPU6050 initialization, sampling, and filtered orientation state.
- `src/main.cpp`: high-level state machine, body pose model, ride height, tilt mode, balance experiment, and failsafe behavior.

High-level behavior generates body/foot pose targets, then passes them through the same IK and servo mapping path. That keeps stand, stow, tilt, balance, and future gait work tied to one low-level geometry model.

## CRSF / ExpressLRS Radio Link

One major goal of Domino was replacing simpler iBUS-style receiver handling with CRSF. CRSF is common in ExpressLRS systems and gives the robot a fast serial RC link, but it also requires correct frame synchronization, CRC validation, packed-channel decoding, filtering, and failsafe behavior.

The robot repo includes the integrated parser in `src/crsf.*` and a small example under [examples/crsf-esp32-reader](examples/crsf-esp32-reader). The reusable extraction now lives in its own repo:

- [Blacksheep909/ESP32_CRSF_Reader](https://github.com/Blacksheep909/ESP32_CRSF_Reader)

## CAD And Simulation Exports

The repo includes:

- STEP files under [cad/step](cad/step).
- USD/USDZ exports under [simulation/usd](simulation/usd).
- URDF reference export, Isaac topology report, and simulation bring-up notes under [simulation/isaac](simulation/isaac).
- CAD and simulation notes in [docs/cad-design.md](docs/cad-design.md) and [docs/simulation-notes.md](docs/simulation-notes.md).

The simulation work is exploratory. The visual model can be imported, but the real leg mechanism is closer to a closed-chain linkage than a URDF-friendly tree. A controllable simulation will need a simplified joint model or explicit constraints rather than a blind CAD import.

## Build And Bring-Up Status

The firmware can be built with PlatformIO:

```powershell
pio run
```

Upload to an ESP32 DevKit:

```powershell
pio run -t upload
```

Open the serial monitor:

```powershell
pio device monitor
```

Hardware bring-up should be done in layers: receiver first, then one servo, then one leg, then all legs on a stand. The current docs do not yet contain enough measured hardware detail to recommend building the full robot directly from the repo.

## Repository Layout

```text
.
|-- cad/
|   `-- step/                STEP exports for assembly, body, and leg modules
|-- hardware/
|   `-- pcb/                 Domino PCB Gerber package and notes
|-- simulation/
|   |-- isaac/               Isaac Sim / Isaac Lab bring-up notes and URDF topology tooling
|   |-- urdf/generated/      Generated URDF reference export
|   `-- usd/                 USD/USDZ exports for Isaac Sim experiments
|-- platformio.ini          PlatformIO build configuration
|-- src/                    Robot firmware
|-- examples/
|   `-- crsf-esp32-reader   Standalone CRSF parser demo
|-- docs/                   Design, calibration, electronics, and simulation notes
|-- include/                Project include directory
|-- lib/                    Project-local libraries
`-- test/                   Future firmware tests
```

## Related Work

This project builds on lessons from my earlier SpotMicro ESP32 fork:

- [Blacksheep909/SpotMicroESP32-Nitro-Fork](https://github.com/Blacksheep909/SpotMicroESP32-Nitro-Fork)

The earlier fork is closer to an instructional build log. Domino is a custom engineering snapshot: the firmware, CAD, PCB work, and simulation experiments are being shaped into a cleaner quadruped platform.

## Credits

Domino's mechanical direction is derived from the ESP32 quadruped robot design by [Tazer Technical](https://www.youtube.com/@TazerTechnical). Thanks to Tazer Technical for publishing the design ideas and build material that helped shape this project.
