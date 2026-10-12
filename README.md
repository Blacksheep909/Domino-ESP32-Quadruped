<p align="center">
  <img src="simulation/standalone/desktop/assets/domino-studio-icon-smooth.png" alt="Domino Quadruped Studio icon" width="112">
</p>

<h1 align="center">Domino Quadruped</h1>

<p align="center">
  <strong>A carbon-frame ESP32 robot dog, and the desktop studio that drives it.</strong><br>
  Custom 3-DoF legs, ExpressLRS radio control, CAD-accurate simulation, and a calibration workflow that tells you the truth about what the robot is running.
</p>

<p align="center">
  <a href="https://github.com/Blacksheep909/Domino-ESP32-Quadruped/releases/latest"><strong>Download for Windows</strong></a> ·
  <a href="#getting-started">Getting started</a> ·
  <a href="#features">Features</a> ·
  <a href="docs/calibration-guide.md">Calibration</a>
</p>

<p align="center">
  <img src="docs/images/studio-simulation-standing.jpg" alt="Domino standing on four feet in Quadruped Studio's Simulation workspace" width="900">
</p>

Domino is a prototype quadruped built around a carbon-and-printed chassis, a scratch-designed three-joint leg, a PCA9685 servo driver and an ESP32 running PlatformIO firmware. **Quadruped Studio** is the Windows desktop app that ships with it: it runs the real firmware kinematics against the CAD model in simulation, then connects to the physical robot for calibration, gait tuning, diagnostics and recording.

Domino is an active engineering prototype, not a finished kit. The firmware, CAD, PCB package and notes are published as a project record; see [build status](docs/project-overview.md#build-and-bring-up-status) for what is and is not validated.

## Features

- **Simulation.** Stand, stow, tilt, ride height and gait modes on the actual CAD and closed-linkage geometry, driven from a keyboard, gamepad or CRSF transmitter.
- **Calibration you can trust.** A five-step wizard for neutral offsets, directions, travel limits and PCA9685 channel routing. The robot reports where its profile came from, and Studio warns loudly if it is running defaults.
- **Safe by design.** One-servo limited jog with a bench-mode handshake, per-servo hard limits, power-fault guards and an explicit safety dock.
- **LIVE workspace.** Digital-twin comparison, telemetry graphs, gait profiles, diagnostics, and saved or exportable sessions.
- **Mission tools.** Local route planning, map overlays, GPS and LiDAR views, behind capability gates.
- **Firmware in the loop.** Build and upload the ESP32 firmware from inside Studio, with a software-in-the-loop executable that runs the production code before any hardware does.

## Screenshots

<table>
  <tr>
    <td width="50%"><img src="docs/images/virtual-lab-simulation-workspace.png" alt="Simulation workspace with the CAD quadruped on terrain" width="100%"><br><sub><b>Simulation.</b> The production firmware kinematics on the real CAD linkage.</sub></td>
    <td width="50%"><img src="docs/images/virtual-lab-real-robot-workspace.png" alt="LIVE workspace comparing the digital twin with the physical robot" width="100%"><br><sub><b>LIVE.</b> Digital twin and physical-robot controls side by side.</sub></td>
  </tr>
  <tr>
    <td width="50%"><img src="docs/images/virtual-lab-live-calibration.png" alt="Five-step servo calibration wizard" width="100%"><br><sub><b>Calibration.</b> Offsets, directions, limits and channel routing, previewed on the CAD model.</sub></td>
    <td width="50%"><img src="docs/images/virtual-lab-live-gaits.png" alt="Gait profile editor" width="100%"><br><sub><b>Gaits.</b> Tune, preview and persist gait profiles on the robot.</sub></td>
  </tr>
</table>

<p><em>Screenshots show simulation and demo data; none depict a live physical connection.</em></p>

## Getting started

1. Download the latest **Domino Quadruped Studio** installer (or the portable zip) from the [releases page](https://github.com/Blacksheep909/Domino-ESP32-Quadruped/releases/latest).
2. Run it. Simulation works offline with no robot attached.
3. To drive hardware, follow the [LIVE hardware bring-up procedure](docs/live-hardware-bring-up.md) and the [calibration guide](docs/calibration-guide.md) before enabling any motion.

Studio targets Windows 10 and 11. See the [Studio guide](docs/quadruped-studio.md) for build availability and controls.

## Build from source

You need Node.js 20 or newer for Studio, and [PlatformIO](https://platformio.org/) for the firmware.

```powershell
# Firmware: build the ESP32 image
pio run -e esp32dev

# Studio: install, test, and run from source
cd simulation/standalone
npm install
npm test
npm start            # browser/source version at http://127.0.0.1:8770

# Firmware software-in-the-loop checks
./simulation/sil/test.ps1
./simulation/sil/test-calibration-storage.ps1
```

## Calibration and persistence

The calibration profile (offsets, directions, limits, channel map) is stored in the ESP32's NVS flash as a versioned, checksummed record with an automatic backup copy. Normal firmware uploads preserve it. On every connection Studio reads the active profile and tells you its source, so a robot that has lost its profile is flagged immediately instead of silently running defaults. Export a JSON backup before any full-chip erase. Details: [calibration guide](docs/calibration-guide.md).

## Documentation

| | |
| --- | --- |
| [Project overview](docs/project-overview.md) | Full tour: tracks, mechanics, electronics, firmware architecture, repository layout and credits |
| [Engineering overview](docs/engineering-overview.md) | Architecture, data flow, code tour and current boundaries |
| [Build guide](docs/build-guide.md) | Build sequence, functional BOM, wiring architecture and commissioning gates |
| [Calibration guide](docs/calibration-guide.md) | Servo centering, the LIVE wizard and how the profile is stored |
| [Quadruped Studio](docs/quadruped-studio.md) | Desktop app guide |
| [Virtual Lab](docs/virtual-lab.md) | Simulation, LIVE workspace, controls and limitations |
| [LIVE bring-up](docs/live-hardware-bring-up.md) | Safe procedure before physical calibration or motion |
| [Control notes](docs/control-notes.md) | Coordinate frames, IK constants, servo mapping and mode flow |
| [CRSF / ExpressLRS](docs/crsf.md) | Receiver parser and migration notes |
| [Electronics](docs/electronics.md) | Power, PCB evolution and control notes |
| [Troubleshooting](docs/troubleshooting.md) | Common faults and fixes |

## Related work

Domino is the successor to my earlier [SpotMicro ESP32 Nitro fork](https://github.com/Blacksheep909/SpotMicroESP32-Nitro-Fork). See the [project overview](docs/project-overview.md#credits) for credits.
