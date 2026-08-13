# Domino Virtual Lab

The Domino Virtual Lab is a local engineering application for developing the
quadruped's firmware, mechanism, controls, and future physical-robot tooling.
It is source code that runs from this repository. It is not a hosted GitHub
Pages site and does not require an internet connection after dependencies are
installed.

## Workspaces

### Simulation

![Domino Virtual Lab simulation workspace](images/virtual-lab-simulation-workspace.png)

The Simulation workspace combines the production control firmware compiled as
a native process with the actual Domino CAD, a closed-linkage solver, Rapier 3D
physics, the RadioMaster Boxer, keyboard/gamepad fallback, gait tuning, joint
inspection, and session recording.

Simulation is the place for unrestricted gait experimentation. Its connection
indicators refer only to the local firmware bridge, controller, and physics
runtime; they do not claim that the physical dog is connected.

The link panel keeps those signals independent. The browser/server heartbeat
shows bridge health, acknowledgement age, and round-trip time. Boxer status is
based only on fresh RadioMaster HID/gamepad packets, while CRSF status and its
accepted-frame count come from the firmware-in-the-loop controller. Keyboard
or demo input therefore cannot make the Boxer indicator appear connected.

### Live

![Domino Real Robot engineering workspace](images/virtual-lab-real-robot-workspace.png)

The Live real-robot workspace is deliberately separate from Simulation. It opens
disarmed and cannot own simulation controls, firmware state, or physics state.
It keeps the same CAD viewport as a digital-twin surface: commanded geometry
will be shown as the expected pose, physical joint/IMU telemetry as the measured
pose, and the difference as joint, body-pose, timing, and graph errors. Until a
physical engineering link exists, measured values remain unavailable rather
than borrowing simulated data.

The current comparison shell includes independent drive and engineering link
health, expected/measured stream state, body-pose deltas, all 12 joint errors,
power measurements, and a synchronized comparison scope. The physical telemetry
contract now accepts independently timestamped expected and measured poses,
computes shortest-path angular errors, and rejects malformed, out-of-order, or
stale packets. Fresh measured telemetry drives a separate translucent CAD model
over the expected pose; the overlay disappears if the stream is more than one
second old. The local WebSocket relay is ready for the future wireless robot
adapter, while the hardware-specific transport remains to be implemented.

![LIVE synchronized recording using local relay verification data](images/virtual-lab-live-recording.png)

The comparison scope records each synchronized source pair once and graphs body
pitch, roll, yaw, or height as expected, measured, and error series. A telemetry
interruption creates a visible gap without ending the session or reusing stale
values. Stopped sessions export analysis-ready CSV containing timestamps, time
alignment, body pose, power, and all 12 driven-joint errors. The screenshot
above uses local relay verification data, not a connected physical robot.

The Live navigation now has three working views. Compare keeps the digital twin,
pose deltas, power readings, and compact scope together. Data provides a larger
signal graph, recorder controls, live engineering metrics, and a newest-first
table of synchronized samples. Sessions keeps completed recordings for the
current app run, summarizes duration, sample count, peak joint error, and average
power, and gives every session independent CSV export and delete actions.
Recording state is shared across all three views, so moving between them cannot
interrupt or split a capture. Calibration, Gaits, and Diagnostics remain visibly
disabled until their hardware-backed workflows are implemented.

Simple mode will present the normal operating workflow. Expert mode will expose
the detailed signals and settings needed for mechanism, gait, power, and
control optimization. Safety limits remain active in both modes.

## Current architecture

The local application runs three cooperating pieces:

1. The browser renders the UI, CAD and physics environment.
2. The local Node server owns HID input, command arbitration, session logs,
   firmware build/upload operations, and browser communication.
3. The SIL executable runs the same C++ controller used by the ESP32 and
   publishes its commanded poses, joint angles, PWM outputs, and state.

The physical implementation will keep two independent wireless paths:

- Boxer/ExpressLRS/CRSF for driving, essential telemetry, and an operator stop;
- ESP32 Wi-Fi for high-rate engineering telemetry, calibration, profiles,
  logs, graphs, and diagnostics.

## Run locally

The current launcher targets Windows. Install Node.js, pnpm, Python/PlatformIO,
and the PlatformIO MinGW toolchain, then run:

```powershell
cd simulation\standalone
pnpm install
cd ..\..
pio pkg install --global --tool platformio/toolchain-gccmingw32
.\simulation\standalone\launch.ps1
```

Open `http://127.0.0.1:8770`. Stop both local processes with:

```powershell
.\simulation\standalone\stop.ps1
```

## Verification

Run the standalone unit, linkage, gait, heartbeat, control-state,
firmware-package and physics tests with:

```powershell
cd simulation\standalone
pnpm test
```

The generated `dist/`, `node_modules/`, and `runtime/` directories are ignored.
Only the source, lockfile, launch scripts, tests, documentation, and required
robot assets belong in Git.

## Documentation media

Repository screenshots live under `docs/images/`. Capture screenshots after
meaningful interface milestones. Short GIFs should demonstrate one focused
interaction - such as switching workspaces, opening the gait lab, inspecting a
joint, or recording a session - and remain short enough to be practical in the
repository.
