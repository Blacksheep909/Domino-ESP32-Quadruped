# Domino Virtual Lab

The Virtual Lab is Domino's standalone, computer-game-like 3D environment for
testing the real firmware controller against the real CAD before connecting
the physical robot. It is a separate project track from the ESP32 firmware
and from the Isaac Sim / Isaac Lab training work.

The program now has two top-level workspaces:

- **Simulation** owns the firmware-in-the-loop 3D environment, Boxer heartbeat,
  physics, joint inspection, and experimental gait tools.
- **Live** is an isolated, safely disarmed real-robot digital twin. It shares
  the 3D CAD viewport for visual comparison while keeping physical telemetry,
  data recording, calibration, gait, and diagnostics state separate from the
  simulator.

Changing to **Live** immediately revokes simulation command ownership.
The real workspace does not reuse simulated connection or telemetry state.
Simulation and physics updates freeze, leaving the CAD as a reference until
independent expected-command and measured-robot streams are available. The
comparison panel is structured to show body-pose deltas, all 12 joint errors,
power, timing, and synchronized expected/measured graphs.

LIVE telemetry uses a canonical `live-telemetry` envelope relayed by the local
WebSocket bridge. Expected and measured poses carry independent timestamps,
16-channel servo arrays, and body roll/pitch/yaw/height. The browser validates
packet order and freshness, computes the 12 driven-joint errors, and renders a
translucent measured CAD overlay only while physical data is fresh. Power is
carried in the same envelope but remains an independent freshness-gated signal.

`CONNECT` opens the physical-link manager. Companion adapters announce a stable
identity, Wi-Fi/Bluetooth/USB transport, robot identity, firmware, signal, and
capabilities. The browser then requests a read-only handshake and receives a
short-lived engineering session. All telemetry and hardware command envelopes
must carry the selected `adapterId` and `sessionId`; the relay rejects stale or
duplicate adapter identities, mismatched sessions, and unsolicited command
acknowledgements. Losing the adapter heartbeat immediately locks calibration and
gait operations without claiming that the physical robot is still connected.

The runnable PC adapter is `live-companion-adapter.mjs`, with the PowerShell
wrapper `start-live-companion.ps1`. It connects the localhost relay to either a
robot Wi-Fi TCP endpoint, wired USB serial, or paired Bluetooth SPP serial port.
It advertises the robot only while that physical link is fresh, reconnects both
sides independently, and forwards telemetry, safety, manual-control,
calibration, and gait traffic through the same session-bound contracts used by
the UI. See [the LIVE companion protocol](../../docs/live-companion-protocol.md)
for launch commands and the required ESP32 JSONL endpoint.

The safety dock adds a separate `live-safety-command` contract for arm, disarm,
E-stop, and physical-latch reset. Arm requires a 1.5-second uninterrupted hold,
fresh expected/measured telemetry, a robot-reported Boxer/ELRS drive link, and a
disarmed robot state. While armed, a session-bound 10 Hz heartbeat drives a
400 ms robot-side watchdog. The UI fails closed on missing acknowledgements,
workspace loss, hidden tabs, stale telemetry, bridge loss, or adapter loss; the
companion adapter must enforce the watchdog and output shutdown independently.

The LIVE comparison scope can record synchronized samples in memory, graph body
pitch, roll, yaw, or height, survive a link interruption without filling the gap
with stale values, and export a stopped session as CSV. Exports include capture
and source timestamps, alignment, expected/measured/error body values, voltage,
current, power, worst joint error, and every driven servo-channel error.

The LIVE Calibration tab guides bench safety, all 12 joint selections, neutral
offset/direction, mechanical limits, review, browser persistence, and JSON
backup. A separate calibration command/acknowledgement envelope keeps physical
jog and robot persistence locked until an adapter explicitly confirms bench
mode and the 10 degree / 5 degree-per-second safety contract.

The optional `diagnostics` object on a `live-telemetry` packet may report
`controllerHz`, `commandHz`, `transmitHz`, `acknowledgementHz`, `esp32LoopHz`,
`commandLatencyMs`, `uptimeMs`, `robotState`, `gaitTargetValid`, `ikValid`,
`jointLimitClips`, and `servoOutputChannels`. LIVE Diagnostics combines those
fields with browser-observed packet order and freshness to find the first broken
stage, retain a severity-tagged event log, and export a JSON diagnostic bundle.

The optional `controller` object carries the robot-side CRSF evidence used for
the LIVE drive badge and arm interlock: exactly 16 bounded channel values, frame
timestamp/rate/loss, failsafe state/count, link quality, dual RSSI, SNR, RF mode,
transmitter power, active antenna, receiver voltage, and Boxer/receiver names.
Simple diagnostics shows the operating essentials and eight mapped controls;
Expert shows all RF fields, all 16 channels, and controller transition history.
The link is arm-ready only while a fresh `boxer-elrs` frame is failsafe-clear,
link quality is at least 50%, and RSSI is at least -105 dBm.

`MANUAL CONTROL` adds a separate, guarded browser-driving path for compatible
adapters. It remains inspectable offline, but requesting authority requires an
armed robot, fresh expected/measured telemetry, a healthy Boxer/ELRS link, and
an explicit physical-E-stop confirmation. A robot-granted authority token lasts
at most 30 seconds. Bounded controls stream at 20 Hz only while HOLD TO DRIVE is
pressed, release sends a neutral stand frame, and every frame declares a 250 ms
robot-side expiry. Leaving LIVE, hiding the tab, stale telemetry, controller
failsafe, disarm, bridge loss, or adapter loss revokes browser control. The
adapter must independently enforce the token, armed state, deadman, bounds,
timeout, neutralization, and physical E-stop.

The LIVE Gaits tab reads and writes the same browser profile library used by the
Simulation gait lab. It supports versioned JSON import/export, a moving local 3D
preview, reachability and risk checks, and a parameter diff against the robot's
reported active profile. Applying a draft is only enabled when telemetry reports
the robot disarmed and advertises persistent gait-profile support. The adapter
must acknowledge validation and storage through the `live-gait-command` /
`live-gait-ack` relay contract; the prior robot profile can then be restored with
an explicit rollback command.

The Simulation link panel reports each part of the control path separately:

- **Local bridge** is driven by a browser-to-server heartbeat, including
  acknowledgement age and round-trip latency;
- **Boxer input** becomes connected only when fresh RadioMaster HID/gamepad
  packets are actually arriving;
- **CRSF to firmware** reflects accepted frames and the SIL firmware's own
  link-alive state.

These indicators deliberately avoid treating a healthy local server or
keyboard fallback as proof that a Boxer is connected.

![Simulation workspace](../../docs/images/virtual-lab-simulation-workspace.png)

![Real Robot workspace](../../docs/images/virtual-lab-real-robot-workspace.png)

![LIVE gait profile transfer](../../docs/images/virtual-lab-live-gaits.png)

![LIVE physical connection manager](../../docs/images/virtual-lab-live-connection-manager.png)

![LIVE latched E-stop](../../docs/images/virtual-lab-live-safety.png)

![LIVE Boxer and ELRS controller diagnostics](../../docs/images/virtual-lab-live-controller-diagnostics.png)

## Firmware deployment workspace

Open **UPLOAD** in the Real Robot header to review, build, and deploy the
real root PlatformIO project. The workspace lists every included file from
`platformio.ini`, `src/`, `include/`, and `lib/`, shows a package SHA-256, and
provides a read-only source viewer before hardware operations are enabled.

The deployment sequence is intentionally gated:

1. Run **BUILD & VALIDATE**. Upload remains disabled unless the current package
   hash has compiled successfully for the `esp32dev` environment.
2. Connect the ESP32 and select its serial port, or leave **AUTO DETECT** when
   only the target USB serial device is connected.
3. Safely support Domino, hold the ESP32 **BOOT** button when required, and
   confirm the hardware checkbox.
4. Select **UPLOAD TO ESP32**. PlatformIO output, flash progress, verification,
   errors, and the final exit state are retained in the deployment log.

Firmware jobs run in the local simulator server, so closing the deployment
window does not terminate an active build. Only repository-scoped firmware
files can be reviewed, only the fixed `esp32dev` PlatformIO target can run, and
the upload API independently enforces both a matching successful build and the
explicit BOOT confirmation.

This local application runs separately from Isaac Sim and Isaac Lab. It combines:

- the production ESP32 firmware compiled as a native process;
- CRSF frames generated from keyboard or direct RadioMaster Boxer USB input;
- all 29 real Domino CAD STL meshes;
- a real-time closed-linkage kinematic solver;
- Rapier 3D rigid-body dynamics at 120 Hz;
- gravity, floor and obstacle collision, friction, and active foot contacts;
- a full 3D plane, orbit camera, shadows, obstacles, and sandbox movement.

The firmware owns mode selection, CRSF filtering, failsafe behavior, IK, trims,
servo limits, and the final 12 PCA9685 outputs. The renderer consumes those
outputs; it does not copy the firmware controller.

The rendered geometry is Domino's actual CAD. Physics currently runs on a
hidden, simplified 12-joint Domino articulation derived from the validated
Isaac proxy. This is necessary because the generated CAD URDF contains
closed-loop and duplicate-child topology that cannot be used directly as a
single tree articulation. The proxy includes the four shoulder actuators and
the eight linkage actuators, realistic mass distribution, servo stiffness,
joint limits, and high-friction 24 mm TPU foot spheres.

The mass proxy currently totals 2.966 kg. It includes two
[CNHL Black Series 1500 mAh 4S packs](https://chinahobbyline.com/collections/uk-warehouse/products/cnhl-black-series-1500mah-14-8v-4s-100c-lipo-battery-with-xt60-plug-4-packs)
at 183 g each, positioned independently in the front and rear electronics bays
using the manufacturer's 75 x 37 x 35 mm envelope.
Keeping the packs as separate colliders preserves their contribution to body
pitch inertia instead of folding their weight into one central chassis mass.

The CAD datum is aligned once against the validated neutral contact pose. It is
not snapped to the floor every frame.

## Launch

This repository contains the application source. It is not a hosted website.
The current launcher targets Windows because the firmware SIL build uses the
PlatformIO MinGW toolchain and PowerShell process management.

Install dependencies after cloning:

```powershell
cd simulation\standalone
pnpm install
cd ..\..
pio pkg install --global --tool platformio/toolchain-gccmingw32
```

Then launch the local application:

```powershell
.\simulation\standalone\launch.ps1
```

Open `http://127.0.0.1:8770`.

For a hands-off renderer and linkage check, open
`http://127.0.0.1:8770/?demo=1`.

## Input

Connect the RadioMaster Boxer in EdgeTX USB joystick mode before or after
launching the sandbox. The local server reads its HID report directly, so it
also works in embedded browsers that do not expose the Gamepad API.

The first eight EdgeTX output channels are passed directly into the firmware
bridge. The current robot mapping is:

- channel 1: roll;
- channel 2: forward command;
- channel 3 / Boxer left-stick vertical: continuous ride height from 220 to 280 mm (1000 µs = low, 2000 µs = high);
- channel 4: turn;
- channel 5 / SA: stand mode;
- channel 6 / SB: unbound and ignored by the current firmware;
- channel 7 / SC: fully down enables the sinusoidal gait;
- channel 8 / SD: tilt mode and a hard gait interlock.

Keyboard fallback uses `W/S`, `A/D`, `Q/E`, `Space`, `T`, `G`, and `R`.

The first gait is a deliberately slow diagonal sinusoidal trot. Right-stick
vertical commands forward/reverse travel and right-stick horizontal commands
turning. Gait cannot run while SD/tilt is active. The renderer and physics proxy
consume the same 12 firmware-authored servo outputs.

### Website gait lab

`TUNE` opens a simulator-only gait lab for evaluating candidate walking values
without rebuilding or changing the ESP32 firmware. Its Stable, Balanced, and
Fast presets expose cadence, stride, foot clearance, ground-contact duty factor,
gait height, stance width, turn gain, command response, swing shape, and diagonal
phase. The footer reports estimated commanded speed, the current support-foot
count, and whether all four CAD foot targets are reachable.

When `OVERRIDE` is enabled, the browser replaces the displayed gait trajectory
only while the firmware is in CAREFUL or TROT mode. Radio mode selection,
interlocks, stand/sit, CRSF handling, and the firmware process continue to run
normally. The override feeds both the real CAD linkage renderer and Rapier, but
it is never sent back to the SIL process or written to `src/main.cpp`. Settings
are stored in the browser so a useful candidate can be compared across runs
before it is deliberately ported to the physical robot firmware.

Each launch records Boxer HID axes, mapped channels, outgoing controls, firmware
mode, target pose, all servo outputs, body height, reset count, and per-foot
contact state. The active recording is available at
`http://127.0.0.1:8770/runtime/debug/latest.jsonl`.

## Current Boundary

This is a firmware-in-the-loop safety sandbox, not a replacement for the Isaac
Lab training environment. The next physics milestone is a validated
closed-linkage collision model whose passive pin constraints match the
four-bar CAD exactly. Until that is complete, use this application to test
radio mapping, firmware modes, commanded poses, travel limits, gross contact
behavior, and obvious unsafe transitions before powering the physical robot.

## Stop

```powershell
.\simulation\standalone\stop.ps1
```

This environment is under active local development and is not ready for public
repository publication.
