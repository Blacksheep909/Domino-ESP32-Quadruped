# Domino Standalone 3D Sandbox

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

The CAD datum is aligned once against the validated neutral contact pose. It is
not snapped to the floor every frame.

## Launch

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
