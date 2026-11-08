# Domino Isaac Lab: learned walking policy ("simulation mode")

This folder trains a walking policy for Domino in NVIDIA Isaac Lab and ships it
to the ESP32. On the robot, **radio switch SB (Boxer, left 3-position) fully
down** runs the learned policy. The sticks still drive (right stick forward /
sideways, left stick turn) and the left-stick vertical (CH3) still sets ride
height, 220-280 mm.

It replaces the older `simulation/isaac` prototype, which modelled every
linkage pin as a 29-body closed loop and trained at about 56 samples/s. This
model trains at **~80,000 samples/s** on the RTX 3080 Ti with 4096 robots.

```
Fusion body_v6 (leg_v9) ──tools/build_urdf.py──> assets/domino/domino.urdf
                                                     │
config/training.json ──> domino_lab (Isaac Lab task, domain randomisation)
                                                     │  scripts/train.py (RSL-RL PPO)
                                                     ▼
                         runs/<run>/model_*.pt, metrics.jsonl, run.json
                                                     │  scripts/export_policy.py
                                                     ▼
                         policy_weights.h ──> src/policy_runner.cpp (ESP32)
```

Everything is also driven from **Studio > POLICY LAB**: start and stop
training, watch the curves, edit the domain randomisation and rewards, export,
check the ESP32 budget, install into the firmware build and upload.

## The model

The v9 leg is a parallelogram: the thigh (L2) and the push rod stay parallel,
so the shank angle is set by the lower servo through the rod. For the foot
that is exactly a serial **hip / thigh / knee** chain, which is what the sim
uses. It needs no closed-loop constraints, so thousands of robots run on the
GPU.

| Piece | Source |
|---|---|
| Hip layout 335 x 124.75 mm, leg plane 38.1 mm out, thigh pivot 21 mm down | `body_v6` joints. These match firmware `BODY_HALF_*`, `kRy`, `kRz`. |
| Thigh 179.8 mm, shank 150.5 mm, 12 mm foot ball | `leg_v9` joint origins |
| Masses: frame 0.81 kg, hip block 0.376 kg, thigh 0.052 kg, shank 0.051 kg | Fusion physical properties |
| Battery and electronics | `robot.extra_body_mass_kg` (default 0.55 kg, not in CAD). Weigh the dog and set it. |
| Servos | 40 kg·cm hips (3.9 N·m), 35 kg·cm legs (3.4 N·m), position servo with armature, 7 rad/s |

The ~80 CAD bodies are merged into 4 visual pieces per side (body, hip
block, thigh, shank), about 13 k triangles in total. Collisions are boxes plus a
foot sphere. Re-export after CAD changes:

1. In Fusion, open `Robot Dog V2 / 01 Master Assembly / body_v6` and run
   `tools/fusion_export_parts.py`. It is read-only and writes
   `assets/cad_export/parts`.
2. Run `python tools/build_urdf.py [extra_body_mass_kg]`.

`scripts/stand_check.py` holds the neutral stand and compares each simulated
foot with the firmware geometry. Today they agree to about 3 mm, and the dog
stands level at 278 mm with about 0.9 N·m on each knee.

## What the policy sees and does (the firmware contract)

The hobby servos report no position, so the actor only gets what the ESP32
has, at 50 Hz (60 floats):

| Term | Size | Notes |
|---|---|---|
| gyro | 3 x 3 history | MPU6050, body frame, x0.25 |
| gravity | 3 x 3 history | gyro + accelerometer complementary filter on the ESP32 |
| previous actions | 12 x 3 history | |
| velocity command | 3 | vx, vy (m/s), yaw rate (rad/s) |
| ride height command | 1 | (h - 0.25 m) / 0.03 m |
| gait clock | 2 | sin and cos at `gait.clock_hz` |

The critic also gets privileged state: body velocity, joint angles and
speeds, foot contacts and body height (asymmetric actor-critic).

**Actions** are 12 foot offsets (FL, FR, BL, BR, each dx, dy, dz), scaled by 40,
20 and 40 mm around the neutral stand (-15.75, 38, ride height). They are
clamped to the same box in sim and firmware. On the robot they go through the
normal `commandLeg()` path, so the CAD four-bar solver, the saved calibration,
trims and servo limits still own every pulse.

## Domain randomisation (all in `config/training.json`)

| What | Default |
|---|---|
| Ground friction / restitution | 0.3-1.3 / 0-0.2 |
| Body mass added, COM shift | -0.25 to +0.6 kg, ±30 / 20 / 10 mm |
| Leg link masses | x0.85-1.15 |
| Servo stiffness / damping / torque | x0.7-1.3 / x0.6-1.4 / x0.75-1.1 (battery sag) |
| Joint friction | 0-0.05 N·m |
| Command latency | 0-2 control steps (0-40 ms) |
| Calibration (trim) error per joint | ±2° |
| Foot fore/aft frame error | ±15 mm |
| IMU mounting tilt, gyro bias | ±3°, ±0.05 rad/s |
| PWM quantisation | 0.66° (PCA9685 12-bit at 50 Hz) |
| Pushes | ±0.4 m/s every 6-12 s |
| Terrain | flat, random rough up to 3 cm, slopes up to 0.15, with a curriculum |
| Observation noise | gyro ±0.15, gravity ±0.05 |

## ESP32-DevKitC budget

The ESP32-D0WD (WROOM-32E) has a 240 MHz LX6 core with a single-precision FPU,
520 KB SRAM (no PSRAM) and 4 MB flash (3 MB app partition). The default actor
is 60 → 128 → 64 → 32 → 12 (ELU):

* 18.5 k parameters, 74 KB as `const float` in flash (2.4 % of the app partition)
* under 2 KB of RAM: observation history plus two activation buffers
* about 18 k multiply-adds per step, estimated at ~0.46 ms of the 20 ms tick.
  The robot reports the measured time in LIVE (`policy.inferUs`).

Firmware with the placeholder uses 33 KB RAM and 389 KB flash. A boot
self-test runs the exported test vector through the network and blocks the
mode if it does not match. The C++ matches rsl_rl to under 1e-6
(`tools/test_firmware_policy.py`).

## Safety on the robot

* Entry needs: SA stand, a fresh radio link, the stand pose reached, a fresh
  IMU, a passed self-test, no IK fault and no tilt mode.
* Exit lowers any lifted foot in place, then recentres through the normal
  neutral transition.
* Faults (tilt over 55°, stale IMU, non-finite output) latch until SB is
  moved off the down position. Failsafe and E-stop behave as before.
* LIVE manual control disables the policy.

## Install (once)

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac_lab/setup-isaac-env.ps1
```

This creates `C:\isaac-projects\sim6` (python.org Python 3.12) with Isaac Sim
6.2 (pip), Isaac Lab 3.0 (from `C:\isaac-projects\IsaacLab3`, PhysX backend),
torch 2.11 cu130 and rsl_rl 5.5. Use `-EnvRoot` / `-IsaacLabRoot` for other
locations and set `DOMINO_PYTHON` to the env's `python.exe`.

Driver: Isaac Sim 6.x needs an NVIDIA 595-series or newer driver. (Isaac Sim
5.1's RTX renderer crashes at startup in `rtx.scenedb` on those drivers; 5.1
needs the 580 branch.) The code still runs on the Isaac Sim 5.1 / Isaac Lab 2.3
stack too: `domino_lab/compat.py` covers the API differences (ProxyArray data,
XYZW quaternions, launcher flags, rsl-rl 5 configs and checkpoints). Checkpoints
from either rsl-rl generation load in both, and recordings always store
quaternions as (w, x, y, z).

Isaac Sim will not start until you accept the
[NVIDIA Omniverse License Agreement](https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html).
The scripts never accept it for you. If you accept it, copy
`local-settings.example.ps1` to `local-settings.ps1` (git-ignored) and
uncomment the `OMNI_KIT_ACCEPT_EULA` line.

## Use

`policy-lab.ps1` wraps the common tasks: `studio` (default; opens Studio at
the Policy Lab page), `watch` (short visible training), `train`, `play`
(newest checkpoint), `tensorboard` and `stand`.

```powershell
# train (headless, 4096 robots); also: Studio > POLICY LAB > START TRAINING
powershell -ExecutionPolicy Bypass -File simulation/isaac_lab/run-train.ps1 -NumEnvs 4096 -Iterations 3000
# watch a checkpoint in Isaac Sim
powershell -ExecutionPolicy Bypass -File simulation/isaac_lab/run-train.ps1 -Play -Checkpoint simulation/isaac_lab/runs/<run>/model_3000.pt
# export + put into the firmware source tree
C:\isaac-projects\sim6\Scripts\python.exe simulation/isaac_lab/scripts/export_policy.py simulation/isaac_lab/runs/<run>/model_3000.pt --install
```

Runs live in `runs/` (git-ignored). Training resumes with `-Resume <model.pt>`.

## Drive it with the radio

`policy-lab.ps1 drive` (or **DRIVE IN SIM** in Studio) opens one robot in
Isaac Sim with a chase camera. Plug the Boxer in over USB and select EdgeTX
"USB Joystick" mode. It then behaves like the real dog:

* SB fully down runs the policy; otherwise the robot holds the neutral stand.
* The right stick drives forward/back and sideways, the left stick turns.
* CH3 sets ride height.

The stick mapping, deadband and speed limits are the firmware's
(`readPolicyCommand`). The HID decoding is the same as Studio's
`boxer-hid.mjs`. Without the radio, use the keyboard in the Isaac window:
W/S, A/D, Q/E, R/F for height, P toggles the policy, Backspace resets.

## Tests

```powershell
python simulation/isaac_lab/tests/test_geometry.py   # or pytest
python simulation/isaac_lab/tools/test_firmware_policy.py simulation/isaac_lab/runs/<run>/policy/policy_weights.h
```
