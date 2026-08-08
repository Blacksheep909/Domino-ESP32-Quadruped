# Domino Firmware Software-in-the-Loop

This is a desktop build of the production Domino firmware. It compiles the
same `src/main.cpp`, CRSF parser, mode logic, inverse kinematics, servo mapping,
trims, and hard angle limits used by the ESP32 build.

That is the synchronization contract: edit the production firmware, run the
SIL build again, and the virtual test uses the new code. The desktop build does
not maintain a Python or JavaScript reimplementation of the controller.

Only the hardware boundary is replaced:

- `Serial2` receives generated, CRC-valid CRSF frames.
- The MPU6050 reports a configurable simulated gravity vector.
- The PCA9685 records the exact 12 pulse outputs instead of energizing servos.
- Arduino time is deterministic, so an 11-second test runs almost instantly.

## Run the safety scenario

```powershell
.\simulation\sil\test.ps1
```

The scenario checks startup stow, CRSF acquisition, debounced stand, tilt
movement, Boxer left-stick ride-height switching, link loss, automatic failsafe
stow, finite servo angles, and valid PCA9685 pulse widths.

## Open the live monitor

```powershell
.\simulation\sil\launch.ps1
```

Then open `http://127.0.0.1:8765`. The linkage command view and all twelve
servo outputs update from the running firmware.

Running `launch.ps1` again stops the previous SIL processes, rebuilds the
current firmware sources, and relaunches the monitor. Stop it manually with:

```powershell
.\simulation\sil\stop.ps1
```

## Current boundary

This first stage validates firmware decisions and commanded motion. It is not
yet a contact-physics or motor-dynamics model. The next stage connects these
same pulse outputs to the CAD-derived Isaac articulation, then adds Boxer input
through USB joystick or raw CRSF over USB-UART. Hardware-in-the-loop should be
done with servo power disconnected until the virtual run passes.

## Planned input path

1. USB game-controller input for the Boxer configured as a joystick.
2. Raw CRSF input from a bound receiver through a 3.3 V USB-UART adapter.
3. ESP32 hardware-in-the-loop with PCA9685 output intercepted and servo power
   disconnected.
4. Physical bring-up only after the same scenario passes in SIL and Isaac.
