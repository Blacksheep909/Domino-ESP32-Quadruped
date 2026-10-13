# LIVE Hardware Bring-Up

This procedure validates the physical side of the Virtual Lab without treating
a successful firmware build as proof of safe robot operation. Stop at the first
failed gate. Keep the chassis supported, use current-limited power, and retain a
physical means of isolating servo power.

## Before powering servos

- Confirm ESP32, receiver, PCA9685, and servo supply share the intended ground
  reference. The current physical build has no IMU and does not poll one.
- Verify servo-rail polarity, fuse/current limiting, regulator rating, and the
  voltage required by the installed servos.
- Disconnect or disable the high-current servo rail for the first USB session.
- Keep the Boxer SA switch low and verify the receiver is bound.
- Build the current firmware with Wi-Fi, Bluetooth, and INA226 disabled initially.

```powershell
pio run
pio run -t upload
pio device monitor
```

Expected evidence: firmware boots, every PCA9685 output remains off, CRSF frames
and link statistics become fresh, and no servo moves.

## Gate 1: read-only USB PC link

Open Quadruped Studio (it bundles the physical companion), select **LIVE**, then **PAIR ROBOT**,
and choose the USB adapter. Verify the robot identity and establish the read-only session. Do not arm.

Expected evidence:

- Firmware version and capabilities match the endpoint.
- Robot state is `DISARMED`.
- Boxer/ELRS channel, rate, RSSI, link-quality, SNR, RF-mode, power, antenna,
  and failsafe fields update.
- Expected pose is present; measured joint pose remains unavailable without
  encoders. IMU attitude is unavailable on the current build.
- Disconnecting the companion immediately returns LIVE to a locked state.

## Gate 2: one-servo calibration

Support the chassis with every foot unloaded. Apply conservative current limits,
connect only one known servo if practical, and open **LIVE → CALIBRATION**.

1. Complete the safety acknowledgement and request bench mode.
2. Open **Edit channel map** and verify the physical PCA9685 output for the
   selected logical joint.
3. Use only the limited ±10 degree jog and confirm no other output energizes.
4. Set neutral, direction, and deliberately conservative mechanical limits.
5. Exit bench mode and confirm all outputs turn off.
6. Save, power-cycle, reconnect read-only, and request the stored profile.

Expected evidence: the intended servo moves at no more than 5 degrees/second,
no other channel receives PWM, the profile survives reset, and a wrong or
duplicate channel map is rejected.

Repeat one joint at a time only after the previous joint passes.

## Gate 2A: motion smoothing and stand response

With the robot disarmed and every output off, open **LIVE → CALIBRATION →
Review and save**. Read the active motion smoothing settings, adjust the stand
height and tilt axis rates or center deadband, save, then read them back. A save
must be rejected while armed or in bench mode. Lower rates reduce sudden
command changes but increase the time to reach a requested pose.

The **Motion smoothing** switch can bypass both ramps and the tilt center
deadband. Save and read back the switch while disarmed before comparing ON and
OFF with small commands on a supported chassis. Mechanical servo limits, mode
guards, and the sit transition remain active when smoothing is OFF.

Support the chassis and test small stand-height and tilt commands. Check that
the 12 channels move in the expected directions without reaching mechanical
stops. If a servo chatters while the commanded pose is steady, inspect power,
grounds, servo condition, and load before slowing the software further.

## Gate 3: watchdog and guarded motion

With Domino still supported and all calibrated limits reviewed:

1. Start disarmed with SA low and a healthy Boxer link.
2. Hold the browser arm control for the full acknowledgement period.
3. Verify outputs enable only after the robot acknowledgement.
4. Release/interrupt the browser heartbeat and confirm outputs disable within
   the 400 ms watchdog contract.
5. Reconnect, arm, request browser manual authority, and confirm the grant alone
   causes no movement.
6. Hold the deadman with zero axes, then test small stand/height/roll commands.
7. Release the deadman and confirm neutral stand within 250 ms.
8. Test careful crawl at low profile values with feet unloaded, then with the
   feet lightly loaded. Verify each lifted foot, planted foot, and weight
   transfer before trying an unsupported floor crawl.
9. Try trot only after a repeatable slow crawl, stable supply voltage, and a
   verified stop response. Without an IMU, there is no automatic balance
   recovery from a stumble.

Do not proceed if an E-stop, disconnect, stale Boxer link, expired lease, or
missing manual frame fails to neutralize or disable motion as documented.

## Gate 4: gait persistence and rollback

Disarm and ensure every output is off. Apply a clearly named conservative gait
profile from **LIVE → GAITS**, request it back, then power-cycle and request it
again. Apply a second distinguishable profile and use **Revert**.

Expected evidence: only disarmed writes succeed, the profile survives reset,
telemetry reports the active values, and revert restores the previous verified
slot after a second power cycle.

## Gate 5: wireless transports

Copy `src/live_robot_secrets.example.h` to the ignored local
`src/live_robot_secrets.h`. Use a unique WPA2 credential, non-default Bluetooth
PIN, and random 16+ character LIVE link key. Build and test one transport at a
time; retain USB as recovery.

- Wi-Fi must join the configured station network and must not create an access
  point.
- Bluetooth must be paired before its SPP COM port is given to the companion.
- A wrong or missing link key must never gain command ownership.
- Loss of the owning wireless connection while armed must enter watchdog; loss
  during calibration must turn every output off.

Never publish the local secrets header, serial port identifier, network address,
or screenshots containing credentials.

## Gate 6: measured power

This gate requires a physically installed INA226 and an appropriately rated
external shunt. Do not enable the monitor based only on a generic breakout-board
label.

1. Measure the shunt resistance and protected path rating.
2. Set the `DOMINO_POWER_*` values documented in `platformio.ini`.
3. Compare bus voltage against a DMM with servo power initially unloaded.
4. Compare signed current against an independent current instrument at several
   known loads.
5. Confirm shunt and connector temperature remain acceptable.
6. Record a LIVE session and verify voltage, current, power, timing, and CSV
   export while changing a bounded robot load.

If the monitor is missing, stale, or unreadable, LIVE must show power as
unavailable rather than zero.

## Evidence to retain

- Firmware commit and build output.
- Sanitized connection/capability screenshot.
- Calibration JSON backup and physical channel map.
- Saved smoothing settings and readback after a power cycle.
- Watchdog/deadman timing observations.
- Gait apply/reboot/revert results.
- DMM/current-instrument comparison table and shunt details.
- Sanitized real-hardware LIVE recording CSV and a short repository GIF.

Record failures as well as passes. A failed gate is useful engineering evidence
and should be fixed before moving to the next stage.
