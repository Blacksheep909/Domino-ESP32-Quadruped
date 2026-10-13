# Domino Quadruped Studio 0.3.10

Studio 0.3.10 ships with firmware **0.8.18**. It restores the 0.3.8 line, the newest Studio and firmware work, and publishes it.

## Why this release exists

Studio 0.3.2, 0.3.3 and 0.3.9 were built from an older code line by mistake. They are missing the newer interface work (help, notifications, connection feedback) and the calibration audit fixes, and their firmware (0.8.16 / 0.8.17) is older than 0.8.18. **Do not use them. Do not flash firmware 0.8.16 or 0.8.17 over 0.8.18.** This release replaces them.

## What is in it

Everything from 0.3.8, including the [calibration audit](calibration-audit-2026-10-10.md):

- Fixes for 15 calibration and bench-mode defects: session reset errors, bench exit while a request was pending, bench permission released when the workspace is hidden or support is withdrawn, one consistent policy for request/jog/save, import validation, jog limits, arm/bench exclusion, upload blocked during a save, and a relay window long enough for the save acknowledgement.
- An imported calibration stays a visible draft until the robot's readback confirms it.
- Firmware 0.8.18 disables all channels on bench entry and drives only the selected output after an explicit jog.
- The newer interface: help, notifications, connection feedback and camera controls.

## Upgrading

Install over any previous version. Firmware 0.8.18 is bundled; if your robot already runs 0.8.18 there is nothing to flash, and its saved calibration is untouched.

## Known issue

The firmware software-in-the-loop gait scenario (`simulation/sil/test.ps1`) reports that gait mode is never reached and the swing/support checks fail on the 0.3.8-line firmware. It also fails with the older prebuilt test executable from that line, so it predates this release. The installer build only compiles the SIL executable and does not run that scenario. Investigate before relying on gait changes.
