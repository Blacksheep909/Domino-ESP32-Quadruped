# Domino Quadruped Studio 0.3.9

Studio 0.3.9 ships with firmware **0.8.17**. It is the same code as 0.3.3, renumbered so it is newer than locally built 0.3.4-0.3.8 installs and updates correctly.

It contains everything from [0.3.2](studio-release-0.3.2.md) and [0.3.3](studio-release-0.3.3.md):

- The robot reports where its calibration profile came from, and Studio warns when it is running defaults.
- Calibration is stored as a versioned, checksummed record with an automatic backup.
- Saving a profile no longer needs bench mode, and an unsaved imported draft survives Studio restarts.
- Disconnecting ends bench mode on the robot.

Upgrade by flashing firmware 0.8.17 from the Firmware workspace (hold BOOT while uploading). Your saved calibration is preserved.
