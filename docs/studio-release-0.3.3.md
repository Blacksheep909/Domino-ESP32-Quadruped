# Domino Quadruped Studio 0.3.3

Studio 0.3.3 ships with firmware **0.8.17**. It fixes imported calibrations that never reached the robot, and bench mode misbehaving after a disconnect.

## Why a calibration could "go back to the old one"

- Importing a JSON file only loaded it into Studio as an unsaved draft. Writing it to the robot needed bench mode, and the draft was lost if Studio restarted or reconnected before the save finished. The robot then still had its old profile.

## Fixes

- **Saving a profile no longer needs bench mode** (firmware 0.8.17). The save only writes flash, and the robot turns every output off first. A bench-mode problem can no longer block saving.
- **The unsaved draft survives restarts.** An imported or edited profile is kept until you save or replace it.
- **Clear import message.** Studio now says the import is a draft and where to send it to the robot.
- **Disconnecting ends bench mode on the robot.** The companion tells the robot to leave bench mode and de-energise its outputs, instead of leaving a stale bench session behind.

## Upgrading

Flash firmware 0.8.17 from the Firmware workspace (hold BOOT while uploading). Your saved calibration is preserved. Import your JSON, review it, press **Send to robot** in Review, and confirm. Studio reads the profile back and shows the robot's source.
