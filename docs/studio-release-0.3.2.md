# Domino Quadruped Studio 0.3.2

Studio 0.3.2 ships with firmware **0.8.16**. It fixes the robot losing its calibration profile without any sign of it.

## Calibration

- **The robot says where its profile came from.** Firmware reports whether the active profile is a saved record (`nvs`), the previous backup (`nvs-backup`), a profile from firmware 0.8.15 or earlier (`nvs-legacy`), or compiled defaults (`default-empty`, `default-fallback`), plus the reason it fell back.
- **A banner warns when the robot is on defaults.** The Calibration page shows a red banner, and the status text explains what to do.
- **Saved profiles survive firmware updates.** The profile is now a versioned, checksummed record with an automatic backup, instead of a raw struct dump that any code change could invalidate. A profile saved by firmware 0.8.15 or earlier is still read.
- **Studio no longer overwrites your saved profile with defaults.** If the robot loses its profile, your last known profile loads as an unsaved draft; review it and save it back to the robot.
- **An unreadable browser profile is kept** instead of being deleted.

## Other

- The SIL build script no longer fails on compiler warnings under Windows PowerShell.
- New host tests for the calibration storage code, and new Studio tests for the restore and origin logic.

## Upgrading

Flash firmware 0.8.16 from the Firmware workspace. Your existing calibration is preserved. Save it once from the Calibration page to upgrade it to the new format and create the backup copy. Export a JSON backup first if you plan a full flash erase.
