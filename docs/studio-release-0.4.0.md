# Domino Quadruped Studio 0.4.0

Studio 0.4.0 ships with firmware **0.8.19**. It is a fresh start: the 0.3.10 line (the 0.3.8 calibration audit fixes, help, notifications and connection feedback, firmware 0.8.18) plus two fixes for gaps that line still had.

## New in 0.4.0

- **The robot reports where its calibration came from.** Firmware says whether the active profile is a saved record (`nvs`), the previous backup (`nvs-backup`), a profile from firmware 0.8.18 or earlier (`nvs-legacy`), or compiled defaults (`default-empty`, `default-fallback`) with the reason it fell back. A robot silently running defaults can no longer go unnoticed.
- **A banner warns when the robot is on defaults.** The Calibration page shows it, and the boot log prints `Calibration source: ...`.
- **Calibration is stored as a versioned, checksummed record with an automatic backup**, instead of a raw struct dump that a code change could invalidate. Profiles saved by 0.8.18 and earlier are still read and migrated on the next save.
- **Studio no longer overwrites your saved profile with the robot's defaults.** If the robot loses its profile, your last known profile loads as an unsaved draft to review and send.
- An unreadable stored browser profile is kept instead of being deleted.

## Upgrading

Install over any previous version. To use the new reporting, flash firmware 0.8.19 from the Firmware workspace (hold BOOT while uploading). Your saved calibration is preserved: it loads from the old format and shows a notice. Save once to upgrade it and create the backup copy. Export a JSON backup first if you plan a full flash erase.

## Not in this release

Saving without bench mode and bench exit on disconnect (from the earlier 0.3.3 attempt) were left out: the 0.3.8 line handles bench and save policy with its own lease and readback checks.

## Known issue

The firmware software-in-the-loop gait scenario (`simulation/sil/test.ps1`) reports that gait mode is never reached on this firmware line. It predates this release, and the installer build does not run it.