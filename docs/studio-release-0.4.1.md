# Domino Quadruped Studio 0.4.1

Studio 0.4.1 ships with firmware **0.8.20**. It fixes a trap that wiped the robot's hip inversions and trims: "Restore defaults" (and any fallback to defaults) produced a profile that does not invert the left hips, so roll and tilt broke the next time a profile was sent.

## What went wrong

The compiled and Studio defaults were the plain CAD basis: zero trims, and no inversion on the left hips (CH0 and CH14). This robot needs those hips inverted and small horn trims. Restoring defaults, building a profile from scratch, or the robot falling back to defaults all dropped them, and the profile was then sent to the robot. Channels were not mixed up; the profile simply no longer matched the robot. The firmware's own SIL test even patched CH0 and CH14 by hand to compensate.

## Fixes

- **The default is now the verified profile for this robot** (left hips inverted, horn trims included), in both the firmware and Studio. "Restore defaults", a flash with no saved profile, and any fallback all give you the verified profile. A test keeps the firmware and Studio tables identical.
- **A send now shows what it changes.** The confirmation dialog compares your profile with the robot's and lists changed joints, with direction changes and output channel changes in red.
- **Clearer fallback banner.** On firmware that reports its origin, the banner now says the robot is running its built-in baseline rather than a saved profile.

## Upgrading

Install over any previous version. To get the baseline on the robot, flash firmware 0.8.20 (hold BOOT while uploading). A saved profile is kept; the baseline only applies when none can be read. If your robot currently holds a defaults-based profile, import your last good profile and send it; the dialog will list the changes before you confirm.

## Known issue

The firmware software-in-the-loop gait scenario (`simulation/sil/test.ps1`) reports that gait mode is never reached. It predates this release.