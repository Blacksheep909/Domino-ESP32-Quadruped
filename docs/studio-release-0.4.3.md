# Domino Quadruped Studio 0.4.3 (hotfix)

Studio 0.4.3 ships with firmware **0.8.22**. Install both: the main fix is in the firmware.

## Hotfix: calibration seemed to be lost between modes and restarts

An audit of the whole path (Studio, the PC link, the firmware's storage and the
radio modes) found that the robot's saved profile was loaded and verified
correctly. Four other bugs made it look lost:

- **After calibrating, the dog went limp and ignored the radio until a reboot.**
  Ending a jog session (leaving the page, unplugging USB, a save, or a keepalive
  timeout) latched radio control off, so going back to stand or walk did nothing.
  Firmware 0.8.22 hands the robot back to the radio, which immediately takes the
  radio-selected pose using the saved profile, just as it does at power-up.
- **Studio's "on robot" copy changed when you edited.** The editable values and
  the copy read from the robot were the same object, so every trim also rewrote
  the "on robot" column, and the save dialog could report no changes. They are
  now kept separate.
- **Jogging started from the walking pose.** The first jog slewed at 5°/s from
  wherever the leg last was (often folded in stow) to the calibrated center and
  then added your +1°, which looked like the calibration being found again. A jog
  session now moves the selected joint to its center first, and says so.
- **Old unsaved edits stuck around.** A draft kept on this PC was used for jogging
  even when it matched the robot. Matching drafts are now cleared on connect, and
  real differences are listed at the top of the page.

## Calibration page rework

- One bar at the top always says whether Studio matches the robot, or how many
  joints have unsaved changes, with **SAVE TO ROBOT**, **DISCARD CHANGES** and
  **RELOAD FROM ROBOT**.
- **Save browser copy** and **Load previous browser copy** are gone. Unsaved
  changes are still kept on this PC automatically.
- **IMPORT FILE…** no longer leaves an "imported" copy that looks applied: it
  shows what the file changes and asks to save it.
- Trims and limits save with one click. The tick box is only needed for
  direction or wiring changes.
- With firmware 0.8.22 a save keeps the jog session open (unless the channel map
  changed), so you can save joint by joint.
- Fast ± clicks are never dropped; the newest target is sent as soon as the
  previous one is acknowledged.
- "Bench mode" is now called a **jog session** in the app.

## Upgrading

Install Studio 0.4.3, then flash firmware 0.8.22 (hold BOOT while uploading).
Your saved calibration is kept. After flashing, open Calibration once and check
the top bar says **Studio matches the robot**; if it lists changes, either save
them or discard them.
