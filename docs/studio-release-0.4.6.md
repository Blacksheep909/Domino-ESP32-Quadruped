## HOTFIX: Domino Quadruped Studio 0.4.6 (firmware 0.8.22)

> **Hotfix. Install this if importing a calibration file, or restoring one, kept putting an older calibration back on the robot.**

### What was happening
The robot stores and reloads its calibration correctly; it always runs whatever was last saved. An older calibration kept coming back through Studio:

- **Importing an old file.** **EXPORT ROBOT'S PROFILE** saves what is *stored on the robot*, not unsaved edits in the panel. A file exported before a save therefore holds the older values, and importing it then saving put them back, with nothing saying the file was older.
- **Studio's restore copy went stale.** Studio keeps the last good robot profile so it can restore a robot that has lost its saved calibration. That copy was not updated after a verified save, so it could hold an older profile and offer it as the one to restore.

### The fixes
- Importing a file **older than the robot's current calibration** is shown in red in the save dialog, with both dates. It cannot be saved without ticking **"replace the robot's newer calibration with this older file"**.
- Importing a file that **matches** what the robot already runs says "Nothing to save" instead of opening the dialog.
- Studio's restore copy is updated after every verified save.

### Tips
- Only **SAVE TO ROBOT** changes the robot. The top bar shows "N joints changed — not on the robot yet" until you do.
- After saving, use **EXPORT ROBOT'S PROFILE** for a backup, and delete old exports so they cannot be imported by mistake.

### Upgrading
Install over any previous version. Firmware is unchanged (0.8.22); your saved calibration is kept.
