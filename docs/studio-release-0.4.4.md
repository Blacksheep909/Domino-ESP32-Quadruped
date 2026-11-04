## Domino Quadruped Studio 0.4.4 (firmware 0.8.22)

A small follow-up to the 0.4.3 calibration hotfix. Firmware is unchanged (0.8.22).

### SAVE TO ROBOT now says why it is greyed out
When saving isn't possible yet, hovering over **SAVE TO ROBOT** shows the reason, and the top bar says **"Can't save yet: …"**. The most common case is firmware older than 0.8.21, which only saves while a jog session is running. The message says to start a jog session or flash firmware 0.8.22.

### No more phantom "unsaved changes"
If you edit a joint and then put it back to the robot's value, Studio no longer reports unsaved changes.

### Upgrading
Install over any previous version. If the robot is not on firmware 0.8.22 yet, flash it from Studio (hold BOOT while uploading); your saved calibration is kept.
