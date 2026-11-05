## HOTFIX: Domino Quadruped Studio 0.4.8 (firmware 0.8.24)

> **Hotfix. Install this and flash firmware 0.8.24 if SAVE TO ROBOT for motion smoothing was always refused.**

### The bug
Like gait profiles before 0.4.7, the firmware only saved motion smoothing settings if every servo output was switched off. A powered robot under radio control always has its outputs on, so smoothing could never be saved.

### The fixes
- **Firmware 0.8.24** saves smoothing whenever the robot is disarmed and not walking, with no calibration jog session running. Saved rates take effect immediately.
- **3D preview panel on Calibration and Gaits** is placed from the page itself: it starts 16 px below the title, notice and save bar (moving down when a notice appears), uses one 16 px gap on every side, ends on the same edge as the page titles, and shrinks on short windows so its caption stays on screen. Previously a notice could slide the save bar under the preview.
- **In-app Guide** rewritten for the current calibration buttons (SAVE TO ROBOT, DISCARD CHANGES, START JOG SESSION). It explains that EXPORT ROBOT'S PROFILE saves what is on the robot, not unsaved edits. The smoothing and gait guides are updated too.
- **RESTORE SELECTED JOINT** was an unstyled default button; it now matches the others.
- Remaining "browser draft" wording removed.

### Upgrading
Install over any previous version, then flash firmware **0.8.24** from Studio (hold BOOT while uploading). Your calibration, gait and settings are kept.
