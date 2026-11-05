## HOTFIX: Domino Quadruped Studio 0.4.9 (firmware 0.8.24)

> **Hotfix. Install this if APPLY DRAFT TO ROBOT on the Gaits page was always greyed out, even with the robot paired and disarmed.**

### The bug
Studio enables **APPLY DRAFT TO ROBOT** only when the robot reports that it can store gait profiles. The robot does report this when it connects, but the PC companion that relays its messages:
- left that setting out of its capability list, so Studio never received it, and
- left it out of every gait reply, along with the "saved" flag and the robot state, so even reading the robot's gait turned apply support off.

As a result no gait could be applied from Studio in any session.

### The fixes
- The companion now relays the robot's gait-storage support, and passes on the saved and undo flags and the robot state in every gait reply.
- Studio no longer turns apply support off when a reply leaves it out.
- When **APPLY DRAFT TO ROBOT** is disabled, the reason is shown under it and on hover (for example: disarm the robot, flash firmware 0.8.23 or later, or fix the preview's IK warning).

### Upgrading
Install over any previous version (or use **RESTART & UPDATE** in Studio's update banner). Firmware 0.8.24 is still current; flash it if you have not, so the robot accepts gait and smoothing saves. Your calibration is kept.
