## HOTFIX: Domino Quadruped Studio 0.4.7 (firmware 0.8.23)

> **Hotfix. Install this and flash firmware 0.8.23 if APPLY DRAFT TO ROBOT on the Gaits page never saved a gait. The robot kept walking with the built-in "Firmware safe" gait.**

### The bug
Firmware 0.8.22 and earlier only stored a gait profile if every servo output was switched off. A powered robot under radio control always has its outputs on, so every gait apply from Studio was refused. The refusal only showed as a plain status line.

### The fixes
- **Firmware 0.8.23** accepts a gait whenever the robot is disarmed and **not walking** (standing, sitting or stowed). The gait is only used once the robot walks, so changing it while standing is safe. While walking it refuses with: *"Stop walking first: set the walk switch to stand, then apply the gait."* The two-slot save (write the spare slot, read it back, then switch) and **REVERT ROBOT** are unchanged.
- **Older gait files**: importing a gait older than the robot's saved gait shows both dates, and applying it asks before replacing the newer gait. This is the same safeguard calibration got in 0.4.6.
- Studio waits 10 seconds for a gait save (was 3), so a slow save is not reported as a failure.
- A refused apply is shown as an error.

### Upgrading
Install over any previous version, then flash firmware **0.8.23** from Studio (hold BOOT while uploading). Your calibration and settings are kept.
