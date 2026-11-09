## Domino Quadruped Studio 0.5.5 (firmware 0.8.29)

A new look, the Policy Lab, a fix for the Gaits preview, and Windows updates that install properly. Flash firmware **0.8.29** from **FIRMWARE** (hold BOOT while uploading). Calibration, gaits and settings are kept.

**Windows users on 0.5.4 or older:** download the `.exe` below and run it once by hand. Earlier versions could fail to install updates without saying so (see below). From 0.5.5 on, Studio tells you when that happens.

### Windows updates
- **Updates that fail now say so.** A silent update could fail and leave Studio on the old version with no message. Studio now notices on the next launch and shows **The update to X didn't install** with a **RUN INSTALLER** button. That opens the installer in a window, so any problem is visible.
- **Every launch checks for updates.** The check now starts as soon as Studio opens, instead of waiting for the window to finish loading.
- **The version in Windows is correct again.** The Start menu and desktop shortcut are now just **Domino Quadruped Studio**. Old shortcuts with a version in their name (such as "v0.4.8") are removed. Installed apps shows the real version.
- Update activity is logged to `%APPDATA%\Domino Quadruped Studio\logs\updater.log`.

### Gaits
- **The preview no longer folds the robot into a crouch.** After visiting COMPARE, the Gaits preview could bend every leg further each frame and leave the robot splayed on every page until a restart.
- The extra grid over the floor in the Gaits preview is gone. The green line still shows the next 6 s of path.

### New look
- Studio's green colour scheme is now slate blue on neutral greys, in light and dark mode.

### Policy Lab (new)
- **POLICY LAB** in the top bar trains a learned walking policy in Isaac Lab (Isaac Sim needed on the PC). You can follow training, check that a policy fits the ESP32, and install it into the firmware you flash. The Isaac Lab training setup is in `simulation/isaac_lab`.

### Firmware 0.8.29
- **Radio SB fully down runs an installed learned policy.** The sticks drive and CH3 sets ride height. No policy is installed by default, so SB down keeps the robot standing until you install one from the Policy Lab.

Windows and Apple-silicon Mac builds are attached.
