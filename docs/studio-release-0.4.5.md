## HOTFIX: Domino Quadruped Studio 0.4.5 (firmware 0.8.22)

> **Hotfix. Install this if uploading firmware fails with "Firmware build workspace belongs to a different source package."**

### The bug
Studio keeps its firmware build files in `%USERPROFILE%\.domino-firmware`. That folder remembered which Studio installation created it. After Studio was installed in a different folder (for example `C:\Program Files` before, the per-user folder now), Studio refused to build or upload firmware at all.

### The fix
A build folder that Studio created is now reused after a reinstall: its files are replaced from the current installation. If you already hit the error, you can also delete `%USERPROFILE%\.domino-firmware` and build again.

### New: update check every time Studio opens
- A few seconds after launch, a small banner bottom-right shows **Checking for updates…**, then **up to date**, or downloads the new version with a progress bar and a **WHAT'S NEW** link.
- When it is downloaded: **RESTART & UPDATE** installs it and reopens Studio in one click. Settings and calibration are kept. **LATER** hides it until next time.
- Restarting is blocked, with the reason shown, while the robot is armed or a calibration jog session is running, because it closes the PC link.
- Offline, it says so and offers **TRY AGAIN**.

### Upgrading
Install over any previous version. From 0.4.5 on, future updates arrive through the banner. Firmware is unchanged (0.8.22).
