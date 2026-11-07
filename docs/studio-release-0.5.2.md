## Domino Quadruped Studio 0.5.2 (firmware 0.8.27)

The first release for Apple-silicon Macs (M1 to M4). Windows users get the same app as 0.5.1; the firmware is unchanged at 0.8.27.

### macOS (Apple silicon)
- Download **Domino-Quadruped-Studio-0.5.2-arm64.dmg**, open it and drag Domino Quadruped Studio into Applications.
- The app is not signed with an Apple Developer ID. The first time, right-click the app and choose **Open**, then **Open** again (or click **Open Anyway** under System Settings → Privacy & Security). If macOS says the app is damaged, run `xattr -cr "/Applications/Domino Quadruped Studio.app"` once in Terminal.
- The simulator, CAD preview, LIVE connection, calibration and gait tools work as on Windows. The robot's USB port shows up as `/dev/cu.usbserial-…` and is picked automatically.
- **UPLOAD FW** needs PlatformIO installed on the Mac, for example `brew install platformio`.
- On a Mac, Studio checks for updates at launch and offers the new .dmg as a download, because macOS won't let an unsigned app replace itself.

### Also in this release
- The local Studio server shuts down cleanly when it is stopped, on every platform.

### From 0.5.1
- Walking starts while you are already holding the stick.
- Every foot turns at the same rate, so there's no sideways foot slip when turning. Flash firmware 0.8.27 if you haven't yet.
