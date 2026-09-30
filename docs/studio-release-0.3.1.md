# Domino Quadruped Studio 0.3.1

Windows x64 desktop application for Domino simulation, calibration inspection,
telemetry, gait development and mission planning.

## Downloads

- **Installer:** `Domino-Quadruped-Studio-0.3.1-x64.exe`. Run it and open
  **Domino Quadruped Studio v0.3.1** from the Start menu.
- **Portable:** `Domino-Quadruped-Studio-0.3.1-x64.zip`. Extract the entire ZIP
  and run `DominoQuadrupedStudio.exe` in the extracted folder.
- `SHA256SUMS.txt` contains download checksums. `latest.yml` and the `.blockmap`
  support the installed application's update checker.

Simulation works offline with the bundled service, CAD and native firmware
simulator. A separate Node terminal is not required for ordinary desktop use.

## Included in this release

- Light and dark theme fixes across maps, sensor graphics, controls, graphs
  and joint inspection.
- Animated calibration inspection with readable joint travel, limit margins
  and CAD proximity information.
- Corrected mechanical joint labels and model preview mapping for the existing
  servo drive convention, including the rear right upper and lower joints.
- A temporary +1 degree preview nudge starts from the pose currently displayed,
  including after scrubbing or sweeping the joint.
- Motion smoothing controls, on/off and response previews in Calibration.
- Mission planning with map overlays, local routes and GPS/LiDAR views.

The app retains the existing calibration profile format and saved channel,
neutral, trim, direction and limit values. Installing Studio does not write
calibration or flash firmware to the robot. Physical jogs and persistent robot
changes require the existing connection and bench-mode checks.

## Verification and current limits

- 324 Studio tests passed with no failures.
- Production frontend and Windows installer packaging passed.
- The installed executable reports version 0.3.1; its application archive
  matches the packaged archive.
- Light/dark views and the rear right model preview were inspected while the
  physical robot link was disconnected.

This is a development release for an active prototype. The physical gait,
clearance and live sensor behavior still require hardware validation. CAD
proximity checks are estimates; they do not certify mechanical clearance.

Read the [Studio guide](https://github.com/Blacksheep909/Domino-ESP32-Quadruped/blob/main/docs/quadruped-studio.md)
for setup, workspaces and the calibration workflow. The `v0.3.1` tag records
the source used for this desktop release.
