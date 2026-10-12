# Studio 0.3.1: appearance and calibration preview audit

Date: 30 September 2026. Application: Domino Quadruped Studio.

## Outcome

The appearance sweep covers both themes across the LIVE pages, simulator and
application dialogs. Shared semantic colours now control the mission map, HUD,
sensor graphics, field surfaces, warnings and active controls. The calibration
presentation now follows the existing physical firmware drive correction. A
preview nudge increments the pose currently visible in the model.

## Calibration preservation and update history

The recent 0.2.29 and 0.3.0 update records were reviewed in this chat. Those
updates changed and installed Studio UI code; they did not write calibration to
the robot. The earlier ramp firmware work explicitly recorded that firmware had
been built but had not been flashed or tested on the physical robot by the agent.
This review describes those recorded actions, rather than a new read of the
robot's current nonvolatile storage.

The existing `DOMINO_PLANAR_DRIVE_CROSS_REVERSED=1` correction exchanges and
reverses upper/lower *drive deltas around their existing electrical neutrals*.
It was already present when this sweep began. Its rationale is recorded in
[the earlier pitch/yaw audit](/C:/Users/Blacksheep/Documents/DominoQuadruped/docs/pitch-yaw-audit-2026-09-28.md).

Studio now describes the mechanical joint moved by each existing output:

| Leg | Hip output | Upper output | Lower output |
| --- | ---: | ---: | ---: |
| FL | CH 0 | CH 2 | CH 1 |
| FR | CH 3 | CH 15 | CH 4 |
| BL | CH 14 | CH 8 | CH 7 |
| BR | CH 9 | CH 11 | CH 10 |

These are corrected presentation labels for the existing logical outputs.
The profile schema, saved channel routes, electrical neutral angles, trims,
directions and limits retain their existing representation. The actual PCA9685
route still comes from each saved profile entry. No profile migration occurs.
The model preview uses a separate output-to-CAD mapping, including the existing
planar sign correction. Simulator joint definitions retain the SIL convention.

This sweep made no robot writes, physical jogs, firmware source edits or flashes.
The firmware files already modified in the working tree were preserved.

## Findings and repairs

1. **High: misleading calibration preview.** The UI treated the nominal upper
   output as the CAD upper joint even though the existing physical correction
   drives the opposite planar joint. Selection, wiring labels, model motion,
   mesh selection and travel annotations now use the separate presentation map.
   Profile serialization retains the original output identities.
2. **High: first preview nudge jumped from a scrubbed pose.** Scrubbing/sweeping
   and temporary jogging had separate starting values. A +1 click could switch
   to the old zero-based value. Offline nudges now increment the visible pose
   and clamp to its configured travel. The preview does not mark the profile
   dirty. Physical trim and robot-save controls retain their bench gates.
3. **Medium: dark mission console in light mode.** Map/HUD fields, grid,
   compass, scale, setup overlay, saved-route controls, preflight states and
   route overlays had hardcoded dark surfaces. These now use theme variables.
4. **Medium: unreadable sensor graphics in dark mode.** LiDAR and GPS SVG
   elements used IDs while the old theme rules targeted classes. Explicit
   selectors now colour the labels, grid and scan rays. LEFT/RIGHT labels are
   anchored inward so they remain inside the radar viewport.
5. **Medium: inconsistent application controls and readouts.** Pairing protocol
   fields, dialog headings, model-health values, active joint controls, ramp
   readouts, range inputs, checkbox accents and disabled controls now follow the
   shared palette. The ramp test-move readout is 18 px in both themes; the old
   dark LIVE utility rule had forced it to 10 px.
6. **Medium: canvas colours did not follow the theme.** Comparison and candidate
   chart lines now use the appropriate theme palette and preserve their distinct
   signal colours. Native form controls and scrollbars use the theme colour
   scheme.

## Review method and evidence

1. Captured and inspected the existing screens in light and dark before editing.
2. Reviewed CSS/DOM selectors and the calibration-to-model conversion path.
3. Applied the theme and preview fixes, then built the production frontend.
4. Inspected the revised screens and checked computed styles where the legacy
   cascade could override the new rules.
5. Exercised the BR Upper model preview while LIVE remained disconnected.
   A scrubbed -26.9 degree pose became -25.9 degrees after one +1 click.
   The offset remained 0.0, output remained CH 11, and physical trims were locked.
6. Built the NSIS installer and replaced the existing Studio installation.

Screenshots were taken from a separate localhost QA origin. The robot link
remained disconnected. No device-location permission, vehicle command or
calibration-save action was exercised. Views were inspected at their native
browser viewport sizes, including 1280 by 720 and 1628 by 1244.

### Main comparison

Before: light theme with a dark mission console.

![Before: mission console in light mode](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/01-gps-lidar-light-before.png)

After: mission console follows light mode.

![After: mission console in light mode](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-gps-lidar.png)

Calibration preview after the relative +1 nudge.

![BR Upper preview after a one-degree nudge](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-calibration-nudge.png)

### Coverage

Every row was reviewed in both themes. Links identify the inspected post-fix
screens; the initial captures are retained alongside them with `before-` names.

| Area | Light evidence | Dark evidence |
| --- | --- | --- |
| Compare | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/after-light-compare.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-compare.png) |
| Data | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/after-light-data.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-data.png) |
| GPS / LiDAR and mission map | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-gps-lidar.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-gps-lidar.png) |
| LiDAR radar / GPS track | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-lidar.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-lidar.png) |
| Calibration joint selection | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-calibration-select.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-calibration-select.png) |
| Wiring dialog | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-wiring.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-wiring.png) |
| Ramp preview | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-ramp.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-ramp.png) |
| Gaits | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/after-light-gaits.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-gaits.png) |
| Diagnostics | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/after-light-diagnostics.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-diagnostics.png) |
| Sessions | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/after-light-sessions.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-sessions.png) |
| Project dialog | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-project.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-project.png) |
| Firmware dialog | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-firmware.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-firmware.png) |
| Pairing dialog | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-connection.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-connection.png) |
| MAVLink fields | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-mavlink.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-mavlink.png) |
| Manual control dialog | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-manual.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-manual.png) |
| Shortcuts dialog | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-shortcuts.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-shortcuts.png) |
| Simulator / inspection | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-simulation.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-simulation.png) |
| Controller dialog | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-controller.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-controller.png) |
| Gait Lab dialog | [Light](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-light-gait-lab.png) | [Dark](/C:/Users/Blacksheep/.codex/visualizations/2026/09/29/01a0ebd2-df9a-7f91-88de-2c59adfb291e/theme-audit-031/final-dark-gait-lab.png) |

The calibration safety, neutral, limits and review steps were also inspected.
Their additional `after-light-calibration-*` captures are retained in the same
evidence folder, as are map-overlay and dark ramp-control captures.

## Verification

- Full Studio Node suite: **324 passed, 0 failed**.
- Regression cases exercise each leg's corrected preview mapping, BR output
  selection, -26.9 to -25.9 first-click behavior, fine steps, asymmetric limits,
  unchanged saved profile serialization and unchanged neutral electrical targets.
- Browser verification exercised the actual offline nudge buttons and confirmed
  physical trims remained disabled. The final dark active joint background is
  RGB 40,119,76; the ramp readout is 18 px.
- `node --check web/src/main.js` and `git diff --check` passed. The latter emits
  existing LF/CRLF conversion warnings.
- Production Vite build and NSIS packaging passed. The existing bundle-size
  advisory remains; it is not a build failure.
- No browser console errors were recorded in the final checked view.

## Scope and remaining limits

This is a theme/presentation and offline preview verification, not a physical
calibration measurement. The disconnected browser cannot establish the dog's
current profile values. Robot readback remains authoritative when connected.
GPS and LiDAR graphics were reviewed in their waiting state; a live sensor stream
and online basemap tile service were not available for this audit. CAD proximity
is a broad bounding-box screen and does not certify real mechanical clearance.
Text contrast, visible control states and native controls were inspected;
this was not a complete screen-reader or accessibility certification.

## Installation verification

The finalized installer completed with exit code 0. Registry inspection found
one Domino installation, DisplayVersion **0.3.1**. The executable reports
ProductVersion **0.3.1.0**. Both Start Menu and public Desktop entries are named
`Domino Quadruped Studio v0.3.1`.

The installed and packaged `app.asar` SHA-256 hashes match:

`3295DF274176614DB19989E881762604EF27A47DCBEA90604DFED1C0AEEF3B9A`

Installed executable:
`C:\Program Files\DominoQuadrupedStudio\DominoQuadrupedStudio.exe`.
The final frontend assets are `index-UcyseTy8.css` and `index-oUK-oh-L.js`.

## Public application documentation

Published commit `7c87281518f04885d628db960995a6360cd17959` to GitHub `main`.
That commit contains exactly the main README, the new Studio user guide and
the standing-simulation JPEG. GitHub readback confirmed their blob identities,
and the rendered repository homepage loaded the image at 1628 by 1244 pixels.
All 25 relative links in the published documentation resolved locally; the
linked existing Studio-branch engineering guides were also checked on GitHub.

The screenshot was captured after the native SIL completed its stand transition:
280 mm commanded height, four ground contacts, closed linkages, and no physical
robot connection. The guide distinguishes the local 0.3.1 development build
from the currently public 0.2.26 source branch and the absence of a published
installer release.

- [Studio section on the repository homepage](https://github.com/Blacksheep909/Domino-ESP32-Quadruped#domino-quadruped-studio)
- [Application guide](https://github.com/Blacksheep909/Domino-ESP32-Quadruped/blob/main/docs/quadruped-studio.md)
