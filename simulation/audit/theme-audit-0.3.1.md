# Studio 0.3.1: appearance and calibration preview audit

Date: 30 September 2026. Application: Domino Quadruped Studio.

## Outcome

The appearance sweep covers both themes across the LIVE pages, simulator and
application dialogs. Shared semantic colours now control the mission map, HUD,
sensor graphics, field surfaces, warnings and active controls. The calibration
presentation now follows the existing physical firmware drive correction. A
preview nudge increments the pose currently visible in the model.

## Calibration mapping and profile compatibility

The `DOMINO_PLANAR_DRIVE_CROSS_REVERSED=1` correction exchanges and
reverses upper/lower *drive deltas around their existing electrical neutrals*.
Its rationale is recorded in
[the pitch/yaw audit](../../docs/pitch-yaw-audit-2026-09-28.md).

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
6. Built the NSIS installer.

Screenshots were taken from a separate localhost QA origin. The robot link
remained disconnected. No device-location permission, vehicle command or
calibration-save action was exercised. Views were inspected at their native
browser viewport sizes, including 1280 by 720 and 1628 by 1244.

### Theme comparison and coverage

The light-theme mission console previously retained dark surfaces. The revised
console follows the selected theme. The offline calibration preview was also
checked after a relative one-degree nudge.

The following areas were reviewed in both light and dark themes:

| Area | Light theme | Dark theme |
| --- | --- | --- |
| Compare | Reviewed | Reviewed |
| Data | Reviewed | Reviewed |
| GPS / LiDAR and mission map | Reviewed | Reviewed |
| LiDAR radar / GPS track | Reviewed | Reviewed |
| Calibration joint selection | Reviewed | Reviewed |
| Wiring dialog | Reviewed | Reviewed |
| Ramp preview | Reviewed | Reviewed |
| Gaits | Reviewed | Reviewed |
| Diagnostics | Reviewed | Reviewed |
| Sessions | Reviewed | Reviewed |
| Project dialog | Reviewed | Reviewed |
| Firmware dialog | Reviewed | Reviewed |
| Pairing dialog | Reviewed | Reviewed |
| MAVLink fields | Reviewed | Reviewed |
| Manual control dialog | Reviewed | Reviewed |
| Shortcuts dialog | Reviewed | Reviewed |
| Simulator / inspection | Reviewed | Reviewed |
| Controller dialog | Reviewed | Reviewed |
| Gait Lab dialog | Reviewed | Reviewed |

Calibration safety, neutral, limits, review steps, and map overlays were also inspected.

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


## Related documentation

- [Repository overview](../../README.md)
- [Application guide](../../docs/quadruped-studio.md)
