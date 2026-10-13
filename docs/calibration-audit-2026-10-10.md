# Calibration correctness audit — 10 October 2026

Scope: Studio calibration import/export, local draft storage, Pair reconnect,
bench permission, physical jog, save acknowledgement/readback, firmware upload,
and ESP32 profile persistence. Changes target Studio 0.3.8 / firmware 0.8.18.

## Evidence for the reported old calibration

The selected good file is Downloads/domino-calibration-robot-active-2026-10-10.json,
revision 1791620392254. Robot readbacks taken before and after the earlier
0.8.17 upload both contain revision 1790843693194 and have identical SHA256
E60182ED303C604A40D9E2D759F9B0B7114C3EDFE3510185D7B5E7355069FC4B.

| Hip logical/physical channel | Good file offset | Robot offset | Direction in both |
| --- | ---: | ---: | ---: |
| 0 | 0 | -5 | -1 |
| 3 | 12 | 6 | 1 |
| 9 | 12 | 4 | 1 |
| 14 | 0 | -10 | -1 |

The old nonzero offsets are stored on the robot, rather than being the compiled
defaults (which have zero offsets). Firmware reads Preferences namespace
`domino-cal`, key `active`. Normal firmware upload preserves that NVS partition.
It does not import a Downloads JSON or send the browser draft to the robot.
There is no evidence yet identifying which prior operation last wrote the old
revision; the defects below are confirmed independently, not a claim that each
caused this particular earlier write.

**Latest hardware verification:** after flashing 0.8.18 and resetting the robot,
readback contains revision **1791620542387**. All twelve joint records (physical
channel, offset, direction, minimum and maximum) match the good Downloads JSON.
The hip offsets are **0, 12, 12, 0**. This later revision was already present;
the audit verification sent no save, jog, or arm command and did not overwrite it.
The earlier backup therefore describes the older state, not the robot's current
state. The current good calibration survived firmware upload and reset.

Studio also stores a browser copy (`domino-live-calibration-v2`), an unsent draft
(`domino-live-calibration-draft-v1`), and a pre-sync backup
(`domino-live-calibration-before-robot-sync-v1`). Those copies are distinct from
robot NVS. An imported draft must remain visible until matching robot readback
confirms the requested save, or the user explicitly chooses robot values.

## Confirmed defects and corrections

1. **Session reset threw ReferenceError.** An obsolete undeclared variable
   blocked profile sync and bench requests. Removed in 0.3.7; browser-function
   tests now execute in strict mode to reproduce actual ES module semantics.
2. **Pending bench entry blocked exit.** Leaving calibration could not exit
   while another request was pending. A separate release sends exit, cancels
   the pending UI request, and ignores its late enter acknowledgement.
3. **Hidden or simulation workspace retained control.** Keepalive now releases
   bench when calibration loses visibility or workspace ownership.
4. **Support acknowledgement withdrawal left outputs active until lease expiry.**
   Unchecking support now releases control immediately.
5. **Inconsistent physical-button prerequisites.** Request, jog, and save use
   one policy covering fresh telemetry, disarmed state, active profile/session,
   support acknowledgement, workspace, battery, and pending requests, with a
   visible reason for blocked controls.
6. **Rejected jog acknowledgements were ignored.** Show the actual refusal and
   close bench permission while keeping user trims.
7. **Import silently normalized invalid numbers and directions.** Strictly
   reject invalid JSON profile values; preserve valid firmware travel limits
   during serialization instead of silently narrowing them.
8. **Replacing a profile retained physical control.** Import, restore, and
   explicit use of robot values now revoke active/pending bench permission.
9. **Jog ignored configured travel limits.** Clamp preview jog to the selected
   joint limits intersected with the physical ±10-degree jog envelope; refuse
   commands outside that intersection.
10. **Bench entry retained prior radio PWM.** Firmware disables all channels
    first. Only an explicit subsequent jog drives the selected physical output.
11. **Arming competed with bench calibration.** Browser, companion, and firmware
    block arming during bench control; browser/companion also guard pending entry.
12. **Firmware upload could interrupt save verification.** Server tracks the
    save through its subsequent profile readback and blocks upload during that
    interval. Disconnect clears pending session state.
13. **Save status remained sending/verifying after disconnect.** Reset marks it
    unconfirmed, preserving the expected save and draft for reconnect readback.
14. **Relay expired save requests too early.** Relay discarded requests after
    five seconds although physical/browser save timeouts were ten seconds.
    Persistence requests now have a twelve-second relay window, so a delayed
    valid acknowledgement still reaches the browser.
15. **Firmware narrowed JSON integers before validation.** Oversized channels
    or directions could wrap to valid values. Validate their types/direction
    before converting to the stored representation.

## Validation and limits

- Full Studio suite: **346 passed**, including new strict browser reset/release,
  hidden/simulation/support-loss, save/reboot/readback mismatch, import bounds,
  arm exclusion, upload exclusion, and delayed relay acknowledgement regressions.
- Vite production build passed.
- ESP32 firmware 0.8.18 compiled successfully after the firmware changes.
- Firmware 0.8.18 installed on COM9; every flash image hash was verified. A
  subsequent serial startup hello confirmed 0.8.18, and read-only profile
  verification after reset confirmed the good calibration on all twelve joints.
- Studio 0.3.8 subsequently installed with user-approved Windows elevation.
  Installed app.asar SHA256 matches the packaged build:
  `32D4E86476DA9803CD4DCB34181F433F9D9EF658F166A5677836532F2285CCB0`.
  The installed app launched and its rendered calibration page loaded. Without
  a paired robot it correctly locks bench mode and displays "Connect the robot
  before requesting physical calibration." The support checkbox starts unchecked.
  Windows Application Control blocked the earlier unpacked executable check;
  verification used the successfully installed app without bypassing that policy.
- Git whitespace check passed (existing LF/CRLF conversion notices only).
- Existing broader simulator behavior is outside this scoped calibration audit.
- Installed UI smoke verification passed; no physical jog or arm was performed.
  No restoration write was needed because the current robot already matched.
- Tests do not prove servo mechanical alignment or gyro hardware health.
  Missing gyro is not a prerequisite in the bench-entry logic inspected here.

Backups of both robot readbacks and the good Downloads JSON are retained under
`backups/calibration-disconnect-install/` in this checkout.
