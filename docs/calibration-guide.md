# Calibration Notes

> Work in progress: the LIVE wizard now provides the software workflow and
> persistent firmware profile, but physical limits still need to be measured on
> a supported robot with an E-stop in reach.

Calibration is critical on a servo quadruped. Small horn offsets or incorrect trim values can make multiple servos fight the linkage at the same time.

## LIVE Calibration Wizard

Open **LIVE -> Calibration** for the guided five-step workflow. The profile
stores four things for each stable logical joint:

- the physical PCA9685 output channel (0-15);
- neutral offset;
- normal or inverted direction;
- conservative logical minimum and maximum travel.

The logical joint and physical output are deliberately separate. For example,
`FR Upper` remains the same 3D/mechanical joint even if its cable is moved from
channel 4 to channel 6. This keeps its neutral and limits attached to the joint
while the firmware routes the final PWM pulse to the configured output.

Use **Edit channel map** from the Select Joint step. All 12 assignments must be
unique. The editor highlights duplicate outputs, then presents a second review
screen listing only the changed routes. Confirmation remains locked until the
user acknowledges that an incorrect route can energize a different actuator.

The channel map is safe to edit offline. Physical jog is separately gated by a
connected adapter, disarmed robot, supported-chassis acknowledgement, and robot
bench-mode acknowledgement. Only one physical output is energized during a
jog. Changing to another mapped output turns the previous one fully off.

Saving to the robot has another confirmation step. Firmware turns all 16 PWM
outputs off before validating, checksumming, rereading, and activating the new
profile in NVS. The robot then leaves bench mode, so further movement requires
a fresh bench-mode request.

On each connection to a disarmed robot, Studio reads that robot's active NVS
profile and fills the calibration controls from it. This includes all 12 joint
offsets, directions, limits, and physical output assignments. The robot is the
source for the connected session, even when a new Studio version has an empty
browser cache. Unsaved browser edits are kept for review. When a saved browser
copy differs from the robot, Studio keeps the previous copy under **Load previous
browser copy** before showing the robot values.

Before a profile can be sent, Studio must have read the current robot profile
in that session. Firmware 0.8.15 and later require the read profile's `savedAt`
revision on every write and reject a stale Studio draft. A successful save is
read back from the robot. Normal Studio firmware upload preserves the ESP32 NVS
partition; a full-chip erase does not. Export Robot Profile provides a portable
JSON backup before any full-chip erase or hardware replacement.

Browser calibration schema v1 files import as the default fixed wiring map and
are rewritten as schema v2.

### Importing a profile and getting it onto the robot

Importing a JSON file only loads it into Studio as a **draft**. It is not on
the robot until you press **Send to robot** in the Review step and confirm.
Since firmware 0.8.17 this save does **not** need bench mode, because it only
writes flash (every output is turned off first), so a problem starting bench
mode can no longer stop a calibration being saved. The draft is kept if you
restart Studio, and a robot read never replaces an unsaved draft. After saving,
Studio reads the profile back from the robot and compares it. Older firmware
still requires bench mode for the save.

Closing the Studio connection now tells the robot to leave bench mode and turn
its outputs off, so a stale bench session no longer carries into the next one.

### How the profile is stored (firmware 0.8.16 and later)

The profile lives in the ESP32's NVS flash (namespace `domino-cal`) as an
explicit, little-endian, checksummed record (`active2`, 212 bytes). It is no
longer a raw C struct dump, so compiler or struct changes cannot make a saved
profile unreadable. Each save first keeps the profile being replaced as
`backup2`, then writes and read-back-verifies the new record.

At boot the firmware tries `active2`, then `backup2`, then the old firmware
0.8.15 blob (`active`, read-only, migrated on the next save). Defaults are used
only when nothing usable exists.

### Where is the robot's profile coming from?

The robot reports the source in `robot-hello` (`calibration`) and with every
profile readback (`origin`). Studio shows a banner on the Calibration page:

| Source | Meaning |
| --- | --- |
| `nvs` | Normal: the current saved record. |
| `nvs-backup` | The newest record was unreadable; the previous profile is in use. Review and save to refresh it. |
| `nvs-legacy` | Profile saved by firmware 0.8.15 or earlier. Save once to upgrade the format. |
| `default-empty` | Nothing was ever saved (first boot, or flash erased). **The robot is on compiled defaults.** |
| `default-fallback` | Something was stored but could not be read. **The robot is on compiled defaults.** The `detail` field names the reason (for example `active2:bad-checksum`). |

When the robot is on defaults, Studio does **not** overwrite your saved browser
copy. It keeps that copy (or the last profile read from a healthy robot) as an
unsaved draft: review it, then save it to the robot. The serial console also
prints `Calibration source: ...` at boot.

The hip, upper and lower trims in `src/leg_controller.h` are compiled into the
firmware and applied under every profile; they are not part of the saved record.

## Servo Centering

The firmware treats 135 degrees as the electrical midpoint for the 270-degree servos.

Before installing horns:

1. Flash the firmware or a simple servo-centering sketch.
2. Command each servo to 135 degrees.
3. Install the horn as close as possible to mechanical neutral.
4. Prefer mechanical re-clocking before large software trims.

## Trim Values

Compiled fallback trim constants live in:

```text
src/leg_controller.h
```

Example:

```cpp
static constexpr float FR_HIP_TRIM_DEG = -45.0f;
```

The LIVE profile layers its validated offsets, direction, limits, and physical
channel routing over those fallbacks. It affects stand, stow, tilt, balance, and
gait. Tune in a neutral stand pose with tilt, balance, and gait disabled.

## Suggested Trim Order

1. Lift the robot on a stand.
2. Confirm the CRSF link is alive.
3. Enter stand mode.
4. Disable tilt, balance, and gait modes.
5. Center all transmitter sticks.
6. Adjust hip trims so the legs point symmetrically.
7. Adjust upper/lower trims so the body sits level.
8. Test stow.
9. Test stand again.
10. Repeat until transitions are smooth.

## Healthy Behavior

- Body is roughly level in stand.
- Front legs mirror each other.
- Rear legs mirror each other.
- Feet land in predictable positions.
- Stow transition does not kick the legs sideways.
- Tilt returns to neutral cleanly when released.

## Warning Signs

- A servo buzzes loudly at neutral.
- A linkage contacts a hard stop.
- One side needs much larger trims than the other.
- The body twists when entering stand.
- A foot moves sideways during stow.

If trims become extreme, stop and inspect horn clocking, servo orientation, channel mapping, and mechanical assembly before continuing.
