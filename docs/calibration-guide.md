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

Browser calibration schema v1 files import as the default fixed wiring map and
are rewritten as schema v2. Older binary NVS blobs do not match the v2 record
size and safely fall back to the compiled wiring and calibration defaults.

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
