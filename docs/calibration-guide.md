# Calibration Notes

> Work in progress: these notes document the current calibration approach. They are not a complete calibration tool or validated production procedure.

Calibration is critical on a servo quadruped. Small horn offsets or incorrect trim values can make multiple servos fight the linkage at the same time.

## Servo Centering

The firmware treats 135 degrees as the electrical midpoint for the 270-degree servos.

Before installing horns:

1. Flash the firmware or a simple servo-centering sketch.
2. Command each servo to 135 degrees.
3. Install the horn as close as possible to mechanical neutral.
4. Prefer mechanical re-clocking before large software trims.

## Trim Values

Trim constants live in:

```text
src/leg_controller.h
```

Example:

```cpp
static constexpr float FR_HIP_TRIM_DEG = -45.0f;
```

Trims affect stand, stow, tilt, balance, and gait. Tune them in a neutral stand pose with tilt, balance, and gait disabled.

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
