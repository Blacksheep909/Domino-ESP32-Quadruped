# Calibration Guide

Calibration is where this kind of robot either becomes controllable or turns into random servo fighting. Work slowly and change one thing at a time.

## Servo Centering

The firmware treats 135 degrees as the electrical midpoint for the 270-degree servos.

Before installing horns:

1. Flash firmware or a simple servo-centering sketch.
2. Command each servo to 135 degrees.
3. Install the horn as close as possible to the intended neutral angle.
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

Trims are global. They affect stand, stow, tilt, and future gaits. Tune them in a neutral stand pose with tilt disabled.

## Suggested Trim Order

1. Lift the robot on a stand.
2. Confirm CRSF link is alive.
3. Enter stand mode.
4. Disable tilt and balance modes.
5. Center all sticks.
6. Adjust hip trims so the legs point symmetrically.
7. Adjust upper/lower trims so the body sits level.
8. Test stow.
9. Test stand again.
10. Repeat until transitions are smooth.

## What Good Looks Like

- Body roughly level in stand.
- Front legs mirror each other.
- Rear legs mirror each other.
- Feet land in predictable positions.
- Stow transition does not kick the legs sideways.
- Tilt returns to neutral cleanly when released.

## Warning Signs

- A servo buzzes loudly at neutral.
- A linkage hits a hard stop.
- One side needs much larger trims than the other.
- The body twists when entering stand.
- A foot moves sideways during stow.

If trims become extreme, stop and check the horn clocking or physical assembly.

