# Control Notes

These notes document the assumptions that matter most when extending the firmware. The code comments in `src/main.cpp`, `src/ik.cpp`, and `src/leg_controller.*` are the source of truth.

## Coordinate Frames

Body/world frame:

- Origin is the center of the hip rectangle.
- `+X` points forward.
- `+Y` points to the robot's left side.
- `+Z` points upward.

Leg frame passed into `IK()`:

- Origin is the hip joint rotation center.
- `+X` points forward.
- `+Y` points left.
- `+Z` points downward from the hip toward the ground.

The body model converts body pose into hip-to-foot vectors in this leg frame before calling inverse kinematics.

## Kinematics

Each leg is treated as a 3-DoF mechanism:

- Hip abduction/adduction angle: `theta1`.
- Upper leg pitch angle: `theta2`.
- Lower linkage / knee angle: `theta3`.

The IK geometry constants are CAD-derived:

- `L2 = 160 mm`
- `L3 = 153 mm`
- `Ry = 38 mm`
- `Rz = 21 mm`

Do not change these values unless the physical leg geometry changes.

## Servo Mapping

The PCA9685 channels are assigned as:

| Leg | Hip | Upper | Lower |
| --- | --- | --- | --- |
| FL | 0 | 1 | 2 |
| FR | 3 | 4 | 15 |
| BL | 14 | 7 | 8 |
| BR | 9 | 10 | 11 |

Servo electrical midpoint is modeled as 135 degrees. The general mapping is:

```cpp
hipAngle   = 135.0f + hipTrimDeg   + hipDir   * theta1;
upperAngle = 135.0f + upperTrimDeg + upperDir * theta2;
lowerAngle = 135.0f + lowerTrimDeg + lowerDir * theta3;
```

The firmware assumes all hip servos are mounted in the same world orientation. Left and right differences are handled by upper/lower direction signs and trim values.

## RC Inputs

The project uses CRSF / ExpressLRS input over `Serial2`. Channel values are converted into microsecond-style values and filtered before the control state machine consumes them.

Important controls:

- Roll stick: body roll in tilt mode.
- Pitch stick: body pitch in tilt mode.
- Yaw stick: body yaw in tilt mode.
- SA: stand/stow command.
- SB: ride-height preset.
- SC: balance mode request.
- SD: body tilt enable.

Switches use hysteresis and debounce timing so mode transitions do not chatter.

## Mode Flow

At a high level, each loop:

1. Reads and parses CRSF frames.
2. Updates link/failsafe state.
3. Converts RC switches into menu intent.
4. Updates the target leg height ramp.
5. Enters or exits tilt and balance modes based on switch state and pose readiness.
6. Computes body pose or balance Z offsets.
7. Calls the leg controller, which calls IK and writes PWM outputs.

If CRSF link health is lost, the firmware moves toward the compact stow target.

## Balance Experiment

`BODY_BALANCE` estimates roll and pitch from the MPU6050 accelerometer, captures a reference orientation when entering balance mode, and computes per-leg Z offsets to resist deviation from that reference.

This is intentionally conservative:

- Large tilt angles cancel balance correction.
- Commands are clamped.
- The output is proportional-only for now.

Treat this as an experimental balancing layer, not a complete dynamic stabilizer.

