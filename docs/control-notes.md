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

### Standalone operation is mandatory

The ESP32 and its physical CRSF receiver are the primary robot control system.
Normal stand, stow, tilt, height, and gait operation must work after boot with
no browser, companion process, USB data connection, Bluetooth connection, or
Wi-Fi connection. LIVE is an optional telemetry and engineering layer, never a
runtime dependency for the radio.

A read-only LIVE connection may observe and record the robot without taking
control. Only an explicit LIVE safety or calibration action may inhibit the
physical path: disarm, E-stop, watchdog, fault, or acknowledged bench mode.
Unplugging an otherwise read-only telemetry connection must leave CRSF control
working. If LIVE has explicitly taken motion authority while armed, loss of
that authority remains fail-closed through the independent watchdog.

Important controls:

- Right-stick horizontal: body roll in tilt mode; differential turn command in gait mode.
- Right-stick vertical: body pitch in tilt mode; forward/reverse command in gait mode.
- Left-stick horizontal: body yaw in tilt mode; ignored by the first gait.
- SA: stand/stow command.
- SB: currently unbound.
- SC: up/middle keeps normal stand; fully down requests gait mode.
- SD: body tilt enable and hard gait interlock.

Switches use hysteresis and debounce timing so mode transitions do not chatter.

## Mode Flow

At a high level, each loop:

1. Reads and parses CRSF frames.
2. Updates link/failsafe state.
3. Converts RC switches into menu intent.
4. Updates the target leg height ramp.
5. Enters or exits tilt, gait, and balance modes based on switch state and pose readiness.
6. Computes body pose, gait foot targets, or balance Z offsets.
7. Calls the leg controller, which calls IK and writes PWM outputs.

If CRSF link health is lost, the firmware moves toward the compact stow target.

## Balance Experiment

`BODY_BALANCE` estimates roll and pitch from the MPU6050 accelerometer, captures a reference orientation when entering balance mode, and computes per-leg Z offsets to resist deviation from that reference.

This is intentionally conservative:

- Large tilt angles cancel balance correction.
- Commands are clamped.
- The output is proportional-only for now.

SC middle is deliberately not mapped to balance during gait bring-up. The
filtered three-position switch passes through its midpoint on the way to fully
down; leaving balance on the midpoint would briefly trigger it on every gait
selection.

## Sinusoidal Gait

`BODY_GAIT` is the first walking milestone. It uses a diagonal trot: FL/BR share
one phase and FR/BL are offset by 180 degrees. A planted foot follows a
half-cosine rearward stroke, then a matching raised return arc. Both horizontal
phases have zero endpoint velocity to reduce touchdown and liftoff slip. The 62%
stance duty adds a short four-foot support window between diagonal transfers.
Stride, lift, frequency, and stick slew are intentionally conservative while
the real mechanism is validated.

Gait requires stand pose readiness, SC fully down, a live CRSF link, and SD/tilt
off. Any tilt request blocks or exits gait immediately.

Treat this as an experimental balancing layer, not a complete dynamic stabilizer.
