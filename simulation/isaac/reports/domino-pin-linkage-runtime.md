# Domino Pin-Linkage Runtime Test

This report records the first Isaac/PhysX closed pin-linkage test for Domino simulation work.

## Result

Status: **passed for generic actuated four-bar linkage physics**.

The test authored a small four-bar linkage directly into the Isaac stage using USD physics primitives:

- Fixed/kinematic ground link.
- One driven crank revolute pin.
- Passive crank-to-coupler revolute pin.
- Passive ground-to-rocker revolute pin.
- Passive loop-closing coupler-to-rocker revolute pin.

The driven crank joint used a sinusoidal angular target while the passive pins and loop-closing constraint were left to PhysX.

## Local Test Summary

| Metric | Result |
| --- | ---: |
| Physics steps | `600` |
| Physics dt | `0.005 s` |
| Drive center | `-55 deg` |
| Drive amplitude | `12 deg` |
| Drive frequency | `0.6 Hz` |
| Max loop-closure error | `0.00000012 m` |
| Max body linear speed | `0.244152 m/s` |

The test did not leave non-finite body poses or velocities, and the loop-closing pin stayed effectively closed over the run.

## Why This Matters

This proves the basic constraint pattern needed for Domino's lower linkage:

1. Drive one primary joint.
2. Let the remaining pin joints behave passively.
3. Close the linkage loop with a PhysX revolute constraint.
4. Keep solver settings conservative while validating the linkage.

That is the missing physics ingredient between the simplified one-leg articulation and a more faithful Domino leg.

## Important Limitation

This test uses generic four-bar dimensions, not Domino's real CAD pivot positions.

It proves that Isaac/PhysX can run the pin-joint pattern without immediate instability. It does **not** yet prove that the actual Domino lower linkage geometry, masses, collision shapes, servo gains, or hard stops are stable.

## Next Work

1. Extract Domino lower-linkage pivot positions from CAD or the URDF mesh/joint export.
2. Replace the generic four-bar dimensions in `prototypes/pin_linkage/run_pin_linkage.py`.
3. Compare the constrained linkage output angle against the simplified `lower_linkage` joint used in `prototypes/one_leg`.
4. Only after the constrained one-joint linkage remains stable, merge the pattern into a one-leg Domino linkage model.
