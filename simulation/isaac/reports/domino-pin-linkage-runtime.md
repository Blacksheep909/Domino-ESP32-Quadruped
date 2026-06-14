# Domino Pin-Linkage Runtime Tests

This report records the Isaac/PhysX closed pin-linkage tests for Domino simulation work.

## Result

Status: **passed for generic four-bar linkage physics and the first CAD-derived Domino lower-linkage loop**.

The tests author small closed linkages directly into the Isaac stage using USD physics primitives:

- A fixed/kinematic reference body.
- One driven revolute input pin.
- Passive revolute pins for the linkage bodies.
- A loop-closing revolute pin.

The driven input uses a sinusoidal angular target while the passive pins and loop-closing constraint are left to PhysX.

## Generic Four-Bar Summary

| Metric | Result |
| --- | ---: |
| Physics steps | `600` |
| Physics dt | `0.005 s` |
| Drive center | `-55 deg` |
| Drive amplitude | `12 deg` |
| Drive frequency | `0.6 Hz` |
| Max loop-closure error | `0.00000012 m` |
| Max body linear speed | `0.244152 m/s` |

This is the stable control test for the constraint pattern. It did not leave non-finite body poses or velocities, and the loop-closing pin stayed effectively closed over the run.

## CAD-Derived Domino Lower Triangle Summary

This is the first test using Domino pivot locations extracted from the generated URDF export.

| Item | Value |
| --- | --- |
| Geometry mode | `domino-lower-triangle` |
| Driven input | `Revolute 59` |
| Passive pins | `Revolute 43`, `Revolute 33`, `Revolute 25/26` closure |
| Source pivot report | `reports/domino-linkage-pivots.md` |

| Metric | Result |
| --- | ---: |
| Physics steps | `600` |
| Physics dt | `0.005 s` |
| Drive center | `-15 deg` |
| Drive amplitude | `8 deg` |
| Drive frequency | `0.4 Hz` |
| Max loop-closure error | `0.00000010 m` |
| Max body linear speed | `0.143320 m/s` |

The test did not produce non-finite body poses or velocities. The duplicated `Revolute 25` / `Revolute 26` closure point stayed effectively closed while the driven input moved.

## Why This Matters

This proves two useful pieces of the Domino simulation path:

1. Isaac/PhysX can run the passive pin-joint pattern without immediate instability.
2. The real exported Domino lower-linkage pivots can be used in a one-actuator loop without the loop exploding.

That is the missing physics ingredient between the simplified one-leg articulation and a more faithful Domino leg.

## Important Limitation

This is still not the finished Domino leg.

The CAD-derived test uses real pivot positions, but simplified rigid bodies, no mesh collisions, no gravity, and conservative drive settings. It proves the one-joint loop can be constrained; it does **not** yet prove the full leg with contacts, masses, servo gains, hard stops, or all four legs is stable.

## Next Work

1. Add the second loop that connects `Revolute 58`, `Revolute 43`, `Revolute 32`, and `Revolute 51`.
2. Compare the constrained linkage output angle against the simplified `lower_linkage` joint used in `prototypes/one_leg`.
3. Merge the stable lower-linkage loop into a full one-leg model.
4. Reintroduce gravity, simple collisions, and joint hard-stop checks before duplicating the pattern to all four legs.
