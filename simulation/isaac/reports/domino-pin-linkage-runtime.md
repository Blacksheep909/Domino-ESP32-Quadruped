# Domino Pin-Linkage Runtime Tests

This report records the Isaac/PhysX closed pin-linkage tests for Domino simulation work.

## Result

Status: **passed for generic four-bar linkage physics, the CAD-derived Domino lower triangle, the CAD-derived Domino upper loop, and a combined two-drive one-leg linkage**.

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

## CAD-Derived Domino Upper Loop Summary

This is the first isolated test of the second loop on the same DOM_P__4__1 leg. The lower input side is held at the CAD rest pose so the test can focus on the `Revolute 58` driven input and the `Revolute 51` closure.

| Item | Value |
| --- | --- |
| Geometry mode | `domino-upper-loop` |
| Driven input | `Revolute 58` |
| Held input side | `Revolute 59` / `Revolute 43` |
| Passive pin | `Revolute 32` |
| Loop closure | `Revolute 51` |
| Source pivot report | `reports/domino-linkage-pivots.md` |

| Metric | Result |
| --- | ---: |
| Physics steps | `600` |
| Physics dt | `0.005 s` |
| Drive center | `0 deg` |
| Drive amplitude | `5 deg` |
| Drive frequency | `0.4 Hz` |
| Max loop-closure error | `0.00000221 m` |
| Max body linear speed | `0.000568 m/s` |

The test did not produce non-finite body poses or velocities. The `Revolute 51` loop closure stayed effectively closed while `Revolute 58` moved.

## CAD-Derived Combined One-Leg Summary

This combines the lower triangle and upper loop into one simplified DOM_P__4__1 leg mechanism. Both pitch inputs are driven, and both loop closures are monitored.

| Item | Value |
| --- | --- |
| Geometry mode | `domino-combined-leg` |
| Lower driven input | `Revolute 59` |
| Upper driven input | `Revolute 58` |
| Lower loop closure | `Revolute 25` / `Revolute 26` |
| Upper loop closure | `Revolute 32` / `Revolute 51` direct closure |
| Source pivot report | `reports/domino-linkage-pivots.md` |

| Metric | Result |
| --- | ---: |
| Physics steps | `600` |
| Physics dt | `0.005 s` |
| Lower drive center | `-15 deg` |
| Lower drive amplitude | `2 deg` |
| Upper drive center | `0 deg` |
| Upper drive amplitude | `2 deg` |
| Drive frequency | `0.2 Hz` |
| Max lower loop-closure error | `0.00001394 m` |
| Max upper loop-closure error | `0.00000294 m` |
| Max body linear speed | `0.037481 m/s` |

The combined leg did not produce non-finite body poses or velocities. This is the first pass where the two CAD-derived linkage loops share the same coupler and run together under two driven inputs.

## Why This Matters

This proves four useful pieces of the Domino simulation path:

1. Isaac/PhysX can run the passive pin-joint pattern without immediate instability.
2. The real exported Domino lower-linkage pivots can be used in a one-actuator loop without the loop exploding.
3. The second exported linkage loop can also be constrained in isolation.
4. Both loops can run together as one simplified two-drive leg mechanism.

That is the missing physics ingredient between the simplified one-leg articulation and a more faithful Domino leg.

## Important Limitation

This is still not the finished Domino leg.

The CAD-derived tests use real pivot positions, but simplified rigid bodies, no mesh collisions, no gravity, and conservative drive settings. They prove the isolated one-joint loops and a simplified combined leg can be constrained; they do **not** yet prove the full leg with contacts, CAD mesh masses, servo gains, hard stops, or all four legs is stable.

## Next Work

1. Compare the constrained combined-leg output against the simplified `lower_linkage` joint used in `prototypes/one_leg`.
2. Merge the stable constrained linkage behavior into the clean one-leg model.
3. Reintroduce gravity, simple collisions, and joint hard-stop checks.
4. Duplicate the pattern to all four legs and wire it to the twelve-servo action space.
