# Domino Pin-Linkage Runtime Tests

This report records the Isaac/PhysX closed pin-linkage tests for Domino simulation work.

## Result

Status: **passed for generic four-bar linkage physics, the CAD-derived Domino lower triangle, and the CAD-derived Domino upper loop**.

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
| Max loop-closure error | `0.00000078 m` |
| Max body linear speed | `0.002128 m/s` |

The test did not produce non-finite body poses or velocities. The `Revolute 51` loop closure stayed effectively closed while `Revolute 58` moved.

## Why This Matters

This proves three useful pieces of the Domino simulation path:

1. Isaac/PhysX can run the passive pin-joint pattern without immediate instability.
2. The real exported Domino lower-linkage pivots can be used in a one-actuator loop without the loop exploding.
3. The second exported linkage loop can also be constrained in isolation.

That is the missing physics ingredient between the simplified one-leg articulation and a more faithful Domino leg.

## Important Limitation

This is still not the finished Domino leg.

The CAD-derived tests use real pivot positions, but simplified rigid bodies, no mesh collisions, no gravity, and conservative drive settings. They prove the isolated one-joint loops can be constrained; they do **not** yet prove the combined leg with both loops active, contacts, masses, servo gains, hard stops, or all four legs is stable.

## Next Work

1. Combine the lower triangle and upper loop into one constrained one-leg mechanism.
2. Compare the constrained linkage output angle against the simplified `lower_linkage` joint used in `prototypes/one_leg`.
3. Merge the stable constrained linkage behavior into the clean one-leg model.
4. Reintroduce gravity, simple collisions, and joint hard-stop checks before duplicating the pattern to all four legs.
