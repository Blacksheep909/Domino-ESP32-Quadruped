# Domino Pin-Linkage Runtime Tests

This report records the Isaac/PhysX closed pin-linkage tests for Domino simulation work.

## Result

Status: **passed for generic four-bar linkage physics, the CAD-derived Domino lower triangle, the CAD-derived Domino upper loop, a combined two-drive one-leg linkage, an all-leg four-module pitch-linkage scene, a fixed-base twelve-actuator scene, and a shared fixed-body twelve-actuator scene**.

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

A separate 1000-step characterization pass records target ranges, actual body pitch ranges, relative linkage angles, tracked pivot motion, drive tracking error, and a local linear calibration fit from drive targets to measured linkage-output proxies. See `reports/domino-combined-linkage-characterization.md`.

## CAD-Derived Four-Leg Pitch-Linkage Summary

This combines four CAD-derived pitch-linkage modules into one fixed-base Isaac scene. Each leg has one lower linkage drive, one upper pitch drive, one lower loop closure, and one upper loop closure.

| Item | Value |
| --- | --- |
| Geometry mode | `domino-four-combined-legs` |
| Driven pitch inputs | `8` |
| Loop closure checks | `8` |
| Source pivot report | `reports/domino-linkage-pivots.md` |

| Metric | Result |
| --- | ---: |
| Stability smoke-test steps | `600` |
| Independent calibration steps | `3200` |
| Physics dt | `0.005 s` |
| Lower drive amplitude | `1 deg` |
| Upper drive amplitude | `1 deg` |
| Drive frequency | `0.15 Hz` |
| Max loop-closure error | `0.00001217 m` |
| Max body linear speed | `0.039359 m/s` |
| Independent calibration matrix rank | `9 / 9` |

All eight CAD loop closures stayed bounded in one scene. A separate independent drive sweep moved one pitch drive at a time and produced a full-rank local calibration fit for the eight pitch inputs. `Revolute 47` on `DOM_P__25__1` is treated as a lower drive for this smoke test because its pivot mirrors the lower drive locations on the other legs, but the CAD URDF currently labels it as `continuous`; that needs a final design decision before policy training. See `reports/domino-four-leg-linkage-runtime.md`.

## CAD-Derived Twelve-Actuator Summary

This extends the fixed-base all-leg pitch-linkage scene with the four shoulder hip ab/ad drives from the CAD URDF. Each leg now exposes three commanded actuators: shoulder, lower linkage drive, and upper pitch drive.

| Item | Value |
| --- | --- |
| Geometry mode | `domino-four-12-actuators` |
| Driven actuator channels | `12` |
| Shoulder hip ab/ad drives | `4` |
| Lower linkage drives | `4` |
| Upper pitch drives | `4` |
| Loop closure checks | `8` |
| Source pivot report | `reports/domino-linkage-pivots.md` |

| Metric | Result |
| --- | ---: |
| Phased-sine smoke-test steps | `600` |
| Independent calibration steps | `2400` |
| Independent calibration samples | `1920` |
| Independent calibration matrix rank | `13 / 13` |
| Max loop-closure error | `0.00001427 m` |
| Max body linear speed | `0.029792 m/s` |

The twelve-actuator independent sweep moved one actuator at a time and produced a full-rank local calibration matrix across the complete command layout. See `reports/domino-12-actuator-runtime.md`.

The stronger follow-up mode, `domino-four-12-fixed-body`, attaches all four shoulder joints to a single shared kinematic body reference. Its independent sweep also passed:

| Metric | Result |
| --- | ---: |
| Independent calibration steps | `1200` |
| Independent calibration samples | `960` |
| Independent calibration matrix rank | `13 / 13` |
| Max loop-closure error | `0.00001425 m` |
| Max body linear speed | `0.029793 m/s` |

## Why This Matters

This proves nine useful pieces of the Domino simulation path:

1. Isaac/PhysX can run the passive pin-joint pattern without immediate instability.
2. The real exported Domino lower-linkage pivots can be used in a one-actuator loop without the loop exploding.
3. The second exported linkage loop can also be constrained in isolation.
4. Both loops can run together as one simplified two-drive leg mechanism.
5. Over a conservative small-angle sweep, the two driven one-leg inputs produce repeatable measured output proxies that can be fitted with a local linear calibration.
6. The four CAD-derived pitch-linkage modules can run together in one fixed-base Isaac scene without immediate constraint explosion.
7. One-drive-at-a-time all-leg sweeps can produce a full-rank local calibration fit for all eight pitch drives.
8. The four shoulder hip ab/ad actuators can be added to the same fixed-base scene, giving the intended twelve actuator channels without destabilizing the closed pitch linkages.
9. The four shoulder joints can attach to one shared fixed body reference while preserving the twelve-actuator, full-rank independent sweep.

That is the missing physics ingredient between the simplified one-leg articulation and a more faithful Domino leg.

## Important Limitation

This is still not the finished Domino leg.

The CAD-derived tests use real pivot positions, but simplified rigid bodies, no mesh collisions, no gravity, and conservative drive settings. They prove the isolated one-joint loops, a simplified combined leg, a fixed-base all-leg pitch-linkage scene, a fixed-base twelve-actuator command layout, and a shared fixed-body twelve-actuator layout can be constrained. The current calibration fit is based on body-angle proxy measurements, so it does **not** yet prove the full robot with a floating/resettable body, contacts, CAD mesh masses, servo gains, hard stops, policy-ready drive-to-output mapping, or training resets is stable.

## Next Work

1. Replace the body-angle proxy outputs with cleaner policy coordinates.
2. Convert the shared fixed-body twelve-actuator scene into a clean Isaac Lab robot with a resettable base.
3. Reintroduce gravity, simple collisions, and joint hard-stop checks.
4. Wire the result to the twelve-servo Isaac Lab action space.
5. Start with a reset/sweep task before policy rewards are added.
