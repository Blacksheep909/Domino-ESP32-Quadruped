# Domino Clean Quadruped Runtime Sweep

This report records the first Isaac Lab runtime articulation test for the clean all-leg Domino quadruped prototype.

## Result

Status: **passed for fixed-base 12-DoF quadruped articulation runtime sweep**.

The generated clean quadruped URDF was imported to USD, spawned as an Isaac Lab `Articulation`, and swept through conservative joint-position targets. The sweep validated the same twelve-action order used by the CAD-derived linkage tests:

- Four shoulder hip ab/ad actions.
- Four lower linkage command actions.
- Four upper pitch command actions.

That is one shoulder actuator plus two pitch/linkage-drive actuators per leg, for 12 total policy channels.

Local test summary:

| Metric | Result |
| --- | ---: |
| Physics steps | `600` |
| Physics dt | `0.005 s` |
| Action count | `12` |
| First action | `dom_p_4_1_shoulder_ab_ad` |
| Last action | `dom_p_21_1_upper_pitch` |
| Root position after sweep | `(0.0, 0.0, 0.38) m` |
| Max joint speed | `3.209306 rad/s` |
| Max tracking error | `0.441188 rad` |
| Max joint-limit violation | `0.0 rad` |

The test did not produce non-finite joint, velocity, or root state.

## What This Proves

This is the first all-leg Isaac Lab articulation gate. The repo now has:

- A generated clean 12-DoF quadruped URDF.
- An Isaac Lab `ArticulationCfg` for the imported USD.
- A repeatable headless sweep that verifies the twelve expected action joints are present and controllable.

The asset is fixed-base for this test. That is deliberate: it validates the action layout and imported joint model before introducing gravity, contacts, resets, and policy training.

## Important Limitation

This does **not** mean the final Domino simulation is complete.

The current clean quadruped is a tree articulation for training bring-up. It preserves the CAD-derived actuator names, hip locations, shoulder axis signs, and joint limits, but it does not physically close the passive two-bar/four-bar linkages. The closed linkage physics is currently validated in the pin-linkage prototype, not in the clean training articulation.

## Next Physics Gate

The next test should make the clean quadruped usable for policy work:

1. Add a floating or resettable base validation.
2. Add a ground plane and simple contact bodies without using the broken ground-plane helper path seen in this local Isaac install.
3. Verify foot body names and contact sensor paths.
4. Sweep all twelve actions under gravity with conservative gains.
5. Only then wrap the asset in a DirectRLEnv stand/height task.
