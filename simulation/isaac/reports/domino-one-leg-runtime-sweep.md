# Domino One-Leg Runtime Sweep

This report records the first Isaac Lab runtime articulation test for the simplified Domino one-leg prototype.

## Result

Status: **passed for simplified one-leg articulation runtime sweep**.

The fixed-base one-leg USD was spawned as an Isaac Lab `Articulation`, the expected driven joints were found, and joint-position targets were applied while stepping physics headlessly.

Validated joints:

- `hip_ab_ad`
- `upper_pitch`
- `lower_linkage`

Local test summary:

| Metric | Result |
| --- | ---: |
| Physics steps | `100` |
| Physics dt | `0.005 s` |
| Root position after sweep | `(0.0, 0.0, 0.35) m` |
| Max joint speed | `4.027763 rad/s` |
| Max tracking error | `0.504844 rad` |

The test did not produce non-finite joint, velocity, or root state, and the patched sweep script exited cleanly.

## Important Limitation

This does **not** mean the real two-bar / four-bar linkage has been solved in Isaac yet.

The current one-leg prototype is intentionally simplified:

- Three driven joints match the firmware abstraction.
- The body mount is fixed for joint-axis validation.
- Primitive geometry is used instead of CAD meshes.
- Passive linkage pins and loop-closing constraints are not physically modeled yet.

That simplification is deliberate. It gives a stable control model before adding closed-chain constraints, which are the part most likely to cause instability.

## Next Physics Gate

The next test should create a one-leg linkage variant with:

1. The same three driven joints.
2. Passive revolute pins for the lower linkage members.
3. One loop-closing constraint added after the driven model is stable.
4. Low solver gains and conservative limits.
5. A sweep report comparing the constrained linkage to the simplified effective-joint model.

If the closed-chain linkage fights the driven joint or becomes unstable, keep the policy model simplified and use the constrained linkage model for validation rather than training.
