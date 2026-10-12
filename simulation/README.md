# Simulation Exports

This folder contains USD-family exports from the Domino CAD-to-Isaac-Sim exploration.

> Work in progress: these files are reference exports, not a finished physics simulation. They are useful for visual import tests, scale checks, and future simulation model development.

## Files

| Path | Purpose |
| --- | --- |
| `isaac/` | Isaac Sim / Isaac Lab bring-up notes, URDF topology report, and validation tooling. |
| `standalone/` | Domino Virtual Lab: interactive 3D firmware and CAD testing environment. |
| `urdf/generated/` | Generated URDF package from the CAD export. This is reference material, not a final training articulation. |
| `usd/Domino_Isaac_SIM.usd` | Lightweight Isaac Sim-oriented USD scene/export. |
| `usd/Domino_Quadruped.usd` | Domino quadruped USD export. |
| `usd/Domino_USD_Parts_Combined_Final.usdz` | Packaged combined USDZ export from the CAD-to-simulation trail. |

## Notes

- The visual geometry is useful.
- The generated URDF currently contains duplicate link names and loop-style child reuse, so it should not be treated as a finished Isaac articulation.
- A raw Isaac Lab URDF import smoke test has passed, but only as an import/build-artifact check. It is not a validated robot physics model.
- A clean one-leg prototype is included under `isaac/prototypes/one_leg` for the first controlled articulation tests.
- The joint hierarchy, physics properties, inertia, collisions, and actuation model still need dedicated simulation work.
- The closed-chain/four-bar leg linkage is the main reason a direct CAD import is not enough.

More context is in [../docs/simulation-notes.md](../docs/simulation-notes.md).
