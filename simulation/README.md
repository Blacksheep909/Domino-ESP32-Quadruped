# Simulation Exports

This folder contains USD-family exports from the Domino CAD-to-Isaac-Sim exploration.

> Work in progress: these files are reference exports, not a finished physics simulation. They are useful for visual import tests, scale checks, and future simulation model development.

## Files

| Path | Purpose |
| --- | --- |
| `usd/Domino_Isaac_SIM.usd` | Lightweight Isaac Sim-oriented USD scene/export. |
| `usd/Domino_Quadruped.usd` | Domino quadruped USD export. |
| `usd/Domino_USD_Parts_Combined_Final.usdz` | Packaged combined USDZ export from the CAD-to-simulation trail. |

## Notes

- The visual geometry is useful.
- The joint hierarchy, physics properties, inertia, and actuation model still need dedicated simulation work.
- The closed-chain/four-bar leg linkage is the main reason a direct CAD import is not enough.

More context is in [../docs/simulation-notes.md](../docs/simulation-notes.md).
