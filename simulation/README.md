# Simulation Exports

This folder contains USD-family exports from the Domino CAD and Isaac Sim experiment trail.

> Work in progress: these files are included as reference exports, not as a finished physics simulation. The visual geometry is useful, but the closed-chain/four-bar leg mechanism still needs a deliberate joint and constraint model before it can behave like the real robot in simulation.

## Files

| Path | Purpose |
| --- | --- |
| `usd/Domino_Isaac_SIM.usd` | Lightweight Isaac Sim-oriented USD scene/export. |
| `usd/Domino_Quadruped.usd` | Domino quadruped USD export. |
| `usd/Domino_USD_Parts_Combined_Final.usdz` | Packaged combined USDZ export from the CAD-to-simulation trail. |

## Notes

- Use these files for visual inspection, scale checks, and Isaac Sim import experiments.
- Do not assume the imported model has correct physics, inertia, joint axes, or servo actuation.
- For controllable simulation, start from one simplified leg and validate the joint model against the CAD before scaling up to the full robot.

More context is in [../docs/simulation-notes.md](../docs/simulation-notes.md).
