# Simulation Notes

Domino includes exploratory CAD-to-simulation work for Isaac Sim. These files are useful for visual inspection and workflow experimentation, but they are not a finished physics model.

## Included Exports

The repo includes the main CAD and simulation exports:

```text
cad/step/
simulation/usd/
```

USD-family exports:

- [`simulation/usd/Domino_Isaac_SIM.usd`](../simulation/usd/Domino_Isaac_SIM.usd)
- [`simulation/usd/Domino_Quadruped.usd`](../simulation/usd/Domino_Quadruped.usd)
- [`simulation/usd/Domino_USD_Parts_Combined_Final.usdz`](../simulation/usd/Domino_USD_Parts_Combined_Final.usdz)

## Main Limitation

The real leg design behaves like a closed-chain/four-bar linkage. That is mechanically useful, but it is awkward for many robotics simulation workflows.

URDF-style robot descriptions usually represent a tree of links and joints. A closed loop must be approximated, constrained separately, or rewritten into an equivalent controllable model.

A direct CAD export can therefore look correct while still failing to behave like the physical robot:

- Visual meshes may import correctly.
- Parts may appear as separate bodies.
- Joint hierarchy may not match the actual linkage.
- Closed-chain constraints may be missing.
- Servo actuation still needs to be mapped to the simulated joint model.

## Recommended Next Step

The next useful simulation milestone is not a full robot import. It is one validated leg:

1. Build a simplified one-leg model.
2. Match the firmware IK assumptions.
3. Validate joint axes, joint limits, and scale.
4. Add visual linkage geometry after the control model works.
5. Expand to four legs only after the one-leg model is controllable.

That path is less literal than importing every CAD mate, but it is more likely to produce a useful simulation.
