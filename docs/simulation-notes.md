# Simulation Notes

Domino includes exploratory CAD-to-simulation work for Isaac Sim. These files are useful for visual inspection and workflow experimentation, but they are not a finished physics model.

## Included Exports

The repo includes the main CAD and simulation exports:

```text
cad/step/
simulation/isaac/
simulation/urdf/generated/
simulation/usd/
```

USD-family exports:

- [`simulation/usd/Domino_Isaac_SIM.usd`](../simulation/usd/Domino_Isaac_SIM.usd)
- [`simulation/usd/Domino_Quadruped.usd`](../simulation/usd/Domino_Quadruped.usd)
- [`simulation/usd/Domino_USD_Parts_Combined_Final.usdz`](../simulation/usd/Domino_USD_Parts_Combined_Final.usdz)

Isaac bring-up notes and topology tooling:

- [`simulation/isaac/README.md`](../simulation/isaac/README.md)
- [`simulation/isaac/analyze-domino-urdf.ps1`](../simulation/isaac/analyze-domino-urdf.ps1)
- [`simulation/isaac/run-domino-urdf-import.ps1`](../simulation/isaac/run-domino-urdf-import.ps1)
- [`simulation/isaac/prototypes/one_leg`](../simulation/isaac/prototypes/one_leg)
- [`simulation/isaac/prototypes/pin_linkage`](../simulation/isaac/prototypes/pin_linkage)
- [`simulation/isaac/reports/domino-urdf-topology.md`](../simulation/isaac/reports/domino-urdf-topology.md)
- [`simulation/isaac/reports/domino-urdf-import-smoke-test.md`](../simulation/isaac/reports/domino-urdf-import-smoke-test.md)

## Main Limitation

The real leg design behaves like a closed-chain/four-bar linkage. That is mechanically useful, but it is awkward for many robotics simulation workflows.

URDF-style robot descriptions usually represent a tree of links and joints. A closed loop must be approximated, constrained separately, or rewritten into an equivalent controllable model.

The current generated URDF is also not a clean tree export. The topology report shows duplicate link names and child links with multiple incoming joints, so a direct importer may either fail or build a model that does not match the physical linkage.

A raw Isaac Lab import smoke test has now completed successfully for USD generation. That is a useful checkpoint: the URDF package and meshes can be consumed by Isaac tooling. It is still not a physics validation, and the generated asset should not be used as the final training model without rewriting the articulation structure.

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

For training, the first Isaac Lab model should expose the twelve real driven joints as the policy action space. Passive linkage pins should either be visual-only or added later as passive constraints once the simplified model can stand, reset, and sweep joints without instability.

A first simplified one-leg URDF and Isaac Lab config template now live under [`simulation/isaac/prototypes/one_leg`](../simulation/isaac/prototypes/one_leg). This is the intended starting point for validating axes, limits, reset behavior, and actuator gains before rebuilding the complete quadruped.

That simplified one-leg model now imports and completes a headless Isaac Lab joint sweep. The next unresolved simulation problem is the passive two-bar/four-bar linkage physics: add passive revolute pins and a loop-closing constraint only after the driven model stays stable.

A generic one-actuator four-bar linkage prototype has now passed in Isaac/PhysX with passive pins and a loop-closing revolute constraint. The next step is to replace that generic linkage with Domino's actual CAD pivot positions and compare it against the simplified effective lower-linkage joint.
