# Domino One-Leg Import Smoke Test

This report records the first Isaac Lab import test for the simplified one-leg prototype under `simulation/isaac/prototypes/one_leg`.

## Result

Status: **passed for clean one-leg USD generation**.

The prototype URDF imported successfully through Isaac Lab's `convert_urdf.py` workflow and wrote the expected root USD plus generated configuration layers.

Unlike the raw CAD-generated full robot URDF, this prototype did not trigger joint-name sanitation warnings. The three driven joints already use readable names:

- `hip_ab_ad`
- `upper_pitch`
- `lower_linkage`

## What This Confirms

- The simplified one-leg model is valid enough for Isaac's URDF importer.
- The model has a clean tree structure with one fixed foot joint and three driven revolute joints.
- The prototype is a better starting point for control bring-up than the direct CAD URDF export.

## What This Does Not Confirm Yet

- Joint axes have not yet been visually confirmed against the physical Domino leg.
- Contact behavior and collision quality have not been tested.
- The actuator gains in `domino_one_leg_cfg.py` are first-pass placeholders.
- The passive four-bar linkage is not physically constrained yet.

## Next Test

The next runtime test should spawn the generated USD in Isaac Lab, reset the articulation, and sweep each driven joint one at a time through conservative limits.

That test should answer three practical questions:

1. Does each joint rotate around the expected axis?
2. Do positive and negative commands match the firmware convention?
3. Does the simplified collision model remain stable under gravity and reset?
