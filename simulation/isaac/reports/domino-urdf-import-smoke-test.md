# Domino URDF Import Smoke Test

This report records the first Isaac Sim / Isaac Lab smoke test for the generated Domino URDF export.

## Result

Status: **passed for raw USD generation, not passed as a training-ready robot model**.

The generated URDF was imported through Isaac Lab's `convert_urdf.py` tool in a local Isaac Sim 5.1 / Isaac Lab environment. The command completed and wrote a USD stage plus the related generated configuration layers.

The import is useful because it proves that the current URDF package and meshes can be consumed by the Isaac tooling. It does not mean the resulting asset is ready for reinforcement learning or accurate physics.

## Important Warnings

- Joint names with spaces were sanitized by the importer, for example `Revolute 1` became `Revolute_1`.
- The source URDF still contains duplicate link names and loop-style child reuse, as documented in [`domino-urdf-topology.md`](domino-urdf-topology.md).
- The four-bar / closed-chain leg linkage still needs a deliberate simulation model rather than a blind CAD-mate import.
- The generated smoke-test USD should be treated as a local build artifact until asset references, scale, physics settings, and joint naming have been reviewed.

## Reproduction

Use the helper script from the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-domino-urdf-import.ps1 `
  -IsaacLabRoot <path-to-IsaacLab> `
  -IsaacPython <path-to-isaac-python> `
  -OutputUsd <output-folder>/domino_raw_import.usd `
  -AcceptEula
```

The script keeps local install paths outside the repo. It also sets conservative import options for early bring-up:

- `--merge-joints`
- `--joint-stiffness 0.0`
- `--joint-damping 0.0`
- `--joint-target-type none`
- `--headless`

These settings produce a neutral imported asset so drives and actuator models can be configured intentionally in Isaac Lab later.

## Next Work

1. Build a clean one-leg USD articulation with readable joint names and simplified collision.
2. Validate joint axes, limits, scale, and reset behavior in Isaac.
3. Add passive linkage geometry as visual-only first.
4. Add loop-closing constraints only after the simplified driven model is stable.
5. Duplicate the validated leg into the full quadruped and expose the twelve real driven joints to Isaac Lab.
