# Domino One-Leg Isaac Prototype

This folder contains a deliberately simplified one-leg model for Isaac Sim / Isaac Lab bring-up.

The aim is not to preserve every CAD mate. The aim is to create a clean, controllable articulation that matches the firmware abstraction:

- `hip_ab_ad`: hip abduction/adduction.
- `upper_pitch`: upper leg pitch.
- `lower_linkage`: effective lower-leg / knee linkage command.

The passive four-bar linkage is represented as simplified visual geometry only. Loop-closing constraints should be added later, after this driven model imports, resets, and sweeps through its limits without instability.

## Files

| File | Purpose |
| --- | --- |
| `domino_one_leg_clean.urdf` | Minimal three-joint URDF with primitive geometry, simple inertials, joint limits, and no mesh dependencies. |
| `domino_one_leg_cfg.py` | Isaac Lab `ArticulationCfg` template for the imported one-leg USD. |

## Geometry Basis

The prototype uses the current firmware geometry constants:

| Quantity | Value |
| --- | ---: |
| Upper virtual link length | `0.160 m` |
| Lower virtual link length | `0.153 m` |
| Hip lateral offset | `0.038 m` |
| Hip vertical offset | `0.021 m` |

The firmware leg frame uses +Z downward. Isaac scenes normally use +Z upward, so this prototype places the foot below the hip along negative Z in the URDF.

## Import

From the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-domino-urdf-import.ps1 `
  -UrdfPath simulation/isaac/prototypes/one_leg/domino_one_leg_clean.urdf `
  -IsaacLabRoot <path-to-IsaacLab> `
  -IsaacPython <path-to-isaac-python> `
  -OutputUsd <output-folder>/domino_one_leg_clean.usd `
  -AcceptEula
```

Then point `DOMINO_ONE_LEG_USD` at the generated USD before using `domino_one_leg_cfg.py` inside an Isaac Lab script.

Import status: this prototype has passed a first Isaac Lab URDF-to-USD smoke test. See [`../../reports/domino-one-leg-import-smoke-test.md`](../../reports/domino-one-leg-import-smoke-test.md).

## Next Checks

1. Import the URDF to USD.
2. Spawn the USD in Isaac Lab with `domino_one_leg_cfg.py`.
3. Sweep `hip_ab_ad`, `upper_pitch`, and `lower_linkage` through conservative ranges.
4. Confirm axes move in the same sense as the firmware commands.
5. Add visual linkage detail or passive constraints only after the three driven joints are stable.
