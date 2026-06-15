# Domino Clean Quadruped Prototype

This folder contains the first clean all-leg Domino training-articulation prototype.

The raw CAD URDF is still kept as reference, but it is not used directly for training because it contains duplicate link names and closed linkage loops. This prototype takes the CAD-derived actuator layout and emits a simple tree articulation that Isaac Lab can import:

- Four shoulder hip ab/ad joints.
- Four lower linkage command joints.
- Four upper pitch command joints.
- CAD-derived hip locations, shoulder axis signs, and joint limits.

The passive two-bar/four-bar linkage is not physically closed in this asset yet. That detail is currently validated in the pin-linkage prototype and should be added back as a separate fixed-base constraint gate before attempting a high-fidelity floating robot.

## Generate URDF

```powershell
<python> simulation/isaac/prototypes/quadruped/generate_quadruped_urdf.py
```

## Import

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-domino-urdf-import.ps1 `
  -UrdfPath simulation/isaac/prototypes/quadruped/domino_quadruped_clean.urdf `
  -IsaacLabRoot <path-to-IsaacLab> `
  -IsaacPython <path-to-isaac-python> `
  -OutputUsd <output-folder>/domino_quadruped_clean.usd `
  -FixBase `
  -NoMergeJoints `
  -AcceptEula
```

## Runtime Sweep

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_quadruped_sweep.py `
  --usd-path <output-folder>/domino_quadruped_clean.usd `
  --headless `
  --steps 600 `
  --report-path <output-folder>/domino_quadruped_sweep_report.json
```

This sweep is a fixed-base articulation check. Passing means the imported USD exposes the full twelve-action Domino order and can accept conservative joint-position targets without non-finite state or joint-limit violations. It is not the final policy-training environment yet.

Runtime status: this prototype has passed a fixed-base Isaac Lab articulation sweep. See [`../../reports/domino-quadruped-runtime-sweep.md`](../../reports/domino-quadruped-runtime-sweep.md).
