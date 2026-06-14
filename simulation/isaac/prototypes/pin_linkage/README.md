# Domino Pin-Linkage Prototype

This prototype is the next physics gate after the simplified one-leg articulation.

The goal is to prove that a small closed linkage can run in Isaac/PhysX with:

- One actuated revolute input pin.
- Passive revolute pins for the other linkage members.
- A loop-closing revolute pin.
- No contacts or gravity in the first pass.

This is intentionally smaller than the full Domino leg. It started as a controlled generic four-bar test, then gained CAD-derived one-joint Domino linkage modes.

## Runtime Test

Run the generic linkage with the Isaac Lab Python environment:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry generic-four-bar `
  --steps 240 `
  --report-path <output-folder>/pin_linkage_report.json
```

Run the CAD-derived Domino lower-linkage loop:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-lower-triangle `
  --steps 600 `
  --drive-amplitude-deg 8 `
  --drive-frequency-hz 0.4 `
  --report-path <output-folder>/domino_lower_triangle_report.json `
  --save-usd <output-folder>/domino_lower_triangle.usd
```

Run the CAD-derived Domino upper linkage loop:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-upper-loop `
  --steps 600 `
  --drive-amplitude-deg 5 `
  --drive-frequency-hz 0.4 `
  --report-path <output-folder>/domino_upper_loop_report.json `
  --save-usd <output-folder>/domino_upper_loop.usd
```

The script authors the linkage directly into the current Isaac stage, applies a sinusoidal target to the driven input joint, steps physics, and reports body state plus loop-closure drift.

Runtime status: the generic linkage, the CAD-derived lower triangle, and the CAD-derived upper loop have all passed headless Isaac/PhysX runs. See [`../../reports/domino-pin-linkage-runtime.md`](../../reports/domino-pin-linkage-runtime.md).

## CAD-Derived Modes

The `domino-lower-triangle` and `domino-upper-loop` modes use pivots extracted by [`../../analyze-domino-linkage-pivots.ps1`](../../analyze-domino-linkage-pivots.ps1).

Lower triangle:

| Role | URDF joint |
| --- | --- |
| Driven input | `Revolute 59` |
| Passive stack pin | `Revolute 43` |
| Passive coupler pin | `Revolute 33` |
| Loop closure | `Revolute 25` / `Revolute 26` |

Upper loop:

| Role | URDF joint |
| --- | --- |
| Held lower input | `Revolute 59` / `Revolute 43` |
| Driven input | `Revolute 58` |
| Passive coupler pin | `Revolute 32` |
| Loop closure | `Revolute 51` |

## What Passing Means

Passing means the isolated one-actuator passive-pin loops can run without non-finite state or obvious constraint explosion. It does **not** mean the Domino lower leg is finished. The next step is to combine both validated loops into a one-leg mechanism, compare the constrained output motion against the simplified one-leg model, and only then merge it into a full one-leg Domino asset.
