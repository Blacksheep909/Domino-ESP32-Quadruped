# Domino Pin-Linkage Prototype

This prototype is the next physics gate after the simplified one-leg articulation.

The goal is to prove that a small closed linkage can run in Isaac/PhysX with:

- One actuated revolute input pin.
- Passive revolute pins for the other linkage members.
- A loop-closing revolute pin.
- No contacts or gravity in the first pass.

This is intentionally smaller than the full Domino leg. It started as a controlled generic four-bar test, then gained a CAD-derived one-joint Domino lower-linkage mode.

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

The script authors the linkage directly into the current Isaac stage, applies a sinusoidal target to the driven input joint, steps physics, and reports body state plus loop-closure drift.

Runtime status: the generic linkage and the first CAD-derived Domino lower-linkage loop have both passed headless Isaac/PhysX runs. See [`../../reports/domino-pin-linkage-runtime.md`](../../reports/domino-pin-linkage-runtime.md).

## CAD-Derived Mode

The `domino-lower-triangle` mode uses pivots extracted by [`../../analyze-domino-linkage-pivots.ps1`](../../analyze-domino-linkage-pivots.ps1). The first validated loop is:

| Role | URDF joint |
| --- | --- |
| Driven input | `Revolute 59` |
| Passive stack pin | `Revolute 43` |
| Passive coupler pin | `Revolute 33` |
| Loop closure | `Revolute 25` / `Revolute 26` |

## What Passing Means

Passing means the one-actuator passive-pin concept can run without non-finite state or obvious constraint explosion. It does **not** mean the Domino lower leg is finished. The next step is to add the second linkage loop, compare the constrained output motion against the simplified one-leg model, and only then merge it into a full one-leg Domino asset.
