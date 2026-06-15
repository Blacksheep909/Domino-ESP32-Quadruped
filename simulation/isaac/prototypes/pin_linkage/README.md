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

The script authors the linkage directly into the current Isaac stage, applies sinusoidal targets to the driven input joints, steps physics, and reports body state, loop-closure drift, drive target ranges, body pitch ranges, relative linkage angles, and tracked pivot motion.

Runtime status: the generic linkage, the CAD-derived lower triangle, the CAD-derived upper loop, and the combined CAD-derived one-leg mechanism have all passed headless Isaac/PhysX runs. See [`../../reports/domino-pin-linkage-runtime.md`](../../reports/domino-pin-linkage-runtime.md).

Motion-characterization status: the combined mechanism is stable, but its lower drive target is not yet calibrated to the effective body/output angle. See [`../../reports/domino-combined-linkage-characterization.md`](../../reports/domino-combined-linkage-characterization.md).

Run the combined CAD-derived one-leg mechanism:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-combined-leg `
  --steps 600 `
  --drive-amplitude-deg 2 `
  --secondary-drive-amplitude-deg 2 `
  --drive-frequency-hz 0.2 `
  --secondary-drive-frequency-hz 0.2 `
  --report-path <output-folder>/domino_combined_leg_report.json `
  --save-usd <output-folder>/domino_combined_leg.usd
```

## CAD-Derived Modes

The `domino-lower-triangle`, `domino-upper-loop`, and `domino-combined-leg` modes use pivots extracted by [`../../analyze-domino-linkage-pivots.ps1`](../../analyze-domino-linkage-pivots.ps1).

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

Combined leg:

| Role | URDF joint |
| --- | --- |
| Lower driven input | `Revolute 59` |
| Upper driven input | `Revolute 58` |
| Shared coupler | `Revolute 43`, `Revolute 33`, `Revolute 32` |
| Lower loop closure | `Revolute 25` / `Revolute 26` |
| Upper loop closure | `Revolute 32` / `Revolute 51` direct closure |

## What Passing Means

Passing means the isolated one-actuator passive-pin loops and a simplified two-drive combined leg can run without non-finite state or obvious constraint explosion. It does **not** mean the Domino lower leg is finished. The next step is to calibrate rest offsets and effective output-angle mapping against the simplified one-leg model, then merge the combined linkage behavior into a full one-leg Domino asset.
