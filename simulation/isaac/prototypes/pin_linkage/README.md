# Domino Pin-Linkage Prototype

This prototype is the next physics gate after the simplified one-leg articulation.

The goal is to prove that a small closed linkage can run in Isaac/PhysX with:

- One actuated revolute input pin.
- Passive revolute pins for the other linkage members.
- A loop-closing revolute pin.
- No contacts or gravity in the first pass.

This started as a controlled generic four-bar test, then gained CAD-derived one-joint, one-leg, and all-leg Domino linkage modes.

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

The script authors the linkage directly into the current Isaac stage, applies sinusoidal targets to the driven input joints, steps physics, and reports body state, loop-closure drift, drive target ranges, body pitch ranges, relative linkage angles, tracked pivot motion, and an optional local linear calibration fit.

Runtime status: the generic linkage, the CAD-derived lower triangle, the CAD-derived upper loop, the combined CAD-derived one-leg mechanism, and a four-leg CAD-derived pitch-linkage scene have all passed headless Isaac/PhysX runs. See [`../../reports/domino-pin-linkage-runtime.md`](../../reports/domino-pin-linkage-runtime.md) and [`../../reports/domino-four-leg-linkage-runtime.md`](../../reports/domino-four-leg-linkage-runtime.md).

Motion-characterization status: the combined mechanism is stable and now has a first local linear calibration fit from drive targets to measured linkage-output proxies. That fit is useful engineering data, but it is not yet the final policy action/state mapping. See [`../../reports/domino-combined-linkage-characterization.md`](../../reports/domino-combined-linkage-characterization.md).

Run the combined CAD-derived one-leg mechanism:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-combined-leg `
  --steps 1000 `
  --fit-start-step 60 `
  --drive-amplitude-deg 2 `
  --secondary-drive-amplitude-deg 2 `
  --drive-frequency-hz 0.2 `
  --secondary-drive-frequency-hz 0.2 `
  --report-path <output-folder>/domino_combined_leg_report.json `
  --save-usd <output-folder>/domino_combined_leg.usd
```

Run the all-leg CAD-derived pitch-linkage scene:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-combined-legs `
  --steps 600 `
  --fit-start-step 60 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.15 `
  --secondary-drive-frequency-hz 0.15 `
  --report-path <output-folder>/domino_four_combined_legs_report.json `
  --save-usd <output-folder>/domino_four_combined_legs.usd
```

## CAD-Derived Modes

The `domino-lower-triangle`, `domino-upper-loop`, `domino-combined-leg`, and `domino-four-combined-legs` modes use pivots extracted by [`../../analyze-domino-linkage-pivots.ps1`](../../analyze-domino-linkage-pivots.ps1).

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

All-leg scene:

| Leg module | Lower driven input | Upper driven input | Lower loop closure | Upper loop closure |
| --- | --- | --- | --- | --- |
| `dom_p_4_1` | `Revolute 59` | `Revolute 58` | `Revolute 25` / `Revolute 26` | `Revolute 32` / `Revolute 51` |
| `dom_p_12_1` | `Revolute 46` | `Revolute 55` | `Revolute 23` / `Revolute 24` | `Revolute 29` / `Revolute 50` |
| `dom_p_25_1` | `Revolute 47` | `Revolute 56` | `Revolute 21` / `Revolute 22` | `Revolute 34` / `Revolute 54` |
| `dom_p_21_1` | `Revolute 48` | `Revolute 57` | `Revolute 27` / `Revolute 28` | `Revolute 31` / `Revolute 53` |

## What Passing Means

Passing means the isolated one-actuator passive-pin loops, a simplified two-drive combined leg, and a fixed-base all-leg pitch-linkage scene can run without non-finite state or obvious constraint explosion. The calibration fit means the combined one-leg case has a repeatable local relationship between commanded drive targets and measured linkage-output proxies over the tested range.

It does **not** mean the Domino robot is finished. The next step is to run independent drive sweeps for the all-leg scene, compare the fitted proxy outputs against the simplified one-leg model, merge the pitch linkage behavior with hip ab/ad articulation, then build the Isaac Lab training environment.
