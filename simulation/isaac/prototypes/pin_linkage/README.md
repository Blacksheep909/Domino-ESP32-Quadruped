# Domino Pin-Linkage Prototype

This prototype is the next physics gate after the simplified one-leg articulation.

The goal is to prove that a small closed linkage can run in Isaac/PhysX with:

- One actuated revolute input pin.
- Passive revolute pins for the other linkage members.
- A loop-closing revolute pin.
- No contacts or gravity in the first pass.

This started as a controlled generic four-bar test, then gained CAD-derived one-joint, one-leg, all-leg pitch-linkage, fixed-base twelve-actuator, shared-body twelve-actuator, and floating shared-body Domino linkage modes.

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

Runtime status: the generic linkage, the CAD-derived lower triangle, the CAD-derived upper loop, the combined CAD-derived one-leg mechanism, a four-leg CAD-derived pitch-linkage scene, a fixed-base twelve-actuator scene, a shared fixed-body twelve-actuator scene, and a floating shared-body twelve-actuator contact scene have all passed headless Isaac/PhysX runs. See [`../../reports/domino-pin-linkage-runtime.md`](../../reports/domino-pin-linkage-runtime.md), [`../../reports/domino-four-leg-linkage-runtime.md`](../../reports/domino-four-leg-linkage-runtime.md), [`../../reports/domino-12-actuator-runtime.md`](../../reports/domino-12-actuator-runtime.md), and [`../../reports/domino-floating-cad-linkage-contact.md`](../../reports/domino-floating-cad-linkage-contact.md).

Motion-characterization status: the combined one-leg mechanism is stable and has a first local linear calibration fit from drive targets to measured linkage-output proxies. The all-leg pitch scene has an independent one-drive-at-a-time calibration sweep that gives a full-rank local fit for all eight pitch drives. The twelve-actuator scenes add the four shoulder hip ab/ad drives and give full-rank local fits across all twelve actuator inputs. The current strongest gate is `domino-four-12-fixed-body`, where all four shoulder joints attach to one shared kinematic body reference. These fits are useful engineering data, but they are not yet the final policy action/state mapping. See [`../../reports/domino-combined-linkage-characterization.md`](../../reports/domino-combined-linkage-characterization.md), [`../../reports/domino-four-leg-linkage-runtime.md`](../../reports/domino-four-leg-linkage-runtime.md), and [`../../reports/domino-12-actuator-runtime.md`](../../reports/domino-12-actuator-runtime.md).

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

Run the all-leg independent drive calibration sweep:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-combined-legs `
  --drive-schedule independent `
  --steps 3200 `
  --independent-segment-steps 400 `
  --independent-settle-steps 80 `
  --fit-start-step 0 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.15 `
  --secondary-drive-frequency-hz 0.15 `
  --report-path <output-folder>/domino_four_independent_calibration_report.json `
  --save-usd <output-folder>/domino_four_independent_calibration.usd
```

Run the fixed-base twelve-actuator scene:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-actuators `
  --steps 600 `
  --fit-start-step 60 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --shoulder-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.15 `
  --secondary-drive-frequency-hz 0.15 `
  --shoulder-drive-frequency-hz 0.15 `
  --report-path <output-folder>/domino_four_12_actuators_report.json `
  --no-print-report
```

Run the fixed-base twelve-actuator independent sweep:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-actuators `
  --drive-schedule independent `
  --steps 2400 `
  --independent-segment-steps 200 `
  --independent-settle-steps 40 `
  --fit-start-step 0 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --shoulder-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.25 `
  --secondary-drive-frequency-hz 0.25 `
  --shoulder-drive-frequency-hz 0.25 `
  --report-path <output-folder>/domino_four_12_actuators_independent_report.json `
  --no-print-report
```

Run the shared-body fixed-base twelve-actuator independent sweep:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-fixed-body `
  --drive-schedule independent `
  --steps 1200 `
  --independent-segment-steps 100 `
  --independent-settle-steps 20 `
  --fit-start-step 0 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --shoulder-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.5 `
  --secondary-drive-frequency-hz 0.5 `
  --shoulder-drive-frequency-hz 0.5 `
  --report-path <output-folder>/domino_four_12_fixed_body_independent_report.json `
  --no-print-report
```

Run the floating shared-body twelve-actuator contact smoke:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-floating-body `
  --steps 300 `
  --drive-amplitude-deg 0 `
  --secondary-drive-amplitude-deg 0 `
  --shoulder-drive-amplitude-deg 0 `
  --drive-frequency-hz 0.5 `
  --secondary-drive-frequency-hz 0.5 `
  --shoulder-drive-frequency-hz 0.5 `
  --max-loop-closure-error-m 0.005 `
  --min-floating-root-height-m 0.02 `
  --report-path <output-folder>/domino_four_12_floating_body_report.json `
  --save-usd <output-folder>/domino_four_12_floating_body.usd `
  --no-print-report
```

## CAD-Derived Modes

The `domino-lower-triangle`, `domino-upper-loop`, `domino-combined-leg`, `domino-four-combined-legs`, `domino-four-12-actuators`, `domino-four-12-fixed-body`, and `domino-four-12-floating-body` modes use pivots extracted by [`../../analyze-domino-linkage-pivots.ps1`](../../analyze-domino-linkage-pivots.ps1).

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

| Leg module | Shoulder hip ab/ad | Lower driven input | Upper driven input | Lower loop closure | Upper loop closure |
| --- | --- | --- | --- | --- | --- |
| `dom_p_4_1` | `Revolute 1` | `Revolute 59` | `Revolute 58` | `Revolute 25` / `Revolute 26` | `Revolute 32` / `Revolute 51` |
| `dom_p_12_1` | `Revolute 2` | `Revolute 46` | `Revolute 55` | `Revolute 23` / `Revolute 24` | `Revolute 29` / `Revolute 50` |
| `dom_p_25_1` | `Revolute 3` | `Revolute 47` | `Revolute 56` | `Revolute 21` / `Revolute 22` | `Revolute 34` / `Revolute 54` |
| `dom_p_21_1` | `Revolute 4` | `Revolute 48` | `Revolute 57` | `Revolute 27` / `Revolute 28` | `Revolute 31` / `Revolute 53` |

## Drive Schedules

`--drive-schedule phased-sine` is the default smoke test. All drives move together with phase offsets, which is useful for constraint stability but not enough for independent calibration because the input matrix is rank deficient.

`--drive-schedule independent` moves one drive at a time while the other drives hold their center positions. Use this mode when fitting the local relationship between the commanded inputs and measured output proxies. In the twelve-actuator modes, one independent cycle covers all twelve real actuator channels: four shoulders, four lower linkage drives, and four upper pitch drives.

The JSON report includes a named `action_space` with the exact action index order. The script also checks generated drive targets against the modeled joint limits by default; use `--disable-drive-limit-checks` only for deliberate stress tests.

## What Passing Means

Passing means the isolated one-actuator passive-pin loops, a simplified two-drive combined leg, a fixed-base all-leg pitch-linkage scene, a fixed-base twelve-actuator scene, and a shared fixed-body twelve-actuator scene can run without non-finite state or obvious constraint explosion. The calibration fits mean the combined one-leg case, the all-leg pitch independent sweep, and the twelve-actuator independent sweeps have repeatable local relationships between commanded drive targets and measured linkage-output proxies over the tested range.

The floating shared-body smoke adds gravity, a static ground box, and four simple spherical contact proxies at the CAD lower-closure points. Passing that test means the CAD-derived linkage can hold a static supported pose under gravity while keeping the loop-closure error finite.

It does **not** mean the Domino robot is finished. The next step is to wrap or convert the floating shared-body twelve-actuator scene into a clean Isaac Lab robot with reset/clone support, hard stops, contact observations, and a twelve-action training environment.
