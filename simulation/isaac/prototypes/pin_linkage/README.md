# Domino Pin-Linkage Prototype

This prototype is the next physics gate after the simplified one-leg articulation.

The goal is to prove that a small closed linkage can run in Isaac/PhysX with:

- One actuated revolute input pin.
- Passive revolute pins for the other linkage members.
- A loop-closing revolute pin.
- No contacts or gravity in the first pass.

This started as a controlled generic four-bar test, then gained CAD-derived one-joint, one-leg, all-leg pitch-linkage, fixed-base twelve-actuator, shared-body twelve-actuator, and floating shared-body Domino linkage modes.

Important visual-fidelity note: the standalone `run_pin_linkage.py` modes are CAD-derived physics proxies. The pivots and actuator layout come from the Domino CAD/URDF export, but those standalone visuals are simple cubes and foot spheres.

The CAD-linkage builder used by the standalone policy-search runner and `DirectRLEnv` path now attaches the exported Domino STL link meshes to the moving proxy rigid bodies by default. In that path, the physics remains a controllable proxy linkage, but the visible robot is real Domino CAD geometry from the URDF mesh export. Use `--disable-actual-cad-visuals` only for proxy debugging, and pair it with `--allow-proxy-visuals` for non-headless runs.

If the viewport shows several dark blocky robots, that is a multi-env/debug view and is not the CAD identity check. Use the single-robot actual-CAD audit and preview output for visual verification.

Visible standalone proxy runs are blocked by default. Pass `--headless` for normal standalone physics gates. Pass `--allow-proxy-visuals` only when deliberately debugging the simplified cube/sphere linkage.

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

Runtime status: the current passive finite-closure model starts from a captured 29-body bent neutral pose instead of asking PhysX to assemble the loops under gravity. A fixed-base sweep has exercised all 12 drives independently; every channel moves the corresponding real CAD foot and the smallest measured foot displacement is about `29.5 mm`. A settled free-body sweep confirms that all 12 channels move under load in both directions without a termination, and a ten-second zero-action hold completes without a reset or detached leg. Stable weight-transfer commands are now verified for all four legs in one continuous no-reset cycle. A replay-trained BC policy reproduces that cycle while unloading every actual-CAD foot. It remains a weight-transfer policy rather than a walking policy. See [`../../README.md`](../../README.md) for the current measured gates and limitations.

The earlier split-closure and servo-slew milestone is retained in [`../../reports/domino-split-linkage-servo-slew-status.md`](../../reports/domino-split-linkage-servo-slew-status.md) for engineering traceability; it is not the current model description.

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

Run the floating shared-body policy-style action/reset smoke:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-floating-body `
  --drive-schedule policy-step `
  --steps 360 `
  --policy-action-scale-deg 0.25 `
  --policy-hold-steps 20 `
  --reset-interval-steps 120 `
  --max-post-reset-position-error-m 0.001 `
  --max-loop-closure-error-m 0.005 `
  --min-floating-root-height-m 0.02 `
  --report-path <output-folder>/domino_four_12_floating_policy_step_reset_report.json `
  --no-print-report
```

Run the floating CAD-linkage `DirectRLEnv` smoke:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_domino_cad_linkage_env_smoke.py `
  --headless `
  --num-envs 4 `
  --steps 120 `
  --command-x-m-s 0.03 `
  --gait-frequency-hz 1.0 `
  --report-path <output-folder>/domino_cad_linkage_env_smoke_4env_command_report.json
```

By default this smoke attaches actual Domino CAD STL visuals to the simulated linkage. To deliberately render the simplified proxy instead, add `--disable-actual-cad-visuals --allow-proxy-visuals`.

Audit the runtime visual scene without launching the renderer:

```powershell
$env:ISAAC_SIM_ROOT = "<isaac-sim-root>" # ISAAC_PATH from Isaac's python wrapper also works
<isaac-python> simulation/isaac/prototypes/pin_linkage/audit_domino_cad_linkage_visuals.py `
  --output-usd <output-folder>/domino_actual_cad_linkage_visual_audit.usda `
  --json-report <output-folder>/domino_actual_cad_linkage_visual_audit.json
```

The audit fails if the source CAD USD contains proxy cubes/spheres, if the runtime scene is missing Domino STL mesh parts, if the visible STL triangle total is not the known Domino export total of `135508`, or if the robot proxy cubes/spheres are visible.

Run a short command-aware RSL-RL training gate against the CAD-linkage env:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_domino_cad_linkage_rsl_rl_train.py `
  --headless `
  --num-envs 4 `
  --iterations 10 `
  --num-steps-per-env 16 `
  --command-x-m-s 0.03 `
  --gait-frequency-hz 1.0 `
  --log-root <output-folder>/domino_rsl_rl `
  --report-path <output-folder>/domino_cad_linkage_rsl_rl_train_4env_command_10iter_report.json
```

Run a locomotion reward/foot-lift smoke with larger clamped drive authority:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_domino_cad_linkage_env_smoke.py `
  --headless `
  --num-envs 4 `
  --steps 120 `
  --action-amplitude 0.5 `
  --command-x-m-s 0.08 `
  --action-scale-deg 30.0 `
  --command-progress-reward-scale 20.0 `
  --command-velocity-tracking-reward-scale 4.0 `
  --gait-contact-reward-scale 1.0 `
  --stance-contact-reward-scale 0.5 `
  --swing-contact-penalty-scale -1.5 `
  --foot-clearance-reward-scale 1.0 `
  --foot-contact-reward-scale 0.0 `
  --episode-length-s 6.0 `
  --report-path <output-folder>/domino_cad_linkage_env_smoke_4env_scale30_foot_height_report.json
```

Search scripted 12-actuator reference gaits:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_domino_cad_linkage_gait_search.py `
  --headless `
  --candidate-count 16 `
  --steps 240 `
  --seed 7 `
  --action-scale-deg 30.0 `
  --command-x-m-s 0.08 `
  --gait-frequency-hz 1.0 `
  --episode-length-s 6.0 `
  --save-best-candidate <output-folder>/domino_cad_linkage_best_gait_seed7.json `
  --report-path <output-folder>/domino_cad_linkage_gait_search_seed7_16_report.json
```

The gait-search action rows are built as three commands per leg in the shared order: shoulder hip ab/ad, lower linkage drive, and upper linkage/pitch drive. That is 4 shoulder actuators plus 8 linkage-drive actuators, for 12 total. The runner validates the action tensor shape before stepping the environment and records the exact action names in its JSON report.

> Historical result: the original scripted search produced a `36.5 mm` forward-displacement candidate and a lower-drift behavior-cloning baseline. Those runs predate the current calibrated-neutral passive-linkage model and are retained for comparison only; they are not the current locomotion baseline.

Verify, build, and inspect the current four-leg weight-transfer teacher:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_domino_cad_linkage_weight_transfer_cycle.py `
  --headless `
  --leg-order 1,3,0,2 `
  --report-path simulation/isaac/out/cad_identity/next_policy/weight_transfer_context_verified_four_foot_cycle_v2.json `
  --no-print-report

<python> simulation/isaac/prototypes/pin_linkage/build_domino_weight_transfer_teacher.py

<isaac-python> simulation/isaac/prototypes/pin_linkage/run_domino_cad_linkage_rsl_rl_play.py `
  --policy-mode reference `
  --num-envs 1 `
  --steps 1360 `
  --startup-zero-steps 120 `
  --reference-gait-candidate simulation/isaac/config/domino_weight_transfer_cycle_teacher.json `
  --include-reference-actions-in-observation `
  --action-scale-deg 8 `
  --servo-target-rate-limit-deg-s 90 `
  --episode-length-s 40 `
  --foot-collision-mode actual-cad-visual-bottom `
  --closure-model passive `
  --terrain-type flat `
  --report-path <output-folder>/domino_weight_transfer_reference_playback.json
```

The teacher contains 1,360 keyframed 12-channel actions in the verified front-left, rear-right, front-right, rear-left order. The environment maps the matching leg to swing state during each segment and keeps the other three in stance. This sequence is the mechanics reference used for replay BC; it is not itself a learned policy.

Play back the newest CAD-linkage checkpoint:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_domino_cad_linkage_rsl_rl_play.py `
  --headless `
  --num-envs 4 `
  --steps 240 `
  --command-x-m-s 0.03 `
  --gait-frequency-hz 1.0 `
  --log-root <output-folder>/domino_rsl_rl `
  --report-path <output-folder>/domino_cad_linkage_rsl_rl_play_4env_command_10iter_report.json
```

Generate the next recommended PPO/refinement commands from the verified baselines:

```powershell
<python> simulation/isaac/prototypes/pin_linkage/plan_domino_cad_linkage_next_runs.py
```

This historical planner continues from the older 70-degree `model_20.pt` PPO checkpoint and writes train/play artifacts under `simulation/isaac/out/cad_identity/next_policy/`. Its commands target the superseded direct-closure model; review and update the generated arguments before using them with the calibrated-neutral passive-linkage environment. The comparison tool remains useful for archived reports:

```powershell
<python> simulation/isaac/prototypes/pin_linkage/compare_domino_cad_linkage_playbacks.py `
  --candidate <output-folder>/rsl_play_candidate.json `
  --json-summary <output-folder>/compare_candidate.json
```

The first generated `-6 deg` yaw/lateral PPO refinement stayed upright and kept actual Domino CAD visuals enabled, but it did not replace the retained BC baselines: playback averaged about `50.6 mm` forward displacement, about `8.6 mm` lateral displacement, and about `0.162 rad` final heading drift.

A later 70-degree heading refinement produced a more useful historical checkpoint curve while keeping the same actual-CAD visual guard. `model_20.pt` was the then-balanced candidate at about `66.4 mm` forward displacement, `-0.5 mm` lateral displacement, and `0.283 rad` final heading drift. Later checkpoints pushed forward displacement higher, up to about `82.5 mm` at `model_79.pt`, but the final checkpoint also increased lateral and heading error. A stricter straightness continuation from `model_20.pt` reduced yaw to about `0.232-0.248 rad`, but forward displacement fell to about `60.3-62.5 mm`. A softer forward-preserving continuation reached about `68.9 mm` forward and `2.5 mm` lateral displacement, but yaw increased to about `0.311 rad`. Treat all of these direct-closure checkpoints as archived diagnostics rather than current walking policies.

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

`--drive-schedule independent` moves one drive at a time while the other drives hold their center positions. Use this mode when fitting the local relationship between the commanded inputs and measured output proxies. In the twelve-actuator modes, one independent cycle covers all twelve real actuator channels: four shoulders, four lower linkage drives, and four upper linkage/pitch drives. That is one shoulder actuator plus two two-bar/four-bar linkage-drive actuators per leg.

`--drive-schedule policy-step` applies bounded normalized action vectors to the same twelve actuator channels and holds each vector for `--policy-hold-steps`. Use this mode with `--reset-interval-steps` to check reset/action mechanics before changing the Isaac Lab CAD-linkage training wrapper.

The JSON report includes a named `action_space` and an `action_contract` block with the exact action index order. The script validates that twelve-actuator CAD scenes expose all four shoulders and all eight linkage-drive inputs before simulation starts. It also checks generated drive targets against the modeled joint limits by default; use `--disable-drive-limit-checks` only for deliberate stress tests.

## What Passing Means

Passing means the isolated one-actuator passive-pin loops, a simplified two-drive combined leg, a fixed-base all-leg pitch-linkage scene, a fixed-base twelve-actuator scene, and a shared fixed-body twelve-actuator scene can run without non-finite state or obvious constraint explosion. The calibration fits mean the combined one-leg case, the all-leg pitch independent sweep, and the twelve-actuator independent sweeps have repeatable local relationships between commanded drive targets and measured linkage-output proxies over the tested range.

The floating shared-body smoke adds gravity, a static ground box, and four simple spherical contact proxies at the CAD lower-closure points. Passing that test means the CAD-derived linkage can hold a static supported pose under gravity while keeping the loop-closure error finite.

The policy-step reset smoke proves the same floating scene accepts policy-shaped twelve-channel commands and can reset all CAD-linkage rigid bodies back to their initial poses during runtime.

The CAD-linkage `DirectRLEnv` and RSL-RL gates prove the floating shared-body scene can be stepped through Isaac Lab's RL API, can emit a 12-output actor, can write checkpoints, and can play back checkpoints without fall terminations over the tested horizon. The base observation has 61 values. The current teacher-conditioned policy appends the 12 reference actions, giving the actor a 73-value observation.

It does **not** mean Domino has a finished walking policy. The current replay-trained checkpoint starts bent, keeps all four legs attached, moves every lower endpoint, and raises every actual-CAD foot during its assigned swing segment. The next policy milestone is an overlapping stance/swing sequence that produces controlled forward velocity. Stair and rough-terrain training come after that flat-ground gait passes the existing stability, contact, and closure gates.
