# Domino Isaac Sim / Isaac Lab Bring-Up

This folder is the working plan for turning Domino from CAD and firmware into a stable Isaac Sim / Isaac Lab robot. The aim is a trainable quadruped model, not just a pretty visual import.

> Current status: the raw CAD-generated URDF is not suitable as a training articulation by itself. The current Isaac Lab model reconstructs the mechanism as a stable 29-body passive-linkage system, starts in a captured bent neutral pose, exposes all 12 servo commands, and has a verified reference-conditioned four-foot linkage-swing controller. This is a mechanism demonstration, not an autonomous walking policy. Forward walking and terrain training are still in progress. The Isaac Sim and Isaac Lab runtimes are not included in this repository.

> Asset fidelity guardrail: the legacy `run_pin_linkage.py` modes are CAD-derived physics proxies with cube and sphere debug visuals. The newer CAD-linkage builder used by the policy-search and `DirectRLEnv` paths attaches the exported Domino STL link meshes to those moving physics bodies by default. Treat it as real Domino CAD visuals on simplified proxy physics, not as a final full-CAD collision model.

Visible, livestreamed, and camera-rendered proxy-only runs are blocked by default unless `--allow-proxy-visuals` is passed. This keeps the simplified cube/sphere debug scene from being mistaken for the actual Domino CAD visual model.

Visible, livestreamed, and camera-rendered multi-environment runs are also blocked by default in the CAD-linkage smoke, training, and playback scripts. Use `--num-envs 1` when checking whether the robot looks like Domino. Pass `--allow-multi-env-viewport` only when intentionally viewing cloned training batches.

## Reproduce the Current Baseline

Install Isaac Sim and Isaac Lab, then clone this repository. The CAD meshes, generated URDF package, 29-body linkage builder, 12-actuator environment, reference gait, reward configuration, launch scripts, and the pre-500-run `model_210` example policy are tracked. Isaac Sim, Isaac Lab, subsequent generated checkpoints, and local run logs are deliberately not committed.

Set the runtime locations if they are not installed at `C:\isaac-sim` and `C:\isaac-projects\IsaacLab`:

```powershell
$env:ISAAC_SIM_ROOT = "<path-to-Isaac-Sim>"
$env:ISAACLAB_ROOT = "<path-to-IsaacLab>"
```

Start a visible, single-environment PPO run using the actual Domino CAD meshes:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-visible-domino-actual-cad-learning.ps1 `
  -NumEnvs 1 `
  -Iterations 100
```

Run 500 iterations headless across ten environments:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-headless-domino-policy-training.ps1 `
  -NumEnvs 10 `
  -Iterations 500
```

The headless launcher resumes the newest compatible local checkpoint when one exists. Pass `-Fresh` to initialize a new actor from the tracked diagonal-trot reference, or pass `-ResumeCheckpoint <model.pt>` to choose a checkpoint explicitly. Checkpoints and reports are written below `simulation/isaac/out/`.

On a fresh clone, the launcher automatically starts from `checkpoints/domino_actual_cad_baseline_model_210.pt`. To inspect that exact policy in a visible single-robot run before continuing PPO:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-visible-domino-actual-cad-learning.ps1 `
  -NumEnvs 1 `
  -Iterations 1 `
  -ResumeCheckpoint simulation/isaac/checkpoints/domino_actual_cad_baseline_model_210.pt
```

This is still a work-in-progress locomotion experiment. The tracked setup demonstrates the real CAD appearance, all 12 servo channels, passive lower and upper linkage pins, reset/termination handling, and an active PPO training loop. It is not presented as a converged walking policy.

## Included Files

| Path | Purpose |
| --- | --- |
| `../usd/` | Existing USD/USDZ visual exports from the CAD-to-Isaac exploration. |
| `../urdf/generated/Domino_URDF_Parts_Combined_Final_description/` | Generated URDF package copied from the CAD export for reference and analysis. |
| `analyze-domino-urdf.ps1` | Repeatable URDF topology validator. |
| `analyze-domino-linkage-pivots.ps1` | Extracts named linkage pivot coordinates from the generated URDF for CAD-derived pin-joint tests. |
| `run-domino-urdf-import.ps1` | Parameterized Isaac Lab URDF import smoke-test helper. |
| `run-visible-domino-policy.ps1` | Visible actual-CAD playback launcher for the bent neutral pose and latest locally verified weight-transfer checkpoint. |
| `run-visible-domino-training.ps1` | Visible or headless BC/PPO launcher for the calibrated neutral, passive-linkage model. |
| `run-visible-domino-actual-cad-learning.ps1` | Current single-robot actual-CAD PPO configuration used for visual inspection. |
| `run-headless-domino-policy-training.ps1` | Clone-friendly 500-iteration headless launcher with automatic local checkpoint resume. |
| `checkpoints/domino_actual_cad_baseline_model_210.pt` | Pre-500-run example policy used to reproduce or continue the current actual-CAD PPO experiment. |
| `prototypes/one_leg/` | Clean three-joint one-leg prototype for the first stable Isaac articulation. |
| `prototypes/actual_cad/` | USD audit/wrapper tools for the real Domino mesh CAD export. |
| `prototypes/pin_linkage/` | Minimal actuated pin-joint constraint prototype with generic and CAD-derived Domino linkage modes. |
| `prototypes/pin_linkage/run_domino_cad_linkage_one_foot_search.py` | Deterministic parallel search for stable single-foot unload commands using the current 12-actuator passive-linkage model. |
| `prototypes/pin_linkage/run_domino_cad_linkage_weight_transfer_search.py` | Searches stable full-body weight-transfer commands and verifies actual-CAD foot unloading for each leg. |
| `prototypes/pin_linkage/run_domino_cad_linkage_weight_transfer_cycle.py` | Runs the four verified commands as one continuous no-reset mechanics cycle. |
| `prototypes/pin_linkage/build_domino_weight_transfer_teacher.py` | Builds the tracked four-leg keyframe teacher after the continuous-cycle mechanics gate passes. |
| `config/domino_weight_transfer_cycle_teacher.json` | Current 1,360-step, 12-channel weight-transfer teacher used by BC and reference playback. |
| `prototypes/pin_linkage/run_domino_cad_linkage_teacher_order_search.py` | Evaluates all 24 four-leg swing orders in parallel against true endpoint, stability, and closure gates. |
| `prototypes/pin_linkage/build_domino_linkage_swing_teacher.py` | Builds the current verified 2,360-step linkage-swing teacher from four individually verified commands. |
| `config/domino_linkage_swing_cycle_teacher.json` | Current 12-channel teacher in rear-left, rear-right, front-right, front-left order. |
| `prototypes/pin_linkage/verify_domino_cad_linkage_reports.py` | Non-GPU verifier for local actual-CAD visual, 12-actuator, and playback report artifacts. |
| `prototypes/pin_linkage/plan_domino_cad_linkage_next_runs.py` | Generates staged PowerShell commands for the next CAD-linkage PPO/refinement experiments from verified baselines. |
| `prototypes/pin_linkage/compare_domino_cad_linkage_playbacks.py` | Compares a new CAD-linkage playback report against the retained forward, lateral-drift, and yaw baselines. |
| `reports/domino-urdf-topology.md` | Current topology report generated from the URDF export. |
| `reports/domino-linkage-pivots.md` | CAD-derived pivot report for the first lower-linkage loops. |
| `reports/domino-linkage-pivots.json` | Machine-readable pivot extraction output. |
| `reports/domino-urdf-import-smoke-test.md` | Result of the first Isaac Lab raw import smoke test. |
| `reports/domino-one-leg-import-smoke-test.md` | Result of the first clean one-leg import smoke test. |
| `reports/domino-one-leg-runtime-sweep.md` | Result of the first clean one-leg Isaac Lab articulation sweep. |
| `reports/domino-pin-linkage-runtime.md` | Results of the generic and CAD-derived actuated passive pin-linkage runtime tests. |
| `reports/domino-combined-linkage-characterization.md` | Motion characterization for the combined two-drive CAD-derived one-leg linkage. |
| `reports/domino-four-leg-linkage-runtime.md` | First all-leg CAD-derived pitch-linkage Isaac/PhysX smoke test. |
| `reports/domino-12-actuator-runtime.md` | First fixed-base all-leg scene exposing the full twelve-actuator Domino command layout. |
| `reports/domino-quadruped-runtime-sweep.md` | First clean all-leg 12-DoF Isaac Lab articulation runtime sweep. |
| `reports/domino-quadruped-contact-stand-env.md` | First floating-base contact smoke, first Domino stand `DirectRLEnv` smoke, first tiny RSL-RL PPO checkpoint smoke, and first playback smoke. |
| `reports/domino-policy-cad-linkage-contract.md` | Automated check that the clean policy action contract matches the CAD-derived shared fixed-body linkage report. |
| `reports/domino-floating-cad-linkage-contact.md` | First gravity/contact smoke for the floating CAD-derived twelve-actuator linkage scene. |
| `reports/domino-floating-cad-policy-reset.md` | First policy-style action and runtime reset smoke for the floating CAD-derived linkage scene. |
| `reports/domino-cad-linkage-direct-rl-env.md` | Isaac Lab `DirectRLEnv`, command-aware RSL-RL runs, signed-axis scripted gait search, 12-actuator BC/PPO warm starts, and current locomotion gap for the floating CAD-derived twelve-actuator linkage scene. |
| `reports/domino-split-linkage-servo-slew-status.md` | Historical split-closure and servo-slew milestone retained for engineering traceability. |

## Current Finding

The generated URDF is useful as a record of the CAD export, but it should not be imported blindly into Isaac as the final training model.

The current real visual source is `../usd/Domino_Quadruped.usd`. Direct USD inspection shows it opens as a mesh-based CAD visual export with 30 mesh prims, 0 rigid bodies, and 0 joints. That makes it useful for verifying the real Domino shape, scale, and CAD appearance, but it is not yet a trainable robot.

The raw-CAD identity audit writes to `out/cad_identity/domino_raw_cad_audit.json` and the meter-based wrapper writes to `out/cad_identity/domino_raw_cad_visual_wrapper.usda`. That wrapper is the clean visual-only file to open when checking the CAD identity before judging any physics or policy scene.

The current standalone policy-search scene is different from the raw visual export. It attaches 29 exported Domino STL meshes, totaling about 135k triangles, to 29 simplified moving physics bodies. The passive finite-link model includes both lower and upper two-pivot closure bodies instead of collapsing each loop into one joint. It reports 0 visible robot proxy cubes and 0 visible robot proxy spheres when CAD visuals are enabled, and the hidden proxy shapes are also marked as guide-purpose transparent geometry. That scene is useful for testing the twelve driven channels and closed-chain linkage behavior while showing the real Domino CAD geometry, but it should still be described as proxy physics rather than a final full-CAD collision model.

The local report verifier checks the current ignored JSON artifacts under `out/cad_identity/` before those artifacts are used as evidence. It verifies the actual-CAD visual audit, hidden proxy visuals, non-render proxy guard, 12 action channels, and the retained playback baselines. New PPO/refinement runs should also be checked with the playback comparison helper before replacing any retained baseline.

The current topology report shows:

- 37 link elements.
- 29 unique link names.
- 36 joint elements.
- 8 duplicate link-name groups.
- 8 child links with multiple incoming joints.
- 11 limited revolute joints and 25 continuous joints.

That means the file contains loop-style structure and repeated link names. Isaac/PhysX articulations need a tree structure for the articulation itself; loops can be closed separately using rigid-body joints between articulation links, but the core articulation still has to be authored cleanly. See the official PhysX articulation notes on tree structure and closing loops: [PhysX Articulations](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Articulations.html#articulation-tree-structure).

## Recommended Strategy

Do not try to make the exported CAD mates become the RL policy model directly. Build the simulator in layers:

1. **Visual model**
   - Use the USD exports for visual inspection, scale checks, orientation checks, and render/reference work.
   - Keep mesh collisions disabled or heavily simplified during early physics tests.

2. **Training articulation**
   - Create a clean 12-DoF tree articulation: hip ab/ad, upper-leg pitch, and lower/knee actuator coordinate for each leg.
   - Keep the action space tied to the twelve real servo commands rather than every passive CAD pin: four shoulder actuators plus two two-bar/four-bar linkage-drive actuators per leg.
   - Treat the upper linkage/pitch command as the second linkage-drive actuator for each leg, not as an extra passive pin or a separate reduced model.
   - Give every training joint a readable name before training; do not keep names like `Revolute 46` in the final model.

3. **Passive linkage fidelity**
   - First pass: treat the lower four-bar linkage as visual geometry attached to the controlled lower/knee body.
   - Second pass: add passive revolute pins and loop-closing rigid-body joints after a one-leg model is stable.
   - If the loop constraints fight the driven joints, keep the training model simplified and use the detailed linkage model only for validation.

4. **Isaac Lab asset config**
   - Spawn the final robot from USD using `ArticulationCfg` and `UsdFileCfg`.
   - Configure actuator groups for the twelve driven joints only.
   - Disable self-collision until basic falling, standing, and joint-limit tests are stable.
   - Use conservative solver settings and low drive gains during first motion tests.

5. **Policy path**
   - Validate one leg.
   - Validate all four legs standing on a raised fixture.
   - Train a stand/height-control task.
   - Keep velocity-command inputs explicit in the policy state, then add gait rewards and longer locomotion training after reset behavior, joint limits, and contacts are reliable.

## Commands

Run the URDF topology report:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/analyze-domino-urdf.ps1
```

Run it against another exported URDF:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/analyze-domino-urdf.ps1 -UrdfPath path\to\robot.urdf
```

Run a raw Isaac Lab import smoke test:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-domino-urdf-import.ps1 `
  -IsaacLabRoot <path-to-IsaacLab> `
  -IsaacPython <path-to-isaac-python> `
  -OutputUsd <output-folder>/domino_raw_import.usd `
  -AcceptEula
```

The helper deliberately takes Isaac paths as arguments or environment variables. Do not hard-code personal workstation paths into this repository.

Run the clean one-leg prototype through the same importer:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-domino-urdf-import.ps1 `
  -UrdfPath simulation/isaac/prototypes/one_leg/domino_one_leg_clean.urdf `
  -IsaacLabRoot <path-to-IsaacLab> `
  -IsaacPython <path-to-isaac-python> `
  -OutputUsd <output-folder>/domino_one_leg_clean.usd `
  -FixBase `
  -AcceptEula
```

Generate and import the clean all-leg quadruped prototype:

```powershell
<python> simulation/isaac/prototypes/quadruped/generate_quadruped_urdf.py

powershell -ExecutionPolicy Bypass -File simulation/isaac/run-domino-urdf-import.ps1 `
  -UrdfPath simulation/isaac/prototypes/quadruped/domino_quadruped_clean.urdf `
  -IsaacLabRoot <path-to-IsaacLab> `
  -IsaacPython <path-to-isaac-python> `
  -OutputUsd <output-folder>/domino_quadruped_clean.usd `
  -FixBase `
  -NoMergeJoints `
  -AcceptEula
```

Run the clean all-leg quadruped articulation sweep:

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_quadruped_sweep.py `
  --usd-path <output-folder>/domino_quadruped_clean.usd `
  --headless `
  --steps 600 `
  --report-path <output-folder>/domino_quadruped_sweep_report.json
```

Import the clean quadruped as a floating-base USD for gravity/contact tests:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-domino-urdf-import.ps1 `
  -UrdfPath simulation/isaac/prototypes/quadruped/domino_quadruped_clean.urdf `
  -IsaacLabRoot <path-to-IsaacLab> `
  -IsaacPython <path-to-isaac-python> `
  -OutputUsd <output-folder>/domino_quadruped_clean_floating.usd `
  -NoMergeJoints `
  -AcceptEula
```

Run the floating-base gravity/contact smoke:

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_quadruped_contact_smoke.py `
  --usd-path <output-folder>/domino_quadruped_clean_floating.usd `
  --headless `
  --steps 1000 `
  --report-path <output-folder>/domino_quadruped_contact_report.json
```

Run the first stand-task `DirectRLEnv` smoke:

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_domino_stand_env_smoke.py `
  --usd-path <output-folder>/domino_quadruped_clean_floating.usd `
  --headless `
  --steps 300 `
  --num-envs 1 `
  --report-path <output-folder>/domino_stand_env_smoke_report.json
```

Run the first checkpoint-producing RSL-RL PPO smoke:

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_domino_stand_rsl_rl_train.py `
  --usd-path <output-folder>/domino_quadruped_clean_floating.usd `
  --headless `
  --num-envs 1 `
  --iterations 1 `
  --num-steps-per-env 8 `
  --log-root <output-folder>/domino_rsl_rl `
  --report-path <output-folder>/domino_stand_rsl_rl_report.json
```

Load and evaluate the newest RSL-RL checkpoint:

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_domino_stand_rsl_rl_play.py `
  --usd-path <output-folder>/domino_quadruped_clean_floating.usd `
  --headless `
  --num-envs 1 `
  --steps 250 `
  --log-root <output-folder>/domino_rsl_rl `
  --report-path <output-folder>/domino_stand_rsl_rl_play_report.json
```

Run a 16-env training-scale smoke after the one-env path is passing:

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_domino_stand_rsl_rl_train.py `
  --usd-path <output-folder>/domino_quadruped_clean_floating.usd `
  --headless `
  --num-envs 16 `
  --iterations 1 `
  --num-steps-per-env 8 `
  --log-root <output-folder>/domino_rsl_rl `
  --report-path <output-folder>/domino_stand_rsl_rl_train_16env_report.json
```

Extract CAD linkage pivots from the generated URDF:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/analyze-domino-linkage-pivots.ps1
```

Run the CAD-derived one-joint lower-linkage test:

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

Run the CAD-derived one-joint upper-linkage test:

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

Run the combined two-drive one-leg linkage test:

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

Run the all-leg CAD-derived pitch-linkage test:

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

Run the fixed-base twelve-actuator independent calibration sweep:

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

Run the shared-body fixed-base twelve-actuator independent calibration sweep:

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

Validate the policy action contract against a CAD-derived linkage JSON report:

```powershell
<python> simulation/isaac/prototypes/quadruped/check_cad_linkage_contract.py `
  --linkage-report <output-folder>/domino_four_12_fixed_body_independent_report.json `
  --urdf-path simulation/isaac/prototypes/quadruped/domino_quadruped_clean.urdf `
  --report-path <output-folder>/domino_policy_cad_contract_report.json
```

Run the floating CAD-derived twelve-actuator linkage contact smoke:

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

Run the floating CAD-derived policy-style action/reset smoke:

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

Verify the local actual-CAD and playback report artifacts without launching Isaac:

```powershell
<python> simulation/isaac/prototypes/pin_linkage/verify_domino_cad_linkage_reports.py
```

This verifier also checks that the top-level Isaac README, the detailed CAD-linkage DirectRLEnv report, and the actual-CAD audit report still contain the same current baseline numbers.

Generate the next recommended CAD-linkage policy experiment commands:

```powershell
<python> simulation/isaac/prototypes/pin_linkage/plan_domino_cad_linkage_next_runs.py
```

That planner was written for the pre-split-closure continuation path, so treat it as a command template rather than the current source of truth. For the current mechanics, continue from the stable `model_79.pt` / `35 deg` / `360 deg/s` servo-slew setup unless a new playback report proves a better checkpoint.

Compare a new playback report against the retained baselines:

```powershell
<python> simulation/isaac/prototypes/pin_linkage/compare_domino_cad_linkage_playbacks.py `
  --candidate <output-folder>/rsl_play_candidate.json `
  --json-summary <output-folder>/compare_candidate.json
```

## Isaac Runtime Notes

NVIDIA's current Isaac Lab import workflow recommends converting robot assets into USD and then writing an asset configuration for spawning and training. The Isaac Lab docs also call out useful URDF import settings such as fixed base selection, fixed-joint merging, joint drive configuration, and setting joint target type to `none` during import when you want to configure drives later. See:

- [Isaac Lab: Importing a New Asset](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)
- [Isaac Lab: Writing an Asset Configuration](https://isaac-sim.github.io/IsaacLab/main/source/how-to/write_articulation_cfg.html)
- [Isaac Sim: Import URDF](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/import_urdf.html)

## Immediate Next Milestone

This remains a work-in-progress simulation and policy-training project. The current baseline is a mechanically stable bring-up model, not a finished walking policy.

The current model uses:

- the exported Domino STL meshes for the visible robot;
- 29 simplified rigid physics bodies with passive finite lower and upper loop closures;
- all 12 real actuator channels: four 40 kg-cm shoulder servos and eight 35 kg-cm linkage servos;
- an authored 29-body neutral pose captured from a converged fixed-base linkage solve;
- CAD-derived visual-bottom foot contacts, explicit friction, realistic stall-torque caps, and a 90 deg/s target slew limit; and
- a 16-degree normalized action range around the bent neutral pose.

The current verified gates are:

| Gate | Result |
| --- | --- |
| Authored neutral, fixed base | Four bent legs, no straight-leg startup lock, approximately `1 micrometre` maximum pin error. |
| Zero-action free body, 10 seconds | No reset or detachment; about `7.2 deg` body tilt and `1.89 mm` maximum pin separation. |
| Independent 12-actuator sweep | Every actuator moves its own real CAD foot; minimum observed foot motion is about `29.5 mm`. |
| Settled free-body 12-actuator sweep | Every shoulder, lower, and upper channel moves under load in both directions with no termination; maximum pin separation stays below `3.1 mm` at the tested scale. |
| Four-leg order search | All 24 orders are evaluated in parallel. The selected rear-left, rear-right, front-right, front-left sequence was one of two orders that passed the stability, closure, drive-motion, and endpoint-motion gates. |
| Keyframed teacher playback | The 2,360-step reference completes with no reset or termination. True CAD-foot motion measured in each foot's own hip-carriage frame is `53.3-85.5 mm`; all eight linkage drives move `4.78-20.54 deg`. Maximum body tilt is `20.90 deg` and maximum pin separation is `3.73 mm`. |
| Reference-conditioned RSL-RL checkpoint | The 73-input, 12-output actor reproduces the appended verified target. Fresh checkpoint playback completes 2,360 steps with zero resets and the same endpoint, drive, tilt, and closure measurements as reference mode. A `0.0001` agreement snap executes the exact target when the policy is already within tolerance, avoiding contact-chaos divergence from floating-point copy error. |
| Visible actual-CAD inspection | A slowed single-robot playback shows the real Domino CAD, four attached bent legs, driven lower and upper links changing angle, passive closure bars remaining closed, and every foot endpoint moving. No proxy robot geometry is visible. |

The teacher and reference-conditioned checkpoint prove that the real CAD visuals, 12-channel action contract, bent neutral startup, servo limits, and closed-linkage physics run together. They do not prove autonomous locomotion learning. The current controller deliberately follows a verified mechanism trajectory; the next policy must learn stable residuals or an independent gait that produces controlled forward velocity.

Launch the current visible checkpoint:

```powershell
$env:ISAAC_SIM_ROOT = "<path-to-Isaac-Sim>"
$env:ISAACLAB_ROOT = "<path-to-IsaacLab>"
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-visible-domino-policy.ps1 `
  -VisibleStartDelayS 8 `
  -VisibleStepDelayS 0.04 `
  -NoHoldOpen
```

Set `ISAAC_SIM_ROOT` and `ISAACLAB_ROOT`, or pass `-IsaacSimRoot` and `-IsaacLabRoot`. Automatic checkpoint selection accepts only the current hip-frame training artifact when either its embedded validation or a matching fresh-process validation report passed. It does not fall back to historical checkpoints. The delay options make the 47.2-second control sequence easier to inspect in the viewport.

Re-run the continuous mechanics gate and rebuild the tracked teacher after changing any of the four promoted command files:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_domino_cad_linkage_weight_transfer_cycle.py `
  --headless `
  --leg-order 1,3,0,2 `
  --report-path simulation/isaac/out/cad_identity/next_policy/weight_transfer_context_verified_four_foot_cycle_v2.json `
  --no-print-report

<python> simulation/isaac/prototypes/pin_linkage/build_domino_weight_transfer_teacher.py
```

The builder refuses to publish a teacher unless the source continuous-cycle report passed its global mechanics gate.

Run the current lower-linkage visibility diagnostic:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-visible-domino-policy.ps1 `
  -PolicyMode fixed `
  -TerrainType flat `
  -ActionScaleDeg 8 `
  -FixedActions "0,1,-1,0,0,0,0,0,0,0,0,0"
```

This is a mechanical one-foot unload probe, not a learned policy. It starts with the same zero-action neutral settle used by policy playback.

Run the current actual-CAD PPO experiment:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-visible-domino-actual-cad-learning.ps1 `
  -NumEnvs 1 `
  -Iterations 100
```

The next policy pass should focus on:

- Preserve the authored neutral pose, zero-action hold, all-12-actuator sweep, and joint-separation gates.
- Turn the verified four-foot unloading sequence into an overlapping stance/swing gait with positive forward progress.
- Train velocity tracking on flat ground before introducing stairs or rough terrain.
- Keep stance/swing labels tied to settled foot trajectories so initial gravity settlement does not count as gait motion.
- Retain per-foot clearance, slip, lateral-drift, yaw, body-height, tilt, and loop-closure gates during PPO refinement.
- Keep verifying all 12 actuator channels: four shoulder hip ab/ad drives, four lower-linkage drives, and four upper pitch/linkage drives.
- Keep the detailed closed-linkage model as the mechanical validation authority, even if policy iteration later moves to a simpler tree articulation.
