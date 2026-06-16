# Domino Clean Quadruped Prototype

This folder contains the first clean all-leg Domino training-articulation prototype.

The raw CAD URDF is still kept as reference, but it is not used directly for training because it contains duplicate link names and closed linkage loops. This prototype takes the CAD-derived actuator layout and emits a simple tree articulation that Isaac Lab can import:

- Four shoulder hip ab/ad joints.
- Four lower linkage command joints.
- Four upper pitch command joints.
- CAD-derived hip locations, shoulder axis signs, and joint limits.

That gives the real 12-servo Domino command layout: one shoulder actuator plus two pitch/linkage-drive actuators per leg. The Isaac Lab config treats this order as a hard contract and the runtime checks fail if any of the 12 imported actuator joints are missing or reordered.

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

This sweep is a fixed-base articulation check. Passing means the imported USD exposes the full twelve-action Domino order, grouped as four shoulders, four lower-linkage drives, and four upper-pitch drives, and can accept conservative joint-position targets without non-finite state or joint-limit violations. It is not the final policy-training environment yet.

Runtime status: this prototype has passed a fixed-base Isaac Lab articulation sweep. See [`../../reports/domino-quadruped-runtime-sweep.md`](../../reports/domino-quadruped-runtime-sweep.md).

## Floating Contact Smoke

Import without `-FixBase`, then run:

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_quadruped_contact_smoke.py `
  --usd-path <output-folder>/domino_quadruped_clean_floating.usd `
  --headless `
  --steps 1000 `
  --report-path <output-folder>/domino_quadruped_contact_report.json
```

This uses a simple static ground-box collider because the local Isaac ground-plane helper currently fails to resolve its internal collision prim. Runtime status: passed. See [`../../reports/domino-quadruped-contact-stand-env.md`](../../reports/domino-quadruped-contact-stand-env.md).

## Stand Environment Smoke

Run the first `DirectRLEnv` wrapper:

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_domino_stand_env_smoke.py `
  --usd-path <output-folder>/domino_quadruped_clean_floating.usd `
  --headless `
  --steps 300 `
  --num-envs 1 `
  --report-path <output-folder>/domino_stand_env_smoke_report.json
```

`DominoStandEnv` has 12 actions, 49 policy observations, reset logic, rewards, and termination checks. The 12 actions are the four shoulder ab/ad actuators plus the two linkage-drive actuators on each leg. The latest observation layout adds four foot-contact flags to the original base, joint, and action terms.

## RSL-RL PPO Smoke

Run a tiny checkpoint-producing training smoke:

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

Runtime status: passed. The smoke produced a 49-input actor/critic, a 12-output actor, and a `model_0.pt` checkpoint. This is only a training-path proof, not a useful walking policy yet.

## RSL-RL Playback Smoke

Load the newest checkpoint under the log root and step it in a fresh env:

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_domino_stand_rsl_rl_play.py `
  --usd-path <output-folder>/domino_quadruped_clean_floating.usd `
  --headless `
  --num-envs 1 `
  --steps 250 `
  --log-root <output-folder>/domino_rsl_rl `
  --report-path <output-folder>/domino_stand_rsl_rl_play_report.json
```

Runtime status: passed. The playback smoke loaded `model_0.pt`, emitted finite 12-channel actions, and completed one stand-task episode horizon without a fall termination.

## Parallel Training Smoke

The stand env sizes its static ground from the cloned-env grid, so it can run small parallel RSL-RL tests without cloned robots starting off the ground plate.

Example 16-env training-scale smoke:

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

Example 16-env playback:

```powershell
<isaac-python> simulation/isaac/prototypes/quadruped/run_domino_stand_rsl_rl_play.py `
  --usd-path <output-folder>/domino_quadruped_clean_floating.usd `
  --headless `
  --num-envs 16 `
  --steps 250 `
  --log-root <output-folder>/domino_rsl_rl `
  --report-path <output-folder>/domino_stand_rsl_rl_play_16env_report.json
```

Runtime status: passed. The contact-aware 16-env smoke used a 10 m ground box, completed a 128-timestep PPO rollout/update with 49 policy observations, wrote a checkpoint, then replayed that checkpoint for one episode horizon across all 16 envs. All 16 done events were expected timeouts, not fall terminations.

The remaining CAD-fidelity gap is important: this training env still uses the clean tree articulation. The CAD-derived passive linkage loops are validated in the pin-linkage prototypes, but they are not yet merged into the floating policy-training articulation.
