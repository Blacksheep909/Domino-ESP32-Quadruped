# Historical Split-Linkage and Servo-Slew Status

> Historical note: this report records an earlier proxy-model milestone. The current passive finite-link, actual-CAD contact, authored-neutral-pose baseline is documented in [`../README.md`](../README.md).

This report captures the current Isaac Lab bring-up state after the closed-linkage and servo-command fixes. Paths below are repository-relative.

## Current Model

The active Domino simulation path is the CAD-linkage `DirectRLEnv`:

- Actual Domino STL visuals are attached to proxy rigid bodies.
- The visible robot uses 29 exported STL meshes and about 135k triangles.
- Proxy cubes and spheres are hidden when CAD visuals are enabled.
- The physics model is still proxy physics, not final full-CAD collision.
- The current proxy closes the lower and upper linkage loops directly between the neighboring moving links; it no longer creates extra one-pivot closure rigid bodies.
- The action space is still the real 12-servo layout: four shoulder hip ab/ad servos, four lower-linkage drives, and four upper pitch/linkage drives.

## Fixes Applied

The lower and upper closure meshes are now treated as CAD visuals attached to their adjacent moving links. Earlier tests created separate passive closure rigid bodies, but those one-pivot bodies overconstrained the closed loops and could make the leg fight itself under contact. The current model closes the lower loop directly from lower driver to lower diagonal, and closes the upper loop directly from coupler to upper driver.

The current visible-check path uses `actual-cad-grounded-support` for stable physics contact while measuring reward, observation, and playback diagnostics from the rendered CAD foot-bottom points. Raw `actual-cad-visual-bottom` contact is still available as an experimental diagnostic mode, but it is not the default because the exported neutral CAD foot-bottom heights are not coplanar and the robot can tip over even at zero action.

The actuator model now includes a conservative servo target slew limiter. The configured default is `360 deg/s`, roughly `60 deg` in `0.167 s`. This prevents a policy from instantly snapping a drive target by a large angle every env step, which was a major cause of shoulder pin separation in high-action playback.

The actuator report records:

- DSSERVO 40KG digital servos on the four shoulder hip ab/ad axes.
- DSservo 35KG digital servos on the eight linkage-drive axes.
- Stall torque converted from kg-cm to N-m.
- Drive stiffness/damping and the current target slew limit.

The gait-search, smoke, play, and train validation paths now use the visible CAD foot-bottom points for contact and clearance metrics when CAD visuals are enabled. The hidden grounded support spheres remain a physics support scaffold, not the source of gait reward truth.

## Verified Runs

These are the most useful verified runs after the direct-closure fix.

| Run | Result |
| --- | --- |
| `direct_closure_grounded_scale20_sweep.json` | Grounded per-channel actuator sweep; no resets; max joint separation about `5.8 mm` during forced one-hot sweeps. |
| `direct_closure_nogravity_float35_linkage_scale20_sweep.json` | Floating no-gravity per-channel sweep; no resets; upper pitch channels move actual endpoints in isolation. |
| `direct_closure_visible_policy_stairs_240.json` | Visible actual-CAD stairs playback; no resets; max joint separation about `0.43 mm`; max body tilt about `8 deg`; policy still drags feet. |
| `direct_closure_grounded_support_scale20_policy_refine.json` | Short PPO continuation from `model_114.pt`; stayed stable and wrote `model_145.pt`, but failed the walking gate because `swing_contact=1.0` and `swing_clearance=0`. |
| `grounded_support_base_start_zero_check.json` | Current grounded-support startup check; 120 zero-action startup steps, then zero policy; no resets/timeouts; max joint separation about `0.47 mm`; max body tilt about `5.6 deg`; rendered CAD foot drift about `14 mm`; visible CAD feet are still not level at startup. |
| `grounded_support_base_start_reference_check.json` | Current scripted linkage-motion check; 120 zero-action startup steps, then reference gait; no resets/timeouts; max joint separation about `0.36 mm`; max body tilt about `7.3 deg`; rendered CAD foot-bottom motion about `62 mm`; rear/other-side visible feet move much less than the two more active feet. |
| `grounded_support_base_start_checkpoint_check.json` | Current checkpoint visual check with startup hold and reduced `12 deg` playback action scale; no resets/timeouts; max joint separation about `0.42 mm`; max body tilt about `7.2 deg`; rendered CAD foot-bottom motion about `64 mm`; this is stable but not a finished walking or visually level base-pose result. |
| `calibrated_fixed_stance_visual_check.json` | Current calibrated fixed-stance check; 120 custom startup steps, then the same fixed action row is held; no resets/timeouts; max joint separation about `0.22 mm`; max body tilt about `6.5 deg`; visible CAD foot drift while held about `1.4 mm`; visible foot-height spread improved to about `28 mm`, but this is still not fully level. |
| `calibrated_fixed_stance_visual_ground_aligned_check.json` | Same calibrated fixed stance with a post-startup global CAD visual lowering pass; no resets/timeouts; max joint separation about `0.19 mm`; the lowest rendered foot bottom is now at about `2 mm` clearance; visible foot-height spread remains about `28 mm`. |

Older split-linkage runs remain useful history, but they are not the current authority for linkage topology:

| Run | Result |
| --- | --- |
| `mechanism_split_closure_linkage_contact_reference_flat_160steps.json` | Reference action playback on flat ground; no resets; max joint separation about `0.67 mm`; foot endpoint motion about `102 mm`. |
| `mechanism_split_closure_linkage_contact_zero_flat_160steps.json` | Zero-action hold on flat ground; no resets; max joint separation about `0.83 mm`; foot drift about `6 mm`. |
| `model79_split_closure_linkage_contact_stairs_playback_240steps.json` | Old `70 deg` model without slew was too aggressive; shoulder separation reached about `50 mm` and it reset. Do not use this as a valid policy. |
| `model79_split_closure_servo_slew_stairs_playback_240steps.json` | Servo slew reduced max joint separation to about `1.9 mm`, but the old `70 deg` policy still saturated and reset. |
| `model79_split_closure_servo_slew_scale35_stairs_playback_240steps.json` | Stable current reference checkpoint; no resets; max joint separation about `0.72 mm`; forward displacement about `67.6 mm`; lateral drift about `26.6 mm`; final yaw drift about `0.61 rad`. |
| `model79_scale35_servo_slew_stairs_refine_train.json` | 25 PPO iterations from the stable `model_79.pt` / `35 deg` / servo-slew setup; produced `model_103.pt`. |
| `model103_split_closure_servo_slew_scale35_stairs_playback_240steps.json` | Latest trained checkpoint; no resets; max joint separation about `0.71 mm`; forward displacement about `65.4 mm`; lateral drift about `25.3 mm`; final yaw drift about `0.61 rad`. |
| `gait_search_servo_slew_scale35_stairs_refine_seed9411_24.json` | Refined 24 reference-gait candidates on the corrected stairs/servo-slew setup. The best candidate stayed stable but only moved about `12.7 mm` forward, so it does not replace `model_79.pt`. Failed candidates in this search are not evidence against the stable policy. |

The old PPO continuation report stayed stable but was generated before the visible-foot reward/diagnostic patch, so it is no longer the authority for swing-contact quality. The current bottleneck is still policy training: the mechanics now hold together and the scripted linkage motion is stable, but the learned checkpoint is not a finished walking policy.

## Current Best Interpretation

The hard physics-separation failure mode is mostly fixed in the current grounded-support visual-check path:

- The grounded-support startup holds without resets or timeouts.
- The feet move under scripted reference actions.
- The hips no longer detach under current direct-closure playback.
- The closed passive pins remain within sub-millimetre separation in the current zero, reference, and checkpoint visual checks.
- The latest reports do not support blaming the front-leg straight-lock only on old saturated checkpoint actions. The current measured startup stance still has visible CAD foot-bottom heights spread by about `56 mm`, so the next sim fix should level the visible CAD stance or recalibrate the servo neutral pose before treating the base pose as clean.

The locomotion policy is not done:

- It moves forward, but slowly.
- It still drifts laterally.
- It accumulates yaw error.
- Swing feet are not unloading cleanly enough.
- It should not be presented as a finished walking policy.

Raw `actual-cad-visual-bottom` contact is not solved yet. It should be treated as an experimental contact mode until the neutral CAD stance is leveled or a better foot collision proxy is built.

The current `actual-cad-grounded-support` mode is useful for keeping the closed-linkage mechanism stable while policy work continues, but it is not visual proof that all feet are planted correctly. The visible CAD foot-bottom positions should be checked directly in every playback report.

## Stance Calibration Path

The playback runner now records `actual_cad_visual_start_stance`, including the visible CAD foot-bottom z values and height spread. It also accepts `--startup-actions`, a comma-separated 12-channel normalized action row, so a calibrated standing pose can be held during the startup window instead of assuming zero action is a good visual base pose.

Use the calibration helper to search for a better constant startup stance:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-domino-stance-calibration.ps1 `
  -IsaacSimRoot C:\isaac-sim `
  -IsaacLabRoot C:\isaac-projects\IsaacLab `
  -RunName grounded_support_stance_calibration
```

The resulting report writes `startup_actions_arg`. Feed that row back into visible playback:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-visible-domino-policy.ps1 `
  -IsaacSimRoot C:\isaac-sim `
  -IsaacLabRoot C:\isaac-projects\IsaacLab `
  -PolicyMode fixed `
  -StartupActions "<startup_actions_arg from calibration report>" `
  -FixedActions "<startup_actions_arg from calibration report>" `
  -RunName calibrated_startup_zero_visual_check
```

Only call the visual base pose improved if the new playback report shows no resets, sub-millimetre joint separation, and a much smaller `actual_cad_visual_start_stance.max_height_spread_m` than the current `56 mm` base-start spread.

Current best fixed-stance row:

```text
1.000000,0.500000,0.000000,-0.500000,1.000000,0.000000,-1.000000,0.000000,-1.000000,0.000000,0.000000,-0.500000
```

This row improves visible foot-height spread from about `56 mm` to about `28 mm` and holds the visible feet nearly still, but it should not be treated as a final neutral pose because all rendered foot bottoms remain above the floor and the spread is still above the `25 mm` warning threshold.

The latest ground-aligned fixed-stance check applies a global visual-only correction after startup, lowering the rendered CAD overlay by about `38 mm`. That puts the lowest visible foot bottom on the support plane without changing proxy physics. This is useful for visual inspection, but it is not a final contact solution: the first two feet still have large visual-to-support XY offsets, about `133 mm` and `107 mm`, because the hidden support spheres are not directly under the rendered foot bottoms once the lower-driver bodies rotate.

## Repeatable Visible Playback

Use `run-visible-domino-policy.ps1` to launch the current visual check with the actual Domino CAD visuals, direct loop closures, `12 deg` playback action scale, a 120-step zero-action base-pose startup hold, grounded-support physics contact, and visible CAD foot-bottom diagnostics:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-visible-domino-policy.ps1
```

Set `ISAAC_SIM_ROOT` and `ISAACLAB_ROOT`, or pass `-IsaacSimRoot` and `-IsaacLabRoot`, before running it. The helper prefers the latest direct-closure checkpoint `model_145.pt`, then falls back to earlier checkpoints if needed.

Useful modes:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-visible-domino-policy.ps1 -PolicyMode zero
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-visible-domino-policy.ps1 -PolicyMode reference
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-visible-domino-policy.ps1 -PolicyMode checkpoint
```

Verify the current gate with:

```powershell
python simulation/isaac/prototypes/pin_linkage/verify_domino_direct_closure_gate.py
```

The optional `--require-visual-bottom` flag is deliberately stricter and is expected to fail until raw visual-bottom contact is made stable.

## Next Runtime Work

The next useful work is not another small random gait search. The next iteration should either:

- Train a fresh PPO continuation using the current visible-foot reward/contact metrics, not the stale hidden-support contact metrics.
- Build a cleaner scripted reference gait from the rendered CAD foot-bottom trajectories, then use that reference for behavior cloning or a reference-action curriculum.
- Fix raw `actual-cad-visual-bottom` contact by leveling the neutral CAD stance or adding a better foot collision proxy before using it for policy training.
