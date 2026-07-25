# Domino CAD Linkage DirectRLEnv Training Gate

> Historical report: these results describe the superseded direct-closure model. The current calibrated-neutral passive-linkage status and measured gates are maintained in [`../README.md`](../README.md).

This report records the Isaac Lab `DirectRLEnv` wrapper, RSL-RL training, and checkpoint playback gates for the floating Domino linkage scene.

Status: **passed for manually authored CAD-derived linkage proxy `DirectRLEnv` environments, command-aware RSL-RL checkpointing/playback, scripted foot-lift smoke tests, signed-axis 12-actuator gait search, stable 12-actuator reference-action warm starts, and short PPO continuation replays on the Domino 12-actuator action contract**.

This is still a bring-up gate, not a trained walking result. The physics model remains a controllable CAD-derived proxy linkage, but the current `DirectRLEnv` builder attaches the exported Domino STL link meshes to the moving rigid bodies by default. The visible robot is therefore actual Domino CAD geometry, while the collision/contact model is still simplified.

Historical training metrics in this report were produced before the STL visual overlay existed, so those older JSON reports show `proxy_cubes_and_foot_spheres` and `actual_cad_visual=false`. The current builder audit now reports `actual_cad_stl_visuals_on_proxy_physics` and `actual_cad_visual=true`.

If an Isaac viewport shows several dark, blocky robots, treat it as a debug/training view until this audit passes. The CAD identity gate is the single-robot actual-CAD audit: source CAD mesh prims, visible runtime STL mesh count, visible STL triangle total, and zero visible robot proxy cubes/spheres.

## Historical Verified Gate

This one-robot CAD identity gate was rerun on 2026-06-28. The local RSL-RL dependency stack used Isaac's bundled CUDA Torch runtime; the overlay dependency that mattered was `tensordict==0.8.3`, because newer `tensordict` wheels could crash against the bundled Torch version used for these runs.

| Gate | Result |
| --- | ---: |
| Raw CAD identity audit | `passed` |
| Raw CAD mesh prims | `30` |
| Raw CAD cube/sphere proxy prims | `0` |
| `DirectRLEnv` one-env smoke | `passed` |
| RSL-RL one-env training smoke | `passed` |
| RSL-RL checkpoint playback smoke | `passed` |
| RSL-RL four-env 10-iteration training gate | `passed` |
| RSL-RL four-env checkpoint playback gate | `passed` |
| Reference-action BC warm start gate | `passed` |
| Reference-action BC playback gate | `passed` |
| Reference-action PPO refinement gate | `passed` |
| Reference-action PPO refinement replay | `passed` |
| Actual CAD visuals in train/play | `true` |
| Train/play visual fidelity | `actual_cad_stl_visuals_on_proxy_physics` |
| Runtime Domino STL mesh parts | `29 / 29 visible` |
| Runtime Domino STL triangles | `135508 / 135508 visible` |
| Visible robot proxy cubes/spheres | `0 / 0` |
| Proxy visual render guard | `guide-purpose + transparent` |
| DirectRLEnv visual-validation smoke | `passed` |
| Train/play action count | `12` |
| Four-env training timesteps | `640` |
| Four-env latest checkpoint | `model_9.pt` |
| Four-env playback steps | `120` |
| Four-env playback terminations/timeouts | `0 / 0` |
| Four-env playback mean displacement velocity x | `0.009 m/s` |
| BC final clipped action MSE | `0.0013` |
| BC playback terminations/timeouts | `0 / 4` |
| BC playback final forward displacement | `~33 mm` |
| PPO refinement timesteps | `3840` |
| PPO refinement latest checkpoint | `model_29.pt` |
| PPO refinement playback terminations/timeouts | `0 / 4` |
| PPO refinement playback final forward displacement | `~32 mm` |
| Reward component diagnostics | `added` |
| Lateral drift reward reference | `fixed to initial body-reference pose` |
| Resumed PPO noise override | `fixed` |
| Current fastest actual-CAD replay | `70 deg / 2.25 Hz heading-refine PPO model_79` |
| Current fastest replay forward/lateral displacement | `~82.5 mm / ~-6.1 mm` |
| Current fastest replay final heading drift | `~0.384 rad` |
| Current balanced continuation candidate | `70 deg / 2.25 Hz heading-refine PPO model_20` |
| Balanced candidate forward/lateral displacement | `~66.4 mm / ~-0.5 mm` |
| Balanced candidate final heading drift | `~0.283 rad` |
| Latest straightness continuation result | `straighter, but slower; not retained` |
| Current forward-distance PPO candidate | `model_20 forward-preserving PPO model_59` |
| Forward-preserving candidate forward/lateral displacement | `~68.9 mm / ~2.5 mm` |
| Forward-preserving candidate final heading drift | `~0.311 rad` |
| Current retained fastest baseline | `weighted 70 deg / 2.25 Hz BC checkpoint` |
| Retained fastest baseline forward/lateral displacement | `~67.1 mm / ~3.4 mm` |
| Retained fastest baseline final heading drift | `~0.292 rad` |
| Current low-yaw comparison baseline | `right/left phase trim -15 deg weighted BC checkpoint` |
| Low-yaw baseline forward/lateral displacement | `~50.1 mm / ~4.5 mm` |
| Low-yaw baseline final heading drift | `~0.142 rad` |
| Current intermediate symmetry baseline | `right/left phase trim -6 deg weighted BC checkpoint` |
| Intermediate baseline forward/lateral displacement | `~59.3 mm / ~3.4 mm` |
| Intermediate baseline final heading drift | `~0.219 rad` |
| Current lower-drift learned baseline | `weighted 60 deg / 2.0 Hz BC checkpoint` |
| Lower-drift baseline forward/lateral displacement | `~63.4 mm / ~-1.9 mm` |
| Latest PPO refinement result | `model_59 improves forward/lateral versus retained BC; model_20 remains cleaner balanced continuation` |
| Latest yaw/lateral PPO probe | `stable, not retained` |
| Latest teacher timing result | `70 deg / 2.25 Hz is the cleanest faster timing tested so far` |

The four-env gate proves that the current Isaac Lab/RSL-RL path can collect rollouts, update PPO, save multiple checkpoints, reload the latest checkpoint, and play it back through the 12-action Domino CAD-visual environment. It does **not** prove that the policy walks well yet; the replayed checkpoint remains far below the `0.03 m/s` forward command and should be treated as a working training pipeline checkpoint, not as a successful gait.

The reference-action behavior-cloned checkpoint proves the policy can copy the existing near-straight 12-action scripted gait candidate through the 73-float observation variant. It stays upright to normal timeout in playback, but it still only reaches about `33 mm` forward displacement over 240 playback steps and should be treated as a warm-start baseline for PPO/refinement rather than a final locomotion policy.

A short PPO refinement was then resumed from the same BC checkpoint with the 12-channel reference action appended to the observation. It stayed physically stable and replayed without fall terminations, but it did **not** improve locomotion: mean forward displacement fell slightly from about `32.7 mm` for BC-only playback to about `32.1 mm`, and maximum body tilt increased from about `12.5 deg` to about `14.7 deg`. The refinement is therefore a pipeline check, not a better policy. The next PPO pass needs reward-component logging before changing reward scales further.

Reward-component reporting has now been added to the env, training reports, and playback reports. That exposed two implementation issues:

- The lateral-drift penalty was measured from the Isaac environment origin, not from the robot's initial body-reference pose. Because the Domino body reference is not centered exactly at the env origin, this created a false lateral penalty of about `-2.01` per step in one refinement replay. The fixed reward now measures sideways displacement relative to the initial body-reference Y position; the same replay reports about `-0.00025`.
- `--init-noise-std` did not take effect after loading BC checkpoints because RSL-RL restored the checkpoint's stored action standard deviation. The train runner now reapplies the requested noise after checkpoint load and records `effective_action_noise_std` in the JSON report.

After those fixes, low-noise PPO refinement from the 45-degree / 1.5 Hz BC checkpoint trained stably, but still did not improve movement. Matched replay results:

| Checkpoint | Forward displacement | Lateral displacement | Fall terminations |
| --- | ---: | ---: | ---: |
| 45 deg / 1.5 Hz BC | `32.7 mm` | `0.8 mm` | `0` |
| 45 deg / 1.5 Hz low-noise PPO refine | `32.6 mm` | `1.6 mm` | `0` |

The older 60-degree / 2.0 Hz near-straight BC checkpoint was replayed through the current actual-CAD visual builder and fixed-drift reward code. At this stage it was the fastest verified learned baseline, although it drifted sideways:

| Checkpoint | Forward displacement | Lateral displacement | Fall terminations |
| --- | ---: | ---: | ---: |
| 60 deg / 2.0 Hz BC | `60.5 mm` | `-15.5 mm` | `0` |
| 60 deg / 2.0 Hz low-noise PPO refine | `57.8 mm` | `-11.3 mm` | `0` |

The 60-degree PPO refinement reduced lateral drift a little, but lost forward displacement. That result is retained as evidence that PPO refinement can easily trade away useful forward motion if the reward balance is wrong.

Playback now records per-actuator reference-action error using the executed action after the env's `[-1, 1]` action clamp. Replaying the 60-degree / 2.0 Hz BC checkpoint on CUDA with the matched `random_001` teacher stayed stable and produced the same displacement pattern as the previous baseline. The channel-level error shows the copied policy is not equally accurate across the 12 actuators:

| Action role | Mean absolute reference error | Mean RMSE | Max absolute error |
| --- | ---: | ---: | ---: |
| Shoulder hip ab/ad | `0.0908` | `0.1023` | `0.2693` |
| Lower linkage drive | `0.1212` | `0.1460` | `0.4509` |
| Upper pitch drive | `0.1469` | `0.1684` | `0.4483` |

The worst individual channels were `dom_p_21_1_upper_pitch` (`0.1945` mean absolute error), `dom_p_4_1_upper_pitch` (`0.1679`), `dom_p_12_1_lower_linkage` (`0.1562`), and `dom_p_12_1_shoulder_ab_ad` (`0.1489`). That suggests the lateral-drift gap between the scripted teacher and the BC policy is not just a shoulder-sign problem; the pitch/linkage channels are losing enough timing/amplitude accuracy to change contact and body yaw. A CPU playback run of the same checkpoint diverged and is not comparable to the CUDA baseline, so CUDA playback is the current authority for this checkpoint.

The train runner now supports weighted reference-action BC by action role. A 600-step weighted BC-only run with shoulder/lower/upper weights `1.0 / 1.5 / 2.0` reduced the executed-action reference error substantially. The 60-degree / 2.0 Hz run became the first low-drift learned baseline:

| Checkpoint | Forward displacement | Lateral displacement | Mean reference-action MSE | Fall terminations |
| --- | ---: | ---: | ---: | ---: |
| Older 60 deg / 2.0 Hz BC | `60.5 mm` | `-15.5 mm` | about `0.0214` | `0` |
| Weighted 60 deg / 2.0 Hz BC | `63.4 mm` | `-1.9 mm` | about `0.0017` | `0` |

Weighted BC per-role executed-action errors are now low enough that the checkpoint follows the teacher more faithfully:

| Action role | Mean absolute reference error | Mean RMSE | Max absolute error |
| --- | ---: | ---: | ---: |
| Shoulder hip ab/ad | `0.0237` | `0.0295` | `0.0939` |
| Lower linkage drive | `0.0348` | `0.0441` | `0.1289` |
| Upper pitch drive | `0.0381` | `0.0473` | `0.1762` |

A follow-up scripted-teacher refinement search around the 60-degree / 2.0 Hz `random_001` teacher tested 16 local perturbations with stronger forward weighting and lateral-drift scoring. The best batch candidate, `refined_006`, did not beat the original teacher when replayed as a single candidate:

| Scripted teacher | Forward displacement | Lateral displacement | Fall termination |
| --- | ---: | ---: | ---: |
| `random_001` base | `60.7 mm` | `-1.5 mm` | `false` |
| `refined_006` | `59.7 mm` | `-29.7 mm` | `false` |

The search also showed that large parallel candidate batches can be misleading for this prototype: the base candidate appeared terminated in the 16-env batch, but replayed cleanly as a single candidate. Future teacher-search winners should be verified with a single-candidate replay before being used for behavior cloning.

A timing sweep around the same `random_001` teacher then tested larger action scales and faster reference timing. The `70 deg / 2.25 Hz` teacher replay was the cleanest improvement over the 60-degree baseline: about `64.6 mm` forward displacement with about `-0.7 mm` lateral displacement. The `80 deg / 2.0 Hz` teacher moved farther forward but drifted laterally too much for the next baseline.

Weighted BC on the `70 deg / 2.25 Hz` teacher now gives the fastest learned CUDA playback so far:

| Checkpoint | Forward displacement | Lateral displacement | Mean reference-action MSE | Fall terminations |
| --- | ---: | ---: | ---: | ---: |
| Weighted 60 deg / 2.0 Hz BC | `63.4 mm` | `-1.9 mm` | about `0.0017` | `0` |
| Weighted 70 deg / 2.25 Hz BC | `67.1 mm` | `3.4 mm` | about `0.0017` | `0` |

The 70-degree checkpoint is now the fastest learned baseline. The 60-degree checkpoint remains useful as the lower-drift baseline because its lateral displacement is smaller on this playback gate.

A short 30-iteration PPO refinement was then resumed from the weighted 70-degree / 2.25 Hz BC checkpoint. This run reduced the reference-action tether, increased forward progress and velocity-tracking rewards, and softened the swing-contact penalty. It stayed upright and produced a faster final checkpoint, but did not solve locomotion quality because the extra forward movement came with much larger lateral drift:

| Checkpoint | Forward displacement | Lateral displacement | Mean reference-action MSE | Fall terminations |
| --- | ---: | ---: | ---: | ---: |
| Weighted 70 deg / 2.25 Hz BC | `67.1 mm` | `3.4 mm` | about `0.0017` | `0` |
| Forward-tether PPO `model_16` | `61.7 mm` | `12.0 mm` | about `0.0046` | `0` |
| Forward-tether PPO `model_29` | `70.2 mm` | `22.9 mm` | about `0.0084` | `0` |

The final PPO checkpoint is useful because it proves the policy can be pushed to move farther while keeping the linkage stable, but it is not the retained baseline. The retained fastest baseline remains the weighted 70-degree BC checkpoint; the retained lower-drift baseline remains the weighted 60-degree BC checkpoint. The next reward iteration needs stronger lateral/yaw control without deleting the forward-progress gain.

Heading-drift reporting and an explicit heading-drift reward term were added after the forward-tether run. The key distinction is that yaw-rate tracking only penalizes current rotation rate; it does not penalize a robot that has already turned away from its initial heading. The new `yaw_drift_sq` term measures body-reference heading drift from the commanded heading trajectory, while playback now records mean absolute heading drift and final heading drift.

Replaying the retained 70-degree BC checkpoint with the new diagnostic showed that it is still the best retained learned baseline, but it is not perfectly straight: mean final heading drift is about `0.292 rad` over the 240-step CUDA playback horizon. A short PPO probe then resumed from that checkpoint with stronger lateral and heading penalties. It stayed upright, but did not produce a better usable policy:

| Checkpoint | Forward displacement | Lateral displacement | Final heading drift | Mean abs heading drift | Fall terminations |
| --- | ---: | ---: | ---: | ---: | ---: |
| Weighted 70 deg / 2.25 Hz BC replay | `67.1 mm` | `3.4 mm` | `0.292 rad` | `0.194 rad` | `0` |
| Yaw/lateral PPO `model_0` | `64.9 mm` | `4.0 mm` | `0.277 rad` | `0.190 rad` | `0` |
| Yaw/lateral PPO `model_19` | `68.0 mm` | `13.1 mm` | `0.314 rad` | `0.189 rad` | `0` |

The early checkpoint slightly reduced heading drift but lost forward motion, while the final checkpoint gained a little forward motion but allowed too much sideways drift and worse final heading. This run is therefore kept as a reward-tuning diagnostic, not as a replacement for the retained BC baseline.

The scripted teacher replay path now uses the same heading-drift diagnostic. A loader bug was fixed in `run_domino_cad_linkage_gait_search.py`: candidate JSON files written as gait-search reports with `best.candidate` are now loaded correctly instead of falling back to the default reference gait. This matters because the retained `random_001` teacher is stored as a search report.

Replaying the actual `70 deg / 2.25 Hz random_001` teacher as scripted actions shows that the reference itself is stable and laterally clean, but already has some heading drift:

| Scripted teacher | Forward displacement | Lateral displacement | Final heading drift | Mean abs heading drift | Max foot clearance | Fall termination |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `random_001` | `64.6 mm` | `-0.7 mm` | `0.255 rad` | `0.189 rad` | `38.4 mm` | `false` |
| Heading-refined `refined_010` | `64.6 mm` | `28.0 mm` | `0.124 rad` | `0.055 rad` | `9.0 mm` | `false` |

The heading-refined candidate proves the search can reduce yaw drift, but it does so by walking sideways and losing useful swing clearance. It is not a replacement teacher. The next teacher search should treat forward displacement, lateral displacement, heading drift, and clearance as a combined objective, then replay candidate winners one at a time before behavior cloning. Batch candidate results are useful for exploration, but single-candidate replay remains the authority.

The gait-search runner now supports saving a ranked top-candidate shortlist and optional lateral/heading soft caps in the score. Two follow-up searches were run around `random_001`:

- A balanced search found a fast candidate, `refined_009`, but single replay confirmed it is a fast curved gait rather than a better straight teacher.
- A stricter search with lateral and heading soft caps did not find a clean improvement. Its best single-replayed candidate, `refined_013`, reduced heading drift slightly but moved less forward and drifted sideways too much.

Single-candidate replay comparison:

| Candidate | Forward displacement | Lateral displacement | Final heading drift | Mean abs heading drift | Max foot clearance | Fall termination |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Current `random_001` | `64.6 mm` | `-0.7 mm` | `0.255 rad` | `0.189 rad` | `38.4 mm` | `false` |
| Balanced search `refined_009` | `91.1 mm` | `-18.4 mm` | `0.424 rad` | `0.256 rad` | `37.1 mm` | `false` |
| Balanced search `refined_014` | `37.1 mm` | `15.8 mm` | `0.023 rad` | `0.056 rad` | `36.9 mm` | `false` |
| Strict search `refined_013` | `56.5 mm` | `18.8 mm` | `0.220 rad` | `0.100 rad` | `23.4 mm` | `false` |

The current `random_001` teacher remains the best verified balanced teacher: it has the cleanest lateral displacement and strong clearance, even though it still has heading drift. The next useful search direction is not simply more PPO; it is either a broader teacher search with explicit straightness constraints or a small analytic correction to shoulder/leg phase symmetry, followed by single-candidate replay and then weighted BC.

A deterministic symmetry-family search was then added. This keeps the current teacher fixed and applies controlled shoulder-bias, shoulder-amplitude, shoulder-phase, and paired leg-phase variants instead of randomizing the whole gait. The first useful variant was a right/left leg-phase trim of `-15 deg`: it does not replace `random_001` as the fastest/balanced teacher, but it is valuable because it directly trades speed for heading stability. A finer trim sweep then found `-6 deg` as the better middle-ground teacher: it gives up less forward displacement while reducing heading drift compared with the original `random_001`.

| Candidate or policy | Forward displacement | Lateral displacement | Final heading drift | Mean abs heading drift | Mean reference-action MSE | Fall terminations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Teacher `random_001` | `64.6 mm` | `-0.7 mm` | `0.255 rad` | `0.189 rad` | n/a | `0` |
| Teacher right/left trim `-6 deg` | `56.3 mm` | `0.3 mm` | `0.188 rad` | `0.157 rad` | n/a | `0` |
| Teacher right/left trim `-15 deg` | `44.6 mm` | `5.6 mm` | `0.096 rad` | `0.110 rad` | n/a | `0` |
| Weighted BC from `random_001` | `67.1 mm` | `3.4 mm` | `0.292 rad` | `0.194 rad` | `0.0017` | `0` |
| Weighted BC from right/left trim `-6 deg` | `59.3 mm` | `3.4 mm` | `0.219 rad` | `0.165 rad` | `0.0015` | `0` |
| Weighted BC from right/left trim `-15 deg` | `50.1 mm` | `4.5 mm` | `0.142 rad` | `0.128 rad` | `0.0014` | `0` |

The `-6 deg` BC checkpoint is the current intermediate symmetry baseline. The `-15 deg` BC checkpoint remains the low-yaw comparison. Both prove that behavior cloning can preserve straighter reference gaits through the 12-actuator policy path, but both are slower than the retained fastest baseline, so they are comparison baselines rather than the main policy.

Current report artifacts:

- `out/cad_identity/domino_raw_cad_audit.json`
- `out/cad_identity/direct_rl_env_single_domino_actual_cad.json`
- `out/cad_identity/direct_rl_env_single_domino_visual_validation_smoke.json`
- `out/cad_identity/rsl_train_smoke.json`
- `out/cad_identity/rsl_play_smoke.json`
- `out/cad_identity/rsl_train_4env_10iter_actual_cad.json`
- `out/cad_identity/rsl_play_4env_10iter_actual_cad.json`
- `out/cad_identity/rsl_train_bc_4env_actual_cad.json`
- `out/cad_identity/rsl_play_bc_4env_actual_cad.json`
- `out/cad_identity/rsl_train_bc_refine_4env_actual_cad.json`
- `out/cad_identity/rsl_play_bc_refine_4env_actual_cad.json`
- `out/cad_identity/rsl_play_bc_fixed_drift_reward_terms_4env_actual_cad.json`
- `out/cad_identity/rsl_train_bc_refine_low_noise_fixed_drift_v2_4env_actual_cad.json`
- `out/cad_identity/rsl_play_bc_refine_low_noise_fixed_drift_v2_4env_actual_cad.json`
- `out/cad_identity/rsl_play_scale60_freq20_bc_fixed_drift_matched_reward_actual_cad.json`
- `out/cad_identity/rsl_play_scale60_freq20_bc_action_error_diag_cuda_actual_cad.json`
- `out/cad_identity/rsl_train_scale60_freq20_weighted_bc_actual_cad.json`
- `out/cad_identity/rsl_play_scale60_freq20_weighted_bc_actual_cad.json`
- `out/cad_identity/rsl_train_scale70_freq225_weighted_bc_actual_cad.json`
- `out/cad_identity/rsl_play_scale70_freq225_weighted_bc_actual_cad.json`
- `out/cad_identity/rsl_train_scale70_freq225_forward_tether_refine_actual_cad.json`
- `out/cad_identity/rsl_play_scale70_freq225_forward_tether_refine_model16_actual_cad.json`
- `out/cad_identity/rsl_play_scale70_freq225_forward_tether_refine_model29_actual_cad.json`
- `out/cad_identity/rsl_play_scale70_freq225_weighted_bc_yawdiag_actual_cad.json`
- `out/cad_identity/rsl_train_scale70_freq225_yaw_lateral_refine_actual_cad.json`
- `out/cad_identity/rsl_play_scale70_freq225_yaw_lateral_refine_model0_actual_cad.json`
- `out/cad_identity/rsl_play_scale70_freq225_yaw_lateral_refine_model19_actual_cad.json`
- `out/cad_identity/domino_cad_linkage_teacher_replay_scale70_freq225_yawdiag_actual_cad.json`
- `out/cad_identity/domino_cad_linkage_gait_search_scale70_freq225_heading_refine_seed314_16_actual_cad.json`
- `out/cad_identity/domino_cad_linkage_gait_replay_scale70_freq225_heading_refined010_single_actual_cad.json`
- `out/cad_identity/teacher_grid/teacher_heading_refined_scale70_freq225_seed314.json`
- `out/cad_identity/domino_cad_linkage_gait_search_scale70_freq225_balanced_refine_seed615_24_actual_cad.json`
- `out/cad_identity/balanced_refine_seed615_single_replays/rank_01_refined_009_single_actual_cad.json`
- `out/cad_identity/balanced_refine_seed615_single_replays/rank_02_refined_014_single_actual_cad.json`
- `out/cad_identity/domino_cad_linkage_gait_search_scale70_freq225_strict_refine_seed911_24_actual_cad.json`
- `out/cad_identity/domino_cad_linkage_gait_replay_scale70_freq225_strict_refined013_single_actual_cad.json`
- `out/cad_identity/teacher_grid/teacher_balanced_refined_scale70_freq225_seed615.json`
- `out/cad_identity/teacher_grid/teacher_strict_refined_scale70_freq225_seed911.json`
- `out/cad_identity/domino_cad_linkage_gait_search_scale70_freq225_symmetry_seed_base_32_actual_cad.json`
- `out/cad_identity/symmetry_base32_single_replays/rank_01_random_001_right_left_phase_trim_-15deg_single_actual_cad.json`
- `out/cad_identity/teacher_grid/teacher_symmetry_scale70_freq225_base32.json`
- `out/cad_identity/rsl_train_scale70_freq225_symmetry_trim_m15_weighted_bc_actual_cad.json`
- `out/cad_identity/rsl_play_scale70_freq225_symmetry_trim_m15_weighted_bc_actual_cad.json`
- `out/cad_identity/domino_cad_linkage_gait_search_scale70_freq225_symmetry_fine_trim_52_actual_cad.json`
- `out/cad_identity/symmetry_fine_trim_single_replays/rank_01_random_001_right_left_phase_trim_-6deg_single_actual_cad.json`
- `out/cad_identity/teacher_grid/teacher_symmetry_fine_trim_scale70_freq225.json`
- `out/cad_identity/rsl_train_scale70_freq225_symmetry_trim_m6_weighted_bc_actual_cad.json`
- `out/cad_identity/rsl_play_scale70_freq225_symmetry_trim_m6_weighted_bc_actual_cad.json`
- `out/cad_identity/rsl_train_scale60_freq20_low_noise_refine_fixed_drift_4env_actual_cad.json`
- `out/cad_identity/rsl_play_scale60_freq20_low_noise_refine_fixed_drift_4env_actual_cad.json`
- `out/cad_identity/domino_cad_linkage_gait_search_scale60_freq20_refine_seed231_16_actual_cad.json`
- `out/cad_identity/domino_cad_linkage_gait_replay_scale60_freq20_base_single_actual_cad.json`
- `out/cad_identity/domino_cad_linkage_gait_replay_scale60_freq20_refined006_single_actual_cad.json`
- `out/cad_identity/teacher_grid/teacher_random001_scale70_freq225.json`
- `out/cad_identity/domino_actual_cad_linkage_visual_audit.json`
- `out/cad_identity/domino_actual_cad_linkage_visual_audit.usda`
- `out/cad_identity/domino_actual_cad_stl_no_render_preview.png`
- `out/cad_identity/next_policy/rsl_train_trim_m6_yaw_lateral_refine.json`
- `out/cad_identity/next_policy/rsl_play_trim_m6_yaw_lateral_refine.json`
- `out/cad_identity/next_policy/compare_trim_m6_yaw_lateral_refine.json`

Current actual-CAD visual audit:

| Check | Result |
| --- | ---: |
| Source CAD mesh prims | `30` |
| Source CAD cube/sphere prims | `0 / 0` |
| STL mesh visuals attached | `29` |
| Visible STL mesh visuals | `29` |
| STL triangle count | `135508` |
| Visible STL triangle count | `135508` |
| Actuator drive count | `12` |
| Revolute joint count | `28` |
| Rigid body count | `21` |
| Hidden proxy cubes | `21` |
| Hidden proxy foot spheres | `4` |
| Visible proxy cubes/spheres | `0 / 0` |
| Proxy visual render guard | `guide-purpose + transparent` |
| Runtime visual bounds | about `0.611 m x 0.238 m x 0.391 m` |

The proxy bodies are still present for physics/collision scaffolding, but the robot visuals are the Domino STL meshes. When actual-CAD visuals are enabled, the proxy cubes/spheres are invisible, guide-purpose, and transparent. The no-render audit intentionally builds the robot without a ground plane so visible cube/sphere counts refer only to robot proxy geometry. Single-robot actual-CAD audit renders or previews are the only visual evidence that should be used for portfolio screenshots; multi-env training viewports can be misleading and are not treated as CAD identity proof.

## Action Contract

The env exposes the full 12-actuator Domino command layout. It is not an 8-action linkage-only model.

| Group | Count |
| --- | ---: |
| Shoulder hip ab/ad drives | `4` |
| Lower linkage drives | `4` |
| Upper linkage/pitch drives | `4` |
| Total action channels | `12` |

Per leg the action order is:

1. Shoulder hip ab/ad actuator.
2. Lower two-bar/four-bar linkage-drive actuator.
3. Upper linkage/pitch actuator.

The runtime report records the exact per-leg order:

| Leg module | Action 1 | Action 2 | Action 3 |
| --- | --- | --- | --- |
| `dom_p_4_1` | `dom_p_4_1_shoulder_ab_ad` | `dom_p_4_1_lower_linkage` | `dom_p_4_1_upper_pitch` |
| `dom_p_12_1` | `dom_p_12_1_shoulder_ab_ad` | `dom_p_12_1_lower_linkage` | `dom_p_12_1_upper_pitch` |
| `dom_p_25_1` | `dom_p_25_1_shoulder_ab_ad` | `dom_p_25_1_lower_linkage` | `dom_p_25_1_upper_pitch` |
| `dom_p_21_1` | `dom_p_21_1_shoulder_ab_ad` | `dom_p_21_1_lower_linkage` | `dom_p_21_1_upper_pitch` |

The CAD builder validates that this action list matches the shared Domino action contract before simulation starts. The env, drive target helper, and scripted gait-search runner now also fail fast if they receive or generate anything other than 12 actions per environment. The gait tooling uses the same per-leg order: shoulder, lower linkage, upper linkage/pitch.

## Observation And Reward State

The CAD-linkage env now exposes a 61-float policy observation:

- Body-reference position, orientation, linear velocity, and angular velocity.
- Projected gravity in the body frame for tilt/fall sensing.
- Commanded x velocity, y velocity, and yaw rate.
- A simple gait phase pair: sine and cosine of the configured phase clock.
- Four CAD-derived foot-proxy positions from the lower-closure linkage pivots.
- Four simple foot-contact flags based on the CAD foot-proxy sphere height.
- The previous 12 normalized actions.
- The 12 normalized drive targets.

The reward currently uses alive bonus, body height, flat orientation, vertical velocity, angular velocity, command x/y velocity tracking, lateral body drift, yaw-rate tracking, heading drift from the commanded yaw trajectory, action size, action rate, stance/swing contact shaping, swing-foot clearance, and optional four-foot support. The done logic separates fall termination from normal episode timeout.

## DirectRLEnv Smoke

Command:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_domino_cad_linkage_env_smoke.py `
  --headless `
  --num-envs 4 `
  --steps 120 `
  --command-x-m-s 0.03 `
  --gait-frequency-hz 1.0 `
  --report-path <output-folder>/domino_cad_linkage_env_smoke_4env_command_report.json
```

Result:

| Metric | Result |
| --- | ---: |
| Geometry | `domino-four-12-floating-body` |
| Visual fidelity | `proxy_cubes_and_foot_spheres` |
| Actual CAD visual attached | `false` |
| Environments | `4` |
| Steps | `120` |
| Action dimension | `12` |
| Observation dimension | `61` |
| Forward command | `0.03 m/s` |
| Terminations | `0` |
| Timeouts | `0` |
| Minimum body-reference height | `0.128856 m` |
| Maximum body-reference speed | `0.050552 m/s` |

The smoke applied nonzero synthetic commands to every actuator channel. The maximum absolute action per channel was approximately `0.5`, so no shoulder or linkage-drive channel was left inactive.

## RSL-RL Command-Aware Training

Command:

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

Result:

| Metric | Result |
| --- | ---: |
| Geometry | `domino-four-12-floating-body` |
| Visual fidelity | `proxy_cubes_and_foot_spheres` |
| Actual CAD visual attached | `false` |
| Environments | `4` |
| PPO iterations | `10` |
| Rollout steps per env | `16` |
| Total timesteps | `640` |
| Actor input dimension | `61` |
| Actor output dimension | `12` |
| Forward command | `0.03 m/s` |
| Latest checkpoint | `model_9.pt` |

Training stayed finite and wrote `model_0.pt` through `model_9.pt`. This proves the command-aware CAD-linkage env can train and checkpoint through RSL-RL, but the run is intentionally short.

## RSL-RL Playback

Command:

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

Result:

| Metric | Result |
| --- | ---: |
| Checkpoint | `model_9.pt` |
| Playback steps | `240` |
| Environments | `4` |
| Action dimension | `12` |
| Observation dimension | `61` |
| Forward command | `0.03 m/s` |
| Mean foot contacts per env | `4.0` |
| Fall terminations | `0` |
| Episode timeouts | `4` |
| Minimum body-reference height | `0.128838 m` |
| Maximum body tilt | `0.538503 deg` |
| Maximum absolute action | `0.381143` |
| Final x displacement | about `0.0008 m` |

The checkpoint plays back stably and keeps all four environments upright until normal timeout. It does not yet walk forward in response to the 0.03 m/s command.

## Locomotion Reward Experiments

The env now has a locomotion-tuning path in addition to the stand gate:

- `--action-scale-deg` controls the drive target offset used by normalized policy actions.
- `--command-progress-reward-scale` adds reward for velocity in the commanded planar direction.
- `--command-velocity-tracking-reward-scale` adds a positive velocity-tracking reward.
- `--gait-contact-reward-scale`, `--stance-contact-reward-scale`, `--swing-contact-penalty-scale`, and `--foot-clearance-reward-scale` shape a simple diagonal trot phase.
- Playback now reports mean body velocity, command velocity error, gait contact match, stance contact, swing contact, and swing clearance.

The important result from these runs is that the current CAD linkage **can** lift foot proxies when driven with enough range, but the short PPO policies still choose the safe all-feet-contact solution.

Scripted 30-degree action-scale smoke:

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

Result:

| Metric | Result |
| --- | ---: |
| Action dimension | `12` |
| Observation dimension | `61` |
| Action scale | `30 deg` |
| Fall terminations | `0` |
| Mean foot contacts per env | `3.570833` |
| Max foot-proxy clearance | `0.017029 m` |
| Minimum body-reference height | `0.121941 m` |

This proves the authored foot proxies are not permanently pinned to the ground. The current model can generate swing clearance under scripted 12-channel action input.

Best low-noise 30-degree PPO playback:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_domino_cad_linkage_rsl_rl_play.py `
  --headless `
  --num-envs 4 `
  --steps 240 `
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
  --report-path <output-folder>/domino_cad_linkage_rsl_rl_play_4env_locomotion_scale30_noise015_60iter_report.json
```

Result:

| Metric | Result |
| --- | ---: |
| Checkpoint | `model_59.pt` |
| PPO timesteps | `7680` |
| Action scale | `30 deg` |
| Initial policy noise | `0.15` |
| Fall terminations | `0` |
| Mean foot contacts per env | `4.0` |
| Mean gait contact match | `0.5` |
| Mean stance contact | `1.0` |
| Mean swing contact | `1.0` |
| Mean swing clearance | `0.0 m` |
| Mean x velocity | about `-0.001 m/s` |
| Mean planar command error | about `0.081 m/s` |

This checkpoint is stable, but it is not locomotion. It keeps all feet planted and does not track the 0.08 m/s forward command.

## Scripted Gait Search

Command:

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

Result:

| Metric | Result |
| --- | ---: |
| Candidate count | `16` |
| Action dimension | `12` |
| Best candidate | `random_014` |
| Final x displacement | `0.016063 m` |
| Final y displacement | `-0.013024 m` |
| Mean foot contacts | `3.604167` |
| Max foot-proxy clearance | `0.018386 m` |
| Minimum body-reference height | `0.122042 m` |
| Maximum body tilt | `3.6629 deg` |
| Fall termination | `false` |

The best seed-7 scripted candidate uses all four shoulder drives and all eight linkage drives. It creates small stable forward displacement and measurable foot unloading, but the displacement is still far below the `0.08 m/s` command target. This is a reference-gait search result, not a trained policy.

Follow-up local refinement around that candidate improved the scripted reference:

| Metric | Result |
| --- | ---: |
| Candidate count | `32` |
| Best candidate | `refined_001` |
| Final x displacement | `0.032443 m` |
| Final y displacement | `-0.013107 m` |
| Mean foot contacts | `3.520833` |
| Max foot-proxy clearance | `0.039531 m` |
| Minimum body-reference height | `0.119986 m` |
| Maximum body tilt | `9.3857 deg` |
| Fall termination | `false` |

After correcting the CAD-axis signs used by the authored PhysX joints, the strongest signed-axis reference is now:

| Metric | Result |
| --- | ---: |
| Candidate count | `32` |
| Best candidate | `refined_016` |
| Final x displacement | `0.036522 m` |
| Final y displacement | `0.029281 m` |
| Mean foot contacts | `3.6375` |
| Max foot-proxy clearance | `0.014161 m` |
| Minimum body-reference height | `0.096726 m` |
| Maximum body tilt | `8.5992 deg` |
| Fall termination | `false` |

This signed-axis candidate was the first useful 73-observation reference. It still has lateral drift and is not a walking controller, but it gave PPO a repeatable 12-channel target that includes every shoulder, lower-linkage, and upper-linkage/pitch drive.

## Reference-Guided PPO

The env now supports an optional scripted action-prior reward and an optional 12-float reference-action observation. These are curriculum tools for getting PPO out of the all-feet-contact local optimum; they are not required by the base 61-observation environment.

The first reference-prior PPO run used the refined candidate as a reward target but kept the base 61-observation space. Playback stayed upright but still kept nearly all feet planted:

| Metric | Result |
| --- | ---: |
| Environments | `16` train / `4` playback |
| PPO iterations | `60` |
| Actor input dimension | `61` |
| Actor output dimension | `12` |
| Mean reference-action tracking | about `0.1103` |
| Mean foot contacts per env | `3.996875` |
| Mean swing clearance | `0.0 m` |
| Mean x velocity | about `0.0011 m/s` |
| Fall terminations | `0` |

The second curriculum run appended the current 12-channel scripted reference action target to the observation, giving a 73-float actor input. Playback stayed upright and began unloading feet:

| Metric | Result |
| --- | ---: |
| Environments | `16` train / `4` playback |
| PPO iterations | `60` |
| Actor input dimension | `73` |
| Actor output dimension | `12` |
| Mean reference-action tracking | about `0.3262` |
| Mean foot contacts per env | `3.108333` |
| Mean swing contact | `0.76875` |
| Mean swing clearance | `0.001687 m` |
| Mean x velocity | about `0.0013 m/s` |
| Mean planar command error | about `0.0824 m/s` |
| Fall terminations | `0` |

This is progress because the learned policy now produces some swing-foot unloading through the real 12-actuator Domino action contract in the CAD-derived linkage proxy. It is still not useful locomotion: forward velocity remains far below the command and the body drifts laterally.

For historical context, an earlier pre-signed-axis stronger forward-progress pass reached better foot unloading, but still not command-following locomotion. The resumed run was interrupted before writing a final training report, so this entry is based on playback of the newest available checkpoint from that older run:

| Metric | Result |
| --- | ---: |
| Playback checkpoint | `model_73.pt` |
| Actor input dimension | `73` |
| Actor output dimension | `12` |
| Mean reference-action tracking | about `0.1250` |
| Mean foot contacts per env | `3.016667` |
| Mean swing contact | `0.739583` |
| Mean swing clearance | `0.011753 m` |
| Mean x velocity | about `0.0043 m/s` |
| Mean planar command error | about `0.0772 m/s` |
| Final x displacement over playback | about `0.022 m` |
| Fall terminations | `0` |

That older checkpoint confirmed the policy path could lift/unload feet through all 12 actuators, but it still mostly oscillated in place. The current signed-axis warm-start path below is the cleaner baseline for further policy work.

The current best signed-axis warm-start path uses supervised reference-action behavior cloning before PPO. The supervised step trains the actor to copy the appended 12-channel reference target directly, with a small raw-output penalty so the policy does not depend on action clipping. A BC-only 8-env run produced:

| Metric | Result |
| --- | ---: |
| Actor input dimension | `73` |
| Actor output dimension | `12` |
| BC steps | `300` |
| Final raw action MSE | `0.001572` |
| Final clipped action MSE | `0.001572` |
| Final raw max action | `0.885766` |

Playback of that `model_bc.pt` checkpoint stayed stable and tracked the signed-axis reference:

| Metric | Result |
| --- | ---: |
| Playback steps | `240` |
| Mean reference-action tracking | about `0.8476` |
| Mean reference-action MSE | about `0.02117` |
| Mean foot contacts per env | `3.6625` |
| Mean swing clearance | `0.001047 m` |
| Final x displacement | about `0.0373 m` |
| Final y displacement | about `0.0325 m` |
| Fall terminations | `0` |

A 40-iteration PPO refinement from that BC checkpoint, using the same 12-actuator signed-axis reference and a lower PPO learning rate, stayed stable:

| Metric | Result |
| --- | ---: |
| Environments | `16` train / `4` playback |
| PPO timesteps | `10240` |
| Actor input dimension | `73` |
| Actor output dimension | `12` |
| Mean reference-action tracking | about `0.8684` |
| Mean reference-action MSE | about `0.01764` |
| Mean foot contacts per env | `3.579167` |
| Mean swing clearance | `0.001424 m` |
| Final x displacement | about `0.0350 m` |
| Final y displacement | about `0.0278 m` |
| Fall terminations | `0` |

That was the cleanest first signed-axis policy-training gate: all 12 actuator outputs were present and bounded, the closed-linkage physics stayed finite, and playback did not fall. It is still not command-following locomotion; the body continued to drift and the measured mean forward velocity remained far below the `0.08 m/s` command. The later near-straight, higher-authority BC checkpoint below is now the stronger baseline.

## Displacement-Reward Pass

The reward path now supports command tracking from body-reference position delta instead of only the PhysX-reported linear velocity. This matters because playback showed a mismatch: the PhysX x velocity could be near zero or negative while the body-reference displacement was clearly positive. The playback report now includes both:

- `mean_body_reference_velocity_m_s`: the PhysX-reported body velocity.
- `mean_body_reference_displacement_velocity_m_s`: the frame-to-frame body-reference displacement divided by env step time.
- `mean_planar_displacement_command_velocity_error_m_s`: command error computed from the displacement-derived velocity.

Replaying the previous signed-axis BC/PPO checkpoint with these diagnostics showed about `0.0073 m/s` actual forward displacement velocity against the `0.08 m/s` command. A short PPO refinement using displacement-derived reward, stronger forward progress, and lateral-drift penalty produced:

| Metric | Result |
| --- | ---: |
| Environments | `16` train / `4` playback |
| PPO timesteps | `12800` |
| Actor input dimension | `73` |
| Actor output dimension | `12` |
| Mean reference-action tracking | about `0.8233` |
| Mean foot contacts per env | `3.729167` |
| Mean swing clearance | `0.000630 m` |
| Mean displacement-derived x velocity | about `0.00810 m/s` |
| Mean displacement-command error | about `0.08063 m/s` |
| Final x displacement | about `0.0389 m` |
| Final y displacement | about `0.0389 m` |
| Fall terminations | `0` |

This is only a small forward improvement, and lateral drift became worse. A follow-up low-drift scripted-reference search found a candidate with less lateral drift in the open-loop reference (`33.7 mm` x, `23.7 mm` y), but PPO from that reference did not improve the learned policy. Playback ended around `36.6 mm` x and `36.1 mm` y, with larger raw action magnitude. The displacement-reward path is useful because it exposes the real command-tracking gap.

## Near-Straight Behavior-Cloned Baselines

A later pass selected a near-straight signed-axis teacher from the seed-211 search rather than the higher-drift `refined_016` candidate. The selected `random_001` candidate produced about `31.6 mm` x displacement and only about `0.9 mm` y displacement at the original 30-degree scale. Running the same teacher at 45-degree action scale and 1.5 Hz improved open-loop displacement to about `48.9 mm` x with about `-1.3 mm` y drift and no fall termination.

Behavior cloning that 12-channel reference produced a lower-drift learned baseline:

| Metric | Result |
| --- | ---: |
| Environments | `12` train / `4` playback |
| Actor input dimension | `73` |
| Actor output dimension | `12` |
| BC steps | `500` |
| Action scale | `45 deg` |
| Reference timing | `1.5 Hz` |
| Final raw action MSE | `0.000793` |
| Final clipped action MSE | `0.000793` |
| Final raw max action | `0.810423` |
| Playback max action | `0.957950` |
| Mean reference-action tracking | about `0.9580` |
| Mean reference-action MSE | about `0.00538` |
| Mean foot contacts per env | `3.533333` |
| Mean swing clearance | `0.001902 m` |
| Mean displacement-derived x velocity | about `0.01187 m/s` |
| Final x displacement | about `0.0570 m` |
| Final y displacement | about `-0.0046 m` |
| Fall terminations | `0` |

At the time of this direct-closure experiment, this checkpoint was the best learned option when lateral drift mattered. It did not satisfy the `0.08 m/s` command, but it was a stronger continuation point than the earlier higher-drift signed-axis PPO checkpoint.

An 80-iteration PPO refinement from this BC checkpoint did not improve the result:

| Metric | Result |
| --- | ---: |
| Environments | `16` train / `4` playback |
| PPO timesteps | `20480` |
| Actor input dimension | `73` |
| Actor output dimension | `12` |
| Mean reference-action tracking | about `0.7820` |
| Mean reference-action MSE | about `0.03133` |
| Mean foot contacts per env | `3.4875` |
| Mean swing clearance | `0.003238 m` |
| Mean displacement-derived x velocity | about `0.00992 m/s` |
| Final x displacement | about `0.0476 m` |
| Final y displacement | about `0.0049 m` |
| Playback max action | `1.286892` |
| Fall terminations | `0` |

The PPO checkpoint stayed upright, but it moved less far forward and exceeded the normalized action range before clipping. The retained baseline is therefore the 45-degree, 1.5 Hz near-straight BC checkpoint.

The same near-straight teacher was then replayed at 60-degree action scale and 2.0 Hz. The open-loop reference stayed stable and reached about `60.7 mm` x displacement with about `-1.5 mm` y drift. The first unweighted behavior-cloned checkpoint moved forward reliably but drifted sideways:

| Metric | Result |
| --- | ---: |
| Environments | `12` train / `4` playback |
| Actor input dimension | `73` |
| Actor output dimension | `12` |
| BC steps | `600` |
| Action scale | `60 deg` |
| Reference timing | `2.0 Hz` |
| Final raw action MSE | `0.001529` |
| Final clipped action MSE | `0.001529` |
| Final raw max action | `0.743857` |
| Playback max action | `0.927953` |
| Mean reference-action tracking | about `0.9697` |
| Mean reference-action MSE | about `0.00380` |
| Mean foot contacts per env | `3.483333` |
| Mean swing clearance | `0.001709 m` |
| Mean displacement-derived x velocity | about `0.01388 m/s` |
| Final x displacement | about `0.0666 m` |
| Final y displacement | about `-0.0183 m` |
| Fall terminations | `0` |

That first 60-degree BC checkpoint moved faster than the 45-degree baseline, but drifted sideways more. A conservative 60-iteration PPO refinement with stronger lateral penalty reduced the final y displacement to about `0.0081 m`, but final x displacement fell to about `0.0493 m`; that refinement is not better than the BC checkpoint.

The weighted 60-degree behavior-cloned checkpoint is the current lower-drift learned baseline: about `63.4 mm` average forward displacement, about `-1.9 mm` average lateral displacement, no fall terminations, and mean reference-action MSE about `0.0017` over the same 240-step CUDA playback gate. The newer weighted 70-degree / 2.25 Hz behavior-cloned checkpoint is the fastest retained BC baseline: about `67.1 mm` average forward displacement, about `3.4 mm` average lateral displacement, no fall terminations, and mean reference-action MSE about `0.0017`.

## Latest 70-Degree PPO Continuation

A new heading-refinement PPO run continued from the weighted 70-degree / 2.25 Hz BC checkpoint. It kept the actual Domino CAD visual overlay enabled, kept visible proxy cubes/spheres at zero, and marked the proxy collision/debug shapes as guide-purpose transparent geometry. Playback of sampled checkpoints shows a clear tradeoff:

| Checkpoint | Forward displacement | Lateral displacement | Final heading drift | Mean reference-action MSE | Fall terminations | Interpretation |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Retained 70-degree BC baseline | `67.1 mm` | `3.4 mm` | `0.292 rad` | `0.0017` | `0` | Current retained fast baseline. |
| Heading-refine `model_20.pt` | `66.4 mm` | `-0.5 mm` | `0.283 rad` | `0.0031` | `0` | Near-fastest balanced candidate; slightly cleaner lateral and yaw than the BC baseline. |
| Heading-refine `model_30.pt` | `76.3 mm` | `-3.1 mm` | `0.356 rad` | `0.0051` | `0` | Faster forward, but heading drift worsens. |
| Heading-refine `model_40.pt` | `80.7 mm` | `-1.0 mm` | `0.397 rad` | `0.0063` | `0` | Faster again, with poor heading drift. |
| Heading-refine `model_79.pt` | `82.5 mm` | `-6.1 mm` | `0.384 rad` | `0.0111` | `0` | New best forward candidate, but not straight enough to call a clean baseline. |

The useful takeaway is not that the final checkpoint walks well. It does not. The useful result is that the reward path can push the actual-CAD linkage scene farther forward without physics collapse, and that `model_20.pt` is a credible next continuation point if the next run should preserve straightness. `model_79.pt` is evidence for forward-progress authority, not a polished locomotion policy.

## Model-20 Straightness Continuation

A stricter yaw/lateral continuation was started from the balanced `model_20.pt` checkpoint. The run was interrupted before the training script wrote its final JSON report, but it did write complete checkpoints through `model_48.pt`. Three sampled checkpoint playbacks were then run with actual Domino CAD visuals enabled and the proxy render guard intact.

| Checkpoint | Forward displacement | Lateral displacement | Final heading drift | Mean reference-action MSE | Fall terminations | Interpretation |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Starting `model_20.pt` | `66.4 mm` | `-0.5 mm` | `0.283 rad` | `0.0031` | `0` | Balanced continuation point from the previous run. |
| Straightness refine `model_30.pt` | `60.3 mm` | `-0.4 mm` | `0.232 rad` | `0.0042` | `0` | Best heading improvement, but too much forward loss. |
| Straightness refine `model_40.pt` | `62.1 mm` | `-1.0 mm` | `0.244 rad` | `0.0055` | `0` | Stable midpoint; still slower than the start. |
| Straightness refine `model_48.pt` | `62.5 mm` | `-1.9 mm` | `0.248 rad` | `0.0062` | `0` | Latest sampled checkpoint; not a new retained baseline. |

This run confirms that stronger yaw/lateral penalties can reduce heading drift without breaking the closed-linkage physics or CAD visual overlay, but the current reward balance trades away too much forward displacement. The next run should not simply increase yaw/lateral weights further. It should either start again from `model_20.pt` with a smaller straightness penalty and stronger progress preservation, or tune the scripted/reference gait itself before another PPO continuation.

## Model-20 Forward-Preserving Continuation

A softer continuation then started again from `model_20.pt` with stronger forward-progress preservation and weaker yaw/lateral penalties than the straightness diagnostic. It completed the full 40-iteration run, wrote `model_59.pt`, and replayed with actual Domino CAD visuals enabled and the proxy render guard intact.

| Checkpoint | Forward displacement | Lateral displacement | Final heading drift | Mean reference-action MSE | Fall terminations | Interpretation |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Starting `model_20.pt` | `66.4 mm` | `-0.5 mm` | `0.283 rad` | `0.0031` | `0` | Cleanest balanced continuation point. |
| Forward-preserving `model_40.pt` | `63.9 mm` | `1.1 mm` | `0.264 rad` | `0.0037` | `0` | Cleaner yaw/lateral than retained BC, but too slow. |
| Forward-preserving `model_50.pt` | `65.7 mm` | `2.0 mm` | `0.279 rad` | `0.0038` | `0` | Near-fastest balanced candidate against retained BC, but not cleaner than `model_20.pt` overall. |
| Forward-preserving `model_59.pt` | `68.9 mm` | `2.5 mm` | `0.311 rad` | `0.0041` | `0` | New forward candidate versus retained BC; yaw is slightly worse. |

The useful result is that forward distance can be recovered without returning to the large lateral drift seen in earlier forward-only PPO probes. The remaining issue is heading: `model_59.pt` improves forward and lateral displacement against the retained 70-degree BC baseline, but final yaw drift is worse by about `0.019 rad`. Use `model_59.pt` only if chasing forward distance; otherwise `model_20.pt` remains the cleaner balanced continuation point.

## What This Proves

- The CAD-derived floating linkage can now be created from an import-safe builder, not only from the standalone smoke script.
- Isaac Lab accepts the CAD-derived linkage proxy wrapper as a vectorized `DirectRLEnv`.
- RSL-RL can collect four-env rollouts, run a command-aware PPO update, write checkpoints, reload the newest checkpoint, and play it back in the proxy linkage env.
- The 12-action contract is enforced in code and reported at runtime: four shoulder actuators plus eight linkage-drive actuators.
- The scripted gait search uses the same 12-action order and records the exact action names, so shoulder drives are not being skipped.
- Reference-guided PPO and BC warm-starts can drive every shoulder, lower-linkage, and upper-linkage/pitch channel through the real 12-action Domino contract.
- The retained near-straight 12-action BC checkpoints play back without fall terminations. The current retained fast BC checkpoint produces about `67.1 mm` forward displacement with about `3.4 mm` lateral drift and about `0.292 rad` final heading drift. The lower-drift 60-degree BC checkpoint produces about `63.4 mm` forward displacement with about `1.9 mm` lateral drift. The `-6 deg` and `-15 deg` symmetry-trim BC checkpoints remain useful yaw comparisons at about `59.3 mm` / `0.219 rad` and `50.1 mm` / `0.142 rad`.
- PPO refinements now stay physically stable in the actual-CAD visual scene, but they are still candidates rather than solved walking policies. The `-6 deg` yaw/lateral refinement reached about `50.6 mm` forward displacement, about `8.6 mm` lateral drift, and about `0.162 rad` final heading drift; that is better yaw than the `-6 deg` BC checkpoint but worse forward/lateral behavior.
- The latest 70-degree PPO continuation produced a useful tradeoff curve. `model_20.pt` is the balanced candidate at about `66.4 mm` forward displacement, `-0.5 mm` lateral displacement, and `0.283 rad` final heading drift. `model_79.pt` is the best forward-distance diagnostic at about `82.5 mm`, but it carries `-6.1 mm` lateral displacement and `0.384 rad` final heading drift. A stricter continuation from `model_20.pt` reduced yaw to about `0.232-0.248 rad` across sampled checkpoints, but forward displacement fell to about `60.3-62.5 mm`. A softer forward-preserving continuation reached `68.9 mm` forward and `2.5 mm` lateral, beating the retained BC baseline on those two metrics while worsening final yaw to `0.311 rad`. None of these checkpoints track the `0.08 m/s` velocity command yet.
- Displacement-derived velocity tracking is now available in reward and playback, which is the correct metric for judging actual body motion.
- The CAD copies are authored under separate Isaac Lab environment roots, and policy observations use positions relative to each environment origin.
- The trained checkpoint plays back without fall terminations over the tested 240-step horizon; the only done events were normal episode timeouts.
- The CAD-derived foot proxies can lift under scripted 12-channel actions when the clamped action scale is large enough.
- These historical PPO refinements did not produce a finished locomotion policy. The final pass proved that heading-drift reward plumbing worked and recorded heading drift directly, but it still needed stronger straight-line command tracking.

## Historical Remaining Work

This is not a trained locomotion policy. The next work is:

- The historical continuation plan kept the weighted 70-degree / 2.25 Hz BC checkpoint as its fast baseline and `model_20.pt` as its balanced PPO candidate. `model_79.pt`, `model20_straight_refine`, and forward-preserving `model_59.pt` remain archived diagnostics, not current walking policies.
- The default next-run planner has now produced and tested `model20_forward_preserving_straight_refine`. The next planner pass should either continue from `model_20.pt` for balance/heading work or from `model_59.pt` only if the immediate goal is forward-distance recovery.
- Use the new reward component diagnostics to tune the next curriculum instead of changing scales blindly.
- Train for substantially longer with more parallel environments once the reference-action curriculum is producing repeatable swing-foot unloading.
- Balance forward-progress shaping against lateral/yaw penalties. The latest 70-degree continuation proves forward motion can be increased, and the `model20_straight_refine` run proves heading drift can be reduced, but the combined teacher/policy objective still needs tuning before PPO replaces the retained BC baseline as a clean portfolio result.
- Improve contact sensing beyond height-based foot flags if practical for the proxy linkage bodies.
- Add hard-stop and safety checks around drive targets and reset poses before longer training.
- Decide whether the final policy should use this detailed closed-linkage model directly or a cleaner 12-DoF training articulation validated against the linkage model.
