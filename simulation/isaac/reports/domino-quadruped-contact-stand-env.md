# Domino Floating Contact, Stand Env, PPO, And Playback Smoke

This report records the first floating-base gravity/contact smoke test, the first Isaac Lab `DirectRLEnv` stand-task smoke test, the first tiny RSL-RL PPO checkpoint smoke, and the first checkpoint playback smoke for the clean Domino quadruped prototype.

## Floating Contact Smoke

Status: **passed for floating-base gravity/contact smoke**.

The clean quadruped URDF was imported to USD without fixed-base mode, spawned with a simple static ground-box collider, and stepped under gravity while holding conservative twelve-joint targets.

| Metric | Result |
| --- | ---: |
| Physics steps | `1000` |
| Physics dt | `0.005 s` |
| Action count | `12` |
| Shoulder actions | `4` |
| Lower-linkage drive actions | `4` |
| Upper-pitch drive actions | `4` |
| Min root height | `0.310606 m` |
| Max root height | `0.379756 m` |
| Max root speed | `1.128161 m/s` |
| Max root tilt | `1.466604 deg` |
| Max tracking error | `0.080197 rad` |
| Max joint speed | `4.650291 rad/s` |
| Max joint-limit violation | `0.0 rad` |
| Status | `passed` |

This proves the clean all-leg articulation can run as a floating robot under gravity and make contact with a simple ground collider without falling through the world, exceeding joint limits, or producing non-finite state.

## DirectRLEnv Stand Smoke

Status: **passed for first stand-task environment smoke**.

The `DominoStandEnv` task wraps the same clean floating quadruped with:

- Twelve action dimensions: four shoulder ab/ad drives, four lower-linkage drives, and four upper-pitch drives.
- Forty-five policy observation dimensions.
- Reset logic for root and joint state.
- Height, orientation, joint-velocity, and action-rate reward terms.
- Termination checks for low root height and excessive tilt.

| Metric | Result |
| --- | ---: |
| Environment steps | `300` |
| Number of environments | `1` |
| Action dimension | `12` |
| Observation dimension | `45` |
| Mean reward | `0.997727` |
| Terminated count | `0` |
| Truncated count | `1` |
| Min root height | `0.311065 m` |
| Max root tilt | `0.716005 deg` |
| Status | `passed` |

This is not trained locomotion yet. It is the first verified policy-task scaffold: reset, step, observe, reward, and done all run against the floating Domino articulation.

## RSL-RL PPO Smoke

Status: **passed for first checkpoint-producing PPO smoke**.

This run used `DominoStandEnv` directly through Isaac Lab's RSL-RL wrapper. It proves the 12-action environment can collect a rollout, update an actor-critic policy, and write a checkpoint. It is deliberately too short to produce useful motion.

| Metric | Result |
| --- | ---: |
| PPO iterations | `1` |
| Rollout steps per env | `8` |
| Number of environments | `1` |
| Actor output dimension | `12` |
| Actor/critic input dimension | `45` |
| Saved checkpoints | `model_0.pt` |
| Status | `passed` |

The actor output dimension is the full Domino actuator contract: four shoulder ab/ad actuators, four lower-linkage drives, and four upper-pitch drives.

## RSL-RL Checkpoint Playback Smoke

Status: **passed for first PPO checkpoint playback smoke**.

The saved `model_0.pt` checkpoint was loaded into a fresh `DominoStandEnv` instance and stepped headlessly for one episode horizon.

| Metric | Result |
| --- | ---: |
| Evaluation steps | `250` |
| Number of environments | `1` |
| Action dimension | `12` |
| Observation dimension | `45` |
| Mean reward | `0.998135` |
| Done count | `1` |
| Min root height | `0.310862 m` |
| Max root tilt | `0.567862 deg` |
| Max absolute policy action | `0.308126` |
| Status | `passed` |

The single done event is the expected stand-task episode timeout, not a fall termination. This still does not prove useful locomotion; it proves the current checkpoint can be loaded, produces finite 12-channel actions, and steps the Domino stand environment without destabilizing the base.

## Parallel Env And PPO Smoke

Status: **passed for parallel stand-env and PPO smoke**.

The stand task now sizes its static ground box from the requested cloned-env grid. This avoids the one-env 4 m ground plate becoming too small when policy training uses multiple environments.

| Gate | Result |
| --- | ---: |
| 4-env env smoke | `passed` |
| 4-env ground size | `6.0 m` |
| 4-env env smoke steps | `100` |
| 4-env PPO smoke | `2 iterations x 16 steps/env` |
| 4-env PPO checkpoints | `model_0.pt`, `model_1.pt` |
| 4-env playback | `250 steps`, `model_1.pt` |
| 4-env playback done count | `4 expected timeouts` |
| 4-env min root height | `0.310751 m` |
| 4-env max root tilt | `0.627761 deg` |
| 16-env env smoke | `passed` |
| 16-env ground size | `10.0 m` |
| 16-env env smoke steps | `50` |
| 16-env PPO smoke | `1 iteration x 8 steps/env` |
| 16-env PPO checkpoint | `model_0.pt` |
| 16-env playback | `250 steps`, `model_0.pt` |
| 16-env playback done count | `16 expected timeouts` |
| 16-env min root height | `0.310835 m` |
| 16-env max root tilt | `0.597089 deg` |
| 16-env max absolute policy action | `0.311672` |

This proves the current stand task can run and train through RSL-RL with cloned environments, not only as a single-robot script.

## Local Isaac Notes

The local Isaac installation still emits warnings about Kit config writes and extension startup. The Domino runners now set `WARP_CACHE_PATH` to the ignored `simulation/isaac/out/warp_cache` folder by default, which avoids the global Warp cache collision that previously blocked `DirectRLEnv` import.

## Next Gate

The next useful milestone is a longer stand-stability training pass. After that, add contact sensors and richer foot/body observations before attempting velocity-command locomotion.

The major fidelity gap is still the same: this policy task uses the clean tree articulation for training. The CAD-derived passive linkage loops are validated separately in the pin-linkage prototypes, but they have not yet been merged into the floating policy-training articulation.
