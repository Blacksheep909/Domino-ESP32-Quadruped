"""Run a tiny RSL-RL PPO smoke test for the Domino CAD-linkage DirectRLEnv."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

from isaaclab.app import AppLauncher

CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))


parser = argparse.ArgumentParser(description="Train the Domino CAD-linkage env briefly with RSL-RL PPO.")
parser.add_argument("--num-envs", type=int, default=1, help="Number of manually authored CAD-linkage environments.")
parser.add_argument("--iterations", type=int, default=1, help="PPO learning iterations to run.")
parser.add_argument("--num-steps-per-env", type=int, default=8, help="Rollout steps per env per PPO iteration.")
parser.add_argument(
    "--physics-device",
    choices=["same", "cpu", "cuda:0"],
    default="same",
    help="PhysX device. 'same' uses the AppLauncher --device value while allowing CPU PhysX with a CUDA policy.",
)
parser.add_argument("--save-interval", type=int, default=None, help="PPO checkpoint interval in learning iterations.")
parser.add_argument(
    "--visible-step-delay-s",
    type=float,
    default=0.0,
    help="Optional wall-clock delay after each control step so visible training motion can be inspected.",
)
parser.add_argument("--seed", type=int, default=42, help="Training seed.")
parser.add_argument("--init-noise-std", type=float, default=None, help="Initial policy action noise standard deviation.")
parser.add_argument("--ppo-learning-rate", type=float, default=None, help="Optional PPO optimizer learning-rate override.")
parser.add_argument("--ppo-entropy-coef", type=float, default=None, help="Optional PPO entropy coefficient override.")
parser.add_argument("--action-scale-deg", type=float, default=None, help="Maximum drive target offset, in degrees, for action=1.")
parser.add_argument(
    "--servo-target-rate-limit-deg-s",
    type=float,
    default=None,
    help="Maximum servo target slew rate in degrees/second. Use 0 to disable the slew limiter.",
)
parser.add_argument(
    "--reset-settle-steps",
    type=int,
    default=None,
    help="Neutral control steps held after a reset before policy actions are applied.",
)
parser.add_argument("--episode-length-s", type=float, default=None, help="Episode length override.")
parser.add_argument("--min-height-m", type=float, default=None, help="Reset when body height falls below this value.")
parser.add_argument("--max-tilt-deg", type=float, default=None, help="Reset when body tilt exceeds this angle.")
parser.add_argument(
    "--actual-cad-ground-clearance-m",
    type=float,
    default=None,
    help="Visual CAD foot-bottom clearance used when aligning the neutral pose to the floor.",
)
parser.add_argument(
    "--ground-size-m",
    type=float,
    default=None,
    help="Minimum width and depth of the shared square ground collider.",
)
parser.add_argument("--command-x-m-s", type=float, default=0.0, help="Forward velocity command.")
parser.add_argument("--command-y-m-s", type=float, default=0.0, help="Lateral velocity command.")
parser.add_argument("--command-yaw-rad-s", type=float, default=0.0, help="Yaw-rate command.")
parser.add_argument("--gait-frequency-hz", type=float, default=1.0, help="Gait phase frequency in observations.")
parser.add_argument("--command-progress-reward-scale", type=float, default=None, help="Forward/lateral progress reward scale.")
parser.add_argument("--command-velocity-reward-scale", type=float, default=None, help="Penalty scale for squared command x/y velocity error.")
parser.add_argument("--command-velocity-tracking-reward-scale", type=float, default=None, help="Positive command velocity tracking reward scale.")
parser.add_argument("--command-velocity-tracking-sigma", type=float, default=None, help="Velocity error sigma for the positive command-tracking reward.")
parser.add_argument(
    "--command-stagnation-penalty-scale",
    type=float,
    default=None,
    help="Penalty scale applied when commanded forward speed stays below the configured stagnation threshold.",
)
parser.add_argument(
    "--command-stagnation-speed-m-s",
    type=float,
    default=None,
    help="Directional speed below which a nonzero locomotion command is considered stagnant.",
)
parser.add_argument("--alive-reward-scale", type=float, default=None, help="Per-step survival reward scale.")
parser.add_argument("--vertical-velocity-reward-scale", type=float, default=None)
parser.add_argument("--angular-velocity-reward-scale", type=float, default=None)
parser.add_argument("--flat-orientation-reward-scale", type=float, default=None)
parser.add_argument("--pitch-orientation-reward-scale", type=float, default=None)
parser.add_argument("--action-reward-scale", type=float, default=None, help="Reward scale for squared action magnitude.")
parser.add_argument("--action-rate-reward-scale", type=float, default=None, help="Reward scale for squared action changes.")
parser.add_argument(
    "--disable-displacement-velocity-rewards",
    action="store_true",
    help="Use PhysX reported body linear velocity instead of body-position delta for command velocity/progress rewards.",
)
parser.add_argument("--lateral-drift-reward-scale", type=float, default=None, help="Reward scale for squared lateral body drift from the env origin.")
parser.add_argument("--yaw-drift-reward-scale", type=float, default=None, help="Reward scale for squared body heading drift from the commanded heading.")
parser.add_argument("--command-yaw-reward-scale", type=float, default=None, help="Penalty scale for squared command yaw-rate error.")
parser.add_argument("--gait-contact-reward-scale", type=float, default=None, help="Alternating stance/swing contact reward scale.")
parser.add_argument("--stance-contact-reward-scale", type=float, default=None, help="Reward scale for commanded stance feet staying in contact.")
parser.add_argument("--swing-contact-penalty-scale", type=float, default=None, help="Penalty scale for commanded swing feet staying in contact.")
parser.add_argument("--foot-clearance-reward-scale", type=float, default=None, help="Swing foot clearance reward scale.")
parser.add_argument("--foot-contact-reward-scale", type=float, default=None, help="All-foot contact reward scale.")
parser.add_argument("--foot-slip-reward-scale", type=float, default=None, help="Contact-only planar foot-slip penalty scale.")
parser.add_argument(
    "--air-time-variance-reward-scale",
    type=float,
    default=None,
    help="Penalty scale for unequal completed foot air/contact durations.",
)
parser.add_argument(
    "--valid-foot-cycle-reward-scale",
    type=float,
    default=None,
    help="Reward scale for qualified liftoff-to-touchdown foot cycles.",
)
parser.add_argument("--front-rear-support-reward-scale", type=float, default=None)
parser.add_argument("--axle-support-imbalance-penalty-scale", type=float, default=None)
parser.add_argument("--same-axle-airborne-penalty-scale", type=float, default=None)
parser.add_argument("--excess-airborne-penalty-scale", type=float, default=None)
parser.add_argument("--front-foot-backward-reach-penalty-scale", type=float, default=None)
parser.add_argument("--front-pair-backward-reach-penalty-scale", type=float, default=None)
parser.add_argument("--front-foot-min-body-x-m", type=float, default=None)
parser.add_argument("--front-foot-reach-normalization-m", type=float, default=None)
parser.add_argument("--front-foot-backward-termination-body-x-m", type=float, default=None)
parser.add_argument("--foot-cycle-min-air-time-s", type=float, default=None)
parser.add_argument("--foot-cycle-target-air-time-s", type=float, default=None)
parser.add_argument("--foot-cycle-min-clearance-m", type=float, default=None)
parser.add_argument(
    "--foot-cycle-min-body-relative-travel-m",
    type=float,
    default=None,
)
parser.add_argument("--reference-gait-candidate", default="", help="Optional scripted gait JSON used as an action-prior reward.")
parser.add_argument(
    "--include-reference-actions-in-observation",
    action="store_true",
    help="Append the current 12-channel scripted reference action target to the policy observation.",
)
parser.add_argument(
    "--reference-action-tracking-reward-scale",
    type=float,
    default=None,
    help="Reward scale for matching the optional scripted 12-actuator reference gait.",
)
parser.add_argument(
    "--reference-action-tracking-sigma",
    type=float,
    default=None,
    help="Sigma for the optional reference-action tracking reward.",
)
parser.add_argument(
    "--reference-action-mse-reward-scale",
    type=float,
    default=None,
    help="Reward scale applied to mean squared error against the optional scripted 12-actuator reference action.",
)
parser.add_argument("--log-root", default="simulation/isaac/out/domino_rsl_rl", help="Training log root for RSL-RL checkpoints and summaries.")
parser.add_argument("--resume-checkpoint", default="", help="Optional RSL-RL checkpoint to load before continuing training.")
parser.add_argument(
    "--resume-load-optimizer",
    action="store_true",
    help="Also restore optimizer state when loading --resume-checkpoint. Leave disabled for BC warm-start checkpoints.",
)
parser.add_argument(
    "--reference-action-bc-steps",
    type=int,
    default=0,
    help="Optional supervised optimizer steps that train the actor to copy appended 12-channel reference actions.",
)
parser.add_argument(
    "--reference-action-identity-init",
    action="store_true",
    help="Initialize the ELU actor as an exact pass-through for the appended 12-channel reference target.",
)
parser.add_argument(
    "--reference-action-bc-settle-steps",
    type=int,
    default=120,
    help="Neutral physics steps completed before reference-action behavior cloning begins.",
)
parser.add_argument("--reference-action-bc-lr", type=float, default=3.0e-4, help="Learning rate for the optional reference-action supervised warm start.")
parser.add_argument(
    "--reference-action-bc-replay-steps",
    type=int,
    default=0,
    help="Teacher-controlled environment steps collected before randomized replay training; zero uses online BC.",
)
parser.add_argument(
    "--reference-action-bc-batch-size",
    type=int,
    default=256,
    help="Random replay batch size used when --reference-action-bc-replay-steps is positive.",
)
parser.add_argument(
    "--reference-action-bc-output-penalty",
    type=float,
    default=1.0e-3,
    help="Small L2 penalty on raw actor outputs during optional reference-action supervised warm start.",
)
parser.add_argument(
    "--reference-action-bc-shoulder-weight",
    type=float,
    default=1.0,
    help="BC loss weight for shoulder hip ab/ad action channels.",
)
parser.add_argument(
    "--reference-action-bc-lower-linkage-weight",
    type=float,
    default=1.0,
    help="BC loss weight for lower-linkage drive action channels.",
)
parser.add_argument(
    "--reference-action-bc-upper-pitch-weight",
    type=float,
    default=1.0,
    help="BC loss weight for upper pitch/linkage action channels.",
)
parser.add_argument(
    "--skip-ppo-after-bc",
    action="store_true",
    help="Stop after the optional reference-action supervised warm start and write a report/checkpoint.",
)
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
parser.add_argument(
    "--graceful-close",
    action="store_true",
    help="Call SimulationApp.close() before exit. Disabled by default because it can hang on some Windows setups.",
)
parser.add_argument("--hold-open", action="store_true", help="Keep the visible Isaac Sim window open after training.")
parser.add_argument(
    "--hold-open-mode",
    choices=["reset", "final", "policy"],
    default="reset",
    help="Visible hold-open behavior: reset to neutral, leave final state, or keep rolling out the trained policy.",
)
parser.add_argument(
    "--hold-open-render-frames",
    type=int,
    default=4,
    help="Viewport refresh frames after preparing the visible hold-open pose.",
)
parser.add_argument(
    "--hold-open-exit-after-frames",
    type=int,
    default=0,
    help="Testing hook: exit hold-open after this many app update frames. Zero keeps the window open until closed.",
)
parser.add_argument(
    "--policy-validation-steps",
    type=int,
    default=160,
    help="Measured rollout steps used to validate the trained policy before live hold-open display.",
)
parser.add_argument(
    "--policy-validation-settle-steps",
    type=int,
    default=120,
    help="Zero-action steps used to settle the authored bent neutral pose before policy validation metrics begin.",
)
parser.add_argument(
    "--policy-validation-ramp-steps",
    type=int,
    default=60,
    help="Validation steps used to blend from neutral actions to the learned policy output.",
)
parser.add_argument(
    "--policy-reference-action-snap-tolerance",
    type=float,
    default=0.0,
    help="During validation, execute the exact appended reference when all policy channels are within this tolerance.",
)
parser.add_argument(
    "--disable-policy-display-gate",
    action="store_true",
    help="Keep displaying the trained policy even if the post-training rollout-quality gate fails.",
)
parser.add_argument(
    "--allow-indefinite-policy-hold-open",
    action="store_true",
    help="Allow --hold-open-mode policy to keep rolling forever after validation. Disabled by default for visual safety.",
)
parser.add_argument(
    "--policy-gate-max-joint-separation-m",
    type=float,
    default=0.003,
    help="Maximum allowed pin/joint separation before a trained policy is considered unsafe to display live.",
)
parser.add_argument(
    "--policy-gate-min-forward-m",
    type=float,
    default=0.001,
    help="Minimum forward body displacement required before live trained-policy display.",
)
parser.add_argument(
    "--policy-gate-max-lateral-m",
    type=float,
    default=0.06,
    help="Maximum absolute lateral body displacement allowed before live trained-policy display.",
)
parser.add_argument(
    "--policy-gate-max-yaw-rad",
    type=float,
    default=0.70,
    help="Maximum absolute heading drift allowed before live trained-policy display.",
)
parser.add_argument(
    "--policy-gate-max-tilt-deg",
    type=float,
    default=30.0,
    help="Maximum body tilt allowed during the validation rollout before live trained-policy display.",
)
parser.add_argument(
    "--policy-gate-max-swing-contact",
    type=float,
    default=0.95,
    help="Maximum mean commanded-swing foot contact fraction before live trained-policy display.",
)
parser.add_argument(
    "--policy-gate-min-gait-contact-match",
    type=float,
    default=0.0,
    help="Minimum mean agreement between measured and commanded diagonal gait contacts.",
)
parser.add_argument(
    "--policy-gate-min-swing-clearance-m",
    type=float,
    default=0.0002,
    help="Minimum mean commanded-swing foot clearance before live trained-policy display.",
)
parser.add_argument(
    "--policy-gate-min-each-cad-foot-clearance-m",
    type=float,
    default=0.010,
    help="Minimum peak ground clearance that every rendered actual-CAD foot must reach during validation.",
)
parser.add_argument(
    "--policy-gate-min-foot-motion-m",
    type=float,
    default=0.035,
    help="Minimum body-relative actual-CAD foot-bottom motion required from every leg before live display.",
)
parser.add_argument(
    "--policy-gate-min-each-linkage-drive-motion-deg",
    type=float,
    default=2.0,
    help="Minimum actual rotation required from every lower/upper drive relative to its hip carriage.",
)
parser.add_argument(
    "--policy-gate-max-visual-foot-motion-m",
    type=float,
    default=0.25,
    help="Maximum rendered actual-CAD foot-bottom displacement allowed before live trained-policy display.",
)
parser.add_argument(
    "--policy-gate-min-valid-cycles-per-foot",
    type=int,
    default=1,
    help="Minimum valid contact-liftoff-swing-touchdown cycles required from every foot in every validation env.",
)
parser.add_argument(
    "--policy-gate-min-valid-cycle-ratio",
    type=float,
    default=0.50,
    help="Minimum fraction of completed foot cycles that pass clearance, travel, tilt, and touchdown-support checks.",
)
parser.add_argument(
    "--policy-gate-max-foot-cycle-domination-ratio",
    type=float,
    default=0.50,
    help="Maximum share of all valid cycles contributed by one foot.",
)
parser.add_argument("--gait-cycle-min-air-steps", type=int, default=3)
parser.add_argument("--gait-cycle-touchdown-confirm-steps", type=int, default=2)
parser.add_argument("--gait-cycle-min-clearance-m", type=float, default=0.004)
parser.add_argument("--gait-cycle-min-body-relative-travel-m", type=float, default=0.020)
parser.add_argument("--gait-cycle-max-tilt-deg", type=float, default=25.0)
parser.add_argument("--gait-cycle-min-touchdown-support-feet", type=int, default=2)
parser.add_argument(
    "--allow-proxy-visuals",
    action="store_true",
    help="Allow visible rendering of the CAD-derived cube/sphere proxy for physics debugging.",
)
parser.add_argument(
    "--allow-multi-env-viewport",
    action="store_true",
    help="Allow a visible viewport with more than one cloned training environment.",
)
parser.add_argument(
    "--disable-actual-cad-visuals",
    action="store_true",
    help="Render only the CAD-derived cube/sphere proxy instead of the exported Domino STL link meshes.",
)
parser.add_argument(
    "--foot-collision-mode",
    choices=["linkage-lower-closure", "actual-cad-visual-bottom", "actual-cad-grounded-support"],
    default=None,
    help="Foot contact proxy source. Grounded-support keeps the low chassis height with common-plane hidden support spheres.",
)
parser.add_argument(
    "--closure-model",
    choices=["direct", "passive"],
    default="passive",
    help="Loop-closure topology. Passive restores the CAD closure bodies; direct is retained only for comparison.",
)
parser.add_argument("--actual-cad-mesh-dir", default="", help="Optional override for the Domino STL mesh folder.")
parser.add_argument("--terrain-type", choices=["flat", "stairs"], default="flat", help="Static terrain scene used for training.")
parser.add_argument("--stairs-step-count", type=int, default=None, help="Number of stair treads when --terrain-type=stairs.")
parser.add_argument("--stairs-step-depth-m", type=float, default=None, help="Step tread depth for stair training.")
parser.add_argument("--stairs-step-height-m", type=float, default=None, help="Step rise height for stair training.")
parser.add_argument("--stairs-width-m", type=float, default=None, help="Width of the stair obstacle.")
parser.add_argument("--stairs-start-x-m", type=float, default=None, help="Local x position where the first stair starts.")
parser.add_argument("--stairs-top-platform-length-m", type=float, default=None, help="Flat platform length after the last stair.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


def is_human_viewable_run(args: argparse.Namespace) -> bool:
    livestream_arg = int(getattr(args, "livestream", -1))
    livestream_env = int(os.environ.get("LIVESTREAM", "0") or 0)
    livestream = livestream_arg if livestream_arg >= 0 else livestream_env
    enable_cameras = bool(getattr(args, "enable_cameras", False) or int(os.environ.get("ENABLE_CAMERAS", "0") or 0))
    return bool((not args.headless) or livestream in {1, 2} or enable_cameras)


VIEWABLE_RUN = is_human_viewable_run(args_cli)

if VIEWABLE_RUN and args_cli.disable_actual_cad_visuals and not args_cli.allow_proxy_visuals:
    raise SystemExit(
        "Visible Domino DirectRLEnv training should use the actual exported CAD STL visuals. "
        "Remove --disable-actual-cad-visuals, run headless, or pass --allow-proxy-visuals when deliberately "
        "debugging the simplified cube/sphere proxy."
    )

if VIEWABLE_RUN and int(args_cli.num_envs) != 1 and not args_cli.allow_multi_env_viewport:
    raise SystemExit(
        "Visible Domino CAD inspection runs should show one robot. "
        "Use --num-envs 1 for visual checks, or pass --allow-multi-env-viewport when deliberately viewing cloned "
        "training environments."
    )

os.environ.setdefault("WARP_CACHE_PATH", str((Path.cwd() / "simulation" / "isaac" / "out" / "warp_cache").resolve()))

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

from domino_action_contract import (  # noqa: E402
    ACTION_JOINT_NAMES,
    CAD_ACTION_ROLES,
    EXPECTED_ACTION_COUNT,
    FOOT_BODY_NAMES,
    action_group_counts,
    per_leg_action_layout,
)
from domino_cad_linkage_env import (  # noqa: E402
    CAD_LINKAGE_REFERENCE_OBSERVATION_DIM,
    DominoCadLinkageEnv,
    DominoCadLinkageEnvCfg,
    projected_gravity_from_quat,
    quat_wxyz_to_rotation_matrix,
    wrap_angle_rad,
    yaw_from_quat_wxyz,
)
from domino_cad_linkage_rsl_rl_cfg import DominoCadLinkagePPORunnerCfg  # noqa: E402
from domino_gait_cycle_metrics import GaitCycleTracker  # noqa: E402
from domino_linkage_motion import (  # noqa: E402
    create_foot_endpoint_motion_tracker,
    create_linkage_motion_tracker,
    foot_endpoint_motion_report,
    linkage_motion_report,
    update_foot_endpoint_motion_tracker,
    update_linkage_motion_tracker,
)
from domino_reference_gait import REFERENCE_GAIT_PARAMETER_NAMES, is_keyframe_sequence, load_reference_candidate  # noqa: E402


class DominoVisibleRslRlVecEnvWrapper(RslRlVecEnvWrapper):
    """Refresh the Kit viewport after each RSL-RL control step."""

    def __init__(self, env: DominoCadLinkageEnv, *args, **kwargs):
        self._domino_env = env
        super().__init__(env, *args, **kwargs)

    def step(self, actions):
        result = super().step(actions)
        try:
            self._domino_env.render(recompute=False)
        except Exception:
            self._domino_env.sim.render()
        simulation_app.update()
        return result


def checkpoint_index(path: Path) -> int:
    try:
        return int(path.stem.split("_")[-1])
    except ValueError:
        return -1


def apply_reference_gait_candidate(env_cfg: DominoCadLinkageEnvCfg, candidate_path: str) -> dict[str, object]:
    candidate = load_reference_candidate(candidate_path)
    env_cfg.reference_gait_name = str(candidate["name"])
    if is_keyframe_sequence(candidate):
        env_cfg.reference_sequence_json = json.dumps(candidate, separators=(",", ":"))
    else:
        for key in REFERENCE_GAIT_PARAMETER_NAMES:
            setattr(env_cfg, f"reference_gait_{key}", float(candidate[key]))
    return candidate


def reference_action_diagnostics(policy, obs: torch.Tensor) -> dict[str, float]:
    actor_obs = policy.get_actor_obs(obs)
    if actor_obs.shape[-1] < EXPECTED_ACTION_COUNT:
        raise RuntimeError("Reference-action diagnostics require observations with appended reference actions.")
    target_actions = torch.clamp(actor_obs[:, -EXPECTED_ACTION_COUNT:].detach(), -1.0, 1.0)
    predicted_actions = policy.actor(policy.actor_obs_normalizer(actor_obs))
    raw_error = predicted_actions - target_actions
    clipped_actions = torch.clamp(predicted_actions, -1.0, 1.0)
    clipped_error = clipped_actions - target_actions
    return {
        "raw_mse": float(torch.mean(torch.square(raw_error)).detach().cpu()),
        "clipped_mse": float(torch.mean(torch.square(clipped_error)).detach().cpu()),
        "raw_mean_abs": float(torch.mean(torch.abs(predicted_actions)).detach().cpu()),
        "raw_max_abs": float(torch.max(torch.abs(predicted_actions)).detach().cpu()),
        "clipped_mean_abs": float(torch.mean(torch.abs(clipped_actions)).detach().cpu()),
    }


def action_noise_std_mean(policy) -> float:
    with torch.no_grad():
        if hasattr(policy, "std"):
            return float(torch.mean(policy.std.detach()).cpu())
        if hasattr(policy, "log_std"):
            return float(torch.mean(torch.exp(policy.log_std.detach())).cpu())
    return float("nan")


def force_action_noise_std(policy, noise_std: float) -> None:
    with torch.no_grad():
        if hasattr(policy, "std"):
            policy.std.fill_(float(noise_std))
            return
        if hasattr(policy, "log_std"):
            policy.log_std.fill_(math.log(max(float(noise_std), 1e-6)))
            return
    raise RuntimeError("Could not set RSL-RL policy action noise; policy has neither std nor log_std.")


def reference_action_bc_weights(
    shoulder_weight: float,
    lower_linkage_weight: float,
    upper_pitch_weight: float,
    device: str,
) -> torch.Tensor:
    role_weights = {
        "shoulder_ab_ad": float(shoulder_weight),
        "lower_linkage_drive": float(lower_linkage_weight),
        "upper_pitch_drive": float(upper_pitch_weight),
    }
    weights = [role_weights[CAD_ACTION_ROLES[name]] for name in ACTION_JOINT_NAMES]
    if any(weight <= 0.0 for weight in weights):
        raise ValueError(f"BC action weights must be positive, received {weights}.")
    return torch.tensor(weights, dtype=torch.float32, device=device).reshape(1, -1)


def weighted_action_mse(error: torch.Tensor, weights: torch.Tensor) -> torch.Tensor:
    return torch.mean(torch.square(error) * weights) / torch.mean(weights)


def pause_omniverse_timeline() -> str:
    try:
        import omni.timeline  # noqa: PLC0415

        omni.timeline.get_timeline_interface().pause()
        return "paused"
    except Exception as exc:  # pragma: no cover - defensive against Kit version differences.
        return f"pause_failed: {exc}"


def body_reference_report(env: DominoCadLinkageEnv) -> list[dict[str, object]]:
    rows = []
    for env_index in range(env.num_envs):
        position, orientation, linear_velocity, angular_velocity = env._body_reference_state(env_index)
        projected_gravity = projected_gravity_from_quat(orientation)
        tilt_rad = math.acos(max(-1.0, min(1.0, -float(projected_gravity[2]))))
        local_position = position - env._env_origins_np[env_index]
        rows.append(
            {
                "env_index": int(env_index),
                "local_position_m": [round(float(value), 6) for value in local_position],
                "body_height_m": round(float(local_position[2]), 6),
                "tilt_deg": round(math.degrees(tilt_rad), 6),
                "linear_velocity_m_s": [round(float(value), 6) for value in linear_velocity],
                "angular_velocity_rad_s": [round(float(value), 6) for value in angular_velocity],
            }
        )
    return rows


def refresh_visible_view(env: DominoCadLinkageEnv, frames: int) -> None:
    for _ in range(max(int(frames), 0)):
        try:
            env.render(recompute=True)
        except Exception:
            env.sim.render()
        simulation_app.update()


def set_domino_inspection_camera(env: DominoCadLinkageEnv | None = None) -> None:
    if not VIEWABLE_RUN:
        return
    try:
        from isaacsim.core.utils.viewports import set_camera_view  # noqa: PLC0415

        focus = np.array((0.08, 0.05, 0.08), dtype=np.float64)
        if env is not None and env.num_envs > 0:
            body_position, _, _, _ = env._body_reference_state(0)
            focus = np.asarray(body_position, dtype=np.float64)
        set_camera_view(
            eye=tuple((focus + np.array((0.72, -0.62, 0.28))).tolist()),
            target=tuple((focus + np.array((0.0, 0.0, -0.04))).tolist()),
        )
    except Exception as exc:  # pragma: no cover - viewport APIs vary across Isaac Sim builds.
        print(f"[WARN] Could not set Domino inspection camera: {exc}")


def policy_observation_tensor(observations):
    if isinstance(observations, dict):
        return observations["policy"]
    if hasattr(observations, "keys") and "policy" in list(observations.keys()):
        return observations["policy"]
    return observations


def move_observations_to_device(observations, device: str):
    if isinstance(observations, dict):
        return {
            key: value.to(device) if hasattr(value, "to") else value
            for key, value in observations.items()
        }
    return observations.to(device)


def body_relative_actual_cad_visual_feet(env: DominoCadLinkageEnv, env_index: int) -> np.ndarray:
    body_position, body_orientation, _, _ = env._body_reference_state(env_index)
    visual_feet = env._actual_cad_visual_foot_positions(env_index, rendered=True)
    if visual_feet.size == 0:
        return visual_feet.reshape(0, 3).astype(np.float32)
    world_from_body = quat_wxyz_to_rotation_matrix(body_orientation)
    return ((visual_feet - body_position.reshape(1, 3)) @ world_from_body).astype(np.float32)


def body_relative_reward_feet(env: DominoCadLinkageEnv, env_index: int) -> np.ndarray:
    body_position, body_orientation, _, _ = env._body_reference_state(env_index)
    foot_positions, _ = env._reward_foot_positions_and_radii(env_index)
    world_from_body = quat_wxyz_to_rotation_matrix(body_orientation)
    return ((foot_positions - body_position.reshape(1, 3)) @ world_from_body).astype(np.float32)


def prepare_hold_open_view(env: DominoCadLinkageEnv, mode: str, render_frames: int) -> dict[str, object]:
    report: dict[str, object] = {
        "enabled": True,
        "mode": mode,
        "timeline": pause_omniverse_timeline(),
        "state_before_display": body_reference_report(env),
    }
    if mode == "reset":
        env.reset()
        report["display_state"] = body_reference_report(env)
    else:
        report["display_state"] = report["state_before_display"]
    refresh_visible_view(env, render_frames)
    report["render_frames"] = max(int(render_frames), 0)
    return report


def continue_visible_trained_policy_loop(
    runner: OnPolicyRunner,
    wrapped_env: RslRlVecEnvWrapper,
    env: DominoCadLinkageEnv,
    device: str,
    hold_open_exit_after_frames: int,
) -> None:
    policy = runner.get_inference_policy(device=device)
    obs = move_observations_to_device(wrapped_env.get_observations(), device)
    hold_open_frame_count = 0
    with torch.inference_mode():
        while simulation_app.is_running():
            actions = policy(obs)
            obs, rewards, _, _ = wrapped_env.step(actions.to(wrapped_env.device))
            if (
                not torch.isfinite(actions).all()
                or not torch.isfinite(policy_observation_tensor(obs)).all()
                or not torch.isfinite(rewards).all()
            ):
                raise RuntimeError("Non-finite action, observation, or reward during visible trained-policy rollout.")
            obs = move_observations_to_device(obs, device)
            try:
                env.render(recompute=False)
            except Exception:
                env.sim.render()
            simulation_app.update()
            hold_open_frame_count += 1
            if hold_open_exit_after_frames and hold_open_frame_count >= hold_open_exit_after_frames:
                break


def validate_trained_policy_rollout(
    runner: OnPolicyRunner,
    wrapped_env: RslRlVecEnvWrapper,
    env: DominoCadLinkageEnv,
    device: str,
    steps: int,
    gate_args: argparse.Namespace,
) -> dict[str, object]:
    env.reset()
    obs = move_observations_to_device(wrapped_env.get_observations(), device)
    policy = runner.get_inference_policy(device=device)

    settle_steps = max(int(gate_args.policy_validation_settle_steps), 0)
    ramp_steps = max(int(gate_args.policy_validation_ramp_steps), 0)
    settle_done_count = 0
    settle_terminated_count = 0
    settle_timeout_count = 0
    finite = True
    zero_actions = torch.zeros(
        (wrapped_env.num_envs, EXPECTED_ACTION_COUNT),
        dtype=torch.float32,
        device=wrapped_env.device,
    )
    with torch.inference_mode():
        for _ in range(settle_steps):
            obs, rewards, dones, _ = wrapped_env.step(zero_actions)
            settle_done_count += int(torch.count_nonzero(dones).detach().cpu())
            settle_terminated_count += int(torch.count_nonzero(env.reset_terminated).detach().cpu())
            settle_timeout_count += int(torch.count_nonzero(env.reset_time_outs).detach().cpu())
            if (
                not torch.isfinite(policy_observation_tensor(obs)).all()
                or not torch.isfinite(rewards).all()
            ):
                finite = False
                break
            obs = move_observations_to_device(obs, device)
    if hasattr(env, "episode_length_buf"):
        env.episode_length_buf[:] = 0
    if hasattr(env, "reset_terminated"):
        env.reset_terminated[:] = False
    if hasattr(env, "reset_time_outs"):
        env.reset_time_outs[:] = False
    env._previous_body_reference_positions = torch.tensor(
        np.vstack([env._body_reference_state(env_index)[0] for env_index in range(env.num_envs)]),
        dtype=torch.float32,
        device=env.device,
    )
    obs = move_observations_to_device(wrapped_env.get_observations(), device)
    startup_leg_stance = env.leg_start_stance_report()
    startup_body_state = body_reference_report(env)

    initial_body_positions = []
    initial_body_yaws = []
    for env_index in range(env.num_envs):
        position, orientation, _, _ = env._body_reference_state(env_index)
        initial_body_positions.append(position.copy())
        initial_body_yaws.append(yaw_from_quat_wxyz(orientation))

    initial_foot_positions = [env._foot_positions(env_index).copy() for env_index in range(env.num_envs)]
    max_foot_displacements = [
        np.zeros(positions.shape[0], dtype=np.float32)
        for positions in initial_foot_positions
    ]
    initial_visual_foot_positions = [
        env._actual_cad_visual_foot_positions(env_index, rendered=True).copy()
        for env_index in range(env.num_envs)
    ]
    max_actual_cad_visual_foot_clearances = [
        (
            positions[:, 2]
            - env._terrain_heights_np(positions, env_index=env_index)
        ).astype(np.float32)
        for env_index, positions in enumerate(initial_visual_foot_positions)
    ]
    max_visual_foot_displacements = [
        np.zeros(positions.shape[0], dtype=np.float32)
        for positions in initial_visual_foot_positions
    ]
    initial_body_relative_visual_foot_positions = [
        body_relative_actual_cad_visual_feet(env, env_index).copy()
        for env_index in range(env.num_envs)
    ]
    max_body_relative_visual_foot_displacements = [
        np.zeros(positions.shape[0], dtype=np.float32)
        for positions in initial_body_relative_visual_foot_positions
    ]
    foot_endpoint_motion_tracker = create_foot_endpoint_motion_tracker(env)
    linkage_motion_tracker = create_linkage_motion_tracker(env)
    gait_cycle_tracker = GaitCycleTracker(
        env.num_envs,
        FOOT_BODY_NAMES,
        min_air_steps=int(gate_args.gait_cycle_min_air_steps),
        touchdown_confirm_steps=int(gate_args.gait_cycle_touchdown_confirm_steps),
        min_clearance_m=float(gate_args.gait_cycle_min_clearance_m),
        min_body_relative_travel_m=float(
            gate_args.gait_cycle_min_body_relative_travel_m
        ),
        max_tilt_deg=float(gate_args.gait_cycle_max_tilt_deg),
        min_touchdown_support_feet=int(
            gate_args.gait_cycle_min_touchdown_support_feet
        ),
    )
    for env_index in range(env.num_envs):
        reward_foot_positions, reward_foot_radii = env._reward_foot_positions_and_radii(
            env_index
        )
        gait_cycle_tracker.initialize_env(
            env_index,
            env._foot_contact_flags_np(
                reward_foot_positions,
                env_index,
                radii_m=reward_foot_radii,
            ),
            env._foot_ground_clearance_np(
                reward_foot_positions,
                env_index,
                radii_m=reward_foot_radii,
            ),
            body_relative_reward_feet(env, env_index),
        )
    done_count = 0
    terminated_count = 0
    timeout_count = 0
    max_body_tilt_deg = 0.0
    min_body_height_m = float("inf")
    swing_contact_sum = 0.0
    swing_clearance_sum = 0.0
    swing_sample_count = 0
    foot_contact_sum = 0.0
    gait_contact_match_sum = 0.0
    max_joint_separation_during_rollout_m = 0.0

    with torch.inference_mode():
        for rollout_step in range(max(int(steps), 0)):
            actions = policy(obs)
            if float(gate_args.policy_reference_action_snap_tolerance) > 0.0:
                reference_from_obs = policy_observation_tensor(obs)[:, -EXPECTED_ACTION_COUNT:]
                close_to_reference = torch.max(
                    torch.abs(actions - reference_from_obs), dim=1
                ).values <= float(gate_args.policy_reference_action_snap_tolerance)
                actions = torch.where(
                    close_to_reference.unsqueeze(1),
                    reference_from_obs,
                    actions,
                )
            if ramp_steps > 0:
                blend = min(1.0, float(rollout_step + 1) / float(ramp_steps))
                actions = actions * blend
            obs, rewards, dones, _ = wrapped_env.step(actions.to(wrapped_env.device))
            if (
                not torch.isfinite(actions).all()
                or not torch.isfinite(policy_observation_tensor(obs)).all()
                or not torch.isfinite(rewards).all()
            ):
                finite = False
                break
            obs = move_observations_to_device(obs, device)
            done_count += int(torch.count_nonzero(dones).detach().cpu())
            terminated_count += int(torch.count_nonzero(env.reset_terminated).detach().cpu())
            timeout_count += int(torch.count_nonzero(env.reset_time_outs).detach().cpu())
            done_flags = dones.detach().cpu().numpy().astype(bool)
            update_linkage_motion_tracker(linkage_motion_tracker, env)
            update_foot_endpoint_motion_tracker(foot_endpoint_motion_tracker, env)
            for env_index in range(env.num_envs):
                position, orientation, _, _ = env._body_reference_state(env_index)
                body_height = float(position[2] - env._env_origins_np[env_index][2])
                min_body_height_m = min(min_body_height_m, body_height)
                projected_gravity = projected_gravity_from_quat(orientation)
                tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, -float(projected_gravity[2])))))
                max_body_tilt_deg = max(max_body_tilt_deg, tilt_deg)

                foot_positions = env._foot_positions(env_index)
                max_foot_displacements[env_index] = np.maximum(
                    max_foot_displacements[env_index],
                    np.linalg.norm(foot_positions - initial_foot_positions[env_index], axis=1).astype(np.float32),
                )
                visual_foot_positions = env._actual_cad_visual_foot_positions(env_index, rendered=True)
                if visual_foot_positions.shape[0] > 0:
                    visual_clearances = (
                        visual_foot_positions[:, 2]
                        - env._terrain_heights_np(visual_foot_positions, env_index=env_index)
                    ).astype(np.float32)
                    max_actual_cad_visual_foot_clearances[env_index] = np.maximum(
                        max_actual_cad_visual_foot_clearances[env_index],
                        visual_clearances,
                    )
                    max_visual_foot_displacements[env_index] = np.maximum(
                        max_visual_foot_displacements[env_index],
                        np.linalg.norm(
                            visual_foot_positions - initial_visual_foot_positions[env_index],
                            axis=1,
                        ).astype(np.float32),
                    )
                    body_relative_visual_feet = body_relative_actual_cad_visual_feet(env, env_index)
                    max_body_relative_visual_foot_displacements[env_index] = np.maximum(
                        max_body_relative_visual_foot_displacements[env_index],
                        np.linalg.norm(
                            body_relative_visual_feet - initial_body_relative_visual_foot_positions[env_index],
                            axis=1,
                        ).astype(np.float32),
                    )
                max_joint_separation_during_rollout_m = max(
                    max_joint_separation_during_rollout_m,
                    max(
                        (
                            float(row["separation_m"])
                            for row in env._joint_separation_rows(env_index)
                        ),
                        default=0.0,
                    ),
                )
                reward_foot_positions, reward_foot_radii = env._reward_foot_positions_and_radii(env_index)
                foot_contacts = env._foot_contact_flags_np(
                    reward_foot_positions,
                    env_index,
                    radii_m=reward_foot_radii,
                )
                ground_clearance = np.maximum(
                    0.0,
                    env._foot_ground_clearance_np(
                        reward_foot_positions,
                        env_index,
                        radii_m=reward_foot_radii,
                    ),
                )
                gait_cycle_tracker.update_env(
                    env_index,
                    rollout_step,
                    foot_contacts,
                    ground_clearance,
                    body_relative_reward_feet(env, env_index),
                    body_tilt_deg=tilt_deg,
                    done=bool(done_flags[env_index]),
                )
                command = env._commands[env_index].detach().cpu().numpy().astype(np.float32)
                desired_stance = env._desired_stance_np(env_index, command)
                desired_swing = 1.0 - desired_stance
                stance_count = max(float(np.sum(desired_stance)), 1.0)
                swing_count = max(float(np.sum(desired_swing)), 1.0)
                foot_contact_sum += float(np.sum(foot_contacts))
                gait_contact_match_sum += float(np.sum(1.0 - np.abs(foot_contacts - desired_stance))) / len(foot_contacts)
                swing_contact_sum += float(np.sum(foot_contacts * desired_swing)) / swing_count
                if float(np.sum(desired_swing)) > 0.0:
                    swing_clearance_sum += float(np.sum(ground_clearance * desired_swing))
                    swing_sample_count += int(np.sum(desired_swing))

    final_displacements = []
    final_yaw_drifts = []
    for env_index in range(env.num_envs):
        position, orientation, _, _ = env._body_reference_state(env_index)
        displacement = position - initial_body_positions[env_index]
        final_displacements.append([round(float(value), 6) for value in displacement])
        final_yaw_drifts.append(round(float(wrap_angle_rad(yaw_from_quat_wxyz(orientation) - initial_body_yaws[env_index])), 6))

    joint_report = env.joint_separation_report()
    max_joint_separation = max(
        max_joint_separation_during_rollout_m,
        float(joint_report.get("max_separation_m", 0.0) or 0.0),
    )
    max_foot_displacement_per_env = [
        [round(float(value), 6) for value in values.tolist()]
        for values in max_foot_displacements
    ]
    min_foot_motion_m = min(
        (float(np.min(values)) for values in max_foot_displacements if values.size),
        default=0.0,
    )
    max_visual_foot_motion_m = max(
        (float(np.max(values)) for values in max_visual_foot_displacements if values.size),
        default=0.0,
    )
    min_body_relative_visual_foot_motion_m = min(
        (float(np.min(values)) for values in max_body_relative_visual_foot_displacements if values.size),
        default=0.0,
    )
    min_each_actual_cad_foot_peak_clearance_m = min(
        (float(np.min(values)) for values in max_actual_cad_visual_foot_clearances if values.size),
        default=0.0,
    )
    final_forward_m = float(np.mean([row[0] for row in final_displacements])) if final_displacements else 0.0
    max_abs_lateral_m = max((abs(float(row[1])) for row in final_displacements), default=0.0)
    max_abs_yaw_rad = max((abs(float(value)) for value in final_yaw_drifts), default=0.0)
    sample_count = max(max(int(steps), 1) * env.num_envs, 1)
    mean_swing_contact = swing_contact_sum / sample_count
    mean_swing_clearance_m = swing_clearance_sum / max(swing_sample_count, 1)
    mean_foot_contacts_per_env = foot_contact_sum / sample_count
    mean_gait_contact_match = gait_contact_match_sum / sample_count
    measured_linkage_motion = linkage_motion_report(linkage_motion_tracker)
    measured_foot_endpoint_motion = foot_endpoint_motion_report(foot_endpoint_motion_tracker)
    gait_cycle_report = gait_cycle_tracker.summary()

    failures = []
    if not finite:
        failures.append("non_finite_action_observation_or_reward")
    if settle_done_count:
        failures.append(f"startup_settle_done_count={settle_done_count}")
    if settle_terminated_count:
        failures.append(f"startup_settle_terminated_count={settle_terminated_count}")
    if done_count:
        failures.append(f"done_count={done_count}")
    if terminated_count:
        failures.append(f"terminated_count={terminated_count}")
    if max_joint_separation > float(gate_args.policy_gate_max_joint_separation_m):
        failures.append(f"joint_separation_m={max_joint_separation:.6f}")
    if final_forward_m < float(gate_args.policy_gate_min_forward_m):
        failures.append(f"forward_m={final_forward_m:.6f}")
    if max_abs_lateral_m > float(gate_args.policy_gate_max_lateral_m):
        failures.append(f"lateral_m={max_abs_lateral_m:.6f}")
    if max_abs_yaw_rad > float(gate_args.policy_gate_max_yaw_rad):
        failures.append(f"yaw_rad={max_abs_yaw_rad:.6f}")
    if max_body_tilt_deg > float(gate_args.policy_gate_max_tilt_deg):
        failures.append(f"tilt_deg={max_body_tilt_deg:.6f}")
    if mean_swing_contact > float(gate_args.policy_gate_max_swing_contact):
        failures.append(f"swing_contact={mean_swing_contact:.6f}")
    if mean_gait_contact_match < float(gate_args.policy_gate_min_gait_contact_match):
        failures.append(f"gait_contact_match={mean_gait_contact_match:.6f}")
    if mean_swing_clearance_m < float(gate_args.policy_gate_min_swing_clearance_m):
        failures.append(f"swing_clearance_m={mean_swing_clearance_m:.6f}")
    if min_each_actual_cad_foot_peak_clearance_m < float(
        gate_args.policy_gate_min_each_cad_foot_clearance_m
    ):
        failures.append(
            "min_each_actual_cad_foot_peak_clearance_m="
            f"{min_each_actual_cad_foot_peak_clearance_m:.6f}"
        )
    min_hip_relative_visual_foot_motion_m = float(
        measured_foot_endpoint_motion["min_each_foot_motion_m"]
    )
    if min_hip_relative_visual_foot_motion_m < float(gate_args.policy_gate_min_foot_motion_m):
        failures.append(
            f"min_hip_carriage_relative_actual_cad_foot_motion_m={min_hip_relative_visual_foot_motion_m:.6f}"
        )
    if float(measured_linkage_motion["min_each_drive_motion_deg"]) < float(
        gate_args.policy_gate_min_each_linkage_drive_motion_deg
    ):
        failures.append(
            "min_each_linkage_drive_motion_deg="
            f"{float(measured_linkage_motion['min_each_drive_motion_deg']):.6f}"
        )
    if max_visual_foot_motion_m > float(gate_args.policy_gate_max_visual_foot_motion_m):
        failures.append(f"visual_foot_motion_m={max_visual_foot_motion_m:.6f}")
    if int(gait_cycle_report["min_valid_cycles_per_env_foot"]) < int(
        gate_args.policy_gate_min_valid_cycles_per_foot
    ):
        failures.append(
            "min_valid_liftoff_touchdown_cycles_per_env_foot="
            f"{int(gait_cycle_report['min_valid_cycles_per_env_foot'])}"
        )
    if float(gait_cycle_report["valid_cycle_ratio"]) < float(
        gate_args.policy_gate_min_valid_cycle_ratio
    ):
        failures.append(
            "valid_liftoff_touchdown_cycle_ratio="
            f"{float(gait_cycle_report['valid_cycle_ratio']):.6f}"
        )
    if float(gait_cycle_report["max_foot_valid_cycle_share"]) > float(
        gate_args.policy_gate_max_foot_cycle_domination_ratio
    ):
        failures.append(
            "max_foot_valid_cycle_share="
            f"{float(gait_cycle_report['max_foot_valid_cycle_share']):.6f}"
        )

    return {
        "enabled": True,
        "steps": int(steps),
        "settle_steps": int(settle_steps),
        "policy_ramp_steps": int(ramp_steps),
        "passed": len(failures) == 0,
        "failures": failures,
        "thresholds": {
            "max_joint_separation_m": float(gate_args.policy_gate_max_joint_separation_m),
            "min_forward_m": float(gate_args.policy_gate_min_forward_m),
            "max_lateral_m": float(gate_args.policy_gate_max_lateral_m),
            "max_yaw_rad": float(gate_args.policy_gate_max_yaw_rad),
            "max_tilt_deg": float(gate_args.policy_gate_max_tilt_deg),
            "max_swing_contact": float(gate_args.policy_gate_max_swing_contact),
            "min_gait_contact_match": float(gate_args.policy_gate_min_gait_contact_match),
            "min_swing_clearance_m": float(gate_args.policy_gate_min_swing_clearance_m),
            "min_each_actual_cad_foot_clearance_m": float(
                gate_args.policy_gate_min_each_cad_foot_clearance_m
            ),
            "min_foot_motion_m": float(gate_args.policy_gate_min_foot_motion_m),
            "min_each_linkage_drive_motion_deg": float(
                gate_args.policy_gate_min_each_linkage_drive_motion_deg
            ),
            "max_visual_foot_motion_m": float(gate_args.policy_gate_max_visual_foot_motion_m),
            "min_valid_cycles_per_foot": int(
                gate_args.policy_gate_min_valid_cycles_per_foot
            ),
            "min_valid_cycle_ratio": float(
                gate_args.policy_gate_min_valid_cycle_ratio
            ),
            "max_foot_cycle_domination_ratio": float(
                gate_args.policy_gate_max_foot_cycle_domination_ratio
            ),
        },
        "done_count": int(done_count),
        "terminated_count": int(terminated_count),
        "timeout_count": int(timeout_count),
        "startup_settle_done_count": int(settle_done_count),
        "startup_settle_terminated_count": int(settle_terminated_count),
        "startup_settle_timeout_count": int(settle_timeout_count),
        "joint_separation": joint_report,
        "max_joint_separation_during_rollout_m": round(max_joint_separation, 6),
        "linkage_drive_motion": measured_linkage_motion,
        "hip_carriage_relative_actual_cad_foot_motion": measured_foot_endpoint_motion,
        "foot_liftoff_touchdown_cycles": gait_cycle_report,
        "final_body_reference_displacement_m": final_displacements,
        "final_yaw_heading_drift_rad": final_yaw_drifts,
        "final_forward_m": round(final_forward_m, 6),
        "max_abs_lateral_m": round(max_abs_lateral_m, 6),
        "max_abs_yaw_rad": round(max_abs_yaw_rad, 6),
        "min_body_reference_height_m": round(min_body_height_m, 6),
        "max_body_tilt_deg": round(max_body_tilt_deg, 6),
        "mean_foot_contacts_per_env": round(mean_foot_contacts_per_env, 6),
        "mean_gait_contact_match": round(mean_gait_contact_match, 6),
        "mean_swing_contact": round(mean_swing_contact, 6),
        "mean_swing_clearance_m": round(mean_swing_clearance_m, 6),
        "min_each_actual_cad_foot_peak_clearance_m": round(
            min_each_actual_cad_foot_peak_clearance_m,
            6,
        ),
        "max_actual_cad_foot_clearance_m": [
            [round(float(value), 6) for value in values.tolist()]
            for values in max_actual_cad_visual_foot_clearances
        ],
        "min_foot_motion_m": round(min_foot_motion_m, 6),
        "min_body_relative_actual_cad_foot_motion_m": round(min_body_relative_visual_foot_motion_m, 6),
        "min_hip_carriage_relative_actual_cad_foot_motion_m": round(
            min_hip_relative_visual_foot_motion_m,
            6,
        ),
        "max_foot_displacement_m": max_foot_displacement_per_env,
        "max_actual_cad_visual_foot_motion_m": round(max_visual_foot_motion_m, 6),
        "max_actual_cad_visual_foot_displacement_m": [
            [round(float(value), 6) for value in values.tolist()]
            for values in max_visual_foot_displacements
        ],
        "max_body_relative_actual_cad_foot_displacement_m": [
            [round(float(value), 6) for value in values.tolist()]
            for values in max_body_relative_visual_foot_displacements
        ],
        "startup_leg_stance": startup_leg_stance,
        "startup_body_state": startup_body_state,
        "leg_state_after_validation": env.leg_start_stance_report(),
        "state_after_validation": body_reference_report(env),
    }


def initialize_reference_action_identity_actor(policy) -> dict[str, object]:
    """Make an ELU actor copy the appended 12-channel reference target exactly."""
    actor = policy.actor
    linear_layers = [module for module in actor.modules() if isinstance(module, torch.nn.Linear)]
    if len(linear_layers) < 2:
        raise RuntimeError("Reference identity initialization requires at least one hidden linear layer.")
    first = linear_layers[0]
    hidden_layers = linear_layers[:-1]
    output = linear_layers[-1]
    if first.in_features < EXPECTED_ACTION_COUNT or output.out_features != EXPECTED_ACTION_COUNT:
        raise RuntimeError("Actor dimensions cannot represent the 12-channel reference identity map.")
    if any(
        layer.in_features < EXPECTED_ACTION_COUNT or layer.out_features < EXPECTED_ACTION_COUNT
        for layer in hidden_layers
    ):
        raise RuntimeError("Every actor hidden layer must carry all 12 reference-action channels.")
    if output.in_features < EXPECTED_ACTION_COUNT:
        raise RuntimeError("Actor output layer cannot decode all 12 reference-action channels.")

    reference_start = first.in_features - EXPECTED_ACTION_COUNT
    positive_bias = 2.0
    with torch.no_grad():
        for parameter in actor.parameters():
            parameter.zero_()
        for action_index in range(EXPECTED_ACTION_COUNT):
            first.weight[action_index, reference_start + action_index] = 1.0
            first.bias[action_index] = positive_bias
            for hidden in hidden_layers[1:]:
                hidden.weight[action_index, action_index] = 1.0
            output.weight[action_index, action_index] = 1.0
            output.bias[action_index] = -positive_bias

    return {
        "enabled": True,
        "linear_layer_count": len(linear_layers),
        "hidden_layer_dims": [int(layer.out_features) for layer in hidden_layers],
        "actor_input_dim": int(first.in_features),
        "reference_input_start": int(reference_start),
        "reference_action_count": EXPECTED_ACTION_COUNT,
        "positive_elu_bias": positive_bias,
        "method": "exact_appended_reference_identity_through_positive_elu_region",
    }


def behavior_clone_reference_actions(
    runner: OnPolicyRunner,
    wrapped_env: RslRlVecEnvWrapper,
    env: DominoCadLinkageEnv,
    steps: int,
    learning_rate: float,
    output_penalty: float,
    action_weights: torch.Tensor,
    replay_steps: int = 0,
    replay_batch_size: int = 256,
) -> dict[str, float | int]:
    if int(steps) <= 0:
        return {"steps": 0}
    policy = runner.alg.policy
    policy.train()
    optimizer = torch.optim.AdamW(policy.actor.parameters(), lr=float(learning_rate), weight_decay=0.0)
    obs = wrapped_env.get_observations().to(runner.device)
    replay_observations = []
    replay_targets = []

    def reference_targets(actor_obs: torch.Tensor) -> torch.Tensor:
        if bool(env.cfg.include_reference_actions_in_observation):
            if actor_obs.shape[-1] < EXPECTED_ACTION_COUNT:
                raise RuntimeError(
                    "Reference-action BC requires appended reference actions."
                )
            return torch.clamp(
                actor_obs[:, -EXPECTED_ACTION_COUNT:].detach(),
                -1.0,
                1.0,
            )
        target_actions = torch.as_tensor(
            env._reference_actions_np(),
            dtype=actor_obs.dtype,
            device=actor_obs.device,
        )
        if target_actions.shape != (actor_obs.shape[0], EXPECTED_ACTION_COUNT):
            raise RuntimeError(
                "Open-policy reference targets do not match the actor batch: "
                f"targets={tuple(target_actions.shape)}, actor={tuple(actor_obs.shape)}."
            )
        return torch.clamp(target_actions, -1.0, 1.0)

    for _ in range(max(int(replay_steps), 0)):
        actor_obs = policy.get_actor_obs(obs).detach()
        target_actions = reference_targets(actor_obs)
        replay_observations.append(actor_obs.clone())
        replay_targets.append(target_actions.clone())
        with torch.inference_mode():
            obs, _, _, _ = wrapped_env.step(target_actions.to(wrapped_env.device))
            obs = obs.to(runner.device)

    replay_actor_obs = None
    replay_target_actions = None
    if replay_observations:
        replay_actor_obs = torch.cat(replay_observations, dim=0)
        replay_target_actions = torch.cat(replay_targets, dim=0)

    def action_diagnostics(
        actor_obs: torch.Tensor,
        target_actions: torch.Tensor,
    ) -> dict[str, float]:
        with torch.inference_mode():
            predicted = policy.actor(policy.actor_obs_normalizer(actor_obs))
            raw_error = predicted - target_actions
            clipped = torch.clamp(predicted, -1.0, 1.0)
            clipped_error = clipped - target_actions
            per_channel_mse = torch.mean(torch.square(clipped_error), dim=0)
            return {
                "raw_mse": float(torch.mean(torch.square(raw_error)).detach().cpu()),
                "clipped_mse": float(torch.mean(torch.square(clipped_error)).detach().cpu()),
                "raw_mean_abs": float(torch.mean(torch.abs(predicted)).detach().cpu()),
                "raw_max_abs": float(torch.max(torch.abs(predicted)).detach().cpu()),
                "clipped_mean_abs": float(torch.mean(torch.abs(clipped)).detach().cpu()),
                "max_channel_clipped_mse": float(torch.max(per_channel_mse).detach().cpu()),
            }

    def diagnostics() -> dict[str, float]:
        if replay_actor_obs is not None and replay_target_actions is not None:
            return action_diagnostics(replay_actor_obs, replay_target_actions)
        actor_obs = policy.get_actor_obs(obs).detach()
        return action_diagnostics(actor_obs, reference_targets(actor_obs))

    initial_diag = diagnostics()
    losses = []
    raw_mses = []
    clipped_mses = []
    for _ in range(int(steps)):
        if replay_actor_obs is not None and replay_target_actions is not None:
            batch_size = min(max(int(replay_batch_size), 1), replay_actor_obs.shape[0])
            indexes = torch.randint(0, replay_actor_obs.shape[0], (batch_size,), device=replay_actor_obs.device)
            actor_obs = replay_actor_obs[indexes]
            target_actions = replay_target_actions[indexes]
        else:
            actor_obs = policy.get_actor_obs(obs)
            target_actions = reference_targets(actor_obs)
        predicted_actions = policy.actor(policy.actor_obs_normalizer(actor_obs))
        raw_mse = weighted_action_mse(predicted_actions - target_actions, action_weights)
        clipped_mse = weighted_action_mse(torch.clamp(predicted_actions, -1.0, 1.0) - target_actions, action_weights)
        output_l2 = torch.mean(torch.square(predicted_actions))
        loss = raw_mse + (float(output_penalty) * output_l2)
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(policy.actor.parameters(), 0.5)
        optimizer.step()
        losses.append(float(loss.detach().cpu()))
        raw_mses.append(float(raw_mse.detach().cpu()))
        clipped_mses.append(float(clipped_mse.detach().cpu()))
        if replay_actor_obs is None:
            with torch.inference_mode():
                obs, _, _, _ = wrapped_env.step(target_actions.to(wrapped_env.device))
                obs = obs.to(runner.device)
    policy.train()
    final_diag = diagnostics()
    return {
        "steps": int(steps),
        "replay_steps": max(int(replay_steps), 0),
        "replay_samples": int(replay_actor_obs.shape[0]) if replay_actor_obs is not None else 0,
        "replay_batch_size": min(max(int(replay_batch_size), 1), int(replay_actor_obs.shape[0])) if replay_actor_obs is not None else 0,
        "learning_rate": float(learning_rate),
        "output_penalty": float(output_penalty),
        "action_weights": {
            "shoulder_ab_ad": round(float(action_weights[0, 0].detach().cpu()), 6),
            "lower_linkage_drive": round(float(action_weights[0, 1].detach().cpu()), 6),
            "upper_pitch_drive": round(float(action_weights[0, 2].detach().cpu()), 6),
        },
        "initial_raw_mse": round(float(initial_diag["raw_mse"]), 6),
        "initial_clipped_mse": round(float(initial_diag["clipped_mse"]), 6),
        "initial_raw_max_abs": round(float(initial_diag["raw_max_abs"]), 6),
        "initial_max_channel_clipped_mse": round(float(initial_diag.get("max_channel_clipped_mse", 0.0)), 6),
        "final_raw_mse": round(float(final_diag["raw_mse"]), 6),
        "final_clipped_mse": round(float(final_diag["clipped_mse"]), 6),
        "final_raw_mean_abs": round(float(final_diag["raw_mean_abs"]), 6),
        "final_raw_max_abs": round(float(final_diag["raw_max_abs"]), 6),
        "final_max_channel_clipped_mse": round(float(final_diag.get("max_channel_clipped_mse", 0.0)), 6),
        "mean_loss": round(float(sum(losses) / max(len(losses), 1)), 6),
        "mean_raw_mse": round(float(sum(raw_mses) / max(len(raw_mses), 1)), 6),
        "mean_clipped_mse": round(float(sum(clipped_mses) / max(len(clipped_mses), 1)), 6),
    }


def settle_behavior_clone_start(
    wrapped_env: RslRlVecEnvWrapper,
    env: DominoCadLinkageEnv,
    steps: int,
) -> dict[str, int | bool]:
    settle_steps = max(int(steps), 0)
    zero_actions = torch.zeros(
        (wrapped_env.num_envs, EXPECTED_ACTION_COUNT),
        dtype=torch.float32,
        device=wrapped_env.device,
    )
    done_count = 0
    terminated_count = 0
    timeout_count = 0
    finite = True
    with torch.inference_mode():
        for _ in range(settle_steps):
            obs, rewards, dones, _ = wrapped_env.step(zero_actions)
            done_count += int(torch.count_nonzero(dones).detach().cpu())
            terminated_count += int(torch.count_nonzero(env.reset_terminated).detach().cpu())
            timeout_count += int(torch.count_nonzero(env.reset_time_outs).detach().cpu())
            if not torch.isfinite(policy_observation_tensor(obs)).all() or not torch.isfinite(rewards).all():
                finite = False
                break
    if hasattr(env, "episode_length_buf"):
        env.episode_length_buf[:] = 0
    if hasattr(env, "reset_terminated"):
        env.reset_terminated[:] = False
    if hasattr(env, "reset_time_outs"):
        env.reset_time_outs[:] = False
    env._previous_body_reference_positions = torch.tensor(
        np.vstack([env._body_reference_state(env_index)[0] for env_index in range(env.num_envs)]),
        dtype=torch.float32,
        device=env.device,
    )
    return {
        "steps": settle_steps,
        "finite": finite,
        "done_count": done_count,
        "terminated_count": terminated_count,
        "timeout_count": timeout_count,
    }


def main() -> None:
    main_started_at = time.perf_counter()
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    torch.backends.cudnn.deterministic = False
    torch.backends.cudnn.benchmark = False

    env_cfg = DominoCadLinkageEnvCfg()
    env_cfg.scene.num_envs = int(args_cli.num_envs)
    env_cfg.sim.device = args_cli.device if args_cli.physics_device == "same" else args_cli.physics_device
    env_cfg.seed = int(args_cli.seed)
    env_cfg.visible_step_delay_s = max(float(args_cli.visible_step_delay_s), 0.0)
    if VIEWABLE_RUN:
        if int(args_cli.num_envs) > 1:
            grid_width = max(1, int(math.ceil(math.sqrt(int(args_cli.num_envs)))))
            grid_rows = int(math.ceil(int(args_cli.num_envs) / grid_width))
            center_x = 0.5 * float(grid_width - 1) * float(env_cfg.scene.env_spacing)
            center_y = 0.5 * float(grid_rows - 1) * float(env_cfg.scene.env_spacing)
            env_cfg.viewer.eye = (center_x + 4.5, center_y - 7.5, 5.5)
            env_cfg.viewer.lookat = (center_x, center_y, 0.15)
        else:
            env_cfg.viewer.eye = (0.90, -0.75, 0.42)
            env_cfg.viewer.lookat = (0.10, 0.06, 0.16)
        env_cfg.viewer.origin_type = "world"
    env_cfg.include_actual_cad_visuals = not bool(args_cli.disable_actual_cad_visuals)
    env_cfg.actual_cad_mesh_dir = str(args_cli.actual_cad_mesh_dir)
    env_cfg.closure_model = str(args_cli.closure_model)
    env_cfg.terrain_type = str(args_cli.terrain_type)
    if args_cli.stairs_step_count is not None:
        env_cfg.stairs_step_count = int(args_cli.stairs_step_count)
    if args_cli.stairs_step_depth_m is not None:
        env_cfg.stairs_step_depth_m = float(args_cli.stairs_step_depth_m)
    if args_cli.stairs_step_height_m is not None:
        env_cfg.stairs_step_height_m = float(args_cli.stairs_step_height_m)
    if args_cli.stairs_width_m is not None:
        env_cfg.stairs_width_m = float(args_cli.stairs_width_m)
    if args_cli.stairs_start_x_m is not None:
        env_cfg.stairs_start_x_m = float(args_cli.stairs_start_x_m)
    if args_cli.stairs_top_platform_length_m is not None:
        env_cfg.stairs_top_platform_length_m = float(args_cli.stairs_top_platform_length_m)
    env_cfg.command_x_m_s = float(args_cli.command_x_m_s)
    env_cfg.command_y_m_s = float(args_cli.command_y_m_s)
    env_cfg.command_yaw_rad_s = float(args_cli.command_yaw_rad_s)
    env_cfg.gait_frequency_hz = float(args_cli.gait_frequency_hz)
    if args_cli.action_scale_deg is not None:
        env_cfg.action_scale_deg = float(args_cli.action_scale_deg)
    if args_cli.servo_target_rate_limit_deg_s is not None:
        env_cfg.servo_target_rate_limit_deg_s = float(args_cli.servo_target_rate_limit_deg_s)
    if args_cli.reset_settle_steps is not None:
        env_cfg.reset_settle_steps = max(int(args_cli.reset_settle_steps), 0)
    if args_cli.episode_length_s is not None:
        env_cfg.episode_length_s = float(args_cli.episode_length_s)
    if args_cli.min_height_m is not None:
        env_cfg.min_height_m = float(args_cli.min_height_m)
    if args_cli.max_tilt_deg is not None:
        env_cfg.max_tilt_deg = float(args_cli.max_tilt_deg)
    if args_cli.actual_cad_ground_clearance_m is not None:
        env_cfg.actual_cad_ground_clearance_m = max(float(args_cli.actual_cad_ground_clearance_m), 0.0)
    if args_cli.ground_size_m is not None:
        env_cfg.ground_size_m = max(float(args_cli.ground_size_m), 1.0)
    if args_cli.command_progress_reward_scale is not None:
        env_cfg.command_progress_reward_scale = float(args_cli.command_progress_reward_scale)
    if args_cli.command_velocity_reward_scale is not None:
        env_cfg.command_velocity_reward_scale = float(args_cli.command_velocity_reward_scale)
    if args_cli.command_velocity_tracking_reward_scale is not None:
        env_cfg.command_velocity_tracking_reward_scale = float(args_cli.command_velocity_tracking_reward_scale)
    if args_cli.command_velocity_tracking_sigma is not None:
        env_cfg.command_velocity_tracking_sigma = max(float(args_cli.command_velocity_tracking_sigma), 1e-6)
    if args_cli.command_stagnation_penalty_scale is not None:
        env_cfg.command_stagnation_penalty_scale = float(args_cli.command_stagnation_penalty_scale)
    if args_cli.command_stagnation_speed_m_s is not None:
        env_cfg.command_stagnation_speed_m_s = max(float(args_cli.command_stagnation_speed_m_s), 1e-6)
    if args_cli.alive_reward_scale is not None:
        env_cfg.alive_reward_scale = float(args_cli.alive_reward_scale)
    if args_cli.vertical_velocity_reward_scale is not None:
        env_cfg.vertical_velocity_reward_scale = float(args_cli.vertical_velocity_reward_scale)
    if args_cli.angular_velocity_reward_scale is not None:
        env_cfg.angular_velocity_reward_scale = float(args_cli.angular_velocity_reward_scale)
    if args_cli.flat_orientation_reward_scale is not None:
        env_cfg.flat_orientation_reward_scale = float(args_cli.flat_orientation_reward_scale)
    if args_cli.pitch_orientation_reward_scale is not None:
        env_cfg.pitch_orientation_reward_scale = float(
            args_cli.pitch_orientation_reward_scale
        )
    if args_cli.action_reward_scale is not None:
        env_cfg.action_reward_scale = float(args_cli.action_reward_scale)
    if args_cli.action_rate_reward_scale is not None:
        env_cfg.action_rate_reward_scale = float(args_cli.action_rate_reward_scale)
    if args_cli.disable_displacement_velocity_rewards:
        env_cfg.use_displacement_velocity_rewards = False
    if args_cli.lateral_drift_reward_scale is not None:
        env_cfg.lateral_drift_reward_scale = float(args_cli.lateral_drift_reward_scale)
    if args_cli.yaw_drift_reward_scale is not None:
        env_cfg.yaw_drift_reward_scale = float(args_cli.yaw_drift_reward_scale)
    if args_cli.command_yaw_reward_scale is not None:
        env_cfg.command_yaw_reward_scale = float(args_cli.command_yaw_reward_scale)
    if args_cli.gait_contact_reward_scale is not None:
        env_cfg.gait_contact_reward_scale = float(args_cli.gait_contact_reward_scale)
    if args_cli.stance_contact_reward_scale is not None:
        env_cfg.stance_contact_reward_scale = float(args_cli.stance_contact_reward_scale)
    if args_cli.swing_contact_penalty_scale is not None:
        env_cfg.swing_contact_penalty_scale = float(args_cli.swing_contact_penalty_scale)
    if args_cli.foot_clearance_reward_scale is not None:
        env_cfg.foot_clearance_reward_scale = float(args_cli.foot_clearance_reward_scale)
    if args_cli.foot_contact_reward_scale is not None:
        env_cfg.foot_contact_reward_scale = float(args_cli.foot_contact_reward_scale)
    if args_cli.foot_slip_reward_scale is not None:
        env_cfg.foot_slip_reward_scale = float(args_cli.foot_slip_reward_scale)
    if args_cli.air_time_variance_reward_scale is not None:
        env_cfg.air_time_variance_reward_scale = float(
            args_cli.air_time_variance_reward_scale
        )
    if args_cli.valid_foot_cycle_reward_scale is not None:
        env_cfg.valid_foot_cycle_reward_scale = float(
            args_cli.valid_foot_cycle_reward_scale
        )
    if args_cli.front_rear_support_reward_scale is not None:
        env_cfg.front_rear_support_reward_scale = float(
            args_cli.front_rear_support_reward_scale
        )
    if args_cli.axle_support_imbalance_penalty_scale is not None:
        env_cfg.axle_support_imbalance_penalty_scale = float(
            args_cli.axle_support_imbalance_penalty_scale
        )
    if args_cli.same_axle_airborne_penalty_scale is not None:
        env_cfg.same_axle_airborne_penalty_scale = float(
            args_cli.same_axle_airborne_penalty_scale
        )
    if args_cli.excess_airborne_penalty_scale is not None:
        env_cfg.excess_airborne_penalty_scale = float(
            args_cli.excess_airborne_penalty_scale
        )
    if args_cli.front_foot_backward_reach_penalty_scale is not None:
        env_cfg.front_foot_backward_reach_penalty_scale = float(
            args_cli.front_foot_backward_reach_penalty_scale
        )
    if args_cli.front_pair_backward_reach_penalty_scale is not None:
        env_cfg.front_pair_backward_reach_penalty_scale = float(
            args_cli.front_pair_backward_reach_penalty_scale
        )
    if args_cli.front_foot_min_body_x_m is not None:
        env_cfg.front_foot_min_body_x_m = float(
            args_cli.front_foot_min_body_x_m
        )
    if args_cli.front_foot_reach_normalization_m is not None:
        env_cfg.front_foot_reach_normalization_m = max(
            float(args_cli.front_foot_reach_normalization_m),
            1.0e-6,
        )
    if args_cli.front_foot_backward_termination_body_x_m is not None:
        env_cfg.front_foot_backward_termination_body_x_m = float(
            args_cli.front_foot_backward_termination_body_x_m
        )
    if args_cli.foot_cycle_min_air_time_s is not None:
        env_cfg.foot_cycle_min_air_time_s = max(
            float(args_cli.foot_cycle_min_air_time_s),
            0.0,
        )
    if args_cli.foot_cycle_target_air_time_s is not None:
        env_cfg.foot_cycle_target_air_time_s = max(
            float(args_cli.foot_cycle_target_air_time_s),
            env_cfg.foot_cycle_min_air_time_s,
            1.0e-6,
        )
    if args_cli.foot_cycle_min_clearance_m is not None:
        env_cfg.foot_cycle_min_clearance_m = max(
            float(args_cli.foot_cycle_min_clearance_m),
            0.0,
        )
    if args_cli.foot_cycle_min_body_relative_travel_m is not None:
        env_cfg.foot_cycle_min_body_relative_travel_m = max(
            float(args_cli.foot_cycle_min_body_relative_travel_m),
            0.0,
        )
    reference_candidate = None
    if args_cli.reference_gait_candidate:
        reference_candidate = apply_reference_gait_candidate(env_cfg, args_cli.reference_gait_candidate)
    if args_cli.include_reference_actions_in_observation:
        env_cfg.include_reference_actions_in_observation = True
        env_cfg.observation_space = CAD_LINKAGE_REFERENCE_OBSERVATION_DIM
    if args_cli.reference_action_tracking_reward_scale is not None:
        env_cfg.reference_action_tracking_reward_scale = float(args_cli.reference_action_tracking_reward_scale)
    if args_cli.reference_action_tracking_sigma is not None:
        env_cfg.reference_action_tracking_sigma = float(args_cli.reference_action_tracking_sigma)
    if args_cli.reference_action_mse_reward_scale is not None:
        env_cfg.reference_action_mse_reward_scale = float(args_cli.reference_action_mse_reward_scale)
    if args_cli.foot_collision_mode is not None:
        env_cfg.foot_contact_mode = str(args_cli.foot_collision_mode).replace("-", "_")
        env_cfg.use_actual_cad_foot_collision = args_cli.foot_collision_mode == "actual-cad-visual-bottom"

    agent_cfg = DominoCadLinkagePPORunnerCfg()
    agent_cfg.seed = int(args_cli.seed)
    agent_cfg.device = args_cli.device
    agent_cfg.num_steps_per_env = int(args_cli.num_steps_per_env)
    agent_cfg.max_iterations = int(args_cli.iterations)
    if args_cli.save_interval is not None:
        agent_cfg.save_interval = max(int(args_cli.save_interval), 1)
    if args_cli.init_noise_std is not None:
        agent_cfg.policy.init_noise_std = float(args_cli.init_noise_std)
    if args_cli.ppo_learning_rate is not None:
        agent_cfg.algorithm.learning_rate = float(args_cli.ppo_learning_rate)
    if args_cli.ppo_entropy_coef is not None:
        agent_cfg.algorithm.entropy_coef = float(args_cli.ppo_entropy_coef)

    log_root = Path(args_cli.log_root).expanduser().resolve()
    log_dir = log_root / agent_cfg.experiment_name / f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_{agent_cfg.run_name}"
    log_dir.mkdir(parents=True, exist_ok=True)
    env_cfg.log_dir = str(log_dir)

    env = DominoCadLinkageEnv(env_cfg)
    set_domino_inspection_camera(env)
    action_dim = gym.spaces.flatdim(env.single_action_space)
    observations, _ = env.reset()
    if action_dim != EXPECTED_ACTION_COUNT:
        raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} action dimensions, found {action_dim}.")
    if observations["policy"].shape[-1] != int(env_cfg.observation_space):
        raise RuntimeError(
            f"Expected {env_cfg.observation_space} observations, found {observations['policy'].shape[-1]}."
        )
    startup_foot_contact_alignment: dict[str, object] = {
        "enabled": False,
        "reason": "requires actual CAD visuals with actual-CAD visual-bottom contacts",
    }
    if (
        bool(env_cfg.include_actual_cad_visuals)
        and str(env._linkage.get("foot_contact_mode", "")) == "actual_cad_visual_bottom"
    ):
        startup_foot_contact_alignment = env.foot_contact_alignment_report()
        startup_foot_contact_alignment["enabled"] = True
        if not bool(startup_foot_contact_alignment["passed"]):
            raise RuntimeError(
                "Refusing to train with misaligned visible CAD feet and physics contacts: "
                f"{startup_foot_contact_alignment}"
            )

    wrapper_type = DominoVisibleRslRlVecEnvWrapper if VIEWABLE_RUN else RslRlVecEnvWrapper
    wrapped_env = wrapper_type(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(wrapped_env, agent_cfg.to_dict(), log_dir=str(log_dir), device=agent_cfg.device)
    resume_checkpoint = ""
    if args_cli.resume_checkpoint:
        resume_path = Path(args_cli.resume_checkpoint).expanduser().resolve()
        if not resume_path.exists():
            raise FileNotFoundError(f"Resume checkpoint does not exist: {resume_path}")
        runner.load(str(resume_path), load_optimizer=bool(args_cli.resume_load_optimizer), map_location=agent_cfg.device)
        resume_checkpoint = resume_path.name
    reference_identity_init: dict[str, object] = {"enabled": False}
    if args_cli.reference_action_identity_init:
        if resume_checkpoint:
            raise ValueError("Reference identity initialization cannot be combined with --resume-checkpoint.")
        if not bool(env_cfg.include_reference_actions_in_observation):
            raise ValueError("Reference identity initialization requires appended reference actions.")
        reference_identity_init = initialize_reference_action_identity_actor(runner.alg.policy)
    if args_cli.init_noise_std is not None:
        force_action_noise_std(runner.alg.policy, float(args_cli.init_noise_std))
    if bool(reference_identity_init.get("enabled")):
        identity_observations = wrapped_env.get_observations().to(runner.device)
        identity_diagnostics = reference_action_diagnostics(
            runner.alg.policy,
            identity_observations,
        )
        reference_identity_init["startup_diagnostics"] = identity_diagnostics
        print(
            "Reference-action identity diagnostics: "
            + json.dumps(identity_diagnostics, sort_keys=True),
            flush=True,
        )
        if float(identity_diagnostics["clipped_mse"]) > 1.0e-8:
            raise RuntimeError(
                "Reference-action identity warm start is not exact; refusing to enter PPO: "
                f"{identity_diagnostics}"
            )
    bc_action_weights = reference_action_bc_weights(
        float(args_cli.reference_action_bc_shoulder_weight),
        float(args_cli.reference_action_bc_lower_linkage_weight),
        float(args_cli.reference_action_bc_upper_pitch_weight),
        agent_cfg.device,
    )
    pre_bc_policy_validation: dict[str, object] | None = None
    if bool(reference_identity_init.get("enabled")) and int(args_cli.policy_validation_steps) > 0:
        # Validate while PhysX is still fresh. Teacher replay exercises all passive
        # constraints and can leave solver warm-start caches that rigid-body
        # teleport reset does not clear completely.
        pre_bc_policy_validation = validate_trained_policy_rollout(
            runner,
            wrapped_env,
            env,
            agent_cfg.device,
            int(args_cli.policy_validation_steps),
            args_cli,
        )
    bc_settle_summary = {"steps": 0, "finite": True, "done_count": 0, "terminated_count": 0, "timeout_count": 0}
    if int(args_cli.reference_action_bc_steps) > 0:
        bc_settle_summary = settle_behavior_clone_start(
            wrapped_env,
            env,
            int(args_cli.reference_action_bc_settle_steps),
        )
        if not bool(bc_settle_summary["finite"]) or int(bc_settle_summary["done_count"]) > 0:
            raise RuntimeError(f"Behavior-clone startup settling failed: {bc_settle_summary}")
    bc_summary = behavior_clone_reference_actions(
        runner,
        wrapped_env,
        env,
        int(args_cli.reference_action_bc_steps),
        float(args_cli.reference_action_bc_lr),
        float(args_cli.reference_action_bc_output_penalty),
        bc_action_weights,
        replay_steps=int(args_cli.reference_action_bc_replay_steps),
        replay_batch_size=int(args_cli.reference_action_bc_batch_size),
    )
    reference_identity_refresh: dict[str, object] = {"enabled": False}
    if bool(reference_identity_init.get("enabled")):
        reference_identity_refresh = initialize_reference_action_identity_actor(runner.alg.policy)
        bc_summary["post_bc_identity_refresh"] = reference_identity_refresh
    if int(args_cli.reference_action_bc_steps) > 0:
        torch.save(
            {
                "model_state_dict": runner.alg.policy.state_dict(),
                "optimizer_state_dict": runner.alg.optimizer.state_dict(),
                "iter": int(getattr(runner, "current_learning_iteration", 0)),
                "infos": {"reference_action_bc": bc_summary},
            },
            str(log_dir / "model_bc.pt"),
        )
    ppo_training_wall_seconds = 0.0
    if not args_cli.skip_ppo_after_bc:
        ppo_training_started_at = time.perf_counter()
        runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
        ppo_training_wall_seconds = time.perf_counter() - ppo_training_started_at

    checkpoints = [path.name for path in sorted(log_dir.glob("model*.pt"), key=checkpoint_index)]
    if not checkpoints:
        raise RuntimeError(f"Training completed but no RSL-RL checkpoints were written under {log_dir}.")

    trained_policy_validation: dict[str, object] = {"enabled": False}
    if pre_bc_policy_validation is not None:
        trained_policy_validation = pre_bc_policy_validation
        trained_policy_validation["timing"] = "fresh_physx_before_teacher_replay"
        trained_policy_validation["validated_policy_reapplied_after_bc"] = bool(
            reference_identity_refresh.get("enabled")
        )
    elif int(args_cli.policy_validation_steps) > 0:
        trained_policy_validation = validate_trained_policy_rollout(
            runner,
            wrapped_env,
            env,
            agent_cfg.device,
            int(args_cli.policy_validation_steps),
            args_cli,
        )
    policy_display_allowed = (
        bool(args_cli.disable_policy_display_gate)
        or not bool(trained_policy_validation.get("enabled", False))
        or bool(trained_policy_validation.get("passed", False))
    )
    indefinite_policy_hold_requested = (
        bool(args_cli.hold_open)
        and VIEWABLE_RUN
        and str(args_cli.hold_open_mode) == "policy"
        and int(args_cli.hold_open_exit_after_frames) == 0
        and not bool(args_cli.allow_indefinite_policy_hold_open)
    )

    hold_open_report: dict[str, object] = {
        "enabled": bool(args_cli.hold_open and VIEWABLE_RUN),
        "mode": str(args_cli.hold_open_mode),
        "state_before_display": body_reference_report(env),
        "policy_display_allowed": bool(policy_display_allowed),
    }
    if args_cli.hold_open and VIEWABLE_RUN and str(args_cli.hold_open_mode) == "policy" and not policy_display_allowed:
        hold_open_report = prepare_hold_open_view(
            env,
            "reset",
            int(args_cli.hold_open_render_frames),
        )
        hold_open_report["mode"] = "policy_rejected_reset"
        hold_open_report["requested_mode"] = "policy"
        hold_open_report["policy_display_allowed"] = False
        hold_open_report["policy_gate_failures"] = list(trained_policy_validation.get("failures", []))
    elif args_cli.hold_open and VIEWABLE_RUN and str(args_cli.hold_open_mode) == "policy" and indefinite_policy_hold_requested:
        hold_open_report = prepare_hold_open_view(
            env,
            "final",
            int(args_cli.hold_open_render_frames),
        )
        hold_open_report["mode"] = "policy_validated_final_hold"
        hold_open_report["requested_mode"] = "policy"
        hold_open_report["policy_display_allowed"] = True
        hold_open_report["policy_live_rollout_allowed"] = False
        hold_open_report["reason"] = (
            "The trained policy passed the bounded validation rollout, but indefinite live policy display is disabled "
            "by default because longer visible rollouts can still drift into disfigured poses."
        )
    elif args_cli.hold_open and VIEWABLE_RUN and str(args_cli.hold_open_mode) == "policy":
        hold_open_report = {
            "enabled": True,
            "mode": "policy",
            "timeline": "live_trained_policy_rollout",
            "state_before_display": body_reference_report(env),
            "render_frames": max(int(args_cli.hold_open_render_frames), 0),
            "policy_display_allowed": True,
            "policy_live_rollout_allowed": True,
        }
        refresh_visible_view(env, int(args_cli.hold_open_render_frames))
    elif args_cli.hold_open and VIEWABLE_RUN:
        hold_open_report = prepare_hold_open_view(
            env,
            str(args_cli.hold_open_mode),
            int(args_cli.hold_open_render_frames),
        )
        hold_open_report["policy_display_allowed"] = bool(policy_display_allowed)

    report_status = "passed"
    validation_failed = (
        bool(trained_policy_validation.get("enabled", False))
        and not bool(trained_policy_validation.get("passed", False))
    )
    if validation_failed and (not args_cli.hold_open or str(args_cli.hold_open_mode) == "policy"):
        report_status = "policy_validation_failed"
    elif validation_failed:
        report_status = "passed_with_policy_validation_warnings"

    total_timesteps = env.num_envs * agent_cfg.max_iterations * agent_cfg.num_steps_per_env
    trained_timesteps = 0 if args_cli.skip_ppo_after_bc else total_timesteps
    report = {
        "status": report_status,
        "geometry": env._linkage["geometry"],
        "closure_model": env._linkage.get("closure_model"),
        "visual_fidelity": env._linkage.get("visual_fidelity"),
        "actual_cad_visual": env._linkage.get("actual_cad_visual"),
        "cad_source": env._linkage.get("cad_source"),
        "actual_cad_visuals": env._linkage.get("actual_cad_visuals"),
        "visual_geometry_counts": env._linkage.get("visual_geometry_counts"),
        "actual_cad_visual_alignment": env._linkage.get("actual_cad_visual_alignment"),
        "startup_foot_contact_alignment": startup_foot_contact_alignment,
        "actual_cad_foot_collision": env._linkage.get("actual_cad_foot_collision"),
        "body_ground_collisions": env._linkage.get("body_ground_collisions"),
        "non_foot_ground_contact": env.non_foot_ground_contact_report(),
        "last_done_diagnostics": getattr(env, "_last_done_diagnostics", []),
        "actuator_model": env._linkage.get("actuator_model"),
        "passive_stabilizers": env._linkage.get("passive_stabilizers", []),
        "joint_separation": env.joint_separation_report(),
        "resolved_floating_height_m": env._linkage.get("resolved_floating_height_m"),
        "terrain": getattr(env, "_terrain_report", None),
        "num_envs": env.num_envs,
        "iterations": agent_cfg.max_iterations,
        "ppo_skipped_after_bc": bool(args_cli.skip_ppo_after_bc),
        "num_steps_per_env": agent_cfg.num_steps_per_env,
        "total_timesteps": total_timesteps,
        "runtime": {
            "policy_device": str(agent_cfg.device),
            "physics_device": str(env_cfg.sim.device),
            "simulation_dt_s": float(env_cfg.sim.dt),
            "control_decimation": int(env_cfg.decimation),
            "use_fabric": bool(env_cfg.sim.use_fabric),
            "replicate_physics": bool(env_cfg.scene.replicate_physics),
        },
        "performance": {
            "ppo_training_wall_seconds": round(float(ppo_training_wall_seconds), 6),
            "ppo_trained_timesteps": int(trained_timesteps),
            "ppo_samples_per_second": (
                round(float(trained_timesteps) / ppo_training_wall_seconds, 6)
                if ppo_training_wall_seconds > 0.0
                else 0.0
            ),
            "main_wall_seconds_before_report": round(time.perf_counter() - main_started_at, 6),
        },
        "action_dim": action_dim,
        "action_names": ACTION_JOINT_NAMES,
        "action_group_counts": action_group_counts(),
        "per_leg_action_layout": per_leg_action_layout(),
        "leg_start_stance": env.leg_start_stance_report(),
        "observation_dim": observations["policy"].shape[-1],
        "command": {
            "x_m_s": float(args_cli.command_x_m_s),
            "y_m_s": float(args_cli.command_y_m_s),
            "yaw_rad_s": float(args_cli.command_yaw_rad_s),
            "gait_frequency_hz": float(args_cli.gait_frequency_hz),
        },
        "training_config": {
            "init_noise_std": float(agent_cfg.policy.init_noise_std),
            "ppo_learning_rate": float(agent_cfg.algorithm.learning_rate),
            "ppo_entropy_coef": float(agent_cfg.algorithm.entropy_coef),
            "action_scale_deg": float(env_cfg.action_scale_deg),
            "servo_target_rate_limit_deg_s": float(env_cfg.servo_target_rate_limit_deg_s),
            "reset_settle_steps": int(env_cfg.reset_settle_steps),
            "visible_step_delay_s": float(env_cfg.visible_step_delay_s),
            "episode_length_s": float(env_cfg.episode_length_s),
            "min_height_m": float(env_cfg.min_height_m),
            "max_tilt_deg": float(env_cfg.max_tilt_deg),
            "save_interval": int(agent_cfg.save_interval),
            "target_height_m": float(env_cfg.target_height_m),
            "vertical_velocity_reward_scale": float(env_cfg.vertical_velocity_reward_scale),
            "angular_velocity_reward_scale": float(env_cfg.angular_velocity_reward_scale),
            "flat_orientation_reward_scale": float(env_cfg.flat_orientation_reward_scale),
            "pitch_orientation_reward_scale": float(
                env_cfg.pitch_orientation_reward_scale
            ),
            "command_progress_reward_scale": float(env_cfg.command_progress_reward_scale),
            "command_velocity_reward_scale": float(env_cfg.command_velocity_reward_scale),
            "command_velocity_tracking_reward_scale": float(env_cfg.command_velocity_tracking_reward_scale),
            "command_velocity_tracking_sigma": float(env_cfg.command_velocity_tracking_sigma),
            "command_stagnation_penalty_scale": float(env_cfg.command_stagnation_penalty_scale),
            "command_stagnation_speed_m_s": float(env_cfg.command_stagnation_speed_m_s),
            "use_displacement_velocity_rewards": bool(env_cfg.use_displacement_velocity_rewards),
            "lateral_drift_reward_scale": float(env_cfg.lateral_drift_reward_scale),
            "yaw_drift_reward_scale": float(env_cfg.yaw_drift_reward_scale),
            "command_yaw_reward_scale": float(env_cfg.command_yaw_reward_scale),
            "gait_contact_reward_scale": float(env_cfg.gait_contact_reward_scale),
            "stance_contact_reward_scale": float(env_cfg.stance_contact_reward_scale),
            "swing_contact_penalty_scale": float(env_cfg.swing_contact_penalty_scale),
            "foot_clearance_reward_scale": float(env_cfg.foot_clearance_reward_scale),
            "foot_contact_reward_scale": float(env_cfg.foot_contact_reward_scale),
            "foot_slip_reward_scale": float(env_cfg.foot_slip_reward_scale),
            "air_time_variance_reward_scale": float(
                env_cfg.air_time_variance_reward_scale
            ),
            "valid_foot_cycle_reward_scale": float(
                env_cfg.valid_foot_cycle_reward_scale
            ),
            "front_rear_support_reward_scale": float(
                env_cfg.front_rear_support_reward_scale
            ),
            "axle_support_imbalance_penalty_scale": float(
                env_cfg.axle_support_imbalance_penalty_scale
            ),
            "same_axle_airborne_penalty_scale": float(
                env_cfg.same_axle_airborne_penalty_scale
            ),
            "excess_airborne_penalty_scale": float(
                env_cfg.excess_airborne_penalty_scale
            ),
            "front_foot_backward_reach_penalty_scale": float(
                env_cfg.front_foot_backward_reach_penalty_scale
            ),
            "front_pair_backward_reach_penalty_scale": float(
                env_cfg.front_pair_backward_reach_penalty_scale
            ),
            "front_foot_min_body_x_m": float(
                env_cfg.front_foot_min_body_x_m
            ),
            "front_foot_reach_normalization_m": float(
                env_cfg.front_foot_reach_normalization_m
            ),
            "terminate_on_front_foot_backward_reach": bool(
                env_cfg.terminate_on_front_foot_backward_reach
            ),
            "front_foot_backward_termination_body_x_m": float(
                env_cfg.front_foot_backward_termination_body_x_m
            ),
            "foot_cycle_min_air_time_s": float(
                env_cfg.foot_cycle_min_air_time_s
            ),
            "foot_cycle_target_air_time_s": float(
                env_cfg.foot_cycle_target_air_time_s
            ),
            "foot_cycle_min_clearance_m": float(
                env_cfg.foot_cycle_min_clearance_m
            ),
            "foot_cycle_min_body_relative_travel_m": float(
                env_cfg.foot_cycle_min_body_relative_travel_m
            ),
            "reference_action_tracking_reward_scale": float(env_cfg.reference_action_tracking_reward_scale),
            "reference_action_tracking_sigma": float(env_cfg.reference_action_tracking_sigma),
            "reference_action_mse_reward_scale": float(env_cfg.reference_action_mse_reward_scale),
            "include_reference_actions_in_observation": bool(env_cfg.include_reference_actions_in_observation),
            "include_actual_cad_visuals": bool(env_cfg.include_actual_cad_visuals),
            "actual_cad_mesh_dir_override": str(env_cfg.actual_cad_mesh_dir),
            "use_actual_cad_foot_collision": bool(env_cfg.use_actual_cad_foot_collision),
            "foot_contact_mode": str(env._linkage.get("foot_contact_mode", env_cfg.foot_contact_mode)),
            "use_actual_cad_visual_foot_bottom_for_rewards": bool(env_cfg.use_actual_cad_visual_foot_bottom_for_rewards),
            "align_actual_cad_visual_bottom_to_ground": bool(env_cfg.align_actual_cad_visual_bottom_to_ground),
            "actual_cad_ground_clearance_m": float(env_cfg.actual_cad_ground_clearance_m),
            "enable_body_collisions": bool(env_cfg.enable_body_collisions),
            "terminate_on_non_foot_ground_contact": bool(env_cfg.terminate_on_non_foot_ground_contact),
            "non_foot_ground_contact_margin_m": float(env_cfg.non_foot_ground_contact_margin_m),
            "terrain_type": str(env_cfg.terrain_type),
            "stairs_step_count": int(env_cfg.stairs_step_count),
            "stairs_step_depth_m": float(env_cfg.stairs_step_depth_m),
            "stairs_step_height_m": float(env_cfg.stairs_step_height_m),
            "stairs_width_m": float(env_cfg.stairs_width_m),
            "stairs_start_x_m": float(env_cfg.stairs_start_x_m),
            "stairs_top_platform_length_m": float(env_cfg.stairs_top_platform_length_m),
            "reference_action_bc_weights": bc_summary.get("action_weights", {}),
            "reference_action_identity_init": reference_identity_init,
        },
        "reference_gait_candidate": reference_candidate,
        "resume_checkpoint": resume_checkpoint,
        "resume_load_optimizer": bool(args_cli.resume_load_optimizer),
        "effective_action_noise_std": round(action_noise_std_mean(runner.alg.policy), 6),
        "reference_action_bc": bc_summary,
        "reference_action_bc_startup_settle": bc_settle_summary,
        "last_reward_terms": env.last_reward_terms_report(),
        "trained_policy_validation": trained_policy_validation,
        "hold_open_display": hold_open_report,
        "checkpoints": checkpoints,
        "latest_checkpoint": checkpoints[-1],
        "log_dir_name": log_dir.name,
    }

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2), flush=True)
    if args_cli.hold_open and VIEWABLE_RUN:
        # Reapply the inspection camera after Isaac Lab's viewer extension has
        # finished installing its default camera state.
        set_domino_inspection_camera(env)
        hold_open_exit_after_frames = max(int(args_cli.hold_open_exit_after_frames), 0)
        if str(args_cli.hold_open_mode) == "policy" and policy_display_allowed and not indefinite_policy_hold_requested:
            print(
                "Training complete; continuing the trained policy rollout in the visible Isaac window. "
                "Close the Isaac window to exit.",
                flush=True,
            )
            continue_visible_trained_policy_loop(
                runner,
                wrapped_env,
                env,
                agent_cfg.device,
                hold_open_exit_after_frames,
            )
        else:
            if str(args_cli.hold_open_mode) == "policy" and not policy_display_allowed:
                print(
                    "Training complete, but the trained policy failed the rollout-quality gate; "
                    "holding the reset pose instead of continuing the bad policy live.",
                    flush=True,
                )
            elif str(args_cli.hold_open_mode) == "policy" and indefinite_policy_hold_requested:
                print(
                    "Training complete and the policy passed the bounded validation rollout; "
                    "holding the validated final pose instead of continuing an indefinite live rollout.",
                    flush=True,
                )
            print(
                f"Training complete; keeping Isaac Sim open in hold-open mode '{hold_open_report.get('mode', args_cli.hold_open_mode)}'. "
                "Close the Isaac window to exit.",
                flush=True,
            )
            hold_open_frame_count = 0
            while simulation_app.is_running():
                simulation_app.update()
                hold_open_frame_count += 1
                if hold_open_exit_after_frames and hold_open_frame_count >= hold_open_exit_after_frames:
                    break

    wrapped_env.close()


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except Exception:
        exit_code = 1
        traceback.print_exc()
    finally:
        if args_cli.graceful_close:
            simulation_app.close(wait_for_replicator=False, skip_cleanup=True)
        os._exit(exit_code)
