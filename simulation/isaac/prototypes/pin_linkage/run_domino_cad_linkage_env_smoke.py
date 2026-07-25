"""Smoke-run the Domino CAD-linkage DirectRLEnv."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time
import traceback

from isaaclab.app import AppLauncher

CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))


parser = argparse.ArgumentParser(description="Run a Domino CAD-linkage DirectRLEnv smoke test.")
parser.add_argument("--steps", type=int, default=120, help="Number of environment steps to run.")
parser.add_argument("--num-envs", type=int, default=1, help="Number of manually authored CAD-linkage environments.")
parser.add_argument("--seed", type=int, default=42, help="Environment seed.")
parser.add_argument("--action-amplitude", type=float, default=0.5, help="Normalized action amplitude.")
parser.add_argument("--action-scale-deg", type=float, default=None, help="Maximum drive target offset, in degrees, for action=1.")
parser.add_argument("--episode-length-s", type=float, default=None, help="Episode length override.")
parser.add_argument("--command-x-m-s", type=float, default=0.0, help="Forward velocity command.")
parser.add_argument("--command-y-m-s", type=float, default=0.0, help="Lateral velocity command.")
parser.add_argument("--command-yaw-rad-s", type=float, default=0.0, help="Yaw-rate command.")
parser.add_argument("--gait-frequency-hz", type=float, default=1.0, help="Gait phase frequency in observations.")
parser.add_argument("--command-progress-reward-scale", type=float, default=None, help="Forward/lateral progress reward scale.")
parser.add_argument("--command-velocity-tracking-reward-scale", type=float, default=None, help="Positive command velocity tracking reward scale.")
parser.add_argument("--gait-contact-reward-scale", type=float, default=None, help="Alternating stance/swing contact reward scale.")
parser.add_argument("--stance-contact-reward-scale", type=float, default=None, help="Reward scale for commanded stance feet staying in contact.")
parser.add_argument("--swing-contact-penalty-scale", type=float, default=None, help="Penalty scale for commanded swing feet staying in contact.")
parser.add_argument("--foot-clearance-reward-scale", type=float, default=None, help="Swing foot clearance reward scale.")
parser.add_argument("--foot-contact-reward-scale", type=float, default=None, help="All-foot contact reward scale.")
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
parser.add_argument("--capture-viewport-path", default="", help="Optional PNG path for the final visible viewport.")
parser.add_argument("--visible-step-delay-s", type=float, default=0.0, help="Wall-clock delay after each visible step.")
parser.add_argument("--hold-open-seconds", type=float, default=0.0, help="Keep the final visible pose open for inspection.")
parser.add_argument(
    "--closure-model",
    choices=["direct", "passive"],
    default="passive",
    help="Loop-closure topology. Passive restores the CAD closure bodies; direct is retained only for comparison.",
)
parser.add_argument(
    "--graceful-close",
    action="store_true",
    help="Call SimulationApp.close() before exit. Disabled by default because it can hang on some Windows setups.",
)
parser.add_argument(
    "--allow-proxy-visuals",
    action="store_true",
    help="Allow visible rendering of the CAD-derived cube/sphere proxy for physics debugging.",
)
parser.add_argument(
    "--allow-multi-env-viewport",
    action="store_true",
    help="Allow a visible viewport with more than one cloned environment.",
)
parser.add_argument(
    "--disable-actual-cad-visuals",
    action="store_true",
    help="Render only the CAD-derived cube/sphere proxy instead of the exported Domino STL link meshes.",
)
parser.add_argument("--actual-cad-mesh-dir", default="", help="Optional override for the Domino STL mesh folder.")
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
        "Visible Domino DirectRLEnv runs should use the actual exported CAD STL visuals. "
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
import torch  # noqa: E402

from domino_action_contract import ACTION_JOINT_NAMES, EXPECTED_ACTION_COUNT, action_group_counts, per_leg_action_layout  # noqa: E402
from domino_cad_linkage_env import CAD_LINKAGE_OBSERVATION_DIM, DominoCadLinkageEnv, DominoCadLinkageEnvCfg  # noqa: E402


def tensor_list(value: torch.Tensor) -> list[float]:
    return [round(float(v), 6) for v in value.detach().cpu().flatten()]


def set_domino_inspection_camera(env: DominoCadLinkageEnv) -> None:
    if not VIEWABLE_RUN:
        return
    from isaacsim.core.utils.viewports import set_camera_view

    body_position, _, _, _ = env._body_reference_state(0)
    focus = body_position.astype(float)
    set_camera_view(
        eye=tuple((focus + [0.72, -0.62, 0.28]).tolist()),
        target=tuple((focus + [0.0, 0.0, -0.04]).tolist()),
    )


def capture_visible_viewport(output_path: str, max_wait_frames: int = 180) -> str:
    if not output_path:
        return ""
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    path = Path(output_path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active Isaac viewport is available for capture.")
    capture_viewport_to_file(viewport, file_path=str(path))
    for _ in range(max(int(max_wait_frames), 1)):
        simulation_app.update()
        if path.exists() and path.stat().st_size > 0:
            return str(path)
    raise RuntimeError(f"Isaac viewport capture did not complete: {path}")


def main() -> None:
    cfg = DominoCadLinkageEnvCfg()
    cfg.scene.num_envs = int(args_cli.num_envs)
    cfg.sim.device = args_cli.device
    cfg.seed = int(args_cli.seed)
    cfg.include_actual_cad_visuals = not bool(args_cli.disable_actual_cad_visuals)
    cfg.actual_cad_mesh_dir = str(args_cli.actual_cad_mesh_dir)
    cfg.closure_model = str(args_cli.closure_model)
    cfg.command_x_m_s = float(args_cli.command_x_m_s)
    cfg.command_y_m_s = float(args_cli.command_y_m_s)
    cfg.command_yaw_rad_s = float(args_cli.command_yaw_rad_s)
    cfg.gait_frequency_hz = float(args_cli.gait_frequency_hz)
    if args_cli.action_scale_deg is not None:
        cfg.action_scale_deg = float(args_cli.action_scale_deg)
    if args_cli.episode_length_s is not None:
        cfg.episode_length_s = float(args_cli.episode_length_s)
    if args_cli.command_progress_reward_scale is not None:
        cfg.command_progress_reward_scale = float(args_cli.command_progress_reward_scale)
    if args_cli.command_velocity_tracking_reward_scale is not None:
        cfg.command_velocity_tracking_reward_scale = float(args_cli.command_velocity_tracking_reward_scale)
    if args_cli.gait_contact_reward_scale is not None:
        cfg.gait_contact_reward_scale = float(args_cli.gait_contact_reward_scale)
    if args_cli.stance_contact_reward_scale is not None:
        cfg.stance_contact_reward_scale = float(args_cli.stance_contact_reward_scale)
    if args_cli.swing_contact_penalty_scale is not None:
        cfg.swing_contact_penalty_scale = float(args_cli.swing_contact_penalty_scale)
    if args_cli.foot_clearance_reward_scale is not None:
        cfg.foot_clearance_reward_scale = float(args_cli.foot_clearance_reward_scale)
    if args_cli.foot_contact_reward_scale is not None:
        cfg.foot_contact_reward_scale = float(args_cli.foot_contact_reward_scale)

    env = DominoCadLinkageEnv(cfg)
    observations, _ = env.reset()
    set_domino_inspection_camera(env)
    action_dim = gym.spaces.flatdim(env.single_action_space)
    if action_dim != EXPECTED_ACTION_COUNT:
        raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} action dimensions, found {action_dim}.")
    if observations["policy"].shape[-1] != CAD_LINKAGE_OBSERVATION_DIM:
        raise RuntimeError(
            f"Expected {CAD_LINKAGE_OBSERVATION_DIM} observations, found {observations['policy'].shape[-1]}."
        )

    action_phase = torch.arange(action_dim, dtype=torch.float32, device=env.device).reshape(1, -1) * 0.41
    env_phase = torch.arange(env.num_envs, dtype=torch.float32, device=env.device).reshape(-1, 1) * 0.23
    max_abs_action_by_channel = torch.zeros(action_dim, dtype=torch.float32, device=env.device)
    total_reward = torch.zeros(env.num_envs, device=env.device)
    terminated_count = 0
    truncated_count = 0
    min_body_height = float("inf")
    max_body_speed = 0.0
    foot_contact_sum = 0.0
    min_foot_height = float("inf")
    max_foot_height = float("-inf")
    max_foot_clearance = 0.0

    for step in range(int(args_cli.steps)):
        actions = float(args_cli.action_amplitude) * torch.sin((0.17 * float(step)) + action_phase + env_phase)
        observations, rewards, terminated, truncated, _ = env.step(actions)
        total_reward += rewards
        terminated_count += int(torch.count_nonzero(terminated).detach().cpu())
        truncated_count += int(torch.count_nonzero(truncated).detach().cpu())
        max_abs_action_by_channel = torch.maximum(max_abs_action_by_channel, torch.max(torch.abs(actions), dim=0)[0])
        for env_index in range(env.num_envs):
            position, _, linear_velocity, _ = env._body_reference_state(env_index)
            min_body_height = min(min_body_height, float(position[2] - env._env_origins_np[env_index][2]))
            max_body_speed = max(max_body_speed, float(torch.linalg.norm(torch.tensor(linear_velocity))))
            foot_positions, foot_radii = env._reward_foot_positions_and_radii(env_index)
            foot_contacts = env._foot_contact_flags_np(foot_positions, env_index, radii_m=foot_radii)
            foot_contact_sum += float(sum(foot_contacts))
            min_foot_height = min(min_foot_height, float(foot_positions[:, 2].min()))
            max_foot_height = max(max_foot_height, float(foot_positions[:, 2].max()))
            clearance = env._foot_ground_clearance_np(foot_positions, env_index, radii_m=foot_radii)
            max_foot_clearance = max(max_foot_clearance, float(clearance.max()))
        if not torch.isfinite(observations["policy"]).all() or not torch.isfinite(rewards).all():
            raise RuntimeError("Non-finite observation or reward.")
        if VIEWABLE_RUN and float(args_cli.visible_step_delay_s) > 0.0:
            time.sleep(float(args_cli.visible_step_delay_s))

    inactive_channels = [
        index
        for index, max_abs_action in enumerate(max_abs_action_by_channel.detach().cpu().tolist())
        if float(max_abs_action) <= 1e-6
    ]
    if inactive_channels:
        raise RuntimeError(f"Smoke action did not exercise all 12 channels; inactive indices: {inactive_channels}.")

    final_states = [env._body_reference_state(env_index) for env_index in range(env.num_envs)]
    report = {
        "status": "passed",
        "steps": int(args_cli.steps),
        "num_envs": env.num_envs,
        "geometry": env._linkage["geometry"],
        "closure_model": env._linkage.get("closure_model"),
        "visual_fidelity": env._linkage.get("visual_fidelity"),
        "actual_cad_visual": env._linkage.get("actual_cad_visual"),
        "cad_source": env._linkage.get("cad_source"),
        "actual_cad_visuals": env._linkage.get("actual_cad_visuals"),
        "visual_geometry_counts": env._linkage.get("visual_geometry_counts"),
        "joint_separation": env.joint_separation_report(),
        "action_dim": action_dim,
        "action_names": ACTION_JOINT_NAMES,
        "action_group_counts": action_group_counts(),
        "per_leg_action_layout": per_leg_action_layout(),
        "observation_dim": observations["policy"].shape[-1],
        "command": {
            "x_m_s": float(args_cli.command_x_m_s),
            "y_m_s": float(args_cli.command_y_m_s),
            "yaw_rad_s": float(args_cli.command_yaw_rad_s),
            "gait_frequency_hz": float(args_cli.gait_frequency_hz),
        },
        "training_config": {
            "action_scale_deg": float(cfg.action_scale_deg),
            "episode_length_s": float(cfg.episode_length_s),
            "command_progress_reward_scale": float(cfg.command_progress_reward_scale),
            "command_velocity_tracking_reward_scale": float(cfg.command_velocity_tracking_reward_scale),
            "gait_contact_reward_scale": float(cfg.gait_contact_reward_scale),
            "stance_contact_reward_scale": float(cfg.stance_contact_reward_scale),
            "swing_contact_penalty_scale": float(cfg.swing_contact_penalty_scale),
            "foot_clearance_reward_scale": float(cfg.foot_clearance_reward_scale),
            "foot_contact_reward_scale": float(cfg.foot_contact_reward_scale),
            "use_actual_cad_foot_collision": bool(cfg.use_actual_cad_foot_collision),
            "align_actual_cad_visual_bottom_to_ground": bool(cfg.align_actual_cad_visual_bottom_to_ground),
            "actual_cad_ground_clearance_m": float(cfg.actual_cad_ground_clearance_m),
        },
        "mean_reward": round(float(torch.mean(total_reward / max(int(args_cli.steps), 1)).detach().cpu()), 6),
        "terminated_count": terminated_count,
        "truncated_count": truncated_count,
        "min_body_reference_height_m": round(min_body_height, 6),
        "max_body_reference_speed_m_s": round(max_body_speed, 6),
        "mean_foot_contacts_per_env": round(foot_contact_sum / max(int(args_cli.steps) * env.num_envs, 1), 6),
        "min_foot_proxy_height_m": round(min_foot_height, 6),
        "max_foot_proxy_height_m": round(max_foot_height, 6),
        "max_foot_proxy_clearance_m": round(max_foot_clearance, 6),
        "max_abs_action_by_channel": tensor_list(max_abs_action_by_channel),
        "final_body_reference_position_m": [
            [round(float(value), 6) for value in state[0]] for state in final_states
        ],
        "final_body_reference_orientation_wxyz": [
            [round(float(value), 6) for value in state[1]] for state in final_states
        ],
        "final_body_reference_linear_velocity_m_s": [
            [round(float(value), 6) for value in state[2]] for state in final_states
        ],
        "final_body_reference_angular_velocity_rad_s": [
            [round(float(value), 6) for value in state[3]] for state in final_states
        ],
    }

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    captured_viewport = capture_visible_viewport(args_cli.capture_viewport_path)
    if captured_viewport:
        print(f"Captured Isaac viewport: {captured_viewport}", flush=True)
    if VIEWABLE_RUN and float(args_cli.hold_open_seconds) > 0.0:
        deadline = time.monotonic() + float(args_cli.hold_open_seconds)
        while time.monotonic() < deadline:
            simulation_app.update()
            time.sleep(1.0 / 60.0)

    print(json.dumps(report, indent=2), flush=True)
    env.close()


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
