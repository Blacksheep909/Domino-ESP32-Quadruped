"""Load and step an RSL-RL checkpoint in the Domino CAD-linkage env."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

import numpy as np

from isaaclab.app import AppLauncher

CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))


parser = argparse.ArgumentParser(description="Evaluate a Domino CAD-linkage RSL-RL PPO checkpoint.")
parser.add_argument(
    "--policy-mode",
    choices=["checkpoint", "zero", "reference", "fixed"],
    default="checkpoint",
    help="Action source for playback: learned checkpoint, zero-action hold, scripted reference, or fixed action row.",
)
parser.add_argument("--checkpoint", default="", help="Checkpoint path. If omitted, the newest CAD-linkage model_*.pt is used.")
parser.add_argument("--log-root", default="simulation/isaac/out/domino_rsl_rl", help="Root searched for checkpoints.")
parser.add_argument("--num-envs", type=int, default=4, help="Number of manually authored CAD-linkage environments.")
parser.add_argument("--steps", type=int, default=120, help="Evaluation environment steps.")
parser.add_argument(
    "--startup-zero-steps",
    type=int,
    default=0,
    help="Step zero actions before playback metrics/policy rollout so the visible robot starts from the base pose.",
)
parser.add_argument(
    "--startup-actions",
    default="",
    help="Optional comma-separated 12-channel normalized action row held during startup instead of zero actions.",
)
parser.add_argument(
    "--policy-ramp-steps",
    type=int,
    default=60,
    help="Playback steps used to blend from the settled neutral action to checkpoint output.",
)
parser.add_argument(
    "--fixed-actions",
    default="",
    help="Optional comma-separated 12-channel normalized action row used when --policy-mode fixed. Defaults to startup actions.",
)
parser.add_argument(
    "--align-rendered-visual-min-foot-after-startup",
    action="store_true",
    help="After startup settling, globally shift rendered CAD visuals so the lowest visible foot bottom reaches terrain clearance.",
)
parser.add_argument("--seed", type=int, default=42, help="Evaluation seed.")
parser.add_argument("--action-scale-deg", type=float, default=None, help="Maximum drive target offset, in degrees, for action=1.")
parser.add_argument(
    "--servo-target-rate-limit-deg-s",
    type=float,
    default=None,
    help="Maximum servo target slew rate in degrees/second. Use 0 to disable the slew limiter.",
)
parser.add_argument("--episode-length-s", type=float, default=None, help="Episode length override.")
parser.add_argument(
    "--reference-action-snap-tolerance",
    type=float,
    default=0.0,
    help="Execute the exact appended reference target when every policy channel is within this tolerance.",
)
parser.add_argument("--floating-height-m", type=float, default=None, help="Initial floating body-reference height override.")
parser.add_argument("--min-height-m", type=float, default=None, help="Minimum body-reference height before reset.")
parser.add_argument("--max-tilt-deg", type=float, default=None, help="Maximum body-reference tilt before reset.")
parser.add_argument("--command-x-m-s", type=float, default=0.0, help="Forward velocity command.")
parser.add_argument("--command-y-m-s", type=float, default=0.0, help="Lateral velocity command.")
parser.add_argument("--command-yaw-rad-s", type=float, default=0.0, help="Yaw-rate command.")
parser.add_argument("--gait-frequency-hz", type=float, default=1.0, help="Gait phase frequency in observations.")
parser.add_argument("--command-progress-reward-scale", type=float, default=None, help="Forward/lateral progress reward scale.")
parser.add_argument("--command-velocity-reward-scale", type=float, default=None, help="Penalty scale for squared command x/y velocity error.")
parser.add_argument("--command-velocity-tracking-reward-scale", type=float, default=None, help="Positive command velocity tracking reward scale.")
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
parser.add_argument("--reference-gait-candidate", default="", help="Optional scripted gait JSON for playback diagnostics.")
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
    help="Sigma for the optional reference-action tracking diagnostic.",
)
parser.add_argument(
    "--reference-action-mse-reward-scale",
    type=float,
    default=None,
    help="Reward scale applied to mean squared error against the optional scripted 12-actuator reference action.",
)
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
parser.add_argument(
    "--capture-viewport-path",
    default="",
    help="Optional PNG path captured from the visible viewport after playback.",
)
parser.add_argument(
    "--graceful-close",
    action="store_true",
    help="Call SimulationApp.close() before exit. Disabled by default because it can hang on some Windows setups.",
)
parser.add_argument("--hold-open", action="store_true", help="Keep a visible Isaac Sim playback window open after the run.")
parser.add_argument(
    "--hold-open-render-frames",
    type=int,
    default=4,
    help="Viewport refresh frames before entering visible playback hold-open.",
)
parser.add_argument(
    "--hold-open-exit-after-frames",
    type=int,
    default=0,
    help="Testing hook: exit hold-open after this many app update frames. Zero keeps the window open until closed.",
)
parser.add_argument(
    "--visible-start-delay-s",
    type=float,
    default=0.0,
    help="Wall-clock pause before a visible rollout begins so the viewport can be inspected.",
)
parser.add_argument(
    "--visible-step-delay-s",
    type=float,
    default=0.0,
    help="Wall-clock delay after each visible policy step for deliberate linkage inspection.",
)
parser.add_argument(
    "--min-each-linkage-drive-motion-deg",
    type=float,
    default=0.0,
    help="Optional minimum actual lower/upper driver rotation relative to its hip carriage. Zero disables this gate.",
)
parser.add_argument(
    "--min-each-foot-motion-m",
    type=float,
    default=0.0,
    help="Optional minimum rendered CAD foot motion in each foot's own hip-carriage frame. Zero disables this gate.",
)
parser.add_argument(
    "--max-joint-separation-m",
    type=float,
    default=0.0,
    help="Optional maximum loop-pin separation observed during playback. Zero disables this gate.",
)
parser.add_argument(
    "--allow-proxy-visuals",
    action="store_true",
    help="Allow visible rendering of the CAD-derived cube/sphere proxy for physics debugging.",
)
parser.add_argument(
    "--allow-multi-env-viewport",
    action="store_true",
    help="Allow a visible viewport with more than one cloned playback environment.",
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
parser.add_argument("--fixed-base", action="store_true", help="Hold the chassis fixed for isolated mechanism playback.")
parser.add_argument("--disable-gravity", action="store_true", help="Disable gravity for isolated mechanism playback.")
parser.add_argument("--actual-cad-mesh-dir", default="", help="Optional override for the Domino STL mesh folder.")
parser.add_argument("--terrain-type", choices=["flat", "stairs"], default="flat", help="Static terrain scene used for playback.")
parser.add_argument("--stairs-step-count", type=int, default=None, help="Number of stair treads when --terrain-type=stairs.")
parser.add_argument("--stairs-step-depth-m", type=float, default=None, help="Step tread depth for stair playback.")
parser.add_argument("--stairs-step-height-m", type=float, default=None, help="Step rise height for stair playback.")
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


def reference_segment_at_step(candidate: dict[str, object] | None, raw_step: int) -> str:
    if not candidate or str(candidate.get("type", "")).lower() != "keyframe_sequence":
        return ""
    segments = candidate.get("segments", [])
    total_steps = int(candidate.get("total_steps", 0))
    if total_steps <= 0 or not isinstance(segments, list):
        return ""
    step = int(raw_step) % total_steps if bool(candidate.get("loop", True)) else max(0, min(int(raw_step), total_steps - 1))
    for segment in segments:
        segment_steps = int(segment.get("steps", 0))
        if step < segment_steps:
            return str(segment.get("name", ""))
        step -= segment_steps
    return ""


VIEWABLE_RUN = is_human_viewable_run(args_cli)

if VIEWABLE_RUN and args_cli.disable_actual_cad_visuals and not args_cli.allow_proxy_visuals:
    raise SystemExit(
        "Visible Domino DirectRLEnv playback should use the actual exported CAD STL visuals. "
        "Remove --disable-actual-cad-visuals, run headless, or pass --allow-proxy-visuals when deliberately "
        "debugging the simplified cube/sphere proxy."
    )

if VIEWABLE_RUN and int(args_cli.num_envs) != 1 and not args_cli.allow_multi_env_viewport:
    raise SystemExit(
        "Visible Domino CAD inspection runs should show one robot. "
        "Use --num-envs 1 for visual checks, or pass --allow-multi-env-viewport when deliberately viewing cloned "
        "playback environments."
    )

os.environ.setdefault("WARP_CACHE_PATH", str((Path.cwd() / "simulation" / "isaac" / "out" / "warp_cache").resolve()))

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

from domino_action_contract import (  # noqa: E402
    ACTION_JOINT_NAMES,
    CAD_ACTION_ROLES,
    EXPECTED_ACTION_COUNT,
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
from domino_linkage_motion import (  # noqa: E402
    create_foot_endpoint_motion_tracker,
    create_linkage_motion_tracker,
    foot_endpoint_motion_report,
    linkage_motion_report,
    update_foot_endpoint_motion_tracker,
    update_linkage_motion_tracker,
)
from domino_reference_gait import REFERENCE_GAIT_PARAMETER_NAMES, is_keyframe_sequence, load_reference_candidate  # noqa: E402


def resolve_checkpoint() -> Path:
    if args_cli.checkpoint:
        checkpoint_path = Path(args_cli.checkpoint).expanduser().resolve()
        if not checkpoint_path.exists():
            raise FileNotFoundError(f"Checkpoint path does not exist: {checkpoint_path}")
        return checkpoint_path

    log_root = Path(args_cli.log_root).expanduser().resolve()
    experiment_root = log_root / DominoCadLinkagePPORunnerCfg().experiment_name
    candidates = sorted(experiment_root.glob("**/model_*.pt"), key=lambda path: path.stat().st_mtime, reverse=True)
    if not candidates:
        raise FileNotFoundError(f"No CAD-linkage model_*.pt checkpoint found under {experiment_root}.")
    return candidates[0]


def apply_reference_gait_candidate(env_cfg: DominoCadLinkageEnvCfg, candidate_path: str) -> dict[str, object]:
    candidate = load_reference_candidate(candidate_path)
    env_cfg.reference_gait_name = str(candidate["name"])
    if is_keyframe_sequence(candidate):
        env_cfg.reference_sequence_json = json.dumps(candidate, separators=(",", ":"))
    else:
        for key in REFERENCE_GAIT_PARAMETER_NAMES:
            setattr(env_cfg, f"reference_gait_{key}", float(candidate[key]))
    return candidate


def accumulate_reward_terms(accumulator: dict[str, object], report: dict[str, object]) -> None:
    accumulator["count"] = int(accumulator["count"]) + 1
    accumulator["total_mean"] = float(accumulator["total_mean"]) + float(report.get("total_mean", 0.0))
    for group_name in ("unscaled_mean", "scaled_mean"):
        group = report.get(group_name, {})
        if not isinstance(group, dict):
            continue
        target = accumulator[group_name]
        if not isinstance(target, dict):
            raise TypeError(f"Expected reward accumulator group {group_name} to be a dict.")
        for name, value in group.items():
            target[name] = float(target.get(name, 0.0)) + float(value)


def averaged_reward_terms(accumulator: dict[str, object]) -> dict[str, object]:
    count = max(int(accumulator["count"]), 1)
    scaled_mean = {
        name: round(float(value) / count, 6)
        for name, value in sorted(accumulator["scaled_mean"].items())
    }
    dominant_scaled = [
        {"name": name, "mean": value}
        for name, value in sorted(scaled_mean.items(), key=lambda item: abs(item[1]), reverse=True)
        if abs(value) > 1e-6
    ]
    return {
        "steps": int(accumulator["count"]),
        "unscaled_mean": {
            name: round(float(value) / count, 6)
            for name, value in sorted(accumulator["unscaled_mean"].items())
        },
        "scaled_mean": scaled_mean,
        "dominant_scaled_mean": dominant_scaled[:8],
        "total_mean": round(float(accumulator["total_mean"]) / count, 6),
    }


def parse_action_row(raw_value: str, expected_count: int, label: str) -> list[float]:
    raw = str(raw_value or "").strip()
    if not raw:
        return [0.0 for _ in range(int(expected_count))]
    values = [float(item.strip()) for item in raw.replace(";", ",").split(",") if item.strip()]
    if len(values) != int(expected_count):
        raise ValueError(f"{label} expected {expected_count} comma-separated actions, received {len(values)}.")
    return [max(-1.0, min(1.0, float(value))) for value in values]


def visual_foot_height_report(
    foot_positions_by_env: list[np.ndarray],
    foot_tracks_by_env: list[list[dict[str, object]]],
    warning_spread_m: float = 0.025,
) -> dict[str, object]:
    """Summarize visible CAD foot-bottom height spread at the measured start pose."""
    env_rows = []
    max_spread = 0.0
    warnings = []
    for env_index, positions in enumerate(foot_positions_by_env):
        positions_np = np.asarray(positions, dtype=np.float64)
        names = [
            str(track.get("name", foot_index))
            for foot_index, track in enumerate(foot_tracks_by_env[env_index])
        ]
        if positions_np.size == 0:
            env_rows.append(
                {
                    "env_index": int(env_index),
                    "foot_count": 0,
                    "foot_z_m": [],
                    "min_z_m": None,
                    "max_z_m": None,
                    "height_spread_m": 0.0,
                }
            )
            continue
        z_values = positions_np[:, 2]
        min_z = float(np.min(z_values))
        max_z = float(np.max(z_values))
        spread = max_z - min_z
        max_spread = max(max_spread, spread)
        env_row = {
            "env_index": int(env_index),
            "foot_count": int(len(z_values)),
            "foot_z_m": [
                {"name": name, "z_m": round(float(z), 6)}
                for name, z in zip(names, z_values)
            ],
            "min_z_m": round(min_z, 6),
            "max_z_m": round(max_z, 6),
            "height_spread_m": round(spread, 6),
        }
        if spread > float(warning_spread_m):
            warnings.append(
                f"env_{env_index} visible CAD foot-bottom height spread is {spread:.6f} m; stance is not visually level"
            )
        env_rows.append(env_row)
    return {
        "warning_threshold_m": round(float(warning_spread_m), 6),
        "max_height_spread_m": round(float(max_spread), 6),
        "level_enough_for_visual_base_pose": bool(max_spread <= float(warning_spread_m)),
        "warnings": warnings,
        "envs": env_rows,
    }


def visual_support_offset_report(
    support_positions_by_env: list[np.ndarray],
    visual_positions_by_env: list[np.ndarray],
    support_tracks_by_env: list[list[dict[str, object]]],
    visual_tracks_by_env: list[list[dict[str, object]]],
) -> dict[str, object]:
    env_rows = []
    max_xy_offset = 0.0
    max_z_offset = 0.0
    max_norm = 0.0
    for env_index, (support_positions, visual_positions) in enumerate(
        zip(support_positions_by_env, visual_positions_by_env)
    ):
        support_np = np.asarray(support_positions, dtype=np.float64)
        visual_np = np.asarray(visual_positions, dtype=np.float64)
        count = min(len(support_np), len(visual_np))
        rows = []
        for foot_index in range(count):
            support_name = str(support_tracks_by_env[env_index][foot_index].get("name", foot_index))
            visual_name = str(visual_tracks_by_env[env_index][foot_index].get("name", foot_index))
            offset = visual_np[foot_index] - support_np[foot_index]
            xy_offset = float(np.linalg.norm(offset[:2]))
            z_offset = float(offset[2])
            norm = float(np.linalg.norm(offset))
            max_xy_offset = max(max_xy_offset, xy_offset)
            max_z_offset = max(max_z_offset, abs(z_offset))
            max_norm = max(max_norm, norm)
            rows.append(
                {
                    "index": int(foot_index),
                    "support_name": support_name,
                    "visual_name": visual_name,
                    "support_position_m": [round(float(value), 6) for value in support_np[foot_index]],
                    "visual_position_m": [round(float(value), 6) for value in visual_np[foot_index]],
                    "visual_minus_support_m": [round(float(value), 6) for value in offset],
                    "xy_offset_m": round(xy_offset, 6),
                    "z_offset_m": round(z_offset, 6),
                    "norm_m": round(norm, 6),
                }
            )
        env_rows.append({"env_index": int(env_index), "feet": rows})
    return {
        "max_xy_offset_m": round(float(max_xy_offset), 6),
        "max_abs_z_offset_m": round(float(max_z_offset), 6),
        "max_norm_m": round(float(max_norm), 6),
        "envs": env_rows,
    }


def align_rendered_visual_min_foot_to_terrain_clearance(env: DominoCadLinkageEnv) -> dict[str, object]:
    env_reports = []
    max_abs_delta = 0.0
    for env_index in range(env.num_envs):
        positions = env._actual_cad_visual_foot_positions(env_index, rendered=True)
        if positions.shape[0] == 0:
            env_reports.append({"env_index": int(env_index), "status": "skipped_no_visual_feet"})
            continue
        terrain_heights = env._terrain_heights_np(positions, env_index=env_index)
        target_z = terrain_heights + float(env.cfg.actual_cad_ground_clearance_m)
        clearances = positions[:, 2] - target_z
        min_clearance = float(np.min(clearances))
        delta = -min_clearance
        linkage = env._linkages[env_index]
        old_lift = float(linkage.get("actual_cad_visual_lift_m", 0.0) or 0.0)
        new_lift = old_lift + delta
        linkage["actual_cad_visual_lift_m"] = float(new_lift)
        max_abs_delta = max(max_abs_delta, abs(float(delta)))
        env_reports.append(
            {
                "env_index": int(env_index),
                "old_visual_lift_m": round(old_lift, 6),
                "new_visual_lift_m": round(float(new_lift), 6),
                "delta_m": round(float(delta), 6),
                "min_clearance_before_m": round(min_clearance, 6),
                "target": "lowest_rendered_cad_foot_bottom_to_terrain_clearance",
            }
        )
    env._apply_actual_cad_visual_lift()
    return {
        "enabled": True,
        "max_abs_delta_m": round(float(max_abs_delta), 6),
        "envs": env_reports,
    }


def reference_action_error_report(
    sample_count: int,
    executed_action_sum: torch.Tensor,
    reference_action_sum: torch.Tensor,
    error_sum: torch.Tensor,
    abs_error_sum: torch.Tensor,
    sq_error_sum: torch.Tensor,
    max_abs_error: torch.Tensor,
) -> dict[str, object]:
    if sample_count <= 0:
        return {"sample_count": 0, "per_channel": [], "by_role": []}

    count = float(sample_count)
    mean_executed = executed_action_sum.detach().cpu() / count
    mean_reference = reference_action_sum.detach().cpu() / count
    mean_error = error_sum.detach().cpu() / count
    mean_abs_error = abs_error_sum.detach().cpu() / count
    rmse = torch.sqrt(torch.clamp(sq_error_sum.detach().cpu() / count, min=0.0))
    max_abs = max_abs_error.detach().cpu()

    per_channel = []
    by_role_values: dict[str, list[dict[str, float]]] = {}
    for index, name in enumerate(ACTION_JOINT_NAMES):
        role = CAD_ACTION_ROLES.get(name, "unknown")
        row = {
            "index": index,
            "name": name,
            "role": role,
            "mean_executed_action": round(float(mean_executed[index]), 6),
            "mean_reference_action": round(float(mean_reference[index]), 6),
            "mean_signed_error": round(float(mean_error[index]), 6),
            "mean_abs_error": round(float(mean_abs_error[index]), 6),
            "rmse": round(float(rmse[index]), 6),
            "max_abs_error": round(float(max_abs[index]), 6),
        }
        per_channel.append(row)
        by_role_values.setdefault(role, []).append(row)

    by_role = []
    for role, rows in sorted(by_role_values.items()):
        by_role.append(
            {
                "role": role,
                "channel_count": len(rows),
                "mean_abs_error": round(sum(row["mean_abs_error"] for row in rows) / max(len(rows), 1), 6),
                "rmse": round(sum(row["rmse"] for row in rows) / max(len(rows), 1), 6),
                "max_abs_error": round(max(row["max_abs_error"] for row in rows), 6),
            }
        )
    return {"sample_count": int(sample_count), "per_channel": per_channel, "by_role": by_role}


def pause_omniverse_timeline() -> str:
    try:
        import omni.timeline  # noqa: PLC0415

        omni.timeline.get_timeline_interface().pause()
        return "paused"
    except Exception as exc:  # pragma: no cover - defensive against Kit version differences.
        return f"pause_failed: {exc}"


def refresh_visible_view(env: DominoCadLinkageEnv, frames: int) -> None:
    for _ in range(max(int(frames), 0)):
        try:
            env.render(recompute=True)
        except Exception:
            env.sim.render()
        simulation_app.update()


def capture_visible_viewport(output_path: str, max_wait_frames: int = 120) -> str:
    if not VIEWABLE_RUN or not str(output_path).strip():
        return ""
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: PLC0415

    path = Path(output_path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        path.unlink()
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active Isaac viewport is available for capture.")
    capture_viewport_to_file(viewport, file_path=str(path))
    for _ in range(max(int(max_wait_frames), 1)):
        simulation_app.update()
        if path.exists() and path.stat().st_size > 0:
            return str(path)
    raise RuntimeError(f"Isaac viewport capture did not complete: {path}")


def set_domino_inspection_camera(body_reference_height_m: float) -> None:
    if not VIEWABLE_RUN:
        return
    try:
        from isaacsim.core.utils.viewports import set_camera_view  # noqa: PLC0415

        body_height = float(body_reference_height_m)
        set_camera_view(
            eye=(0.85, -0.70, max(0.38, body_height + 0.22)),
            target=(0.08, 0.05, max(0.08, body_height - 0.14)),
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


def select_policy_actions(
    policy_mode: str,
    policy,
    observations,
    env: DominoCadLinkageEnv,
    action_dim: int,
    device: str,
    fixed_action_values: list[float] | None = None,
):
    if policy_mode == "zero":
        return torch.zeros((env.num_envs, action_dim), dtype=torch.float32, device=device)
    if policy_mode == "reference":
        return torch.tensor(env._reference_actions_np(), dtype=torch.float32, device=device)
    if policy_mode == "fixed":
        if fixed_action_values is None:
            raise RuntimeError("Fixed policy mode requires fixed action values.")
        return torch.tensor(
            fixed_action_values,
            dtype=torch.float32,
            device=device,
        ).reshape(1, action_dim).repeat(env.num_envs, 1)
    if policy is None:
        raise RuntimeError("Checkpoint policy was not initialized.")
    return policy(observations)


def continue_visible_policy_loop(
    wrapped_env: RslRlVecEnvWrapper,
    env: DominoCadLinkageEnv,
    policy,
    observations,
    action_dim: int,
    device: str,
    hold_open_exit_after_frames: int,
    fixed_action_values: list[float] | None = None,
) -> None:
    hold_open_frame_count = 0
    obs = observations
    with torch.inference_mode():
        while simulation_app.is_running():
            actions = select_policy_actions(
                str(args_cli.policy_mode),
                policy,
                obs,
                env,
                action_dim,
                device,
                fixed_action_values=fixed_action_values,
            )
            obs, rewards, _, _ = wrapped_env.step(actions.to(wrapped_env.device))
            if (
                not torch.isfinite(actions).all()
                or not torch.isfinite(policy_observation_tensor(obs)).all()
                or not torch.isfinite(rewards).all()
            ):
                raise RuntimeError("Non-finite action, observation, or reward during visible policy playback.")
            obs = move_observations_to_device(obs, device)
            try:
                env.render(recompute=False)
            except Exception:
                env.sim.render()
            simulation_app.update()
            hold_open_frame_count += 1
            if hold_open_exit_after_frames and hold_open_frame_count >= hold_open_exit_after_frames:
                break


def main() -> None:
    checkpoint_path = resolve_checkpoint() if args_cli.policy_mode == "checkpoint" else None

    env_cfg = DominoCadLinkageEnvCfg()
    env_cfg.scene.num_envs = int(args_cli.num_envs)
    env_cfg.sim.device = args_cli.device
    env_cfg.seed = int(args_cli.seed)
    if VIEWABLE_RUN:
        env_cfg.viewer.eye = (0.90, -0.75, 0.42)
        env_cfg.viewer.lookat = (0.10, 0.06, 0.16)
        env_cfg.viewer.origin_type = "world"
    env_cfg.include_actual_cad_visuals = not bool(args_cli.disable_actual_cad_visuals)
    env_cfg.actual_cad_mesh_dir = str(args_cli.actual_cad_mesh_dir)
    env_cfg.closure_model = str(args_cli.closure_model)
    env_cfg.fixed_base = bool(args_cli.fixed_base)
    env_cfg.enable_gravity = not bool(args_cli.disable_gravity)
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
    if args_cli.episode_length_s is not None:
        env_cfg.episode_length_s = float(args_cli.episode_length_s)
    if args_cli.floating_height_m is not None:
        env_cfg.floating_height_m = float(args_cli.floating_height_m)
    if args_cli.min_height_m is not None:
        env_cfg.min_height_m = float(args_cli.min_height_m)
    if args_cli.max_tilt_deg is not None:
        env_cfg.max_tilt_deg = float(args_cli.max_tilt_deg)
    if args_cli.command_progress_reward_scale is not None:
        env_cfg.command_progress_reward_scale = float(args_cli.command_progress_reward_scale)
    if args_cli.command_velocity_reward_scale is not None:
        env_cfg.command_velocity_reward_scale = float(args_cli.command_velocity_reward_scale)
    if args_cli.command_velocity_tracking_reward_scale is not None:
        env_cfg.command_velocity_tracking_reward_scale = float(args_cli.command_velocity_tracking_reward_scale)
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

    env = DominoCadLinkageEnv(env_cfg)
    set_domino_inspection_camera(float(env._linkage.get("target_height_m", env_cfg.target_height_m)))
    action_dim = gym.spaces.flatdim(env.single_action_space)
    observations, _ = env.reset()
    if action_dim != EXPECTED_ACTION_COUNT:
        raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} action dimensions, found {action_dim}.")
    if observations["policy"].shape[-1] != int(env_cfg.observation_space):
        raise RuntimeError(
            f"Expected {env_cfg.observation_space} observations, found {observations['policy'].shape[-1]}."
        )

    wrapped_env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    policy = None
    if args_cli.policy_mode == "checkpoint":
        if checkpoint_path is None:
            raise RuntimeError("Checkpoint policy mode requires a checkpoint path.")
        runner = OnPolicyRunner(wrapped_env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        runner.load(str(checkpoint_path), load_optimizer=False, map_location=agent_cfg.device)
        policy = runner.get_inference_policy(device=agent_cfg.device)

    obs = wrapped_env.get_observations().to(agent_cfg.device)
    startup_done_count = 0
    startup_terminated_count = 0
    startup_timeout_count = 0
    startup_zero_steps = max(int(args_cli.startup_zero_steps), 0)
    startup_action_values = parse_action_row(args_cli.startup_actions, action_dim, "--startup-actions")
    startup_action_source = "custom" if str(args_cli.startup_actions or "").strip() else "zero"
    fixed_action_raw = args_cli.fixed_actions if str(args_cli.fixed_actions or "").strip() else args_cli.startup_actions
    fixed_action_values = parse_action_row(fixed_action_raw, action_dim, "--fixed-actions")
    post_startup_visual_alignment = {"enabled": False}
    if startup_zero_steps:
        startup_actions = torch.tensor(
            startup_action_values,
            dtype=torch.float32,
            device=agent_cfg.device,
        ).reshape(1, action_dim).repeat(wrapped_env.num_envs, 1)
        with torch.inference_mode():
            for _ in range(startup_zero_steps):
                obs, rewards, dones, _ = wrapped_env.step(startup_actions.to(wrapped_env.device))
                startup_done_count += int(torch.count_nonzero(dones).detach().cpu())
                startup_terminated_count += int(torch.count_nonzero(env.reset_terminated).detach().cpu())
                startup_timeout_count += int(torch.count_nonzero(env.reset_time_outs).detach().cpu())
                if (
                    not torch.isfinite(rewards).all()
                    or not torch.isfinite(policy_observation_tensor(obs)).all()
                ):
                    raise RuntimeError("Non-finite observation or reward during startup zero-action hold.")
                obs = move_observations_to_device(obs, agent_cfg.device)
                if VIEWABLE_RUN:
                    try:
                        env.render(recompute=False)
                    except Exception:
                        env.sim.render()
                    simulation_app.update()
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
        # The appended reference action depends on episode_length_buf. Refresh
        # after rewinding the counter so checkpoint mode does not execute one
        # stale step-120 target at rollout step zero.
        obs = move_observations_to_device(wrapped_env.get_observations(), agent_cfg.device)
    if bool(args_cli.align_rendered_visual_min_foot_after_startup):
        post_startup_visual_alignment = align_rendered_visual_min_foot_to_terrain_clearance(env)

    total_reward = torch.zeros(wrapped_env.num_envs, device=wrapped_env.device)
    done_count = 0
    terminated_count = 0
    timeout_count = 0
    termination_events: list[dict[str, object]] = []
    per_env_done_count = torch.zeros(wrapped_env.num_envs, dtype=torch.int64, device=wrapped_env.device)
    per_env_terminated_count = torch.zeros(wrapped_env.num_envs, dtype=torch.int64, device=wrapped_env.device)
    per_env_timeout_count = torch.zeros(wrapped_env.num_envs, dtype=torch.int64, device=wrapped_env.device)
    min_body_height = float("inf")
    max_body_tilt_deg = 0.0
    max_joint_separation_during_rollout_m = 0.0
    worst_joint_separation_during_rollout = None
    worst_joint_separation_by_name: dict[str, dict[str, object]] = {}
    min_body_height_by_env = torch.full((wrapped_env.num_envs,), float("inf"), dtype=torch.float32, device=wrapped_env.device)
    max_body_tilt_deg_by_env = torch.zeros(wrapped_env.num_envs, dtype=torch.float32, device=wrapped_env.device)
    max_action_abs = 0.0
    max_executed_action_abs = 0.0
    max_executed_action_abs_by_channel = torch.zeros(action_dim, dtype=torch.float32, device=wrapped_env.device)
    foot_contact_sum = 0.0
    gait_contact_match_sum = 0.0
    stance_contact_sum = 0.0
    swing_contact_sum = 0.0
    swing_clearance_sum = 0.0
    swing_sample_count = 0
    linear_velocity_sum = torch.zeros((wrapped_env.num_envs, 3), dtype=torch.float32, device=wrapped_env.device)
    displacement_velocity_sum = torch.zeros((wrapped_env.num_envs, 3), dtype=torch.float32, device=wrapped_env.device)
    command_velocity_error_sum = torch.zeros(wrapped_env.num_envs, dtype=torch.float32, device=wrapped_env.device)
    displacement_command_velocity_error_sum = torch.zeros(wrapped_env.num_envs, dtype=torch.float32, device=wrapped_env.device)
    yaw_rate_error_sum = torch.zeros(wrapped_env.num_envs, dtype=torch.float32, device=wrapped_env.device)
    yaw_heading_drift_sum = torch.zeros(wrapped_env.num_envs, dtype=torch.float32, device=wrapped_env.device)
    reference_action_tracking_sum = torch.zeros(wrapped_env.num_envs, dtype=torch.float32, device=wrapped_env.device)
    reference_action_mse_sum = torch.zeros(wrapped_env.num_envs, dtype=torch.float32, device=wrapped_env.device)
    reference_action_sample_count = 0
    reference_executed_action_sum = torch.zeros(action_dim, dtype=torch.float32, device=wrapped_env.device)
    reference_action_sum = torch.zeros(action_dim, dtype=torch.float32, device=wrapped_env.device)
    reference_action_error_sum = torch.zeros(action_dim, dtype=torch.float32, device=wrapped_env.device)
    reference_action_abs_error_sum = torch.zeros(action_dim, dtype=torch.float32, device=wrapped_env.device)
    reference_action_sq_error_sum = torch.zeros(action_dim, dtype=torch.float32, device=wrapped_env.device)
    reference_action_max_abs_error = torch.zeros(action_dim, dtype=torch.float32, device=wrapped_env.device)
    reward_term_accumulator: dict[str, object] = {
        "count": 0,
        "unscaled_mean": {},
        "scaled_mean": {},
        "total_mean": 0.0,
    }
    final_displacements = None
    final_linear_velocities = None
    final_yaw_heading_drift_rad = None
    previous_positions = [
        env._body_reference_state(env_index)[0].copy()
        for env_index in range(env.num_envs)
    ]
    initial_foot_positions = [env._foot_positions(env_index).copy() for env_index in range(env.num_envs)]
    min_foot_positions = [positions.copy() for positions in initial_foot_positions]
    max_foot_positions = [positions.copy() for positions in initial_foot_positions]
    max_foot_displacement = [
        np.zeros(positions.shape[0], dtype=np.float32)
        for positions in initial_foot_positions
    ]
    final_foot_positions = [positions.copy() for positions in initial_foot_positions]
    initial_visual_foot_positions_unlifted = [
        env._actual_cad_visual_foot_positions(env_index, rendered=False).copy()
        for env_index in range(env.num_envs)
    ]
    initial_visual_foot_positions_rendered = [
        env._actual_cad_visual_foot_positions(env_index, rendered=True).copy()
        for env_index in range(env.num_envs)
    ]
    min_visual_foot_positions_rendered = [
        positions.copy() for positions in initial_visual_foot_positions_rendered
    ]
    max_visual_foot_positions_rendered = [
        positions.copy() for positions in initial_visual_foot_positions_rendered
    ]
    max_visual_foot_displacement_rendered = [
        np.zeros(positions.shape[0], dtype=np.float32)
        for positions in initial_visual_foot_positions_rendered
    ]
    final_visual_foot_positions_rendered = [
        positions.copy() for positions in initial_visual_foot_positions_rendered
    ]
    initial_body_relative_visual_foot_positions = [
        body_relative_actual_cad_visual_feet(env, env_index).copy()
        for env_index in range(env.num_envs)
    ]
    max_body_relative_visual_foot_displacement = [
        np.zeros(positions.shape[0], dtype=np.float32)
        for positions in initial_body_relative_visual_foot_positions
    ]
    foot_endpoint_motion_tracker = create_foot_endpoint_motion_tracker(env)
    linkage_motion_tracker = create_linkage_motion_tracker(env)
    policy_ramp_steps = max(int(args_cli.policy_ramp_steps), 0)
    startup_leg_stance = env.leg_start_stance_report()
    startup_front_rear_kinematic_symmetry = env.front_rear_kinematic_symmetry_report()
    startup_body_state = [
        {
            "env_index": int(env_index),
            "position_m": [round(float(value), 6) for value in env._body_reference_state(env_index)[0]],
            "tilt_deg": round(
                math.degrees(
                    math.acos(
                        max(
                            -1.0,
                            min(
                                1.0,
                                -float(projected_gravity_from_quat(env._body_reference_state(env_index)[1])[2]),
                            ),
                        )
                    )
                ),
                6,
            ),
        }
        for env_index in range(env.num_envs)
    ]

    if VIEWABLE_RUN and float(args_cli.visible_start_delay_s) > 0.0:
        time.sleep(float(args_cli.visible_start_delay_s))

    with torch.inference_mode():
        for rollout_step in range(int(args_cli.steps)):
            if args_cli.policy_mode == "zero":
                actions = torch.zeros((wrapped_env.num_envs, action_dim), dtype=torch.float32, device=agent_cfg.device)
            elif args_cli.policy_mode == "reference":
                actions = torch.tensor(env._reference_actions_np(), dtype=torch.float32, device=agent_cfg.device)
            elif args_cli.policy_mode == "fixed":
                actions = torch.tensor(
                    fixed_action_values,
                    dtype=torch.float32,
                    device=agent_cfg.device,
                ).reshape(1, action_dim).repeat(wrapped_env.num_envs, 1)
            else:
                if policy is None:
                    raise RuntimeError("Checkpoint policy was not initialized.")
                actions = policy(obs)
                if float(args_cli.reference_action_snap_tolerance) > 0.0:
                    reference_from_obs = policy_observation_tensor(obs)[:, -EXPECTED_ACTION_COUNT:]
                    close_to_reference = torch.max(
                        torch.abs(actions - reference_from_obs), dim=1
                    ).values <= float(args_cli.reference_action_snap_tolerance)
                    actions = torch.where(
                        close_to_reference.unsqueeze(1),
                        reference_from_obs,
                        actions,
                    )
                if policy_ramp_steps > 0:
                    blend = min(1.0, float(rollout_step + 1) / float(policy_ramp_steps))
                    actions = actions * blend
            max_action_abs = max(max_action_abs, float(torch.max(torch.abs(actions)).detach().cpu()))
            executed_actions = torch.clamp(actions, -1.0, 1.0)
            max_executed_action_abs = max(max_executed_action_abs, float(torch.max(torch.abs(executed_actions)).detach().cpu()))
            max_executed_action_abs_by_channel = torch.maximum(
                max_executed_action_abs_by_channel,
                torch.max(torch.abs(executed_actions), dim=0)[0],
            )
            if reference_candidate is not None:
                reference_actions = torch.tensor(
                    env._reference_actions_np(),
                    dtype=torch.float32,
                    device=actions.device,
                )
                reference_delta = executed_actions - reference_actions
                reference_error = torch.mean(torch.square(reference_delta), dim=1)
                sigma_action = max(float(env.cfg.reference_action_tracking_sigma), 1e-6)
                reference_action_tracking_sum += torch.exp(-reference_error / (sigma_action * sigma_action))
                reference_action_mse_sum += reference_error
                abs_delta = torch.abs(reference_delta)
                reference_action_sample_count += int(reference_delta.shape[0])
                reference_executed_action_sum += torch.sum(executed_actions, dim=0)
                reference_action_sum += torch.sum(reference_actions, dim=0)
                reference_action_error_sum += torch.sum(reference_delta, dim=0)
                reference_action_abs_error_sum += torch.sum(abs_delta, dim=0)
                reference_action_sq_error_sum += torch.sum(torch.square(reference_delta), dim=0)
                reference_action_max_abs_error = torch.maximum(reference_action_max_abs_error, torch.max(abs_delta, dim=0)[0])
            pre_step_episode_lengths = env.episode_length_buf.detach().cpu().tolist()
            obs, rewards, dones, _ = wrapped_env.step(actions.to(wrapped_env.device))
            if VIEWABLE_RUN and float(args_cli.visible_step_delay_s) > 0.0:
                time.sleep(float(args_cli.visible_step_delay_s))
            total_reward += rewards
            accumulate_reward_terms(reward_term_accumulator, env.last_reward_terms_report())
            done_count += int(torch.count_nonzero(dones).detach().cpu())
            terminated_count += int(torch.count_nonzero(env.reset_terminated).detach().cpu())
            timeout_count += int(torch.count_nonzero(env.reset_time_outs).detach().cpu())
            per_env_done_count += dones.to(dtype=torch.int64)
            per_env_terminated_count += env.reset_terminated.to(dtype=torch.int64)
            per_env_timeout_count += env.reset_time_outs.to(dtype=torch.int64)
            for env_index in torch.nonzero(dones, as_tuple=False).flatten().detach().cpu().tolist():
                reference_step = int(pre_step_episode_lengths[env_index])
                diagnostic_rows = getattr(env, "_last_done_diagnostics", [])
                diagnostic = diagnostic_rows[env_index] if env_index < len(diagnostic_rows) else {}
                termination_events.append(
                    {
                        "rollout_step": int(rollout_step),
                        "env_index": int(env_index),
                        "reference_step": reference_step,
                        "reference_segment": reference_segment_at_step(reference_candidate, reference_step),
                        "terminated": bool(env.reset_terminated[env_index].detach().cpu()),
                        "timed_out": bool(env.reset_time_outs[env_index].detach().cpu()),
                        **diagnostic,
                    }
                )
            for env_index in range(env.num_envs):
                position, orientation, linear_velocity, angular_velocity = env._body_reference_state(env_index)
                displacement_velocity = (position - previous_positions[env_index]) / max(float(env.step_dt), 1e-6)
                previous_positions[env_index] = position.copy()
                body_height = float(position[2] - env._env_origins_np[env_index][2])
                min_body_height = min(min_body_height, body_height)
                min_body_height_by_env[env_index] = min(float(min_body_height_by_env[env_index].detach().cpu()), body_height)
                projected_gravity = projected_gravity_from_quat(orientation)
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(projected_gravity[2])))))
                max_body_tilt_deg = max(max_body_tilt_deg, tilt)
                max_body_tilt_deg_by_env[env_index] = max(float(max_body_tilt_deg_by_env[env_index].detach().cpu()), tilt)
                reference_step = int(pre_step_episode_lengths[env_index])
                reference_segment = reference_segment_at_step(reference_candidate, reference_step)
                for separation_row in env._joint_separation_rows(env_index):
                    separation_m = float(separation_row["separation_m"])
                    event = {
                        **separation_row,
                        "separation_m": round(separation_m, 9),
                        "rollout_step": int(rollout_step),
                        "reference_step": reference_step,
                        "reference_segment": reference_segment,
                    }
                    if separation_m > max_joint_separation_during_rollout_m:
                        max_joint_separation_during_rollout_m = separation_m
                        worst_joint_separation_during_rollout = event
                    joint_name = str(separation_row["name"])
                    previous_worst = worst_joint_separation_by_name.get(joint_name)
                    if previous_worst is None or separation_m > float(previous_worst["separation_m"]):
                        worst_joint_separation_by_name[joint_name] = event
                foot_positions = env._foot_positions(env_index)
                final_foot_positions[env_index] = foot_positions.copy()
                min_foot_positions[env_index] = np.minimum(min_foot_positions[env_index], foot_positions)
                max_foot_positions[env_index] = np.maximum(max_foot_positions[env_index], foot_positions)
                max_foot_displacement[env_index] = np.maximum(
                    max_foot_displacement[env_index],
                    np.linalg.norm(foot_positions - initial_foot_positions[env_index], axis=1).astype(np.float32),
                )
                visual_foot_positions = env._actual_cad_visual_foot_positions(env_index, rendered=True)
                final_visual_foot_positions_rendered[env_index] = visual_foot_positions.copy()
                if visual_foot_positions.shape[0] > 0:
                    min_visual_foot_positions_rendered[env_index] = np.minimum(
                        min_visual_foot_positions_rendered[env_index],
                        visual_foot_positions,
                    )
                    max_visual_foot_positions_rendered[env_index] = np.maximum(
                        max_visual_foot_positions_rendered[env_index],
                        visual_foot_positions,
                    )
                    max_visual_foot_displacement_rendered[env_index] = np.maximum(
                        max_visual_foot_displacement_rendered[env_index],
                        np.linalg.norm(
                            visual_foot_positions - initial_visual_foot_positions_rendered[env_index],
                            axis=1,
                        ).astype(np.float32),
                    )
                    body_relative_visual_feet = body_relative_actual_cad_visual_feet(env, env_index)
                    max_body_relative_visual_foot_displacement[env_index] = np.maximum(
                        max_body_relative_visual_foot_displacement[env_index],
                        np.linalg.norm(
                            body_relative_visual_feet - initial_body_relative_visual_foot_positions[env_index],
                            axis=1,
                        ).astype(np.float32),
                    )
                reward_foot_positions, reward_foot_radii = env._reward_foot_positions_and_radii(env_index)
                foot_contacts = env._foot_contact_flags_np(
                    reward_foot_positions,
                    env_index,
                    radii_m=reward_foot_radii,
                )
                foot_contact_sum += float(sum(foot_contacts))
                command = torch.tensor(
                    [float(args_cli.command_x_m_s), float(args_cli.command_y_m_s), float(args_cli.command_yaw_rad_s)],
                    dtype=torch.float32,
                    device=wrapped_env.device,
                )
                desired_stance = env._desired_stance_np(env_index, command.detach().cpu().numpy())
                desired_swing = 1.0 - desired_stance
                stance_count = max(float(sum(desired_stance)), 1.0)
                swing_count = max(float(sum(desired_swing)), 1.0)
                gait_contact_match_sum += float(sum(1.0 - abs(foot_contacts - desired_stance))) / len(foot_contacts)
                stance_contact_sum += float(sum(foot_contacts * desired_stance)) / stance_count
                swing_contact_sum += float(sum(foot_contacts * desired_swing)) / swing_count
                if float(sum(desired_swing)) > 0.0:
                    ground_clearance = torch.tensor(
                        env._foot_ground_clearance_np(
                            reward_foot_positions,
                            env_index,
                            radii_m=reward_foot_radii,
                        ),
                        dtype=torch.float32,
                        device=wrapped_env.device,
                    )
                    swing_clearance_sum += float(torch.sum(torch.clamp(ground_clearance, min=0.0) * torch.tensor(desired_swing, dtype=torch.float32, device=wrapped_env.device)).detach().cpu())
                    swing_sample_count += int(sum(desired_swing))
                linear_velocity_tensor = torch.tensor(linear_velocity, dtype=torch.float32, device=wrapped_env.device)
                displacement_velocity_tensor = torch.tensor(displacement_velocity, dtype=torch.float32, device=wrapped_env.device)
                angular_velocity_tensor = torch.tensor(angular_velocity, dtype=torch.float32, device=wrapped_env.device)
                initial_orientation = env._initial_body_states[env_index]["body_reference"]["orientation"].reshape(-1)
                current_yaw = yaw_from_quat_wxyz(orientation)
                initial_yaw = yaw_from_quat_wxyz(initial_orientation)
                target_yaw = initial_yaw + float(command[2]) * float(env.episode_length_buf[env_index].detach().cpu()) * float(env.step_dt)
                yaw_heading_drift = wrap_angle_rad(current_yaw - target_yaw)
                linear_velocity_sum[env_index] += linear_velocity_tensor
                displacement_velocity_sum[env_index] += displacement_velocity_tensor
                command_velocity_error_sum[env_index] += torch.linalg.norm(linear_velocity_tensor[:2] - command[:2])
                displacement_command_velocity_error_sum[env_index] += torch.linalg.norm(
                    displacement_velocity_tensor[:2] - command[:2]
                )
                yaw_rate_error_sum[env_index] += torch.abs(angular_velocity_tensor[2] - command[2])
                yaw_heading_drift_sum[env_index] += abs(float(yaw_heading_drift))
            update_linkage_motion_tracker(linkage_motion_tracker, env)
            update_foot_endpoint_motion_tracker(foot_endpoint_motion_tracker, env)
            final_displacements = [
                (env._body_reference_state(env_index)[0] - env._initial_body_states[env_index]["body_reference"]["position"]).tolist()
                for env_index in range(env.num_envs)
            ]
            final_linear_velocities = [
                env._body_reference_state(env_index)[2].tolist()
                for env_index in range(env.num_envs)
            ]
            final_yaw_heading_drift_rad = []
            for env_index in range(env.num_envs):
                _, orientation, _, _ = env._body_reference_state(env_index)
                initial_orientation = env._initial_body_states[env_index]["body_reference"]["orientation"].reshape(-1)
                current_yaw = yaw_from_quat_wxyz(orientation)
                initial_yaw = yaw_from_quat_wxyz(initial_orientation)
                target_yaw = initial_yaw + float(args_cli.command_yaw_rad_s) * float(env.episode_length_buf[env_index].detach().cpu()) * float(env.step_dt)
                final_yaw_heading_drift_rad.append(wrap_angle_rad(current_yaw - target_yaw))
            if (
                not torch.isfinite(actions).all()
                or not torch.isfinite(policy_observation_tensor(obs)).all()
                or not torch.isfinite(rewards).all()
            ):
                raise RuntimeError("Non-finite action, observation, or reward during checkpoint playback.")
            obs = move_observations_to_device(obs, agent_cfg.device)

    measured_linkage_motion = linkage_motion_report(linkage_motion_tracker)
    measured_foot_endpoint_motion = foot_endpoint_motion_report(foot_endpoint_motion_tracker)
    failure_reasons = []
    if int(done_count) > 0:
        failure_reasons.append(f"environment reset {int(done_count)} time(s) during playback")
    if int(terminated_count) > 0:
        failure_reasons.append(f"non-timeout termination {int(terminated_count)} time(s) during playback")
    if not math.isfinite(float(min_body_height)):
        failure_reasons.append("body height was not sampled")
    elif float(min_body_height) < float(env_cfg.min_height_m):
        failure_reasons.append(
            f"body reference height fell below min_height_m ({float(min_body_height):.6f} < {float(env_cfg.min_height_m):.6f})"
        )
    if not math.isfinite(float(max_body_tilt_deg)):
        failure_reasons.append("body tilt was not sampled")
    elif float(max_body_tilt_deg) > float(env_cfg.max_tilt_deg):
        failure_reasons.append(
            f"body tilt exceeded max_tilt_deg ({float(max_body_tilt_deg):.6f} > {float(env_cfg.max_tilt_deg):.6f})"
        )
    minimum_linkage_motion_deg = float(args_cli.min_each_linkage_drive_motion_deg)
    if (
        minimum_linkage_motion_deg > 0.0
        and float(measured_linkage_motion["min_each_drive_motion_deg"]) < minimum_linkage_motion_deg
    ):
        failure_reasons.append(
            "minimum actual linkage-drive rotation was below the playback gate "
            f"({float(measured_linkage_motion['min_each_drive_motion_deg']):.6f} < {minimum_linkage_motion_deg:.6f} deg)"
        )
    minimum_foot_motion_m = float(measured_foot_endpoint_motion["min_each_foot_motion_m"])
    required_foot_motion_m = float(args_cli.min_each_foot_motion_m)
    if required_foot_motion_m > 0.0 and minimum_foot_motion_m < required_foot_motion_m:
        failure_reasons.append(
            "minimum hip-carriage-relative rendered CAD foot motion was below the playback gate "
            f"({minimum_foot_motion_m:.6f} < {required_foot_motion_m:.6f} m)"
        )
    maximum_joint_separation_m = float(args_cli.max_joint_separation_m)
    if (
        maximum_joint_separation_m > 0.0
        and float(max_joint_separation_during_rollout_m) > maximum_joint_separation_m
    ):
        failure_reasons.append(
            "maximum loop-pin separation exceeded the playback gate "
            f"({float(max_joint_separation_during_rollout_m):.6f} > {maximum_joint_separation_m:.6f} m)"
        )

    report = {
        "status": "failed" if failure_reasons else "passed",
        "failure_reasons": failure_reasons,
        "policy_mode": str(args_cli.policy_mode),
        "startup_zero_steps": startup_zero_steps,
        "policy_ramp_steps": policy_ramp_steps,
        "visible_start_delay_s": float(args_cli.visible_start_delay_s),
        "visible_step_delay_s": float(args_cli.visible_step_delay_s),
        "startup_action_source": startup_action_source,
        "startup_action_values": [round(float(value), 6) for value in startup_action_values],
        "startup_front_rear_kinematic_symmetry": startup_front_rear_kinematic_symmetry,
        "fixed_action_values": [round(float(value), 6) for value in fixed_action_values],
        "startup_done_count": startup_done_count,
        "startup_terminated_count": startup_terminated_count,
        "startup_timeout_count": startup_timeout_count,
        "checkpoint": checkpoint_path.name if checkpoint_path is not None else "",
        "checkpoint_run": checkpoint_path.parent.name if checkpoint_path is not None else "",
        "steps": int(args_cli.steps),
        "num_envs": wrapped_env.num_envs,
        "geometry": env._linkage["geometry"],
        "closure_model": env._linkage.get("closure_model"),
        "visual_fidelity": env._linkage.get("visual_fidelity"),
        "actual_cad_visual": env._linkage.get("actual_cad_visual"),
        "cad_source": env._linkage.get("cad_source"),
        "actual_cad_visuals": env._linkage.get("actual_cad_visuals"),
        "visual_geometry_counts": env._linkage.get("visual_geometry_counts"),
        "actual_cad_visual_alignment": env._linkage.get("actual_cad_visual_alignment"),
        "post_startup_visual_alignment": post_startup_visual_alignment,
        "actual_cad_foot_collision": env._linkage.get("actual_cad_foot_collision"),
        "actuator_model": env._linkage.get("actuator_model"),
        "passive_stabilizers": env._linkage.get("passive_stabilizers", []),
        "joint_separation": env.joint_separation_report(),
        "max_joint_separation_during_rollout_m": round(max_joint_separation_during_rollout_m, 6),
        "worst_joint_separation_during_rollout": worst_joint_separation_during_rollout,
        "worst_joint_separation_by_name": sorted(
            worst_joint_separation_by_name.values(),
            key=lambda row: float(row["separation_m"]),
            reverse=True,
        ),
        "linkage_drive_motion": measured_linkage_motion,
        "hip_carriage_relative_actual_cad_foot_motion": measured_foot_endpoint_motion,
        "resolved_floating_height_m": env._linkage.get("resolved_floating_height_m"),
        "terrain": getattr(env, "_terrain_report", None),
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
            "action_scale_deg": float(env_cfg.action_scale_deg),
            "servo_target_rate_limit_deg_s": float(env_cfg.servo_target_rate_limit_deg_s),
            "episode_length_s": float(env_cfg.episode_length_s),
            "floating_height_m": float(env_cfg.floating_height_m),
            "min_height_m": float(env_cfg.min_height_m),
            "max_tilt_deg": float(env_cfg.max_tilt_deg),
            "target_height_m": float(env_cfg.target_height_m),
            "command_progress_reward_scale": float(env_cfg.command_progress_reward_scale),
            "command_velocity_reward_scale": float(env_cfg.command_velocity_reward_scale),
            "command_velocity_tracking_reward_scale": float(env_cfg.command_velocity_tracking_reward_scale),
            "use_displacement_velocity_rewards": bool(env_cfg.use_displacement_velocity_rewards),
            "lateral_drift_reward_scale": float(env_cfg.lateral_drift_reward_scale),
            "yaw_drift_reward_scale": float(env_cfg.yaw_drift_reward_scale),
            "command_yaw_reward_scale": float(env_cfg.command_yaw_reward_scale),
            "gait_contact_reward_scale": float(env_cfg.gait_contact_reward_scale),
            "stance_contact_reward_scale": float(env_cfg.stance_contact_reward_scale),
            "swing_contact_penalty_scale": float(env_cfg.swing_contact_penalty_scale),
            "foot_clearance_reward_scale": float(env_cfg.foot_clearance_reward_scale),
            "foot_contact_reward_scale": float(env_cfg.foot_contact_reward_scale),
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
            "align_rendered_visual_min_foot_after_startup": bool(args_cli.align_rendered_visual_min_foot_after_startup),
            "actual_cad_ground_clearance_m": float(env_cfg.actual_cad_ground_clearance_m),
            "terrain_type": str(env_cfg.terrain_type),
            "stairs_step_count": int(env_cfg.stairs_step_count),
            "stairs_step_depth_m": float(env_cfg.stairs_step_depth_m),
            "stairs_step_height_m": float(env_cfg.stairs_step_height_m),
            "stairs_width_m": float(env_cfg.stairs_width_m),
            "stairs_start_x_m": float(env_cfg.stairs_start_x_m),
            "stairs_top_platform_length_m": float(env_cfg.stairs_top_platform_length_m),
        },
        "reference_gait_candidate": reference_candidate,
        "mean_reference_action_tracking": [
            round(float(value), 6)
            for value in (reference_action_tracking_sum / max(int(args_cli.steps), 1)).detach().cpu()
        ],
        "mean_reference_action_mse": [
            round(float(value), 6)
            for value in (reference_action_mse_sum / max(int(args_cli.steps), 1)).detach().cpu()
        ],
        "reference_action_error_by_channel": reference_action_error_report(
            reference_action_sample_count,
            reference_executed_action_sum,
            reference_action_sum,
            reference_action_error_sum,
            reference_action_abs_error_sum,
            reference_action_sq_error_sum,
            reference_action_max_abs_error,
        ),
        "mean_foot_contacts_per_env": round(foot_contact_sum / max(int(args_cli.steps) * wrapped_env.num_envs, 1), 6),
        "foot_endpoint_motion": {
            "source": [
                [str(track.get("name", index)) for index, track in enumerate(env._foot_tracks_by_env[env_index])]
                for env_index in range(env.num_envs)
            ],
            "body": [
                [str(track.get("body", "")) for track in env._foot_tracks_by_env[env_index]]
                for env_index in range(env.num_envs)
            ],
            "radius_m": [
                [round(float(track.get("radius_m", env.cfg.foot_proxy_radius_m)), 6) for track in env._foot_tracks_by_env[env_index]]
                for env_index in range(env.num_envs)
            ],
            "initial_position_m": [
                [[round(float(value), 6) for value in row] for row in positions]
                for positions in initial_foot_positions
            ],
            "final_position_m": [
                [[round(float(value), 6) for value in row] for row in positions]
                for positions in final_foot_positions
            ],
            "final_displacement_m": [
                [[round(float(value), 6) for value in row] for row in (final_foot_positions[env_index] - initial_foot_positions[env_index])]
                for env_index in range(env.num_envs)
            ],
            "range_xyz_m": [
                [[round(float(value), 6) for value in row] for row in (max_foot_positions[env_index] - min_foot_positions[env_index])]
                for env_index in range(env.num_envs)
            ],
            "max_displacement_m": [
                [round(float(value), 6) for value in values]
                for values in max_foot_displacement
            ],
            "max_displacement_any_foot_m": round(
                max((float(np.max(values)) for values in max_foot_displacement), default=0.0),
                6,
            ),
        },
        "actual_cad_visual_foot_bottom_motion": {
            "source": [
                [
                    str(track.get("name", index))
                    for index, track in enumerate(env._actual_cad_visual_foot_tracks_by_env[env_index])
                ]
                for env_index in range(env.num_envs)
            ],
            "body": [
                [str(track.get("body", "")) for track in env._actual_cad_visual_foot_tracks_by_env[env_index]]
                for env_index in range(env.num_envs)
            ],
            "visual_z_lift_m": [
                round(float(env._linkages[env_index].get("actual_cad_visual_lift_m", 0.0) or 0.0), 6)
                for env_index in range(env.num_envs)
            ],
            "initial_unlifted_position_m": [
                [[round(float(value), 6) for value in row] for row in positions]
                for positions in initial_visual_foot_positions_unlifted
            ],
            "initial_rendered_position_m": [
                [[round(float(value), 6) for value in row] for row in positions]
                for positions in initial_visual_foot_positions_rendered
            ],
            "final_rendered_position_m": [
                [[round(float(value), 6) for value in row] for row in positions]
                for positions in final_visual_foot_positions_rendered
            ],
            "final_rendered_displacement_m": [
                [
                    [round(float(value), 6) for value in row]
                    for row in (
                        final_visual_foot_positions_rendered[env_index]
                        - initial_visual_foot_positions_rendered[env_index]
                    )
                ]
                for env_index in range(env.num_envs)
            ],
            "rendered_range_xyz_m": [
                [
                    [round(float(value), 6) for value in row]
                    for row in (
                        max_visual_foot_positions_rendered[env_index]
                        - min_visual_foot_positions_rendered[env_index]
                    )
                ]
                for env_index in range(env.num_envs)
            ],
            "max_rendered_displacement_m": [
                [round(float(value), 6) for value in values]
                for values in max_visual_foot_displacement_rendered
            ],
            "max_rendered_displacement_any_foot_m": round(
                max((float(np.max(values)) for values in max_visual_foot_displacement_rendered), default=0.0),
                6,
            ),
            "max_body_relative_displacement_m": [
                [round(float(value), 6) for value in values]
                for values in max_body_relative_visual_foot_displacement
            ],
            "min_body_relative_displacement_any_foot_m": round(
                minimum_foot_motion_m,
                6,
            ),
        },
        "actual_cad_visual_start_stance": visual_foot_height_report(
            initial_visual_foot_positions_rendered,
            env._actual_cad_visual_foot_tracks_by_env,
        ),
        "startup_leg_stance": startup_leg_stance,
        "startup_body_state": startup_body_state,
        "leg_state_after_rollout": env.leg_start_stance_report(),
        "actual_cad_visual_to_support_offset": visual_support_offset_report(
            initial_foot_positions,
            initial_visual_foot_positions_rendered,
            env._foot_tracks_by_env,
            env._actual_cad_visual_foot_tracks_by_env,
        ),
        "mean_gait_contact_match": round(gait_contact_match_sum / max(int(args_cli.steps) * wrapped_env.num_envs, 1), 6),
        "mean_stance_contact": round(stance_contact_sum / max(int(args_cli.steps) * wrapped_env.num_envs, 1), 6),
        "mean_swing_contact": round(swing_contact_sum / max(int(args_cli.steps) * wrapped_env.num_envs, 1), 6),
        "mean_swing_clearance_m": round(swing_clearance_sum / max(swing_sample_count, 1), 6),
        "mean_reward": round(float(torch.mean(total_reward / max(int(args_cli.steps), 1)).detach().cpu()), 6),
        "mean_reward_terms": averaged_reward_terms(reward_term_accumulator),
        "mean_body_reference_velocity_m_s": [
            [round(float(value), 6) for value in row]
            for row in (linear_velocity_sum / max(int(args_cli.steps), 1)).detach().cpu()
        ],
        "mean_body_reference_displacement_velocity_m_s": [
            [round(float(value), 6) for value in row]
            for row in (displacement_velocity_sum / max(int(args_cli.steps), 1)).detach().cpu()
        ],
        "mean_planar_command_velocity_error_m_s": [
            round(float(value), 6)
            for value in (command_velocity_error_sum / max(int(args_cli.steps), 1)).detach().cpu()
        ],
        "mean_planar_displacement_command_velocity_error_m_s": [
            round(float(value), 6)
            for value in (displacement_command_velocity_error_sum / max(int(args_cli.steps), 1)).detach().cpu()
        ],
        "mean_yaw_rate_error_rad_s": [
            round(float(value), 6)
            for value in (yaw_rate_error_sum / max(int(args_cli.steps), 1)).detach().cpu()
        ],
        "mean_abs_yaw_heading_drift_rad": [
            round(float(value), 6)
            for value in (yaw_heading_drift_sum / max(int(args_cli.steps), 1)).detach().cpu()
        ],
        "done_count": done_count,
        "terminated_count": terminated_count,
        "timeout_count": timeout_count,
        "termination_events": termination_events,
        "per_env_done_count": [int(value) for value in per_env_done_count.detach().cpu().tolist()],
        "per_env_terminated_count": [int(value) for value in per_env_terminated_count.detach().cpu().tolist()],
        "per_env_timeout_count": [int(value) for value in per_env_timeout_count.detach().cpu().tolist()],
        "min_body_reference_height_m": round(min_body_height, 6),
        "min_body_reference_height_by_env_m": [
            round(float(value), 6) for value in min_body_height_by_env.detach().cpu().tolist()
        ],
        "max_body_tilt_deg": round(max_body_tilt_deg, 6),
        "max_body_tilt_by_env_deg": [
            round(float(value), 6) for value in max_body_tilt_deg_by_env.detach().cpu().tolist()
        ],
        "max_action_abs": round(max_action_abs, 6),
        "max_executed_action_abs": round(max_executed_action_abs, 6),
        "max_executed_action_abs_by_channel": [
            round(float(value), 6) for value in max_executed_action_abs_by_channel.detach().cpu().tolist()
        ],
        "final_body_reference_displacement_m": [
            [round(float(value), 6) for value in displacement] for displacement in (final_displacements or [])
        ],
        "final_body_reference_linear_velocity_m_s": [
            [round(float(value), 6) for value in velocity] for velocity in (final_linear_velocities or [])
        ],
        "final_yaw_heading_drift_rad": [
            round(float(value), 6) for value in (final_yaw_heading_drift_rad or [])
        ],
    }

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2), flush=True)
    if args_cli.hold_open and VIEWABLE_RUN:
        # Isaac Lab's viewer extension applies its default camera after scene
        # construction, so frame Domino again at the final hold-open handoff.
        set_domino_inspection_camera(float(env._linkage.get("target_height_m", env_cfg.target_height_m)))
        refresh_visible_view(env, int(args_cli.hold_open_render_frames))
        captured_viewport_path = capture_visible_viewport(args_cli.capture_viewport_path)
        if captured_viewport_path:
            print(f"Captured Isaac viewport: {captured_viewport_path}", flush=True)
        hold_open_exit_after_frames = max(int(args_cli.hold_open_exit_after_frames), 0)
        print(
            "Playback report complete; continuing the live policy loop in the visible Isaac window. "
            "Close the window to exit.",
            flush=True,
        )
        continue_visible_policy_loop(
            wrapped_env,
            env,
            policy,
            obs,
            action_dim,
            agent_cfg.device,
            hold_open_exit_after_frames,
            fixed_action_values=fixed_action_values,
        )
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
