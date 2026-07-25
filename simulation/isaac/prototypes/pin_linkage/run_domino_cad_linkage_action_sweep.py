"""Sweep each Domino CAD-linkage actuator channel and report foot motion."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

from isaaclab.app import AppLauncher

CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))


parser = argparse.ArgumentParser(description="Run per-channel Domino CAD-linkage actuator sweeps.")
parser.add_argument("--settle-steps", type=int, default=30, help="Zero-action settle steps before each sweep.")
parser.add_argument("--sweep-steps", type=int, default=90, help="Steps to hold each one-hot actuator command.")
parser.add_argument("--action-value", type=float, default=1.0, help="Absolute normalized one-hot actuator command.")
parser.add_argument(
    "--active-channels",
    default="all",
    help="Comma-separated action channel indexes to sweep, or 'all'. Use 1,4,7,10 for the lower-linkage drives.",
)
parser.add_argument("--action-scale-deg", type=float, default=20.0, help="Maximum drive target offset for action=1.")
parser.add_argument(
    "--servo-target-rate-limit-deg-s",
    type=float,
    default=180.0,
    help="Maximum servo target slew rate in degrees/second. Use 0 to disable.",
)
parser.add_argument(
    "--foot-collision-mode",
    choices=["linkage-lower-closure", "actual-cad-visual-bottom", "actual-cad-grounded-support"],
    default="actual-cad-grounded-support",
    help="Foot contact proxy source.",
)
parser.add_argument("--terrain-type", choices=["flat", "stairs"], default="flat", help="Terrain used during the sweep.")
parser.add_argument("--floating-height-m", type=float, default=None, help="Optional starting body height for contact-free sweeps.")
parser.add_argument("--disable-gravity", action="store_true", help="Disable gravity for isolated linkage kinematic sweeps.")
parser.add_argument("--fixed-base", action="store_true", help="Anchor the body reference as kinematic for isolated linkage sweeps.")
parser.add_argument(
    "--closure-model",
    choices=["direct", "passive"],
    default="passive",
    help="Loop-closure topology to test. Passive restores tiny closure bodies at the CAD closure pivots.",
)
parser.add_argument("--seed", type=int, default=240704, help="Environment seed.")
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
parser.add_argument("--no-print-report", action="store_true", help="Write the report without printing full JSON.")
parser.add_argument(
    "--disable-actual-cad-visuals",
    action="store_true",
    help="Render only the simplified proxy. Intended for headless debugging.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

os.environ.setdefault("WARP_CACHE_PATH", str((Path.cwd() / "simulation" / "isaac" / "out" / "warp_cache").resolve()))

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from domino_action_contract import ACTION_JOINT_NAMES, EXPECTED_ACTION_COUNT, per_leg_action_layout  # noqa: E402
from domino_cad_linkage_builder import DOMINO_FOUR_COMBINED_LEG_SPECS  # noqa: E402
from domino_cad_linkage_env import (  # noqa: E402
    DominoCadLinkageEnv,
    DominoCadLinkageEnvCfg,
    quat_wxyz_to_rotation_matrix,
    to_numpy,
)
from domino_linkage_motion import hip_carriage_relative_actual_cad_visual_feet  # noqa: E402


def rounded_array(values: np.ndarray, decimals: int = 6) -> list:
    return np.round(np.asarray(values, dtype=np.float64), decimals).tolist()


def parse_active_channels(raw: str) -> list[int]:
    value = str(raw or "").strip().lower()
    if value in {"", "all"}:
        return list(range(EXPECTED_ACTION_COUNT))
    indexes = [int(item.strip()) for item in value.replace(";", ",").split(",") if item.strip()]
    for index in indexes:
        if index < 0 or index >= EXPECTED_ACTION_COUNT:
            raise ValueError(f"Action channel index out of range: {index}")
    return indexes


def foot_positions(env: DominoCadLinkageEnv) -> np.ndarray:
    return env._foot_positions(0).copy()


def body_position(env: DominoCadLinkageEnv) -> np.ndarray:
    return env._body_reference_state(0)[0].copy()


def body_pose(env: DominoCadLinkageEnv, body_name: str) -> tuple[np.ndarray, np.ndarray]:
    position, orientation = env._body_views_by_env[0][body_name].get_world_pose()
    return to_numpy(position).astype(np.float64).reshape(-1), to_numpy(orientation).astype(np.float64).reshape(-1)


def quat_delta_deg(initial: np.ndarray, final: np.ndarray) -> float:
    q0 = np.asarray(initial, dtype=np.float64).reshape(-1)[:4]
    q1 = np.asarray(final, dtype=np.float64).reshape(-1)[:4]
    q0 = q0 / max(np.linalg.norm(q0), 1.0e-9)
    q1 = q1 / max(np.linalg.norm(q1), 1.0e-9)
    dot = max(-1.0, min(1.0, abs(float(np.dot(q0, q1)))))
    return float(np.degrees(2.0 * np.arccos(dot)))


def body_pose_map(env: DominoCadLinkageEnv) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    names = []
    for spec in DOMINO_FOUR_COMBINED_LEG_SPECS:
        prefix = spec["id"]
        names.extend(
            [
                f"{prefix}_ground",
                f"{prefix}_lower_driver",
                f"{prefix}_coupler",
                f"{prefix}_lower_diagonal",
                f"{prefix}_upper_driver",
                f"{prefix}_lower_closure",
                f"{prefix}_upper_closure",
            ]
        )
    views = env._body_views_by_env[0]
    return {name: body_pose(env, name) for name in names if name in views}


def body_orientation_delta_report(
    initial_poses: dict[str, tuple[np.ndarray, np.ndarray]],
    final_poses: dict[str, tuple[np.ndarray, np.ndarray]],
) -> dict[str, float]:
    return {
        name: round(quat_delta_deg(initial_poses[name][1], final_poses[name][1]), 6)
        for name in sorted(initial_poses)
        if name in final_poses
    }


def linkage_point_positions(env: DominoCadLinkageEnv) -> dict[str, np.ndarray]:
    positions: dict[str, np.ndarray] = {}
    linkage = env._linkages[0]
    body_points = {
        "lower_driver": ("lower_drive", "lower_passive", "lower_closure"),
        "coupler": ("lower_passive", "lower_coupler", "upper_closure"),
        "lower_diagonal": ("lower_coupler", "lower_closure"),
        "upper_driver": ("upper_drive", "upper_closure"),
        "lower_closure": ("lower_closure",),
        "upper_closure": ("upper_closure",),
    }
    for spec in DOMINO_FOUR_COMBINED_LEG_SPECS:
        prefix = spec["id"]
        for body_suffix, point_names in body_points.items():
            body_name = f"{prefix}_{body_suffix}"
            if body_name not in env._body_views_by_env[0]:
                continue
            for point_name in point_names:
                key = f"{prefix}.{body_suffix}.{point_name}"
                positions[key] = env._body_world_endpoint(0, body_name, linkage["points"][f"{prefix}_{point_name}"])
    return positions


def body_relative_point_positions(env: DominoCadLinkageEnv, positions: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    body_pos, body_quat, _, _ = env._body_reference_state(0)
    rotation = quat_wxyz_to_rotation_matrix(body_quat)
    return {
        key: ((np.asarray(value, dtype=np.float64) - body_pos.reshape(3)) @ rotation).astype(np.float32)
        for key, value in positions.items()
    }


def point_displacement_report(initial: dict[str, np.ndarray], final: dict[str, np.ndarray]) -> dict[str, float]:
    return {
        key: round(float(np.linalg.norm(np.asarray(final[key], dtype=np.float64) - np.asarray(initial[key], dtype=np.float64))), 6)
        for key in sorted(initial)
        if key in final
    }


def body_relative_foot_positions(env: DominoCadLinkageEnv) -> np.ndarray:
    body_pos, body_quat, _, _ = env._body_reference_state(0)
    rotation = quat_wxyz_to_rotation_matrix(body_quat)
    return ((foot_positions(env) - body_pos.reshape(1, 3)) @ rotation).astype(np.float32)


def actual_cad_visual_foot_positions(env: DominoCadLinkageEnv) -> np.ndarray:
    return env._actual_cad_visual_foot_positions(0, rendered=True).copy()


def body_relative_actual_cad_visual_foot_positions(env: DominoCadLinkageEnv) -> np.ndarray:
    body_pos, body_quat, _, _ = env._body_reference_state(0)
    rotation = quat_wxyz_to_rotation_matrix(body_quat)
    visual_feet = actual_cad_visual_foot_positions(env)
    if visual_feet.size == 0:
        return visual_feet.reshape(0, 3).astype(np.float32)
    return ((visual_feet - body_pos.reshape(1, 3)) @ rotation).astype(np.float32)


def step_actions(env: DominoCadLinkageEnv, actions: torch.Tensor, steps: int) -> tuple[int, int]:
    done_count = 0
    terminated_count = 0
    for _ in range(int(steps)):
        _, _, terminated, truncated, _ = env.step(actions)
        terminated_count += int(torch.count_nonzero(terminated).detach().cpu())
        done_count += int(torch.count_nonzero(torch.logical_or(terminated, truncated)).detach().cpu())
    return done_count, terminated_count


def reset_and_settle(env: DominoCadLinkageEnv, settle_steps: int) -> None:
    env.reset()
    zero = torch.zeros((1, EXPECTED_ACTION_COUNT), dtype=torch.float32, device=env.device)
    step_actions(env, zero, int(settle_steps))


def sweep_channel(env: DominoCadLinkageEnv, channel_index: int, sign: float) -> dict[str, object]:
    reset_and_settle(env, int(args_cli.settle_steps))
    initial_feet = foot_positions(env)
    initial_relative_feet = body_relative_foot_positions(env)
    initial_visual_feet = actual_cad_visual_foot_positions(env)
    initial_relative_visual_feet = body_relative_actual_cad_visual_foot_positions(env)
    initial_hip_relative_visual_feet = hip_carriage_relative_actual_cad_visual_feet(env, 0)
    initial_body_poses = body_pose_map(env)
    initial_linkage_points = linkage_point_positions(env)
    initial_relative_linkage_points = body_relative_point_positions(env, initial_linkage_points)
    initial_targets_deg = [float(spec.get("current_target_deg", spec["center_deg"])) for spec in env._drive_specs_by_env[0]]
    initial_body = body_position(env)
    min_feet = initial_feet.copy()
    max_feet = initial_feet.copy()
    min_relative_feet = initial_relative_feet.copy()
    max_relative_feet = initial_relative_feet.copy()
    min_visual_feet = initial_visual_feet.copy()
    max_visual_feet = initial_visual_feet.copy()
    min_relative_visual_feet = initial_relative_visual_feet.copy()
    max_relative_visual_feet = initial_relative_visual_feet.copy()
    min_hip_relative_visual_feet = initial_hip_relative_visual_feet.copy()
    max_hip_relative_visual_feet = initial_hip_relative_visual_feet.copy()

    actions = torch.zeros((1, EXPECTED_ACTION_COUNT), dtype=torch.float32, device=env.device)
    actions[0, channel_index] = float(sign) * abs(float(args_cli.action_value))

    done_count = 0
    terminated_count = 0
    for _ in range(int(args_cli.sweep_steps)):
        _, _, terminated, truncated, _ = env.step(actions)
        terminated_count += int(torch.count_nonzero(terminated).detach().cpu())
        done_count += int(torch.count_nonzero(torch.logical_or(terminated, truncated)).detach().cpu())
        feet = foot_positions(env)
        relative_feet = body_relative_foot_positions(env)
        visual_feet = actual_cad_visual_foot_positions(env)
        relative_visual_feet = body_relative_actual_cad_visual_foot_positions(env)
        hip_relative_visual_feet = hip_carriage_relative_actual_cad_visual_feet(env, 0)
        min_feet = np.minimum(min_feet, feet)
        max_feet = np.maximum(max_feet, feet)
        min_relative_feet = np.minimum(min_relative_feet, relative_feet)
        max_relative_feet = np.maximum(max_relative_feet, relative_feet)
        if visual_feet.size:
            min_visual_feet = np.minimum(min_visual_feet, visual_feet)
            max_visual_feet = np.maximum(max_visual_feet, visual_feet)
            min_relative_visual_feet = np.minimum(min_relative_visual_feet, relative_visual_feet)
            max_relative_visual_feet = np.maximum(max_relative_visual_feet, relative_visual_feet)
            min_hip_relative_visual_feet = np.minimum(min_hip_relative_visual_feet, hip_relative_visual_feet)
            max_hip_relative_visual_feet = np.maximum(max_hip_relative_visual_feet, hip_relative_visual_feet)

    final_feet = foot_positions(env)
    final_relative_feet = body_relative_foot_positions(env)
    final_visual_feet = actual_cad_visual_foot_positions(env)
    final_relative_visual_feet = body_relative_actual_cad_visual_foot_positions(env)
    final_hip_relative_visual_feet = hip_carriage_relative_actual_cad_visual_feet(env, 0)
    final_body_poses = body_pose_map(env)
    final_linkage_points = linkage_point_positions(env)
    final_relative_linkage_points = body_relative_point_positions(env, final_linkage_points)
    final_targets_deg = [float(spec.get("current_target_deg", spec["center_deg"])) for spec in env._drive_specs_by_env[0]]
    final_body = body_position(env)
    displacement = final_feet - initial_feet
    relative_displacement = final_relative_feet - initial_relative_feet
    displacement_norm = np.linalg.norm(displacement, axis=1)
    relative_displacement_norm = np.linalg.norm(relative_displacement, axis=1)
    visual_displacement = final_visual_feet - initial_visual_feet if final_visual_feet.size else np.zeros((0, 3), dtype=np.float32)
    relative_visual_displacement = (
        final_relative_visual_feet - initial_relative_visual_feet
        if final_relative_visual_feet.size
        else np.zeros((0, 3), dtype=np.float32)
    )
    hip_relative_visual_displacement = (
        final_hip_relative_visual_feet - initial_hip_relative_visual_feet
        if final_hip_relative_visual_feet.size
        else np.zeros((0, 3), dtype=np.float32)
    )
    visual_displacement_norm = np.linalg.norm(visual_displacement, axis=1) if visual_displacement.size else np.zeros(0)
    relative_visual_displacement_norm = (
        np.linalg.norm(relative_visual_displacement, axis=1) if relative_visual_displacement.size else np.zeros(0)
    )
    hip_relative_visual_displacement_norm = (
        np.linalg.norm(hip_relative_visual_displacement, axis=1)
        if hip_relative_visual_displacement.size
        else np.zeros(0)
    )
    z_delta = displacement[:, 2]
    relative_z_delta = relative_displacement[:, 2]
    xy_delta = displacement[:, :2]
    relative_xy_delta = relative_displacement[:, :2]
    joint_report = env.joint_separation_report()
    dominant_foot_index = int(np.argmax(relative_displacement_norm))
    dominant_visual_foot_index = int(np.argmax(relative_visual_displacement_norm)) if len(relative_visual_displacement_norm) else -1

    return {
        "channel_index": int(channel_index),
        "action_name": ACTION_JOINT_NAMES[channel_index],
        "sign": float(sign),
        "done_count": int(done_count),
        "terminated_count": int(terminated_count),
        "max_joint_separation_m": float(joint_report["max_separation_m"]),
        "dominant_foot_index": dominant_foot_index,
        "dominant_foot_motion_m": round(float(relative_displacement_norm[dominant_foot_index]), 6),
        "foot_displacement_m": rounded_array(displacement),
        "foot_displacement_norm_m": rounded_array(displacement_norm),
        "foot_z_delta_m": rounded_array(z_delta),
        "foot_xy_delta_m": rounded_array(xy_delta),
        "foot_range_xyz_m": rounded_array(max_feet - min_feet),
        "body_relative_foot_displacement_m": rounded_array(relative_displacement),
        "body_relative_foot_displacement_norm_m": rounded_array(relative_displacement_norm),
        "body_relative_foot_z_delta_m": rounded_array(relative_z_delta),
        "body_relative_foot_xy_delta_m": rounded_array(relative_xy_delta),
        "body_relative_foot_range_xyz_m": rounded_array(max_relative_feet - min_relative_feet),
        "actual_cad_visual_foot_displacement_m": rounded_array(visual_displacement),
        "actual_cad_visual_foot_displacement_norm_m": rounded_array(visual_displacement_norm),
        "actual_cad_visual_foot_range_xyz_m": rounded_array(max_visual_feet - min_visual_feet),
        "body_relative_actual_cad_visual_foot_displacement_m": rounded_array(relative_visual_displacement),
        "body_relative_actual_cad_visual_foot_displacement_norm_m": rounded_array(relative_visual_displacement_norm),
        "body_relative_actual_cad_visual_foot_range_xyz_m": rounded_array(
            max_relative_visual_feet - min_relative_visual_feet
        ),
        "hip_carriage_relative_actual_cad_visual_foot_displacement_m": rounded_array(
            hip_relative_visual_displacement
        ),
        "hip_carriage_relative_actual_cad_visual_foot_displacement_norm_m": rounded_array(
            hip_relative_visual_displacement_norm
        ),
        "hip_carriage_relative_actual_cad_visual_foot_range_xyz_m": rounded_array(
            max_hip_relative_visual_feet - min_hip_relative_visual_feet
        ),
        "dominant_actual_cad_visual_foot_index": dominant_visual_foot_index,
        "dominant_actual_cad_visual_foot_motion_m": (
            round(float(relative_visual_displacement_norm[dominant_visual_foot_index]), 6)
            if dominant_visual_foot_index >= 0
            else 0.0
        ),
        "body_displacement_m": rounded_array(final_body - initial_body),
        "body_orientation_delta_deg": body_orientation_delta_report(initial_body_poses, final_body_poses),
        "linkage_point_displacement_norm_m": point_displacement_report(initial_linkage_points, final_linkage_points),
        "body_relative_linkage_point_displacement_norm_m": point_displacement_report(
            initial_relative_linkage_points,
            final_relative_linkage_points,
        ),
        "initial_target_deg": rounded_array(np.asarray(initial_targets_deg, dtype=np.float64)),
        "final_target_deg": rounded_array(np.asarray(final_targets_deg, dtype=np.float64)),
        "target_delta_deg": rounded_array(np.asarray(final_targets_deg, dtype=np.float64) - np.asarray(initial_targets_deg, dtype=np.float64)),
    }


def main() -> None:
    cfg = DominoCadLinkageEnvCfg()
    cfg.scene.num_envs = 1
    cfg.sim.device = args_cli.device
    cfg.seed = int(args_cli.seed)
    cfg.include_actual_cad_visuals = not bool(args_cli.disable_actual_cad_visuals)
    cfg.fixed_base = bool(args_cli.fixed_base)
    cfg.closure_model = str(args_cli.closure_model)
    cfg.enable_gravity = not bool(args_cli.disable_gravity)
    if args_cli.floating_height_m is not None:
        cfg.floating_height_m = float(args_cli.floating_height_m)
    cfg.action_scale_deg = float(args_cli.action_scale_deg)
    cfg.servo_target_rate_limit_deg_s = float(args_cli.servo_target_rate_limit_deg_s)
    cfg.min_height_m = -10.0
    cfg.max_tilt_deg = 180.0
    cfg.foot_contact_mode = str(args_cli.foot_collision_mode).replace("-", "_")
    cfg.use_actual_cad_foot_collision = str(args_cli.foot_collision_mode) == "actual-cad-visual-bottom"
    cfg.terrain_type = str(args_cli.terrain_type)
    cfg.episode_length_s = max(6.0, (int(args_cli.settle_steps) + int(args_cli.sweep_steps) + 20) * 0.02)

    env = DominoCadLinkageEnv(cfg)
    observations, _ = env.reset()
    action_dim = gym.spaces.flatdim(env.single_action_space)
    if action_dim != EXPECTED_ACTION_COUNT:
        raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} actions, found {action_dim}.")
    if observations["policy"].shape[0] != 1:
        raise RuntimeError("The action sweep expects one environment.")

    active_channels = parse_active_channels(args_cli.active_channels)
    rows = []
    for channel_index in active_channels:
        rows.append(sweep_channel(env, channel_index, +1.0))
        rows.append(sweep_channel(env, channel_index, -1.0))

    report = {
        "status": "passed",
        "settle_steps": int(args_cli.settle_steps),
        "sweep_steps": int(args_cli.sweep_steps),
        "action_value": float(args_cli.action_value),
        "action_scale_deg": float(cfg.action_scale_deg),
        "servo_target_rate_limit_deg_s": float(cfg.servo_target_rate_limit_deg_s),
        "floating_height_m": float(cfg.floating_height_m),
        "enable_gravity": bool(cfg.enable_gravity),
        "fixed_base": bool(cfg.fixed_base),
        "closure_model": str(cfg.closure_model),
        "foot_collision_mode": str(args_cli.foot_collision_mode),
        "terrain_type": str(args_cli.terrain_type),
        "action_names": ACTION_JOINT_NAMES,
        "active_channels": active_channels,
        "per_leg_action_layout": per_leg_action_layout(),
        "rows": rows,
    }

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if not bool(args_cli.no_print_report):
        print(json.dumps(report, indent=2), flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if bool(getattr(args_cli, "graceful_close", False)):
            simulation_app.close()
