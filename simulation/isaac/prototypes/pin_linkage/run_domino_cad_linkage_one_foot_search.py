"""Search shoulder/lower/upper commands for a stable single-foot unload."""

from __future__ import annotations

import argparse
from itertools import product
import json
import math
import os
from pathlib import Path
import sys

from isaaclab.app import AppLauncher

CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))


parser = argparse.ArgumentParser(description="Search a stable three-servo one-foot unload for Domino.")
parser.add_argument("--leg-index", type=int, required=True, choices=range(4), help="Target leg index in the 12-action contract.")
parser.add_argument("--seed", type=int, default=240722, help="Deterministic Isaac environment seed.")
parser.add_argument("--candidate-values", default="-1,0,1", help="Comma-separated shoulder/lower/upper action values.")
parser.add_argument("--settle-steps", type=int, default=120, help="Zero-action neutral settle before candidate commands.")
parser.add_argument("--hold-steps", type=int, default=60, help="Steps to hold each candidate command.")
parser.add_argument("--action-scale-deg", type=float, default=8.0, help="Physical target offset represented by action magnitude 1.")
parser.add_argument("--servo-target-rate-limit-deg-s", type=float, default=90.0, help="Servo target slew limit.")
parser.add_argument("--max-stable-tilt-deg", type=float, default=30.0, help="Maximum tilt accepted as a stable unload.")
parser.add_argument("--max-stable-joint-separation-m", type=float, default=0.001, help="Maximum pin error accepted as stable.")
parser.add_argument("--min-stable-body-height-m", type=float, default=0.22, help="Minimum body height accepted as stable.")
parser.add_argument("--report-path", default="", help="Optional JSON report path.")
parser.add_argument("--no-print-report", action="store_true", help="Suppress full JSON on stdout.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

os.environ.setdefault("WARP_CACHE_PATH", str((Path.cwd() / "simulation" / "isaac" / "out" / "warp_cache").resolve()))

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from domino_action_contract import ACTION_JOINT_NAMES, EXPECTED_ACTION_COUNT, per_leg_action_layout  # noqa: E402
from domino_cad_linkage_env import (  # noqa: E402
    DominoCadLinkageEnv,
    DominoCadLinkageEnvCfg,
    projected_gravity_from_quat,
    quat_wxyz_to_rotation_matrix,
)


def parse_candidate_values(raw: str) -> list[float]:
    values = [float(item.strip()) for item in str(raw).replace(";", ",").split(",") if item.strip()]
    if not values:
        raise ValueError("At least one candidate value is required.")
    return sorted({max(-1.0, min(1.0, value)) for value in values})


def body_relative_visual_feet(env: DominoCadLinkageEnv, env_index: int) -> np.ndarray:
    body_position, body_orientation, _, _ = env._body_reference_state(env_index)
    visual_feet = env._actual_cad_visual_foot_positions(env_index, rendered=True)
    world_from_body = quat_wxyz_to_rotation_matrix(body_orientation)
    return ((visual_feet - body_position.reshape(1, 3)) @ world_from_body).astype(np.float32)


def body_tilt_deg(orientation: np.ndarray) -> float:
    projected_gravity = projected_gravity_from_quat(orientation)
    return math.degrees(math.acos(max(-1.0, min(1.0, -float(projected_gravity[2])))))


def main() -> None:
    candidate_values = parse_candidate_values(args_cli.candidate_values)
    local_candidates = [tuple(float(value) for value in row) for row in product(candidate_values, repeat=3)]
    local_candidates = [row for row in local_candidates if any(abs(value) > 1.0e-9 for value in row)]
    if not local_candidates:
        raise ValueError("The candidate grid contains only the all-zero action row.")

    cfg = DominoCadLinkageEnvCfg()
    cfg.seed = int(args_cli.seed)
    cfg.scene.num_envs = len(local_candidates)
    cfg.scene.env_spacing = 1.0
    cfg.sim.device = args_cli.device
    cfg.include_actual_cad_visuals = False
    cfg.hide_proxy_visuals_when_actual_cad = True
    cfg.fixed_base = False
    cfg.enable_gravity = True
    cfg.closure_model = "passive"
    cfg.foot_contact_mode = "actual_cad_visual_bottom"
    cfg.use_actual_cad_foot_collision = True
    cfg.terrain_type = "flat"
    cfg.action_scale_deg = float(args_cli.action_scale_deg)
    cfg.servo_target_rate_limit_deg_s = float(args_cli.servo_target_rate_limit_deg_s)
    cfg.min_height_m = -10.0
    cfg.max_tilt_deg = 180.0
    cfg.episode_length_s = max(8.0, (int(args_cli.settle_steps) + int(args_cli.hold_steps) + 20) * 0.02)

    env = DominoCadLinkageEnv(cfg)
    observations, _ = env.reset()
    if gym.spaces.flatdim(env.single_action_space) != EXPECTED_ACTION_COUNT:
        raise RuntimeError("Domino one-foot search requires the 12-action contract.")
    if observations["policy"].shape[0] != len(local_candidates):
        raise RuntimeError("Candidate/environment count mismatch.")

    zero_actions = torch.zeros((env.num_envs, EXPECTED_ACTION_COUNT), dtype=torch.float32, device=env.device)
    settle_done = torch.zeros(env.num_envs, dtype=torch.int64, device=env.device)
    settle_terminated = torch.zeros_like(settle_done)
    finite = True
    for _ in range(max(int(args_cli.settle_steps), 0)):
        observations, rewards, terminated, truncated, _ = env.step(zero_actions)
        settle_done += torch.logical_or(terminated, truncated).to(dtype=torch.int64)
        settle_terminated += terminated.to(dtype=torch.int64)
        finite = finite and bool(torch.isfinite(observations["policy"]).all()) and bool(torch.isfinite(rewards).all())
    if hasattr(env, "episode_length_buf"):
        env.episode_length_buf[:] = 0

    initial_body_positions = []
    initial_visual_feet = []
    initial_body_relative_feet = []
    for env_index in range(env.num_envs):
        body_position, _, _, _ = env._body_reference_state(env_index)
        initial_body_positions.append(body_position.copy())
        initial_visual_feet.append(env._actual_cad_visual_foot_positions(env_index, rendered=True).copy())
        initial_body_relative_feet.append(body_relative_visual_feet(env, env_index))

    actions = torch.zeros((env.num_envs, EXPECTED_ACTION_COUNT), dtype=torch.float32, device=env.device)
    channel_start = int(args_cli.leg_index) * 3
    for env_index, candidate in enumerate(local_candidates):
        actions[env_index, channel_start : channel_start + 3] = torch.tensor(candidate, device=env.device)

    done_counts = torch.zeros(env.num_envs, dtype=torch.int64, device=env.device)
    terminated_counts = torch.zeros_like(done_counts)
    max_tilts = np.zeros(env.num_envs, dtype=np.float64)
    min_heights = np.full(env.num_envs, np.inf, dtype=np.float64)
    max_joint_separation = np.zeros(env.num_envs, dtype=np.float64)
    peak_visual_clearance = np.full((env.num_envs, 4), -np.inf, dtype=np.float64)
    max_body_relative_motion = np.zeros((env.num_envs, 4), dtype=np.float64)

    for step_index in range(max(int(args_cli.hold_steps), 0)):
        observations, rewards, terminated, truncated, _ = env.step(actions)
        done_counts += torch.logical_or(terminated, truncated).to(dtype=torch.int64)
        terminated_counts += terminated.to(dtype=torch.int64)
        finite = finite and bool(torch.isfinite(observations["policy"]).all()) and bool(torch.isfinite(rewards).all())
        for env_index in range(env.num_envs):
            body_position, body_orientation, _, _ = env._body_reference_state(env_index)
            body_height = float(body_position[2] - env._env_origins_np[env_index][2])
            min_heights[env_index] = min(min_heights[env_index], body_height)
            max_tilts[env_index] = max(max_tilts[env_index], body_tilt_deg(body_orientation))
            visual_feet = env._actual_cad_visual_foot_positions(env_index, rendered=True)
            terrain_heights = env._terrain_heights_np(visual_feet, env_index=env_index)
            peak_visual_clearance[env_index] = np.maximum(
                peak_visual_clearance[env_index],
                visual_feet[:, 2] - terrain_heights,
            )
            relative_feet = body_relative_visual_feet(env, env_index)
            max_body_relative_motion[env_index] = np.maximum(
                max_body_relative_motion[env_index],
                np.linalg.norm(relative_feet - initial_body_relative_feet[env_index], axis=1),
            )
            if step_index % 5 == 0 or step_index + 1 == int(args_cli.hold_steps):
                joint_rows = env._joint_separation_rows(env_index)
                if joint_rows:
                    max_joint_separation[env_index] = max(
                        max_joint_separation[env_index],
                        max(float(row["separation_m"]) for row in joint_rows),
                    )

    rows = []
    target_leg = int(args_cli.leg_index)
    stance_legs = [index for index in range(4) if index != target_leg]
    for env_index, candidate in enumerate(local_candidates):
        final_body_position, _, _, _ = env._body_reference_state(env_index)
        final_visual_feet = env._actual_cad_visual_foot_positions(env_index, rendered=True)
        body_displacement = final_body_position - initial_body_positions[env_index]
        target_peak_clearance = float(peak_visual_clearance[env_index, target_leg])
        target_peak_lift = target_peak_clearance - float(initial_visual_feet[env_index][target_leg, 2])
        stance_peak_clearance = float(np.max(peak_visual_clearance[env_index, stance_legs]))
        stable = (
            finite
            and int(settle_done[env_index].detach().cpu()) == 0
            and int(done_counts[env_index].detach().cpu()) == 0
            and float(max_tilts[env_index]) <= float(args_cli.max_stable_tilt_deg)
            and float(min_heights[env_index]) >= float(args_cli.min_stable_body_height_m)
            and float(max_joint_separation[env_index]) <= float(args_cli.max_stable_joint_separation_m)
        )
        score = (
            target_peak_clearance
            + (0.5 * target_peak_lift)
            - (0.25 * max(stance_peak_clearance, 0.0))
            - (0.001 * float(max_tilts[env_index]))
            - (0.4 * float(np.linalg.norm(body_displacement[:2])))
            - (0.8 * max(0.0, -float(body_displacement[2])))
            - (4.0 * max(0.0, float(max_joint_separation[env_index]) - 0.003))
        )
        rows.append(
            {
                "candidate_index": int(env_index),
                "local_actions": [round(float(value), 6) for value in candidate],
                "action_row": [round(float(value), 6) for value in actions[env_index].detach().cpu().tolist()],
                "stable": bool(stable),
                "score": round(float(score), 6),
                "settle_done_count": int(settle_done[env_index].detach().cpu()),
                "settle_terminated_count": int(settle_terminated[env_index].detach().cpu()),
                "done_count": int(done_counts[env_index].detach().cpu()),
                "terminated_count": int(terminated_counts[env_index].detach().cpu()),
                "max_tilt_deg": round(float(max_tilts[env_index]), 6),
                "min_body_height_m": round(float(min_heights[env_index]), 6),
                "max_joint_separation_m": round(float(max_joint_separation[env_index]), 6),
                "target_peak_visual_clearance_m": round(target_peak_clearance, 6),
                "target_peak_visual_lift_from_start_m": round(target_peak_lift, 6),
                "stance_peak_visual_clearance_m": round(stance_peak_clearance, 6),
                "target_body_relative_motion_m": round(float(max_body_relative_motion[env_index, target_leg]), 6),
                "body_displacement_m": [round(float(value), 6) for value in body_displacement.tolist()],
                "final_visual_foot_bottom_m": [
                    [round(float(value), 6) for value in foot.tolist()] for foot in final_visual_feet
                ],
            }
        )

    rows.sort(key=lambda row: (bool(row["stable"]), float(row["score"])), reverse=True)
    report = {
        "status": "passed" if any(bool(row["stable"]) for row in rows) else "no_stable_candidate",
        "seed": int(args_cli.seed),
        "target_leg_index": target_leg,
        "target_leg": per_leg_action_layout()[target_leg],
        "candidate_values": candidate_values,
        "candidate_count": len(local_candidates),
        "settle_steps": int(args_cli.settle_steps),
        "hold_steps": int(args_cli.hold_steps),
        "action_scale_deg": float(args_cli.action_scale_deg),
        "servo_target_rate_limit_deg_s": float(args_cli.servo_target_rate_limit_deg_s),
        "stability_thresholds": {
            "max_tilt_deg": float(args_cli.max_stable_tilt_deg),
            "max_joint_separation_m": float(args_cli.max_stable_joint_separation_m),
            "min_body_height_m": float(args_cli.min_stable_body_height_m),
        },
        "action_names": ACTION_JOINT_NAMES,
        "best": rows[0],
        "stable_candidate_count": sum(bool(row["stable"]) for row in rows),
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
