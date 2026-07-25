"""Search a constant 12-servo startup stance for the Domino CAD linkage.

The goal is not locomotion.  This runner finds a repeatable startup action row
that makes the rendered Domino CAD foot bottoms more level while the proxy
closed-linkage physics remains stable.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys

from isaaclab.app import AppLauncher

CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))


parser = argparse.ArgumentParser(description="Calibrate a constant Domino visible-CAD startup stance.")
parser.add_argument("--settle-steps", type=int, default=120, help="Steps to hold each candidate action row.")
parser.add_argument("--iterations", type=int, default=2, help="Coordinate-descent passes over the selected channels.")
parser.add_argument(
    "--candidate-values",
    default="-1.0,-0.5,0.0,0.5,1.0",
    help="Comma-separated normalized action values tested for each selected channel.",
)
parser.add_argument(
    "--active-channels",
    default="all",
    help="Comma-separated channel indexes to tune, or 'all'. Use 'linkage' for lower/upper linkage channels only.",
)
parser.add_argument(
    "--search-mode",
    choices=["coordinate", "leg-pairs"],
    default="coordinate",
    help="Tune one channel at a time or jointly search each leg's lower/upper linkage pair.",
)
parser.add_argument(
    "--initial-actions",
    default="",
    help="Optional comma-separated 12-channel normalized action row to refine instead of starting from all zeros.",
)
parser.add_argument("--action-scale-deg", type=float, default=12.0, help="Action target scale used by the env.")
parser.add_argument(
    "--servo-target-rate-limit-deg-s",
    type=float,
    default=180.0,
    help="Servo target slew limit used during settling.",
)
parser.add_argument("--target-foot-z-m", type=float, default=0.002, help="Target rendered CAD foot-bottom height.")
parser.add_argument(
    "--target-planar-reach-m",
    type=float,
    default=0.065,
    help="Target rendered hip-to-foot planar reach for the base stance.",
)
parser.add_argument(
    "--foot-collision-mode",
    choices=["linkage-lower-closure", "actual-cad-visual-bottom", "actual-cad-grounded-support"],
    default="actual-cad-grounded-support",
    help="Physics contact mode used while searching.",
)
parser.add_argument("--floating-height-m", type=float, default=None, help="Optional starting body height override.")
parser.add_argument("--fixed-base", action="store_true", help="Hold the chassis kinematic while calibrating leg stance.")
parser.add_argument("--disable-gravity", action="store_true", help="Disable gravity while calibrating leg stance.")
parser.add_argument(
    "--closure-model",
    choices=["direct", "passive"],
    default="passive",
    help="Loop-closure topology used during the stance search.",
)
parser.add_argument("--terrain-type", choices=["flat", "stairs"], default="flat", help="Terrain used during search.")
parser.add_argument("--seed", type=int, default=240704, help="Environment seed.")
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
parser.add_argument("--no-print-report", action="store_true", help="Write the report without printing full JSON.")
parser.add_argument(
    "--disable-actual-cad-visuals",
    action="store_true",
    help="Disable the real CAD visuals. Mostly useful for parser/debug smoke tests.",
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
from domino_cad_linkage_env import (  # noqa: E402
    DominoCadLinkageEnv,
    DominoCadLinkageEnvCfg,
    projected_gravity_from_quat,
)


def rounded(values, decimals: int = 6):
    return np.round(np.asarray(values, dtype=np.float64), decimals).tolist()


def parse_values(raw: str) -> list[float]:
    values = [float(item.strip()) for item in str(raw).replace(";", ",").split(",") if item.strip()]
    if not values:
        raise ValueError("At least one candidate value is required.")
    return [max(-1.0, min(1.0, value)) for value in values]


def parse_active_channels(raw: str) -> list[int]:
    value = str(raw or "").strip().lower()
    if value in {"", "all"}:
        return list(range(EXPECTED_ACTION_COUNT))
    if value == "linkage":
        return [
            index
            for index, name in enumerate(ACTION_JOINT_NAMES)
            if "lower_linkage" in name or "upper_pitch" in name
        ]
    indexes = [int(item.strip()) for item in value.replace(";", ",").split(",") if item.strip()]
    for index in indexes:
        if index < 0 or index >= EXPECTED_ACTION_COUNT:
            raise ValueError(f"Action channel index out of range: {index}")
    return indexes


def parse_action_row(raw: str) -> np.ndarray:
    value = str(raw or "").strip()
    if not value:
        return np.zeros(EXPECTED_ACTION_COUNT, dtype=np.float32)
    rows = [float(item.strip()) for item in value.replace(";", ",").split(",") if item.strip()]
    if len(rows) != EXPECTED_ACTION_COUNT:
        raise ValueError(f"Expected {EXPECTED_ACTION_COUNT} initial actions, found {len(rows)}.")
    return np.asarray([max(-1.0, min(1.0, item)) for item in rows], dtype=np.float32)


def body_tilt_deg(env: DominoCadLinkageEnv) -> float:
    _, orientation, _, _ = env._body_reference_state(0)
    projected_gravity = projected_gravity_from_quat(orientation)
    tilt_rad = math.acos(max(-1.0, min(1.0, -float(projected_gravity[2]))))
    return math.degrees(tilt_rad)


def rigid_body_pose_report(env: DominoCadLinkageEnv) -> dict[str, dict[str, list[float]]]:
    states = env._capture_rigid_body_states(0)
    return {
        name: {
            "position_m": rounded(state["position"]),
            "orientation_wxyz": rounded(state["orientation"]),
        }
        for name, state in sorted(states.items())
    }


def step_candidate(env: DominoCadLinkageEnv, action_row: np.ndarray, settle_steps: int) -> dict[str, object]:
    env.reset()
    actions = torch.tensor(action_row.reshape(1, EXPECTED_ACTION_COUNT), dtype=torch.float32, device=env.device)
    done_count = 0
    terminated_count = 0
    timeout_count = 0
    finite = True
    for _ in range(int(settle_steps)):
        observations, rewards, terminated, truncated, _ = env.step(actions)
        done = torch.logical_or(terminated, truncated)
        done_count += int(torch.count_nonzero(done).detach().cpu())
        terminated_count += int(torch.count_nonzero(terminated).detach().cpu())
        timeout_count += int(torch.count_nonzero(truncated).detach().cpu())
        finite = finite and bool(torch.isfinite(rewards).all())
        if isinstance(observations, dict) and "policy" in observations:
            finite = finite and bool(torch.isfinite(observations["policy"]).all())

    visual_feet = env._actual_cad_visual_foot_positions(0, rendered=True)
    if visual_feet.shape[0] != EXPECTED_ACTION_COUNT // 3:
        visual_feet = np.zeros((0, 3), dtype=np.float32)
    if visual_feet.size:
        z_values = visual_feet[:, 2].astype(np.float64)
        z_spread = float(np.max(z_values) - np.min(z_values))
        mean_abs_target_error = float(np.mean(np.abs(z_values - float(args_cli.target_foot_z_m))))
    else:
        z_values = np.zeros(0, dtype=np.float64)
        z_spread = 999.0
        mean_abs_target_error = 999.0

    stance_report = env.leg_start_stance_report()
    stance_env = (stance_report.get("envs") or [{}])[0]
    visual_reaches = [
        float((leg.get("rendered_visual_endpoint") or {}).get("planar_reach_m"))
        for leg in stance_env.get("legs", [])
        if (leg.get("rendered_visual_endpoint") or {}).get("planar_reach_m") is not None
    ]
    support_reaches = [
        float((leg.get("support_endpoint") or {}).get("planar_reach_m"))
        for leg in stance_env.get("legs", [])
        if (leg.get("support_endpoint") or {}).get("planar_reach_m") is not None
    ]
    if visual_reaches:
        visual_reach_spread = float(max(visual_reaches) - min(visual_reaches))
        visual_mean_abs_reach_error = float(
            np.mean(np.abs(np.asarray(visual_reaches, dtype=np.float64) - float(args_cli.target_planar_reach_m)))
        )
        max_visual_reach = float(max(visual_reaches))
    else:
        visual_reach_spread = 999.0
        visual_mean_abs_reach_error = 999.0
        max_visual_reach = 999.0
    support_reach_spread = float(max(support_reaches) - min(support_reaches)) if support_reaches else 999.0
    straight_leg_penalty = max(0.0, max_visual_reach - 0.12)

    joint_report = env.joint_separation_report()
    max_joint_sep = float(joint_report.get("max_separation_m", 999.0))
    tilt = body_tilt_deg(env)
    body_position, body_orientation, body_linear_velocity, body_angular_velocity = env._body_reference_state(0)
    score = (
        z_spread
        + (0.35 * mean_abs_target_error)
        + (2.0 * visual_reach_spread)
        + (0.8 * support_reach_spread)
        + (0.6 * visual_mean_abs_reach_error)
        + (4.0 * straight_leg_penalty)
        + (5.0 * max_joint_sep)
        + (0.001 * tilt)
        + (10.0 * float(done_count))
        + (100.0 if not finite else 0.0)
    )
    return {
        "score": round(float(score), 6),
        "done_count": int(done_count),
        "terminated_count": int(terminated_count),
        "timeout_count": int(timeout_count),
        "finite": bool(finite),
        "max_joint_separation_m": round(max_joint_sep, 6),
        "joint_separation": joint_report,
        "body_tilt_deg": round(float(tilt), 6),
        "body_reference": {
            "position_m": rounded(body_position),
            "orientation_wxyz": rounded(body_orientation),
            "linear_velocity_m_s": rounded(body_linear_velocity),
            "angular_velocity_rad_s": rounded(body_angular_velocity),
        },
        "visual_foot_z_m": [round(float(value), 6) for value in z_values.tolist()],
        "visual_foot_height_spread_m": round(float(z_spread), 6),
        "mean_abs_target_foot_z_error_m": round(float(mean_abs_target_error), 6),
        "visual_foot_positions_m": rounded(visual_feet),
        "target_planar_reach_m": round(float(args_cli.target_planar_reach_m), 6),
        "visual_planar_reach_m": [round(float(value), 6) for value in visual_reaches],
        "support_planar_reach_m": [round(float(value), 6) for value in support_reaches],
        "visual_planar_reach_spread_m": round(float(visual_reach_spread), 6),
        "support_planar_reach_spread_m": round(float(support_reach_spread), 6),
        "mean_abs_target_planar_reach_error_m": round(float(visual_mean_abs_reach_error), 6),
        "max_visual_planar_reach_m": round(float(max_visual_reach), 6),
        "straight_leg_penalty_m": round(float(straight_leg_penalty), 6),
        "leg_start_stance": stance_report,
    }


def candidate_result(env: DominoCadLinkageEnv, action_row: np.ndarray, settle_steps: int, note: str) -> dict[str, object]:
    metrics = step_candidate(env, action_row, settle_steps)
    metrics["note"] = note
    metrics["action_values"] = [round(float(value), 6) for value in action_row.tolist()]
    metrics["action_by_channel"] = [
        {"index": index, "name": name, "value": round(float(action_row[index]), 6)}
        for index, name in enumerate(ACTION_JOINT_NAMES)
    ]
    return metrics


def main() -> None:
    cfg = DominoCadLinkageEnvCfg()
    cfg.scene.num_envs = 1
    cfg.sim.device = args_cli.device
    cfg.seed = int(args_cli.seed)
    cfg.include_actual_cad_visuals = not bool(args_cli.disable_actual_cad_visuals)
    cfg.fixed_base = bool(args_cli.fixed_base)
    cfg.enable_gravity = not bool(args_cli.disable_gravity)
    cfg.closure_model = str(args_cli.closure_model)
    if args_cli.floating_height_m is not None:
        cfg.floating_height_m = float(args_cli.floating_height_m)
    cfg.action_scale_deg = float(args_cli.action_scale_deg)
    cfg.servo_target_rate_limit_deg_s = float(args_cli.servo_target_rate_limit_deg_s)
    cfg.foot_contact_mode = str(args_cli.foot_collision_mode).replace("-", "_")
    cfg.use_actual_cad_foot_collision = str(args_cli.foot_collision_mode) == "actual-cad-visual-bottom"
    cfg.terrain_type = str(args_cli.terrain_type)
    cfg.max_tilt_deg = 180.0
    cfg.min_height_m = -10.0
    cfg.episode_length_s = max(6.0, (int(args_cli.settle_steps) + 20) * 0.02)

    env = DominoCadLinkageEnv(cfg)
    observations, _ = env.reset()
    action_dim = gym.spaces.flatdim(env.single_action_space)
    if action_dim != EXPECTED_ACTION_COUNT:
        raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} actions, found {action_dim}.")
    if observations["policy"].shape[0] != 1:
        raise RuntimeError("Stance calibration expects exactly one environment.")

    values = parse_values(args_cli.candidate_values)
    active_channels = parse_active_channels(args_cli.active_channels)
    current = parse_action_row(args_cli.initial_actions)
    initial_note = "initial_custom_action" if str(args_cli.initial_actions or "").strip() else "initial_zero_action"
    history = [candidate_result(env, current.copy(), int(args_cli.settle_steps), initial_note)]
    best = history[0]

    for iteration in range(int(args_cli.iterations)):
        improved = False
        if str(args_cli.search_mode) == "leg-pairs":
            name_to_index = {name: index for index, name in enumerate(ACTION_JOINT_NAMES)}
            channel_groups = []
            for leg in per_leg_action_layout():
                lower_name, upper_name = leg["linkage_drives"]
                pair = (name_to_index[lower_name], name_to_index[upper_name])
                if all(channel_index in active_channels for channel_index in pair):
                    channel_groups.append((str(leg["leg_id"]), pair))

            for leg_id, (lower_index, upper_index) in channel_groups:
                pair_results = []
                for lower_value in values:
                    for upper_value in values:
                        candidate = current.copy()
                        candidate[lower_index] = float(lower_value)
                        candidate[upper_index] = float(upper_value)
                        result = candidate_result(
                            env,
                            candidate,
                            int(args_cli.settle_steps),
                            (
                                f"iteration_{iteration}_{leg_id}_pair_"
                                f"lower_{lower_value:g}_upper_{upper_value:g}"
                            ),
                        )
                        pair_results.append(result)
                        history.append(result)
                pair_best = min(pair_results, key=lambda row: float(row["score"]))
                if float(pair_best["score"]) < float(best["score"]):
                    current = np.asarray(pair_best["action_values"], dtype=np.float32)
                    best = pair_best
                    improved = True
        else:
            for channel_index in active_channels:
                channel_results = []
                for value in values:
                    candidate = current.copy()
                    candidate[channel_index] = float(value)
                    result = candidate_result(
                        env,
                        candidate,
                        int(args_cli.settle_steps),
                        f"iteration_{iteration}_channel_{channel_index}_value_{value:g}",
                    )
                    channel_results.append(result)
                    history.append(result)
                channel_best = min(channel_results, key=lambda row: float(row["score"]))
                if float(channel_best["score"]) < float(best["score"]):
                    current = np.asarray(channel_best["action_values"], dtype=np.float32)
                    best = channel_best
                    improved = True
        if not improved:
            break

    final_check = candidate_result(env, current.copy(), int(args_cli.settle_steps), "final_best_recheck")
    if float(final_check["score"]) <= float(best["score"]) + 1.0e-6:
        best = final_check
    history.append(final_check)
    best_by_height_spread = min(history, key=lambda row: float(row["visual_foot_height_spread_m"]))
    final_pose_body_states = rigid_body_pose_report(env)

    report = {
        "status": "passed",
        "settle_steps": int(args_cli.settle_steps),
        "iterations_requested": int(args_cli.iterations),
        "candidate_values": values,
        "active_channels": active_channels,
        "search_mode": str(args_cli.search_mode),
        "target_foot_z_m": float(args_cli.target_foot_z_m),
        "target_planar_reach_m": float(args_cli.target_planar_reach_m),
        "action_scale_deg": float(cfg.action_scale_deg),
        "servo_target_rate_limit_deg_s": float(cfg.servo_target_rate_limit_deg_s),
        "foot_collision_mode": str(args_cli.foot_collision_mode),
        "fixed_base": bool(cfg.fixed_base),
        "enable_gravity": bool(cfg.enable_gravity),
        "closure_model": str(cfg.closure_model),
        "floating_height_m": float(cfg.floating_height_m),
        "terrain_type": str(args_cli.terrain_type),
        "action_names": ACTION_JOINT_NAMES,
        "per_leg_action_layout": per_leg_action_layout(),
        "best": best,
        "best_by_height_spread": best_by_height_spread,
        "startup_actions_arg": ",".join(f"{float(value):.6f}" for value in best["action_values"]),
        "startup_actions_arg_best_by_height_spread": ",".join(
            f"{float(value):.6f}" for value in best_by_height_spread["action_values"]
        ),
        "final_pose_body_states": final_pose_body_states,
        "history": history,
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
