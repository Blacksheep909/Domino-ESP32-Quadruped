"""Evaluate every Domino four-leg linkage-swing order in parallel."""

from __future__ import annotations

import argparse
from itertools import permutations
import json
import math
import os
from pathlib import Path
import sys

from isaaclab.app import AppLauncher

REPO_ROOT = Path(__file__).resolve().parents[4]
CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--seed", type=int, default=240731)
parser.add_argument("--settle-steps", type=int, default=120)
parser.add_argument("--metric-stride", type=int, default=2)
parser.add_argument("--min-foot-motion-m", type=float, default=0.050)
parser.add_argument("--min-each-linkage-drive-motion-deg", type=float, default=4.0)
parser.add_argument("--max-joint-separation-m", type=float, default=0.001)
parser.add_argument("--min-body-height-m", type=float, default=0.22)
parser.add_argument("--max-body-tilt-deg", type=float, default=30.0)
parser.add_argument("--report-path", default="")
parser.add_argument("--no-print-report", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

os.environ.setdefault(
    "WARP_CACHE_PATH",
    str((REPO_ROOT / "simulation" / "isaac" / "out" / "warp_cache").resolve()),
)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from domino_action_contract import ACTION_JOINT_NAMES, EXPECTED_ACTION_COUNT  # noqa: E402
from domino_cad_linkage_env import (  # noqa: E402
    DominoCadLinkageEnv,
    DominoCadLinkageEnvCfg,
    projected_gravity_from_quat,
)
from domino_linkage_motion import (  # noqa: E402
    create_foot_endpoint_motion_tracker,
    create_linkage_motion_tracker,
    foot_endpoint_motion_report,
    linkage_motion_report,
    update_foot_endpoint_motion_tracker,
    update_linkage_motion_tracker,
)


LEG_LABELS = ["front_right", "front_left", "rear_left", "rear_right"]
PHASES = (
    ("transfer", 60, "zero", "transfer"),
    ("lift", 60, "transfer", "lift"),
    ("lift_hold", 20, "lift", "lift"),
    ("sweep", 70, "lift", "sweep"),
    ("sweep_hold", 20, "sweep", "sweep"),
    ("sweep_return", 70, "sweep", "lift"),
    ("place", 50, "lift", "transfer"),
    ("return", 80, "transfer", "zero"),
    ("neutral", 160, "zero", "zero"),
)


def load_commands() -> tuple[dict[int, dict[str, np.ndarray]], float, float]:
    commands: dict[int, dict[str, np.ndarray]] = {}
    action_scales = set()
    servo_rates = set()
    for leg_index, label in enumerate(LEG_LABELS):
        path = REPO_ROOT / "simulation" / "isaac" / "config" / f"domino_{label}_swing_hipframe.json"
        command = json.loads(path.read_text(encoding="utf-8-sig"))
        if int(command.get("target_leg_index", -1)) != leg_index:
            raise ValueError(f"Target leg mismatch in {path.name}.")
        metrics = command.get("verified_metrics", {})
        if not bool(metrics.get("passed")):
            raise RuntimeError(f"Command is not verified: {path.name}")
        rows = {"zero": np.zeros(EXPECTED_ACTION_COUNT, dtype=np.float32)}
        for source, target in (
            ("transfer_actions", "transfer"),
            ("lift_actions", "lift"),
            ("sweep_actions", "sweep"),
        ):
            row = np.asarray(command[source], dtype=np.float32).reshape(-1)
            if row.shape != (EXPECTED_ACTION_COUNT,):
                raise ValueError(f"{path.name} {source} violates the 12-action contract.")
            rows[target] = np.clip(row, -1.0, 1.0)
        commands[leg_index] = rows
        action_scales.add(float(command["action_scale_deg"]))
        servo_rates.add(float(command["servo_target_rate_limit_deg_s"]))
    if len(action_scales) != 1 or len(servo_rates) != 1:
        raise ValueError("All linkage-swing commands must use one action scale and slew rate.")
    return commands, action_scales.pop(), servo_rates.pop()


def body_tilt_deg(orientation: np.ndarray) -> float:
    gravity = projected_gravity_from_quat(orientation)
    return math.degrees(math.acos(max(-1.0, min(1.0, -float(gravity[2])))))


def fast_physics_step(env: DominoCadLinkageEnv, actions: torch.Tensor) -> None:
    env._pre_physics_step(actions.to(env.device))
    for _ in range(env.cfg.decimation):
        env._sim_step_counter += 1
        env._apply_action()
        env.scene.write_data_to_sim()
        env.sim.step(render=False)
        env.scene.update(dt=env.physics_dt)
    env.episode_length_buf += 1
    env.common_step_counter += 1


def main() -> None:
    orders = list(permutations(range(4)))
    commands, action_scale_deg, servo_rate_deg_s = load_commands()
    cfg = DominoCadLinkageEnvCfg()
    cfg.seed = int(args_cli.seed)
    cfg.scene.num_envs = len(orders)
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
    cfg.action_scale_deg = action_scale_deg
    cfg.servo_target_rate_limit_deg_s = servo_rate_deg_s
    cfg.min_height_m = -10.0
    cfg.max_tilt_deg = 180.0
    cfg.episode_length_s = 120.0

    env = DominoCadLinkageEnv(cfg)
    observations, _ = env.reset()
    if gym.spaces.flatdim(env.single_action_space) != EXPECTED_ACTION_COUNT:
        raise RuntimeError("Domino order search requires the 12-action contract.")
    if observations["policy"].shape[0] != len(orders):
        raise RuntimeError("Order/environment count mismatch.")

    zero = torch.zeros((len(orders), EXPECTED_ACTION_COUNT), dtype=torch.float32, device=env.device)
    for _ in range(max(int(args_cli.settle_steps), 0)):
        fast_physics_step(env, zero)
    env.episode_length_buf[:] = 0

    foot_tracker = create_foot_endpoint_motion_tracker(env)
    drive_tracker = create_linkage_motion_tracker(env)
    max_tilts = np.zeros(len(orders), dtype=np.float64)
    min_heights = np.full(len(orders), np.inf, dtype=np.float64)
    max_pins = np.zeros(len(orders), dtype=np.float64)
    first_threshold_crossing: list[dict[str, object] | None] = [None] * len(orders)
    final_displacements = np.zeros((len(orders), 3), dtype=np.float64)
    initial_positions = np.stack([env._body_reference_state(index)[0] for index in range(len(orders))])
    stride = max(int(args_cli.metric_stride), 1)
    rollout_step = 0

    def sample(leg_position: int, phase_name: str) -> None:
        update_foot_endpoint_motion_tracker(foot_tracker, env)
        update_linkage_motion_tracker(drive_tracker, env)
        for env_index in range(len(orders)):
            position, orientation, linear_velocity, angular_velocity = env._body_reference_state(env_index)
            values = (position, orientation, linear_velocity, angular_velocity)
            finite = all(np.isfinite(value).all() for value in values)
            height = float(position[2] - env._env_origins_np[env_index][2]) if finite else -math.inf
            tilt = body_tilt_deg(orientation) if finite else math.inf
            pin = max(
                (float(row["separation_m"]) for row in env._joint_separation_rows(env_index)),
                default=0.0,
            ) if finite else math.inf
            max_tilts[env_index] = max(max_tilts[env_index], tilt)
            min_heights[env_index] = min(min_heights[env_index], height)
            max_pins[env_index] = max(max_pins[env_index], pin)
            if first_threshold_crossing[env_index] is None and (
                not finite
                or height < float(args_cli.min_body_height_m)
                or tilt > float(args_cli.max_body_tilt_deg)
                or pin > float(args_cli.max_joint_separation_m)
            ):
                first_threshold_crossing[env_index] = {
                    "rollout_step": int(rollout_step),
                    "leg_position": int(leg_position),
                    "leg_index": int(orders[env_index][leg_position]),
                    "phase": phase_name,
                    "body_height_m": round(height, 6),
                    "body_tilt_deg": round(tilt, 6),
                    "max_joint_separation_m": round(pin, 6),
                }

    current = np.zeros((len(orders), EXPECTED_ACTION_COUNT), dtype=np.float32)
    for leg_position in range(4):
        for phase_name, phase_steps, start_key, end_key in PHASES:
            starts = np.stack([commands[order[leg_position]][start_key] for order in orders])
            ends = np.stack([commands[order[leg_position]][end_key] for order in orders])
            for phase_step in range(phase_steps):
                alpha = float(phase_step + 1) / float(max(phase_steps, 1))
                current = starts + alpha * (ends - starts)
                fast_physics_step(
                    env,
                    torch.tensor(current, dtype=torch.float32, device=env.device),
                )
                rollout_step += 1
                if rollout_step % stride == 0 or phase_step + 1 == phase_steps:
                    sample(leg_position, phase_name)

    foot_rows = foot_endpoint_motion_report(foot_tracker)["envs"]
    drive_rows = linkage_motion_report(drive_tracker)["envs"]
    rows = []
    for env_index, order in enumerate(orders):
        final_position, final_orientation, final_linear_velocity, final_angular_velocity = env._body_reference_state(env_index)
        final_displacements[env_index] = final_position - initial_positions[env_index]
        min_foot_motion = float(foot_rows[env_index]["min_each_foot_motion_m"])
        min_drive_motion = float(drive_rows[env_index]["min_each_drive_motion_deg"])
        passed = (
            first_threshold_crossing[env_index] is None
            and min_foot_motion >= float(args_cli.min_foot_motion_m)
            and min_drive_motion >= float(args_cli.min_each_linkage_drive_motion_deg)
        )
        rows.append(
            {
                "order": list(order),
                "order_labels": [LEG_LABELS[index] for index in order],
                "passed": bool(passed),
                "first_threshold_crossing": first_threshold_crossing[env_index],
                "min_body_height_m": round(float(min_heights[env_index]), 6),
                "max_body_tilt_deg": round(float(max_tilts[env_index]), 6),
                "max_joint_separation_m": round(float(max_pins[env_index]), 6),
                "min_each_foot_motion_m": round(min_foot_motion, 6),
                "min_each_linkage_drive_motion_deg": round(min_drive_motion, 6),
                "final_body_displacement_m": np.round(final_displacements[env_index], 6).tolist(),
                "final_body_tilt_deg": round(body_tilt_deg(final_orientation), 6),
                "final_linear_speed_m_s": round(float(np.linalg.norm(final_linear_velocity)), 6),
                "final_angular_speed_rad_s": round(float(np.linalg.norm(final_angular_velocity)), 6),
                "foot_motion": foot_rows[env_index],
                "linkage_drive_motion": drive_rows[env_index],
            }
        )
    rows.sort(
        key=lambda row: (
            bool(row["passed"]),
            -float(row["max_body_tilt_deg"]),
            -float(row["max_joint_separation_m"]),
            float(row["min_each_foot_motion_m"]),
        ),
        reverse=True,
    )
    report = {
        "status": "passed" if any(bool(row["passed"]) for row in rows) else "no_passing_order",
        "seed": int(args_cli.seed),
        "order_count": len(orders),
        "passed_count": sum(bool(row["passed"]) for row in rows),
        "action_names": ACTION_JOINT_NAMES,
        "action_scale_deg": action_scale_deg,
        "servo_target_rate_limit_deg_s": servo_rate_deg_s,
        "steps": rollout_step,
        "thresholds": {
            "min_foot_motion_m": float(args_cli.min_foot_motion_m),
            "min_each_linkage_drive_motion_deg": float(args_cli.min_each_linkage_drive_motion_deg),
            "max_joint_separation_m": float(args_cli.max_joint_separation_m),
            "min_body_height_m": float(args_cli.min_body_height_m),
            "max_body_tilt_deg": float(args_cli.max_body_tilt_deg),
        },
        "best": rows[0],
        "orders": rows,
    }
    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if not args_cli.no_print_report:
        print(json.dumps(report, indent=2), flush=True)
    else:
        print(
            json.dumps(
                {
                    "status": report["status"],
                    "passed_count": report["passed_count"],
                    "best": report["best"],
                },
                indent=2,
            ),
            flush=True,
        )
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if bool(getattr(args_cli, "graceful_close", False)):
            simulation_app.close()
