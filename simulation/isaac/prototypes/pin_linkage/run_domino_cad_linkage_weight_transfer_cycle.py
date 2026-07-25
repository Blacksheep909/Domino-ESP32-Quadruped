"""Verify all four Domino weight-transfer commands in one continuous simulation."""

from __future__ import annotations

import argparse
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


parser = argparse.ArgumentParser(description="Run a continuous four-foot Domino weight-transfer cycle.")
parser.add_argument("--leg-order", default="1,3,0,2", help="Comma-separated leg order for the cycle.")
parser.add_argument("--cycles", type=int, default=1, help="Number of complete four-foot cycles.")
parser.add_argument("--seed", type=int, default=240727, help="Deterministic Isaac seed.")
parser.add_argument("--settle-steps", type=int, default=120, help="Initial neutral settle.")
parser.add_argument("--transfer-steps", type=int, default=60, help="Weight-transfer ramp per foot.")
parser.add_argument("--lift-steps", type=int, default=60, help="Lift ramp per foot.")
parser.add_argument("--hold-steps", type=int, default=20, help="Lift hold per foot.")
parser.add_argument("--release-steps", type=int, default=40, help="Ramp from lift back to the transfer pose.")
parser.add_argument("--return-steps", type=int, default=80, help="Ramp from transfer back to neutral.")
parser.add_argument("--neutral-steps", type=int, default=80, help="Neutral recovery between feet.")
parser.add_argument(
    "--transition-mode",
    choices=["neutral", "direct"],
    default="neutral",
    help="Return through neutral or transition directly from one lift to the next support pose.",
)
parser.add_argument(
    "--skip-transfer-legs",
    default="",
    help="Comma-separated destination legs that transition directly to their final lift pose.",
)
parser.add_argument("--metric-stride", type=int, default=2, help="Physics steps between detailed samples.")
parser.add_argument("--min-target-clearance-m", type=float, default=0.012, help="Required target CAD-foot clearance.")
parser.add_argument("--min-body-relative-lift-m", type=float, default=0.008, help="Required target lift in body coordinates.")
parser.add_argument("--support-clearance-m", type=float, default=0.008, help="Maximum clearance counted as support contact.")
parser.add_argument("--min-support-feet", type=int, default=2, choices=range(1, 4), help="Support feet required at peak lift.")
parser.add_argument("--max-stable-tilt-deg", type=float, default=30.0, help="Maximum body tilt over the cycle.")
parser.add_argument("--max-stable-joint-separation-m", type=float, default=0.001, help="Maximum loop-pin error.")
parser.add_argument("--min-stable-body-height-m", type=float, default=0.22, help="Minimum body-reference height.")
parser.add_argument("--terrain-type", choices=["flat", "stairs"], default="flat", help="Cycle terrain.")
parser.add_argument("--report-path", default="", help="Optional JSON report path.")
parser.add_argument("--disable-actual-cad-visuals", action="store_true", help="Disable STL visuals for headless diagnostics.")
parser.add_argument("--allow-proxy-visuals", action="store_true", help="Permit a visible proxy-only diagnostic.")
parser.add_argument("--hold-open", action="store_true", help="Keep a visible run open after the cycle.")
parser.add_argument("--hold-open-exit-after-frames", type=int, default=0, help="Bound a visible hold-open; zero is unbounded.")
parser.add_argument("--no-print-report", action="store_true", help="Suppress full JSON output.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if not bool(args_cli.headless) and bool(args_cli.disable_actual_cad_visuals) and not bool(args_cli.allow_proxy_visuals):
    raise SystemExit("Visible cycle verification requires the actual exported Domino CAD visuals.")

os.environ.setdefault("WARP_CACHE_PATH", str((REPO_ROOT / "simulation" / "isaac" / "out" / "warp_cache").resolve()))

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


COMMAND_PATHS = {
    0: REPO_ROOT / "simulation" / "isaac" / "config" / "domino_front_right_weight_transfer.json",
    1: REPO_ROOT / "simulation" / "isaac" / "config" / "domino_front_left_weight_transfer.json",
    2: REPO_ROOT / "simulation" / "isaac" / "config" / "domino_rear_left_weight_transfer.json",
    3: REPO_ROOT / "simulation" / "isaac" / "config" / "domino_rear_right_weight_transfer.json",
}


def parse_leg_order(raw: str) -> list[int]:
    order = [int(item.strip()) for item in str(raw).replace(";", ",").split(",") if item.strip()]
    if sorted(order) != list(range(4)):
        raise ValueError("Leg order must contain each index 0, 1, 2, and 3 exactly once.")
    return order


def parse_leg_set(raw: str) -> set[int]:
    indexes = {int(item.strip()) for item in str(raw).replace(";", ",").split(",") if item.strip()}
    if any(index not in range(4) for index in indexes):
        raise ValueError("Skip-transfer leg indexes must be between 0 and 3.")
    return indexes


def load_commands() -> dict[int, dict[str, object]]:
    commands: dict[int, dict[str, object]] = {}
    for leg_index, path in COMMAND_PATHS.items():
        if not path.exists():
            raise FileNotFoundError(f"Missing verified weight-transfer command: {path.relative_to(REPO_ROOT)}")
        command = json.loads(path.read_text(encoding="utf-8"))
        if int(command.get("target_leg_index", -1)) != leg_index:
            raise ValueError(f"Weight-transfer command leg mismatch in {path.relative_to(REPO_ROOT)}")
        for key in ("transfer_actions", "final_actions"):
            values = np.asarray(command.get(key), dtype=np.float32).reshape(-1)
            if values.shape != (EXPECTED_ACTION_COUNT,):
                raise ValueError(f"Expected {EXPECTED_ACTION_COUNT} {key} values in {path.relative_to(REPO_ROOT)}")
            command[key] = values
        commands[leg_index] = command
    scales = {float(command["action_scale_deg"]) for command in commands.values()}
    rates = {float(command["servo_target_rate_limit_deg_s"]) for command in commands.values()}
    if len(scales) != 1 or len(rates) != 1:
        raise ValueError("All four commands must use one action scale and servo slew rate.")
    return commands


def body_tilt_deg(orientation: np.ndarray) -> float:
    gravity = projected_gravity_from_quat(orientation)
    return math.degrees(math.acos(max(-1.0, min(1.0, -float(gravity[2])))))


def body_relative_visual_feet(env: DominoCadLinkageEnv) -> np.ndarray:
    body_position, body_orientation, _, _ = env._body_reference_state(0)
    feet = env._actual_cad_visual_foot_positions(0, rendered=True)
    return ((feet - body_position.reshape(1, 3)) @ quat_wxyz_to_rotation_matrix(body_orientation)).astype(np.float32)


def visual_clearances(env: DominoCadLinkageEnv) -> np.ndarray:
    feet = env._actual_cad_visual_foot_positions(0, rendered=True)
    return (feet[:, 2] - env._terrain_heights_np(feet, env_index=0)).astype(np.float32)


def physics_step(env: DominoCadLinkageEnv, actions: torch.Tensor, render: bool) -> None:
    env._pre_physics_step(actions.to(env.device))
    for _ in range(env.cfg.decimation):
        env._sim_step_counter += 1
        env._apply_action()
        env.scene.write_data_to_sim()
        env.sim.step(render=False)
        if render and env._sim_step_counter % env.cfg.sim.render_interval == 0:
            env.sim.render()
        env.scene.update(dt=env.physics_dt)
    env.episode_length_buf += 1
    env.common_step_counter += 1


def main() -> None:
    leg_order = parse_leg_order(args_cli.leg_order)
    skip_transfer_legs = parse_leg_set(args_cli.skip_transfer_legs)
    commands = load_commands()
    action_scale_deg = float(commands[0]["action_scale_deg"])
    servo_rate = float(commands[0]["servo_target_rate_limit_deg_s"])

    cfg = DominoCadLinkageEnvCfg()
    cfg.seed = int(args_cli.seed)
    cfg.scene.num_envs = 1
    cfg.sim.device = args_cli.device
    cfg.include_actual_cad_visuals = not bool(args_cli.disable_actual_cad_visuals)
    cfg.hide_proxy_visuals_when_actual_cad = True
    cfg.fixed_base = False
    cfg.enable_gravity = True
    cfg.closure_model = "passive"
    cfg.foot_contact_mode = "actual_cad_visual_bottom"
    cfg.use_actual_cad_foot_collision = True
    cfg.terrain_type = str(args_cli.terrain_type)
    cfg.action_scale_deg = action_scale_deg
    cfg.servo_target_rate_limit_deg_s = servo_rate
    cfg.min_height_m = -10.0
    cfg.max_tilt_deg = 180.0
    cfg.episode_length_s = 120.0
    if not bool(args_cli.headless):
        cfg.viewer.eye = (0.90, -0.75, 0.42)
        cfg.viewer.lookat = (0.10, 0.06, 0.16)
        cfg.viewer.origin_type = "world"

    env = DominoCadLinkageEnv(cfg)
    observations, _ = env.reset()
    if gym.spaces.flatdim(env.single_action_space) != EXPECTED_ACTION_COUNT:
        raise RuntimeError("Domino cycle verification requires the 12-action contract.")
    if observations["policy"].shape[0] != 1:
        raise RuntimeError("Cycle verification must use exactly one robot.")

    render = not bool(args_cli.headless)
    zero_np = np.zeros((1, EXPECTED_ACTION_COUNT), dtype=np.float32)
    zero = torch.zeros((1, EXPECTED_ACTION_COUNT), dtype=torch.float32, device=env.device)
    for _ in range(max(int(args_cli.settle_steps), 0)):
        physics_step(env, zero, render)

    startup_stance = env.leg_start_stance_report()
    startup_body_position, startup_body_orientation, _, _ = env._body_reference_state(0)
    global_max_tilt = body_tilt_deg(startup_body_orientation)
    global_min_height = float(startup_body_position[2] - env._env_origins_np[0][2])
    global_max_pin = 0.0
    finite = True
    phase_rows: list[dict[str, object]] = []
    current_np = zero_np.copy()
    stride = max(int(args_cli.metric_stride), 1)
    sample_counter = 0

    for cycle_index in range(max(int(args_cli.cycles), 1)):
        for leg_index in leg_order:
            command = commands[leg_index]
            transfer_np = np.asarray(command["transfer_actions"], dtype=np.float32).reshape(1, -1)
            final_np = np.asarray(command["final_actions"], dtype=np.float32).reshape(1, -1)
            initial_body_position, _, _, _ = env._body_reference_state(0)
            initial_relative_feet = body_relative_visual_feet(env)
            initial_clearances = visual_clearances(env)
            target_peak_clearance = float(initial_clearances[leg_index])
            target_peak_relative_lift = 0.0
            clearances_at_peak = initial_clearances.astype(np.float64).copy()
            tilt_at_peak = global_max_tilt
            height_at_peak = global_min_height
            phase_max_tilt = 0.0
            phase_min_height = float("inf")
            phase_max_pin = 0.0

            def sample() -> None:
                nonlocal finite, global_max_tilt, global_min_height, global_max_pin
                nonlocal target_peak_clearance, target_peak_relative_lift, clearances_at_peak
                nonlocal tilt_at_peak, height_at_peak, phase_max_tilt, phase_min_height, phase_max_pin
                body_position, body_orientation, linear_velocity, angular_velocity = env._body_reference_state(0)
                if not all(
                    np.isfinite(values).all()
                    for values in (body_position, body_orientation, linear_velocity, angular_velocity)
                ):
                    finite = False
                    return
                body_height = float(body_position[2] - env._env_origins_np[0][2])
                tilt = body_tilt_deg(body_orientation)
                relative_lift = float(body_relative_visual_feet(env)[leg_index, 2] - initial_relative_feet[leg_index, 2])
                clearances = visual_clearances(env).astype(np.float64)
                target_peak_relative_lift = max(target_peak_relative_lift, relative_lift)
                if float(clearances[leg_index]) > target_peak_clearance:
                    target_peak_clearance = float(clearances[leg_index])
                    clearances_at_peak = clearances
                    tilt_at_peak = tilt
                    height_at_peak = body_height
                joint_rows = env._joint_separation_rows(0)
                pin = max((float(row["separation_m"]) for row in joint_rows), default=0.0)
                phase_max_tilt = max(phase_max_tilt, tilt)
                phase_min_height = min(phase_min_height, body_height)
                phase_max_pin = max(phase_max_pin, pin)
                global_max_tilt = max(global_max_tilt, tilt)
                global_min_height = min(global_min_height, body_height)
                global_max_pin = max(global_max_pin, pin)

            def run_ramp(start_np: np.ndarray, end_np: np.ndarray, steps: int) -> None:
                nonlocal sample_counter, current_np
                for step_index in range(max(int(steps), 0)):
                    alpha = float(step_index + 1) / float(max(int(steps), 1))
                    current_np = start_np + alpha * (end_np - start_np)
                    physics_step(env, torch.tensor(current_np, dtype=torch.float32, device=env.device), render)
                    sample_counter += 1
                    if sample_counter % stride == 0 or step_index + 1 == int(steps):
                        sample()

            if leg_index in skip_transfer_legs and np.any(np.abs(current_np) > 1.0e-9):
                run_ramp(
                    current_np.copy(),
                    final_np,
                    int(args_cli.transfer_steps) + int(args_cli.lift_steps),
                )
            else:
                run_ramp(current_np.copy(), transfer_np, int(args_cli.transfer_steps))
                run_ramp(current_np.copy(), final_np, int(args_cli.lift_steps))
            run_ramp(current_np.copy(), current_np.copy(), int(args_cli.hold_steps))

            stance_legs = [index for index in range(4) if index != leg_index]
            support_count = int(
                np.count_nonzero(clearances_at_peak[stance_legs] <= float(args_cli.support_clearance_m))
            )
            lift_passed = (
                finite
                and target_peak_clearance >= float(args_cli.min_target_clearance_m)
                and target_peak_relative_lift >= float(args_cli.min_body_relative_lift_m)
                and support_count >= int(args_cli.min_support_feet)
                and phase_max_tilt <= float(args_cli.max_stable_tilt_deg)
                and phase_min_height >= float(args_cli.min_stable_body_height_m)
                and phase_max_pin <= float(args_cli.max_stable_joint_separation_m)
            )
            phase_row = {
                "cycle_index": cycle_index,
                "target_leg_index": leg_index,
                "target_leg": per_leg_action_layout()[leg_index],
                "lift_passed": bool(lift_passed),
                "target_peak_clearance_m": round(target_peak_clearance, 6),
                "target_clearance_lift_m": round(target_peak_clearance - float(initial_clearances[leg_index]), 6),
                "target_peak_body_relative_lift_m": round(target_peak_relative_lift, 6),
                "support_count_at_peak": support_count,
                "clearances_at_target_peak_m": np.round(clearances_at_peak, 6).tolist(),
                "tilt_at_target_peak_deg": round(tilt_at_peak, 6),
                "height_at_target_peak_m": round(height_at_peak, 6),
                "lift_max_tilt_deg": round(phase_max_tilt, 6),
                "lift_min_body_height_m": round(phase_min_height, 6),
                "lift_max_joint_separation_m": round(phase_max_pin, 6),
                "body_displacement_at_lift_m": np.round(
                    env._body_reference_state(0)[0] - initial_body_position, 6
                ).tolist(),
            }

            if str(args_cli.transition_mode) == "neutral":
                run_ramp(current_np.copy(), transfer_np, int(args_cli.release_steps))
                run_ramp(current_np.copy(), zero_np, int(args_cli.return_steps))
                run_ramp(current_np.copy(), zero_np, int(args_cli.neutral_steps))
            recovery_position, recovery_orientation, recovery_linear_velocity, recovery_angular_velocity = (
                env._body_reference_state(0)
            )
            recovery_tilt = body_tilt_deg(recovery_orientation)
            recovery_height = float(recovery_position[2] - env._env_origins_np[0][2])
            recovery_speed = float(np.linalg.norm(recovery_linear_velocity))
            recovery_angular_speed = float(np.linalg.norm(recovery_angular_velocity))
            recovery_passed = (
                finite
                and phase_max_tilt <= float(args_cli.max_stable_tilt_deg)
                and phase_min_height >= float(args_cli.min_stable_body_height_m)
                and phase_max_pin <= float(args_cli.max_stable_joint_separation_m)
                and recovery_tilt <= float(args_cli.max_stable_tilt_deg)
                and recovery_height >= float(args_cli.min_stable_body_height_m)
            )
            phase_row.update(
                {
                    "recovery_passed": bool(recovery_passed),
                    "passed": bool(lift_passed and recovery_passed),
                    "sequence_max_tilt_deg": round(phase_max_tilt, 6),
                    "sequence_min_body_height_m": round(phase_min_height, 6),
                    "sequence_max_joint_separation_m": round(phase_max_pin, 6),
                    "recovery_tilt_deg": round(recovery_tilt, 6),
                    "recovery_body_height_m": round(recovery_height, 6),
                    "recovery_linear_speed_m_s": round(recovery_speed, 6),
                    "recovery_angular_speed_rad_s": round(recovery_angular_speed, 6),
                    "body_displacement_after_recovery_m": np.round(
                        recovery_position - initial_body_position, 6
                    ).tolist(),
                    "clearances_after_recovery_m": np.round(visual_clearances(env), 6).tolist(),
                }
            )
            phase_rows.append(phase_row)

    final_body_position, final_body_orientation, _, _ = env._body_reference_state(0)
    final_relative_feet = body_relative_visual_feet(env)
    global_mechanics_passed = (
        finite
        and global_max_tilt <= float(args_cli.max_stable_tilt_deg)
        and global_min_height >= float(args_cli.min_stable_body_height_m)
        and global_max_pin <= float(args_cli.max_stable_joint_separation_m)
    )
    report = {
        "status": "passed" if global_mechanics_passed and all(bool(row["passed"]) for row in phase_rows) else "failed",
        "seed": int(args_cli.seed),
        "cycles": max(int(args_cli.cycles), 1),
        "leg_order": leg_order,
        "action_names": ACTION_JOINT_NAMES,
        "action_scale_deg": action_scale_deg,
        "servo_target_rate_limit_deg_s": servo_rate,
        "terrain_type": str(args_cli.terrain_type),
        "transition_mode": str(args_cli.transition_mode),
        "skip_transfer_legs": sorted(skip_transfer_legs),
        "actual_cad_visual": bool(cfg.include_actual_cad_visuals),
        "visual_fidelity": env._linkage.get("visual_fidelity"),
        "actual_cad_visuals": env._linkage.get("actual_cad_visuals"),
        "startup_leg_stance": startup_stance,
        "thresholds": {
            "min_target_clearance_m": float(args_cli.min_target_clearance_m),
            "min_body_relative_lift_m": float(args_cli.min_body_relative_lift_m),
            "support_clearance_m": float(args_cli.support_clearance_m),
            "min_support_feet": int(args_cli.min_support_feet),
            "max_stable_tilt_deg": float(args_cli.max_stable_tilt_deg),
            "max_stable_joint_separation_m": float(args_cli.max_stable_joint_separation_m),
            "min_stable_body_height_m": float(args_cli.min_stable_body_height_m),
        },
        "global_mechanics_passed": bool(global_mechanics_passed),
        "finite": bool(finite),
        "global_max_tilt_deg": round(global_max_tilt, 6),
        "global_min_body_height_m": round(global_min_height, 6),
        "global_max_joint_separation_m": round(global_max_pin, 6),
        "final_body_displacement_m": np.round(final_body_position - startup_body_position, 6).tolist(),
        "final_body_orientation_wxyz": np.round(final_body_orientation, 6).tolist(),
        "final_body_relative_cad_feet_m": np.round(final_relative_feet, 6).tolist(),
        "phase_rows": phase_rows,
    }
    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if not bool(args_cli.no_print_report):
        print(json.dumps(report, indent=2), flush=True)

    if bool(args_cli.hold_open) and render:
        frame_count = 0
        while simulation_app.is_running():
            physics_step(env, zero, True)
            frame_count += 1
            if int(args_cli.hold_open_exit_after_frames) > 0 and frame_count >= int(args_cli.hold_open_exit_after_frames):
                break
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if bool(getattr(args_cli, "graceful_close", False)):
            simulation_app.close()
