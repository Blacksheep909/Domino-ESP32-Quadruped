"""Search coordinated 12-servo commands that unload one Domino CAD foot."""

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


parser = argparse.ArgumentParser(description="Search a stable two-stage Domino weight-transfer command.")
parser.add_argument("--leg-index", type=int, required=True, choices=range(4), help="CAD foot to unload.")
parser.add_argument("--population-size", type=int, default=48, help="Parallel candidates per generation.")
parser.add_argument("--generations", type=int, default=6, help="Evolution generations.")
parser.add_argument("--elite-count", type=int, default=8, help="Candidates retained between generations.")
parser.add_argument("--seed", type=int, default=240723, help="Deterministic search and Isaac seed.")
parser.add_argument("--settle-steps", type=int, default=120, help="Neutral settling steps.")
parser.add_argument("--transfer-steps", type=int, default=80, help="Ramp steps for body weight transfer.")
parser.add_argument("--lift-steps", type=int, default=80, help="Ramp steps from transfer to final lift command.")
parser.add_argument("--hold-steps", type=int, default=40, help="Final command hold steps.")
parser.add_argument("--release-steps", type=int, default=0, help="Optional ramp from lift back to transfer.")
parser.add_argument("--return-steps", type=int, default=0, help="Optional ramp from transfer back to neutral.")
parser.add_argument("--neutral-steps", type=int, default=0, help="Optional neutral recovery hold.")
parser.add_argument("--metric-stride", type=int, default=2, help="Physics steps between detailed metric samples.")
parser.add_argument("--action-scale-deg", type=float, default=8.0, help="Physical target offset for action magnitude 1.")
parser.add_argument("--servo-target-rate-limit-deg-s", type=float, default=90.0, help="Servo target slew limit.")
parser.add_argument("--initial-sigma", type=float, default=0.55, help="First-generation mutation standard deviation.")
parser.add_argument("--final-sigma", type=float, default=0.12, help="Last-generation mutation standard deviation.")
parser.add_argument("--initial-candidate-path", default="", help="Optional report or command used to seed refinement.")
parser.add_argument("--initial-candidate-pool-path", default="", help="Optional report whose passing rows form one population.")
parser.add_argument("--prefix-command-order", default="", help="Comma-separated verified command prefix replayed before every candidate.")
parser.add_argument("--min-target-clearance-m", type=float, default=0.012, help="Required actual CAD foot clearance.")
parser.add_argument("--min-body-relative-lift-m", type=float, default=0.008, help="Required target-foot lift in body coordinates.")
parser.add_argument(
    "--min-target-linkage-drive-motion-deg",
    type=float,
    default=6.0,
    help="Required actual rotation from both target lower/upper drives relative to the hip carriage.",
)
parser.add_argument("--support-clearance-m", type=float, default=0.008, help="Maximum clearance counted as support contact.")
parser.add_argument("--min-support-feet", type=int, default=2, choices=range(1, 4), help="Support feet required at peak lift.")
parser.add_argument("--max-stable-tilt-deg", type=float, default=30.0, help="Maximum body tilt during a passing trial.")
parser.add_argument("--max-stable-joint-separation-m", type=float, default=0.001, help="Maximum loop-pin error.")
parser.add_argument("--min-stable-body-height-m", type=float, default=0.22, help="Minimum body-reference height.")
parser.add_argument("--report-path", default="", help="Optional JSON report path.")
parser.add_argument("--command-path", default="", help="Optional JSON path for the best two-stage command.")
parser.add_argument("--no-print-report", action="store_true", help="Suppress full JSON output.")
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
from domino_linkage_motion import (  # noqa: E402
    create_linkage_motion_tracker,
    linkage_motion_report,
    update_linkage_motion_tracker,
)


def body_tilt_deg(orientation: np.ndarray) -> float:
    gravity = projected_gravity_from_quat(orientation)
    return math.degrees(math.acos(max(-1.0, min(1.0, -float(gravity[2])))))


def body_relative_visual_feet(env: DominoCadLinkageEnv, env_index: int) -> np.ndarray:
    body_position, body_orientation, _, _ = env._body_reference_state(env_index)
    feet = env._actual_cad_visual_foot_positions(env_index, rendered=True)
    world_from_body = quat_wxyz_to_rotation_matrix(body_orientation)
    return ((feet - body_position.reshape(1, 3)) @ world_from_body).astype(np.float32)


def visual_clearances(env: DominoCadLinkageEnv, env_index: int) -> np.ndarray:
    feet = env._actual_cad_visual_foot_positions(env_index, rendered=True)
    return (feet[:, 2] - env._terrain_heights_np(feet, env_index=env_index)).astype(np.float32)


def initial_population(rng: np.random.Generator, population_size: int, target_leg: int) -> np.ndarray:
    population = np.zeros((population_size, EXPECTED_ACTION_COUNT), dtype=np.float32)
    target_start = target_leg * 3
    seeds: list[np.ndarray] = []

    for shoulder in (-1.0, 0.0, 1.0):
        for lower, upper in ((1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (-1.0, -1.0)):
            candidate = np.zeros(EXPECTED_ACTION_COUNT, dtype=np.float32)
            candidate[target_start : target_start + 3] = (shoulder, lower, upper)
            seeds.append(candidate)

    # Explicit weight-shift seeds exercise shoulder and linkage support patterns.
    for direction in (-1.0, 1.0):
        candidate = np.zeros(EXPECTED_ACTION_COUNT, dtype=np.float32)
        candidate[target_start : target_start + 3] = (direction, 1.0, -1.0)
        for leg_index in range(4):
            if leg_index == target_leg:
                continue
            start = leg_index * 3
            candidate[start] = -direction
            candidate[start + 1 : start + 3] = (0.45, 0.45)
        seeds.append(candidate)

    for index in range(population_size):
        if index < len(seeds):
            population[index] = seeds[index]
            continue
        candidate = rng.normal(0.0, 0.42, EXPECTED_ACTION_COUNT).astype(np.float32)
        candidate[0::3] = rng.uniform(-1.0, 1.0, 4)
        candidate[target_start + 1 : target_start + 3] = rng.uniform(-1.0, 1.0, 2)
        population[index] = np.clip(candidate, -1.0, 1.0)
    return population


def next_population(
    rng: np.random.Generator,
    elites: np.ndarray,
    population_size: int,
    sigma: float,
    target_leg: int,
) -> np.ndarray:
    population = np.zeros((population_size, EXPECTED_ACTION_COUNT), dtype=np.float32)
    elite_count = min(len(elites), population_size)
    population[:elite_count] = elites[:elite_count]
    target_start = target_leg * 3
    for index in range(elite_count, population_size):
        first = elites[int(rng.integers(0, len(elites)))]
        second = elites[int(rng.integers(0, len(elites)))]
        blend = float(rng.uniform(0.25, 0.75))
        child = (blend * first) + ((1.0 - blend) * second)
        child += rng.normal(0.0, sigma, EXPECTED_ACTION_COUNT)
        # Keep exploring the two target linkage drives even after the support pattern converges.
        child[target_start + 1 : target_start + 3] += rng.normal(0.0, 0.5 * sigma, 2)
        population[index] = np.clip(child, -1.0, 1.0)
    return population


def transfer_actions(final_actions: np.ndarray, target_leg: int) -> np.ndarray:
    transfer = np.asarray(final_actions, dtype=np.float32).copy()
    target_start = target_leg * 3
    transfer[:, target_start + 1 : target_start + 3] = 0.0
    return transfer


def load_initial_candidate(path: str) -> np.ndarray:
    data = json.loads(Path(path).expanduser().resolve().read_text(encoding="utf-8"))
    if isinstance(data.get("best"), dict) and data["best"].get("final_actions") is not None:
        values = data["best"]["final_actions"]
    elif data.get("final_actions") is not None:
        values = data["final_actions"]
    else:
        raise ValueError("Initial candidate JSON must contain best.final_actions or final_actions.")
    candidate = np.asarray(values, dtype=np.float32).reshape(-1)
    if candidate.shape != (EXPECTED_ACTION_COUNT,):
        raise ValueError(f"Expected {EXPECTED_ACTION_COUNT} initial candidate actions, found {candidate.shape}.")
    return np.clip(candidate, -1.0, 1.0)


def load_candidate_pool(path: str) -> np.ndarray:
    data = json.loads(Path(path).expanduser().resolve().read_text(encoding="utf-8"))
    rows = []
    if isinstance(data.get("best"), dict):
        rows.append(data["best"])
    for generation in data.get("generation_reports", []):
        rows.extend(generation.get("top", []))
    candidates = {}
    for row in rows:
        if not bool(row.get("passed")) or row.get("final_actions") is None:
            continue
        candidate = np.asarray(row["final_actions"], dtype=np.float32).reshape(-1)
        if candidate.shape != (EXPECTED_ACTION_COUNT,):
            continue
        candidates[tuple(np.round(candidate, 6).tolist())] = np.clip(candidate, -1.0, 1.0)
    if not candidates:
        raise ValueError("Initial candidate pool report contains no passing final_actions rows.")
    return np.vstack(list(candidates.values())).astype(np.float32)


def parse_prefix_order(raw: str, target_leg: int) -> list[int]:
    order = [int(item.strip()) for item in str(raw).replace(";", ",").split(",") if item.strip()]
    if any(index not in range(4) for index in order) or len(set(order)) != len(order):
        raise ValueError("Prefix command order must contain unique leg indexes between 0 and 3.")
    if target_leg in order:
        raise ValueError("Target leg cannot appear in the prefix command order.")
    return order


def load_verified_command(leg_index: int) -> dict[str, np.ndarray]:
    labels = ["front_right", "front_left", "rear_left", "rear_right"]
    path = REPO_ROOT / "simulation" / "isaac" / "config" / f"domino_{labels[leg_index]}_weight_transfer.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    return {
        "transfer_actions": np.asarray(data["transfer_actions"], dtype=np.float32).reshape(-1),
        "final_actions": np.asarray(data["final_actions"], dtype=np.float32).reshape(-1),
    }


def open_loop_ramp(
    env: DominoCadLinkageEnv,
    start_np: np.ndarray,
    end_np: np.ndarray,
    steps: int,
) -> np.ndarray:
    current = start_np.copy()
    for step_index in range(max(int(steps), 0)):
        alpha = float(step_index + 1) / float(max(int(steps), 1))
        current = start_np + alpha * (end_np - start_np)
        fast_physics_step(env, torch.tensor(current, dtype=torch.float32, device=env.device))
    return current


def fast_physics_step(env: DominoCadLinkageEnv, actions: torch.Tensor) -> None:
    """Advance the authoritative drive/PhysX path without unused RL bookkeeping."""
    env._pre_physics_step(actions.to(env.device))
    for _ in range(env.cfg.decimation):
        env._sim_step_counter += 1
        env._apply_action()
        env.scene.write_data_to_sim()
        env.sim.step(render=False)
        env.scene.update(dt=env.physics_dt)
    env.episode_length_buf += 1
    env.common_step_counter += 1


def evaluate_population(
    env: DominoCadLinkageEnv,
    population: np.ndarray,
    target_leg: int,
    prefix_order: list[int],
    prefix_commands: dict[int, dict[str, np.ndarray]],
) -> list[dict[str, object]]:
    env.reset()
    if hasattr(env, "episode_length_buf"):
        env.episode_length_buf[:] = 0
    population_size = len(population)
    zero = torch.zeros((population_size, EXPECTED_ACTION_COUNT), dtype=torch.float32, device=env.device)
    finite = np.ones(population_size, dtype=bool)
    done_counts = np.zeros(population_size, dtype=np.int64)
    terminated_counts = np.zeros(population_size, dtype=np.int64)

    for _ in range(max(int(args_cli.settle_steps), 0)):
        fast_physics_step(env, zero)
    prefix_current = np.zeros((population_size, EXPECTED_ACTION_COUNT), dtype=np.float32)
    for prefix_leg in prefix_order:
        transfer = np.tile(prefix_commands[prefix_leg]["transfer_actions"], (population_size, 1))
        final = np.tile(prefix_commands[prefix_leg]["final_actions"], (population_size, 1))
        prefix_current = open_loop_ramp(env, prefix_current, transfer, int(args_cli.transfer_steps))
        prefix_current = open_loop_ramp(env, prefix_current, final, int(args_cli.lift_steps))
        prefix_current = open_loop_ramp(env, prefix_current, final, int(args_cli.hold_steps))
        prefix_current = open_loop_ramp(env, prefix_current, transfer, int(args_cli.release_steps))
        prefix_current = open_loop_ramp(env, prefix_current, np.zeros_like(prefix_current), int(args_cli.return_steps))
        prefix_current = open_loop_ramp(env, prefix_current, np.zeros_like(prefix_current), int(args_cli.neutral_steps))
    if hasattr(env, "episode_length_buf"):
        env.episode_length_buf[:] = 0

    initial_body_positions = np.zeros((population_size, 3), dtype=np.float32)
    initial_relative_feet = np.zeros((population_size, 4, 3), dtype=np.float32)
    initial_clearances = np.zeros((population_size, 4), dtype=np.float32)
    for env_index in range(population_size):
        initial_body_positions[env_index] = env._body_reference_state(env_index)[0]
        initial_relative_feet[env_index] = body_relative_visual_feet(env, env_index)
        initial_clearances[env_index] = visual_clearances(env, env_index)
    linkage_motion_tracker = create_linkage_motion_tracker(env)

    target_peak_clearance = initial_clearances[:, target_leg].astype(np.float64)
    target_peak_relative_lift = np.zeros(population_size, dtype=np.float64)
    clearances_at_peak = initial_clearances.astype(np.float64).copy()
    tilt_at_peak = np.zeros(population_size, dtype=np.float64)
    height_at_peak = initial_body_positions[:, 2].astype(np.float64)
    max_tilts = np.zeros(population_size, dtype=np.float64)
    min_heights = np.full(population_size, np.inf, dtype=np.float64)
    max_joint_separation = np.zeros(population_size, dtype=np.float64)
    max_target_body_motion = np.zeros(population_size, dtype=np.float64)

    final_np = np.asarray(population, dtype=np.float32)
    transfer_np = transfer_actions(final_np, target_leg)
    previous_np = np.zeros_like(final_np)
    stride = max(int(args_cli.metric_stride), 1)
    global_step = 0

    def run_phase(start_np: np.ndarray, end_np: np.ndarray, steps: int) -> None:
        nonlocal global_step
        for phase_step in range(max(int(steps), 0)):
            alpha = float(phase_step + 1) / float(max(int(steps), 1))
            action_np = start_np + alpha * (end_np - start_np)
            action_tensor = torch.tensor(action_np, dtype=torch.float32, device=env.device)
            fast_physics_step(env, action_tensor)
            global_step += 1
            if global_step % stride != 0 and phase_step + 1 != int(steps):
                continue
            update_linkage_motion_tracker(linkage_motion_tracker, env)
            for env_index in range(population_size):
                body_position, body_orientation, _, _ = env._body_reference_state(env_index)
                if not np.isfinite(body_position).all() or not np.isfinite(body_orientation).all():
                    finite[env_index] = False
                    continue
                body_height = float(body_position[2] - env._env_origins_np[env_index][2])
                tilt = body_tilt_deg(body_orientation)
                max_tilts[env_index] = max(max_tilts[env_index], tilt)
                min_heights[env_index] = min(min_heights[env_index], body_height)
                relative_feet = body_relative_visual_feet(env, env_index)
                relative_delta = relative_feet[target_leg] - initial_relative_feet[env_index, target_leg]
                relative_lift = float(relative_delta[2])
                target_peak_relative_lift[env_index] = max(target_peak_relative_lift[env_index], relative_lift)
                max_target_body_motion[env_index] = max(
                    max_target_body_motion[env_index],
                    float(np.linalg.norm(relative_delta)),
                )
                clearances = visual_clearances(env, env_index).astype(np.float64)
                if float(clearances[target_leg]) > float(target_peak_clearance[env_index]):
                    target_peak_clearance[env_index] = float(clearances[target_leg])
                    clearances_at_peak[env_index] = clearances
                    tilt_at_peak[env_index] = tilt
                    height_at_peak[env_index] = body_height
                if global_step % (5 * stride) == 0 or phase_step + 1 == int(steps):
                    joint_rows = env._joint_separation_rows(env_index)
                    if joint_rows:
                        max_joint_separation[env_index] = max(
                            max_joint_separation[env_index],
                            max(float(row["separation_m"]) for row in joint_rows),
                        )

    run_phase(previous_np, transfer_np, int(args_cli.transfer_steps))
    run_phase(transfer_np, final_np, int(args_cli.lift_steps))
    run_phase(final_np, final_np, int(args_cli.hold_steps))
    run_phase(final_np, transfer_np, int(args_cli.release_steps))
    run_phase(transfer_np, previous_np, int(args_cli.return_steps))
    run_phase(previous_np, previous_np, int(args_cli.neutral_steps))

    rows: list[dict[str, object]] = []
    measured_linkage_motion = linkage_motion_report(linkage_motion_tracker)
    stance_legs = [index for index in range(4) if index != target_leg]
    for env_index in range(population_size):
        final_body_position, final_body_orientation, final_linear_velocity, final_angular_velocity = (
            env._body_reference_state(env_index)
        )
        body_displacement = final_body_position - initial_body_positions[env_index]
        recovery_height = float(final_body_position[2] - env._env_origins_np[env_index][2])
        recovery_tilt = body_tilt_deg(final_body_orientation)
        recovery_linear_speed = float(np.linalg.norm(final_linear_velocity))
        recovery_angular_speed = float(np.linalg.norm(final_angular_velocity))
        stance_clearances = clearances_at_peak[env_index, stance_legs]
        support_count = int(np.count_nonzero(stance_clearances <= float(args_cli.support_clearance_m)))
        target_drive_rows = measured_linkage_motion["envs"][env_index]["drives"][
            target_leg * 2 : target_leg * 2 + 2
        ]
        target_drive_motion_deg = [
            float(row["max_relative_rotation_deg"])
            for row in target_drive_rows
        ]
        min_target_drive_motion_deg = min(target_drive_motion_deg, default=0.0)
        clearance_lift = float(target_peak_clearance[env_index] - initial_clearances[env_index, target_leg])
        stable_mechanics = (
            bool(finite[env_index])
            and int(done_counts[env_index]) == 0
            and float(max_tilts[env_index]) <= float(args_cli.max_stable_tilt_deg)
            and float(min_heights[env_index]) >= float(args_cli.min_stable_body_height_m)
            and float(max_joint_separation[env_index]) <= float(args_cli.max_stable_joint_separation_m)
        )
        passed = (
            stable_mechanics
            and float(target_peak_clearance[env_index]) >= float(args_cli.min_target_clearance_m)
            and float(target_peak_relative_lift[env_index]) >= float(args_cli.min_body_relative_lift_m)
            and min_target_drive_motion_deg >= float(args_cli.min_target_linkage_drive_motion_deg)
            and support_count >= int(args_cli.min_support_feet)
        )
        support_excess = float(np.sum(np.maximum(stance_clearances - float(args_cli.support_clearance_m), 0.0)))
        score = (
            2.4 * float(target_peak_clearance[env_index])
            + 1.5 * float(target_peak_relative_lift[env_index])
            + 0.5 * max(clearance_lift, 0.0)
            + 0.0015 * min_target_drive_motion_deg
            - 1.2 * support_excess
            - 0.0012 * float(max_tilts[env_index])
            - 0.30 * float(np.linalg.norm(body_displacement[:2]))
            - 0.0015 * recovery_tilt
            - 0.03 * recovery_linear_speed
            - 0.01 * recovery_angular_speed
            - 8.0 * max(0.0, float(max_joint_separation[env_index]) - 0.003)
            - 2.0 * max(0.0, float(args_cli.min_stable_body_height_m) - float(min_heights[env_index]))
            - 0.001 * float(np.mean(np.abs(final_np[env_index])))
        )
        if not stable_mechanics:
            score -= 1.0
        rows.append(
            {
                "candidate_index": env_index,
                "passed": bool(passed),
                "stable_mechanics": bool(stable_mechanics),
                "score": round(float(score), 7),
                "transfer_actions": np.round(transfer_np[env_index], 6).tolist(),
                "final_actions": np.round(final_np[env_index], 6).tolist(),
                "target_peak_clearance_m": round(float(target_peak_clearance[env_index]), 6),
                "target_clearance_lift_m": round(clearance_lift, 6),
                "target_peak_body_relative_lift_m": round(float(target_peak_relative_lift[env_index]), 6),
                "target_max_body_relative_motion_m": round(float(max_target_body_motion[env_index]), 6),
                "target_linkage_drive_motion_deg": [
                    round(float(value), 6) for value in target_drive_motion_deg
                ],
                "min_target_linkage_drive_motion_deg": round(min_target_drive_motion_deg, 6),
                "support_count_at_peak": support_count,
                "clearances_at_target_peak_m": np.round(clearances_at_peak[env_index], 6).tolist(),
                "tilt_at_target_peak_deg": round(float(tilt_at_peak[env_index]), 6),
                "height_at_target_peak_m": round(float(height_at_peak[env_index]), 6),
                "max_tilt_deg": round(float(max_tilts[env_index]), 6),
                "min_body_height_m": round(float(min_heights[env_index]), 6),
                "max_joint_separation_m": round(float(max_joint_separation[env_index]), 6),
                "body_displacement_m": np.round(body_displacement, 6).tolist(),
                "recovery_tilt_deg": round(recovery_tilt, 6),
                "recovery_body_height_m": round(recovery_height, 6),
                "recovery_linear_speed_m_s": round(recovery_linear_speed, 6),
                "recovery_angular_speed_rad_s": round(recovery_angular_speed, 6),
                "done_count": int(done_counts[env_index]),
                "terminated_count": int(terminated_counts[env_index]),
                "finite": bool(finite[env_index]),
            }
        )
    return sorted(rows, key=lambda row: (bool(row["passed"]), float(row["score"])), reverse=True)


def main() -> None:
    target_leg = int(args_cli.leg_index)
    candidate_pool = load_candidate_pool(args_cli.initial_candidate_pool_path) if args_cli.initial_candidate_pool_path else None
    population_size = len(candidate_pool) if candidate_pool is not None else max(int(args_cli.population_size), 4)
    elite_count = max(1, min(int(args_cli.elite_count), population_size))
    generations = max(int(args_cli.generations), 1)
    if candidate_pool is not None and generations != 1:
        raise ValueError("Candidate-pool evaluation must use --generations 1.")
    prefix_order = parse_prefix_order(args_cli.prefix_command_order, target_leg)
    prefix_commands = {leg_index: load_verified_command(leg_index) for leg_index in prefix_order}
    rng = np.random.default_rng(int(args_cli.seed))

    cfg = DominoCadLinkageEnvCfg()
    cfg.seed = int(args_cli.seed)
    cfg.scene.num_envs = population_size
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
    total_steps = (
        int(args_cli.settle_steps)
        + int(args_cli.transfer_steps)
        + int(args_cli.lift_steps)
        + int(args_cli.hold_steps)
        + int(args_cli.release_steps)
        + int(args_cli.return_steps)
        + int(args_cli.neutral_steps)
    )
    cfg.episode_length_s = max(12.0, (total_steps + 40) * 0.02)

    env = DominoCadLinkageEnv(cfg)
    observations, _ = env.reset()
    if gym.spaces.flatdim(env.single_action_space) != EXPECTED_ACTION_COUNT:
        raise RuntimeError("Domino weight-transfer search requires the 12-action contract.")
    if observations["policy"].shape[0] != population_size:
        raise RuntimeError("Population/environment count mismatch.")

    population = candidate_pool.copy() if candidate_pool is not None else initial_population(rng, population_size, target_leg)
    if args_cli.initial_candidate_path:
        initial_candidate = load_initial_candidate(args_cli.initial_candidate_path)
        population[0] = initial_candidate
        for candidate_index in range(1, population_size):
            population[candidate_index] = np.clip(
                initial_candidate + rng.normal(0.0, float(args_cli.initial_sigma), EXPECTED_ACTION_COUNT),
                -1.0,
                1.0,
            )
    generation_reports: list[dict[str, object]] = []
    global_best: dict[str, object] | None = None
    for generation in range(generations):
        rows = evaluate_population(env, population, target_leg, prefix_order, prefix_commands)
        best = dict(rows[0])
        best["generation"] = generation
        if global_best is None or (bool(best["passed"]), float(best["score"])) > (
            bool(global_best["passed"]),
            float(global_best["score"]),
        ):
            global_best = dict(best)
        generation_reports.append(
            {
                "generation": generation,
                "passed_count": sum(bool(row["passed"]) for row in rows),
                "stable_mechanics_count": sum(bool(row["stable_mechanics"]) for row in rows),
                "best": best,
                "top": rows[: min(10, len(rows))],
            }
        )
        print(
            f"generation={generation} passed={generation_reports[-1]['passed_count']} "
            f"stable={generation_reports[-1]['stable_mechanics_count']} "
            f"best_clearance={float(best['target_peak_clearance_m']):.4f} "
            f"best_relative_lift={float(best['target_peak_body_relative_lift_m']):.4f} "
            f"best_tilt={float(best['max_tilt_deg']):.2f} score={float(best['score']):.5f}",
            flush=True,
        )
        elites = np.asarray([row["final_actions"] for row in rows[:elite_count]], dtype=np.float32)
        if generations == 1:
            sigma = float(args_cli.final_sigma)
        else:
            progress = float(generation) / float(generations - 1)
            sigma = float(args_cli.initial_sigma) * ((float(args_cli.final_sigma) / float(args_cli.initial_sigma)) ** progress)
        population = next_population(rng, elites, population_size, sigma, target_leg)

    assert global_best is not None
    report = {
        "status": "passed" if bool(global_best["passed"]) else "no_passing_candidate",
        "seed": int(args_cli.seed),
        "target_leg_index": target_leg,
        "target_leg": per_leg_action_layout()[target_leg],
        "prefix_command_order": prefix_order,
        "population_size": population_size,
        "generations": generations,
        "elite_count": elite_count,
        "settle_steps": int(args_cli.settle_steps),
        "transfer_steps": int(args_cli.transfer_steps),
        "lift_steps": int(args_cli.lift_steps),
        "hold_steps": int(args_cli.hold_steps),
        "release_steps": int(args_cli.release_steps),
        "return_steps": int(args_cli.return_steps),
        "neutral_steps": int(args_cli.neutral_steps),
        "action_scale_deg": float(args_cli.action_scale_deg),
        "servo_target_rate_limit_deg_s": float(args_cli.servo_target_rate_limit_deg_s),
        "thresholds": {
            "min_target_clearance_m": float(args_cli.min_target_clearance_m),
            "min_body_relative_lift_m": float(args_cli.min_body_relative_lift_m),
            "min_target_linkage_drive_motion_deg": float(args_cli.min_target_linkage_drive_motion_deg),
            "support_clearance_m": float(args_cli.support_clearance_m),
            "min_support_feet": int(args_cli.min_support_feet),
            "max_stable_tilt_deg": float(args_cli.max_stable_tilt_deg),
            "max_stable_joint_separation_m": float(args_cli.max_stable_joint_separation_m),
            "min_stable_body_height_m": float(args_cli.min_stable_body_height_m),
        },
        "action_names": ACTION_JOINT_NAMES,
        "best": global_best,
        "generation_reports": generation_reports,
    }
    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if args_cli.command_path and bool(global_best["passed"]):
        command_path = Path(args_cli.command_path).expanduser().resolve()
        command_path.parent.mkdir(parents=True, exist_ok=True)
        command_path.write_text(
            json.dumps(
                {
                    "name": f"domino_{per_leg_action_layout()[target_leg]['leg_id']}_weight_transfer",
                    "target_leg_index": target_leg,
                    "target_leg": per_leg_action_layout()[target_leg],
                    "action_scale_deg": float(args_cli.action_scale_deg),
                    "servo_target_rate_limit_deg_s": float(args_cli.servo_target_rate_limit_deg_s),
                    "transfer_steps": int(args_cli.transfer_steps),
                    "lift_steps": int(args_cli.lift_steps),
                    "hold_steps": int(args_cli.hold_steps),
                    "release_steps": int(args_cli.release_steps),
                    "return_steps": int(args_cli.return_steps),
                    "neutral_steps": int(args_cli.neutral_steps),
                    "transfer_actions": global_best["transfer_actions"],
                    "final_actions": global_best["final_actions"],
                    "verified_metrics": global_best,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
    if not bool(args_cli.no_print_report):
        print(json.dumps(report, indent=2), flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if bool(getattr(args_cli, "graceful_close", False)):
            simulation_app.close()
