"""Search a stable bridge between verified Domino foot-unload commands."""

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


parser = argparse.ArgumentParser(description="Search a full-body bridge between Domino unload commands.")
parser.add_argument("--prefix-order", required=True, help="Stable comma-separated command prefix ending at the source leg.")
parser.add_argument("--target-leg-index", type=int, required=True, choices=range(4), help="Command to execute after the bridge.")
parser.add_argument("--population-size", type=int, default=24, help="Parallel bridge candidates per generation.")
parser.add_argument("--generations", type=int, default=6, help="Evolution generations.")
parser.add_argument("--elite-count", type=int, default=8, help="Candidates retained between generations.")
parser.add_argument("--seed", type=int, default=240728, help="Deterministic search and Isaac seed.")
parser.add_argument("--settle-steps", type=int, default=80, help="Initial neutral settle.")
parser.add_argument("--command-ramp-steps", type=int, default=60, help="Ramp length for verified command stages.")
parser.add_argument("--command-hold-steps", type=int, default=20, help="Hold length for verified lift stages.")
parser.add_argument("--bridge-ramp-steps", type=int, default=60, help="Ramp into and out of the candidate bridge.")
parser.add_argument("--bridge-hold-steps", type=int, default=20, help="Candidate bridge hold length.")
parser.add_argument("--metric-stride", type=int, default=4, help="Physics steps between detailed samples.")
parser.add_argument("--initial-sigma", type=float, default=0.55, help="Initial mutation standard deviation.")
parser.add_argument("--final-sigma", type=float, default=0.12, help="Final mutation standard deviation.")
parser.add_argument("--min-target-clearance-m", type=float, default=0.012, help="Required target CAD-foot clearance.")
parser.add_argument("--min-body-relative-lift-m", type=float, default=0.008, help="Required target lift in body coordinates.")
parser.add_argument("--support-clearance-m", type=float, default=0.008, help="Maximum clearance counted as support contact.")
parser.add_argument("--min-support-feet", type=int, default=2, choices=range(1, 4), help="Support feet required at peak lift.")
parser.add_argument("--max-stable-tilt-deg", type=float, default=30.0, help="Maximum body tilt.")
parser.add_argument("--max-stable-joint-separation-m", type=float, default=0.001, help="Maximum loop-pin error.")
parser.add_argument("--min-stable-body-height-m", type=float, default=0.22, help="Minimum body-reference height.")
parser.add_argument("--report-path", default="", help="Optional JSON report path.")
parser.add_argument("--bridge-path", default="", help="Optional JSON path for the best bridge.")
parser.add_argument("--no-print-report", action="store_true", help="Suppress full JSON output.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

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


def parse_prefix(raw: str) -> list[int]:
    prefix = [int(item.strip()) for item in str(raw).replace(";", ",").split(",") if item.strip()]
    if not prefix or any(index not in range(4) for index in prefix):
        raise ValueError("Prefix order must contain valid leg indexes.")
    if len(set(prefix)) != len(prefix):
        raise ValueError("Prefix order cannot repeat a leg.")
    if int(args_cli.target_leg_index) in prefix:
        raise ValueError("Target leg cannot already appear in the prefix.")
    return prefix


def load_commands() -> dict[int, dict[str, object]]:
    commands = {}
    for leg_index, path in COMMAND_PATHS.items():
        command = json.loads(path.read_text(encoding="utf-8"))
        if int(command["target_leg_index"]) != leg_index:
            raise ValueError(f"Command leg mismatch in {path.relative_to(REPO_ROOT)}")
        command["transfer_actions"] = np.asarray(command["transfer_actions"], dtype=np.float32).reshape(-1)
        command["final_actions"] = np.asarray(command["final_actions"], dtype=np.float32).reshape(-1)
        commands[leg_index] = command
    return commands


def body_tilt_deg(orientation: np.ndarray) -> float:
    gravity = projected_gravity_from_quat(orientation)
    return math.degrees(math.acos(max(-1.0, min(1.0, -float(gravity[2])))))


def body_relative_visual_feet(env: DominoCadLinkageEnv, env_index: int) -> np.ndarray:
    body_position, body_orientation, _, _ = env._body_reference_state(env_index)
    feet = env._actual_cad_visual_foot_positions(env_index, rendered=True)
    return ((feet - body_position.reshape(1, 3)) @ quat_wxyz_to_rotation_matrix(body_orientation)).astype(np.float32)


def visual_clearances(env: DominoCadLinkageEnv, env_index: int) -> np.ndarray:
    feet = env._actual_cad_visual_foot_positions(env_index, rendered=True)
    return (feet[:, 2] - env._terrain_heights_np(feet, env_index=env_index)).astype(np.float32)


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


def seeded_population(
    rng: np.random.Generator,
    population_size: int,
    commands: dict[int, dict[str, object]],
    source_leg: int,
    target_leg: int,
) -> np.ndarray:
    source_final = np.asarray(commands[source_leg]["final_actions"], dtype=np.float32)
    source_transfer = np.asarray(commands[source_leg]["transfer_actions"], dtype=np.float32)
    target_transfer = np.asarray(commands[target_leg]["transfer_actions"], dtype=np.float32)
    target_final = np.asarray(commands[target_leg]["final_actions"], dtype=np.float32)
    seeds = [
        np.zeros(EXPECTED_ACTION_COUNT, dtype=np.float32),
        source_final,
        source_transfer,
        target_transfer,
        0.5 * (source_final + target_transfer),
        0.5 * (source_transfer + target_transfer),
        0.5 * (source_final + target_final),
    ]
    seeds.extend(np.asarray(commands[index]["transfer_actions"], dtype=np.float32) for index in range(4))
    population = np.zeros((population_size, EXPECTED_ACTION_COUNT), dtype=np.float32)
    for index in range(population_size):
        if index < len(seeds):
            population[index] = np.clip(seeds[index], -1.0, 1.0)
        else:
            center = seeds[int(rng.integers(0, len(seeds)))]
            population[index] = np.clip(center + rng.normal(0.0, 0.5, EXPECTED_ACTION_COUNT), -1.0, 1.0)
    return population


def next_population(
    rng: np.random.Generator,
    elites: np.ndarray,
    population_size: int,
    sigma: float,
) -> np.ndarray:
    population = np.zeros((population_size, EXPECTED_ACTION_COUNT), dtype=np.float32)
    keep = min(len(elites), population_size)
    population[:keep] = elites[:keep]
    for index in range(keep, population_size):
        first = elites[int(rng.integers(0, len(elites)))]
        second = elites[int(rng.integers(0, len(elites)))]
        blend = float(rng.uniform(0.25, 0.75))
        child = blend * first + (1.0 - blend) * second
        population[index] = np.clip(child + rng.normal(0.0, sigma, EXPECTED_ACTION_COUNT), -1.0, 1.0)
    return population


def evaluate_population(
    env: DominoCadLinkageEnv,
    population: np.ndarray,
    prefix: list[int],
    target_leg: int,
    commands: dict[int, dict[str, object]],
) -> list[dict[str, object]]:
    env.reset()
    population_size = len(population)
    zero_np = np.zeros((population_size, EXPECTED_ACTION_COUNT), dtype=np.float32)
    current_np = zero_np.copy()
    finite = np.ones(population_size, dtype=bool)
    max_tilts = np.zeros(population_size, dtype=np.float64)
    min_heights = np.full(population_size, np.inf, dtype=np.float64)
    max_pins = np.zeros(population_size, dtype=np.float64)
    stride = max(int(args_cli.metric_stride), 1)
    sample_counter = 0

    def sample_mechanics() -> None:
        for env_index in range(population_size):
            position, orientation, linear_velocity, angular_velocity = env._body_reference_state(env_index)
            if not all(np.isfinite(value).all() for value in (position, orientation, linear_velocity, angular_velocity)):
                finite[env_index] = False
                continue
            max_tilts[env_index] = max(max_tilts[env_index], body_tilt_deg(orientation))
            min_heights[env_index] = min(
                min_heights[env_index], float(position[2] - env._env_origins_np[env_index][2])
            )
            joint_rows = env._joint_separation_rows(env_index)
            max_pins[env_index] = max(
                max_pins[env_index],
                max((float(row["separation_m"]) for row in joint_rows), default=0.0),
            )

    def run_ramp(start_np: np.ndarray, end_np: np.ndarray, steps: int, sample: bool = True) -> np.ndarray:
        nonlocal sample_counter
        current = start_np.copy()
        for step_index in range(max(int(steps), 0)):
            alpha = float(step_index + 1) / float(max(int(steps), 1))
            current = start_np + alpha * (end_np - start_np)
            fast_physics_step(env, torch.tensor(current, dtype=torch.float32, device=env.device))
            sample_counter += 1
            if sample and (sample_counter % stride == 0 or step_index + 1 == int(steps)):
                sample_mechanics()
        return current

    current_np = run_ramp(current_np, zero_np, int(args_cli.settle_steps), sample=False)
    sample_mechanics()
    for leg_index in prefix:
        transfer = np.tile(np.asarray(commands[leg_index]["transfer_actions"], dtype=np.float32), (population_size, 1))
        final = np.tile(np.asarray(commands[leg_index]["final_actions"], dtype=np.float32), (population_size, 1))
        current_np = run_ramp(current_np, transfer, int(args_cli.command_ramp_steps))
        current_np = run_ramp(current_np, final, int(args_cli.command_ramp_steps))
        current_np = run_ramp(current_np, final, int(args_cli.command_hold_steps))

    initial_body_positions = np.zeros((population_size, 3), dtype=np.float32)
    initial_relative_feet = np.zeros((population_size, 4, 3), dtype=np.float32)
    initial_clearances = np.zeros((population_size, 4), dtype=np.float32)
    for env_index in range(population_size):
        initial_body_positions[env_index] = env._body_reference_state(env_index)[0]
        initial_relative_feet[env_index] = body_relative_visual_feet(env, env_index)
        initial_clearances[env_index] = visual_clearances(env, env_index)

    target_peak_clearance = initial_clearances[:, target_leg].astype(np.float64)
    target_peak_relative_lift = np.zeros(population_size, dtype=np.float64)
    clearances_at_peak = initial_clearances.astype(np.float64).copy()

    def sample_target() -> None:
        sample_mechanics()
        for env_index in range(population_size):
            clearances = visual_clearances(env, env_index).astype(np.float64)
            relative_feet = body_relative_visual_feet(env, env_index)
            relative_lift = float(relative_feet[target_leg, 2] - initial_relative_feet[env_index, target_leg, 2])
            target_peak_relative_lift[env_index] = max(target_peak_relative_lift[env_index], relative_lift)
            if float(clearances[target_leg]) > target_peak_clearance[env_index]:
                target_peak_clearance[env_index] = float(clearances[target_leg])
                clearances_at_peak[env_index] = clearances

    def run_target_ramp(start_np: np.ndarray, end_np: np.ndarray, steps: int) -> np.ndarray:
        nonlocal sample_counter
        current = start_np.copy()
        for step_index in range(max(int(steps), 0)):
            alpha = float(step_index + 1) / float(max(int(steps), 1))
            current = start_np + alpha * (end_np - start_np)
            fast_physics_step(env, torch.tensor(current, dtype=torch.float32, device=env.device))
            sample_counter += 1
            if sample_counter % stride == 0 or step_index + 1 == int(steps):
                sample_target()
        return current

    current_np = run_target_ramp(current_np, population, int(args_cli.bridge_ramp_steps))
    current_np = run_target_ramp(current_np, population, int(args_cli.bridge_hold_steps))
    target_transfer = np.tile(
        np.asarray(commands[target_leg]["transfer_actions"], dtype=np.float32), (population_size, 1)
    )
    target_final = np.tile(np.asarray(commands[target_leg]["final_actions"], dtype=np.float32), (population_size, 1))
    current_np = run_target_ramp(current_np, target_transfer, int(args_cli.bridge_ramp_steps))
    current_np = run_target_ramp(current_np, target_final, int(args_cli.command_ramp_steps))
    run_target_ramp(current_np, target_final, int(args_cli.command_hold_steps))

    stance_legs = [index for index in range(4) if index != target_leg]
    rows = []
    for env_index in range(population_size):
        final_position = env._body_reference_state(env_index)[0]
        body_displacement = final_position - initial_body_positions[env_index]
        support_count = int(
            np.count_nonzero(
                clearances_at_peak[env_index, stance_legs] <= float(args_cli.support_clearance_m)
            )
        )
        stable = (
            bool(finite[env_index])
            and float(max_tilts[env_index]) <= float(args_cli.max_stable_tilt_deg)
            and float(min_heights[env_index]) >= float(args_cli.min_stable_body_height_m)
            and float(max_pins[env_index]) <= float(args_cli.max_stable_joint_separation_m)
        )
        passed = (
            stable
            and float(target_peak_clearance[env_index]) >= float(args_cli.min_target_clearance_m)
            and float(target_peak_relative_lift[env_index]) >= float(args_cli.min_body_relative_lift_m)
            and support_count >= int(args_cli.min_support_feet)
        )
        support_excess = float(
            np.sum(
                np.maximum(
                    clearances_at_peak[env_index, stance_legs] - float(args_cli.support_clearance_m), 0.0
                )
            )
        )
        score = (
            2.4 * min(float(target_peak_clearance[env_index]), 0.05)
            + 1.5 * min(float(target_peak_relative_lift[env_index]), 0.10)
            - 1.2 * support_excess
            - 0.01 * float(max_tilts[env_index])
            - 0.3 * float(np.linalg.norm(body_displacement[:2]))
            - 20.0 * max(0.0, float(max_pins[env_index]) - 0.003)
            - 8.0 * max(0.0, float(args_cli.min_stable_body_height_m) - float(min_heights[env_index]))
            - 0.001 * float(np.mean(np.abs(population[env_index])))
        )
        if not stable:
            score -= 5.0
        rows.append(
            {
                "candidate_index": env_index,
                "passed": bool(passed),
                "stable_mechanics": bool(stable),
                "score": round(float(score), 7),
                "bridge_actions": np.round(population[env_index], 6).tolist(),
                "target_peak_clearance_m": round(float(target_peak_clearance[env_index]), 6),
                "target_peak_body_relative_lift_m": round(float(target_peak_relative_lift[env_index]), 6),
                "support_count_at_peak": support_count,
                "clearances_at_target_peak_m": np.round(clearances_at_peak[env_index], 6).tolist(),
                "max_tilt_deg": round(float(max_tilts[env_index]), 6),
                "min_body_height_m": round(float(min_heights[env_index]), 6),
                "max_joint_separation_m": round(float(max_pins[env_index]), 6),
                "body_displacement_from_source_m": np.round(body_displacement, 6).tolist(),
                "finite": bool(finite[env_index]),
            }
        )
    return sorted(rows, key=lambda row: (bool(row["passed"]), float(row["score"])), reverse=True)


def main() -> None:
    prefix = parse_prefix(args_cli.prefix_order)
    source_leg = prefix[-1]
    target_leg = int(args_cli.target_leg_index)
    commands = load_commands()
    population_size = max(4, int(args_cli.population_size))
    generations = max(1, int(args_cli.generations))
    elite_count = max(1, min(int(args_cli.elite_count), population_size))
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
    cfg.action_scale_deg = float(commands[0]["action_scale_deg"])
    cfg.servo_target_rate_limit_deg_s = float(commands[0]["servo_target_rate_limit_deg_s"])
    cfg.min_height_m = -10.0
    cfg.max_tilt_deg = 180.0
    cfg.episode_length_s = 120.0

    env = DominoCadLinkageEnv(cfg)
    observations, _ = env.reset()
    if gym.spaces.flatdim(env.single_action_space) != EXPECTED_ACTION_COUNT:
        raise RuntimeError("Domino transition search requires the 12-action contract.")
    if observations["policy"].shape[0] != population_size:
        raise RuntimeError("Population/environment count mismatch.")

    population = seeded_population(rng, population_size, commands, source_leg, target_leg)
    generation_reports = []
    global_best = None
    for generation in range(generations):
        rows = evaluate_population(env, population, prefix, target_leg, commands)
        best = dict(rows[0])
        best["generation"] = generation
        if global_best is None or (bool(best["passed"]), float(best["score"])) > (
            bool(global_best["passed"]), float(global_best["score"])
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
            f"clearance={float(best['target_peak_clearance_m']):.4f} "
            f"relative_lift={float(best['target_peak_body_relative_lift_m']):.4f} "
            f"tilt={float(best['max_tilt_deg']):.2f} score={float(best['score']):.5f}",
            flush=True,
        )
        elites = np.asarray([row["bridge_actions"] for row in rows[:elite_count]], dtype=np.float32)
        progress = float(generation) / float(max(generations - 1, 1))
        sigma = float(args_cli.initial_sigma) * (
            (float(args_cli.final_sigma) / float(args_cli.initial_sigma)) ** progress
        )
        population = next_population(rng, elites, population_size, sigma)

    assert global_best is not None
    report = {
        "status": "passed" if bool(global_best["passed"]) else "no_passing_candidate",
        "seed": int(args_cli.seed),
        "prefix_order": prefix,
        "source_leg_index": source_leg,
        "target_leg_index": target_leg,
        "source_leg": per_leg_action_layout()[source_leg],
        "target_leg": per_leg_action_layout()[target_leg],
        "population_size": population_size,
        "generations": generations,
        "action_names": ACTION_JOINT_NAMES,
        "action_scale_deg": float(cfg.action_scale_deg),
        "servo_target_rate_limit_deg_s": float(cfg.servo_target_rate_limit_deg_s),
        "best": global_best,
        "generation_reports": generation_reports,
    }
    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if args_cli.bridge_path and bool(global_best["passed"]):
        bridge_path = Path(args_cli.bridge_path).expanduser().resolve()
        bridge_path.parent.mkdir(parents=True, exist_ok=True)
        bridge_path.write_text(
            json.dumps(
                {
                    "name": f"domino_transition_{source_leg}_to_{target_leg}",
                    "prefix_order": prefix,
                    "source_leg_index": source_leg,
                    "target_leg_index": target_leg,
                    "action_scale_deg": float(cfg.action_scale_deg),
                    "servo_target_rate_limit_deg_s": float(cfg.servo_target_rate_limit_deg_s),
                    "bridge_ramp_steps": int(args_cli.bridge_ramp_steps),
                    "bridge_hold_steps": int(args_cli.bridge_hold_steps),
                    "bridge_actions": global_best["bridge_actions"],
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
