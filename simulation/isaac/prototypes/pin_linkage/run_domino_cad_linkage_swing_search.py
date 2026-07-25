"""Search an airborne Domino linkage sweep from a verified foot-unload pose."""

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


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--leg-index", type=int, required=True, choices=range(4), help="CAD leg to sweep while unloaded.")
parser.add_argument("--population-size", type=int, default=32, help="Parallel sweep candidates per generation.")
parser.add_argument("--generations", type=int, default=5, help="Evolution generations.")
parser.add_argument("--elite-count", type=int, default=8, help="Candidates retained between generations.")
parser.add_argument("--seed", type=int, default=240730, help="Deterministic search and Isaac seed.")
parser.add_argument("--settle-steps", type=int, default=100, help="Initial neutral settle.")
parser.add_argument("--transfer-steps", type=int, default=60, help="Ramp into the verified support pose.")
parser.add_argument("--lift-steps", type=int, default=60, help="Ramp from support to verified foot lift.")
parser.add_argument("--lift-hold-steps", type=int, default=20, help="Hold before the sweep baseline is captured.")
parser.add_argument("--sweep-steps", type=int, default=70, help="Ramp from lift pose to candidate sweep pose.")
parser.add_argument("--sweep-hold-steps", type=int, default=20, help="Candidate sweep hold.")
parser.add_argument("--return-steps", type=int, default=70, help="Ramp from sweep back to lift.")
parser.add_argument("--place-steps", type=int, default=50, help="Ramp from lift back to support.")
parser.add_argument("--neutral-steps", type=int, default=80, help="Ramp to neutral and recover.")
parser.add_argument("--recovery-hold-steps", type=int, default=40, help="Neutral recovery hold.")
parser.add_argument("--metric-stride", type=int, default=2, help="Physics steps between detailed samples.")
parser.add_argument(
    "--action-scale-deg",
    type=float,
    default=0.0,
    help="Optional wider action range; verified source actions are rescaled to preserve their physical targets.",
)
parser.add_argument("--initial-sigma", type=float, default=0.45, help="First-generation target-drive mutation sigma.")
parser.add_argument("--final-sigma", type=float, default=0.08, help="Final target-drive mutation sigma.")
parser.add_argument("--initial-report-path", default="", help="Optional prior sweep report used to seed refinement.")
parser.add_argument("--min-sweep-clearance-m", type=float, default=0.010, help="Minimum CAD-foot clearance during sweep.")
parser.add_argument("--min-fore-aft-motion-m", type=float, default=0.030, help="Required hip-carriage-relative fore/aft endpoint travel.")
parser.add_argument("--min-total-foot-motion-m", type=float, default=0.035, help="Required total hip-carriage-relative endpoint travel.")
parser.add_argument(
    "--min-each-linkage-drive-motion-deg",
    type=float,
    default=6.0,
    help="Required lower and upper driver rotation relative to the hip carriage over the staged cycle.",
)
parser.add_argument(
    "--min-sweep-linkage-drive-motion-deg",
    type=float,
    default=6.0,
    help="Required motion from at least one target linkage drive during the airborne sweep.",
)
parser.add_argument("--support-clearance-m", type=float, default=0.008, help="Maximum clearance counted as support.")
parser.add_argument("--min-support-feet", type=int, default=2, choices=range(1, 4), help="Support feet required throughout sweep.")
parser.add_argument("--max-stable-tilt-deg", type=float, default=25.0, help="Maximum body tilt.")
parser.add_argument("--max-stable-joint-separation-m", type=float, default=0.001, help="Maximum loop-pin error.")
parser.add_argument("--min-stable-body-height-m", type=float, default=0.22, help="Minimum body-reference height.")
parser.add_argument("--report-path", default="", help="Optional JSON report path.")
parser.add_argument("--command-path", default="", help="Optional JSON path for the verified staged command.")
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
from domino_linkage_motion import (  # noqa: E402
    hip_carriage_relative_actual_cad_visual_feet,
    linkage_drive_relative_rotations,
    rotation_delta_deg,
)


LEG_LABELS = ["front_right", "front_left", "rear_left", "rear_right"]


def load_verified_command(leg_index: int) -> tuple[Path, dict[str, object]]:
    path = REPO_ROOT / "simulation" / "isaac" / "config" / f"domino_{LEG_LABELS[leg_index]}_weight_transfer.json"
    command = json.loads(path.read_text(encoding="utf-8-sig"))
    if int(command.get("target_leg_index", -1)) != leg_index:
        raise ValueError(f"Target leg mismatch in {path.name}.")
    if not bool(command.get("verified_metrics", {}).get("passed")):
        raise RuntimeError(f"Source command is not verified: {path.name}")
    return path, command


def body_tilt_deg(orientation: np.ndarray) -> float:
    gravity = projected_gravity_from_quat(orientation)
    return math.degrees(math.acos(max(-1.0, min(1.0, -float(gravity[2])))))


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


def initial_population(
    rng: np.random.Generator,
    population_size: int,
    lift_actions: np.ndarray,
    target_leg: int,
) -> np.ndarray:
    target_start = target_leg * 3
    values = (-1.0, -0.5, 0.0, 0.5, 1.0)
    seeds = []
    for lower in values:
        for upper in values:
            candidate = lift_actions.copy()
            candidate[target_start + 1 : target_start + 3] = (lower, upper)
            seeds.append(candidate)
    population = np.tile(lift_actions, (population_size, 1)).astype(np.float32)
    for index in range(population_size):
        if index < len(seeds):
            population[index] = seeds[index]
        else:
            population[index, target_start + 1 : target_start + 3] = rng.uniform(-1.0, 1.0, 2)
    return population


def next_population(
    rng: np.random.Generator,
    elites: np.ndarray,
    population_size: int,
    sigma: float,
    lift_actions: np.ndarray,
    target_leg: int,
) -> np.ndarray:
    target_slice = slice(target_leg * 3, target_leg * 3 + 3)
    population = np.tile(lift_actions, (population_size, 1)).astype(np.float32)
    keep = min(len(elites), population_size)
    population[:keep] = elites[:keep]
    for index in range(keep, population_size):
        first = elites[int(rng.integers(0, len(elites)))]
        second = elites[int(rng.integers(0, len(elites)))]
        blend = float(rng.uniform(0.25, 0.75))
        target = blend * first[target_slice] + (1.0 - blend) * second[target_slice]
        mutation = rng.normal(0.0, sigma, 3)
        mutation[0] *= 0.5
        population[index, target_slice] = np.clip(target + mutation, -1.0, 1.0)
    return population


def load_seed_elites(path: str) -> np.ndarray:
    report = json.loads(Path(path).expanduser().resolve().read_text(encoding="utf-8-sig"))
    rows = []
    if isinstance(report.get("best"), dict):
        rows.append(report["best"])
    for generation in report.get("generation_reports", []):
        rows.extend(generation.get("top", []))
    candidates: dict[tuple[float, ...], np.ndarray] = {}
    for row in rows:
        values = np.asarray(row.get("sweep_actions", []), dtype=np.float32).reshape(-1)
        if values.shape != (EXPECTED_ACTION_COUNT,):
            continue
        candidates[tuple(np.round(values, 6).tolist())] = np.clip(values, -1.0, 1.0)
    if not candidates:
        raise ValueError("Initial sweep report contains no valid sweep_actions rows.")
    return np.vstack(list(candidates.values())).astype(np.float32)


def evaluate_population(
    env: DominoCadLinkageEnv,
    population: np.ndarray,
    target_leg: int,
    transfer_actions: np.ndarray,
    lift_actions: np.ndarray,
) -> list[dict[str, object]]:
    env.reset()
    population_size = len(population)
    zero_np = np.zeros((population_size, EXPECTED_ACTION_COUNT), dtype=np.float32)
    transfer_np = np.tile(transfer_actions, (population_size, 1)).astype(np.float32)
    lift_np = np.tile(lift_actions, (population_size, 1)).astype(np.float32)
    finite = np.ones(population_size, dtype=bool)
    max_tilts = np.zeros(population_size, dtype=np.float64)
    min_heights = np.full(population_size, np.inf, dtype=np.float64)
    max_pins = np.zeros(population_size, dtype=np.float64)
    neutral_rotations: list[np.ndarray] | None = None
    total_target_drive_motion = np.zeros((population_size, 2), dtype=np.float64)
    stride = max(int(args_cli.metric_stride), 1)
    step_counter = 0

    def sample_mechanics() -> None:
        for env_index in range(population_size):
            position, orientation, linear_velocity, angular_velocity = env._body_reference_state(env_index)
            if not all(np.isfinite(value).all() for value in (position, orientation, linear_velocity, angular_velocity)):
                finite[env_index] = False
                continue
            max_tilts[env_index] = max(max_tilts[env_index], body_tilt_deg(orientation))
            min_heights[env_index] = min(
                min_heights[env_index],
                float(position[2] - env._env_origins_np[env_index][2]),
            )
            max_pins[env_index] = max(
                max_pins[env_index],
                max((float(row["separation_m"]) for row in env._joint_separation_rows(env_index)), default=0.0),
            )
            if neutral_rotations is not None:
                _, current_rotations = linkage_drive_relative_rotations(env, env_index)
                for local_drive_index, drive_index in enumerate(range(target_leg * 2, target_leg * 2 + 2)):
                    total_target_drive_motion[env_index, local_drive_index] = max(
                        total_target_drive_motion[env_index, local_drive_index],
                        rotation_delta_deg(neutral_rotations[env_index][drive_index], current_rotations[drive_index]),
                    )

    def run_ramp(start_np: np.ndarray, end_np: np.ndarray, steps: int, sample_swing=None) -> np.ndarray:
        nonlocal step_counter
        current = start_np.copy()
        for step_index in range(max(int(steps), 0)):
            alpha = float(step_index + 1) / float(max(int(steps), 1))
            current = start_np + alpha * (end_np - start_np)
            fast_physics_step(env, torch.tensor(current, dtype=torch.float32, device=env.device))
            step_counter += 1
            if step_counter % stride == 0 or step_index + 1 == int(steps):
                sample_mechanics()
                if sample_swing is not None:
                    sample_swing()
        return current

    current_np = zero_np.copy()
    current_np = run_ramp(current_np, zero_np, int(args_cli.settle_steps))
    finite[:] = True
    max_tilts[:] = 0.0
    min_heights[:] = np.inf
    max_pins[:] = 0.0
    neutral_rotations = [
        linkage_drive_relative_rotations(env, env_index)[1]
        for env_index in range(population_size)
    ]
    current_np = run_ramp(current_np, transfer_np, int(args_cli.transfer_steps))
    current_np = run_ramp(current_np, lift_np, int(args_cli.lift_steps))
    current_np = run_ramp(current_np, lift_np, int(args_cli.lift_hold_steps))

    swing_start_body_positions = np.zeros((population_size, 3), dtype=np.float32)
    swing_start_relative_feet = np.zeros((population_size, 4, 3), dtype=np.float32)
    swing_start_clearances = np.zeros((population_size, 4), dtype=np.float32)
    swing_start_rotations: list[np.ndarray] = []
    for env_index in range(population_size):
        swing_start_body_positions[env_index] = env._body_reference_state(env_index)[0]
        swing_start_relative_feet[env_index] = hip_carriage_relative_actual_cad_visual_feet(env, env_index)
        swing_start_clearances[env_index] = visual_clearances(env, env_index)
        _, rotations = linkage_drive_relative_rotations(env, env_index)
        swing_start_rotations.append(rotations)

    min_sweep_clearances = swing_start_clearances[:, target_leg].astype(np.float64)
    max_fore_aft_motion = np.zeros(population_size, dtype=np.float64)
    max_total_motion = np.zeros(population_size, dtype=np.float64)
    min_support_counts = np.full(population_size, 3, dtype=np.int64)
    target_drive_motion = np.zeros((population_size, 2), dtype=np.float64)
    stance_legs = [index for index in range(4) if index != target_leg]

    def sample_swing() -> None:
        for env_index in range(population_size):
            relative_feet = hip_carriage_relative_actual_cad_visual_feet(env, env_index)
            delta = relative_feet[target_leg] - swing_start_relative_feet[env_index, target_leg]
            max_fore_aft_motion[env_index] = max(max_fore_aft_motion[env_index], abs(float(delta[0])))
            max_total_motion[env_index] = max(max_total_motion[env_index], float(np.linalg.norm(delta)))
            clearances = visual_clearances(env, env_index).astype(np.float64)
            min_sweep_clearances[env_index] = min(min_sweep_clearances[env_index], float(clearances[target_leg]))
            support_count = int(
                np.count_nonzero(clearances[stance_legs] <= float(args_cli.support_clearance_m))
            )
            min_support_counts[env_index] = min(min_support_counts[env_index], support_count)
            _, current_rotations = linkage_drive_relative_rotations(env, env_index)
            for local_drive_index, drive_index in enumerate(range(target_leg * 2, target_leg * 2 + 2)):
                target_drive_motion[env_index, local_drive_index] = max(
                    target_drive_motion[env_index, local_drive_index],
                    rotation_delta_deg(swing_start_rotations[env_index][drive_index], current_rotations[drive_index]),
                )

    current_np = run_ramp(current_np, population, int(args_cli.sweep_steps), sample_swing)
    current_np = run_ramp(current_np, population, int(args_cli.sweep_hold_steps), sample_swing)
    current_np = run_ramp(current_np, lift_np, int(args_cli.return_steps))
    current_np = run_ramp(current_np, transfer_np, int(args_cli.place_steps))
    current_np = run_ramp(current_np, zero_np, int(args_cli.neutral_steps))
    run_ramp(current_np, zero_np, int(args_cli.recovery_hold_steps))

    rows: list[dict[str, object]] = []
    for env_index in range(population_size):
        final_position, final_orientation, final_linear_velocity, final_angular_velocity = env._body_reference_state(env_index)
        body_displacement = final_position - swing_start_body_positions[env_index]
        min_total_drive_motion = float(np.min(total_target_drive_motion[env_index]))
        max_sweep_drive_motion = float(np.max(target_drive_motion[env_index]))
        stable = (
            bool(finite[env_index])
            and float(max_tilts[env_index]) <= float(args_cli.max_stable_tilt_deg)
            and float(min_heights[env_index]) >= float(args_cli.min_stable_body_height_m)
            and float(max_pins[env_index]) <= float(args_cli.max_stable_joint_separation_m)
        )
        drive_gate = (
            min_total_drive_motion >= float(args_cli.min_each_linkage_drive_motion_deg)
            and max_sweep_drive_motion >= float(args_cli.min_sweep_linkage_drive_motion_deg)
        )
        airborne_gate = float(min_sweep_clearances[env_index]) >= float(args_cli.min_sweep_clearance_m)
        endpoint_gate = (
            float(max_fore_aft_motion[env_index]) >= float(args_cli.min_fore_aft_motion_m)
            and float(max_total_motion[env_index]) >= float(args_cli.min_total_foot_motion_m)
        )
        support_gate = int(min_support_counts[env_index]) >= int(args_cli.min_support_feet)
        passed = stable and drive_gate and airborne_gate and endpoint_gate and support_gate
        feasibility_ratio = min(
            float(min_sweep_clearances[env_index]) / max(float(args_cli.min_sweep_clearance_m), 1.0e-6),
            float(max_fore_aft_motion[env_index]) / max(float(args_cli.min_fore_aft_motion_m), 1.0e-6),
            float(max_total_motion[env_index]) / max(float(args_cli.min_total_foot_motion_m), 1.0e-6),
            min_total_drive_motion / max(float(args_cli.min_each_linkage_drive_motion_deg), 1.0e-6),
            max_sweep_drive_motion / max(float(args_cli.min_sweep_linkage_drive_motion_deg), 1.0e-6),
            float(min_support_counts[env_index]) / max(float(args_cli.min_support_feet), 1.0),
        )
        score = (
            3.0 * min(float(max_fore_aft_motion[env_index]), 0.15)
            + 1.0 * min(float(max_total_motion[env_index]), 0.20)
            + 0.003 * min(min_total_drive_motion, 20.0)
            + 0.002 * min(max_sweep_drive_motion, 20.0)
            + 0.5 * min(float(min_sweep_clearances[env_index]), 0.04)
            - 0.004 * float(max_tilts[env_index])
            - 0.30 * float(np.linalg.norm(body_displacement[:2]))
            - 20.0 * max(0.0, float(max_pins[env_index]) - 0.003)
            - 2.0 * max(0.0, float(args_cli.min_sweep_clearance_m) - float(min_sweep_clearances[env_index]))
        )
        if not stable:
            score -= 2.0
        rows.append(
            {
                "candidate_index": env_index,
                "passed": bool(passed),
                "stable_mechanics": bool(stable),
                "drive_gate_passed": bool(drive_gate),
                "airborne_gate_passed": bool(airborne_gate),
                "endpoint_motion_gate_passed": bool(endpoint_gate),
                "support_gate_passed": bool(support_gate),
                "feasibility_ratio": round(float(feasibility_ratio), 6),
                "score": round(float(score), 7),
                "sweep_actions": np.round(population[env_index], 6).tolist(),
                "swing_start_clearance_m": round(float(swing_start_clearances[env_index, target_leg]), 6),
                "min_sweep_clearance_m": round(float(min_sweep_clearances[env_index]), 6),
                "max_hip_carriage_relative_fore_aft_motion_m": round(float(max_fore_aft_motion[env_index]), 6),
                "max_hip_carriage_relative_total_motion_m": round(float(max_total_motion[env_index]), 6),
                "target_linkage_drive_motion_deg": np.round(total_target_drive_motion[env_index], 6).tolist(),
                "min_target_linkage_drive_motion_deg": round(min_total_drive_motion, 6),
                "sweep_linkage_drive_motion_deg": np.round(target_drive_motion[env_index], 6).tolist(),
                "max_sweep_linkage_drive_motion_deg": round(max_sweep_drive_motion, 6),
                "min_support_count_during_sweep": int(min_support_counts[env_index]),
                "max_tilt_deg": round(float(max_tilts[env_index]), 6),
                "min_body_height_m": round(float(min_heights[env_index]), 6),
                "max_joint_separation_m": round(float(max_pins[env_index]), 6),
                "body_displacement_from_swing_start_m": np.round(body_displacement, 6).tolist(),
                "recovery_tilt_deg": round(body_tilt_deg(final_orientation), 6),
                "recovery_linear_speed_m_s": round(float(np.linalg.norm(final_linear_velocity)), 6),
                "recovery_angular_speed_rad_s": round(float(np.linalg.norm(final_angular_velocity)), 6),
                "finite": bool(finite[env_index]),
            }
        )
    return sorted(
        rows,
        key=lambda row: (
            bool(row["passed"]),
            bool(row["stable_mechanics"]),
            float(row["feasibility_ratio"]),
            float(row["score"]),
        ),
        reverse=True,
    )


def main() -> None:
    target_leg = int(args_cli.leg_index)
    command_path, command = load_verified_command(target_leg)
    source_action_scale_deg = float(command["action_scale_deg"])
    action_scale_deg = (
        float(args_cli.action_scale_deg)
        if float(args_cli.action_scale_deg) > 0.0
        else source_action_scale_deg
    )
    source_action_factor = source_action_scale_deg / action_scale_deg
    transfer_actions = np.clip(
        np.asarray(command["transfer_actions"], dtype=np.float32).reshape(-1) * source_action_factor,
        -1.0,
        1.0,
    )
    lift_actions = np.clip(
        np.asarray(command["final_actions"], dtype=np.float32).reshape(-1) * source_action_factor,
        -1.0,
        1.0,
    )
    if transfer_actions.shape != (EXPECTED_ACTION_COUNT,) or lift_actions.shape != (EXPECTED_ACTION_COUNT,):
        raise ValueError("Verified support command does not satisfy the 12-action contract.")
    population_size = max(int(args_cli.population_size), 4)
    generations = max(int(args_cli.generations), 1)
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
    cfg.action_scale_deg = action_scale_deg
    cfg.servo_target_rate_limit_deg_s = float(command["servo_target_rate_limit_deg_s"])
    cfg.min_height_m = -10.0
    cfg.max_tilt_deg = 180.0
    cfg.episode_length_s = 120.0

    env = DominoCadLinkageEnv(cfg)
    observations, _ = env.reset()
    if gym.spaces.flatdim(env.single_action_space) != EXPECTED_ACTION_COUNT:
        raise RuntimeError("Domino swing search requires the 12-action contract.")
    if observations["policy"].shape[0] != population_size:
        raise RuntimeError("Population/environment count mismatch.")

    if args_cli.initial_report_path:
        seed_elites = load_seed_elites(args_cli.initial_report_path)
        population = next_population(
            rng,
            seed_elites,
            population_size,
            float(args_cli.initial_sigma),
            lift_actions,
            target_leg,
        )
    else:
        population = initial_population(rng, population_size, lift_actions, target_leg)
    generation_reports = []
    global_best = None
    for generation in range(generations):
        rows = evaluate_population(env, population, target_leg, transfer_actions, lift_actions)
        best = dict(rows[0])
        best["generation"] = generation
        if global_best is None or (
            bool(best["passed"]),
            bool(best["stable_mechanics"]),
            float(best["feasibility_ratio"]),
            float(best["score"]),
        ) > (
            bool(global_best["passed"]),
            bool(global_best["stable_mechanics"]),
            float(global_best["feasibility_ratio"]),
            float(global_best["score"]),
        ):
            global_best = dict(best)
        generation_reports.append(
            {
                "generation": generation,
                "passed_count": sum(bool(row["passed"]) for row in rows),
                "stable_count": sum(bool(row["stable_mechanics"]) for row in rows),
                "drive_gate_count": sum(bool(row["drive_gate_passed"]) for row in rows),
                "airborne_gate_count": sum(bool(row["airborne_gate_passed"]) for row in rows),
                "endpoint_gate_count": sum(bool(row["endpoint_motion_gate_passed"]) for row in rows),
                "best": best,
                "top": rows[: min(10, len(rows))],
            }
        )
        print(
            f"generation={generation} passed={generation_reports[-1]['passed_count']} "
            f"stable={generation_reports[-1]['stable_count']} drives={best['target_linkage_drive_motion_deg']} "
            f"clearance={float(best['min_sweep_clearance_m']):.4f} "
            f"fore_aft={float(best['max_hip_carriage_relative_fore_aft_motion_m']):.4f} "
            f"feasibility={float(best['feasibility_ratio']):.3f} "
            f"score={float(best['score']):.5f}",
            flush=True,
        )
        elites = np.asarray([row["sweep_actions"] for row in rows[:elite_count]], dtype=np.float32)
        progress = float(generation) / float(max(generations - 1, 1))
        sigma = float(args_cli.initial_sigma) * (
            (float(args_cli.final_sigma) / float(args_cli.initial_sigma)) ** progress
        )
        population = next_population(rng, elites, population_size, sigma, lift_actions, target_leg)

    assert global_best is not None
    report = {
        "status": "passed" if bool(global_best["passed"]) else "no_passing_candidate",
        "seed": int(args_cli.seed),
        "target_leg_index": target_leg,
        "target_leg": per_leg_action_layout()[target_leg],
        "source_command": command_path.relative_to(REPO_ROOT).as_posix(),
        "population_size": population_size,
        "generations": generations,
        "action_names": ACTION_JOINT_NAMES,
        "source_action_scale_deg": source_action_scale_deg,
        "source_action_rescale_factor": source_action_factor,
        "action_scale_deg": float(cfg.action_scale_deg),
        "servo_target_rate_limit_deg_s": float(cfg.servo_target_rate_limit_deg_s),
        "phase_steps": {
            "settle": int(args_cli.settle_steps),
            "transfer": int(args_cli.transfer_steps),
            "lift": int(args_cli.lift_steps),
            "lift_hold": int(args_cli.lift_hold_steps),
            "sweep": int(args_cli.sweep_steps),
            "sweep_hold": int(args_cli.sweep_hold_steps),
            "return": int(args_cli.return_steps),
            "place": int(args_cli.place_steps),
            "neutral": int(args_cli.neutral_steps),
            "recovery_hold": int(args_cli.recovery_hold_steps),
        },
        "thresholds": {
            "min_sweep_clearance_m": float(args_cli.min_sweep_clearance_m),
            "min_fore_aft_motion_m": float(args_cli.min_fore_aft_motion_m),
            "min_total_foot_motion_m": float(args_cli.min_total_foot_motion_m),
            "min_each_linkage_drive_motion_deg": float(args_cli.min_each_linkage_drive_motion_deg),
            "min_sweep_linkage_drive_motion_deg": float(args_cli.min_sweep_linkage_drive_motion_deg),
            "support_clearance_m": float(args_cli.support_clearance_m),
            "min_support_feet": int(args_cli.min_support_feet),
            "max_stable_tilt_deg": float(args_cli.max_stable_tilt_deg),
            "max_stable_joint_separation_m": float(args_cli.max_stable_joint_separation_m),
            "min_stable_body_height_m": float(args_cli.min_stable_body_height_m),
        },
        "best": global_best,
        "generation_reports": generation_reports,
    }
    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if args_cli.command_path and bool(global_best["passed"]):
        output_path = Path(args_cli.command_path).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(
                {
                    "name": f"domino_{LEG_LABELS[target_leg]}_verified_swing",
                    "target_leg_index": target_leg,
                    "target_leg": per_leg_action_layout()[target_leg],
                    "action_scale_deg": float(cfg.action_scale_deg),
                    "servo_target_rate_limit_deg_s": float(cfg.servo_target_rate_limit_deg_s),
                    "transfer_actions": np.round(transfer_actions, 6).tolist(),
                    "lift_actions": np.round(lift_actions, 6).tolist(),
                    "sweep_actions": global_best["sweep_actions"],
                    "verified_metrics": global_best,
                },
                indent=2,
            )
            + "\n",
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
