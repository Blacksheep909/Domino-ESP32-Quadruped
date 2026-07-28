"""Search scripted 12-actuator reference gaits for the Domino CAD-linkage env."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import random
import sys
import traceback

from isaaclab.app import AppLauncher

CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))


parser = argparse.ArgumentParser(description="Search scripted gait references for the Domino CAD-linkage env.")
parser.add_argument("--steps", type=int, default=240, help="Environment steps per candidate.")
parser.add_argument(
    "--settle-steps",
    type=int,
    default=120,
    help="Zero-action steps used to settle the authored neutral pose before gait metrics begin.",
)
parser.add_argument("--candidate-count", type=int, default=16, help="Number of candidates to evaluate in parallel.")
parser.add_argument(
    "--candidate-offset",
    type=int,
    default=0,
    help="Starting index for a deterministic --symmetry-candidate batch.",
)
parser.add_argument("--seed", type=int, default=42, help="Candidate generation seed.")
parser.add_argument("--action-scale-deg", type=float, default=30.0, help="Maximum drive target offset, in degrees, for action=1.")
parser.add_argument(
    "--servo-target-rate-limit-deg-s",
    type=float,
    default=None,
    help="Maximum servo target slew rate in degrees/second. Use 0 to disable the slew limiter.",
)
parser.add_argument("--command-x-m-s", type=float, default=0.08, help="Forward velocity command.")
parser.add_argument("--command-y-m-s", type=float, default=0.0, help="Lateral velocity command.")
parser.add_argument("--command-yaw-rad-s", type=float, default=0.0, help="Yaw-rate command.")
parser.add_argument("--gait-frequency-hz", type=float, default=1.0, help="Base gait phase frequency.")
parser.add_argument("--episode-length-s", type=float, default=6.0, help="Episode length.")
parser.add_argument("--floating-height-m", type=float, default=None, help="Initial body-reference height override.")
parser.add_argument("--min-height-m", type=float, default=None, help="Minimum body-reference height before reset.")
parser.add_argument("--max-tilt-deg", type=float, default=None, help="Maximum body-reference tilt before reset.")
parser.add_argument("--load-candidate", default="", help="Optional JSON candidate to replay instead of searching.")
parser.add_argument("--refine-candidate", default="", help="Optional JSON candidate to perturb into a local search batch.")
parser.add_argument("--symmetry-candidate", default="", help="Optional JSON candidate used to generate deterministic symmetry/phase variants.")
parser.add_argument("--refine-scale", type=float, default=0.35, help="Perturbation scale for --refine-candidate local search.")
parser.add_argument("--score-forward-weight", type=float, default=25.0, help="Score weight for final forward displacement.")
parser.add_argument("--score-lateral-weight", type=float, default=1.5, help="Score penalty weight for absolute final lateral displacement.")
parser.add_argument("--score-clearance-weight", type=float, default=0.75, help="Score weight for maximum foot clearance.")
parser.add_argument(
    "--score-min-foot-clearance-weight",
    type=float,
    default=8.0,
    help="Score weight for the least clearance achieved by any of the four CAD feet.",
)
parser.add_argument(
    "--score-min-foot-motion-weight",
    type=float,
    default=4.0,
    help="Score weight for the least body-relative motion achieved by any CAD foot.",
)
parser.add_argument(
    "--score-min-foot-clearance-m",
    type=float,
    default=0.001,
    help="Soft minimum peak ground clearance required from every CAD foot.",
)
parser.add_argument(
    "--score-min-foot-motion-m",
    type=float,
    default=0.010,
    help="Soft minimum body-relative displacement required from every CAD foot.",
)
parser.add_argument("--score-unload-weight", type=float, default=0.10, help="Score weight for reducing mean foot contacts below four.")
parser.add_argument("--score-tilt-weight", type=float, default=0.03, help="Score penalty weight for maximum body tilt in degrees.")
parser.add_argument("--score-heading-weight", type=float, default=1.0, help="Score penalty weight for absolute final body heading drift.")
parser.add_argument("--score-max-lateral-m", type=float, default=0.0, help="Optional soft cap for absolute final lateral displacement.")
parser.add_argument("--score-max-heading-rad", type=float, default=0.0, help="Optional soft cap for absolute final heading drift.")
parser.add_argument("--score-constraint-penalty", type=float, default=100.0, help="Penalty multiplier for exceeding optional search soft caps.")
parser.add_argument("--save-best-candidate", default="", help="Optional path to write the best candidate JSON.")
parser.add_argument("--save-top-candidates-dir", default="", help="Optional folder to write ranked top-candidate JSON files.")
parser.add_argument("--save-top-candidate-count", type=int, default=0, help="Number of ranked candidates to write when --save-top-candidates-dir is set.")
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
parser.add_argument("--no-print-report", action="store_true", help="Write reports without printing the full JSON to stdout.")
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
    help="Allow a visible viewport with more than one cloned candidate environment.",
)
parser.add_argument(
    "--disable-actual-cad-visuals",
    action="store_true",
    help="Render only the CAD-derived cube/sphere proxy instead of the exported Domino STL link meshes.",
)
parser.add_argument("--actual-cad-mesh-dir", default="", help="Optional override for the Domino STL mesh folder.")
parser.add_argument(
    "--foot-collision-mode",
    choices=["linkage-lower-closure", "actual-cad-visual-bottom", "actual-cad-grounded-support"],
    default=None,
    help="Foot contact proxy source. Grounded-support uses CAD foot XY locations with common ground-height support spheres.",
)
parser.add_argument("--terrain-type", choices=["flat", "stairs"], default="flat", help="Static terrain scene used for gait search.")
parser.add_argument("--stairs-step-count", type=int, default=None, help="Number of stair treads when --terrain-type=stairs.")
parser.add_argument("--stairs-step-depth-m", type=float, default=None, help="Step tread depth for stair gait search.")
parser.add_argument("--stairs-step-height-m", type=float, default=None, help="Step rise height for stair gait search.")
parser.add_argument("--stairs-width-m", type=float, default=None, help="Width of the stair obstacle.")
parser.add_argument("--stairs-start-x-m", type=float, default=None, help="Local x position where the first stair starts.")
parser.add_argument("--stairs-top-platform-length-m", type=float, default=None, help="Flat platform length after the last stair.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if not args_cli.headless and args_cli.disable_actual_cad_visuals and not args_cli.allow_proxy_visuals:
    raise SystemExit(
        "Visible Domino gait-search runs should use the actual exported CAD STL visuals. "
        "Remove --disable-actual-cad-visuals, run headless, or pass --allow-proxy-visuals when deliberately "
        "debugging the simplified cube/sphere proxy."
    )

if not args_cli.headless and int(args_cli.candidate_count) != 1 and not args_cli.allow_multi_env_viewport:
    raise SystemExit(
        "Visible Domino CAD inspection runs should show one robot. "
        "Use --candidate-count 1 for visual checks, or pass --allow-multi-env-viewport when deliberately viewing cloned "
        "gait-search candidates."
    )

os.environ.setdefault("WARP_CACHE_PATH", str((Path.cwd() / "simulation" / "isaac" / "out" / "warp_cache").resolve()))

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from domino_action_contract import (  # noqa: E402
    ACTION_JOINT_NAMES,
    EXPECTED_ACTION_COUNT,
    action_group_counts,
    per_leg_action_layout,
)
from domino_cad_linkage_env import (  # noqa: E402
    CAD_LINKAGE_OBSERVATION_DIM,
    DominoCadLinkageEnv,
    DominoCadLinkageEnvCfg,
    projected_gravity_from_quat,
    wrap_angle_rad,
    yaw_from_quat_wxyz,
)
from domino_reference_gait import (  # noqa: E402
    LEG_PHASE_PARAMETER_NAMES,
    SHOULDER_SIGN_PARAMETER_NAMES,
    candidate_with_defaults,
    default_reference_candidate,
    reference_actions_for_base_phases,
)


def default_candidate() -> dict[str, float]:
    return dict(default_reference_candidate())


def lower_waveform_bounds(candidate: dict[str, float]) -> tuple[float, float]:
    lower_bias = float(candidate["lower_bias"])
    lower_amp = abs(float(candidate["lower_amp"]))
    return lower_bias - lower_amp, lower_bias + lower_amp


def constrain_lower_waveform(candidate: dict[str, float]) -> dict[str, float]:
    candidate = dict(candidate)
    lower_amp = min(abs(float(candidate["lower_amp"])), 0.45)
    lower_bias = float(candidate["lower_bias"])
    lower_bias = min(lower_bias, -0.08 - lower_amp)
    lower_bias = max(lower_bias, -0.98 + lower_amp)
    candidate["lower_amp"] = lower_amp
    candidate["lower_bias"] = lower_bias
    return candidate


def generate_candidates(count: int, seed: int) -> list[dict[str, float]]:
    rng = random.Random(seed)
    candidates = [constrain_lower_waveform(default_candidate())]
    phase_choices = [0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0, math.pi, -math.pi / 2.0]
    leg_phase_patterns = [
        [0.0, math.pi, 0.0, math.pi],
        [0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi],
        [0.0, 1.5 * math.pi, math.pi, 0.5 * math.pi],
        [0.0, math.pi, math.pi, 0.0],
    ]
    while len(candidates) < count:
        leg_phases = rng.choice(leg_phase_patterns)
        if rng.random() < 0.6:
            leg_phases = [rng.choice(phase_choices) for _ in range(4)]
        shoulder_signs = [rng.choice([-1.0, 1.0]) for _ in range(4)]
        lower_amp = rng.uniform(0.15, 0.45)
        lower_bias = rng.uniform(-0.95 + lower_amp, -0.10 - lower_amp)
        candidates.append(
            constrain_lower_waveform(
                {
                    "name": f"random_{len(candidates):03d}",
                    "lower_amp": lower_amp,
                    "upper_amp": rng.uniform(0.25, 0.95),
                    "shoulder_amp": rng.uniform(0.0, 0.35),
                    "lower_bias": lower_bias,
                    "upper_bias": rng.uniform(-0.20, 0.35),
                    "shoulder_bias": rng.uniform(-0.10, 0.10),
                    "lower_phase": rng.choice(phase_choices),
                    "upper_phase": rng.choice(phase_choices),
                    "shoulder_phase": rng.choice(phase_choices),
                    "frequency_scale": rng.uniform(0.65, 1.35),
                    **{name: float(leg_phases[index]) for index, name in enumerate(LEG_PHASE_PARAMETER_NAMES)},
                    **{name: float(shoulder_signs[index]) for index, name in enumerate(SHOULDER_SIGN_PARAMETER_NAMES)},
                }
            )
        )
    return candidates[:count]


def clamp_candidate_value(name: str, value: float) -> float:
    ranges = {
        "lower_amp": (0.10, 1.00),
        "upper_amp": (0.10, 1.00),
        "shoulder_amp": (0.00, 0.50),
        "lower_bias": (-0.60, 0.20),
        "upper_bias": (-0.35, 0.50),
        "shoulder_bias": (-0.20, 0.20),
        "frequency_scale": (0.40, 1.60),
    }
    lower, upper = ranges[name]
    return max(lower, min(upper, float(value)))


def wrap_phase(value: float) -> float:
    return ((float(value) + math.pi) % (2.0 * math.pi)) - math.pi


def generate_refined_candidates(base: dict[str, float], count: int, seed: int, scale: float) -> list[dict[str, float]]:
    rng = random.Random(seed)
    base_candidate = dict(default_candidate())
    base_candidate.update(base)
    base_candidate["name"] = f"{base_candidate.get('name', 'candidate')}_base"
    base_candidate = constrain_lower_waveform(base_candidate)
    candidates = [base_candidate]
    scalar_fields = [
        "lower_amp",
        "upper_amp",
        "shoulder_amp",
        "lower_bias",
        "upper_bias",
        "shoulder_bias",
        "frequency_scale",
    ]
    phase_fields = ["lower_phase", "upper_phase", "shoulder_phase", *LEG_PHASE_PARAMETER_NAMES]
    while len(candidates) < count:
        candidate = dict(base_candidate)
        candidate["name"] = f"refined_{len(candidates):03d}"
        for field in scalar_fields:
            candidate[field] = clamp_candidate_value(field, float(base_candidate[field]) + rng.gauss(0.0, 0.20 * scale))
        for field in phase_fields:
            candidate[field] = wrap_phase(float(base_candidate[field]) + rng.gauss(0.0, math.pi * scale))
        for field in SHOULDER_SIGN_PARAMETER_NAMES:
            candidate[field] = float(base_candidate[field])
            if rng.random() < 0.15 * scale:
                candidate[field] *= -1.0
        candidates.append(constrain_lower_waveform(candidate))
    return candidates[:count]


def variant_from_base(base: dict[str, float], name: str) -> dict[str, float]:
    candidate = dict(base)
    candidate["name"] = name
    return candidate


def generate_symmetry_candidates(base: dict[str, float], count: int) -> list[dict[str, float]]:
    base_candidate = candidate_with_defaults(base)
    base_name = str(base_candidate.get("name", "candidate"))
    candidates = [variant_from_base(base_candidate, f"{base_name}_base")]

    for delta in (-0.12, -0.08, -0.04, 0.04, 0.08, 0.12):
        candidate = variant_from_base(base_candidate, f"{base_name}_shoulder_bias_{delta:+.2f}")
        candidate["shoulder_bias"] = clamp_candidate_value("shoulder_bias", float(base_candidate["shoulder_bias"]) + delta)
        candidates.append(candidate)

    for scale in (0.60, 0.75, 0.90, 1.10, 1.25, 1.40):
        candidate = variant_from_base(base_candidate, f"{base_name}_shoulder_amp_x{scale:.2f}")
        candidate["shoulder_amp"] = clamp_candidate_value("shoulder_amp", float(base_candidate["shoulder_amp"]) * scale)
        candidates.append(candidate)

    for delta in (-math.pi / 4.0, -math.pi / 6.0, -math.pi / 12.0, math.pi / 12.0, math.pi / 6.0, math.pi / 4.0):
        candidate = variant_from_base(base_candidate, f"{base_name}_shoulder_phase_{math.degrees(delta):+.0f}deg")
        candidate["shoulder_phase"] = wrap_phase(float(base_candidate["shoulder_phase"]) + delta)
        candidates.append(candidate)

    trim_values = tuple(
        math.radians(value)
        for value in (-20.0, -16.0, -14.0, -12.0, -10.0, -8.0, -6.0, -4.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 20.0)
    )
    for trim in trim_values:
        candidate = variant_from_base(base_candidate, f"{base_name}_right_left_phase_trim_{math.degrees(trim):+.0f}deg")
        candidate["leg_phase_1"] = wrap_phase(float(base_candidate["leg_phase_1"]) + trim)
        candidate["leg_phase_3"] = wrap_phase(float(base_candidate["leg_phase_3"]) - trim)
        candidates.append(candidate)

        candidate = variant_from_base(base_candidate, f"{base_name}_front_rear_phase_trim_{math.degrees(trim):+.0f}deg")
        candidate["leg_phase_0"] = wrap_phase(float(base_candidate["leg_phase_0"]) + trim)
        candidate["leg_phase_2"] = wrap_phase(float(base_candidate["leg_phase_2"]) - trim)
        candidates.append(candidate)

    for trim in (-math.pi / 12.0, math.pi / 12.0):
        for bias_delta in (-0.04, 0.04):
            candidate = variant_from_base(
                base_candidate,
                f"{base_name}_phase_trim_{math.degrees(trim):+.0f}deg_bias_{bias_delta:+.2f}",
            )
            candidate["leg_phase_1"] = wrap_phase(float(base_candidate["leg_phase_1"]) + trim)
            candidate["leg_phase_3"] = wrap_phase(float(base_candidate["leg_phase_3"]) - trim)
            candidate["shoulder_bias"] = clamp_candidate_value("shoulder_bias", float(base_candidate["shoulder_bias"]) + bias_delta)
            candidates.append(candidate)

    return candidates[:count]


def load_candidate(path: str) -> list[dict[str, float]]:
    data = json.loads(Path(path).expanduser().resolve().read_text(encoding="utf-8-sig"))
    if isinstance(data, dict) and "candidate" in data:
        return [candidate_with_defaults(data["candidate"])]
    if isinstance(data, dict) and isinstance(data.get("best"), dict) and "candidate" in data["best"]:
        return [candidate_with_defaults(data["best"]["candidate"])]
    if isinstance(data, dict):
        return [candidate_with_defaults(data)]
    if isinstance(data, list):
        return [candidate_with_defaults(candidate) for candidate in data]
    raise ValueError(f"Unsupported candidate JSON format in {path}.")


def candidate_actions(candidates: list[dict[str, float]], step: int, step_dt: float, device: str) -> torch.Tensor:
    rows = []
    base_phase = 2.0 * math.pi * float(args_cli.gait_frequency_hz) * step * step_dt
    for candidate in candidates:
        row = reference_actions_for_base_phases(candidate, np.asarray([base_phase], dtype=np.float32))[0].tolist()
        if len(row) != EXPECTED_ACTION_COUNT:
            raise RuntimeError(f"Expected scripted gait row to contain {EXPECTED_ACTION_COUNT} actions, found {len(row)}.")
        rows.append(row)
    actions = torch.tensor(rows, dtype=torch.float32, device=device)
    if tuple(actions.shape) != (len(candidates), EXPECTED_ACTION_COUNT):
        raise RuntimeError(f"Expected action tensor shape ({len(candidates)}, {EXPECTED_ACTION_COUNT}), found {tuple(actions.shape)}.")
    return actions


def score_candidate(metrics: dict[str, float]) -> float:
    if metrics["terminated"]:
        return -1000.0 + metrics["final_x_m"]
    constraint_penalty = 0.0
    max_lateral = float(args_cli.score_max_lateral_m)
    max_heading = float(args_cli.score_max_heading_rad)
    if max_lateral > 0.0:
        constraint_penalty += max(0.0, abs(metrics["final_y_m"]) - max_lateral)
    if max_heading > 0.0:
        constraint_penalty += max(0.0, abs(metrics["final_yaw_heading_drift_rad"]) - max_heading)
    constraint_penalty += max(
        0.0,
        float(args_cli.score_min_foot_clearance_m) - metrics["min_peak_foot_clearance_m"],
    )
    constraint_penalty += max(
        0.0,
        float(args_cli.score_min_foot_motion_m) - metrics["min_body_relative_foot_motion_m"],
    )
    return (
        (float(args_cli.score_forward_weight) * metrics["final_x_m"])
        + (float(args_cli.score_clearance_weight) * metrics["max_foot_clearance_m"])
        + (float(args_cli.score_min_foot_clearance_weight) * metrics["min_peak_foot_clearance_m"])
        + (float(args_cli.score_min_foot_motion_weight) * metrics["min_body_relative_foot_motion_m"])
        + (float(args_cli.score_unload_weight) * (4.0 - metrics["mean_foot_contacts"]))
        - (float(args_cli.score_lateral_weight) * abs(metrics["final_y_m"]))
        - (float(args_cli.score_heading_weight) * abs(metrics["final_yaw_heading_drift_rad"]))
        - (float(args_cli.score_tilt_weight) * metrics["max_tilt_deg"])
        - (float(args_cli.score_constraint_penalty) * constraint_penalty)
    )


def candidate_filename(rank: int, candidate: dict[str, float | str]) -> str:
    raw_name = str(candidate.get("name", f"candidate_{rank:02d}"))
    safe_name = "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in raw_name).strip("_")
    return f"rank_{rank:02d}_{safe_name or 'candidate'}.json"


def main() -> None:
    candidate_count = int(args_cli.candidate_count)
    candidate_offset = max(int(args_cli.candidate_offset), 0)
    if args_cli.load_candidate:
        if candidate_offset:
            raise ValueError("--candidate-offset is only supported with --symmetry-candidate.")
        candidates = load_candidate(args_cli.load_candidate)
    elif args_cli.refine_candidate:
        if candidate_offset:
            raise ValueError("--candidate-offset is only supported with --symmetry-candidate.")
        candidates = generate_refined_candidates(
            load_candidate(args_cli.refine_candidate)[0],
            candidate_count,
            int(args_cli.seed),
            float(args_cli.refine_scale),
        )
    elif args_cli.symmetry_candidate:
        symmetry_candidates = generate_symmetry_candidates(
            load_candidate(args_cli.symmetry_candidate)[0],
            candidate_offset + candidate_count,
        )
        candidates = symmetry_candidates[candidate_offset : candidate_offset + candidate_count]
    else:
        if candidate_offset:
            raise ValueError("--candidate-offset is only supported with --symmetry-candidate.")
        candidates = generate_candidates(candidate_count, int(args_cli.seed))
    if len(candidates) < 1:
        raise RuntimeError("At least one gait candidate is required.")

    cfg = DominoCadLinkageEnvCfg()
    cfg.scene.num_envs = len(candidates)
    cfg.sim.device = args_cli.device
    cfg.seed = int(args_cli.seed)
    cfg.include_actual_cad_visuals = not bool(args_cli.disable_actual_cad_visuals)
    cfg.actual_cad_mesh_dir = str(args_cli.actual_cad_mesh_dir)
    cfg.action_scale_deg = float(args_cli.action_scale_deg)
    if args_cli.servo_target_rate_limit_deg_s is not None:
        cfg.servo_target_rate_limit_deg_s = float(args_cli.servo_target_rate_limit_deg_s)
    if args_cli.foot_collision_mode is not None:
        cfg.foot_contact_mode = str(args_cli.foot_collision_mode).replace("-", "_")
        cfg.use_actual_cad_foot_collision = args_cli.foot_collision_mode == "actual-cad-visual-bottom"
    cfg.terrain_type = str(args_cli.terrain_type)
    if args_cli.stairs_step_count is not None:
        cfg.stairs_step_count = int(args_cli.stairs_step_count)
    if args_cli.stairs_step_depth_m is not None:
        cfg.stairs_step_depth_m = float(args_cli.stairs_step_depth_m)
    if args_cli.stairs_step_height_m is not None:
        cfg.stairs_step_height_m = float(args_cli.stairs_step_height_m)
    if args_cli.stairs_width_m is not None:
        cfg.stairs_width_m = float(args_cli.stairs_width_m)
    if args_cli.stairs_start_x_m is not None:
        cfg.stairs_start_x_m = float(args_cli.stairs_start_x_m)
    if args_cli.stairs_top_platform_length_m is not None:
        cfg.stairs_top_platform_length_m = float(args_cli.stairs_top_platform_length_m)
    cfg.command_x_m_s = float(args_cli.command_x_m_s)
    cfg.command_y_m_s = float(args_cli.command_y_m_s)
    cfg.command_yaw_rad_s = float(args_cli.command_yaw_rad_s)
    cfg.gait_frequency_hz = float(args_cli.gait_frequency_hz)
    cfg.episode_length_s = float(args_cli.episode_length_s)
    if args_cli.floating_height_m is not None:
        cfg.floating_height_m = float(args_cli.floating_height_m)
    if args_cli.min_height_m is not None:
        cfg.min_height_m = float(args_cli.min_height_m)
    if args_cli.max_tilt_deg is not None:
        cfg.max_tilt_deg = float(args_cli.max_tilt_deg)
    cfg.foot_contact_reward_scale = 0.0
    cfg.command_progress_reward_scale = 20.0
    cfg.command_velocity_tracking_reward_scale = 4.0
    cfg.gait_contact_reward_scale = 1.0
    cfg.stance_contact_reward_scale = 0.5
    cfg.swing_contact_penalty_scale = -1.5
    cfg.foot_clearance_reward_scale = 1.0

    env = DominoCadLinkageEnv(cfg)
    observations, _ = env.reset()
    action_dim = gym.spaces.flatdim(env.single_action_space)
    if action_dim != EXPECTED_ACTION_COUNT:
        raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} action dimensions, found {action_dim}.")
    if observations["policy"].shape[-1] != CAD_LINKAGE_OBSERVATION_DIM:
        raise RuntimeError(
            f"Expected {CAD_LINKAGE_OBSERVATION_DIM} observations, found {observations['policy'].shape[-1]}."
        )

    zero_actions = torch.zeros((env.num_envs, EXPECTED_ACTION_COUNT), dtype=torch.float32, device=env.device)
    for settle_step in range(max(int(args_cli.settle_steps), 0)):
        observations, rewards, terminated_tensor, timeout_tensor, _ = env.step(zero_actions)
        if (
            not torch.isfinite(observations["policy"]).all()
            or not torch.isfinite(rewards).all()
            or bool(torch.any(terminated_tensor).detach().cpu())
            or bool(torch.any(timeout_tensor).detach().cpu())
        ):
            terminated_envs = torch.nonzero(terminated_tensor, as_tuple=False).flatten().detach().cpu().tolist()
            timeout_envs = torch.nonzero(timeout_tensor, as_tuple=False).flatten().detach().cpu().tolist()
            raise RuntimeError(
                "Neutral-pose settling failed before gait search metrics began: "
                f"step={settle_step + 1}, terminated_envs={terminated_envs}, timeout_envs={timeout_envs}."
            )
    if hasattr(env, "episode_length_buf"):
        env.episode_length_buf.zero_()

    initial_positions = [env._body_reference_state(env_index)[0].copy() for env_index in range(env.num_envs)]
    initial_yaws = [
        yaw_from_quat_wxyz(env._body_reference_state(env_index)[1])
        for env_index in range(env.num_envs)
    ]
    initial_body_relative_visual_feet = [
        env._actual_cad_visual_foot_positions(env_index, rendered=True) - initial_positions[env_index][None, :]
        for env_index in range(env.num_envs)
    ]
    total_reward = torch.zeros(env.num_envs, device=env.device)
    terminated = [False for _ in candidates]
    timeout = [False for _ in candidates]
    foot_contact_sum = np.zeros(env.num_envs, dtype=np.float64)
    max_foot_clearance = np.zeros(env.num_envs, dtype=np.float64)
    max_foot_clearance_by_foot = np.zeros((env.num_envs, 4), dtype=np.float64)
    peak_foot_clearance_local_phase = np.zeros((env.num_envs, 4), dtype=np.float64)
    max_body_relative_foot_motion = np.zeros((env.num_envs, 4), dtype=np.float64)
    min_body_height = np.full(env.num_envs, np.inf, dtype=np.float64)
    max_tilt_deg = np.zeros(env.num_envs, dtype=np.float64)
    velocity_sum = np.zeros((env.num_envs, 3), dtype=np.float64)
    abs_yaw_heading_drift_sum = np.zeros(env.num_envs, dtype=np.float64)

    for step in range(int(args_cli.steps)):
        actions = candidate_actions(candidates, step, env.step_dt, env.device)
        observations, rewards, terminated_tensor, timeout_tensor, _ = env.step(actions)
        total_reward += rewards
        if not torch.isfinite(observations["policy"]).all() or not torch.isfinite(rewards).all():
            raise RuntimeError("Non-finite observation or reward during gait search.")
        for env_index in range(env.num_envs):
            terminated[env_index] = terminated[env_index] or bool(terminated_tensor[env_index].detach().cpu())
            timeout[env_index] = timeout[env_index] or bool(timeout_tensor[env_index].detach().cpu())
            position, orientation, linear_velocity, _ = env._body_reference_state(env_index)
            velocity_sum[env_index] += linear_velocity
            target_yaw = initial_yaws[env_index] + float(args_cli.command_yaw_rad_s) * float(step + 1) * float(env.step_dt)
            abs_yaw_heading_drift_sum[env_index] += abs(wrap_angle_rad(yaw_from_quat_wxyz(orientation) - target_yaw))
            min_body_height[env_index] = min(min_body_height[env_index], float(position[2] - env._env_origins_np[env_index][2]))
            projected_gravity = projected_gravity_from_quat(orientation)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(projected_gravity[2])))))
            max_tilt_deg[env_index] = max(max_tilt_deg[env_index], tilt)
            foot_positions, foot_radii = env._reward_foot_positions_and_radii(env_index)
            foot_contacts = env._foot_contact_flags_np(foot_positions, env_index, radii_m=foot_radii)
            foot_contact_sum[env_index] += float(sum(foot_contacts))
            clearance = env._foot_ground_clearance_np(foot_positions, env_index, radii_m=foot_radii)
            max_foot_clearance[env_index] = max(max_foot_clearance[env_index], float(clearance.max()))
            improved_clearance = clearance > max_foot_clearance_by_foot[env_index]
            if np.any(improved_clearance):
                candidate = candidates[env_index]
                base_phase = 2.0 * math.pi * float(args_cli.gait_frequency_hz) * float(step) * float(env.step_dt)
                for foot_index in np.flatnonzero(improved_clearance):
                    local_phase = (
                        base_phase * float(candidate["frequency_scale"])
                        + float(candidate[f"leg_phase_{int(foot_index)}"])
                    )
                    peak_foot_clearance_local_phase[env_index, foot_index] = wrap_phase(local_phase)
            max_foot_clearance_by_foot[env_index] = np.maximum(
                max_foot_clearance_by_foot[env_index],
                np.maximum(clearance, 0.0),
            )
            visual_feet = env._actual_cad_visual_foot_positions(env_index, rendered=True)
            body_relative_visual_feet = visual_feet - position[None, :]
            max_body_relative_foot_motion[env_index] = np.maximum(
                max_body_relative_foot_motion[env_index],
                np.linalg.norm(
                    body_relative_visual_feet - initial_body_relative_visual_feet[env_index],
                    axis=1,
                ),
            )

    results = []
    for env_index, candidate in enumerate(candidates):
        final_position, final_orientation, _, _ = env._body_reference_state(env_index)
        displacement = final_position - initial_positions[env_index]
        target_yaw = initial_yaws[env_index] + float(args_cli.command_yaw_rad_s) * float(args_cli.steps) * float(env.step_dt)
        final_yaw_heading_drift = wrap_angle_rad(yaw_from_quat_wxyz(final_orientation) - target_yaw)
        metrics = {
            "candidate_index": env_index,
            "candidate": candidate,
            "score": 0.0,
            "terminated": bool(terminated[env_index]),
            "timeout": bool(timeout[env_index]),
            "mean_reward": round(float((total_reward[env_index] / max(int(args_cli.steps), 1)).detach().cpu()), 6),
            "final_x_m": float(displacement[0]),
            "final_y_m": float(displacement[1]),
            "final_z_m": float(displacement[2]),
            "final_yaw_heading_drift_rad": float(final_yaw_heading_drift),
            "mean_abs_yaw_heading_drift_rad": float(abs_yaw_heading_drift_sum[env_index] / max(int(args_cli.steps), 1)),
            "mean_velocity_m_s": [
                float(value) for value in (velocity_sum[env_index] / max(int(args_cli.steps), 1)).tolist()
            ],
            "mean_foot_contacts": float(foot_contact_sum[env_index] / max(int(args_cli.steps), 1)),
            "max_foot_clearance_m": float(max_foot_clearance[env_index]),
            "peak_foot_clearance_by_foot_m": [
                float(value) for value in max_foot_clearance_by_foot[env_index].tolist()
            ],
            "peak_foot_clearance_local_phase_by_foot_rad": [
                float(value) for value in peak_foot_clearance_local_phase[env_index].tolist()
            ],
            "min_peak_foot_clearance_m": float(np.min(max_foot_clearance_by_foot[env_index])),
            "max_body_relative_foot_motion_by_foot_m": [
                float(value) for value in max_body_relative_foot_motion[env_index].tolist()
            ],
            "min_body_relative_foot_motion_m": float(np.min(max_body_relative_foot_motion[env_index])),
            "min_body_height_m": float(min_body_height[env_index]),
            "max_tilt_deg": float(max_tilt_deg[env_index]),
        }
        metrics["score"] = float(score_candidate(metrics))
        results.append(metrics)

    results.sort(key=lambda item: item["score"], reverse=True)
    report = {
        "status": "passed",
        "steps": int(args_cli.steps),
        "settle_steps": int(args_cli.settle_steps),
        "candidate_count": len(candidates),
        "geometry": env._linkage["geometry"],
        "visual_fidelity": env._linkage.get("visual_fidelity"),
        "actual_cad_visual": env._linkage.get("actual_cad_visual"),
        "cad_source": env._linkage.get("cad_source"),
        "actual_cad_visuals": env._linkage.get("actual_cad_visuals"),
        "visual_geometry_counts": env._linkage.get("visual_geometry_counts"),
        "actual_cad_foot_collision": env._linkage.get("actual_cad_foot_collision"),
        "actuator_model": env._linkage.get("actuator_model"),
        "joint_separation": env.joint_separation_report(),
        "terrain": getattr(env, "_terrain_report", None),
        "action_dim": action_dim,
        "action_names": ACTION_JOINT_NAMES,
        "action_group_counts": action_group_counts(),
        "per_leg_action_layout": per_leg_action_layout(),
        "observation_dim": observations["policy"].shape[-1],
        "config": {
            "action_scale_deg": float(args_cli.action_scale_deg),
            "servo_target_rate_limit_deg_s": float(cfg.servo_target_rate_limit_deg_s),
            "command_x_m_s": float(args_cli.command_x_m_s),
            "command_y_m_s": float(args_cli.command_y_m_s),
            "command_yaw_rad_s": float(args_cli.command_yaw_rad_s),
            "gait_frequency_hz": float(args_cli.gait_frequency_hz),
            "episode_length_s": float(args_cli.episode_length_s),
            "settle_steps": int(args_cli.settle_steps),
            "score_forward_weight": float(args_cli.score_forward_weight),
            "score_lateral_weight": float(args_cli.score_lateral_weight),
            "score_clearance_weight": float(args_cli.score_clearance_weight),
            "score_min_foot_clearance_weight": float(args_cli.score_min_foot_clearance_weight),
            "score_min_foot_motion_weight": float(args_cli.score_min_foot_motion_weight),
            "score_min_foot_clearance_m": float(args_cli.score_min_foot_clearance_m),
            "score_min_foot_motion_m": float(args_cli.score_min_foot_motion_m),
            "score_unload_weight": float(args_cli.score_unload_weight),
            "score_tilt_weight": float(args_cli.score_tilt_weight),
            "score_heading_weight": float(args_cli.score_heading_weight),
            "score_max_lateral_m": float(args_cli.score_max_lateral_m),
            "score_max_heading_rad": float(args_cli.score_max_heading_rad),
            "score_constraint_penalty": float(args_cli.score_constraint_penalty),
            "foot_collision_mode": str(args_cli.foot_collision_mode or "linkage-lower-closure"),
            "terrain_type": str(cfg.terrain_type),
            "stairs_step_count": int(cfg.stairs_step_count),
            "stairs_step_depth_m": float(cfg.stairs_step_depth_m),
            "stairs_step_height_m": float(cfg.stairs_step_height_m),
            "stairs_width_m": float(cfg.stairs_width_m),
            "stairs_start_x_m": float(cfg.stairs_start_x_m),
            "stairs_top_platform_length_m": float(cfg.stairs_top_platform_length_m),
        },
        "best": results[0],
        "results": results,
    }

    if args_cli.save_best_candidate:
        best_path = Path(args_cli.save_best_candidate).expanduser().resolve()
        best_path.parent.mkdir(parents=True, exist_ok=True)
        best_path.write_text(json.dumps({"candidate": results[0]["candidate"], "metrics": results[0]}, indent=2), encoding="utf-8")

    if args_cli.save_top_candidates_dir:
        top_dir = Path(args_cli.save_top_candidates_dir).expanduser().resolve()
        top_dir.mkdir(parents=True, exist_ok=True)
        top_count = int(args_cli.save_top_candidate_count)
        if top_count <= 0:
            top_count = min(4, len(results))
        for rank, result in enumerate(results[:top_count], start=1):
            candidate_path = top_dir / candidate_filename(rank, result["candidate"])
            candidate_path.write_text(
                json.dumps({"candidate": result["candidate"], "metrics": result}, indent=2),
                encoding="utf-8",
            )

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    if not args_cli.no_print_report:
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
