"""Generate the next Domino CAD-linkage policy experiment commands.

The generated commands are intentionally conservative: they keep actual CAD
visuals enabled, keep the 12-action reference curriculum available, and write
new artifacts under the ignored Isaac output folder.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[4]
PIN_LINKAGE_DIR = Path("simulation/isaac/prototypes/pin_linkage")
OUT_ROOT = Path("simulation/isaac/out/cad_identity/next_policy")


BASELINES: dict[str, dict[str, Any]] = {
    "fastest_70deg_2p25hz": {
        "checkpoint": Path(
            "simulation/isaac/out/cad_identity/rsl_bc_weighted_scale70_freq225/"
            "domino_cad_linkage_direct/2026-06-21_00-30-08_smoke/model_bc.pt"
        ),
        "candidate": Path("simulation/isaac/out/cad_identity/teacher_grid/teacher_random001_scale70_freq225.json"),
        "action_scale_deg": 70.0,
        "gait_frequency_hz": 2.25,
        "description": "Fastest retained behavior-cloned baseline.",
    },
    "trim_m6_balance": {
        "checkpoint": Path(
            "simulation/isaac/out/cad_identity/rsl_bc_symmetry_trim_m6_scale70_freq225/"
            "domino_cad_linkage_direct/2026-06-28_15-38-09_smoke/model_bc.pt"
        ),
        "candidate": Path(
            "simulation/isaac/out/cad_identity/teacher_grid/symmetry_fine_trim_top/"
            "rank_01_random_001_right_left_phase_trim_-6deg.json"
        ),
        "action_scale_deg": 70.0,
        "gait_frequency_hz": 2.25,
        "description": "Intermediate symmetry baseline with lower heading drift than random_001.",
    },
    "trim_m15_low_yaw": {
        "checkpoint": Path(
            "simulation/isaac/out/cad_identity/rsl_bc_symmetry_trim_m15_scale70_freq225/"
            "domino_cad_linkage_direct/2026-06-28_15-21-54_smoke/model_bc.pt"
        ),
        "candidate": Path(
            "simulation/isaac/out/cad_identity/teacher_grid/symmetry_base32_top/"
            "rank_01_random_001_right_left_phase_trim_-15deg.json"
        ),
        "action_scale_deg": 70.0,
        "gait_frequency_hz": 2.25,
        "description": "Low-yaw behavior-cloned comparison baseline.",
    },
    "lower_drift_60deg_2hz": {
        "checkpoint": Path(
            "simulation/isaac/out/cad_identity/rsl_bc_weighted_scale60_freq20/"
            "domino_cad_linkage_direct/2026-06-20_18-37-16_smoke/model_bc.pt"
        ),
        "candidate": Path("simulation/isaac/out/cad_identity/teacher_grid/teacher_random001_scale70_freq225.json"),
        "action_scale_deg": 60.0,
        "gait_frequency_hz": 2.0,
        "description": "Lower-lateral-drift weighted BC comparison baseline.",
    },
    "model20_balanced_candidate": {
        "checkpoint": Path(
            "simulation/isaac/out/cad_identity/next_policy/fastest_70deg_heading_refine/"
            "domino_cad_linkage_direct/2026-06-28_23-36-28_smoke/model_20.pt"
        ),
        "candidate": Path("simulation/isaac/out/cad_identity/teacher_grid/teacher_random001_scale70_freq225.json"),
        "action_scale_deg": 70.0,
        "gait_frequency_hz": 2.25,
        "description": "Near-fastest balanced PPO candidate with lower yaw/lateral drift than the retained 70-degree BC baseline.",
    },
    "model59_forward_candidate": {
        "checkpoint": Path(
            "simulation/isaac/out/cad_identity/next_policy/model20_forward_preserving_straight_refine/"
            "domino_cad_linkage_direct/2026-06-29_12-17-51_smoke/model_59.pt"
        ),
        "candidate": Path("simulation/isaac/out/cad_identity/teacher_grid/teacher_random001_scale70_freq225.json"),
        "action_scale_deg": 70.0,
        "gait_frequency_hz": 2.25,
        "description": "Forward-distance PPO candidate that beats retained 70-degree BC on forward/lateral displacement but has worse heading drift.",
    },
}


EXPERIMENTS: dict[str, dict[str, Any]] = {
    "model59_heading_recovery_refine": {
        "baseline": "model59_forward_candidate",
        "seed": 7306,
        "num_envs": 16,
        "iterations": 40,
        "num_steps_per_env": 32,
        "init_noise_std": 0.04,
        "ppo_learning_rate": 3.0e-5,
        "ppo_entropy_coef": 0.0006,
        "reward": {
            "command_progress_reward_scale": 3.6,
            "command_velocity_reward_scale": -4.0,
            "command_velocity_tracking_reward_scale": 1.5,
            "lateral_drift_reward_scale": -850.0,
            "yaw_drift_reward_scale": -4.0,
            "command_yaw_reward_scale": -2.0,
            "gait_contact_reward_scale": 2.0,
            "stance_contact_reward_scale": 0.5,
            "swing_contact_penalty_scale": -4.0,
            "foot_clearance_reward_scale": 3.5,
            "foot_contact_reward_scale": 0.0,
            "reference_action_tracking_reward_scale": 2.5,
            "reference_action_tracking_sigma": 0.50,
            "reference_action_mse_reward_scale": -4.0,
        },
        "reason": "Default next PPO run: continue from model_59 and recover heading/lateral stability without deleting the forward-distance gain.",
    },
    "model20_forward_preserving_straight_refine": {
        "baseline": "model20_balanced_candidate",
        "seed": 7305,
        "num_envs": 16,
        "iterations": 40,
        "num_steps_per_env": 32,
        "init_noise_std": 0.05,
        "ppo_learning_rate": 4.0e-5,
        "ppo_entropy_coef": 0.0008,
        "reward": {
            "command_progress_reward_scale": 3.8,
            "command_velocity_reward_scale": -4.0,
            "command_velocity_tracking_reward_scale": 1.5,
            "lateral_drift_reward_scale": -650.0,
            "yaw_drift_reward_scale": -2.5,
            "command_yaw_reward_scale": -1.5,
            "gait_contact_reward_scale": 2.0,
            "stance_contact_reward_scale": 0.5,
            "swing_contact_penalty_scale": -4.0,
            "foot_clearance_reward_scale": 3.5,
            "foot_contact_reward_scale": 0.0,
            "reference_action_tracking_reward_scale": 3.0,
            "reference_action_tracking_sigma": 0.45,
            "reference_action_mse_reward_scale": -5.0,
        },
        "reason": "Default next run: continue from model_20 while preserving forward distance more strongly than the stricter straightness diagnostic.",
    },
    "trim_m6_yaw_lateral_refine": {
        "baseline": "trim_m6_balance",
        "seed": 7301,
        "num_envs": 12,
        "iterations": 60,
        "num_steps_per_env": 32,
        "init_noise_std": 0.12,
        "ppo_learning_rate": 1.0e-4,
        "ppo_entropy_coef": 0.002,
        "reward": {
            "command_progress_reward_scale": 3.0,
            "command_velocity_reward_scale": -4.0,
            "command_velocity_tracking_reward_scale": 1.25,
            "lateral_drift_reward_scale": -600.0,
            "yaw_drift_reward_scale": -2.0,
            "command_yaw_reward_scale": -1.5,
            "gait_contact_reward_scale": 2.0,
            "stance_contact_reward_scale": 0.5,
            "swing_contact_penalty_scale": -4.0,
            "foot_clearance_reward_scale": 3.5,
            "foot_contact_reward_scale": 0.0,
            "reference_action_tracking_reward_scale": 3.0,
            "reference_action_tracking_sigma": 0.45,
            "reference_action_mse_reward_scale": -5.0,
        },
        "reason": "First next run: try to keep the -6 deg heading improvement while recovering forward command tracking.",
    },
    "fastest_70deg_heading_refine": {
        "baseline": "fastest_70deg_2p25hz",
        "seed": 7302,
        "num_envs": 16,
        "iterations": 80,
        "num_steps_per_env": 32,
        "init_noise_std": 0.10,
        "ppo_learning_rate": 8.0e-5,
        "ppo_entropy_coef": 0.0015,
        "reward": {
            "command_progress_reward_scale": 3.0,
            "command_velocity_reward_scale": -4.0,
            "command_velocity_tracking_reward_scale": 1.5,
            "lateral_drift_reward_scale": -700.0,
            "yaw_drift_reward_scale": -3.0,
            "command_yaw_reward_scale": -1.5,
            "gait_contact_reward_scale": 2.0,
            "stance_contact_reward_scale": 0.5,
            "swing_contact_penalty_scale": -4.0,
            "foot_clearance_reward_scale": 3.5,
            "foot_contact_reward_scale": 0.0,
            "reference_action_tracking_reward_scale": 2.5,
            "reference_action_tracking_sigma": 0.50,
            "reference_action_mse_reward_scale": -4.0,
        },
        "reason": "Try to reduce the fastest baseline's yaw drift without deleting forward motion.",
    },
    "trim_m15_forward_recovery": {
        "baseline": "trim_m15_low_yaw",
        "seed": 7303,
        "num_envs": 12,
        "iterations": 60,
        "num_steps_per_env": 32,
        "init_noise_std": 0.12,
        "ppo_learning_rate": 1.0e-4,
        "ppo_entropy_coef": 0.002,
        "reward": {
            "command_progress_reward_scale": 4.0,
            "command_velocity_reward_scale": -3.5,
            "command_velocity_tracking_reward_scale": 1.5,
            "lateral_drift_reward_scale": -450.0,
            "yaw_drift_reward_scale": -1.0,
            "command_yaw_reward_scale": -1.0,
            "gait_contact_reward_scale": 2.0,
            "stance_contact_reward_scale": 0.5,
            "swing_contact_penalty_scale": -4.0,
            "foot_clearance_reward_scale": 3.5,
            "foot_contact_reward_scale": 0.0,
            "reference_action_tracking_reward_scale": 2.0,
            "reference_action_tracking_sigma": 0.55,
            "reference_action_mse_reward_scale": -3.0,
        },
        "reason": "Try to recover forward displacement from the low-yaw teacher while keeping it straighter than random_001.",
    },
}


def repo_relative(path: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def rel(path: Path) -> str:
    return path.as_posix()


def check_required_artifacts(selected: list[str]) -> None:
    missing: list[str] = []
    for experiment_name in selected:
        experiment = EXPERIMENTS[experiment_name]
        baseline = BASELINES[str(experiment["baseline"])]
        for key in ("checkpoint", "candidate"):
            path = REPO_ROOT / baseline[key]
            if not path.exists():
                missing.append(f"{experiment_name}:{key}:{rel(baseline[key])}")
    if missing:
        raise SystemExit("Missing required baseline artifacts:\n" + "\n".join(f"- {item}" for item in missing))


def ps_quote(value: str) -> str:
    return "'" + value.replace("'", "''") + "'"


def command_lines(executable: str, script: Path, args: list[str]) -> list[str]:
    lines = [f"& {executable} {rel(script)} `"]
    for index, arg in enumerate(args):
        suffix = " `" if index < len(args) - 1 else ""
        lines.append(f"  {arg}{suffix}")
    return lines


def train_args(experiment_name: str, experiment: dict[str, Any]) -> list[str]:
    baseline = BASELINES[str(experiment["baseline"])]
    log_root = OUT_ROOT / experiment_name
    report_path = OUT_ROOT / f"rsl_train_{experiment_name}.json"
    args = [
        "--headless",
        f"--num-envs {int(experiment['num_envs'])}",
        f"--iterations {int(experiment['iterations'])}",
        f"--num-steps-per-env {int(experiment['num_steps_per_env'])}",
        f"--seed {int(experiment['seed'])}",
        f"--resume-checkpoint {ps_quote(rel(baseline['checkpoint']))}",
        f"--init-noise-std {float(experiment['init_noise_std']):g}",
        f"--ppo-learning-rate {float(experiment['ppo_learning_rate']):g}",
        f"--ppo-entropy-coef {float(experiment['ppo_entropy_coef']):g}",
        f"--action-scale-deg {float(baseline['action_scale_deg']):g}",
        "--command-x-m-s 0.08",
        "--command-y-m-s 0.0",
        "--command-yaw-rad-s 0.0",
        f"--gait-frequency-hz {float(baseline['gait_frequency_hz']):g}",
        "--episode-length-s 6.0",
        f"--reference-gait-candidate {ps_quote(rel(baseline['candidate']))}",
        "--include-reference-actions-in-observation",
    ]
    for key, value in experiment["reward"].items():
        cli_name = key.replace("_", "-")
        args.append(f"--{cli_name} {float(value):g}")
    args.extend(
        [
            f"--log-root {ps_quote(rel(log_root))}",
            f"--report-path {ps_quote(rel(report_path))}",
        ]
    )
    return args


def play_args(experiment_name: str, experiment: dict[str, Any]) -> list[str]:
    baseline = BASELINES[str(experiment["baseline"])]
    log_root = OUT_ROOT / experiment_name
    report_path = OUT_ROOT / f"rsl_play_{experiment_name}.json"
    args = [
        "--headless",
        "--num-envs 4",
        "--steps 240",
        f"--seed {int(experiment['seed']) + 1000}",
        f"--action-scale-deg {float(baseline['action_scale_deg']):g}",
        "--command-x-m-s 0.08",
        "--command-y-m-s 0.0",
        "--command-yaw-rad-s 0.0",
        f"--gait-frequency-hz {float(baseline['gait_frequency_hz']):g}",
        "--episode-length-s 6.0",
        f"--reference-gait-candidate {ps_quote(rel(baseline['candidate']))}",
        "--include-reference-actions-in-observation",
    ]
    for key, value in experiment["reward"].items():
        if key.startswith("reference_action_bc_"):
            continue
        cli_name = key.replace("_", "-")
        args.append(f"--{cli_name} {float(value):g}")
    args.extend(
        [
            f"--log-root {ps_quote(rel(log_root))}",
            f"--report-path {ps_quote(rel(report_path))}",
        ]
    )
    return args


def build_manifest(selected: list[str]) -> dict[str, Any]:
    manifest: dict[str, Any] = {
        "status": "planned",
        "note": "Run one train/play pair at a time, then compare against verify_domino_cad_linkage_reports.py baselines.",
        "output_root": rel(OUT_ROOT),
        "experiments": {},
    }
    for name in selected:
        experiment = EXPERIMENTS[name]
        baseline = BASELINES[str(experiment["baseline"])]
        manifest["experiments"][name] = {
            "reason": experiment["reason"],
            "baseline": experiment["baseline"],
            "baseline_description": baseline["description"],
            "checkpoint": rel(baseline["checkpoint"]),
            "candidate": rel(baseline["candidate"]),
            "train_report": rel(OUT_ROOT / f"rsl_train_{name}.json"),
            "play_report": rel(OUT_ROOT / f"rsl_play_{name}.json"),
        }
    return manifest


def build_powershell(selected: list[str], isaac_python_var: str, python_var: str) -> str:
    lines = [
        "# Domino CAD-linkage next policy experiments.",
        "# Run from the repository root. Run one train/play pair at a time.",
        f"$IsaacPython = {ps_quote(isaac_python_var)}",
        f"$Python = {ps_quote(python_var)}",
        "",
        "& $Python simulation/isaac/prototypes/pin_linkage/verify_domino_cad_linkage_reports.py",
        "",
    ]
    for name in selected:
        experiment = EXPERIMENTS[name]
        lines.extend([f"# {name}: {experiment['reason']}", "# Train"])
        lines.extend(command_lines("$IsaacPython", PIN_LINKAGE_DIR / "run_domino_cad_linkage_rsl_rl_train.py", train_args(name, experiment)))
        lines.extend(["", "# Play back newest checkpoint from this experiment"])
        lines.extend(command_lines("$IsaacPython", PIN_LINKAGE_DIR / "run_domino_cad_linkage_rsl_rl_play.py", play_args(name, experiment)))
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate next Domino CAD-linkage policy experiment commands.")
    parser.add_argument(
        "--experiment",
        action="append",
        choices=sorted(EXPERIMENTS),
        help="Experiment to include. Repeat to include more. Default: first recommended experiment only.",
    )
    parser.add_argument("--all", action="store_true", help="Include every planned experiment.")
    parser.add_argument("--isaac-python", default="<path-to-isaac-python>", help="Value assigned to $IsaacPython in PowerShell output.")
    parser.add_argument("--python", default="<python>", help="Value assigned to $Python in PowerShell output.")
    parser.add_argument("--write-powershell", default="", help="Optional path to write the generated PowerShell script.")
    parser.add_argument("--write-json", default="", help="Optional path to write a JSON manifest.")
    args = parser.parse_args()

    selected = sorted(EXPERIMENTS) if args.all else (args.experiment or ["model59_heading_recovery_refine"])
    check_required_artifacts(selected)
    manifest = build_manifest(selected)
    powershell = build_powershell(selected, args.isaac_python, args.python)

    if args.write_json:
        output_path = (REPO_ROOT / args.write_json).resolve() if not Path(args.write_json).is_absolute() else Path(args.write_json).resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    if args.write_powershell:
        output_path = (
            (REPO_ROOT / args.write_powershell).resolve()
            if not Path(args.write_powershell).is_absolute()
            else Path(args.write_powershell).resolve()
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(powershell, encoding="utf-8")

    print(json.dumps(manifest, indent=2), flush=True)
    print("\n--- powershell ---", flush=True)
    print(powershell, flush=True)


if __name__ == "__main__":
    main()
