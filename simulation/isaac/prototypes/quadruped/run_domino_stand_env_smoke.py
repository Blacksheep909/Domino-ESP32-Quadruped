"""Smoke-run the DominoStandEnv DirectRLEnv without starting training."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import traceback

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Run a Domino stand DirectRLEnv smoke test.")
parser.add_argument("--usd-path", required=True, help="Path to the imported floating-base quadruped USD.")
parser.add_argument("--steps", type=int, default=300, help="Number of environment steps to run.")
parser.add_argument("--num-envs", type=int, default=1, help="Number of cloned environments.")
parser.add_argument("--seed", type=int, default=42, help="Environment seed.")
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
parser.add_argument(
    "--graceful-close",
    action="store_true",
    help="Call SimulationApp.close() before exit. Disabled by default because it can hang on some Windows setups.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

usd_path = Path(args_cli.usd_path).expanduser().resolve()
if not usd_path.exists():
    raise FileNotFoundError(f"USD path does not exist: {usd_path}")
os.environ["DOMINO_QUADRUPED_USD"] = str(usd_path)
os.environ.setdefault("WARP_CACHE_PATH", str((Path.cwd() / "simulation" / "isaac" / "out" / "warp_cache").resolve()))

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

from domino_quadruped_cfg import (  # noqa: E402
    EXPECTED_ACTION_COUNT,
    POLICY_OBSERVATION_DIM,
    action_group_counts,
)
from domino_stand_env import DominoStandEnv, DominoStandEnvCfg  # noqa: E402


def tensor_list(value: torch.Tensor) -> list[float]:
    return [round(float(v), 6) for v in value.detach().cpu().flatten()]


def main():
    cfg = DominoStandEnvCfg()
    cfg.scene.num_envs = int(args_cli.num_envs)
    cfg.sim.device = args_cli.device
    cfg.seed = int(args_cli.seed)

    env = DominoStandEnv(cfg)
    observations, _ = env.reset()
    action_dim = gym.spaces.flatdim(env.single_action_space)
    if action_dim != EXPECTED_ACTION_COUNT:
        raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} action dimensions, found {action_dim}.")
    if observations["policy"].shape[-1] != POLICY_OBSERVATION_DIM:
        raise RuntimeError(
            f"Expected {POLICY_OBSERVATION_DIM} policy observations, found {observations['policy'].shape[-1]}."
        )
    actions = torch.zeros(env.num_envs, action_dim, device=env.device)

    total_reward = torch.zeros(env.num_envs, device=env.device)
    terminated_count = 0
    truncated_count = 0
    min_root_height = float("inf")
    max_root_tilt = 0.0
    foot_contact_sum = 0.0
    max_foot_contact_force = 0.0

    for _ in range(args_cli.steps):
        observations, rewards, terminated, truncated, _ = env.step(actions)
        total_reward += rewards
        terminated_count += int(torch.count_nonzero(terminated).detach().cpu())
        truncated_count += int(torch.count_nonzero(truncated).detach().cpu())
        min_root_height = min(min_root_height, float(torch.min(env._robot.data.root_pos_w[:, 2]).detach().cpu()))
        tilt = torch.acos(torch.clamp(-env._robot.data.projected_gravity_b[:, 2], -1.0, 1.0))
        max_root_tilt = max(max_root_tilt, float(torch.rad2deg(torch.max(tilt)).detach().cpu()))
        foot_contact_flags = env._get_foot_contact_flags()
        foot_contact_forces = env._get_foot_contact_forces()
        foot_contact_sum += float(torch.sum(foot_contact_flags).detach().cpu())
        max_foot_contact_force = max(
            max_foot_contact_force, float(torch.max(foot_contact_forces).detach().cpu())
        )
        if not torch.isfinite(observations["policy"]).all() or not torch.isfinite(rewards).all():
            raise RuntimeError("Non-finite observation or reward.")

    report = {
        "status": "passed",
        "usd_file": usd_path.name,
        "steps": args_cli.steps,
        "num_envs": env.num_envs,
        "ground_size_m": round(float(env._ground_size_m), 6),
        "action_dim": action_dim,
        "action_group_counts": action_group_counts(),
        "observation_dim": observations["policy"].shape[-1],
        "foot_contact_dim": len(env._foot_body_ids),
        "mean_foot_contacts_per_env": round(foot_contact_sum / max(args_cli.steps * env.num_envs, 1), 6),
        "max_foot_contact_force_n": round(max_foot_contact_force, 6),
        "mean_reward": round(float(torch.mean(total_reward / max(args_cli.steps, 1)).detach().cpu()), 6),
        "terminated_count": terminated_count,
        "truncated_count": truncated_count,
        "min_root_height_m": round(min_root_height, 6),
        "max_root_tilt_deg": round(max_root_tilt, 6),
        "final_root_position_m": tensor_list(env._robot.data.root_pos_w),
    }

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2), flush=True)


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
