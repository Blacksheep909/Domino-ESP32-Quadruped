"""Load and step an RSL-RL DominoStandEnv checkpoint."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import traceback

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Evaluate a DominoStandEnv RSL-RL PPO checkpoint.")
parser.add_argument("--usd-path", required=True, help="Path to the imported floating-base quadruped USD.")
parser.add_argument("--checkpoint", default="", help="Checkpoint path. If omitted, the newest model_*.pt is used.")
parser.add_argument("--log-root", default="simulation/isaac/out/domino_rsl_rl", help="Root searched for checkpoints.")
parser.add_argument("--num-envs", type=int, default=1, help="Number of cloned environments.")
parser.add_argument("--steps", type=int, default=250, help="Evaluation environment steps.")
parser.add_argument("--seed", type=int, default=42, help="Evaluation seed.")
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
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

from domino_quadruped_cfg import (  # noqa: E402
    EXPECTED_ACTION_COUNT,
    POLICY_OBSERVATION_DIM,
    action_group_counts,
)
from domino_stand_env import DominoStandEnv, DominoStandEnvCfg  # noqa: E402
from domino_stand_rsl_rl_cfg import DominoStandPPORunnerCfg  # noqa: E402


def resolve_checkpoint() -> Path:
    if args_cli.checkpoint:
        checkpoint_path = Path(args_cli.checkpoint).expanduser().resolve()
        if not checkpoint_path.exists():
            raise FileNotFoundError(f"Checkpoint path does not exist: {checkpoint_path}")
        return checkpoint_path

    log_root = Path(args_cli.log_root).expanduser().resolve()
    candidates = sorted(log_root.glob("**/model_*.pt"), key=lambda path: path.stat().st_mtime, reverse=True)
    if not candidates:
        raise FileNotFoundError(f"No model_*.pt checkpoint found under {log_root}.")
    return candidates[0]


def root_tilt_deg(projected_gravity_b: torch.Tensor) -> torch.Tensor:
    return torch.rad2deg(torch.acos(torch.clamp(-projected_gravity_b[:, 2], -1.0, 1.0)))


def main() -> None:
    checkpoint_path = resolve_checkpoint()

    env_cfg = DominoStandEnvCfg()
    env_cfg.scene.num_envs = int(args_cli.num_envs)
    env_cfg.sim.device = args_cli.device
    env_cfg.seed = int(args_cli.seed)

    agent_cfg = DominoStandPPORunnerCfg()
    agent_cfg.seed = int(args_cli.seed)
    agent_cfg.device = args_cli.device

    env = DominoStandEnv(env_cfg)
    action_dim = gym.spaces.flatdim(env.single_action_space)
    observations, _ = env.reset()
    if action_dim != EXPECTED_ACTION_COUNT:
        raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} action dimensions, found {action_dim}.")
    if observations["policy"].shape[-1] != POLICY_OBSERVATION_DIM:
        raise RuntimeError(
            f"Expected {POLICY_OBSERVATION_DIM} policy observations, found {observations['policy'].shape[-1]}."
        )

    wrapped_env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(wrapped_env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(str(checkpoint_path), load_optimizer=False, map_location=agent_cfg.device)
    policy = runner.get_inference_policy(device=agent_cfg.device)

    obs = wrapped_env.get_observations().to(agent_cfg.device)
    total_reward = torch.zeros(wrapped_env.num_envs, device=wrapped_env.device)
    done_count = 0
    min_root_height = float("inf")
    max_root_tilt = 0.0
    max_action_abs = 0.0
    foot_contact_sum = 0.0
    max_foot_contact_force = 0.0

    with torch.inference_mode():
        for _ in range(args_cli.steps):
            actions = policy(obs)
            max_action_abs = max(max_action_abs, float(torch.max(torch.abs(actions)).detach().cpu()))
            obs, rewards, dones, _ = wrapped_env.step(actions.to(wrapped_env.device))
            total_reward += rewards
            done_count += int(torch.count_nonzero(dones).detach().cpu())
            min_root_height = min(min_root_height, float(torch.min(env._robot.data.root_pos_w[:, 2]).detach().cpu()))
            max_root_tilt = max(
                max_root_tilt,
                float(torch.max(root_tilt_deg(env._robot.data.projected_gravity_b)).detach().cpu()),
            )
            foot_contact_flags = env._get_foot_contact_flags()
            foot_contact_forces = env._get_foot_contact_forces()
            foot_contact_sum += float(torch.sum(foot_contact_flags).detach().cpu())
            max_foot_contact_force = max(
                max_foot_contact_force, float(torch.max(foot_contact_forces).detach().cpu())
            )
            if not torch.isfinite(actions).all() or not torch.isfinite(obs["policy"]).all() or not torch.isfinite(rewards).all():
                raise RuntimeError("Non-finite action, observation, or reward during checkpoint playback.")
            obs = obs.to(agent_cfg.device)

    report = {
        "status": "passed",
        "usd_file": usd_path.name,
        "checkpoint": checkpoint_path.name,
        "checkpoint_run": checkpoint_path.parent.name,
        "steps": args_cli.steps,
        "num_envs": wrapped_env.num_envs,
        "ground_size_m": round(float(env._ground_size_m), 6),
        "action_dim": action_dim,
        "action_group_counts": action_group_counts(),
        "observation_dim": observations["policy"].shape[-1],
        "foot_contact_dim": len(env._foot_body_ids),
        "mean_foot_contacts_per_env": round(foot_contact_sum / max(args_cli.steps * wrapped_env.num_envs, 1), 6),
        "max_foot_contact_force_n": round(max_foot_contact_force, 6),
        "mean_reward": round(float(torch.mean(total_reward / max(args_cli.steps, 1)).detach().cpu()), 6),
        "done_count": done_count,
        "min_root_height_m": round(min_root_height, 6),
        "max_root_tilt_deg": round(max_root_tilt, 6),
        "max_action_abs": round(max_action_abs, 6),
    }

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2), flush=True)
    wrapped_env.close()


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
