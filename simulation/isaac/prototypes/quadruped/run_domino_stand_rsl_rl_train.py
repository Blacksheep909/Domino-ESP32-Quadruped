"""Run a tiny RSL-RL PPO training smoke test for DominoStandEnv."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import traceback

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Train DominoStandEnv briefly with RSL-RL PPO.")
parser.add_argument("--usd-path", required=True, help="Path to the imported floating-base quadruped USD.")
parser.add_argument("--num-envs", type=int, default=1, help="Number of cloned environments.")
parser.add_argument("--iterations", type=int, default=1, help="PPO learning iterations to run.")
parser.add_argument("--num-steps-per-env", type=int, default=8, help="Rollout steps per env per PPO iteration.")
parser.add_argument("--seed", type=int, default=42, help="Training seed.")
parser.add_argument("--log-root", default="simulation/isaac/out/domino_rsl_rl", help="Ignored training log root.")
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


def main() -> None:
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    torch.backends.cudnn.deterministic = False
    torch.backends.cudnn.benchmark = False

    env_cfg = DominoStandEnvCfg()
    env_cfg.scene.num_envs = int(args_cli.num_envs)
    env_cfg.sim.device = args_cli.device
    env_cfg.seed = int(args_cli.seed)

    agent_cfg = DominoStandPPORunnerCfg()
    agent_cfg.seed = int(args_cli.seed)
    agent_cfg.device = args_cli.device
    agent_cfg.num_steps_per_env = int(args_cli.num_steps_per_env)
    agent_cfg.max_iterations = int(args_cli.iterations)

    log_root = Path(args_cli.log_root).expanduser().resolve()
    log_dir = log_root / agent_cfg.experiment_name / f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_{agent_cfg.run_name}"
    log_dir.mkdir(parents=True, exist_ok=True)
    env_cfg.log_dir = str(log_dir)

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
    runner = OnPolicyRunner(wrapped_env, agent_cfg.to_dict(), log_dir=str(log_dir), device=agent_cfg.device)
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    checkpoints = sorted(path.name for path in log_dir.glob("model_*.pt"))
    if not checkpoints:
        raise RuntimeError(f"Training completed but no RSL-RL checkpoints were written under {log_dir}.")

    report = {
        "status": "passed",
        "usd_file": usd_path.name,
        "num_envs": env.num_envs,
        "ground_size_m": round(float(env._ground_size_m), 6),
        "iterations": agent_cfg.max_iterations,
        "num_steps_per_env": agent_cfg.num_steps_per_env,
        "action_dim": action_dim,
        "action_group_counts": action_group_counts(),
        "observation_dim": observations["policy"].shape[-1],
        "checkpoints": checkpoints,
        "log_dir_name": log_dir.name,
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
