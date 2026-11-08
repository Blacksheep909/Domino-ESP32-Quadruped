"""Train the Domino walking policy with RSL-RL PPO in Isaac Lab.

Normally started through ../run-train.ps1 (or the Policy Lab page in Studio):

    run-train.ps1 -NumEnvs 4096 -Iterations 3000            # headless
    run-train.ps1 -NumEnvs 16 -Iterations 50 -Visible        # watch it

Each run writes simulation/isaac_lab/runs/<stamp>_<name>/ with:
    run.json        config snapshot + status (Studio polls this)
    metrics.jsonl   one line per PPO iteration (rewards, terms, fps, ...)
    model_*.pt      RSL-RL checkpoints
    policy/         firmware header + metadata exported at the end
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime

# Load h5py (and its HDF5 DLLs) before Isaac Sim starts: in a visible window
# Kit loads its own HDF5 first and h5py then fails with "DLL load failed".
import h5py  # noqa: F401,E402  isort: skip
from isaaclab.app import AppLauncher

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from domino_lab import compat as _compat  # noqa: E402  (no Isaac imports)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

parser = argparse.ArgumentParser(description="Train Domino with RSL-RL.")
parser.add_argument("--num_envs", type=int, default=None)
parser.add_argument("--max_iterations", type=int, default=None)
parser.add_argument("--seed", type=int, default=None)
parser.add_argument("--run_name", type=str, default=None)
parser.add_argument("--resume", type=str, default=None, help="Checkpoint .pt to continue from.")
parser.add_argument("--config", type=str, default=None, help="Training JSON (defaults to config/training.json).")
_compat.add_launcher_args(parser)
args = parser.parse_args()
if args.config:
    os.environ["DOMINO_TRAINING_CONFIG"] = os.path.abspath(args.config)

app_launcher = AppLauncher(_compat.launcher_args(args))
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import statistics  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

import domino_lab  # noqa: E402,F401  (registers the tasks)
from domino_lab import config as domino_config  # noqa: E402

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


class JsonlRunner(OnPolicyRunner):
    """OnPolicyRunner that also appends a JSON line per iteration (metrics.jsonl,
    read by Studio's Policy Lab). Works with rsl-rl 3.x (runner.log) and 5.x
    (runner.logger.log)."""

    metrics_path: str = ""
    run_json: dict = {}
    run_json_path: str = ""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        logger = getattr(self, "logger", None)
        if logger is not None and hasattr(logger, "log"):  # rsl-rl >= 5
            original = logger.log

            def log(**k):
                rec = self._record(
                    it=k["it"], collect=k["collect_time"], learn=k["learn_time"], losses=k.get("loss_dict", {}),
                    lr=k["learning_rate"], std=k["action_std"], rew=logger.rewbuffer, length=logger.lenbuffer,
                    ep_infos=logger.ep_extras, steps=logger.cfg["num_steps_per_env"] * logger.num_envs,
                )
                original(**k)
                self._write(rec)

            logger.log = log

    def log(self, locs: dict, width: int = 80, pad: int = 35):  # rsl-rl 3.x
        rec = self._record(
            it=locs["it"], collect=locs["collection_time"], learn=locs["learn_time"], losses=locs.get("loss_dict", {}),
            lr=self.alg.learning_rate, std=self.alg.policy.action_std, rew=locs["rewbuffer"], length=locs["lenbuffer"],
            ep_infos=locs["ep_infos"], steps=self.num_steps_per_env * self.env.num_envs,
        )
        super().log(locs, width, pad)
        self._write(rec)

    @staticmethod
    def _record(it, collect, learn, losses, lr, std, rew, length, ep_infos, steps):
        rec = {
            "it": int(it),
            "time": time.time(),
            "collection_s": float(collect),
            "learn_s": float(learn),
            "fps": int(steps / max(collect + learn, 1e-9)),
            "noise_std": float(std.mean().item()) if isinstance(std, torch.Tensor) else float(std),
            "lr": float(lr),
        }
        for k, v in losses.items():
            rec[f"loss/{k}"] = float(v)
        if len(rew) > 0:
            rec["mean_reward"] = float(statistics.mean(rew))
            rec["mean_episode_length"] = float(statistics.mean(length))
        if ep_infos:
            for key in dict.fromkeys(k for info in ep_infos for k in info):
                vals = []
                for info in ep_infos:
                    if key not in info:
                        continue
                    v = info[key]
                    if not isinstance(v, torch.Tensor):
                        v = torch.tensor([v])
                    vals.append(v.float().reshape(-1).to("cpu"))
                if vals:
                    rec[key] = float(torch.cat(vals).mean().item())
        return rec

    def _write(self, rec):
        with open(self.metrics_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec) + chr(10))
        self.run_json["iteration"] = rec["it"]
        self.run_json["updated"] = rec["time"]
        if "mean_reward" in rec:
            self.run_json["mean_reward"] = rec["mean_reward"]
        _write_json(self.run_json_path, self.run_json)


def _write_json(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


def main():
    cfg = domino_config.load()
    task = "Domino-Walk-v0"
    env_cfg = load_cfg_from_registry(task, "env_cfg_entry_point")
    agent_cfg = load_cfg_from_registry(task, "rsl_rl_cfg_entry_point")
    if args.num_envs:
        env_cfg.scene.num_envs = args.num_envs
    if args.max_iterations:
        agent_cfg.max_iterations = args.max_iterations
    if args.seed is not None:
        agent_cfg.seed = args.seed
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args.device if args.device else env_cfg.sim.device

    name = args.run_name or cfg.get("run_name", "domino_walk")
    log_dir = os.path.join(ROOT, "runs", datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_" + name)
    os.makedirs(log_dir, exist_ok=True)
    run_json_path = os.path.join(log_dir, "run.json")
    run_json = {
        "name": name,
        "status": "starting",
        "started": time.time(),
        "pid": os.getpid(),
        "num_envs": env_cfg.scene.num_envs,
        "max_iterations": agent_cfg.max_iterations,
        "iteration": 0,
        "resume": args.resume,
        "config": cfg,
    }
    _write_json(run_json_path, run_json)

    env = gym.make(task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=None)
    runner = JsonlRunner(env, _compat.runner_cfg(agent_cfg), log_dir=log_dir, device=agent_cfg.device)
    runner.metrics_path = os.path.join(log_dir, "metrics.jsonl")
    runner.run_json = run_json
    runner.run_json_path = run_json_path
    if args.resume:
        _compat.load_checkpoint(runner, args.resume)

    obs_dims = {k: int(v.shape[-1]) for k, v in env.get_observations().items()}
    run_json.update(status="training", obs_dims=obs_dims)
    _write_json(run_json_path, run_json)
    print(f"[domino] run dir: {log_dir}  obs dims: {obs_dims}", flush=True)
    try:
        runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
        run_json["status"] = "finished"
    except KeyboardInterrupt:
        run_json["status"] = "stopped"
    except Exception as exc:  # keep the status honest for Studio
        run_json["status"] = "failed"
        run_json["error"] = repr(exc)
        raise
    finally:
        last = os.path.join(log_dir, f"model_{runner.current_learning_iteration}.pt")
        if run_json["status"] != "failed":
            runner.save(last)
        run_json["finished"] = time.time()
        _write_json(run_json_path, run_json)
        try:
            from export_policy import export  # noqa: E402

            if os.path.exists(last):
                info = export(last, os.path.join(log_dir, "policy"))
                run_json["export"] = info
                _write_json(run_json_path, run_json)
        except Exception as exc:  # export problems must not hide training results
            print(f"[domino] export failed: {exc!r}", flush=True)
        env.close()


if __name__ == "__main__":
    sys.path.insert(0, HERE)
    main()
    simulation_app.close()
