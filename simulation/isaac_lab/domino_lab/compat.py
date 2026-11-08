"""Small shims so domino_lab runs on Isaac Lab 2.3 (Isaac Sim 5.1) and Isaac Lab 3.x
(Isaac Sim 6.x) while the project moves over.

* Isaac Lab 3 returns asset / sensor data as ``ProxyArray``; ``T()`` unwraps it to a
  torch tensor (and passes a 2.x tensor straight through).
* Isaac Lab 3 orders quaternions (x, y, z, w); 2.x used (w, x, y, z). The project's
  recordings and renderers use (w, x, y, z), so convert at the edges with
  ``quat_wxyz()``. Isaac Lab's own math helpers take whatever the running version uses.
"""

from __future__ import annotations

from importlib import metadata


def _lab_major() -> int:
    try:
        return int(metadata.version("isaaclab").split(".")[0])
    except Exception:  # noqa: BLE001 - source checkouts without metadata
        try:
            import isaaclab

            return int(str(getattr(isaaclab, "__version__", "2")).split(".")[0])
        except Exception:  # noqa: BLE001
            return 2


LAB_MAJOR = _lab_major()
LAB3 = LAB_MAJOR >= 3


def T(x):
    """Torch view of an Isaac Lab data property (ProxyArray in 3.x, tensor in 2.x)."""
    return x.torch if hasattr(x, "torch") else x


def quat_wxyz(q):
    """Quaternion(s) from Isaac Lab data in (w, x, y, z) order."""
    q = T(q)
    return q[..., [3, 0, 1, 2]] if LAB3 else q


def add_launcher_args(parser) -> None:
    """Isaac Lab launcher flags plus ``--headless`` / ``--enable_cameras`` on every
    version (3.x dropped both: headless is the default and a window is a visualizer)."""
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    have = {opt for a in parser._actions for opt in a.option_strings}
    if "--headless" not in have:
        parser.add_argument("--headless", action="store_true", help="no viewer window")
    if "--enable_cameras" not in have:
        parser.add_argument("--enable_cameras", action="store_true", help="enable camera sensors")


def launcher_args(args):
    """Translate the version-neutral flags for the running Isaac Lab."""
    if LAB3:
        if not getattr(args, "headless", False) and getattr(args, "visualizer", None) is None:
            args.visualizer = ["kit"]  # windowed run: open the Kit viewport
    return args


def env_index(env_ids, num_envs: int, device):
    """Environment ids as a long tensor (3.x may pass None or slice(None) for all)."""
    import torch

    if env_ids is None or isinstance(env_ids, slice):
        return torch.arange(num_envs, device=device)[env_ids if isinstance(env_ids, slice) else slice(None)]
    if not isinstance(env_ids, torch.Tensor):
        return torch.as_tensor(env_ids, device=device, dtype=torch.long)
    return env_ids


def runner_cfg(agent_cfg) -> dict:
    """RSL-RL runner config dict for the installed rsl-rl (5.x moved policy settings to
    actor / critic model configs; Isaac Lab 3 ships the converter)."""
    try:
        from isaaclab_rl.rsl_rl import handle_deprecated_rsl_rl_cfg
        from importlib import metadata as _md

        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, _md.version("rsl-rl-lib"))
        from isaaclab.utils import to_dict  # 3.x

        return to_dict(agent_cfg)
    except ImportError:  # Isaac Lab 2.x
        return agent_cfg.to_dict()


def convert_rslrl3_checkpoint(d: dict) -> dict:
    """rsl-rl 3.x checkpoint (one ActorCritic ``model_state_dict``) -> rsl-rl 5.x layout
    (separate actor / critic MLPModels). The optimizer state does not carry over."""
    m = d["model_state_dict"]
    actor, critic = {}, {}
    for k, v in m.items():
        if k == "std":
            actor["distribution.std_param"] = v
        elif k.startswith("actor_obs_normalizer."):
            actor["obs_normalizer." + k.split(".", 1)[1]] = v
        elif k.startswith("critic_obs_normalizer."):
            critic["obs_normalizer." + k.split(".", 1)[1]] = v
        elif k.startswith("actor."):
            actor["mlp." + k.split(".", 1)[1]] = v
        elif k.startswith("critic."):
            critic["mlp." + k.split(".", 1)[1]] = v
        elif k == "log_std":
            actor["distribution.log_std_param"] = v
    return {"actor_state_dict": actor, "critic_state_dict": critic, "iter": d.get("iter", 0), "infos": d.get("infos")}


def load_checkpoint(runner, path: str, optimizer: bool = True, iteration: bool = True) -> None:
    """Load a checkpoint from either rsl-rl generation into ``runner``."""
    import inspect

    import torch

    if "load_optimizer" in inspect.signature(runner.load).parameters:  # rsl-rl 3.x
        runner.load(path, load_optimizer=optimizer)
        return
    d = torch.load(path, weights_only=False, map_location=runner.device)
    converted = "model_state_dict" in d
    if converted:
        d = convert_rslrl3_checkpoint(d)
    cfg = {"actor": True, "critic": True, "optimizer": optimizer and not converted, "iteration": iteration, "rnd": False}
    if runner.alg.load(d, cfg, True):
        runner.current_learning_iteration = d["iter"]


def stochastic_policy(runner):
    """Callable sampling actions with the exploration noise (as during training)."""
    alg = runner.alg
    if hasattr(alg, "get_policy"):  # rsl-rl 5.x
        pol = alg.get_policy()
        return lambda obs: pol(obs, stochastic_output=True)
    return alg.policy.act
