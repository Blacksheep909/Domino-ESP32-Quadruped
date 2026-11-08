"""Domino Isaac Lab tasks.

`geometry` is importable without Isaac Lab (tests and the exporter use it);
the gym registrations only happen when gymnasium is importable.
"""

try:
    import gymnasium as gym
except ImportError:  # pragma: no cover - plain Python (tests / exporter)
    gym = None

if gym is not None:
    for _id, _cfg in (("Domino-Walk-v0", "DominoWalkEnvCfg"), ("Domino-Walk-Play-v0", "DominoWalkEnvCfg_PLAY")):
        gym.register(
            id=_id,
            entry_point="isaaclab.envs:ManagerBasedRLEnv",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": f"domino_lab.env_cfg:{_cfg}",
                "rsl_rl_cfg_entry_point": "domino_lab.agents:DominoPPORunnerCfg",
            },
        )
