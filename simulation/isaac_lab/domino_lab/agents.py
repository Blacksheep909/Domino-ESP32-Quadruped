"""RSL-RL PPO settings. Actor size is chosen for the ESP32 (see README)."""

from __future__ import annotations

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

from . import config as domino_config

T = domino_config.load()["training"]


@configclass
class DominoPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = T["steps_per_env"]
    max_iterations = T["max_iterations"]
    save_interval = T["save_interval"]
    seed = T["seed"]
    experiment_name = "domino_walk"
    obs_groups = {"policy": ["policy"], "critic": ["critic"]}
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=T["init_noise_std"],
        actor_obs_normalization=True,
        critic_obs_normalization=True,
        actor_hidden_dims=list(T["actor_hidden"]),
        critic_hidden_dims=list(T["critic_hidden"]),
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=T["entropy_coef"],
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=T["learning_rate"],
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
