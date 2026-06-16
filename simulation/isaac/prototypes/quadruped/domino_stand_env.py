"""First DirectRLEnv stand task for the clean Domino quadruped prototype."""

from __future__ import annotations

from collections.abc import Sequence
import math

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

from domino_quadruped_cfg import (
    ACTION_JOINT_NAMES,
    DOMINO_QUADRUPED_CFG,
    EXPECTED_ACTION_COUNT,
    FOOT_BODY_NAMES,
    POLICY_OBSERVATION_DIM,
    validate_action_layout,
    validate_foot_body_layout,
)


def create_static_ground_box(prim_path: str = "/World/Ground", size_m: float = 4.0) -> None:
    """Create a static collision box whose top face sits at world Z=0."""
    stage = omni.usd.get_context().get_stage()
    ground = UsdGeom.Cube.Define(stage, prim_path)
    ground.CreateSizeAttr(1.0)
    xform = UsdGeom.XformCommonAPI(ground)
    xform.SetTranslate(Gf.Vec3d(0.0, 0.0, -0.025))
    xform.SetScale(Gf.Vec3f(float(size_m), float(size_m), 0.05))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())


def calculate_ground_size_m(num_envs: int, env_spacing_m: float, margin_m: float, min_size_m: float) -> float:
    grid_width = max(1, math.ceil(math.sqrt(max(num_envs, 1))))
    cloned_env_span_m = max(0.0, float(grid_width - 1) * float(env_spacing_m))
    return max(float(min_size_m), cloned_env_span_m + 2.0 * float(margin_m))


@configclass
class DominoStandEnvCfg(DirectRLEnvCfg):
    # env
    decimation = 4
    episode_length_s = 5.0
    action_scale = 0.08
    action_space = EXPECTED_ACTION_COUNT
    observation_space = POLICY_OBSERVATION_DIM
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 200, render_interval=decimation)

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1, env_spacing=2.0, replicate_physics=True)
    ground_margin_m = 2.0
    ground_min_size_m = 4.0

    # robot
    robot = DOMINO_QUADRUPED_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*", history_length=3, update_period=1 / 200, track_air_time=True
    )

    # stand task
    target_height_m = 0.31
    min_height_m = 0.08
    max_tilt_deg = 75.0
    alive_reward_scale = 1.0
    height_reward_scale = -8.0
    flat_orientation_reward_scale = -2.0
    joint_velocity_reward_scale = -0.002
    action_rate_reward_scale = -0.02
    foot_contact_threshold_n = 1.0


class DominoStandEnv(DirectRLEnv):
    cfg: DominoStandEnvCfg

    def __init__(self, cfg: DominoStandEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self._action_joint_ids, action_joint_names = self._robot.find_joints(ACTION_JOINT_NAMES, preserve_order=True)
        validate_action_layout(action_joint_names)
        self._foot_body_ids, foot_body_names = self._contact_sensor.find_bodies(FOOT_BODY_NAMES, preserve_order=True)
        validate_foot_body_layout(foot_body_names)

        self._actions = torch.zeros(self.num_envs, self.cfg.action_space, device=self.device)
        self._previous_actions = torch.zeros_like(self._actions)
        self._processed_joint_targets = self._robot.data.default_joint_pos.clone()
        self._max_tilt_rad = math.radians(self.cfg.max_tilt_deg)
        if not hasattr(self, "_ground_size_m"):
            self._ground_size_m = float(self.cfg.ground_min_size_m)

    def _setup_scene(self):
        self._robot = Articulation(self.cfg.robot)
        self.scene.articulations["robot"] = self._robot
        self._contact_sensor = ContactSensor(self.cfg.contact_sensor)
        self.scene.sensors["contact_sensor"] = self._contact_sensor
        self._ground_size_m = calculate_ground_size_m(
            self.cfg.scene.num_envs,
            self.cfg.scene.env_spacing,
            self.cfg.ground_margin_m,
            self.cfg.ground_min_size_m,
        )
        create_static_ground_box(size_m=self._ground_size_m)
        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=["/World/Ground"])
        light_cfg = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._previous_actions = self._actions.clone()
        self._actions = torch.clamp(actions.clone(), -1.0, 1.0)
        self._processed_joint_targets = self._robot.data.default_joint_pos.clone()
        self._processed_joint_targets[:, self._action_joint_ids] += self.cfg.action_scale * self._actions

    def _apply_action(self) -> None:
        self._robot.set_joint_position_target(self._processed_joint_targets)

    def _get_foot_contact_forces(self) -> torch.Tensor:
        net_forces = self._contact_sensor.data.net_forces_w_history
        return torch.max(torch.norm(net_forces[:, :, self._foot_body_ids], dim=-1), dim=1)[0]

    def _get_foot_contact_flags(self) -> torch.Tensor:
        return (self._get_foot_contact_forces() > self.cfg.foot_contact_threshold_n).to(dtype=torch.float32)

    def _get_observations(self) -> dict:
        joint_pos_error = self._robot.data.joint_pos[:, self._action_joint_ids] - self._robot.data.default_joint_pos[
            :, self._action_joint_ids
        ]
        obs = torch.cat(
            (
                self._robot.data.root_lin_vel_b,
                self._robot.data.root_ang_vel_b,
                self._robot.data.projected_gravity_b,
                joint_pos_error,
                self._robot.data.joint_vel[:, self._action_joint_ids],
                self._actions,
                self._get_foot_contact_flags(),
            ),
            dim=-1,
        )
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        height_error = torch.square(self._robot.data.root_pos_w[:, 2] - self.cfg.target_height_m)
        flat_orientation = torch.sum(torch.square(self._robot.data.projected_gravity_b[:, :2]), dim=1)
        joint_velocity = torch.sum(torch.square(self._robot.data.joint_vel[:, self._action_joint_ids]), dim=1)
        action_rate = torch.sum(torch.square(self._actions - self._previous_actions), dim=1)
        reward = (
            self.cfg.alive_reward_scale
            + self.cfg.height_reward_scale * height_error
            + self.cfg.flat_orientation_reward_scale * flat_orientation
            + self.cfg.joint_velocity_reward_scale * joint_velocity
            + self.cfg.action_rate_reward_scale * action_rate
        )
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        root_height = self._robot.data.root_pos_w[:, 2]
        tilt = torch.acos(torch.clamp(-self._robot.data.projected_gravity_b[:, 2], -1.0, 1.0))
        died = (root_height < self.cfg.min_height_m) | (tilt > self._max_tilt_rad)
        return died, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self._robot._ALL_INDICES
        super()._reset_idx(env_ids)

        self._actions[env_ids] = 0.0
        self._previous_actions[env_ids] = 0.0

        joint_pos = self._robot.data.default_joint_pos[env_ids]
        joint_vel = self._robot.data.default_joint_vel[env_ids]
        default_root_state = self._robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        self._robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
