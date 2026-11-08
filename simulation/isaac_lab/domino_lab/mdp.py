"""Domino-specific MDP pieces for Isaac Lab.

* `FootSpaceAction`  - the policy outputs 12 foot offsets (mm-scaled) around the
  neutral stance at the commanded ride height, exactly what the ESP32 does in
  `src/policy_runner.cpp`. The term solves them with the firmware's own CAD
  linkage solver (domino_lab/linkage.py, a port of solveCadEndpoint) into the
  three real servo axes per leg (hip, thigh drive, arm drive); the pinned
  linkage in the physics does the rest. It also models the servo chain:
  per-robot command latency, calibration (trim) error, foot-frame offset and
  12-bit PWM quantisation.
* `BodyHeightCommand` - the ride height the remote's left stick sets.
* IMU observations with per-robot mounting tilt and gyro bias, the gait clock,
  and the rewards used by `env_cfg.py`.
"""

from __future__ import annotations

import math
from dataclasses import MISSING
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import ActionTerm, ActionTermCfg, CommandTerm, CommandTermCfg, SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply_inverse, yaw_quat

from . import geometry as g
from .compat import T, env_index, quat_wxyz  # noqa: F401
from . import linkage as lk

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv


# The 12 servo-driven joints, leg-major (FL, FR, BL, BR x hip, thigh, arm).
JOINT_ORDER = [f"{leg}_{j}_joint" for leg in g.LEG_NAMES for j in ("hip", "thigh", "arm")]
FOOT_BODIES = [f"{leg}_foot" for leg in g.LEG_NAMES]


# ---------------------------------------------------------------------------
# Hidden per-robot randomisation state
# ---------------------------------------------------------------------------
def _dr(env) -> dict:
    """Per-env hidden parameters shared by actions / observations."""
    st = getattr(env, "_domino_dr", None)
    if st is None:
        n, dev = env.num_envs, env.device
        st = {
            "gyro_bias": torch.zeros(n, 3, device=dev),
            "tilt_bias": torch.zeros(n, 3, device=dev),
        }
        env._domino_dr = st
    return st


def resample_imu_bias(env: ManagerBasedEnv, env_ids: torch.Tensor, gyro_bias: float, tilt_bias_deg: float):
    st = _dr(env)
    env_ids = env_index(env_ids, env.num_envs, env.device)
    n = len(env_ids)
    st["gyro_bias"][env_ids] = (torch.rand(n, 3, device=env.device) * 2 - 1) * gyro_bias
    tilt = (torch.rand(n, 3, device=env.device) * 2 - 1) * math.radians(tilt_bias_deg)
    tilt[:, 2] = 0.0
    st["tilt_bias"][env_ids] = tilt


def randomize_servo_strength(env: ManagerBasedEnv, env_ids: torch.Tensor | None, asset_cfg: SceneEntityCfg, scale_range):
    """Per-robot servo strength (stall torque scale): battery sag and spread."""
    from .actuators import ServoActuator

    asset: Articulation = env.scene[asset_cfg.name]
    env_ids = env_index(env_ids, env.num_envs, env.device)
    s = torch.empty(len(env_ids), 1, device=env.device).uniform_(*scale_range)
    for act in asset.actuators.values():
        if isinstance(act, ServoActuator):
            act.strength[env_ids] = s.expand(-1, act.strength.shape[1])


def scale_effort_limits(env: ManagerBasedEnv, env_ids: torch.Tensor | None, asset_cfg: SceneEntityCfg, scale_range):
    asset: Articulation = env.scene[asset_cfg.name]
    env_ids = env_index(env_ids, env.num_envs, env.device)
    base = T(asset.data.joint_effort_limits)[env_ids].clone()
    if not hasattr(env, "_domino_effort_nominal"):
        env._domino_effort_nominal = T(asset.data.joint_effort_limits).clone()
    base = env._domino_effort_nominal[env_ids]
    s = torch.empty(len(env_ids), 1, device=env.device).uniform_(*scale_range)
    asset.write_joint_effort_limit_to_sim(base * s, env_ids=env_ids)


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
class BodyHeightCommand(CommandTerm):
    """Ride height (metres from hip axis to foot centre)."""

    cfg: BodyHeightCommandCfg

    def __init__(self, cfg: BodyHeightCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self.height = torch.full((self.num_envs, 1), cfg.range_m[1], device=self.device)
        # The sampled command is a target; the command the policy sees slews toward
        # it like the CH3 stick through the firmware (no instant height steps).
        self.target = self.height.clone()
        self.metrics["error_height"] = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        return f"BodyHeightCommand: range {self.cfg.range_m}"

    @property
    def command(self) -> torch.Tensor:
        return self.height

    def _update_metrics(self):
        self.metrics["error_height"] = torch.abs(measured_body_height(self._env).squeeze(-1) - self.height[:, 0])

    def _resample_command(self, env_ids):
        lo, hi = self.cfg.range_m
        env_ids = env_index(env_ids, self.num_envs, self.device)
        h = torch.empty(len(env_ids), device=self.device).uniform_(lo, hi)
        # A third of the robots hold full height (the common case on the dog).
        full = torch.rand(len(env_ids), device=self.device) < self.cfg.full_height_fraction
        h[full] = hi
        self.target[env_ids, 0] = h

    def reset(self, env_ids=None):
        extras = super().reset(env_ids)
        ids = env_index(env_ids, self.num_envs, self.device)
        self.height[ids] = self.target[ids]  # a fresh episode starts at its height
        return extras

    def _update_command(self):
        step = self.cfg.slew_m_per_s * self._env.step_dt
        if step <= 0:
            self.height[:] = self.target
        else:
            self.height += torch.clamp(self.target - self.height, -step, step)


@configclass
class BodyHeightCommandCfg(CommandTermCfg):
    class_type: type = BodyHeightCommand
    range_m: tuple[float, float] = (0.22, 0.28)
    full_height_fraction: float = 0.33
    slew_m_per_s: float = 0.08  # 0 = instant (the old behaviour)


# ---------------------------------------------------------------------------
# Action
# ---------------------------------------------------------------------------
class FootSpaceAction(ActionTerm):
    cfg: FootSpaceActionCfg
    _asset: Articulation

    def __init__(self, cfg: FootSpaceActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self._joint_ids, names = self._asset.find_joints(JOINT_ORDER, preserve_order=True)
        assert names == JOINT_ORDER, names
        n, dev = self.num_envs, self.device
        self._ik = lk.LinkageIK(device=dev)
        self._side = torch.tensor([lk.SIDE[leg] for leg in lk.LEG_ORDER], device=dev, dtype=torch.float32)
        self._deg = math.pi / 180.0
        self._raw = torch.zeros(n, 12, device=dev)
        self._filtered = torch.zeros(n, 12, device=dev)
        self.prev1 = torch.zeros(n, 12, device=dev)
        self.prev2 = torch.zeros(n, 12, device=dev)
        self._processed = torch.zeros(n, 12, device=dev)
        self._max_delay = int(cfg.delay_steps[1])
        self._history = torch.zeros(self._max_delay + 1, n, 12, device=dev)
        self._delay = torch.zeros(n, dtype=torch.long, device=dev)
        self._calib = torch.zeros(n, 12, device=dev)
        self._foot_dx = torch.zeros(n, 1, device=dev)
        self._quant = math.radians(cfg.quantization_deg) if cfg.quantization_deg > 0 else 0.0
        self._targets = torch.zeros(n, 12, device=dev)
        self._last_q = torch.zeros(n, 12, device=dev)
        self.solve_failures = torch.zeros(n, device=dev)

    @property
    def action_dim(self) -> int:
        return 12

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed

    def _height_mm(self) -> torch.Tensor:
        return self._env.command_manager.get_command(self.cfg.height_command)[:, 0] * 1000.0

    def _solve(self, x, y_out, z, env_ids=slice(None)):
        """Foot commands (n, 4) mm -> servo joint targets (n, 12) rad, with
        the closed-form firmware linkage inverse. Failed solves hold the
        previous target for that leg, like the firmware."""
        root, up, lo, ok = self._ik.solve(x, self._side * y_out, z)
        leg_q = torch.stack([root, (lo - self._ik.offset) * self._deg, up * self._deg], dim=-1)  # (n, 4, 3)
        prev = self._last_q[env_ids].view(-1, 4, 3)
        q = torch.where(ok.unsqueeze(-1), leg_q, prev).reshape(-1, 12)
        if isinstance(env_ids, slice):
            self.solve_failures += (~ok).float().sum(dim=-1)
        self._last_q[env_ids] = q
        return q

    def _advance_clock(self):
        env = self._env
        if not hasattr(env, "_domino_phase"):
            env._domino_phase = torch.zeros(self.num_envs, device=self.device)
        cmd = env.command_manager.get_command(self.cfg.velocity_command)
        f = g.gait_frequency_hz(torch, cmd[:, 0], cmd[:, 1], cmd[:, 2], self.cfg.clock_hz_min, self.cfg.clock_hz_max)
        env._domino_phase = torch.remainder(env._domino_phase + 2 * math.pi * f * env.step_dt, 2 * math.pi)

    def process_actions(self, actions: torch.Tensor):
        self._advance_clock()
        self.prev2[:] = self.prev1
        self.prev1[:] = self._raw
        self._raw[:] = actions
        al = self.cfg.action_filter_alpha
        self._filtered[:] = al * actions + (1.0 - al) * self._filtered
        a = torch.clamp(self._filtered, -self.cfg.clip, self.cfg.clip)
        self._processed[:] = a
        x, y, z = g.actions_to_foot_mm(torch, a, self._height_mm())
        x = x + self._foot_dx
        q = self._solve(x, y, z) + self._calib
        if self._quant > 0:
            q = torch.round(q / self._quant) * self._quant
        self._history = torch.roll(self._history, 1, dims=0)
        self._history[0] = q
        idx = self._delay.view(1, -1, 1).expand(1, -1, 12)
        self._targets[:] = torch.gather(self._history, 0, idx)[0]

    def apply_actions(self):
        self._asset.set_joint_position_target(self._targets, joint_ids=self._joint_ids)

    def reset(self, env_ids=None):
        env_ids = env_index(env_ids, self.num_envs, self.device)
        n = len(env_ids)
        dev = self.device
        self._raw[env_ids] = 0.0
        self.prev1[env_ids] = 0.0
        self.prev2[env_ids] = 0.0
        self._filtered[env_ids] = 0.0
        self._processed[env_ids] = 0.0
        if hasattr(self._env, "_domino_phase"):
            self._env._domino_phase[env_ids] = 0.0
        lo_, hi_ = self.cfg.delay_steps
        self._delay[env_ids] = torch.randint(int(lo_), int(hi_) + 1, (n,), device=dev)
        self._calib[env_ids] = (torch.rand(n, 12, device=dev) * 2 - 1) * math.radians(self.cfg.calibration_error_deg)
        self._foot_dx[env_ids] = (torch.rand(n, 1, device=dev) * 2 - 1) * self.cfg.foot_x_offset_mm
        self._last_q[env_ids] = 0.0
        # Fill the latency pipe with the neutral stance so a reset never kicks.
        x, y, z = g.actions_to_foot_mm(torch, torch.zeros(n, 12, device=dev), self._height_mm()[env_ids])
        q = self._solve(x, y, z, env_ids)
        self._history[:, env_ids] = q.unsqueeze(0)
        self._targets[env_ids] = q


@configclass
class FootSpaceActionCfg(ActionTermCfg):
    class_type: type = FootSpaceAction
    asset_name: str = "robot"
    height_command: str = "body_height"
    velocity_command: str = "base_velocity"
    clock_hz_min: float = 1.8
    clock_hz_max: float = 3.0
    clip: float = 1.5
    delay_steps: tuple[int, int] = (0, 2)
    calibration_error_deg: float = 2.0
    foot_x_offset_mm: float = 15.0
    quantization_deg: float = 0.66
    # First-order low-pass on the network output before it becomes foot
    # targets: filtered = alpha * action + (1 - alpha) * filtered_prev. Same
    # filter runs in the firmware (kPolicyActionFilterAlpha). 1.0 = off.
    action_filter_alpha: float = 1.0


# ---------------------------------------------------------------------------
# Observations (everything the ESP32 can compute)
# ---------------------------------------------------------------------------
def imu_gyro(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    return T(asset.data.root_ang_vel_b) + _dr(env)["gyro_bias"]


def imu_gravity(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    gvec = T(asset.data.projected_gravity_b)
    tilt = _dr(env)["tilt_bias"]
    gvec = gvec + torch.cross(tilt, gvec, dim=-1)
    return gvec / torch.linalg.norm(gvec, dim=-1, keepdim=True).clamp(min=1e-6)


def height_command_obs(env: ManagerBasedRLEnv, command_name: str = "body_height") -> torch.Tensor:
    h = env.command_manager.get_command(command_name)
    mid = (g.RIDE_HEIGHT_MIN_MM + g.RIDE_HEIGHT_MAX_MM) / 2000.0
    half = (g.RIDE_HEIGHT_MAX_MM - g.RIDE_HEIGHT_MIN_MM) / 2000.0
    return (h - mid) / half


def _phase(env) -> torch.Tensor:
    ph = getattr(env, "_domino_phase", None)
    if ph is None:
        ph = torch.zeros(env.num_envs, device=env.device)
        env._domino_phase = ph
    return ph


def gait_clock(env: ManagerBasedRLEnv) -> torch.Tensor:
    """sin, cos of the gait phase (advanced by FootSpaceAction at a rate set by
    the command; see geometry.gait_frequency_hz)."""
    ph = _phase(env)
    return torch.stack([torch.sin(ph), torch.cos(ph)], dim=-1)


def measured_body_height(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    ids = getattr(env, "_domino_foot_ids", None)
    if ids is None:
        ids, _ = asset.find_bodies(FOOT_BODIES, preserve_order=True)
        env._domino_foot_ids = ids
    feet_z = T(asset.data.body_pos_w)[:, ids, 2].mean(dim=1)
    return (T(asset.data.root_pos_w)[:, 2] - feet_z).unsqueeze(-1)


def foot_contacts(env: ManagerBasedEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    f = T(sensor.data.net_forces_w)[:, sensor_cfg.body_ids, :]
    return (torch.linalg.norm(f, dim=-1) > 1.0).float()


# ---------------------------------------------------------------------------
# Rewards
# ---------------------------------------------------------------------------
def _moving(env, command_name: str) -> torch.Tensor:
    cmd = env.command_manager.get_command(command_name)
    return (torch.linalg.norm(cmd[:, :2], dim=1) + 0.5 * torch.abs(cmd[:, 2])) > 0.05


def track_body_height_exp(env, std: float, command_name: str = "body_height") -> torch.Tensor:
    target = env.command_manager.get_command(command_name)[:, 0] - g.COMMAND_TO_FOOT_Z_M
    err = measured_body_height(env)[:, 0] - target
    return torch.exp(-torch.square(err) / std**2)


def _stance_desired(env) -> torch.Tensor:
    """(N, 4) 1 = foot should be planted (FL, FR, BL, BR); diagonal trot."""
    pair_a = (torch.sin(_phase(env)) >= 0).float()  # FL + BR planted
    return torch.stack([pair_a, 1 - pair_a, 1 - pair_a, pair_a], dim=-1)


def trot_schedule(env, sensor_cfg: SceneEntityCfg, command_name: str = "base_velocity") -> torch.Tensor:
    """Diagonal trot timing from the gait clock (FL+BR vs FR+BL). When the
    command is zero it rewards keeping all four feet planted instead."""
    contact = foot_contacts(env, sensor_cfg)  # (N, 4) in FL, FR, BL, BR order
    pair_a = (torch.sin(_phase(env)) >= 0).float()  # FL + BR on the ground
    desired = torch.stack([pair_a, 1 - pair_a, 1 - pair_a, pair_a], dim=-1)
    moving = _moving(env, command_name)
    match = (contact == desired).float().mean(dim=1)
    stand = contact.mean(dim=1)
    return torch.where(moving, match, stand)


def stand_still_actions(env, command_name: str = "base_velocity") -> torch.Tensor:
    a = env.action_manager.action
    return torch.sum(torch.square(a), dim=1) * (~_moving(env, command_name)).float()


def track_lin_vel_xy_yaw_frame_exp(env, std: float, command_name: str, asset_cfg=SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    vel_yaw = quat_apply_inverse(yaw_quat(T(asset.data.root_quat_w)), T(asset.data.root_lin_vel_w)[:, :3])
    err = torch.sum(torch.square(env.command_manager.get_command(command_name)[:, :2] - vel_yaw[:, :2]), dim=1)
    return torch.exp(-err / std**2)


def track_ang_vel_z_world_exp(env, std: float, command_name: str, asset_cfg=SceneEntityCfg("robot")) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    err = torch.square(env.command_manager.get_command(command_name)[:, 2] - T(asset.data.root_ang_vel_w)[:, 2])
    return torch.exp(-err / std**2)


def foot_clearance_penalty(env, target_m: float, command_name: str = "base_velocity",
                           asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalise swing feet that skim the ground: for every foot moving
    horizontally, the squared shortfall below `target_m` of lift, measured
    from the lowest foot (works on rough ground). Only while walking."""
    asset: Articulation = env.scene[asset_cfg.name]
    ids = getattr(env, "_domino_foot_ids", None)
    if ids is None:
        ids, _ = asset.find_bodies(FOOT_BODIES, preserve_order=True)
        env._domino_foot_ids = ids
    z = T(asset.data.body_pos_w)[:, ids, 2]
    lift = z - z.min(dim=1, keepdim=True).values
    speed = torch.linalg.norm(T(asset.data.body_lin_vel_w)[:, ids, :2], dim=-1)
    shortfall = torch.clamp(target_m - lift, min=0.0)
    return torch.sum(torch.square(shortfall) * torch.tanh(4.0 * speed), dim=1) * _moving(env, command_name).float()


def swing_height_tracking(env, height_m: float, command_name: str = "base_velocity",
                          asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Squared error between each swinging foot's lift (above the lowest foot)
    and an arc peaking at `height_m` mid-swing, following the gait clock.
    Applies to every foot equally, so a dragging rear leg costs the same as a
    dragging front leg. Only while walking."""
    asset: Articulation = env.scene[asset_cfg.name]
    ids = getattr(env, "_domino_foot_ids", None)
    if ids is None:
        ids, _ = asset.find_bodies(FOOT_BODIES, preserve_order=True)
        env._domino_foot_ids = ids
    z = T(asset.data.body_pos_w)[:, ids, 2]
    lift = z - z.min(dim=1, keepdim=True).values
    ph = _phase(env)
    # pair A (FL, BR) swings in [pi, 2 pi), pair B (FR, BL) in [0, pi)
    prog_a = torch.clamp((ph - math.pi) / math.pi, 0.0, 1.0)
    prog_b = torch.clamp(ph / math.pi, 0.0, 1.0)
    swing = 1.0 - _stance_desired(env)
    prog = torch.stack([prog_a, prog_b, prog_b, prog_a], dim=-1)
    # Smooth half-sine arc: no vertical snap at lift-off (the earlier sqrt
    # profile left the ground near-vertically, which read as twitchy).
    target = height_m * torch.sin(math.pi * prog).clamp(min=0.0)
    err = torch.square(lift - target) * swing
    return err.sum(dim=1) * _moving(env, command_name).float()


def action_smoothness_l2(env) -> torch.Tensor:
    """Second difference of the actions (a_t - 2 a_t-1 + a_t-2): damps
    oscillation / fidgeting without slowing deliberate steps much."""
    term = env.action_manager.get_term("feet")
    d2 = term.raw_actions - 2.0 * term.prev1 + term.prev2
    return torch.sum(torch.square(d2), dim=1)


# ---------------------------------------------------------------------------
# Spot-style smoothness: a level, gliding body and soft foot placement
# ---------------------------------------------------------------------------


def base_heave_accel(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Squared vertical and pitch/roll acceleration of the body, from the
    change in velocity over one control step (what you see as jolting).
    Clipped so the step after a reset or a push does not dominate."""
    asset: Articulation = env.scene[asset_cfg.name]
    v = torch.cat([T(asset.data.root_lin_vel_w)[:, 2:3], T(asset.data.root_ang_vel_b)[:, :2]], dim=1)
    prev = getattr(env, "_domino_prev_base_vel", None)
    if prev is None or prev.shape != v.shape:
        prev = v.clone()
    acc = (v - prev) / env.step_dt
    fresh = env.episode_length_buf <= 1
    acc[fresh] = 0.0
    env._domino_prev_base_vel = v.clone()
    acc = torch.clamp(acc, -30.0, 30.0)
    # vertical m/s^2, plus angular rad/s^2 scaled to a similar size
    return torch.square(acc[:, 0]) + 0.05 * torch.sum(torch.square(acc[:, 1:]), dim=1)


def foot_touchdown_speed(env, sensor_cfg: SceneEntityCfg,
                         asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Squared downward speed of each foot at the step it lands: rewards
    setting the foot down instead of stamping it."""
    sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    first = sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids].float()
    asset: Articulation = env.scene[asset_cfg.name]
    ids = getattr(env, "_domino_foot_ids", None)
    if ids is None:
        ids, _ = asset.find_bodies(FOOT_BODIES, preserve_order=True)
        env._domino_foot_ids = ids
    vz = T(asset.data.body_lin_vel_w)[:, ids, 2]
    return torch.sum(first * torch.square(torch.clamp(vz, max=0.0)), dim=1)


def body_level_exp(env, std_rad: float = 0.05, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Positive reward for a level body: exp(-(tilt / std)^2)."""
    asset: Articulation = env.scene[asset_cfg.name]
    tilt2 = torch.sum(torch.square(T(asset.data.projected_gravity_b)[:, :2]), dim=1)
    return torch.exp(-tilt2 / std_rad**2)


def foot_hover_penalty(env, sensor_cfg: SceneEntityCfg, max_air_s: float = 0.5) -> torch.Tensor:
    """Seconds each foot has been off the ground beyond `max_air_s`, summed:
    stops the policy standing on three legs with one held up (a local optimum
    when stability penalties are strong and the foot-lift rewards only score
    landings)."""
    sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    air = T(sensor.data.current_air_time)[:, sensor_cfg.body_ids]
    return torch.sum(torch.clamp(air - max_air_s, min=0.0), dim=1)
