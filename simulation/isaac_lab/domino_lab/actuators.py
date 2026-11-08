"""Hobby servo model for Domino's DS servos.

A DS 30 / 40 kg-cm digital servo is a position loop around a geared DC motor:
  * torque falls linearly with speed, from stall torque at rest to zero at the
    no-load speed (DC motor torque-speed curve, Isaac Lab's DCMotor);
  * its controller ignores position errors inside a small deadband;
  * battery sag and unit-to-unit spread change its strength, so the stall
    torque is scaled per robot (`strength`, randomised by
    mdp.randomize_servo_strength).
The PD gains stand in for the servo's internal proportional loop and motor
back-EMF damping and are randomised per robot as well.
"""

from __future__ import annotations

import math

import torch

from isaaclab.actuators import DCMotor, DCMotorCfg
from isaaclab.utils import configclass
from isaaclab.utils.types import ArticulationActions


class ServoActuator(DCMotor):
    cfg: ServoActuatorCfg

    def __init__(self, cfg: ServoActuatorCfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        self.strength = torch.ones_like(self.computed_effort)
        self.deadband = math.radians(cfg.deadband_deg)
        self._servo_vel = torch.zeros_like(self.computed_effort)

    @property
    def _no_load_speed(self) -> torch.Tensor:
        # Isaac Lab 3 renamed velocity_limit -> actuator_velocity_limit.
        v = getattr(self, "actuator_velocity_limit", None)
        return v if v is not None else self.velocity_limit

    def compute(self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor) -> ArticulationActions:
        self._servo_vel[:] = joint_vel
        err = control_action.joint_positions - joint_pos
        if self.deadband > 0:
            err = torch.sign(err) * torch.clamp(err.abs() - self.deadband, min=0.0)
        self.computed_effort = self.stiffness * err - self.damping * joint_vel
        self.applied_effort = self._servo_clip(self.computed_effort, joint_vel)
        control_action.joint_efforts = self.applied_effort
        control_action.joint_positions = None
        control_action.joint_velocities = None
        return control_action

    def _servo_clip(self, effort: torch.Tensor, joint_vel: torch.Tensor) -> torch.Tensor:
        stall = self._saturation_effort * self.strength
        vmax = self._no_load_speed
        v = torch.clip(joint_vel, min=-vmax, max=vmax)
        top = torch.clip(stall * (1.0 - v / vmax), min=0.0)
        bottom = torch.clip(stall * (-1.0 - v / vmax), max=0.0)
        return torch.clip(effort, min=bottom, max=top)

    def _clip_effort(self, effort: torch.Tensor, joint_vel: torch.Tensor | None = None) -> torch.Tensor:
        # 2.x calls _clip_effort(effort); 3.x calls _clip_effort(effort, joint_vel).
        return self._servo_clip(effort, self._servo_vel if joint_vel is None else joint_vel)


@configclass
class ServoActuatorCfg(DCMotorCfg):
    class_type: type = ServoActuator
    deadband_deg: float = 0.3
