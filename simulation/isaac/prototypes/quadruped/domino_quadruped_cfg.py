"""Isaac Lab asset config for the clean Domino 12-DoF quadruped prototype."""

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg


DOMINO_QUADRUPED_USD_PATH = os.environ.get("DOMINO_QUADRUPED_USD", "")

ACTION_JOINT_NAMES = [
    "dom_p_4_1_shoulder_ab_ad",
    "dom_p_4_1_lower_linkage",
    "dom_p_4_1_upper_pitch",
    "dom_p_12_1_shoulder_ab_ad",
    "dom_p_12_1_lower_linkage",
    "dom_p_12_1_upper_pitch",
    "dom_p_25_1_shoulder_ab_ad",
    "dom_p_25_1_lower_linkage",
    "dom_p_25_1_upper_pitch",
    "dom_p_21_1_shoulder_ab_ad",
    "dom_p_21_1_lower_linkage",
    "dom_p_21_1_upper_pitch",
]

DEFAULT_JOINT_POS = {
    "dom_p_4_1_shoulder_ab_ad": 0.0,
    "dom_p_4_1_lower_linkage": -0.75,
    "dom_p_4_1_upper_pitch": 0.25,
    "dom_p_12_1_shoulder_ab_ad": 0.0,
    "dom_p_12_1_lower_linkage": -0.75,
    "dom_p_12_1_upper_pitch": 0.25,
    "dom_p_25_1_shoulder_ab_ad": 0.0,
    "dom_p_25_1_lower_linkage": -0.75,
    "dom_p_25_1_upper_pitch": 0.25,
    "dom_p_21_1_shoulder_ab_ad": 0.0,
    "dom_p_21_1_lower_linkage": 0.25,
    "dom_p_21_1_upper_pitch": 0.25,
}

DOMINO_QUADRUPED_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=DOMINO_QUADRUPED_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=100.0,
            max_angular_velocity=100.0,
            max_depenetration_velocity=10.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=2,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.38),
        joint_pos=DEFAULT_JOINT_POS,
    ),
    actuators={
        "domino_servos": ImplicitActuatorCfg(
            joint_names_expr=ACTION_JOINT_NAMES,
            effort_limit_sim=8.0,
            velocity_limit_sim=6.0,
            stiffness=12.0,
            damping=1.5,
        ),
    },
)
