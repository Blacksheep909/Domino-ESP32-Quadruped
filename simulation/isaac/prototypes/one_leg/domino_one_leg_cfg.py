"""Isaac Lab asset config template for the simplified Domino one-leg prototype.

Set DOMINO_ONE_LEG_USD to the USD generated from domino_one_leg_clean.urdf before
using this config in an Isaac Lab scene.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg


DOMINO_ONE_LEG_USD_PATH = os.environ.get("DOMINO_ONE_LEG_USD", "")

DOMINO_ONE_LEG_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=DOMINO_ONE_LEG_USD_PATH,
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
        pos=(0.0, 0.0, 0.35),
        joint_pos={
            "hip_ab_ad": 0.0,
            "upper_pitch": 0.25,
            "lower_linkage": -0.75,
        },
    ),
    actuators={
        "driven_leg_joints": ImplicitActuatorCfg(
            joint_names_expr=["hip_ab_ad", "upper_pitch", "lower_linkage"],
            effort_limit_sim=8.0,
            velocity_limit_sim=6.0,
            stiffness=12.0,
            damping=1.5,
        ),
    },
)
"""Simplified Domino one-leg articulation config."""
