"""Isaac Lab asset config for the clean Domino 12-DoF quadruped prototype."""

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from domino_action_contract import (
    ACTION_JOINT_NAMES,
    BASE_POLICY_OBSERVATION_DIM,
    CAD_ACTION_LIMITS_DEG,
    CAD_ACTION_ROLES,
    DEFAULT_JOINT_POS,
    EXPECTED_ACTION_COUNT,
    EXPECTED_FOOT_COUNT,
    EXPECTED_LINKAGE_DRIVE_ACTION_COUNT,
    EXPECTED_LINKAGE_DRIVE_ACTIONS_PER_LEG,
    EXPECTED_SHOULDER_ACTION_COUNT,
    FOOT_BODY_NAMES,
    LEG_ACTION_LAYOUT,
    LINKAGE_DRIVE_ACTION_JOINT_NAMES,
    LOWER_LINKAGE_ACTION_JOINT_NAMES,
    POLICY_OBSERVATION_DIM,
    SHOULDER_ACTION_JOINT_NAMES,
    UPPER_PITCH_ACTION_JOINT_NAMES,
    action_group_counts,
    per_leg_action_layout,
    validate_action_layout,
    validate_foot_body_layout,
)

DOMINO_QUADRUPED_USD_PATH = os.environ.get("DOMINO_QUADRUPED_USD", "")

DOMINO_QUADRUPED_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=DOMINO_QUADRUPED_USD_PATH,
        activate_contact_sensors=True,
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
        # The neutral 0.25 / -0.75 rad linkage pose places the 24 mm feet
        # on the ground at a base height of about 0.315 m.
        pos=(0.0, 0.0, 0.32),
        joint_pos=DEFAULT_JOINT_POS,
    ),
    actuators={
        "shoulder_servos_40kg": ImplicitActuatorCfg(
            joint_names_expr=SHOULDER_ACTION_JOINT_NAMES,
            effort_limit_sim=3.92,
            velocity_limit_sim=6.0,
            stiffness=35.0,
            damping=2.0,
        ),
        "linkage_servos_35kg": ImplicitActuatorCfg(
            joint_names_expr=LINKAGE_DRIVE_ACTION_JOINT_NAMES,
            effort_limit_sim=3.43,
            velocity_limit_sim=6.0,
            stiffness=35.0,
            damping=2.0,
        ),
    },
)
