"""Isaac Lab asset config for the clean Domino 12-DoF quadruped prototype."""

from __future__ import annotations

from collections.abc import Sequence
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg


DOMINO_QUADRUPED_USD_PATH = os.environ.get("DOMINO_QUADRUPED_USD", "")

LEG_ACTION_LAYOUT = [
    {
        "leg_id": "dom_p_4_1",
        "shoulder": "dom_p_4_1_shoulder_ab_ad",
        "lower_linkage": "dom_p_4_1_lower_linkage",
        "upper_pitch": "dom_p_4_1_upper_pitch",
    },
    {
        "leg_id": "dom_p_12_1",
        "shoulder": "dom_p_12_1_shoulder_ab_ad",
        "lower_linkage": "dom_p_12_1_lower_linkage",
        "upper_pitch": "dom_p_12_1_upper_pitch",
    },
    {
        "leg_id": "dom_p_25_1",
        "shoulder": "dom_p_25_1_shoulder_ab_ad",
        "lower_linkage": "dom_p_25_1_lower_linkage",
        "upper_pitch": "dom_p_25_1_upper_pitch",
    },
    {
        "leg_id": "dom_p_21_1",
        "shoulder": "dom_p_21_1_shoulder_ab_ad",
        "lower_linkage": "dom_p_21_1_lower_linkage",
        "upper_pitch": "dom_p_21_1_upper_pitch",
    },
]

SHOULDER_ACTION_JOINT_NAMES = [entry["shoulder"] for entry in LEG_ACTION_LAYOUT]
LOWER_LINKAGE_ACTION_JOINT_NAMES = [entry["lower_linkage"] for entry in LEG_ACTION_LAYOUT]
UPPER_PITCH_ACTION_JOINT_NAMES = [entry["upper_pitch"] for entry in LEG_ACTION_LAYOUT]

ACTION_JOINT_NAMES = [
    joint_name
    for entry in LEG_ACTION_LAYOUT
    for joint_name in (entry["shoulder"], entry["lower_linkage"], entry["upper_pitch"])
]
FOOT_BODY_NAMES = [f"{entry['leg_id']}_foot" for entry in LEG_ACTION_LAYOUT]
EXPECTED_ACTION_COUNT = 12
EXPECTED_FOOT_COUNT = 4
BASE_POLICY_OBSERVATION_DIM = 3 + 3 + 3 + (EXPECTED_ACTION_COUNT * 3)
POLICY_OBSERVATION_DIM = BASE_POLICY_OBSERVATION_DIM + EXPECTED_FOOT_COUNT


def action_group_counts() -> dict[str, int]:
    return {
        "shoulder_hip_ab_ad": len(SHOULDER_ACTION_JOINT_NAMES),
        "lower_linkage_drive": len(LOWER_LINKAGE_ACTION_JOINT_NAMES),
        "upper_pitch_drive": len(UPPER_PITCH_ACTION_JOINT_NAMES),
        "total": len(ACTION_JOINT_NAMES),
    }


def validate_action_layout(joint_names: Sequence[str]) -> None:
    """Fail fast if the imported USD does not expose the intended 12-servo layout."""
    actual = list(joint_names)
    if actual != ACTION_JOINT_NAMES:
        missing = [name for name in ACTION_JOINT_NAMES if name not in actual]
        extra = [name for name in actual if name not in ACTION_JOINT_NAMES]
        raise RuntimeError(
            "Domino action layout mismatch. "
            f"Expected {ACTION_JOINT_NAMES}; found {actual}; missing {missing}; extra {extra}."
        )
    if len(actual) != EXPECTED_ACTION_COUNT or len(set(actual)) != EXPECTED_ACTION_COUNT:
        raise RuntimeError(
            f"Domino action layout must contain {EXPECTED_ACTION_COUNT} unique actuator joints; found {actual}."
        )
    counts = action_group_counts()
    if counts != {"shoulder_hip_ab_ad": 4, "lower_linkage_drive": 4, "upper_pitch_drive": 4, "total": 12}:
        raise RuntimeError(f"Unexpected Domino action group counts: {counts}.")


def validate_foot_body_layout(body_names: Sequence[str]) -> None:
    actual = list(body_names)
    if actual != FOOT_BODY_NAMES:
        missing = [name for name in FOOT_BODY_NAMES if name not in actual]
        extra = [name for name in actual if name not in FOOT_BODY_NAMES]
        raise RuntimeError(
            "Domino foot body layout mismatch. "
            f"Expected {FOOT_BODY_NAMES}; found {actual}; missing {missing}; extra {extra}."
        )
    if len(actual) != EXPECTED_FOOT_COUNT or len(set(actual)) != EXPECTED_FOOT_COUNT:
        raise RuntimeError(f"Domino foot layout must contain {EXPECTED_FOOT_COUNT} unique bodies; found {actual}.")


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
