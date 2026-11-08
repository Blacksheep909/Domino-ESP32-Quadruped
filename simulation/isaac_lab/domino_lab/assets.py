"""Domino articulation config: the pinned-linkage USD from tools/make_usd.py.

Per leg the hip, the thigh (lower-drive servo) and the arm (upper-drive
servo) are servo-driven; the bell crank, push rod, parallel rod and knee are
passive pins, and the push rod and parallel rod close their loops with pin
joints (loops.json). Joint zero is Studio's authored stand pose, where every
loop is closed.
"""

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from . import geometry as g
from .actuators import ServoActuatorCfg

ASSET_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "domino")
USD_PATH = os.path.join(ASSET_DIR, "usd", "domino.usd")


def domino_cfg(robot: dict) -> ArticulationCfg:
    """Servo-driven joints use the hobby-servo model in actuators.py; the
    linkage pins are passive."""
    stand_h = g.NEUTRAL_FOOT_BELOW_HIP_M + g.FOOT_RADIUS
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=USD_PATH,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=100.0,
                max_angular_velocity=100.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                # Loop-closing pins need a few more iterations to stay tight.
                enabled_self_collisions=False, solver_position_iteration_count=16, solver_velocity_iteration_count=1
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, stand_h + 0.01), joint_pos={".*": 0.0}),
        soft_joint_pos_limit_factor=0.95,
        actuators={
            # DS 40 kg-cm (hip) and 30 kg-cm (thigh / arm drives) servos:
            # torque-speed curve, deadband, per-robot strength (actuators.py).
            "hip": ServoActuatorCfg(
                joint_names_expr=[".*_hip_joint"],
                saturation_effort=robot["hip_stall_nm"],
                effort_limit=robot["hip_stall_nm"],
                velocity_limit=robot["hip_no_load_rad_s"],
                stiffness=robot["servo_stiffness"],
                damping=robot["servo_damping"],
                armature=robot["servo_armature"],
                deadband_deg=robot["servo_deadband_deg"],
            ),
            "leg_servos": ServoActuatorCfg(
                joint_names_expr=[".*_thigh_joint", ".*_arm_joint"],
                saturation_effort=robot["leg_stall_nm"],
                effort_limit=robot["leg_stall_nm"],
                velocity_limit=robot["leg_no_load_rad_s"],
                stiffness=robot["servo_stiffness"],
                damping=robot["servo_damping"],
                armature=robot["servo_armature"],
                deadband_deg=robot["servo_deadband_deg"],
            ),
            "pins": ImplicitActuatorCfg(
                joint_names_expr=[".*_(crank|pushrod|rod|knee)_joint"],
                effort_limit_sim=100.0,
                velocity_limit_sim=100.0,
                stiffness=0.0,
                damping=0.002,
                # Keeps the loop pins tight under foot impacts (P4 p99 < 0.5 mm)
                # while barely changing the leg's step response (see
                # scripts/step_response.py: rise time unchanged, overshoot 4 -> 8 %).
                armature=0.002,
            ),
        },
    )
