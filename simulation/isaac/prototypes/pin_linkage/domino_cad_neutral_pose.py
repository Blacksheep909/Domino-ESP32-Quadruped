"""Calibrated pre-physics pose for the finite-link Domino CAD articulation.

The rear transforms were captured from a fixed-base, gravity-disabled Isaac
Sim solve.  The front pivot layouts are exact 335 mm translations of the rear
layouts, so their startup transforms are derived from the corresponding rear
mechanisms instead of the earlier inconsistent front calibration.
"""

from __future__ import annotations


CAPTURE_RESOLVED_FLOATING_HEIGHT_M = 0.336392
FRONT_FROM_REAR_TRANSLATION_M = (0.335, 0.0, 0.0)

# Positions are for env origin (0, 0, 0). Quaternions use Isaac's wxyz order.
_REAR_NEUTRAL_BODY_POSES = {
    "body_reference": ((0.099, 0.062375, 0.346892), (1.0, 0.0, 0.0, 0.0)),
    "dom_p_21_1_coupler": ((-0.032428, -0.035733, 0.341581), (0.980856, 0.000001, -0.194735, 0.0)),
    "dom_p_21_1_ground": ((-0.022833, -0.018667, 0.339892), (1.0, 0.000001, 0.0, 0.0)),
    "dom_p_21_1_lower_closure": ((-0.091303, -0.048100, 0.185718), (0.980866, 0.000001, -0.194683, 0.0)),
    "dom_p_21_1_lower_diagonal": ((-0.060889, -0.041850, 0.259635), (0.980861, 0.000001, -0.194707, 0.0)),
    "dom_p_21_1_lower_driver": ((-0.044592, -0.037367, 0.281722), (0.980861, 0.000001, -0.194707, 0.0)),
    "dom_p_21_1_upper_closure": ((-0.024332, -0.035600, 0.368140), (1.0, 0.000001, -0.000062, 0.0)),
    "dom_p_21_1_upper_driver": ((-0.000164, -0.031800, 0.362767), (0.980854, 0.000001, -0.194746, 0.0)),
    "dom_p_25_1_coupler": ((-0.032429, 0.160483, 0.341580), (0.980851, -0.000002, -0.194760, 0.0)),
    "dom_p_25_1_ground": ((-0.022833, 0.143417, 0.339891), (1.0, -0.000002, 0.000001, 0.0)),
    "dom_p_25_1_lower_closure": ((-0.091761, 0.172849, 0.185907), (0.980866, -0.000002, -0.194686, 0.0)),
    "dom_p_25_1_lower_diagonal": ((-0.061118, 0.166600, 0.259728), (0.980859, -0.000002, -0.194722, 0.000001)),
    "dom_p_25_1_lower_driver": ((-0.044745, 0.162116, 0.281785), (0.980859, -0.000003, -0.194722, 0.0)),
    "dom_p_25_1_upper_closure": ((-0.024334, 0.160350, 0.368138), (1.0, -0.000002, -0.000062, 0.0)),
    "dom_p_25_1_upper_driver": ((-0.000165, 0.156550, 0.362766), (0.980849, -0.000003, -0.194771, 0.000001)),
}

FRONT_REAR_BODY_PAIRS = {
    "dom_p_4_1_coupler": "dom_p_21_1_coupler",
    "dom_p_4_1_ground": "dom_p_21_1_ground",
    "dom_p_4_1_lower_closure": "dom_p_21_1_lower_closure",
    "dom_p_4_1_lower_diagonal": "dom_p_21_1_lower_diagonal",
    "dom_p_4_1_lower_driver": "dom_p_21_1_lower_driver",
    "dom_p_4_1_upper_closure": "dom_p_21_1_upper_closure",
    "dom_p_4_1_upper_driver": "dom_p_21_1_upper_driver",
    "dom_p_12_1_coupler": "dom_p_25_1_coupler",
    "dom_p_12_1_ground": "dom_p_25_1_ground",
    "dom_p_12_1_lower_closure": "dom_p_25_1_lower_closure",
    "dom_p_12_1_lower_diagonal": "dom_p_25_1_lower_diagonal",
    "dom_p_12_1_lower_driver": "dom_p_25_1_lower_driver",
    "dom_p_12_1_upper_closure": "dom_p_25_1_upper_closure",
    "dom_p_12_1_upper_driver": "dom_p_25_1_upper_driver",
}


def _translated_front_pose(rear_body_name: str):
    position, orientation = _REAR_NEUTRAL_BODY_POSES[rear_body_name]
    return (
        tuple(position[index] + FRONT_FROM_REAR_TRANSLATION_M[index] for index in range(3)),
        orientation,
    )


CALIBRATED_NEUTRAL_BODY_POSES = {
    **_REAR_NEUTRAL_BODY_POSES,
    **{
        front_body_name: _translated_front_pose(rear_body_name)
        for front_body_name, rear_body_name in FRONT_REAR_BODY_PAIRS.items()
    },
}
