"""Shared Domino quadruped action, foot, and joint-limit contract."""

from __future__ import annotations

from collections.abc import Sequence


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
LINKAGE_DRIVE_ACTION_JOINT_NAMES = [
    joint_name
    for entry in LEG_ACTION_LAYOUT
    for joint_name in (entry["lower_linkage"], entry["upper_pitch"])
]

ACTION_JOINT_NAMES = [
    joint_name
    for entry in LEG_ACTION_LAYOUT
    for joint_name in (entry["shoulder"], entry["lower_linkage"], entry["upper_pitch"])
]
FOOT_BODY_NAMES = [f"{entry['leg_id']}_foot" for entry in LEG_ACTION_LAYOUT]

EXPECTED_ACTION_COUNT = 12
EXPECTED_SHOULDER_ACTION_COUNT = 4
EXPECTED_LINKAGE_DRIVE_ACTIONS_PER_LEG = 2
EXPECTED_LINKAGE_DRIVE_ACTION_COUNT = 8
EXPECTED_FOOT_COUNT = 4
BASE_POLICY_OBSERVATION_DIM = 3 + 3 + 3 + (EXPECTED_ACTION_COUNT * 3)
POLICY_OBSERVATION_DIM = BASE_POLICY_OBSERVATION_DIM + EXPECTED_FOOT_COUNT

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

CAD_ACTION_ROLES = {
    **{name: "shoulder_ab_ad" for name in SHOULDER_ACTION_JOINT_NAMES},
    **{name: "lower_linkage_drive" for name in LOWER_LINKAGE_ACTION_JOINT_NAMES},
    **{name: "upper_pitch_drive" for name in UPPER_PITCH_ACTION_JOINT_NAMES},
}

CAD_ACTION_LIMITS_DEG = {
    **{name: (-30.0, 30.0) for name in SHOULDER_ACTION_JOINT_NAMES},
    **{name: (-120.0, 0.0) for name in LOWER_LINKAGE_ACTION_JOINT_NAMES[:3]},
    "dom_p_21_1_lower_linkage": (-30.0, 90.0),
    **{name: (-30.0, 60.0) for name in UPPER_PITCH_ACTION_JOINT_NAMES},
}


def action_group_counts() -> dict[str, int]:
    return {
        "shoulder_hip_ab_ad": len(SHOULDER_ACTION_JOINT_NAMES),
        "lower_linkage_drive": len(LOWER_LINKAGE_ACTION_JOINT_NAMES),
        "upper_pitch_drive": len(UPPER_PITCH_ACTION_JOINT_NAMES),
        "linkage_drive_total": len(LINKAGE_DRIVE_ACTION_JOINT_NAMES),
        "total": len(ACTION_JOINT_NAMES),
    }


def per_leg_action_layout() -> list[dict[str, object]]:
    return [
        {
            "leg_id": entry["leg_id"],
            "shoulder": entry["shoulder"],
            "linkage_drives": [entry["lower_linkage"], entry["upper_pitch"]],
            "action_order": [entry["shoulder"], entry["lower_linkage"], entry["upper_pitch"]],
        }
        for entry in LEG_ACTION_LAYOUT
    ]


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
    expected_counts = {
        "shoulder_hip_ab_ad": EXPECTED_SHOULDER_ACTION_COUNT,
        "lower_linkage_drive": 4,
        "upper_pitch_drive": 4,
        "linkage_drive_total": EXPECTED_LINKAGE_DRIVE_ACTION_COUNT,
        "total": EXPECTED_ACTION_COUNT,
    }
    if counts != expected_counts:
        raise RuntimeError(f"Unexpected Domino action group counts: {counts}.")
    for entry in LEG_ACTION_LAYOUT:
        linkage_drives = [entry["lower_linkage"], entry["upper_pitch"]]
        if len(linkage_drives) != EXPECTED_LINKAGE_DRIVE_ACTIONS_PER_LEG:
            raise RuntimeError(f"Unexpected Domino linkage-drive layout for {entry['leg_id']}: {linkage_drives}.")


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
