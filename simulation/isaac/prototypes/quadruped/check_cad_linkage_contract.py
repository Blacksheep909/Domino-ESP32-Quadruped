"""Compare the Domino policy action contract with a CAD-derived linkage report."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from domino_action_contract import (
    ACTION_JOINT_NAMES,
    CAD_ACTION_LIMITS_DEG,
    CAD_ACTION_ROLES,
    DEFAULT_JOINT_POS,
    EXPECTED_ACTION_COUNT,
    EXPECTED_LINKAGE_DRIVE_ACTION_COUNT,
    EXPECTED_LINKAGE_DRIVE_ACTIONS_PER_LEG,
    EXPECTED_SHOULDER_ACTION_COUNT,
    LEG_ACTION_LAYOUT,
    action_group_counts,
    per_leg_action_layout,
)


parser = argparse.ArgumentParser(description="Validate policy actions against a CAD-derived Domino linkage report.")
parser.add_argument("--linkage-report", required=True, help="JSON report from the CAD-derived pin-linkage prototype.")
parser.add_argument(
    "--urdf-path",
    default="simulation/isaac/prototypes/quadruped/domino_quadruped_clean.urdf",
    help="Clean quadruped URDF used to import the policy-training USD.",
)
parser.add_argument("--limit-tolerance-deg", type=float, default=0.001, help="Allowed target-limit mismatch.")
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
args = parser.parse_args()


def rad_to_deg(value: float) -> float:
    return math.degrees(float(value))


def load_urdf_limits(path: Path) -> dict[str, tuple[float, float]]:
    root = ET.parse(path).getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        limits[name] = (rad_to_deg(limit.attrib["lower"]), rad_to_deg(limit.attrib["upper"]))
    return limits


def as_limit_pair(value) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"Expected [lower, upper] limit pair, got {value!r}")
    return (float(value[0]), float(value[1]))


def max_abs_pair_delta(lhs: tuple[float, float], rhs: tuple[float, float]) -> float:
    return max(abs(lhs[0] - rhs[0]), abs(lhs[1] - rhs[1]))


def main() -> None:
    linkage_report_path = Path(args.linkage_report).expanduser().resolve()
    urdf_path = Path(args.urdf_path).expanduser().resolve()
    linkage_report = json.loads(linkage_report_path.read_text(encoding="utf-8"))
    urdf_limits = load_urdf_limits(urdf_path)

    action_space = linkage_report.get("action_space", [])
    action_names = [entry.get("name") for entry in action_space]
    failures: list[str] = []
    per_action: list[dict] = []

    if linkage_report.get("status") != "passed":
        failures.append(f"Linkage report status is {linkage_report.get('status')!r}, expected 'passed'.")
    if linkage_report.get("geometry") != "domino-four-12-fixed-body":
        failures.append(
            f"Linkage report geometry is {linkage_report.get('geometry')!r}, expected 'domino-four-12-fixed-body'."
        )
    if action_names != ACTION_JOINT_NAMES:
        failures.append(f"Action order mismatch. Expected {ACTION_JOINT_NAMES}; found {action_names}.")
    if len(action_space) != EXPECTED_ACTION_COUNT:
        failures.append(f"Expected {EXPECTED_ACTION_COUNT} CAD linkage actions, found {len(action_space)}.")

    counts = action_group_counts()
    if counts["shoulder_hip_ab_ad"] != EXPECTED_SHOULDER_ACTION_COUNT:
        failures.append(f"Expected four shoulder actuators, found {counts['shoulder_hip_ab_ad']}.")
    if counts["linkage_drive_total"] != EXPECTED_LINKAGE_DRIVE_ACTION_COUNT:
        failures.append(f"Expected eight linkage-drive actuators, found {counts['linkage_drive_total']}.")
    for entry in LEG_ACTION_LAYOUT:
        linkage_drive_names = [entry["lower_linkage"], entry["upper_pitch"]]
        missing_linkage_drives = [name for name in linkage_drive_names if name not in action_names]
        if len(linkage_drive_names) != EXPECTED_LINKAGE_DRIVE_ACTIONS_PER_LEG or missing_linkage_drives:
            failures.append(
                f"{entry['leg_id']} must expose two linkage-drive actuators; missing {missing_linkage_drives}."
            )

    max_policy_to_cad_limit_delta = 0.0
    max_urdf_to_cad_limit_delta = 0.0
    default_limit_violations: list[str] = []

    for index, action_name in enumerate(ACTION_JOINT_NAMES):
        cad_entry = action_space[index] if index < len(action_space) else {}
        cad_limits = as_limit_pair(cad_entry.get("target_limit_deg", [float("nan"), float("nan")]))
        contract_limits = CAD_ACTION_LIMITS_DEG[action_name]
        urdf_limit = urdf_limits.get(action_name, (float("nan"), float("nan")))
        default_deg = rad_to_deg(DEFAULT_JOINT_POS[action_name])
        role = cad_entry.get("role")
        expected_role = CAD_ACTION_ROLES[action_name]

        policy_to_cad_delta = max_abs_pair_delta(contract_limits, cad_limits)
        urdf_to_cad_delta = max_abs_pair_delta(urdf_limit, cad_limits)
        max_policy_to_cad_limit_delta = max(max_policy_to_cad_limit_delta, policy_to_cad_delta)
        max_urdf_to_cad_limit_delta = max(max_urdf_to_cad_limit_delta, urdf_to_cad_delta)

        if policy_to_cad_delta > args.limit_tolerance_deg:
            failures.append(
                f"{action_name} contract limits {contract_limits} do not match CAD report limits {cad_limits}."
            )
        if urdf_to_cad_delta > args.limit_tolerance_deg:
            failures.append(f"{action_name} URDF limits {urdf_limit} do not match CAD report limits {cad_limits}.")
        if role != expected_role:
            failures.append(f"{action_name} CAD role {role!r} does not match expected role {expected_role!r}.")
        if default_deg < cad_limits[0] - args.limit_tolerance_deg or default_deg > cad_limits[1] + args.limit_tolerance_deg:
            default_limit_violations.append(action_name)

        per_action.append(
            {
                "index": index,
                "name": action_name,
                "cad_joint": cad_entry.get("joint"),
                "role": role,
                "cad_limit_deg": [round(cad_limits[0], 6), round(cad_limits[1], 6)],
                "urdf_limit_deg": [round(urdf_limit[0], 6), round(urdf_limit[1], 6)],
                "default_deg": round(default_deg, 6),
                "policy_to_cad_limit_delta_deg": round(policy_to_cad_delta, 6),
                "urdf_to_cad_limit_delta_deg": round(urdf_to_cad_delta, 6),
            }
        )

    if default_limit_violations:
        failures.append(f"Default joint positions outside CAD limits: {default_limit_violations}.")

    calibration = linkage_report.get("characterization", {}).get("linear_calibration_fit", {})
    if calibration.get("status") != "fit":
        failures.append("CAD linkage report does not include a fitted linear calibration.")
    if calibration.get("rank_deficient") is not False:
        failures.append("CAD linkage calibration is rank deficient.")
    if calibration.get("input_count_with_intercept") != EXPECTED_ACTION_COUNT + 1:
        failures.append(
            "CAD linkage calibration input count does not cover all 12 actuator channels plus intercept."
        )

    report = {
        "status": "passed" if not failures else "failed",
        "linkage_report_file": linkage_report_path.name,
        "urdf_file": urdf_path.name,
        "cad_geometry": linkage_report.get("geometry"),
        "cad_steps": linkage_report.get("steps"),
        "action_count": len(action_space),
        "action_group_counts": counts,
        "per_leg_action_layout": per_leg_action_layout(),
        "max_policy_to_cad_limit_delta_deg": round(max_policy_to_cad_limit_delta, 6),
        "max_urdf_to_cad_limit_delta_deg": round(max_urdf_to_cad_limit_delta, 6),
        "calibration_status": calibration.get("status"),
        "calibration_rank": calibration.get("matrix_rank"),
        "calibration_input_count_with_intercept": calibration.get("input_count_with_intercept"),
        "calibration_rank_deficient": calibration.get("rank_deficient"),
        "per_action": per_action,
        "failures": failures,
    }

    if args.report_path:
        report_path = Path(args.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2), flush=True)
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
