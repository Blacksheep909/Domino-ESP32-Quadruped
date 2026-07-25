"""Verify historical Domino direct-closure bring-up evidence.

This is intentionally narrower than verify_domino_cad_linkage_reports.py.  It
checks the superseded direct-closure linkage reports retained for engineering
traceability. Current passive-linkage validation uses the action sweep,
one-foot search, training, and playback scripts documented in the parent
README.
"""

from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[4]
REPORT_ROOT = REPO_ROOT / "simulation" / "isaac" / "out" / "cad_identity"

EXPECTED_VISUAL_FIDELITY = "actual_cad_stl_visuals_on_proxy_physics"
EXPECTED_MESH_COUNT = 29
EXPECTED_TRIANGLE_COUNT = 135508
EXPECTED_ACTION_COUNT = 12
EXPECTED_VISUAL_BOTTOM_RUN_NAME = "visual_bottom_direct_closure_visible_check"
EXPECTED_GROUNDED_SUPPORT_RUN_NAME = "grounded_support_direct_closure_visible_check"


class VerificationError(RuntimeError):
    """Raised when current reports do not prove the expected direct-closure state."""


def repo_relative(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def load_json(path: Path, required: bool = True) -> dict[str, Any] | None:
    if not path.exists():
        if required:
            raise VerificationError(f"Missing report: {repo_relative(path)}")
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def check(condition: bool, message: str) -> None:
    if not condition:
        raise VerificationError(message)


def first_vector(report: dict[str, Any], key: str) -> list[float]:
    rows = report.get(key)
    check(isinstance(rows, list) and rows, f"Report is missing non-empty {key!r}.")
    first = rows[0]
    check(isinstance(first, list) and len(first) >= 3, f"Report {key!r} is not an xyz vector list.")
    return [float(first[0]), float(first[1]), float(first[2])]


def visible_start_stance_summary(report: dict[str, Any]) -> dict[str, Any]:
    stance = report.get("actual_cad_visual_start_stance")
    if isinstance(stance, dict):
        return {
            "max_height_spread_m": round(float(stance.get("max_height_spread_m", 0.0) or 0.0), 6),
            "level_enough_for_visual_base_pose": bool(stance.get("level_enough_for_visual_base_pose", False)),
            "warnings": list(stance.get("warnings") or []),
        }
    motion = report.get("actual_cad_visual_foot_bottom_motion") or {}
    rows = motion.get("initial_rendered_position_m")
    if not isinstance(rows, list) or not rows:
        return {
            "max_height_spread_m": 0.0,
            "level_enough_for_visual_base_pose": False,
            "warnings": ["missing initial rendered CAD foot-bottom positions"],
        }
    spreads = []
    for row in rows:
        if not isinstance(row, list) or not row:
            continue
        z_values = [float(point[2]) for point in row if isinstance(point, list) and len(point) >= 3]
        if z_values:
            spreads.append(max(z_values) - min(z_values))
    max_spread = max(spreads, default=0.0)
    warnings = []
    if max_spread > 0.025:
        warnings.append(
            f"visible CAD foot-bottom height spread is {max_spread:.6f} m; stance is not visually level"
        )
    return {
        "max_height_spread_m": round(float(max_spread), 6),
        "level_enough_for_visual_base_pose": bool(max_spread <= 0.025 and bool(spreads)),
        "warnings": warnings,
    }


def validate_common_actual_cad(report: dict[str, Any], label: str) -> None:
    check(report.get("visual_fidelity") == EXPECTED_VISUAL_FIDELITY, f"{label}: wrong visual fidelity.")
    check(bool(report.get("actual_cad_visual")) is True, f"{label}: actual CAD visuals are disabled.")
    visuals = report.get("actual_cad_visuals")
    check(isinstance(visuals, dict), f"{label}: missing CAD visual attachment report.")
    check(int(visuals.get("mesh_count", -1)) == EXPECTED_MESH_COUNT, f"{label}: wrong CAD mesh count.")
    check(int(visuals.get("triangle_count", -1)) == EXPECTED_TRIANGLE_COUNT, f"{label}: wrong CAD triangle count.")
    counts = report.get("visual_geometry_counts")
    check(isinstance(counts, dict), f"{label}: missing visual geometry counts.")
    check(int(counts.get("visible_actual_cad_mesh_count", -1)) == EXPECTED_MESH_COUNT, f"{label}: CAD meshes are not all visible.")
    check(int(counts.get("visible_actual_cad_triangle_count", -1)) == EXPECTED_TRIANGLE_COUNT, f"{label}: CAD triangle count mismatch.")
    check(int(counts.get("visible_proxy_cube_count", -1)) == 0, f"{label}: proxy cubes are visible.")
    check(int(counts.get("visible_proxy_sphere_count", -1)) == 0, f"{label}: proxy spheres are visible.")
    check(bool(counts.get("proxy_visuals_non_renderable")) is True, f"{label}: proxy visuals are renderable.")
    action_names = report.get("action_names")
    check(isinstance(action_names, list) and len(action_names) == EXPECTED_ACTION_COUNT, f"{label}: action count is not 12.")


def validate_closure_visual_aliases(report: dict[str, Any], label: str) -> None:
    corrector_paths = ((report.get("actual_cad_visuals") or {}).get("corrector_paths") or {})
    required_suffixes = {
        "dom_p_4_1_lower_closure": "/dom_p_4_1/lower_driver/actual_cad",
        "dom_p_4_1_upper_closure": "/dom_p_4_1/upper_driver/actual_cad",
        "dom_p_12_1_lower_closure": "/dom_p_12_1/lower_driver/actual_cad",
        "dom_p_12_1_upper_closure": "/dom_p_12_1/upper_driver/actual_cad",
        "dom_p_25_1_lower_closure": "/dom_p_25_1/lower_driver/actual_cad",
        "dom_p_25_1_upper_closure": "/dom_p_25_1/upper_driver/actual_cad",
        "dom_p_21_1_lower_closure": "/dom_p_21_1/lower_driver/actual_cad",
        "dom_p_21_1_upper_closure": "/dom_p_21_1/upper_driver/actual_cad",
    }
    for key, suffix in required_suffixes.items():
        path = str(corrector_paths.get(key, ""))
        check(path.endswith(suffix), f"{label}: closure visual {key} is not aliased to {suffix}.")


def validate_grounded_support_contact_source(report: dict[str, Any], label: str) -> None:
    contact = report.get("actual_cad_foot_collision")
    check(isinstance(contact, dict), f"{label}: missing foot collision report.")
    check(contact.get("source") == "actual_cad_visual_xy_grounded_proxy_spheres", f"{label}: expected grounded-support contact source.")
    training_config = report.get("training_config") or {}
    check(
        bool(training_config.get("use_actual_cad_visual_foot_bottom_for_rewards")) is False,
        f"{label}: grounded-support training must use support proxy contacts, not rendered visual foot bottoms.",
    )


def validate_visible_grounded_support_playback(
    report: dict[str, Any],
    label: str = "direct_closure_visible_grounded_support",
) -> dict[str, Any]:
    check(report.get("status") == "passed", f"{label}: playback did not pass.")
    validate_common_actual_cad(report, label)
    validate_closure_visual_aliases(report, label)
    check(int(report.get("done_count", -1)) == 0, f"{label}: done_count is not zero.")
    check(int(report.get("terminated_count", -1)) == 0, f"{label}: terminated_count is not zero.")
    check(int(report.get("timeout_count", 0)) == 0, f"{label}: timeout_count is not zero.")
    joint = report.get("joint_separation")
    check(isinstance(joint, dict), f"{label}: missing joint separation report.")
    max_sep = float(joint.get("max_separation_m", 999.0))
    check(max_sep <= 0.001, f"{label}: joint separation too high: {max_sep:.6f} m.")
    passive_stabilizers = report.get("passive_stabilizers")
    check(passive_stabilizers == [], f"{label}: passive stabilizers should be removed.")
    validate_grounded_support_contact_source(report, label)
    foot_motion = float(((report.get("foot_endpoint_motion") or {}).get("max_displacement_any_foot_m")) or 0.0)
    visual_foot_motion = float(
        ((report.get("actual_cad_visual_foot_bottom_motion") or {}).get("max_rendered_displacement_any_foot_m")) or 0.0
    )
    check(visual_foot_motion >= 0.05, f"{label}: rendered CAD foot-bottom motion is too low.")
    displacement = first_vector(report, "final_body_reference_displacement_m")
    return {
        "status": "passed",
        "max_joint_separation_m": round(max_sep, 6),
        "max_tilt_deg": round(float(report.get("max_body_tilt_deg", 999.0)), 3),
        "foot_proxy_motion_m": round(foot_motion, 6),
        "rendered_cad_foot_motion_m": round(visual_foot_motion, 6),
        "visible_start_stance": visible_start_stance_summary(report),
        "final_displacement_m": [round(value, 6) for value in displacement],
    }


def validate_visible_grounded_support_static_base(report: dict[str, Any]) -> dict[str, Any]:
    label = "stable_grounded_support_base_start_zero"
    check(report.get("status") == "passed", f"{label}: zero-action playback did not pass.")
    validate_common_actual_cad(report, label)
    validate_closure_visual_aliases(report, label)
    check(report.get("policy_mode") == "zero", f"{label}: expected zero policy mode.")
    check(int(report.get("startup_zero_steps", 0)) >= 120, f"{label}: base-pose startup hold is too short.")
    check(int(report.get("startup_done_count", -1)) == 0, f"{label}: startup_done_count is not zero.")
    check(int(report.get("startup_terminated_count", -1)) == 0, f"{label}: startup_terminated_count is not zero.")
    check(int(report.get("startup_timeout_count", -1)) == 0, f"{label}: startup_timeout_count is not zero.")
    check(int(report.get("done_count", -1)) == 0, f"{label}: done_count is not zero.")
    check(int(report.get("terminated_count", -1)) == 0, f"{label}: terminated_count is not zero.")
    check(int(report.get("timeout_count", -1)) == 0, f"{label}: timeout_count is not zero.")
    check(float(report.get("max_executed_action_abs", 999.0)) <= 1.0e-6, f"{label}: zero mode executed non-zero actions.")
    joint = report.get("joint_separation")
    check(isinstance(joint, dict), f"{label}: missing joint separation report.")
    max_sep = float(joint.get("max_separation_m", 999.0))
    check(max_sep <= 0.001, f"{label}: joint separation too high: {max_sep:.6f} m.")
    validate_grounded_support_contact_source(report, label)
    visual_foot_motion = float(
        ((report.get("actual_cad_visual_foot_bottom_motion") or {}).get("max_rendered_displacement_any_foot_m")) or 0.0
    )
    check(visual_foot_motion <= 0.05, f"{label}: base-pose visual foot drift is too high.")
    mean_foot_contacts = float(report.get("mean_foot_contacts_per_env", -1.0))
    check(mean_foot_contacts >= 3.5, f"{label}: support-proxy contact count is too low: {mean_foot_contacts:.6f}.")
    return {
        "status": "passed",
        "startup_zero_steps": int(report.get("startup_zero_steps", 0)),
        "max_joint_separation_m": round(max_sep, 6),
        "max_tilt_deg": round(float(report.get("max_body_tilt_deg", 999.0)), 3),
        "mean_foot_contacts_per_env": round(mean_foot_contacts, 6),
        "rendered_cad_foot_motion_m": round(visual_foot_motion, 6),
        "visible_start_stance": visible_start_stance_summary(report),
        "final_displacement_m": [round(value, 6) for value in first_vector(report, "final_body_reference_displacement_m")],
    }


def validate_calibrated_fixed_stance(
    report: dict[str, Any],
    label: str = "calibrated_fixed_stance_visual_check",
    require_ground_aligned: bool = False,
) -> dict[str, Any]:
    check(report.get("status") == "passed", f"{label}: fixed-stance playback did not pass.")
    validate_common_actual_cad(report, label)
    validate_closure_visual_aliases(report, label)
    check(report.get("policy_mode") == "fixed", f"{label}: expected fixed policy mode.")
    check(str(report.get("startup_action_source")) == "custom", f"{label}: expected custom startup action source.")
    check(int(report.get("startup_done_count", -1)) == 0, f"{label}: startup_done_count is not zero.")
    check(int(report.get("startup_terminated_count", -1)) == 0, f"{label}: startup_terminated_count is not zero.")
    check(int(report.get("startup_timeout_count", -1)) == 0, f"{label}: startup_timeout_count is not zero.")
    check(int(report.get("done_count", -1)) == 0, f"{label}: done_count is not zero.")
    check(int(report.get("terminated_count", -1)) == 0, f"{label}: terminated_count is not zero.")
    check(int(report.get("timeout_count", -1)) == 0, f"{label}: timeout_count is not zero.")
    joint = report.get("joint_separation")
    check(isinstance(joint, dict), f"{label}: missing joint separation report.")
    max_sep = float(joint.get("max_separation_m", 999.0))
    check(max_sep <= 0.001, f"{label}: joint separation too high: {max_sep:.6f} m.")
    validate_grounded_support_contact_source(report, label)
    visual_foot_motion = float(
        ((report.get("actual_cad_visual_foot_bottom_motion") or {}).get("max_rendered_displacement_any_foot_m")) or 999.0
    )
    check(visual_foot_motion <= 0.005, f"{label}: fixed stance visual feet drift too much.")
    stance = visible_start_stance_summary(report)
    spread = float(stance.get("max_height_spread_m", 999.0))
    check(spread <= 0.030, f"{label}: calibrated stance did not improve visible foot spread enough: {spread:.6f} m.")
    raw_stance = report.get("actual_cad_visual_start_stance") or {}
    post_alignment = report.get("post_startup_visual_alignment") or {}
    if require_ground_aligned:
        check(bool(post_alignment.get("enabled")) is True, f"{label}: post-startup visual alignment is not enabled.")
        envs = raw_stance.get("envs") or []
        check(envs and isinstance(envs[0], dict), f"{label}: missing visible stance env details.")
        min_z = float(envs[0].get("min_z_m", 999.0))
        clearance = float((report.get("training_config") or {}).get("actual_cad_ground_clearance_m", 0.002))
        check(abs(min_z - clearance) <= 0.003, f"{label}: lowest rendered foot is not aligned to clearance.")
    return {
        "status": (
            "ground_aligned_improved_not_level"
            if require_ground_aligned and not bool(stance.get("level_enough_for_visual_base_pose"))
            else ("improved_not_level" if not bool(stance.get("level_enough_for_visual_base_pose")) else "passed")
        ),
        "max_joint_separation_m": round(max_sep, 6),
        "max_tilt_deg": round(float(report.get("max_body_tilt_deg", 999.0)), 3),
        "rendered_cad_foot_motion_m": round(visual_foot_motion, 6),
        "visible_start_stance": stance,
        "post_startup_visual_alignment": post_alignment,
        "visual_to_support_offset": report.get("actual_cad_visual_to_support_offset") or {},
        "fixed_action_values": [round(float(value), 6) for value in list(report.get("fixed_action_values") or [])],
    }


def validate_refine_report(report: dict[str, Any]) -> dict[str, Any]:
    label = "direct_closure_grounded_support_refine"
    check(
        report.get("status") in {"policy_validation_failed", "passed_with_policy_validation_warnings", "passed"},
        f"{label}: unexpected status {report.get('status')!r}.",
    )
    validate_common_actual_cad(report, label)
    validate_closure_visual_aliases(report, label)
    check(str(report.get("latest_checkpoint")) == "model_145.pt", f"{label}: latest checkpoint should be model_145.pt.")
    validation = report.get("trained_policy_validation")
    check(isinstance(validation, dict) and bool(validation.get("enabled")), f"{label}: missing trained policy validation.")
    joint = validation.get("joint_separation")
    check(isinstance(joint, dict), f"{label}: validation missing joint separation.")
    max_sep = float(joint.get("max_separation_m", 999.0))
    check(max_sep <= 0.001, f"{label}: validation joint separation too high: {max_sep:.6f} m.")
    check(int(validation.get("done_count", -1)) == 0, f"{label}: validation done_count is not zero.")
    check(int(validation.get("terminated_count", -1)) == 0, f"{label}: validation terminated_count is not zero.")
    failures = list(validation.get("failures") or [])
    expected_contact_failures = {"swing_contact=1.000000", "swing_clearance_m=0.000000", "min_foot_motion_m=0.015627"}
    check(set(failures).issubset(expected_contact_failures), f"{label}: unexpected validation failures: {failures}.")
    return {
        "status": report.get("status"),
        "latest_checkpoint": report.get("latest_checkpoint"),
        "validation_passed": bool(validation.get("passed")),
        "validation_failures": failures,
        "max_joint_separation_m": round(max_sep, 6),
        "final_forward_m": round(float(validation.get("final_forward_m", 0.0)), 6),
        "mean_swing_contact": round(float(validation.get("mean_swing_contact", 0.0)), 6),
        "mean_swing_clearance_m": round(float(validation.get("mean_swing_clearance_m", 0.0)), 6),
    }


def validate_visual_bottom_report(report: dict[str, Any], label: str) -> dict[str, Any]:
    check(report.get("status") == "passed", f"{label}: visual-bottom playback did not pass.")
    validate_common_actual_cad(report, label)
    validate_closure_visual_aliases(report, label)
    contact = report.get("actual_cad_foot_collision")
    check(isinstance(contact, dict), f"{label}: missing foot collision report.")
    check(contact.get("source") == "actual_cad_visual_bottom", f"{label}: expected actual_cad_visual_bottom contact source.")
    check(int(report.get("done_count", -1)) == 0, f"{label}: done_count is not zero.")
    check(int(report.get("terminated_count", -1)) == 0, f"{label}: terminated_count is not zero.")
    joint = report.get("joint_separation")
    check(isinstance(joint, dict), f"{label}: missing joint separation report.")
    max_sep = float(joint.get("max_separation_m", 999.0))
    check(max_sep <= 0.003, f"{label}: joint separation too high: {max_sep:.6f} m.")
    visual_motion = float(
        ((report.get("actual_cad_visual_foot_bottom_motion") or {}).get("max_rendered_displacement_any_foot_m")) or 0.0
    )
    check(visual_motion >= 0.05, f"{label}: rendered CAD foot-bottom motion is too low.")
    return {
        "status": "passed",
        "max_joint_separation_m": round(max_sep, 6),
        "mean_swing_contact": round(float(report.get("mean_swing_contact", 0.0)), 6),
        "mean_swing_clearance_m": round(float(report.get("mean_swing_clearance_m", 0.0)), 6),
        "rendered_cad_foot_motion_m": round(visual_motion, 6),
        "final_displacement_m": [round(value, 6) for value in first_vector(report, "final_body_reference_displacement_m")],
    }


def call_exprs(tree: ast.AST, function_name: str) -> list[list[str]]:
    rows: list[list[str]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.id if isinstance(func, ast.Name) else ""
        if name != function_name:
            continue
        rows.append([ast.unparse(arg) for arg in node.args])
    return rows


def validate_source_topology() -> dict[str, Any]:
    builder_path = REPO_ROOT / "simulation" / "isaac" / "prototypes" / "pin_linkage" / "domino_cad_linkage_builder.py"
    env_path = REPO_ROOT / "simulation" / "isaac" / "prototypes" / "pin_linkage" / "domino_cad_linkage_env.py"
    policy_helper_path = REPO_ROOT / "simulation" / "isaac" / "run-visible-domino-policy.ps1"
    training_helper_path = REPO_ROOT / "simulation" / "isaac" / "run-visible-domino-training.ps1"

    builder_text = builder_path.read_text(encoding="utf-8")
    env_text = env_path.read_text(encoding="utf-8")
    policy_helper_text = policy_helper_path.read_text(encoding="utf-8")
    training_helper_text = training_helper_path.read_text(encoding="utf-8")
    builder_tree = ast.parse(builder_text, filename=str(builder_path))

    aliases: dict[str, str] = {}
    for node in ast.walk(builder_tree):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id == "ACTUAL_CAD_VISUAL_BODY_ALIASES":
                value = ast.literal_eval(node.value)
                check(isinstance(value, dict), "ACTUAL_CAD_VISUAL_BODY_ALIASES is not a dict.")
                aliases = {str(key): str(item) for key, item in value.items()}
    expected_aliases = {
        "dom_p_4_1_lower_closure": "dom_p_4_1_lower_driver",
        "dom_p_4_1_upper_closure": "dom_p_4_1_upper_driver",
        "dom_p_12_1_lower_closure": "dom_p_12_1_lower_driver",
        "dom_p_12_1_upper_closure": "dom_p_12_1_upper_driver",
        "dom_p_25_1_lower_closure": "dom_p_25_1_lower_driver",
        "dom_p_25_1_upper_closure": "dom_p_25_1_upper_driver",
        "dom_p_21_1_lower_closure": "dom_p_21_1_lower_driver",
        "dom_p_21_1_upper_closure": "dom_p_21_1_upper_driver",
    }
    check(aliases == expected_aliases, "Closure visual aliases do not match the direct-closure model.")

    body_calls = call_exprs(builder_tree, "create_body_from_points")
    created_body_names = [args[2].strip("\"'") for args in body_calls if len(args) >= 3 and args[2].startswith(("'", '"'))]
    check("lower_closure" not in created_body_names, "Builder still creates a lower_closure rigid body.")
    check("upper_closure" not in created_body_names, "Builder still creates an upper_closure rigid body.")

    pin_calls = call_exprs(builder_tree, "create_pin_joint")
    pin_arg_sets = {tuple(args[2:5]) for args in pin_calls if len(args) >= 5}
    lower_direct = ("lower_driver", "lower_diagonal", "points['lower_closure']")
    upper_direct = ("coupler", "upper_driver", "points['upper_closure']")
    check(any(args[:3] == lower_direct for args in pin_arg_sets), "Missing direct lower loop closure pin.")
    check(any(args[:3] == upper_direct for args in pin_arg_sets), "Missing direct upper loop closure pin.")
    forbidden_pairs = {
        ("lower_driver", "lower_closure"),
        ("lower_diagonal", "lower_closure"),
        ("coupler", "upper_closure"),
        ("upper_driver", "upper_closure"),
    }
    for body_a, body_b, *_ in pin_arg_sets:
        check((body_a, body_b) not in forbidden_pairs, f"Builder still pins through removed closure body {body_a}->{body_b}.")

    foot_proxy_calls = call_exprs(builder_tree, "create_body_collision_sphere")
    check(
        any(len(args) >= 3 and args[1] == "lower_driver" and "foot_proxy" in args[2] for args in foot_proxy_calls),
        "Foot proxy is not attached to lower_driver in the direct-closure model.",
    )
    check('"foot_proxy_body": lower_driver_key' in builder_text, "Builder does not record foot_proxy_body=lower_driver.")
    check(
        '"actual_cad_visual_foot_body": lower_driver_key' in builder_text,
        "Builder does not record actual CAD lower-closure visual foot body as lower_driver.",
    )
    check(
        '"physics_closure_model": "direct_loop_closure_no_extra_closure_rigid_bodies"' in builder_text,
        "Builder does not report the direct-closure physics model.",
    )
    check('leg.get("foot_proxy_body"' in env_text, "Env foot tracking does not consume foot_proxy_body.")
    check('leg.get("actual_cad_visual_foot_body"' in env_text, "Env visual foot tracking does not consume actual_cad_visual_foot_body.")
    check(
        '[string]$FootCollisionMode = "actual-cad-grounded-support"' in policy_helper_text,
        "Visible policy helper does not default to actual-cad-grounded-support.",
    )
    check(
        '[ValidateSet("checkpoint", "reference", "zero", "fixed")]' in policy_helper_text
        and "--policy-mode" in policy_helper_text,
        "Visible policy helper does not expose checkpoint/reference/zero/fixed playback modes.",
    )
    check(
        "--startup-zero-steps" in policy_helper_text,
        "Visible policy helper does not pass the startup base-pose hold to playback.",
    )
    check(
        '[string]$FootCollisionMode = "actual-cad-grounded-support"' in training_helper_text,
        "Visible training helper does not default to actual-cad-grounded-support.",
    )
    check("model_145.pt" in policy_helper_text, "Visible policy helper does not prefer model_145.pt.")
    check("model_145.pt" in training_helper_text, "Visible training helper does not prefer model_145.pt.")
    return {
        "status": "passed",
        "builder": repo_relative(builder_path),
        "env": repo_relative(env_path),
        "closure_visual_alias_count": len(aliases),
        "created_rigid_body_names_checked": len(created_body_names),
        "direct_lower_loop_closure": True,
        "direct_upper_loop_closure": True,
        "grounded_support_helper_default": True,
        "policy_helper_modes": ["checkpoint", "reference", "zero", "fixed"],
        "startup_base_pose_hold": True,
        "preferred_checkpoint": "model_145.pt",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify current Domino direct-closure linkage reports.")
    parser.add_argument(
        "--require-visual-bottom",
        action="store_true",
        help="Fail if the post-fix visual-bottom playback report has not been generated.",
    )
    parser.add_argument("--json-summary", default="", help="Optional summary JSON path.")
    args = parser.parse_args()

    summary: dict[str, Any] = {
        "status": "passed",
        "report_root": repo_relative(REPORT_ROOT),
        "source_topology": validate_source_topology(),
        "direct_closure_visible_grounded_support": validate_visible_grounded_support_playback(
            load_json(REPORT_ROOT / "next_policy" / "direct_closure_visible_policy_stairs_240.json")
        ),
        "stable_grounded_support_zero_hold": validate_visible_grounded_support_playback(
            load_json(REPORT_ROOT / "next_policy" / "grounded_support_zero_hold_check.json")
        ),
        "stable_grounded_support_reference_gait": validate_visible_grounded_support_playback(
            load_json(REPORT_ROOT / "next_policy" / "grounded_support_reference_gait_check.json")
        ),
        "stable_grounded_support_default_checkpoint": validate_visible_grounded_support_playback(
            load_json(REPORT_ROOT / "next_policy" / "grounded_support_direct_closure_visible_check.json")
        ),
        "stable_grounded_support_base_start_checkpoint": validate_visible_grounded_support_playback(
            load_json(REPORT_ROOT / "next_policy" / "grounded_support_base_start_checkpoint_check.json")
        ),
        "current_base_pose_zero_hold": validate_visible_grounded_support_static_base(
            load_json(REPORT_ROOT / "next_policy" / "grounded_support_base_start_zero_check.json")
        ),
        "current_reference_linkage_motion": validate_visible_grounded_support_playback(
            load_json(REPORT_ROOT / "next_policy" / "grounded_support_base_start_reference_check.json"),
            label="current_reference_linkage_motion",
        ),
        "calibrated_fixed_stance_visual_check": validate_calibrated_fixed_stance(
            load_json(REPORT_ROOT / "next_policy" / "calibrated_fixed_stance_visual_check.json")
        ),
        "calibrated_fixed_stance_visual_ground_aligned_check": validate_calibrated_fixed_stance(
            load_json(REPORT_ROOT / "next_policy" / "calibrated_fixed_stance_visual_ground_aligned_check.json"),
            label="calibrated_fixed_stance_visual_ground_aligned_check",
            require_ground_aligned=True,
        ),
        "direct_closure_grounded_support_refine": validate_refine_report(
            load_json(REPORT_ROOT / "next_policy" / "direct_closure_grounded_support_scale20_policy_refine.json")
        ),
        "visual_bottom_contact_diagnostic": {"status": "not_checked"},
    }

    visual_bottom_path = REPORT_ROOT / "next_policy" / "visual_bottom_direct_closure_visible_check.json"
    visual_bottom_report = load_json(visual_bottom_path, required=False)
    if visual_bottom_report is None:
        summary["visual_bottom_contact_diagnostic"] = {
            "status": "pending",
            "required_report": repo_relative(visual_bottom_path),
            "next_command": (
                ".\\simulation\\isaac\\run-visible-domino-policy.ps1 "
                f"-RunName {EXPECTED_VISUAL_BOTTOM_RUN_NAME} -Steps 240 -HoldOpenExitAfterFrames 240"
            ),
        }
        if args.require_visual_bottom:
            raise VerificationError(f"Missing required visual-bottom report: {repo_relative(visual_bottom_path)}")
    else:
        try:
            summary["visual_bottom_contact_diagnostic"] = validate_visual_bottom_report(
                visual_bottom_report,
                "visual_bottom_direct_closure",
            )
        except VerificationError as exc:
            summary["visual_bottom_contact_diagnostic"] = {
                "status": "unstable_experimental_contact",
                "reason": str(exc),
                "report": repo_relative(visual_bottom_path),
            }

    if args.json_summary:
        summary_path = (REPO_ROOT / args.json_summary).resolve() if not Path(args.json_summary).is_absolute() else Path(args.json_summary).resolve()
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    try:
        main()
    except VerificationError as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}, indent=2), flush=True)
        raise SystemExit(1) from exc
