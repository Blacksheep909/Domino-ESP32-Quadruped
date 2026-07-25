"""Verify local Domino CAD-linkage audit/playback report artifacts.

This is a non-GPU consistency check for the current Isaac bring-up state. It
does not replace rerunning Isaac Sim, but it catches stale or wrong report
artifacts before they are used as evidence in docs or portfolio notes.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[4]
EXPECTED_VISUAL_FIDELITY = "actual_cad_stl_visuals_on_proxy_physics"
EXPECTED_MESH_COUNT = 29
EXPECTED_TRIANGLE_COUNT = 135508
EXPECTED_ACTION_COUNT = 12


PLAYBACK_BASELINES = {
    "fastest_70deg_2p25hz": {
        "path": "rsl_play_scale70_freq225_weighted_bc_yawdiag_actual_cad.json",
        "candidate": "random_001",
        "min_forward_m": 0.060,
        "max_abs_lateral_m": 0.010,
        "max_abs_final_yaw_rad": 0.350,
        "max_mean_reference_mse": 0.0030,
    },
    "symmetry_trim_m6": {
        "path": "rsl_play_scale70_freq225_symmetry_trim_m6_weighted_bc_actual_cad.json",
        "candidate": "random_001_right_left_phase_trim_-6deg",
        "min_forward_m": 0.050,
        "max_abs_lateral_m": 0.010,
        "max_abs_final_yaw_rad": 0.250,
        "max_mean_reference_mse": 0.0030,
    },
    "symmetry_trim_m15": {
        "path": "rsl_play_scale70_freq225_symmetry_trim_m15_weighted_bc_actual_cad.json",
        "candidate": "random_001_right_left_phase_trim_-15deg",
        "min_forward_m": 0.040,
        "max_abs_lateral_m": 0.010,
        "max_abs_final_yaw_rad": 0.180,
        "max_mean_reference_mse": 0.0030,
    },
    "lower_drift_60deg_2hz": {
        "path": "rsl_play_scale60_freq20_weighted_bc_actual_cad.json",
        "candidate": "random_001",
        "min_forward_m": 0.055,
        "max_abs_lateral_m": 0.010,
        "max_mean_reference_mse": 0.0030,
    },
}

PLAYBACK_CANDIDATES = {
    "fastest_70deg_heading_refine_model20": {
        "path": "next_policy/rsl_play_fastest_70deg_heading_refine_model20.json",
        "candidate": "random_001",
        "min_forward_m": 0.060,
        "max_abs_lateral_m": 0.002,
        "max_abs_final_yaw_rad": 0.300,
        "max_mean_reference_mse": 0.0040,
        "require_proxy_non_renderable": True,
    },
    "fastest_70deg_heading_refine_model30": {
        "path": "next_policy/rsl_play_fastest_70deg_heading_refine_model30.json",
        "candidate": "random_001",
        "min_forward_m": 0.070,
        "max_abs_lateral_m": 0.005,
        "max_abs_final_yaw_rad": 0.380,
        "max_mean_reference_mse": 0.0060,
        "require_proxy_non_renderable": True,
    },
    "fastest_70deg_heading_refine_model40": {
        "path": "next_policy/rsl_play_fastest_70deg_heading_refine_model40.json",
        "candidate": "random_001",
        "min_forward_m": 0.075,
        "max_abs_lateral_m": 0.003,
        "max_abs_final_yaw_rad": 0.420,
        "max_mean_reference_mse": 0.0070,
        "require_proxy_non_renderable": True,
    },
    "fastest_70deg_heading_refine_model79": {
        "path": "next_policy/rsl_play_fastest_70deg_heading_refine.json",
        "candidate": "random_001",
        "min_forward_m": 0.080,
        "max_abs_lateral_m": 0.008,
        "max_abs_final_yaw_rad": 0.420,
        "max_mean_reference_mse": 0.0120,
        "require_proxy_non_renderable": True,
    },
    "model20_straight_refine_model30": {
        "path": "next_policy/rsl_play_model20_straight_refine_model30.json",
        "candidate": "random_001",
        "min_forward_m": 0.058,
        "max_abs_lateral_m": 0.002,
        "max_abs_final_yaw_rad": 0.250,
        "max_mean_reference_mse": 0.0050,
        "require_proxy_non_renderable": True,
    },
    "model20_straight_refine_model40": {
        "path": "next_policy/rsl_play_model20_straight_refine_model40.json",
        "candidate": "random_001",
        "min_forward_m": 0.060,
        "max_abs_lateral_m": 0.002,
        "max_abs_final_yaw_rad": 0.270,
        "max_mean_reference_mse": 0.0060,
        "require_proxy_non_renderable": True,
    },
    "model20_straight_refine_model48": {
        "path": "next_policy/rsl_play_model20_straight_refine.json",
        "candidate": "random_001",
        "min_forward_m": 0.060,
        "max_abs_lateral_m": 0.003,
        "max_abs_final_yaw_rad": 0.290,
        "max_mean_reference_mse": 0.0070,
        "require_proxy_non_renderable": True,
    },
    "model20_forward_preserving_model40": {
        "path": "next_policy/rsl_play_model20_forward_preserving_straight_refine_model40.json",
        "candidate": "random_001",
        "min_forward_m": 0.062,
        "max_abs_lateral_m": 0.002,
        "max_abs_final_yaw_rad": 0.280,
        "max_mean_reference_mse": 0.0045,
        "require_proxy_non_renderable": True,
    },
    "model20_forward_preserving_model50": {
        "path": "next_policy/rsl_play_model20_forward_preserving_straight_refine_model50.json",
        "candidate": "random_001",
        "min_forward_m": 0.064,
        "max_abs_lateral_m": 0.003,
        "max_abs_final_yaw_rad": 0.290,
        "max_mean_reference_mse": 0.0045,
        "require_proxy_non_renderable": True,
    },
    "model20_forward_preserving_model59": {
        "path": "next_policy/rsl_play_model20_forward_preserving_straight_refine.json",
        "candidate": "random_001",
        "min_forward_m": 0.067,
        "max_abs_lateral_m": 0.004,
        "max_abs_final_yaw_rad": 0.330,
        "max_mean_reference_mse": 0.0050,
        "require_proxy_non_renderable": True,
    },
}

CURRENT_ACTUAL_CAD_SCENE_CHECKS = {
    "zero_actual_cad_stable_proxy_flat": {
        "path": "next_policy/zero_actual_cad_stable_proxy_flat_160steps.json",
        "status": "passed",
        "max_joint_separation_m": 0.003,
        "max_tilt_deg": 30.0,
        "max_terminated_count": 0,
        "max_done_count": 0,
        "require_visible_collision_ground": True,
        "forbid_visible_floor": True,
        "require_actual_cad_visual_bottom_alignment": True,
        "require_stable_proxy_contact": True,
    },
    "model79_actual_cad_stable_proxy_stairs": {
        "path": "next_policy/policy_model79_actual_cad_stable_proxy_stairs_240steps.json",
        "status": "passed",
        "max_joint_separation_m": 0.003,
        "max_tilt_deg": 30.0,
        "max_terminated_count": 0,
        "max_done_count": 0,
        "min_forward_m": 0.020,
        "max_abs_lateral_m": 0.060,
        "require_visible_collision_ground": True,
        "forbid_visible_floor": True,
        "require_actual_cad_visual_bottom_alignment": True,
        "require_stable_proxy_contact": True,
    },
    "visible_training_smoke_model80": {
        "path": "next_policy/visible_actual_cad_stable_proxy_training_smoke.json",
        "status": {"passed", "passed_with_policy_validation_warnings", "policy_validation_failed"},
        "max_joint_separation_m": 0.003,
        "max_tilt_deg": 30.0,
        "max_terminated_count": 0,
        "max_done_count": 0,
        "min_validation_forward_m": 0.020,
        "max_validation_abs_lateral_m": 0.060,
        "require_checkpoint": "model_80.pt",
        "require_visible_collision_ground": True,
        "forbid_visible_floor": True,
        "require_actual_cad_visual_bottom_alignment": True,
        "require_stable_proxy_contact": True,
        "allow_policy_validation_warnings": True,
    },
    "visible_policy_model80_bounded": {
        "path": "next_policy/visible_actual_cad_stable_proxy_policy_model80.json",
        "status": "passed",
        "max_joint_separation_m": 0.003,
        "max_tilt_deg": 30.0,
        "max_terminated_count": 0,
        "max_done_count": 0,
        "min_forward_m": 0.020,
        "max_abs_lateral_m": 0.060,
        "require_visible_collision_ground": True,
        "forbid_visible_floor": True,
        "require_actual_cad_visual_bottom_alignment": True,
        "require_stable_proxy_contact": True,
    },
}


class VerificationError(RuntimeError):
    """Raised when a report artifact does not prove the expected sim state."""


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise VerificationError(f"Missing report: {repo_relative(path)}")
    return json.loads(path.read_text(encoding="utf-8"))


def repo_relative(path: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def mean(values: list[float]) -> float:
    if not values:
        raise VerificationError("Cannot take mean of an empty list.")
    return sum(float(value) for value in values) / float(len(values))


def vector_mean(report: dict[str, Any], key: str, index: int) -> float:
    rows = report.get(key)
    if not isinstance(rows, list) or not rows:
        raise VerificationError(f"Report is missing non-empty vector list {key!r}.")
    return mean([float(row[index]) for row in rows])


def scalar_mean(report: dict[str, Any], key: str) -> float:
    values = report.get(key)
    if not isinstance(values, list) or not values:
        raise VerificationError(f"Report is missing non-empty scalar list {key!r}.")
    return mean([float(value) for value in values])


def check(condition: bool, message: str) -> None:
    if not condition:
        raise VerificationError(message)


def validate_visual_counts(counts: dict[str, Any], prefix: str, require_visible_triangle_count: bool = True) -> bool:
    check(int(counts.get("visible_actual_cad_mesh_count", -1)) == EXPECTED_MESH_COUNT, f"{prefix}: wrong visible CAD mesh count.")
    visible_triangle_count_verified = "visible_actual_cad_triangle_count" in counts
    if require_visible_triangle_count or visible_triangle_count_verified:
        check(
            int(counts.get("visible_actual_cad_triangle_count", -1)) == EXPECTED_TRIANGLE_COUNT,
            f"{prefix}: wrong visible CAD triangle count.",
        )
    check(int(counts.get("visible_proxy_cube_count", -1)) == 0, f"{prefix}: proxy cubes are visible.")
    check(int(counts.get("visible_proxy_sphere_count", -1)) == 0, f"{prefix}: proxy spheres are visible.")
    check(bool(counts.get("proxy_visuals_hidden")) is True, f"{prefix}: proxy visuals are not marked hidden.")
    return visible_triangle_count_verified


def validate_cad_identity_report(report: dict[str, Any]) -> dict[str, Any]:
    check(report.get("status") == "passed", "CAD identity audit did not pass.")
    source = report.get("source_cad_usd")
    runtime = report.get("runtime_linkage_visual")
    check(isinstance(source, dict), "CAD identity audit is missing source_cad_usd.")
    check(isinstance(runtime, dict), "CAD identity audit is missing runtime_linkage_visual.")
    check(int(source.get("mesh_count", -1)) >= 30, "Source CAD USD has too few mesh prims.")
    check(int(source.get("cube_count", -1)) == 0, "Source CAD USD contains cube proxy prims.")
    check(int(source.get("sphere_count", -1)) == 0, "Source CAD USD contains sphere proxy prims.")
    check(bool(source.get("is_mesh_cad_visual")) is True, "Source CAD USD is not marked as mesh CAD visual.")
    check(runtime.get("visual_fidelity") == EXPECTED_VISUAL_FIDELITY, "Runtime visual fidelity is not the CAD STL overlay.")
    check(bool(runtime.get("actual_cad_visual")) is True, "Runtime linkage does not report actual CAD visuals.")
    visuals = runtime.get("actual_cad_visuals")
    check(isinstance(visuals, dict), "Runtime linkage is missing actual_cad_visuals.")
    check(int(visuals.get("mesh_count", -1)) == EXPECTED_MESH_COUNT, "Runtime linkage attached the wrong CAD mesh count.")
    check(int(visuals.get("triangle_count", -1)) == EXPECTED_TRIANGLE_COUNT, "Runtime linkage attached the wrong CAD triangle count.")
    counts = runtime.get("visual_geometry_counts")
    check(isinstance(counts, dict), "Runtime linkage is missing visual_geometry_counts.")
    validate_visual_counts(counts, "cad_identity", require_visible_triangle_count=True)
    check(
        bool(counts.get("proxy_visuals_non_renderable")) is True,
        "cad_identity: proxy collision/debug geometry is not marked guide-purpose transparent.",
    )
    return {
        "source_mesh_count": int(source["mesh_count"]),
        "visible_runtime_mesh_count": int(counts["visible_actual_cad_mesh_count"]),
        "visible_runtime_triangle_count": int(counts["visible_actual_cad_triangle_count"]),
    }


def validate_action_contract(report: dict[str, Any], label: str) -> None:
    check(int(report.get("action_dim", -1)) == EXPECTED_ACTION_COUNT, f"{label}: action_dim is not 12.")
    action_names = report.get("action_names")
    check(isinstance(action_names, list) and len(action_names) == EXPECTED_ACTION_COUNT, f"{label}: missing 12 action names.")
    check(len(set(action_names)) == EXPECTED_ACTION_COUNT, f"{label}: action names are not unique.")
    counts = report.get("action_group_counts")
    check(isinstance(counts, dict), f"{label}: missing action_group_counts.")
    expected_counts = {
        "shoulder_hip_ab_ad": 4,
        "lower_linkage_drive": 4,
        "upper_pitch_drive": 4,
        "linkage_drive_total": 8,
        "total": EXPECTED_ACTION_COUNT,
    }
    for key, expected in expected_counts.items():
        check(int(counts.get(key, -1)) == expected, f"{label}: action group {key!r} is not {expected}.")


def validate_playback_report(report: dict[str, Any], label: str, cfg: dict[str, Any]) -> dict[str, Any]:
    check(report.get("status") == "passed", f"{label}: playback did not pass.")
    check(report.get("visual_fidelity") == EXPECTED_VISUAL_FIDELITY, f"{label}: playback visual fidelity is not actual CAD.")
    check(bool(report.get("actual_cad_visual")) is True, f"{label}: actual CAD visuals are disabled.")
    visuals = report.get("actual_cad_visuals")
    check(isinstance(visuals, dict), f"{label}: missing actual CAD attachment details.")
    check(int(visuals.get("mesh_count", -1)) == EXPECTED_MESH_COUNT, f"{label}: wrong attached CAD mesh count.")
    check(int(visuals.get("triangle_count", -1)) == EXPECTED_TRIANGLE_COUNT, f"{label}: wrong attached CAD triangle count.")
    counts = report.get("visual_geometry_counts")
    check(isinstance(counts, dict), f"{label}: missing visual geometry counts.")
    visible_triangle_count_verified = validate_visual_counts(counts, label, require_visible_triangle_count=False)
    if cfg.get("require_proxy_non_renderable"):
        check(
            bool(counts.get("proxy_visuals_non_renderable")) is True,
            f"{label}: proxy collision/debug geometry is not marked guide-purpose transparent.",
        )
    validate_action_contract(report, label)
    candidate = report.get("reference_gait_candidate")
    check(isinstance(candidate, dict), f"{label}: missing reference gait candidate.")
    check(candidate.get("name") == cfg["candidate"], f"{label}: wrong candidate {candidate.get('name')!r}.")
    terminated = int(report.get("terminated_count", 0))
    done = int(report.get("done_count", 0))
    check(terminated == 0, f"{label}: playback had {terminated} fall terminations.")
    check(done == 0, f"{label}: playback had {done} done events.")
    forward_m = vector_mean(report, "final_body_reference_displacement_m", 0)
    lateral_m = vector_mean(report, "final_body_reference_displacement_m", 1)
    check(forward_m >= float(cfg["min_forward_m"]), f"{label}: forward displacement is too low: {forward_m:.6f} m.")
    check(abs(lateral_m) <= float(cfg["max_abs_lateral_m"]), f"{label}: lateral drift is too high: {lateral_m:.6f} m.")
    if "max_abs_final_yaw_rad" in cfg:
        final_yaw = scalar_mean(report, "final_yaw_heading_drift_rad")
        check(abs(final_yaw) <= float(cfg["max_abs_final_yaw_rad"]), f"{label}: final yaw drift is too high: {final_yaw:.6f} rad.")
    else:
        final_yaw = None
    ref_mse = scalar_mean(report, "mean_reference_action_mse")
    check(ref_mse <= float(cfg["max_mean_reference_mse"]), f"{label}: reference action MSE is too high: {ref_mse:.6f}.")
    return {
        "candidate": candidate["name"],
        "forward_mm": round(forward_m * 1000.0, 1),
        "lateral_mm": round(lateral_m * 1000.0, 1),
        "final_yaw_rad": None if final_yaw is None else round(final_yaw, 3),
        "mean_reference_action_mse": round(ref_mse, 4),
        "visible_triangle_count_verified": visible_triangle_count_verified,
    }


def vector_from_rows(report: dict[str, Any], key: str) -> list[float]:
    rows = report.get(key)
    if not isinstance(rows, list) or not rows:
        raise VerificationError(f"Report is missing non-empty vector list {key!r}.")
    first = rows[0]
    if not isinstance(first, list) or len(first) < 3:
        raise VerificationError(f"Report vector list {key!r} does not contain xyz rows.")
    return [float(first[0]), float(first[1]), float(first[2])]


def validate_current_actual_cad_scene_report(report: dict[str, Any], label: str, cfg: dict[str, Any]) -> dict[str, Any]:
    expected_status = cfg.get("status", "passed")
    if isinstance(expected_status, set):
        check(report.get("status") in expected_status, f"{label}: unexpected status {report.get('status')!r}.")
    else:
        check(report.get("status") == expected_status, f"{label}: expected status {expected_status!r}, got {report.get('status')!r}.")
    check(report.get("visual_fidelity") == EXPECTED_VISUAL_FIDELITY, f"{label}: visual fidelity is not actual CAD.")
    check(bool(report.get("actual_cad_visual")) is True, f"{label}: actual CAD visuals are disabled.")
    visuals = report.get("actual_cad_visuals")
    check(isinstance(visuals, dict), f"{label}: missing actual CAD attachment details.")
    check(int(visuals.get("mesh_count", -1)) == EXPECTED_MESH_COUNT, f"{label}: wrong attached CAD mesh count.")
    check(int(visuals.get("triangle_count", -1)) == EXPECTED_TRIANGLE_COUNT, f"{label}: wrong attached CAD triangle count.")
    counts = report.get("visual_geometry_counts")
    check(isinstance(counts, dict), f"{label}: missing visual geometry counts.")
    validate_visual_counts(counts, label, require_visible_triangle_count=True)
    check(bool(counts.get("proxy_visuals_non_renderable")) is True, f"{label}: proxy visuals are not guide/non-renderable.")
    validate_action_contract(report, label)

    if bool(cfg.get("require_actual_cad_visual_bottom_alignment", False)):
        alignment = report.get("actual_cad_visual_alignment")
        check(isinstance(alignment, dict), f"{label}: missing actual CAD visual alignment report.")
        check(bool(alignment.get("visual_bottom_aligned_to_ground")) is True, f"{label}: CAD visual bottom is not aligned to the ground.")
        check(
            float(alignment.get("visual_z_lift_m", 0.0)) > 0.0,
            f"{label}: CAD visual world-z lift was not applied.",
        )
        clearance_m = float(alignment.get("ground_clearance_m", 0.002))
        min_bottom_m = float(alignment.get("min_visual_bottom_z_with_offset_m", 999.0))
        check(
            abs(min_bottom_m - clearance_m) <= 1.0e-4,
            f"{label}: CAD visual bottom clearance is {min_bottom_m:.6f} m, expected {clearance_m:.6f} m.",
        )

    contact = report.get("actual_cad_foot_collision")
    check(isinstance(contact, dict), f"{label}: missing actual_cad_foot_collision details.")
    if bool(cfg.get("require_stable_proxy_contact", False)):
        check(contact.get("source") == "linkage_lower_closure", f"{label}: expected stable lower-closure proxy contacts.")
        check(bool(contact.get("enabled")) is False, f"{label}: actual CAD visual foot collision should not be enabled.")

    terrain = report.get("terrain")
    check(isinstance(terrain, dict), f"{label}: missing terrain report.")
    if bool(cfg.get("require_hidden_physics_ground", False)):
        ground = terrain.get("ground_box")
        check(isinstance(ground, dict), f"{label}: missing physics ground box.")
        check(bool(ground.get("collision")) is True, f"{label}: physics ground must keep collision enabled.")
        check(bool(ground.get("visible")) is False, f"{label}: physics ground should be hidden in actual-CAD visual checks.")
    if bool(cfg.get("require_visible_collision_ground", False)):
        ground = terrain.get("ground_box")
        check(isinstance(ground, dict), f"{label}: missing physics ground box.")
        check(bool(ground.get("collision")) is True, f"{label}: ground collision is disabled.")
        check(bool(ground.get("visible")) is True, f"{label}: real collision ground is hidden.")
    if bool(cfg.get("forbid_visible_floor", False)):
        check(terrain.get("visible_ground_box") in (None, {}), f"{label}: fake visible-only ground is still present.")
        visible_stairs = terrain.get("visible_stairs")
        check(visible_stairs in (None, []) or len(visible_stairs) == 0, f"{label}: fake visible-only stairs are still present.")

    joint_separation = report.get("joint_separation")
    check(isinstance(joint_separation, dict), f"{label}: missing joint separation report.")
    max_separation = float(joint_separation.get("max_separation_m", 999.0))
    check(
        max_separation <= float(cfg.get("max_joint_separation_m", 0.003)),
        f"{label}: joint separation too high: {max_separation:.6f} m.",
    )
    done_count = int(report.get("done_count", report.get("trained_policy_validation", {}).get("done_count", 0)))
    terminated_count = int(report.get("terminated_count", report.get("trained_policy_validation", {}).get("terminated_count", 0)))
    check(done_count <= int(cfg.get("max_done_count", 0)), f"{label}: had {done_count} done events.")
    check(terminated_count <= int(cfg.get("max_terminated_count", 0)), f"{label}: had {terminated_count} terminations.")

    validation = report.get("trained_policy_validation")
    if isinstance(validation, dict) and validation.get("enabled"):
        tilt_deg = float(validation.get("max_body_tilt_deg", 999.0))
        displacement = vector_from_rows(validation, "final_body_reference_displacement_m")
        validation_passed = bool(validation.get("passed", False))
        if not validation_passed and not bool(cfg.get("allow_policy_validation_warnings", False)):
            raise VerificationError(f"{label}: trained policy validation did not pass: {validation.get('failures')}.")
    else:
        tilt_deg = float(report.get("max_body_tilt_deg", 999.0))
        displacement = vector_from_rows(report, "final_body_reference_displacement_m")
        validation_passed = None
    check(tilt_deg <= float(cfg.get("max_tilt_deg", 30.0)), f"{label}: max tilt too high: {tilt_deg:.3f} deg.")
    if "min_forward_m" in cfg:
        check(displacement[0] >= float(cfg["min_forward_m"]), f"{label}: forward displacement too low: {displacement[0]:.6f} m.")
    if "max_abs_lateral_m" in cfg:
        check(abs(displacement[1]) <= float(cfg["max_abs_lateral_m"]), f"{label}: lateral displacement too high: {displacement[1]:.6f} m.")
    if "min_validation_forward_m" in cfg:
        check(displacement[0] >= float(cfg["min_validation_forward_m"]), f"{label}: validation forward displacement too low: {displacement[0]:.6f} m.")
    if "max_validation_abs_lateral_m" in cfg:
        check(abs(displacement[1]) <= float(cfg["max_validation_abs_lateral_m"]), f"{label}: validation lateral displacement too high: {displacement[1]:.6f} m.")
    if "require_checkpoint" in cfg:
        checkpoints = report.get("checkpoints")
        check(isinstance(checkpoints, list), f"{label}: missing checkpoint list.")
        check(str(cfg["require_checkpoint"]) in checkpoints, f"{label}: required checkpoint {cfg['require_checkpoint']!r} was not produced.")

    return {
        "status": report.get("status"),
        "max_joint_separation_m": round(max_separation, 6),
        "max_tilt_deg": round(tilt_deg, 3),
        "done_count": done_count,
        "terminated_count": terminated_count,
        "forward_mm": round(displacement[0] * 1000.0, 1),
        "lateral_mm": round(displacement[1] * 1000.0, 1),
        "validation_passed": validation_passed,
        "validation_failures": validation.get("failures", []) if isinstance(validation, dict) else [],
    }


def require_doc_tokens(path: Path, tokens: list[str]) -> dict[str, Any]:
    if not path.exists():
        raise VerificationError(f"Missing documentation file: {repo_relative(path)}")
    text = path.read_text(encoding="utf-8")
    missing = [token for token in tokens if token not in text]
    if missing:
        raise VerificationError(f"{repo_relative(path)} is missing expected current-status text: {missing}")
    return {"path": repo_relative(path), "checked_tokens": len(tokens)}


def playback_tokens(playback: dict[str, Any], include_yaw: bool = True) -> list[str]:
    tokens = [f"{float(playback['forward_mm']):.1f} mm", f"{float(playback['lateral_mm']):.1f} mm"]
    if include_yaw and playback.get("final_yaw_rad") is not None:
        tokens.append(f"{float(playback['final_yaw_rad']):.3f} rad")
    return tokens


def validate_documentation(summary: dict[str, Any]) -> dict[str, Any]:
    playbacks = summary["playbacks"]
    if not isinstance(playbacks, dict):
        raise VerificationError("Verification summary is missing playback metrics.")
    candidates = summary.get("candidates", {})
    if not isinstance(candidates, dict):
        raise VerificationError("Verification summary has invalid candidate metrics.")

    readme_tokens = [
        "verify_domino_cad_linkage_reports.py",
        "plan_domino_cad_linkage_next_runs.py",
        "Weighted 70 deg / 2.25 Hz BC",
        "Weighted 60 deg / 2.0 Hz BC",
        "Right/left phase trim `-6 deg` BC",
        "Right/left phase trim `-15 deg` BC",
    ]
    readme_tokens.extend(playback_tokens(playbacks["fastest_70deg_2p25hz"]))
    readme_tokens.extend(playback_tokens(playbacks["lower_drift_60deg_2hz"], include_yaw=False))
    readme_tokens.extend(playback_tokens(playbacks["symmetry_trim_m6"]))
    readme_tokens.extend(playback_tokens(playbacks["symmetry_trim_m15"]))
    readme_tokens.extend(playback_tokens(candidates["fastest_70deg_heading_refine_model20"]))
    readme_tokens.extend(playback_tokens(candidates["fastest_70deg_heading_refine_model79"]))
    readme_tokens.extend(playback_tokens(candidates["model20_forward_preserving_model59"]))

    direct_report_tokens = [
        "Runtime Domino STL mesh parts",
        "Runtime Domino STL triangles",
        str(EXPECTED_TRIANGLE_COUNT),
        "Current intermediate symmetry baseline",
        "Current low-yaw comparison baseline",
        "Latest 70-Degree PPO Continuation",
    ]
    direct_report_tokens.extend(playback_tokens(playbacks["fastest_70deg_2p25hz"]))
    direct_report_tokens.extend(playback_tokens(playbacks["lower_drift_60deg_2hz"], include_yaw=False))
    direct_report_tokens.extend(playback_tokens(playbacks["symmetry_trim_m6"]))
    direct_report_tokens.extend(playback_tokens(playbacks["symmetry_trim_m15"]))
    for key in (
        "fastest_70deg_heading_refine_model20",
        "fastest_70deg_heading_refine_model30",
        "fastest_70deg_heading_refine_model40",
        "fastest_70deg_heading_refine_model79",
        "model20_straight_refine_model30",
        "model20_straight_refine_model40",
        "model20_straight_refine_model48",
        "model20_forward_preserving_model40",
        "model20_forward_preserving_model50",
        "model20_forward_preserving_model59",
    ):
        direct_report_tokens.extend(playback_tokens(candidates[key]))

    cad_audit_tokens = [
        "Domino Actual CAD Asset Audit",
        str(EXPECTED_MESH_COUNT),
        str(EXPECTED_TRIANGLE_COUNT),
        "Visible proxy cubes",
        "Visible proxy spheres",
        "Proxy visual render guard",
    ]

    return {
        "readme": require_doc_tokens(REPO_ROOT / "simulation/isaac/README.md", readme_tokens),
        "direct_rl_report": require_doc_tokens(
            REPO_ROOT / "simulation/isaac/reports/domino-cad-linkage-direct-rl-env.md",
            direct_report_tokens,
        ),
        "cad_audit_report": require_doc_tokens(
            REPO_ROOT / "simulation/isaac/reports/domino-actual-cad-asset-audit.md",
            cad_audit_tokens,
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify local Domino CAD-linkage report artifacts.")
    parser.add_argument(
        "--report-root",
        default="simulation/isaac/out/cad_identity",
        help="Folder containing local CAD identity and playback JSON reports.",
    )
    parser.add_argument("--json-summary", default="", help="Optional path to write the verification summary JSON.")
    parser.add_argument("--skip-doc-checks", action="store_true", help="Only verify JSON reports, not Markdown status docs.")
    args = parser.parse_args()

    report_root = (REPO_ROOT / args.report_root).resolve() if not Path(args.report_root).is_absolute() else Path(args.report_root).resolve()
    summary: dict[str, Any] = {
        "status": "passed",
        "report_root": repo_relative(report_root),
        "cad_identity": validate_cad_identity_report(load_json(report_root / "domino_actual_cad_linkage_visual_audit.json")),
        "playbacks": {},
        "candidates": {},
        "current_actual_cad_scene": {},
    }
    for label, cfg in PLAYBACK_BASELINES.items():
        summary["playbacks"][label] = validate_playback_report(load_json(report_root / str(cfg["path"])), label, cfg)
    for label, cfg in PLAYBACK_CANDIDATES.items():
        summary["candidates"][label] = validate_playback_report(load_json(report_root / str(cfg["path"])), label, cfg)
    for label, cfg in CURRENT_ACTUAL_CAD_SCENE_CHECKS.items():
        summary["current_actual_cad_scene"][label] = validate_current_actual_cad_scene_report(
            load_json(report_root / str(cfg["path"])),
            label,
            cfg,
        )
    if not args.skip_doc_checks:
        summary["documentation"] = validate_documentation(summary)

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
