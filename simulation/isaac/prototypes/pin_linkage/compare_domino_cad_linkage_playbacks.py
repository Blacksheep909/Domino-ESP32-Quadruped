"""Compare Domino CAD-linkage playback reports against retained baselines."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[4]
EXPECTED_VISUAL_FIDELITY = "actual_cad_stl_visuals_on_proxy_physics"
EXPECTED_ACTION_COUNT = 12
EXPECTED_MESH_COUNT = 29
EXPECTED_TRIANGLE_COUNT = 135508

BASELINES = {
    "fastest_70deg_2p25hz": {
        "path": "simulation/isaac/out/cad_identity/rsl_play_scale70_freq225_weighted_bc_yawdiag_actual_cad.json",
        "role": "fastest retained learned baseline",
    },
    "lower_drift_60deg_2hz": {
        "path": "simulation/isaac/out/cad_identity/rsl_play_scale60_freq20_weighted_bc_actual_cad.json",
        "role": "lower lateral drift comparison",
    },
    "symmetry_trim_m6": {
        "path": "simulation/isaac/out/cad_identity/rsl_play_scale70_freq225_symmetry_trim_m6_weighted_bc_actual_cad.json",
        "role": "middle-ground symmetry baseline",
    },
    "symmetry_trim_m15": {
        "path": "simulation/isaac/out/cad_identity/rsl_play_scale70_freq225_symmetry_trim_m15_weighted_bc_actual_cad.json",
        "role": "low-yaw comparison",
    },
}


class ComparisonError(RuntimeError):
    """Raised when a playback report cannot be compared safely."""


def repo_path(path: str | Path) -> Path:
    path = Path(path)
    return path if path.is_absolute() else REPO_ROOT / path


def repo_relative(path: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise ComparisonError(f"Missing playback report: {repo_relative(path)}")
    return json.loads(path.read_text(encoding="utf-8"))


def mean(values: list[float]) -> float:
    if not values:
        raise ComparisonError("Cannot average an empty list.")
    return sum(float(value) for value in values) / float(len(values))


def vector_mean(report: dict[str, Any], key: str, index: int) -> float:
    rows = report.get(key)
    if not isinstance(rows, list) or not rows:
        raise ComparisonError(f"Report is missing non-empty vector list {key!r}.")
    return mean([float(row[index]) for row in rows])


def scalar_mean(report: dict[str, Any], key: str) -> float | None:
    values = report.get(key)
    if not isinstance(values, list) or not values:
        return None
    return mean([float(value) for value in values])


def validate_playback_identity(report: dict[str, Any], label: str) -> list[str]:
    errors: list[str] = []
    if report.get("status") != "passed":
        errors.append("status is not passed")
    if report.get("visual_fidelity") != EXPECTED_VISUAL_FIDELITY:
        errors.append("visual fidelity is not actual Domino CAD STL on proxy physics")
    if bool(report.get("actual_cad_visual")) is not True:
        errors.append("actual CAD visuals are disabled")
    if int(report.get("action_dim", -1)) != EXPECTED_ACTION_COUNT:
        errors.append("action_dim is not 12")

    visuals = report.get("actual_cad_visuals")
    if not isinstance(visuals, dict):
        errors.append("missing actual_cad_visuals")
    else:
        if int(visuals.get("mesh_count", -1)) != EXPECTED_MESH_COUNT:
            errors.append("wrong attached Domino STL mesh count")
        if int(visuals.get("triangle_count", -1)) != EXPECTED_TRIANGLE_COUNT:
            errors.append("wrong attached Domino STL triangle count")

    counts = report.get("visual_geometry_counts")
    if not isinstance(counts, dict):
        errors.append("missing visual_geometry_counts")
    else:
        if int(counts.get("visible_actual_cad_mesh_count", -1)) != EXPECTED_MESH_COUNT:
            errors.append("wrong visible Domino STL mesh count")
        if "visible_actual_cad_triangle_count" in counts and int(counts["visible_actual_cad_triangle_count"]) != EXPECTED_TRIANGLE_COUNT:
            errors.append("wrong visible Domino STL triangle count")
        if int(counts.get("visible_proxy_cube_count", -1)) != 0:
            errors.append("visible proxy cubes are present")
        if int(counts.get("visible_proxy_sphere_count", -1)) != 0:
            errors.append("visible proxy spheres are present")

    terminated = int(report.get("terminated_count", 0))
    done = int(report.get("done_count", 0))
    if terminated:
        errors.append(f"{terminated} fall terminations")
    if done:
        errors.append(f"{done} done events")

    if errors:
        raise ComparisonError(f"{label}: " + "; ".join(errors))
    return errors


def summarize_playback(report: dict[str, Any], label: str, path: Path) -> dict[str, Any]:
    validate_playback_identity(report, label)
    candidate = report.get("reference_gait_candidate")
    candidate_name = candidate.get("name") if isinstance(candidate, dict) else None
    counts = report.get("visual_geometry_counts") or {}
    return {
        "label": label,
        "path": repo_relative(path),
        "role": "",
        "checkpoint": report.get("checkpoint"),
        "checkpoint_run": report.get("checkpoint_run"),
        "candidate": candidate_name,
        "forward_mm": round(vector_mean(report, "final_body_reference_displacement_m", 0) * 1000.0, 1),
        "lateral_mm": round(vector_mean(report, "final_body_reference_displacement_m", 1) * 1000.0, 1),
        "abs_lateral_mm": round(abs(vector_mean(report, "final_body_reference_displacement_m", 1)) * 1000.0, 1),
        "final_yaw_rad": None
        if scalar_mean(report, "final_yaw_heading_drift_rad") is None
        else round(float(scalar_mean(report, "final_yaw_heading_drift_rad")), 3),
        "abs_final_yaw_rad": None
        if scalar_mean(report, "final_yaw_heading_drift_rad") is None
        else round(abs(float(scalar_mean(report, "final_yaw_heading_drift_rad"))), 3),
        "mean_reference_action_mse": None
        if scalar_mean(report, "mean_reference_action_mse") is None
        else round(float(scalar_mean(report, "mean_reference_action_mse")), 4),
        "mean_reward": None if report.get("mean_reward") is None else round(float(report["mean_reward"]), 4),
        "max_body_tilt_deg": None if report.get("max_body_tilt_deg") is None else round(float(report["max_body_tilt_deg"]), 2),
        "terminated_count": int(report.get("terminated_count", 0)),
        "done_count": int(report.get("done_count", 0)),
        "actual_cad_meshes": int((report.get("actual_cad_visuals") or {}).get("mesh_count", -1)),
        "visible_actual_cad_triangles": counts.get("visible_actual_cad_triangle_count"),
        "proxy_visuals_hidden": bool(counts.get("proxy_visuals_hidden")),
        "proxy_visuals_non_renderable": bool(counts.get("proxy_visuals_non_renderable", False)),
    }


def comparison(candidate: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    result = {
        "baseline": baseline["label"],
        "baseline_role": baseline.get("role", ""),
        "delta_forward_mm": round(float(candidate["forward_mm"]) - float(baseline["forward_mm"]), 1),
        "delta_abs_lateral_mm": round(float(candidate["abs_lateral_mm"]) - float(baseline["abs_lateral_mm"]), 1),
        "beats_forward": float(candidate["forward_mm"]) > float(baseline["forward_mm"]),
        "beats_abs_lateral": float(candidate["abs_lateral_mm"]) < float(baseline["abs_lateral_mm"]),
    }
    if candidate.get("abs_final_yaw_rad") is not None and baseline.get("abs_final_yaw_rad") is not None:
        result["delta_abs_yaw_rad"] = round(float(candidate["abs_final_yaw_rad"]) - float(baseline["abs_final_yaw_rad"]), 3)
        result["beats_abs_yaw"] = float(candidate["abs_final_yaw_rad"]) < float(baseline["abs_final_yaw_rad"])
    if candidate.get("mean_reference_action_mse") is not None and baseline.get("mean_reference_action_mse") is not None:
        result["delta_reference_mse"] = round(
            float(candidate["mean_reference_action_mse"]) - float(baseline["mean_reference_action_mse"]),
            4,
        )
        result["beats_reference_mse"] = float(candidate["mean_reference_action_mse"]) < float(
            baseline["mean_reference_action_mse"]
        )
    return result


def verdict(candidate: dict[str, Any], comparisons: list[dict[str, Any]]) -> dict[str, Any]:
    fastest = next(item for item in comparisons if item["baseline"] == "fastest_70deg_2p25hz")
    m6 = next(item for item in comparisons if item["baseline"] == "symmetry_trim_m6")
    m15 = next(item for item in comparisons if item["baseline"] == "symmetry_trim_m15")
    near_fastest_forward = float(fastest["delta_forward_mm"]) >= -2.0
    balanced_upgrade_vs_fastest = bool(
        near_fastest_forward and fastest["beats_abs_lateral"] and fastest.get("beats_abs_yaw", False)
    )
    if fastest["beats_forward"]:
        summary = "new best forward policy candidate"
    elif balanced_upgrade_vs_fastest:
        summary = "near-fastest balanced policy candidate"
    else:
        summary = "stable but not a new best forward policy"
    return {
        "beats_fastest_forward": bool(fastest["beats_forward"]),
        "near_fastest_forward": bool(near_fastest_forward),
        "beats_fastest_lateral": bool(fastest["beats_abs_lateral"]),
        "beats_fastest_yaw": bool(fastest.get("beats_abs_yaw", False)),
        "balanced_upgrade_vs_fastest": bool(balanced_upgrade_vs_fastest),
        "beats_m6_forward": bool(m6["beats_forward"]),
        "beats_m6_yaw": bool(m6.get("beats_abs_yaw", False)),
        "beats_m15_yaw": bool(m15.get("beats_abs_yaw", False)),
        "usable_next_policy": bool(
            candidate["terminated_count"] == 0
            and candidate["done_count"] == 0
            and candidate["actual_cad_meshes"] == EXPECTED_MESH_COUNT
            and candidate["proxy_visuals_hidden"]
        ),
        "summary": summary,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare a Domino CAD-linkage playback report against retained baselines.")
    parser.add_argument(
        "--candidate",
        default="simulation/isaac/out/cad_identity/next_policy/rsl_play_trim_m6_yaw_lateral_refine.json",
        help="Candidate playback JSON report.",
    )
    parser.add_argument("--json-summary", default="", help="Optional output JSON summary path.")
    args = parser.parse_args()

    candidate_path = repo_path(args.candidate)
    candidate_summary = summarize_playback(load_json(candidate_path), "candidate", candidate_path)

    baseline_summaries: dict[str, Any] = {}
    for label, config in BASELINES.items():
        path = repo_path(config["path"])
        summary = summarize_playback(load_json(path), label, path)
        summary["role"] = config["role"]
        baseline_summaries[label] = summary

    comparisons = [comparison(candidate_summary, baseline) for baseline in baseline_summaries.values()]
    result = {
        "status": "passed",
        "candidate": candidate_summary,
        "baselines": baseline_summaries,
        "comparisons": comparisons,
        "verdict": verdict(candidate_summary, comparisons),
    }

    if args.json_summary:
        summary_path = repo_path(args.json_summary)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    try:
        main()
    except ComparisonError as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}, indent=2), flush=True)
        raise SystemExit(1) from exc
