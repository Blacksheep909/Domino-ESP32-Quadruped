"""Audit Domino CAD-linkage visuals without launching Isaac Sim rendering."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any


def isaac_roots_from_environment() -> list[Path]:
    roots: list[Path] = []
    for key in ("ISAAC_SIM_ROOT", "ISAAC_PATH"):
        value = os.environ.get(key)
        if value:
            roots.append(Path(value).expanduser())
    carb_app_path = os.environ.get("CARB_APP_PATH")
    if carb_app_path:
        carb_path = Path(carb_app_path).expanduser()
        roots.append(carb_path.parent if carb_path.name.lower() == "kit" else carb_path)
    return roots


def bootstrap_pxr_from_isaac_root(isaac_sim_root: Path) -> bool:
    root = isaac_sim_root.expanduser()
    if not root.exists():
        return False
    candidates = sorted((root / "extscache").glob("omni.usd.libs-*"))
    if not candidates:
        return False
    usd_lib = candidates[-1]
    sys.path.insert(0, str(usd_lib))
    for dll_dir in (usd_lib / "bin", root / "kit", root / "kit" / "python"):
        if dll_dir.exists() and hasattr(os, "add_dll_directory"):
            os.add_dll_directory(str(dll_dir))
    return True


try:
    from pxr import Usd, UsdGeom
except ModuleNotFoundError:
    for isaac_root in isaac_roots_from_environment():
        if bootstrap_pxr_from_isaac_root(isaac_root):
            break
    try:
        from pxr import Usd, UsdGeom
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Could not import pxr. Run with Isaac Sim Python or set ISAAC_SIM_ROOT/ISAAC_PATH before running this script."
        ) from exc


REPO_ROOT = Path(__file__).resolve().parents[4]
ACTUAL_CAD_DIR = Path(__file__).resolve().parents[1] / "actual_cad"
if str(ACTUAL_CAD_DIR) not in sys.path:
    sys.path.insert(0, str(ACTUAL_CAD_DIR))

from domino_actual_cad_usd import inspect_usd  # noqa: E402
from domino_cad_linkage_builder import (  # noqa: E402
    ACTUAL_CAD_STL_SOURCE,
    ACTUAL_CAD_VISUAL_USD,
    EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT,
    DominoCadLinkageBuildConfig,
    build_domino_four_12_floating_linkage,
    validate_domino_actual_cad_visuals,
)


def repo_relative(path: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def scrub_paths(summary: dict[str, Any]) -> dict[str, Any]:
    result = dict(summary)
    if "path" in result:
        result["path"] = repo_relative(Path(str(result["path"])))
    if "wrapper" in result and isinstance(result["wrapper"], dict):
        wrapper = dict(result["wrapper"])
        if "path" in wrapper:
            wrapper["path"] = repo_relative(Path(str(wrapper["path"])))
        result["wrapper"] = wrapper
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Audit the runtime Domino CAD-linkage visual scene.")
    parser.add_argument("--cad-usd", default=ACTUAL_CAD_VISUAL_USD, help="Real Domino CAD visual USD source.")
    parser.add_argument("--actual-cad-mesh-dir", default="", help="Optional override for the Domino STL mesh folder.")
    parser.add_argument("--output-usd", default="", help="Optional path to export the generated audit USD.")
    parser.add_argument("--json-report", default="", help="Optional JSON audit report path.")
    parser.add_argument("--min-source-mesh-count", type=int, default=20, help="Minimum mesh count expected in CAD USD.")
    args = parser.parse_args()

    cad_usd = (REPO_ROOT / args.cad_usd).resolve() if not Path(args.cad_usd).is_absolute() else Path(args.cad_usd).resolve()
    source_summary = inspect_usd(cad_usd)
    if int(source_summary["mesh_count"]) < int(args.min_source_mesh_count):
        raise RuntimeError(f"Source CAD USD has only {source_summary['mesh_count']} mesh prims.")
    if int(source_summary["cube_count"]) or int(source_summary["sphere_count"]):
        raise RuntimeError("Source CAD USD contains Cube/Sphere proxy visuals; this is not the clean Domino CAD source.")

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    linkage = build_domino_four_12_floating_linkage(
        stage,
        DominoCadLinkageBuildConfig(
            include_ground=False,
            include_actual_cad_visuals=True,
            hide_proxy_visuals_when_actual_cad=True,
            actual_cad_mesh_dir=str(args.actual_cad_mesh_dir),
        ),
    )
    runtime_visual_counts = validate_domino_actual_cad_visuals(stage, linkage, require_hidden_proxy=True)

    output_usd = ""
    if args.output_usd:
        output_path = (REPO_ROOT / args.output_usd).resolve() if not Path(args.output_usd).is_absolute() else Path(args.output_usd).resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(output_path))
        output_usd = repo_relative(output_path)

    report = {
        "status": "passed",
        "source_cad_usd": scrub_paths(source_summary),
        "runtime_linkage_visual": {
            "geometry": linkage["geometry"],
            "visual_fidelity": linkage["visual_fidelity"],
            "actual_cad_visual": bool(linkage["actual_cad_visual"]),
            "expected_domino_mesh_parts": EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT,
            "cad_source": {
                "visual_mesh_usd": ACTUAL_CAD_VISUAL_USD,
                "visual_mesh_stl": ACTUAL_CAD_STL_SOURCE,
                "physics_status": linkage["cad_source"]["physics_status"],
            },
            "actual_cad_visuals": linkage["actual_cad_visuals"],
            "visual_geometry_counts": runtime_visual_counts,
        },
        "output_usd": output_usd,
    }

    if args.json_report:
        report_path = (REPO_ROOT / args.json_report).resolve() if not Path(args.json_report).is_absolute() else Path(args.json_report).resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
