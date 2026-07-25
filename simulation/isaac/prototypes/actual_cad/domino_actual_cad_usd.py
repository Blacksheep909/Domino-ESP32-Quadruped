"""Audit and wrap the real Domino CAD USD visual export.

This script does not launch Isaac Sim or create physics. It uses USD directly
to distinguish the real mesh CAD export from the simplified pin-linkage proxy
scenes.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[4]


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
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
except ModuleNotFoundError:
    for isaac_root in isaac_roots_from_environment():
        if bootstrap_pxr_from_isaac_root(isaac_root):
            break
    try:
        from pxr import Sdf, Usd, UsdGeom, UsdPhysics
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Could not import pxr. Run with Isaac Sim Python or set ISAAC_SIM_ROOT/ISAAC_PATH before running this script."
        ) from exc


def asset_paths_from_list_op(list_op: Any) -> list[str]:
    if not list_op:
        return []
    if hasattr(list_op, "GetAddedOrExplicitItems"):
        items = list_op.GetAddedOrExplicitItems()
    else:
        items = list(getattr(list_op, "explicitItems", [])) + list(getattr(list_op, "addedItems", []))
    return [str(getattr(item, "assetPath", "")) for item in items if getattr(item, "assetPath", "")]


def inspect_usd(path: Path, sample_limit: int = 40) -> dict[str, Any]:
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"Could not open USD: {path}")

    summary: dict[str, Any] = {
        "path": path.as_posix(),
        "default_prim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "prim_count": 0,
        "type_counts": {},
        "mesh_count": 0,
        "cube_count": 0,
        "sphere_count": 0,
        "rigid_body_count": 0,
        "joint_count": 0,
        "revolute_joint_count": 0,
        "reference_count": 0,
        "payload_count": 0,
        "asset_paths": [],
        "sample_prims": [],
    }

    asset_paths = set()
    type_counts: dict[str, int] = {}
    for prim in stage.Traverse():
        prim_type = str(prim.GetTypeName())
        summary["prim_count"] += 1
        type_counts[prim_type] = type_counts.get(prim_type, 0) + 1
        if prim_type == "Mesh":
            summary["mesh_count"] += 1
        if prim_type == "Cube":
            summary["cube_count"] += 1
        if prim_type == "Sphere":
            summary["sphere_count"] += 1
        if "Joint" in prim_type:
            summary["joint_count"] += 1
        if prim_type == "PhysicsRevoluteJoint":
            summary["revolute_joint_count"] += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            summary["rigid_body_count"] += 1
        if prim.HasAuthoredReferences():
            summary["reference_count"] += 1
            asset_paths.update(asset_paths_from_list_op(prim.GetMetadata("references")))
        if prim.HasAuthoredPayloads():
            summary["payload_count"] += 1
            asset_paths.update(asset_paths_from_list_op(prim.GetMetadata("payload")))
        if len(summary["sample_prims"]) < sample_limit and (
            prim_type in {"Mesh", "Cube", "Sphere", "Xform", "PhysicsRevoluteJoint"}
            or prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ):
            summary["sample_prims"].append({"path": str(prim.GetPath()), "type": prim_type})

    summary["type_counts"] = dict(sorted(type_counts.items()))
    summary["asset_paths"] = sorted(asset_paths)
    summary["is_mesh_cad_visual"] = bool(summary["mesh_count"] >= 1 and summary["cube_count"] == 0)
    summary["is_physics_proxy_visual"] = bool(summary["mesh_count"] == 0 and (summary["cube_count"] or summary["sphere_count"]))
    return summary


def relative_asset_path(asset: Path, output: Path) -> str:
    try:
        return os.path.relpath(asset, output.parent).replace("\\", "/")
    except ValueError:
        return asset.as_posix()


def create_visual_wrapper(cad_usd: Path, output_usd: Path, root_prim_path: str, scale: float) -> None:
    output_usd.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    cad_root = UsdGeom.Xform.Define(stage, root_prim_path)
    cad_root.GetPrim().GetReferences().AddReference(relative_asset_path(cad_usd, output_usd))
    UsdGeom.XformCommonAPI(cad_root).SetScale((float(scale), float(scale), float(scale)))
    stage.GetRootLayer().Save()


def repo_relative(path: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def scrub_report_paths(summary: dict[str, Any]) -> dict[str, Any]:
    result = dict(summary)
    if "path" in result:
        result["path"] = repo_relative(Path(str(result["path"])))
    wrapper = result.get("wrapper")
    if isinstance(wrapper, dict):
        scrubbed_wrapper = dict(wrapper)
        if "path" in scrubbed_wrapper:
            scrubbed_wrapper["path"] = repo_relative(Path(str(scrubbed_wrapper["path"])))
        inspection = scrubbed_wrapper.get("inspection")
        if isinstance(inspection, dict):
            scrubbed_wrapper["inspection"] = scrub_report_paths(inspection)
        result["wrapper"] = scrubbed_wrapper
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Audit the real Domino CAD USD and optionally create a visual wrapper.")
    parser.add_argument("--cad-usd", default="simulation/usd/Domino_Quadruped.usd", help="Real Domino CAD USD visual export.")
    parser.add_argument("--write-wrapper", action="store_true", help="Write a meter-based visual wrapper USD.")
    parser.add_argument("--output-usd", default="simulation/isaac/out/domino_actual_cad_visual/domino_actual_cad_visual.usda")
    parser.add_argument("--root-prim-path", default="/World/DominoActualCadVisual")
    parser.add_argument("--scale", type=float, default=0.001, help="Scale applied by the wrapper stage.")
    parser.add_argument("--json-report", default="", help="Optional JSON report path.")
    parser.add_argument("--min-mesh-count", type=int, default=20, help="Minimum mesh count expected for the real CAD visual.")
    args = parser.parse_args()

    cad_usd = Path(args.cad_usd).expanduser().resolve()
    if not cad_usd.exists():
        raise FileNotFoundError(f"CAD USD does not exist: {cad_usd}")

    summary = inspect_usd(cad_usd)
    if int(summary["mesh_count"]) < int(args.min_mesh_count):
        raise RuntimeError(f"Expected at least {args.min_mesh_count} mesh prims, found {summary['mesh_count']}.")
    if int(summary["cube_count"]) or int(summary["sphere_count"]):
        raise RuntimeError("The selected CAD USD contains Cube/Sphere proxy visuals; use the real mesh CAD export instead.")

    if args.write_wrapper:
        output_usd = Path(args.output_usd).expanduser().resolve()
        create_visual_wrapper(cad_usd, output_usd, args.root_prim_path, float(args.scale))
        wrapper_summary = inspect_usd(output_usd)
        summary["wrapper"] = {
            "path": output_usd.as_posix(),
            "root_prim_path": args.root_prim_path,
            "scale": float(args.scale),
            "inspection": wrapper_summary,
        }

    report = scrub_report_paths(summary)

    if args.json_report:
        report_path = Path(args.json_report).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
