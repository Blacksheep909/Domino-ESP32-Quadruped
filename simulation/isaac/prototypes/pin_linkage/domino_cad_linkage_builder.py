"""Import-safe USD builder for the Domino four-leg CAD linkage proxy.

This module deliberately does not create an AppLauncher or SimulationContext.
It can be imported by Isaac Lab environments after the caller has launched
Isaac.

The generated bodies in this module are simplified proxy rigid bodies whose
joint pivots come from the Domino CAD/URDF export. By default the real exported
Domino STL link meshes are attached as the visible geometry, while the proxy
cubes and foot spheres remain hidden collision/debug scaffolding.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import struct
import sys

import numpy as np

CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))

from domino_action_contract import (  # noqa: E402
    ACTION_JOINT_NAMES,
    EXPECTED_ACTION_COUNT,
    SERVO_MECHANICAL_TRAVEL_FROM_NEUTRAL_DEG,
    VALIDATED_INITIAL_POLICY_ACTION_SCALE_DEG,
    action_group_counts,
    centered_servo_limits_deg,
    per_leg_action_layout,
    validate_action_layout,
)
from domino_cad_neutral_pose import (  # noqa: E402
    CALIBRATED_NEUTRAL_BODY_POSES,
    CAPTURE_RESOLVED_FLOATING_HEIGHT_M,
    FRONT_FROM_REAR_TRANSLATION_M,
    FRONT_REAR_BODY_PAIRS,
)
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

try:
    from pxr import PhysxSchema
except ImportError:
    PhysxSchema = None


PROXY_VISUAL_FIDELITY = "proxy_cubes_and_foot_spheres"
ACTUAL_CAD_STL_VISUAL_FIDELITY = "actual_cad_stl_visuals_on_proxy_physics"
ACTUAL_CAD_VISUAL_USD = "simulation/usd/Domino_Quadruped.usd"
ACTUAL_CAD_STL_SOURCE = "simulation/urdf/generated/Domino_URDF_Parts_Combined_Final_description/meshes"
LINKAGE_PIVOT_SOURCE = "simulation/isaac/reports/domino-linkage-pivots.json"
PHYSX_RIGID_BODY_SOLVER_POSITION_ITERATIONS = 255
PHYSX_RIGID_BODY_SOLVER_VELOCITY_ITERATIONS = 4
PHYSX_RIGID_BODY_MAX_DEPENETRATION_VELOCITY_M_S = 1.0
PHYSX_RIGID_BODY_LINEAR_DAMPING = 0.06
PHYSX_RIGID_BODY_ANGULAR_DAMPING = 0.10
PHYSX_PIN_JOINT_ARMATURE = 0.004
PHYSX_PIN_JOINT_MAX_VELOCITY_DEG_S = 720.0
PHYSX_PIN_JOINT_PROJECTION_LINEAR_TOLERANCE_M = 0.0001
PHYSX_PIN_JOINT_PROJECTION_ANGULAR_TOLERANCE_DEG = 0.5
KG_CM_TO_N_M = 9.80665 * 0.01
SHOULDER_SERVO_STALL_TORQUE_KG_CM = 40.0
LINKAGE_SERVO_STALL_TORQUE_KG_CM = 35.0
SHOULDER_SERVO_STALL_TORQUE_N_M = SHOULDER_SERVO_STALL_TORQUE_KG_CM * KG_CM_TO_N_M
LINKAGE_SERVO_STALL_TORQUE_N_M = LINKAGE_SERVO_STALL_TORQUE_KG_CM * KG_CM_TO_N_M
SHOULDER_SERVO_DRIVE_STIFFNESS = 45.0
SHOULDER_SERVO_DRIVE_DAMPING = 2.0
LINKAGE_SERVO_DRIVE_STIFFNESS = 40.0
LINKAGE_SERVO_DRIVE_DAMPING = 1.8
SERVO_TARGET_RATE_LIMIT_DEG_S = 360.0
DOMINO_CONTACT_MATERIAL_PATH = "/World/PhysicsMaterials/DominoFootGround"
DOMINO_CONTACT_STATIC_FRICTION = 1.25
DOMINO_CONTACT_DYNAMIC_FRICTION = 1.05
DOMINO_CONTACT_RESTITUTION = 0.0
DOMINO_TPU_FOOT_MATERIAL_PATH = "/World/PhysicsMaterials/DominoTpuFoot"
DOMINO_TPU_FOOT_STATIC_FRICTION = 1.40
DOMINO_TPU_FOOT_DYNAMIC_FRICTION = 1.20
DOMINO_TPU_FOOT_RESTITUTION = 0.02
DOMINO_TPU_FOOT_FRICTION_COMBINE_MODE = "max"
DOMINO_TPU_FOOT_RESTITUTION_COMBINE_MODE = "min"
DOMINO_VISUAL_MATERIAL_ROOT = "/World/Looks/Domino"
DOMINO_VISUAL_MATERIALS = {
    "frame_graphite": {
        "path": f"{DOMINO_VISUAL_MATERIAL_ROOT}/FrameGraphite",
        "diffuse_color": (0.10, 0.12, 0.14),
        "roughness": 0.50,
        "metallic": 0.0,
    },
    "passive_carbon": {
        "path": f"{DOMINO_VISUAL_MATERIAL_ROOT}/LegBlack",
        "diffuse_color": (0.025, 0.032, 0.040),
        "roughness": 0.44,
        "metallic": 0.02,
    },
    "tpu": {
        "path": f"{DOMINO_VISUAL_MATERIAL_ROOT}/TpuBlack",
        "diffuse_color": (0.012, 0.014, 0.016),
        "roughness": 0.82,
        "metallic": 0.0,
    },
}
FOOT_CLOSURE_STABILIZER_STIFFNESS = 2.4
FOOT_CLOSURE_STABILIZER_DAMPING = 0.85
FOOT_CLOSURE_STABILIZER_MAX_FORCE_N_M = 1.8
FOOT_CLOSURE_STABILIZER_LIMIT_DEG = 8.0
SERVO_ACTUATOR_MODEL = {
    "source": "user_provided_domino_servo_specs",
    "torque_units": "N*m derived from kg*cm stall torque using 1 kgf*cm = 0.0980665 N*m",
    "speed_note": "Conservative target slew limiter used because the provided servo photos/specs do not include transit speed.",
    "mechanical_travel_from_neutral_deg": SERVO_MECHANICAL_TRAVEL_FROM_NEUTRAL_DEG,
    "roles": {
        "shoulder_ab_ad": {
            "servo": "DSSERVO 40KG digital servo",
            "mount": "shoulder_hip_ab_ad",
            "quantity": 4,
            "spline": "25T",
            "supply_voltage_v": "6.0-8.5",
            "stall_torque_kg_cm": SHOULDER_SERVO_STALL_TORQUE_KG_CM,
            "stall_torque_n_m": round(SHOULDER_SERVO_STALL_TORQUE_N_M, 6),
            "drive_stiffness": SHOULDER_SERVO_DRIVE_STIFFNESS,
            "drive_damping": SHOULDER_SERVO_DRIVE_DAMPING,
            "target_rate_limit_deg_s": SERVO_TARGET_RATE_LIMIT_DEG_S,
        },
        "lower_linkage_drive": {
            "servo": "DSservo 35KG digital servo",
            "mount": "lower_two_bar_linkage_drive",
            "quantity": 4,
            "stall_torque_kg_cm": LINKAGE_SERVO_STALL_TORQUE_KG_CM,
            "stall_torque_n_m": round(LINKAGE_SERVO_STALL_TORQUE_N_M, 6),
            "drive_stiffness": LINKAGE_SERVO_DRIVE_STIFFNESS,
            "drive_damping": LINKAGE_SERVO_DRIVE_DAMPING,
            "target_rate_limit_deg_s": SERVO_TARGET_RATE_LIMIT_DEG_S,
        },
        "upper_pitch_drive": {
            "servo": "DSservo 35KG digital servo",
            "mount": "upper_two_bar_linkage_drive",
            "quantity": 4,
            "stall_torque_kg_cm": LINKAGE_SERVO_STALL_TORQUE_KG_CM,
            "stall_torque_n_m": round(LINKAGE_SERVO_STALL_TORQUE_N_M, 6),
            "drive_stiffness": LINKAGE_SERVO_DRIVE_STIFFNESS,
            "drive_damping": LINKAGE_SERVO_DRIVE_DAMPING,
            "target_rate_limit_deg_s": SERVO_TARGET_RATE_LIMIT_DEG_S,
        },
    },
}


ORIGINAL_ACTUAL_CAD_VISUAL_LINKS_BY_BODY = {
    "body_reference": ("base_link",),
    "dom_p_4_1_ground": ("DOM_P__4__1",),
    "dom_p_4_1_lower_driver": ("DOM_P__5__1",),
    "dom_p_4_1_coupler": ("DOM_P_1",),
    "dom_p_4_1_lower_diagonal": ("DOM_P__2__1",),
    "dom_p_4_1_lower_closure": ("DOM_P__1__1",),
    "dom_p_4_1_upper_driver": ("DOM_P__6__1",),
    "dom_p_4_1_upper_closure": ("DOM_P__3__1",),
    "dom_p_12_1_ground": ("DOM_P__12__1",),
    "dom_p_12_1_lower_driver": ("DOM_P__13__1",),
    "dom_p_12_1_coupler": ("DOM_P__7__1",),
    "dom_p_12_1_lower_diagonal": ("DOM_P__10__1",),
    "dom_p_12_1_lower_closure": ("DOM_P__11__1",),
    "dom_p_12_1_upper_driver": ("DOM_P__8__1",),
    "dom_p_12_1_upper_closure": ("DOM_P__9__1",),
    "dom_p_25_1_ground": ("DOM_P__25__1",),
    "dom_p_25_1_lower_driver": ("DOM_P__16__1",),
    "dom_p_25_1_coupler": ("DOM_P__15__1",),
    "dom_p_25_1_lower_diagonal": ("DOM_P__14__1",),
    "dom_p_25_1_lower_closure": ("DOM_P__26__1",),
    "dom_p_25_1_upper_driver": ("DOM_P__17__1",),
    "dom_p_25_1_upper_closure": ("DOM_P__27__1",),
    "dom_p_21_1_ground": ("DOM_P__21__1",),
    "dom_p_21_1_lower_driver": ("DOM_P__18__1",),
    "dom_p_21_1_coupler": ("DOM_P__24__1",),
    "dom_p_21_1_lower_diagonal": ("DOM_P__23__1",),
    "dom_p_21_1_lower_closure": ("DOM_P__22__1",),
    "dom_p_21_1_upper_driver": ("DOM_P__19__1",),
    "dom_p_21_1_upper_closure": ("DOM_P__20__1",),
}

# The front CAD export is assembled around a different linkage pose and does
# not remain registered when its rigid bodies use the rear-derived mechanism.
# Domino's front and rear pivot layouts are exact 335 mm translations, so the
# simulation duplicates each real rear CAD body at the corresponding front
# body.  The original front STL files remain in the source export.
FRONT_RUNTIME_VISUAL_FROM_REAR_BODY = {
    "dom_p_4_1_ground": "dom_p_21_1_ground",
    "dom_p_4_1_lower_driver": "dom_p_21_1_lower_driver",
    "dom_p_4_1_coupler": "dom_p_21_1_coupler",
    "dom_p_4_1_lower_diagonal": "dom_p_21_1_lower_diagonal",
    "dom_p_4_1_lower_closure": "dom_p_21_1_lower_closure",
    "dom_p_4_1_upper_driver": "dom_p_21_1_upper_driver",
    "dom_p_4_1_upper_closure": "dom_p_21_1_upper_closure",
    "dom_p_12_1_ground": "dom_p_25_1_ground",
    "dom_p_12_1_lower_driver": "dom_p_25_1_lower_driver",
    "dom_p_12_1_coupler": "dom_p_25_1_coupler",
    "dom_p_12_1_lower_diagonal": "dom_p_25_1_lower_diagonal",
    "dom_p_12_1_lower_closure": "dom_p_25_1_lower_closure",
    "dom_p_12_1_upper_driver": "dom_p_25_1_upper_driver",
    "dom_p_12_1_upper_closure": "dom_p_25_1_upper_closure",
}
ACTUAL_CAD_VISUAL_LINKS_BY_BODY = dict(ORIGINAL_ACTUAL_CAD_VISUAL_LINKS_BY_BODY)
for _front_body_key, _rear_body_key in FRONT_RUNTIME_VISUAL_FROM_REAR_BODY.items():
    ACTUAL_CAD_VISUAL_LINKS_BY_BODY[_front_body_key] = ORIGINAL_ACTUAL_CAD_VISUAL_LINKS_BY_BODY[_rear_body_key]

ACTUAL_CAD_VISUAL_SOURCE_TRANSLATIONS_M = {
    body_key: FRONT_FROM_REAR_TRANSLATION_M
    for body_key in FRONT_RUNTIME_VISUAL_FROM_REAR_BODY
}
EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT = sum(len(link_names) for link_names in ACTUAL_CAD_VISUAL_LINKS_BY_BODY.values())
EXPECTED_ACTUAL_CAD_TOTAL_TRIANGLE_COUNT = 137236

# The Fusion URDF repeats each loop-closing link declaration, once from each
# parent branch.  Direct-closure compatibility mode has no closure body, so it
# still needs a visual fallback.  The default finite-link closure model owns
# these meshes on their real two-pivot rigid bodies and does not use aliases.
ACTUAL_CAD_VISUAL_BODY_ALIASES = {
    "dom_p_4_1_lower_closure": "dom_p_4_1_lower_diagonal",
    "dom_p_4_1_upper_closure": "dom_p_4_1_upper_driver",
    "dom_p_12_1_lower_closure": "dom_p_12_1_lower_diagonal",
    "dom_p_12_1_upper_closure": "dom_p_12_1_upper_driver",
    "dom_p_25_1_lower_closure": "dom_p_25_1_lower_diagonal",
    "dom_p_25_1_upper_closure": "dom_p_25_1_upper_driver",
    "dom_p_21_1_lower_closure": "dom_p_21_1_lower_diagonal",
    "dom_p_21_1_upper_closure": "dom_p_21_1_upper_driver",
}


def actual_cad_visual_material_key(body_key: str) -> str:
    """Keep the chassis graphite and render the complete leg assembly black."""
    if body_key == "body_reference":
        return "frame_graphite"
    if body_key.endswith("_lower_closure"):
        return "tpu"
    return "passive_carbon"


def get_or_create_domino_visual_material(stage, material_key: str):
    spec = DOMINO_VISUAL_MATERIALS[material_key]
    material_path = str(spec["path"])
    material_prim = stage.GetPrimAtPath(material_path)
    if material_prim.IsValid():
        return UsdShade.Material(material_prim)

    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    diffuse = spec["diffuse_color"]
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(float(diffuse[0]), float(diffuse[1]), float(diffuse[2]))
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(spec["roughness"]))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(spec["metallic"]))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


@dataclass(frozen=True)
class DominoCadLinkageBuildConfig:
    root_prim_path: str = "/World/DominoFour12FloatingBody"
    world_translation_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    floating_height_m: float = 0.12
    foot_proxy_radius_m: float = 0.024
    enable_gravity: bool = True
    include_ground: bool = True
    ground_prim_path: str = "/World/Ground"
    ground_size_m: float = 10.0
    ground_thickness_m: float = 0.05
    include_actual_cad_visuals: bool = True
    hide_proxy_visuals_when_actual_cad: bool = True
    fixed_base: bool = False
    closure_model: str = "passive"
    actual_cad_mesh_dir: str = ""
    use_actual_cad_foot_collision: bool = False
    foot_contact_mode: str = ""
    use_calibrated_neutral_pose: bool = True
    align_actual_cad_visual_bottom_to_ground: bool = True
    actual_cad_ground_clearance_m: float = 0.002
    enable_body_collisions: bool = True


def default_actual_cad_mesh_dir() -> Path:
    return Path(__file__).resolve().parents[3] / "urdf" / "generated" / "Domino_URDF_Parts_Combined_Final_description" / "meshes"


def safe_prim_name(value: str) -> str:
    return str(value).replace("__", "_u_").replace(" ", "_").replace("-", "_")


@lru_cache(maxsize=None)
def load_binary_stl_mesh_m(path_string: str) -> tuple[tuple[tuple[float, float, float], ...], tuple[int, ...], tuple[int, ...]]:
    path = Path(path_string)
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL is too small to be a binary STL: {path}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + (50 * int(triangle_count))
    if len(data) < expected_size:
        raise ValueError(f"Binary STL is truncated: {path}")

    points = []
    indices = []
    counts = []
    offset = 84
    for _ in range(int(triangle_count)):
        offset += 12
        counts.append(3)
        for _ in range(3):
            x, y, z = struct.unpack_from("<3f", data, offset)
            points.append((float(x) * 0.001, float(y) * 0.001, float(z) * 0.001))
            indices.append(len(indices))
            offset += 12
        offset += 2
    return tuple(points), tuple(counts), tuple(indices)


def create_stl_visual_mesh(
    stage,
    body: dict,
    link_name: str,
    mesh_dir: Path,
    world_offset: np.ndarray,
    source_translation_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material_key: str = "frame_graphite",
) -> dict:
    mesh_path = mesh_dir / f"{link_name}.stl"
    if not mesh_path.exists():
        raise FileNotFoundError(f"Missing Domino CAD mesh: {mesh_path}")

    points_m, counts, indices = load_binary_stl_mesh_m(str(mesh_path))
    center = np.asarray(body["center"], dtype=np.float64)
    offset = np.asarray(world_offset, dtype=np.float64)
    source_translation = np.asarray(source_translation_m, dtype=np.float64)
    corrector_path = f"{body['path']}/actual_cad"
    UsdGeom.Xform.Define(stage, corrector_path)
    local_points = [
        Gf.Vec3f(
            float(point[0] + source_translation[0] + offset[0] - center[0]),
            float(point[1] + source_translation[1] + offset[1] - center[1]),
            float(point[2] + source_translation[2] + offset[2] - center[2]),
        )
        for point in points_m
    ]

    mesh = UsdGeom.Mesh.Define(stage, f"{corrector_path}/{safe_prim_name(link_name)}")
    mesh.CreatePointsAttr(local_points)
    mesh.CreateFaceVertexCountsAttr(list(counts))
    mesh.CreateFaceVertexIndicesAttr(list(indices))
    mesh.CreateDoubleSidedAttr(True)
    material = get_or_create_domino_visual_material(stage, material_key)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="preview",
    )
    diffuse = DOMINO_VISUAL_MATERIALS[material_key]["diffuse_color"]
    mesh.CreateDisplayColorAttr(
        [Gf.Vec3f(float(diffuse[0]), float(diffuse[1]), float(diffuse[2]))]
    )
    return {
        "link_name": link_name,
        "path": str(mesh.GetPath()),
        "triangle_count": len(counts),
        "source_translation_m": source_translation.tolist(),
        "visual_material": material_key,
    }


def attach_actual_cad_visuals(stage, bodies: dict, world_offset: np.ndarray, mesh_dir: Path) -> dict:
    attached = []
    corrector_paths = {}
    missing_bodies = []
    for body_key, link_names in ACTUAL_CAD_VISUAL_LINKS_BY_BODY.items():
        resolved_body_key = body_key if body_key in bodies else ACTUAL_CAD_VISUAL_BODY_ALIASES.get(body_key, body_key)
        body = bodies.get(resolved_body_key)
        if body is None:
            missing_bodies.append(body_key)
            continue
        corrector_paths[resolved_body_key] = f"{body['path']}/actual_cad"
        source_translation = ACTUAL_CAD_VISUAL_SOURCE_TRANSLATIONS_M.get(body_key, (0.0, 0.0, 0.0))
        for link_name in link_names:
            attached.append(
                create_stl_visual_mesh(
                    stage,
                    body,
                    link_name,
                    mesh_dir,
                    world_offset,
                    source_translation_m=source_translation,
                    material_key=actual_cad_visual_material_key(body_key),
                )
            )
    if missing_bodies:
        raise RuntimeError(f"Could not attach CAD visuals; missing proxy bodies: {missing_bodies}")
    cad_collision_fits = fit_ground_collisions_to_actual_cad(
        stage,
        bodies,
        world_offset,
        mesh_dir,
    )
    return {
        "source": ACTUAL_CAD_STL_SOURCE,
        "mesh_count": len(attached),
        "expected_mesh_count": EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT,
        "triangle_count": int(sum(item["triangle_count"] for item in attached)),
        "expected_triangle_count": EXPECTED_ACTUAL_CAD_TOTAL_TRIANGLE_COUNT,
        "corrector_paths": corrector_paths,
        "attached_links": attached,
        "visual_materials": {
            key: {
                "path": str(spec["path"]),
                "diffuse_color": list(spec["diffuse_color"]),
                "roughness": float(spec["roughness"]),
                "metallic": float(spec["metallic"]),
            }
            for key, spec in DOMINO_VISUAL_MATERIALS.items()
        },
        "front_runtime_visual_from_rear_body": dict(FRONT_RUNTIME_VISUAL_FROM_REAR_BODY),
        "front_from_rear_translation_m": list(FRONT_FROM_REAR_TRANSLATION_M),
        "cad_collision_fits": cad_collision_fits,
    }


def fit_ground_collisions_to_actual_cad(
    stage,
    bodies: dict,
    world_offset: np.ndarray,
    mesh_dir: Path,
) -> dict:
    """Fit hidden ground colliders to real CAD bounds for non-foot bodies."""
    fitted = {}
    offset = np.asarray(world_offset, dtype=np.float64)
    for body_key in ACTUAL_CAD_VISUAL_LINKS_BY_BODY:
        if body_key.endswith("_lower_closure"):
            # The real foot ball already has its own measured sphere collider.
            continue
        resolved_body_key = (
            body_key
            if body_key in bodies
            else ACTUAL_CAD_VISUAL_BODY_ALIASES.get(body_key, body_key)
        )
        body = bodies.get(resolved_body_key)
        if body is None or not body.get("ground_collision_path"):
            continue
        points_world = actual_cad_body_point_cloud_m(body_key, mesh_dir) + offset.reshape(1, 3)
        local_points = points_world - np.asarray(body["center"], dtype=np.float64).reshape(1, 3)
        local_min = local_points.min(axis=0)
        local_max = local_points.max(axis=0)
        local_center = 0.5 * (local_min + local_max)
        extents = np.maximum(local_max - local_min, np.array([0.002, 0.002, 0.002]))
        collision_prim = stage.GetPrimAtPath(str(body["ground_collision_path"]))
        if not collision_prim.IsValid():
            raise RuntimeError(
                f"Missing ground collider while fitting actual CAD body {body_key}: "
                f"{body['ground_collision_path']}"
            )
        xform = UsdGeom.XformCommonAPI(collision_prim)
        xform.SetTranslate(
            Gf.Vec3d(
                float(local_center[0]),
                float(local_center[1]),
                float(local_center[2]),
            )
        )
        xform.SetScale(Gf.Vec3f(float(extents[0]), float(extents[1]), float(extents[2])))
        body["collision_local_center_m"] = local_center.tolist()
        body["collision_half_extents_m"] = (0.5 * extents).tolist()
        body["ground_collision_fit_source"] = "actual_cad_local_aabb"
        fitted[resolved_body_key] = {
            "source_body": body_key,
            "local_center_m": local_center.tolist(),
            "half_extents_m": (0.5 * extents).tolist(),
            "collision_path": str(body["ground_collision_path"]),
        }
    return {
        "fit_source": "actual_cad_local_aabb",
        "fitted_body_count": len(fitted),
        "excluded_actual_foot_bodies": [
            body_key
            for body_key in ACTUAL_CAD_VISUAL_LINKS_BY_BODY
            if body_key.endswith("_lower_closure")
        ],
        "bodies": fitted,
    }


def set_actual_cad_visual_local_lift(stage, corrector_paths: dict, lift_m: float) -> None:
    for corrector_path in corrector_paths.values():
        prim = stage.GetPrimAtPath(str(corrector_path))
        if prim.IsValid():
            UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(0.0, 0.0, float(lift_m)))


def actual_cad_body_point_cloud_m(body_key: str, mesh_dir: Path) -> np.ndarray:
    link_names = ACTUAL_CAD_VISUAL_LINKS_BY_BODY.get(body_key)
    if not link_names:
        raise KeyError(f"No actual-CAD visual links are mapped to body: {body_key}")
    points = np.vstack(
        [
            np.asarray(load_binary_stl_mesh_m(str(mesh_dir / f"{link_name}.stl"))[0])
            for link_name in link_names
        ]
    )
    source_translation = np.asarray(
        ACTUAL_CAD_VISUAL_SOURCE_TRANSLATIONS_M.get(body_key, (0.0, 0.0, 0.0)),
        dtype=np.float64,
    )
    return points + source_translation.reshape(1, 3)


def fit_distal_foot_sphere(
    points_m: np.ndarray,
    fit_height_m: float = 0.015,
    shell_scale: float = 1.05,
    max_shell_residual_m: float = 0.0001,
) -> dict:
    """Measure the spherical foot directly from the distal CAD vertices."""
    visual_min = points_m.min(axis=0)
    distal_points = points_m[points_m[:, 2] <= float(visual_min[2]) + float(fit_height_m)]
    if len(distal_points) < 16:
        raise RuntimeError(f"Not enough distal CAD vertices to fit a foot sphere: {len(distal_points)}")

    x_radius = 0.5 * float(np.ptp(distal_points[:, 0]))
    y_radius = 0.5 * float(np.ptp(distal_points[:, 1]))
    radius_m = 0.5 * (x_radius + y_radius)
    if radius_m <= 0.0 or abs(x_radius - y_radius) > 0.0005:
        raise RuntimeError(
            "Distal CAD geometry is not the expected spherical Domino foot: "
            f"x_radius={x_radius:.6f}m, y_radius={y_radius:.6f}m"
        )

    center_m = np.array(
        [
            0.5 * float(distal_points[:, 0].min() + distal_points[:, 0].max()),
            0.5 * float(distal_points[:, 1].min() + distal_points[:, 1].max()),
            float(visual_min[2]) + radius_m,
        ],
        dtype=np.float64,
    )
    distances = np.linalg.norm(points_m - center_m.reshape(1, 3), axis=1)
    shell_points = points_m[distances <= radius_m * float(shell_scale)]
    shell_residuals = np.abs(np.linalg.norm(shell_points - center_m.reshape(1, 3), axis=1) - radius_m)
    residual_p95_m = float(np.percentile(shell_residuals, 95.0))
    if len(shell_points) < 64 or residual_p95_m > float(max_shell_residual_m):
        raise RuntimeError(
            "Measured Domino foot sphere does not match the CAD surface: "
            f"shell_points={len(shell_points)}, p95_residual={residual_p95_m:.6f}m"
        )

    return {
        "center_m": center_m,
        "radius_m": radius_m,
        "visual_bottom_m": center_m - np.array([0.0, 0.0, radius_m], dtype=np.float64),
        "fit_height_m": float(fit_height_m),
        "distal_sample_count": int(len(distal_points)),
        "shell_sample_count": int(len(shell_points)),
        "shell_residual_p95_m": residual_p95_m,
        "shell_residual_max_m": float(np.max(shell_residuals)),
        "axis_radii_m": {"x": x_radius, "y": y_radius},
    }


def actual_cad_foot_collision_points(mesh_dir: Path, foot_proxy_radius_m: float, z_band_m: float = 0.002) -> dict:
    feet = {}
    min_visual_z = float("inf")
    for spec in DOMINO_FOUR_COMBINED_LEG_SPECS:
        body_key = f"{spec['id']}_lower_closure"
        points_m = actual_cad_body_point_cloud_m(body_key, mesh_dir)
        visual_min = points_m.min(axis=0)
        visual_max = points_m.max(axis=0)
        min_z = float(visual_min[2])
        foot_sphere = fit_distal_foot_sphere(points_m)
        bottom_points = points_m[points_m[:, 2] <= min_z + float(z_band_m)]
        if len(bottom_points) == 0:
            bottom_points = points_m[points_m[:, 2] == min_z]
        bottom_mean_xy = np.mean(bottom_points[:, :2], axis=0)
        visual_bottom = np.asarray(foot_sphere["visual_bottom_m"], dtype=np.float64)
        sphere_center = np.asarray(foot_sphere["center_m"], dtype=np.float64)
        feet[body_key] = {
            "visual_bottom_without_offset_m": visual_bottom,
            "visual_bottom_xy_source": "cad_distal_sphere_center",
            "lowest_vertex_mean_without_offset_m": np.array(
                [float(bottom_mean_xy[0]), float(bottom_mean_xy[1]), min_z],
                dtype=np.float64,
            ),
            "sphere_center_without_offset_m": sphere_center,
            "radius_m": float(foot_sphere["radius_m"]),
            "fit_source": "measured_from_distal_cad_sphere",
            "fit_height_m": float(foot_sphere["fit_height_m"]),
            "fit_distal_sample_count": int(foot_sphere["distal_sample_count"]),
            "fit_shell_sample_count": int(foot_sphere["shell_sample_count"]),
            "fit_shell_residual_p95_m": float(foot_sphere["shell_residual_p95_m"]),
            "fit_shell_residual_max_m": float(foot_sphere["shell_residual_max_m"]),
            "fit_axis_radii_m": dict(foot_sphere["axis_radii_m"]),
            "configured_fallback_radius_m": float(foot_proxy_radius_m),
            "visual_bounds_without_offset_m": {"min_m": visual_min, "max_m": visual_max},
            "bottom_sample_count": int(len(bottom_points)),
            "bottom_z_band_m": float(z_band_m),
        }
        min_visual_z = min(min_visual_z, min_z)
    return {"feet": feet, "min_visual_bottom_z_without_offset_m": min_visual_z}


def actual_cad_grounded_support_feet(
    actual_cad_foot_geometry: dict,
    foot_proxy_radius_m: float,
    ground_clearance_m: float,
    support_center_z_without_offset_m: float | None = None,
    rendered_visual_lift_m: float = 0.0,
    align_to_rendered_visual_bottom: bool = False,
) -> dict:
    """Build CAD-derived support contacts for the hidden physics feet."""
    min_visual_z = float(actual_cad_foot_geometry["min_visual_bottom_z_without_offset_m"])
    support_center_z = (
        float(support_center_z_without_offset_m)
        if support_center_z_without_offset_m is not None
        else min_visual_z + float(foot_proxy_radius_m)
    )
    feet = {}
    for spec in DOMINO_FOUR_COMBINED_LEG_SPECS:
        body_key = f"{spec['id']}_lower_closure"
        visual_foot = actual_cad_foot_geometry["feet"][body_key]
        visual_bottom = np.asarray(visual_foot["visual_bottom_without_offset_m"], dtype=np.float64)
        support_radius_m = float(visual_foot.get("radius_m", foot_proxy_radius_m))
        if align_to_rendered_visual_bottom:
            support_center = visual_bottom + np.array(
                [0.0, 0.0, float(rendered_visual_lift_m) + support_radius_m],
                dtype=np.float64,
            )
        else:
            support_center = np.array(
                [float(visual_bottom[0]), float(visual_bottom[1]), support_center_z],
                dtype=np.float64,
            )
        feet[body_key] = {
            "sphere_center_without_offset_m": support_center,
            "visual_bottom_without_offset_m": visual_bottom,
            "radius_m": support_radius_m,
            "bottom_sample_count": int(visual_foot["bottom_sample_count"]),
        }
    result = {
        "source": (
            "actual_cad_rendered_visual_bottom_proxy_spheres"
            if align_to_rendered_visual_bottom
            else "actual_cad_visual_xy_grounded_proxy_spheres"
        ),
        "feet": feet,
        "min_visual_bottom_z_without_offset_m": min_visual_z,
        "ground_clearance_m": float(ground_clearance_m),
    }
    if not align_to_rendered_visual_bottom:
        result["support_center_z_without_offset_m"] = float(support_center_z)
    else:
        result["rendered_visual_lift_m"] = float(rendered_visual_lift_m)
    return result


def quat_wxyz_rotation_matrix(orientation: tuple[float, float, float, float]) -> np.ndarray:
    w, x, y, z = (float(value) for value in orientation)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def calibrated_neutral_foot_alignment(actual_cad_foot_geometry: dict) -> dict:
    """Resolve CAD foot bottoms in the captured neutral rigid-body pose."""
    feet = {}
    min_bottom_z_m = float("inf")
    for spec in DOMINO_FOUR_COMBINED_LEG_SPECS:
        body_key = f"{spec['id']}_lower_closure"
        captured_position, captured_orientation = CALIBRATED_NEUTRAL_BODY_POSES[body_key]
        authored_body_center = 0.5 * (
            np.asarray(spec["points"]["lower_closure_driver"], dtype=np.float64)
            + np.asarray(spec["points"]["lower_closure_diagonal"], dtype=np.float64)
        )
        foot = actual_cad_foot_geometry["feet"][body_key]
        authored_foot_center = np.asarray(foot["sphere_center_without_offset_m"], dtype=np.float64)
        local_foot_center = authored_foot_center - authored_body_center
        world_foot_center = (
            np.asarray(captured_position, dtype=np.float64)
            + quat_wxyz_rotation_matrix(captured_orientation) @ local_foot_center
        )
        radius_m = float(foot["radius_m"])
        bottom_z_m = float(world_foot_center[2] - radius_m)
        feet[body_key] = {
            "captured_center_m": world_foot_center,
            "captured_bottom_z_m": bottom_z_m,
            "local_center_m": local_foot_center,
            "radius_m": radius_m,
        }
        min_bottom_z_m = min(min_bottom_z_m, bottom_z_m)
    return {
        "capture_resolved_floating_height_m": float(CAPTURE_RESOLVED_FLOATING_HEIGHT_M),
        "min_captured_bottom_z_m": min_bottom_z_m,
        "feet": feet,
    }


def _prim_is_under_root(prim, root_prim_path: str) -> bool:
    path = str(prim.GetPath())
    root = str(root_prim_path).rstrip("/")
    return path == root or path.startswith(f"{root}/")


def _is_visible(prim) -> bool:
    imageable = UsdGeom.Imageable(prim)
    return imageable.ComputeVisibility() != UsdGeom.Tokens.invisible


def _is_guide_purpose(prim) -> bool:
    purpose = UsdGeom.Imageable(prim).GetPurposeAttr().Get()
    return purpose == UsdGeom.Tokens.guide


def _is_fully_transparent(prim) -> bool:
    opacity = UsdGeom.Gprim(prim).GetDisplayOpacityAttr().Get()
    return bool(opacity) and all(float(value) <= 0.0 for value in opacity)


def _mesh_triangle_count(prim) -> int:
    mesh = UsdGeom.Mesh(prim)
    counts = mesh.GetFaceVertexCountsAttr().Get()
    if counts is None:
        return 0
    return len(counts)


def _world_bound_m(stage, root_prim_path: str) -> dict[str, list[float] | None]:
    prim = stage.GetPrimAtPath(root_prim_path)
    if not prim:
        return {"min_m": None, "max_m": None, "size_m": None}
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=False)
    aligned = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    if aligned.IsEmpty():
        return {"min_m": None, "max_m": None, "size_m": None}
    lower = aligned.GetMin()
    upper = aligned.GetMax()
    size = upper - lower
    return {
        "min_m": [round(float(lower[index]), 6) for index in range(3)],
        "max_m": [round(float(upper[index]), 6) for index in range(3)],
        "size_m": [round(float(size[index]), 6) for index in range(3)],
    }


def domino_linkage_visual_geometry_counts(stage, root_prim_path: str) -> dict:
    counts = {
        "root_prim_path": str(root_prim_path),
        "mesh_count": 0,
        "visible_mesh_count": 0,
        "actual_cad_mesh_count": 0,
        "visible_actual_cad_mesh_count": 0,
        "proxy_cube_count": 0,
        "visible_proxy_cube_count": 0,
        "guide_proxy_cube_count": 0,
        "transparent_proxy_cube_count": 0,
        "proxy_sphere_count": 0,
        "visible_proxy_sphere_count": 0,
        "guide_proxy_sphere_count": 0,
        "transparent_proxy_sphere_count": 0,
        "expected_actual_cad_mesh_count": EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT,
        "actual_cad_triangle_count": 0,
        "visible_actual_cad_triangle_count": 0,
        "expected_actual_cad_triangle_count": EXPECTED_ACTUAL_CAD_TOTAL_TRIANGLE_COUNT,
    }
    for prim in stage.Traverse():
        if not _prim_is_under_root(prim, root_prim_path):
            continue
        path = str(prim.GetPath())
        type_name = str(prim.GetTypeName())
        visible = _is_visible(prim)
        if type_name == "Mesh":
            counts["mesh_count"] += 1
            if visible:
                counts["visible_mesh_count"] += 1
            if "/actual_cad/" in path:
                triangle_count = _mesh_triangle_count(prim)
                counts["actual_cad_mesh_count"] += 1
                counts["actual_cad_triangle_count"] += triangle_count
                if visible:
                    counts["visible_actual_cad_mesh_count"] += 1
                    counts["visible_actual_cad_triangle_count"] += triangle_count
        elif type_name == "Cube":
            counts["proxy_cube_count"] += 1
            if visible:
                counts["visible_proxy_cube_count"] += 1
            if _is_guide_purpose(prim):
                counts["guide_proxy_cube_count"] += 1
            if _is_fully_transparent(prim):
                counts["transparent_proxy_cube_count"] += 1
        elif type_name == "Sphere":
            counts["proxy_sphere_count"] += 1
            if visible:
                counts["visible_proxy_sphere_count"] += 1
            if _is_guide_purpose(prim):
                counts["guide_proxy_sphere_count"] += 1
            if _is_fully_transparent(prim):
                counts["transparent_proxy_sphere_count"] += 1
    counts["actual_cad_mesh_complete"] = counts["actual_cad_mesh_count"] == EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT
    counts["visible_actual_cad_mesh_complete"] = counts["visible_actual_cad_mesh_count"] == EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT
    counts["actual_cad_triangle_count_matches"] = (
        counts["actual_cad_triangle_count"] == EXPECTED_ACTUAL_CAD_TOTAL_TRIANGLE_COUNT
    )
    counts["visible_actual_cad_triangle_count_matches"] = (
        counts["visible_actual_cad_triangle_count"] == EXPECTED_ACTUAL_CAD_TOTAL_TRIANGLE_COUNT
    )
    counts["proxy_visuals_hidden"] = counts["visible_proxy_cube_count"] == 0 and counts["visible_proxy_sphere_count"] == 0
    counts["proxy_visuals_non_renderable"] = (
        counts["guide_proxy_cube_count"] == counts["proxy_cube_count"]
        and counts["guide_proxy_sphere_count"] == counts["proxy_sphere_count"]
        and counts["transparent_proxy_cube_count"] == counts["proxy_cube_count"]
        and counts["transparent_proxy_sphere_count"] == counts["proxy_sphere_count"]
    )
    counts["world_bound_m"] = _world_bound_m(stage, root_prim_path)
    return counts


def validate_domino_actual_cad_visuals(stage, linkage: dict, require_hidden_proxy: bool = True) -> dict:
    counts = domino_linkage_visual_geometry_counts(stage, str(linkage["root_prim_path"]))
    errors = []
    attached = linkage.get("actual_cad_visuals") or {}
    if not linkage.get("actual_cad_visual"):
        errors.append("actual CAD visual attachment is disabled or missing")
    if int(attached.get("mesh_count", 0)) != EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT:
        errors.append(
            f"attached {attached.get('mesh_count', 0)} Domino CAD meshes; expected {EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT}"
        )
    if int(attached.get("triangle_count", 0)) != EXPECTED_ACTUAL_CAD_TOTAL_TRIANGLE_COUNT:
        errors.append(
            f"attached Domino CAD meshes contain {attached.get('triangle_count', 0)} triangles; expected "
            f"{EXPECTED_ACTUAL_CAD_TOTAL_TRIANGLE_COUNT}"
        )
    if not counts["actual_cad_mesh_complete"]:
        errors.append(
            f"stage has {counts['actual_cad_mesh_count']} Domino CAD mesh prims; expected {EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT}"
        )
    if not counts["visible_actual_cad_mesh_complete"]:
        errors.append(
            f"stage has {counts['visible_actual_cad_mesh_count']} visible Domino CAD mesh prims; expected "
            f"{EXPECTED_ACTUAL_CAD_VISUAL_LINK_COUNT}"
        )
    if not counts["actual_cad_triangle_count_matches"]:
        errors.append(
            f"stage has {counts['actual_cad_triangle_count']} Domino CAD triangles; expected "
            f"{EXPECTED_ACTUAL_CAD_TOTAL_TRIANGLE_COUNT}"
        )
    if not counts["visible_actual_cad_triangle_count_matches"]:
        errors.append(
            f"stage has {counts['visible_actual_cad_triangle_count']} visible Domino CAD triangles; expected "
            f"{EXPECTED_ACTUAL_CAD_TOTAL_TRIANGLE_COUNT}"
        )
    if require_hidden_proxy and not counts["proxy_visuals_hidden"]:
        errors.append(
            "proxy visuals are visible "
            f"(cubes={counts['visible_proxy_cube_count']}, spheres={counts['visible_proxy_sphere_count']})"
        )
    if require_hidden_proxy and not counts.get("proxy_visuals_non_renderable"):
        errors.append("proxy visuals are not marked as guide-purpose transparent geometry")
    if errors:
        raise RuntimeError("Domino actual-CAD visual validation failed: " + "; ".join(errors))
    return counts


DOMINO_FOUR_COMBINED_LEG_SPECS = [
    {
        "id": "dom_p_4_1",
        "hip_link": "DOM_P__4__1",
        "shoulder_joint": "Revolute 1",
        "shoulder_axis": "-X",
        "shoulder_limit_deg": centered_servo_limits_deg(0.0),
        "shoulder_center_deg": 0.0,
        "lower_drive_joint": "Revolute 59",
        "lower_drive_axis": "-Y",
        "upper_drive_joint": "Revolute 58",
        "upper_drive_axis": "-Y",
        "lower_passive_joint": "Revolute 43",
        "lower_coupler_joint": "Revolute 33",
        "lower_closure_joints": ("Revolute 25", "Revolute 26"),
        "upper_closure_joints": ("Revolute 32", "Revolute 51"),
        "lower_drive_limit_deg": centered_servo_limits_deg(22.5),
        # The front-right linkage is the rear-right mechanism translated
        # forward in CAD, so it uses the same physical drive-centre angles.
        "lower_drive_center_deg": 22.5,
        "upper_drive_center_deg": 22.5,
        "phase_deg": 0.0,
        "points": {
            "hip_origin": (0.266500, 0.000000, 0.010500),
            "upper_drive": (0.347000, -0.028000, 0.010500),
            "lower_drive": (0.323000, -0.028000, -0.010500),
            "lower_passive": (0.323000, -0.036000, -0.010500),
            "lower_coupler": (0.294708, -0.035600, 0.017777),
            "upper_closure_coupler": (0.312637, -0.035600, 0.028134),
            "upper_closure_driver": (0.336647, -0.035600, 0.049137),
            "lower_closure_diagonal": (0.210315, -0.048100, -0.123892),
            "lower_closure_driver": (0.182024, -0.048100, -0.095615),
            # Compatibility names retain the old collapsed-pivot coordinates.
            "upper_closure": (0.336647, -0.035600, 0.049137),
            "lower_closure": (0.182024, -0.048100, -0.095615),
        },
    },
    {
        "id": "dom_p_12_1",
        "hip_link": "DOM_P__12__1",
        "shoulder_joint": "Revolute 2",
        "shoulder_axis": "X",
        "shoulder_limit_deg": centered_servo_limits_deg(0.0),
        "shoulder_center_deg": 0.0,
        "lower_drive_joint": "Revolute 46",
        "lower_drive_axis": "-Y",
        "upper_drive_joint": "Revolute 55",
        "upper_drive_axis": "-Y",
        "lower_passive_joint": "Revolute 44",
        "lower_coupler_joint": "Revolute 36",
        "lower_closure_joints": ("Revolute 23", "Revolute 24"),
        "upper_closure_joints": ("Revolute 29", "Revolute 50"),
        "lower_drive_limit_deg": centered_servo_limits_deg(22.5),
        # The front-left lower axis is reversed relative to rear-left.  Equal
        # physical angles therefore require the opposite lower command sign.
        "lower_drive_center_deg": 22.5,
        "upper_drive_center_deg": 22.5,
        "phase_deg": 90.0,
        "points": {
            "hip_origin": (0.266500, 0.124750, 0.010500),
            "upper_drive": (0.347000, 0.152750, 0.010500),
            "lower_drive": (0.323000, 0.152750, -0.010500),
            "lower_passive": (0.323000, 0.160750, -0.010500),
            "lower_coupler": (0.294708, 0.160350, 0.017777),
            "upper_closure_coupler": (0.312637, 0.160350, 0.028134),
            "upper_closure_driver": (0.336647, 0.160350, 0.049137),
            "lower_closure_diagonal": (0.209962, 0.172850, -0.123538),
            "lower_closure_driver": (0.181670, 0.172850, -0.095261),
            "upper_closure": (0.336647, 0.160350, 0.049137),
            "lower_closure": (0.181670, 0.172850, -0.095261),
        },
    },
    {
        "id": "dom_p_25_1",
        "hip_link": "DOM_P__25__1",
        "shoulder_joint": "Revolute 3",
        "shoulder_axis": "X",
        "shoulder_limit_deg": centered_servo_limits_deg(0.0),
        "shoulder_center_deg": 0.0,
        "lower_drive_joint": "Revolute 47",
        "lower_drive_axis": "Y",
        "upper_drive_joint": "Revolute 56",
        "upper_drive_axis": "-Y",
        "lower_passive_joint": "Revolute 45",
        "lower_coupler_joint": "Revolute 35",
        "lower_closure_joints": ("Revolute 21", "Revolute 22"),
        "upper_closure_joints": ("Revolute 34", "Revolute 54"),
        "lower_drive_limit_deg": centered_servo_limits_deg(-22.5),
        "lower_drive_center_deg": -22.5,
        "upper_drive_center_deg": 22.5,
        "phase_deg": 180.0,
        "notes": [
            "The CAD URDF marks Revolute 47 as continuous even though its location mirrors the other lower drive pivots.",
            "This smoke test drives it with the same conservative lower-input range used for the other lower linkages.",
        ],
        "points": {
            "hip_origin": (-0.068500, 0.124750, 0.010500),
            "upper_drive": (0.012000, 0.152750, 0.010500),
            "lower_drive": (-0.012000, 0.152750, -0.010500),
            "lower_passive": (-0.012000, 0.160750, -0.010500),
            "lower_coupler": (-0.040292, 0.160350, 0.017777),
            "upper_closure_coupler": (-0.022363, 0.160350, 0.028134),
            "upper_closure_driver": (0.001647, 0.160350, 0.049137),
            "lower_closure_diagonal": (-0.125038, 0.172850, -0.123538),
            "lower_closure_driver": (-0.153330, 0.172850, -0.095261),
            "upper_closure": (0.001647, 0.160350, 0.049137),
            "lower_closure": (-0.153330, 0.172850, -0.095261),
        },
    },
    {
        "id": "dom_p_21_1",
        "hip_link": "DOM_P__21__1",
        "shoulder_joint": "Revolute 4",
        "shoulder_axis": "-X",
        "shoulder_limit_deg": centered_servo_limits_deg(0.0),
        "shoulder_center_deg": 0.0,
        "lower_drive_joint": "Revolute 48",
        "lower_drive_axis": "-Y",
        "upper_drive_joint": "Revolute 57",
        "upper_drive_axis": "-Y",
        "lower_passive_joint": "Revolute 42",
        "lower_coupler_joint": "Revolute 37",
        "lower_closure_joints": ("Revolute 27", "Revolute 28"),
        "upper_closure_joints": ("Revolute 31", "Revolute 53"),
        "lower_drive_limit_deg": centered_servo_limits_deg(22.5),
        "lower_drive_center_deg": 22.5,
        "upper_drive_center_deg": 22.5,
        "phase_deg": 270.0,
        "points": {
            "hip_origin": (-0.068500, 0.000000, 0.010500),
            "upper_drive": (0.012000, -0.028000, 0.010500),
            "lower_drive": (-0.012000, -0.028000, -0.010500),
            "lower_passive": (-0.012000, -0.036000, -0.010500),
            "lower_coupler": (-0.040292, -0.035600, 0.017777),
            "upper_closure_coupler": (-0.022363, -0.035600, 0.028134),
            "upper_closure_driver": (0.001647, -0.035600, 0.049137),
            "lower_closure_diagonal": (-0.124685, -0.048100, -0.123892),
            "lower_closure_driver": (-0.152976, -0.048100, -0.095615),
            "upper_closure": (0.001647, -0.035600, 0.049137),
            "lower_closure": (-0.152976, -0.048100, -0.095615),
        },
    },
]


def local_endpoint(point: np.ndarray, center: np.ndarray) -> Gf.Vec3f:
    point = np.asarray(point, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64)
    return Gf.Vec3f(float(point[0] - center[0]), float(point[1] - center[1]), float(point[2] - center[2]))


def set_schema_attr(api, create_attr_name: str, value) -> bool:
    create_attr = getattr(api, create_attr_name, None)
    if create_attr is None:
        return False
    create_attr().Set(value)
    return True


def apply_rigid_body(
    prim,
    mass: float,
    kinematic: bool,
    enable_gravity: bool,
    diagonal_inertia: np.ndarray | None = None,
) -> None:
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_api.CreateRigidBodyEnabledAttr(True)
    rigid_api.CreateKinematicEnabledAttr(bool(kinematic))

    if PhysxSchema is not None:
        physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physx_api.GetDisableGravityAttr().Set(bool(kinematic or not enable_gravity))
        physx_api.CreateLinearDampingAttr().Set(float(PHYSX_RIGID_BODY_LINEAR_DAMPING))
        physx_api.CreateAngularDampingAttr().Set(float(PHYSX_RIGID_BODY_ANGULAR_DAMPING))
        set_schema_attr(
            physx_api,
            "CreateSolverPositionIterationCountAttr",
            int(PHYSX_RIGID_BODY_SOLVER_POSITION_ITERATIONS),
        )
        set_schema_attr(
            physx_api,
            "CreateSolverVelocityIterationCountAttr",
            int(PHYSX_RIGID_BODY_SOLVER_VELOCITY_ITERATIONS),
        )
        set_schema_attr(
            physx_api,
            "CreateMaxDepenetrationVelocityAttr",
            float(PHYSX_RIGID_BODY_MAX_DEPENETRATION_VELOCITY_M_S),
        )
        set_schema_attr(physx_api, "CreateStabilizationThresholdAttr", 0.0)
        set_schema_attr(physx_api, "CreateEnableCCDAttr", not bool(kinematic))
        set_schema_attr(physx_api, "CreateEnableSpeculativeCCDAttr", not bool(kinematic))

    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(float(mass))
    inertia = np.asarray(
        diagonal_inertia if diagonal_inertia is not None else (0.0002, 0.0002, 0.0002),
        dtype=np.float64,
    )
    inertia = np.maximum(inertia, np.array([1.0e-5, 1.0e-5, 1.0e-5], dtype=np.float64))
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(float(inertia[0]), float(inertia[1]), float(inertia[2])))


def configure_proxy_visual(geom, visible: bool) -> None:
    if visible:
        return
    prim = geom.GetPrim()
    imageable = UsdGeom.Imageable(prim)
    imageable.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
    imageable.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
    gprim = UsdGeom.Gprim(prim)
    gprim.CreateDisplayOpacityAttr([0.0])
    gprim.CreateDisplayColorAttr([Gf.Vec3f(0.0, 0.0, 0.0)])


def bind_domino_contact_material(stage, prim) -> None:
    material_prim = stage.GetPrimAtPath(DOMINO_CONTACT_MATERIAL_PATH)
    if material_prim.IsValid():
        material = UsdShade.Material(material_prim)
    else:
        material = UsdShade.Material.Define(stage, DOMINO_CONTACT_MATERIAL_PATH)
        material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        material_api.CreateStaticFrictionAttr(float(DOMINO_CONTACT_STATIC_FRICTION))
        material_api.CreateDynamicFrictionAttr(float(DOMINO_CONTACT_DYNAMIC_FRICTION))
        material_api.CreateRestitutionAttr(float(DOMINO_CONTACT_RESTITUTION))
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material,
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics",
    )


def bind_domino_tpu_foot_material(stage, prim) -> None:
    material_prim = stage.GetPrimAtPath(DOMINO_TPU_FOOT_MATERIAL_PATH)
    if material_prim.IsValid():
        material = UsdShade.Material(material_prim)
    else:
        material = UsdShade.Material.Define(stage, DOMINO_TPU_FOOT_MATERIAL_PATH)
        material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        material_api.CreateStaticFrictionAttr(float(DOMINO_TPU_FOOT_STATIC_FRICTION))
        material_api.CreateDynamicFrictionAttr(float(DOMINO_TPU_FOOT_DYNAMIC_FRICTION))
        material_api.CreateRestitutionAttr(float(DOMINO_TPU_FOOT_RESTITUTION))
        if PhysxSchema is not None:
            physx_material_api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
            physx_material_api.CreateFrictionCombineModeAttr(
                DOMINO_TPU_FOOT_FRICTION_COMBINE_MODE
            )
            physx_material_api.CreateRestitutionCombineModeAttr(
                DOMINO_TPU_FOOT_RESTITUTION_COMBINE_MODE
            )
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="physics",
    )


def create_body_from_points(
    stage,
    root: str,
    name: str,
    points: list[np.ndarray],
    width: float,
    mass: float,
    kinematic: bool,
    enable_gravity: bool,
    proxy_visible: bool = True,
    enable_collision: bool = True,
):
    point_array = np.vstack([np.asarray(point, dtype=np.float64) for point in points])
    center = point_array.mean(axis=0)
    extents = point_array.max(axis=0) - point_array.min(axis=0)
    visual_scale = np.maximum(extents, np.array([width, width, width], dtype=np.float64))
    inertia = (float(mass) / 12.0) * np.array(
        [
            (visual_scale[1] * visual_scale[1]) + (visual_scale[2] * visual_scale[2]),
            (visual_scale[0] * visual_scale[0]) + (visual_scale[2] * visual_scale[2]),
            (visual_scale[0] * visual_scale[0]) + (visual_scale[1] * visual_scale[1]),
        ],
        dtype=np.float64,
    )

    body_path = f"{root}/{name}"
    body = UsdGeom.Xform.Define(stage, body_path)
    UsdGeom.XformCommonAPI(body).SetTranslate(Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
    apply_rigid_body(
        body.GetPrim(),
        mass=mass,
        kinematic=kinematic,
        enable_gravity=enable_gravity,
        diagonal_inertia=inertia,
    )

    visual = UsdGeom.Cube.Define(stage, f"{body_path}/visual")
    visual.CreateSizeAttr(1.0)
    configure_proxy_visual(visual, visible=proxy_visible)
    UsdGeom.XformCommonAPI(visual).SetScale(
        Gf.Vec3f(float(visual_scale[0]), float(visual_scale[1]), float(visual_scale[2]))
    )
    collision_path = ""
    if enable_collision:
        collision = UsdGeom.Cube.Define(stage, f"{body_path}/ground_collision")
        collision.CreateSizeAttr(1.0)
        configure_proxy_visual(collision, visible=False)
        UsdGeom.XformCommonAPI(collision).SetScale(
            Gf.Vec3f(float(visual_scale[0]), float(visual_scale[1]), float(visual_scale[2]))
        )
        UsdPhysics.CollisionAPI.Apply(collision.GetPrim())
        bind_domino_contact_material(stage, collision.GetPrim())
        collision_path = str(collision.GetPath())
    return {
        "path": Sdf.Path(body_path),
        "center": center,
        "points": point_array,
        "ground_collision_path": collision_path,
        "collision_half_extents_m": (
            (0.5 * visual_scale).tolist()
            if enable_collision
            else None
        ),
        "collision_local_center_m": (
            [0.0, 0.0, 0.0]
            if enable_collision
            else None
        ),
        "ground_collision_fit_source": (
            "joint_point_proxy_aabb"
            if enable_collision
            else "disabled"
        ),
    }


def create_robot_self_collision_filter(stage, root_prim_path: str) -> dict:
    """Disable robot-on-robot contacts while retaining robot-ground contacts."""
    group_path = f"{str(root_prim_path).rstrip('/')}/robot_collision_group"
    group = UsdPhysics.CollisionGroup.Define(stage, group_path)
    colliders = Usd.CollectionAPI.Apply(group.GetPrim(), "colliders")
    colliders.CreateIncludesRel().SetTargets([Sdf.Path(root_prim_path)])
    group.CreateFilteredGroupsRel().SetTargets([Sdf.Path(group_path)])
    return {
        "path": group_path,
        "included_root": str(root_prim_path),
        "self_collision_filtered": True,
    }


def apply_calibrated_neutral_body_poses(
    stage,
    bodies: dict,
    world_translation_m: tuple[float, float, float],
    resolved_floating_height_m: float,
) -> dict[str, object]:
    missing = sorted(set(bodies) - set(CALIBRATED_NEUTRAL_BODY_POSES))
    if missing:
        raise RuntimeError(f"Calibrated Domino neutral pose is missing rigid bodies: {missing}")

    world_translation = np.asarray(world_translation_m, dtype=np.float64)
    height_delta = float(resolved_floating_height_m) - float(CAPTURE_RESOLVED_FLOATING_HEIGHT_M)
    for body_name, body in bodies.items():
        captured_position, orientation = CALIBRATED_NEUTRAL_BODY_POSES[body_name]
        position = np.asarray(captured_position, dtype=np.float64) + world_translation
        position[2] += height_delta
        prim = stage.GetPrimAtPath(body["path"])
        xformable = UsdGeom.Xformable(prim)
        translate_op = next(
            (op for op in xformable.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate),
            None,
        )
        if translate_op is None:
            translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        translate_op.Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
        orient_op = next(
            (op for op in xformable.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeOrient),
            None,
        )
        if orient_op is None:
            orient_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        w, x, y, z = (float(value) for value in orientation)
        orient_op.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))

    return {
        "applied": True,
        "source": "rear_fixed_base_capture_with_front_pairs_translated_from_rear",
        "body_count": len(bodies),
        "capture_resolved_floating_height_m": float(CAPTURE_RESOLVED_FLOATING_HEIGHT_M),
        "resolved_floating_height_m": float(resolved_floating_height_m),
        "front_from_rear_translation_m": list(FRONT_FROM_REAR_TRANSLATION_M),
        "front_rear_body_pairs": dict(FRONT_REAR_BODY_PAIRS),
    }


def create_static_ground_box(
    stage,
    prim_path: str,
    size_m: float,
    thickness_m: float,
    top_z_m: float = 0.0,
    center_xy_m: tuple[float, float] = (0.0, 0.0),
    visible: bool = True,
    collision: bool = True,
):
    ground = UsdGeom.Cube.Define(stage, prim_path)
    ground.CreateSizeAttr(1.0)
    xform = UsdGeom.XformCommonAPI(ground)
    xform.SetTranslate(
        Gf.Vec3d(
            float(center_xy_m[0]),
            float(center_xy_m[1]),
            float(top_z_m) - (0.5 * float(thickness_m)),
        )
    )
    xform.SetScale(Gf.Vec3f(float(size_m), float(size_m), float(thickness_m)))
    if not visible:
        configure_proxy_visual(ground, visible=False)
    if collision:
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
        bind_domino_contact_material(stage, ground.GetPrim())
    return {
        "path": prim_path,
        "size_m": float(size_m),
        "thickness_m": float(thickness_m),
        "top_z_m": float(top_z_m),
        "center_xy_m": [float(center_xy_m[0]), float(center_xy_m[1])],
        "visible": bool(visible),
        "collision": bool(collision),
    }


def create_static_box(
    stage,
    prim_path: str,
    center_m: tuple[float, float, float],
    size_m: tuple[float, float, float],
    color: tuple[float, float, float],
    visible: bool = True,
    collision: bool = True,
):
    box = UsdGeom.Cube.Define(stage, prim_path)
    box.CreateSizeAttr(1.0)
    xform = UsdGeom.XformCommonAPI(box)
    xform.SetTranslate(Gf.Vec3d(float(center_m[0]), float(center_m[1]), float(center_m[2])))
    xform.SetScale(Gf.Vec3f(float(size_m[0]), float(size_m[1]), float(size_m[2])))
    box.CreateDisplayColorAttr([Gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))])
    if not visible:
        configure_proxy_visual(box, visible=False)
    if collision:
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
        bind_domino_contact_material(stage, box.GetPrim())
    return {
        "path": prim_path,
        "center_m": list(center_m),
        "size_m": list(size_m),
        "visible": bool(visible),
        "collision": bool(collision),
    }


def create_static_stairs_terrain(
    stage,
    root_prim_path: str,
    origin_m: tuple[float, float, float],
    step_count: int,
    step_depth_m: float,
    step_height_m: float,
    width_m: float,
    start_x_m: float,
    top_platform_length_m: float,
    top_z_offset_m: float = 0.0,
    visible: bool = True,
    collision: bool = True,
) -> dict:
    UsdGeom.Xform.Define(stage, root_prim_path)
    origin = np.asarray(origin_m, dtype=np.float64)
    steps = []
    for index in range(int(step_count)):
        height = float(index + 1) * float(step_height_m)
        center = (
            float(origin[0]) + float(start_x_m) + (float(index) + 0.5) * float(step_depth_m),
            float(origin[1]),
            float(top_z_offset_m) + (0.5 * height),
        )
        color_value = 0.34 + (0.035 * (index % 4))
        steps.append(
            create_static_box(
                stage,
                f"{root_prim_path}/step_{index:02d}",
                center,
                (float(step_depth_m), float(width_m), height),
                (color_value, color_value, color_value),
                visible=visible,
                collision=collision,
            )
        )
    top_height = float(step_count) * float(step_height_m)
    platform = None
    if float(top_platform_length_m) > 0.0:
        platform = create_static_box(
            stage,
            f"{root_prim_path}/top_platform",
            (
                float(origin[0]) + float(start_x_m) + (float(step_count) * float(step_depth_m)) + 0.5 * float(top_platform_length_m),
                float(origin[1]),
                float(top_z_offset_m) + (0.5 * top_height),
            ),
            (float(top_platform_length_m), float(width_m), top_height),
            (0.48, 0.48, 0.46),
            visible=visible,
            collision=collision,
        )
    return {
        "type": "stairs",
        "root_prim_path": root_prim_path,
        "origin_m": [float(value) for value in origin],
        "step_count": int(step_count),
        "step_depth_m": float(step_depth_m),
        "step_height_m": float(step_height_m),
        "width_m": float(width_m),
        "start_x_m": float(start_x_m),
        "top_platform_length_m": float(top_platform_length_m),
        "top_height_m": top_height,
        "top_z_offset_m": float(top_z_offset_m),
        "visible": bool(visible),
        "collision": bool(collision),
        "steps": steps,
        "top_platform": platform,
    }


def create_body_collision_sphere(stage, body: dict, name: str, center: np.ndarray, radius_m: float, proxy_visible: bool = True):
    sphere_path = f"{body['path']}/{name}"
    sphere = UsdGeom.Sphere.Define(stage, sphere_path)
    sphere.CreateRadiusAttr(float(radius_m))
    configure_proxy_visual(sphere, visible=proxy_visible)
    local = local_endpoint(center, body["center"])
    UsdGeom.XformCommonAPI(sphere).SetTranslate(Gf.Vec3d(float(local[0]), float(local[1]), float(local[2])))
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    bind_domino_tpu_foot_material(stage, sphere.GetPrim())
    return {
        "path": str(sphere_path),
        "radius_m": float(radius_m),
        "center_m": np.asarray(center).tolist(),
        "local_center_m": np.asarray(local).tolist(),
        "contact_material": "tpu",
        "contact_material_path": DOMINO_TPU_FOOT_MATERIAL_PATH,
    }


def create_pin_joint(
    stage,
    path: str,
    body0: dict,
    body1: dict,
    pivot: np.ndarray,
    lower_deg: float | None = None,
    upper_deg: float | None = None,
    axis: str = "Y",
    enable_projection: bool = True,
):
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0["path"]])
    joint.CreateBody1Rel().SetTargets([body1["path"]])
    joint.CreateLocalPos0Attr().Set(local_endpoint(pivot, body0["center"]))
    joint.CreateLocalPos1Attr().Set(local_endpoint(pivot, body1["center"]))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateAxisAttr(axis)
    joint.CreateCollisionEnabledAttr(False)
    if lower_deg is not None:
        joint.CreateLowerLimitAttr(float(lower_deg))
    if upper_deg is not None:
        joint.CreateUpperLimitAttr(float(upper_deg))
    if PhysxSchema is not None:
        physx_joint_api = PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
        set_schema_attr(physx_joint_api, "CreateArmatureAttr", float(PHYSX_PIN_JOINT_ARMATURE))
        set_schema_attr(physx_joint_api, "CreateMaxJointVelocityAttr", float(PHYSX_PIN_JOINT_MAX_VELOCITY_DEG_S))
        joint.GetPrim().CreateAttribute("physxJoint:enableProjection", Sdf.ValueTypeNames.Bool).Set(
            bool(enable_projection)
        )
        joint.GetPrim().CreateAttribute(
            "physxJoint:projectionLinearTolerance",
            Sdf.ValueTypeNames.Float,
        ).Set(float(PHYSX_PIN_JOINT_PROJECTION_LINEAR_TOLERANCE_M))
        joint.GetPrim().CreateAttribute(
            "physxJoint:projectionAngularTolerance",
            Sdf.ValueTypeNames.Float,
        ).Set(float(PHYSX_PIN_JOINT_PROJECTION_ANGULAR_TOLERANCE_DEG))
    return joint


def axis_token(axis_label: str) -> str:
    label = str(axis_label).strip().upper()
    if label.startswith("-"):
        label = label[1:]
    if label not in {"X", "Y", "Z"}:
        raise ValueError(f"Unsupported Domino joint axis label: {axis_label}")
    return label


def axis_sign(axis_label: str) -> float:
    return -1.0 if str(axis_label).strip().startswith("-") else 1.0


def normalized_foot_contact_mode(config: DominoCadLinkageBuildConfig) -> str:
    mode = str(config.foot_contact_mode or "").strip().lower().replace("-", "_")
    if not mode:
        mode = "actual_cad_visual_bottom" if config.use_actual_cad_foot_collision else "linkage_lower_closure"
    if mode == "actual_cad_foot_collision":
        mode = "actual_cad_visual_bottom"
    if mode not in {"linkage_lower_closure", "actual_cad_visual_bottom", "actual_cad_grounded_support"}:
        raise ValueError(f"Unsupported Domino foot contact mode: {config.foot_contact_mode!r}")
    return mode


def normalized_closure_model(config: DominoCadLinkageBuildConfig) -> str:
    value = str(config.closure_model or "direct").strip().lower().replace("-", "_")
    aliases = {
        "direct": "direct",
        "direct_loop": "direct",
        "direct_loop_closure": "direct",
        "passive": "passive",
        "split": "passive",
        "split_closure": "passive",
        "passive_closure": "passive",
        "passive_closure_links": "passive",
    }
    if value not in aliases:
        raise ValueError(f"Unsupported Domino closure model: {config.closure_model!r}")
    return aliases[value]


def signed_limit_deg(limit_deg: tuple[float, float], sign: float) -> tuple[float, float]:
    signed = [float(sign) * float(limit_deg[0]), float(sign) * float(limit_deg[1])]
    return min(signed), max(signed)


def apply_angular_drive(joint, stiffness: float, damping: float, max_force: float, target_deg: float):
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(stiffness))
    drive.CreateDampingAttr(float(damping))
    drive.CreateMaxForceAttr(float(max_force))
    drive.CreateTargetPositionAttr(float(target_deg))
    drive.CreateTargetVelocityAttr(0.0)
    return drive


def make_drive_spec(
    joint: str,
    drive,
    center_deg: float,
    amplitude_source: str,
    frequency_source: str,
    phase_deg: float,
    role: str,
    axis: str,
    target_limit_deg: tuple[float, float],
    action_name: str,
    command_center_deg: float | None = None,
    command_limit_deg: tuple[float, float] | None = None,
    target_sign: float = 1.0,
    cad_axis: str | None = None,
    actuator_model: dict | None = None,
):
    command_center = float(center_deg if command_center_deg is None else command_center_deg)
    command_limit = command_limit_deg or target_limit_deg
    actuator = dict(actuator_model or {})
    return {
        "joint": joint,
        "drive": drive,
        "center_deg": float(center_deg),
        "command_center_deg": command_center,
        "amplitude_source": amplitude_source,
        "frequency_source": frequency_source,
        "phase_deg": float(phase_deg),
        "role": role,
        "axis": axis,
        "cad_axis": cad_axis or axis,
        "target_sign": float(target_sign),
        "command_limit_deg": [float(command_limit[0]), float(command_limit[1])],
        "target_limit_deg": [float(target_limit_deg[0]), float(target_limit_deg[1])],
        "action_name": action_name,
        "actuator_model": actuator,
        "target_rate_limit_deg_s": (
            float(actuator["target_rate_limit_deg_s"])
            if actuator.get("target_rate_limit_deg_s") is not None
            else None
        ),
    }


def joint_key(leg_id: str, joint_name: str) -> str:
    return f"{leg_id}_{joint_name.lower().replace(' ', '_')}"


def build_domino_combined_leg_instance(
    stage,
    root: str,
    spec: dict,
    shared_base: dict,
    world_offset: np.ndarray,
    config: DominoCadLinkageBuildConfig,
    actual_cad_foot_collision: dict | None = None,
) -> dict:
    leg_root = f"{root}/{spec['id']}"
    UsdGeom.Xform.Define(stage, leg_root)
    points = {name: np.array(value, dtype=np.float64) + world_offset for name, value in spec["points"].items()}
    prefix = spec["id"]
    proxy_visible = not (config.include_actual_cad_visuals and config.hide_proxy_visuals_when_actual_cad)

    base_anchor_key = shared_base["key"]
    base_anchor = shared_base["body"]
    ground_key = f"{prefix}_ground"
    lower_driver_key = f"{prefix}_lower_driver"
    coupler_key = f"{prefix}_coupler"
    lower_diagonal_key = f"{prefix}_lower_diagonal"
    upper_driver_key = f"{prefix}_upper_driver"
    lower_closure_key = f"{prefix}_lower_closure"
    upper_closure_key = f"{prefix}_upper_closure"
    bodies = {}
    drives = []
    passive_stabilizers = []
    joint_checks = []
    actual_cad_foot = None
    closure_model = normalized_closure_model(config)
    use_passive_closure = closure_model == "passive"
    if actual_cad_foot_collision is not None:
        actual_cad_foot = actual_cad_foot_collision.get("feet", {}).get(lower_closure_key)

    def record_joint_check(name: str, body_a_key: str, body_b_key: str, pivot: np.ndarray, role: str) -> None:
        joint_checks.append(
            {
                "name": name,
                "role": role,
                "body_a": body_a_key,
                "body_b": body_b_key,
                "pivot": np.asarray(pivot, dtype=np.float64).tolist(),
            }
        )

    ground = create_body_from_points(
        stage,
        leg_root,
        "hip_carriage",
        [points["hip_origin"], points["upper_drive"], points["lower_drive"]],
        width=0.014,
        mass=0.12,
        kinematic=False,
        enable_gravity=config.enable_gravity,
        proxy_visible=proxy_visible,
        enable_collision=bool(config.enable_body_collisions),
    )
    lower_driver = create_body_from_points(
        stage,
        leg_root,
        "lower_driver",
        [points["lower_drive"], points["lower_passive"], points["lower_closure_driver"]],
        width=0.010,
        mass=0.08,
        kinematic=False,
        enable_gravity=config.enable_gravity,
        proxy_visible=proxy_visible,
        enable_collision=bool(config.enable_body_collisions),
    )
    coupler = create_body_from_points(
        stage,
        leg_root,
        "shared_coupler",
        [points["lower_passive"], points["lower_coupler"], points["upper_closure_coupler"]],
        width=0.008,
        mass=0.05,
        kinematic=False,
        enable_gravity=config.enable_gravity,
        proxy_visible=proxy_visible,
        enable_collision=bool(config.enable_body_collisions),
    )
    lower_diagonal = create_body_from_points(
        stage,
        leg_root,
        "lower_diagonal",
        [points["lower_coupler"], points["lower_closure_diagonal"]],
        width=0.008,
        mass=0.04,
        kinematic=False,
        enable_gravity=config.enable_gravity,
        proxy_visible=proxy_visible,
        enable_collision=bool(config.enable_body_collisions),
    )
    upper_driver = create_body_from_points(
        stage,
        leg_root,
        "upper_driver",
        [points["upper_drive"], points["upper_closure_driver"]],
        width=0.010,
        mass=0.06,
        kinematic=False,
        enable_gravity=config.enable_gravity,
        proxy_visible=proxy_visible,
        enable_collision=bool(config.enable_body_collisions),
    )
    lower_closure_body = None
    upper_closure_body = None
    if use_passive_closure:
        lower_closure_body = create_body_from_points(
            stage,
            leg_root,
            "lower_closure",
            [points["lower_closure_driver"], points["lower_closure_diagonal"]],
            width=0.006,
            mass=0.015,
            kinematic=False,
            enable_gravity=config.enable_gravity,
            proxy_visible=proxy_visible,
            enable_collision=bool(config.enable_body_collisions),
        )
        upper_closure_body = create_body_from_points(
            stage,
            leg_root,
            "upper_closure",
            [points["upper_closure_coupler"], points["upper_closure_driver"]],
            width=0.006,
            mass=0.015,
            kinematic=False,
            enable_gravity=config.enable_gravity,
            proxy_visible=proxy_visible,
            enable_collision=bool(config.enable_body_collisions),
        )
    shoulder_joint_name = joint_key(prefix, spec["shoulder_joint"])
    shoulder_sign = axis_sign(spec["shoulder_axis"])
    shoulder_limit_deg = signed_limit_deg(spec["shoulder_limit_deg"], shoulder_sign)
    shoulder_center_deg = shoulder_sign * float(spec["shoulder_center_deg"])
    shoulder_joint = create_pin_joint(
        stage,
        f"{leg_root}/joints/{shoulder_joint_name}",
        base_anchor,
        ground,
        points["hip_origin"],
        lower_deg=shoulder_limit_deg[0],
        upper_deg=shoulder_limit_deg[1],
        axis=axis_token(spec["shoulder_axis"]),
    )
    record_joint_check(shoulder_joint_name, base_anchor_key, ground_key, points["hip_origin"], "shoulder_hip_ab_ad")
    shoulder_drive = apply_angular_drive(
        shoulder_joint,
        stiffness=SHOULDER_SERVO_DRIVE_STIFFNESS,
        damping=SHOULDER_SERVO_DRIVE_DAMPING,
        max_force=SHOULDER_SERVO_STALL_TORQUE_N_M,
        target_deg=shoulder_center_deg,
    )
    drives.append(
        make_drive_spec(
            shoulder_joint_name,
            shoulder_drive,
            shoulder_center_deg,
            amplitude_source="shoulder",
            frequency_source="shoulder",
            phase_deg=spec["phase_deg"],
            role="shoulder_ab_ad",
            axis=axis_token(spec["shoulder_axis"]),
            target_limit_deg=shoulder_limit_deg,
            action_name=f"{prefix}_shoulder_ab_ad",
            command_center_deg=spec["shoulder_center_deg"],
            command_limit_deg=spec["shoulder_limit_deg"],
            target_sign=shoulder_sign,
            cad_axis=spec["shoulder_axis"],
            actuator_model=SERVO_ACTUATOR_MODEL["roles"]["shoulder_ab_ad"],
        )
    )

    lower_joint_name = joint_key(prefix, spec["lower_drive_joint"])
    upper_joint_name = joint_key(prefix, spec["upper_drive_joint"])
    lower_sign = axis_sign(spec["lower_drive_axis"])
    lower_limit_deg = signed_limit_deg(spec["lower_drive_limit_deg"], lower_sign)
    lower_center_deg = lower_sign * float(spec["lower_drive_center_deg"])
    upper_sign = axis_sign(spec["upper_drive_axis"])
    upper_command_limit_deg = centered_servo_limits_deg(float(spec["upper_drive_center_deg"]))
    upper_limit_deg = signed_limit_deg(upper_command_limit_deg, upper_sign)
    upper_center_deg = upper_sign * float(spec["upper_drive_center_deg"])
    lower_drive_joint = create_pin_joint(
        stage,
        f"{leg_root}/joints/{lower_joint_name}",
        ground,
        lower_driver,
        points["lower_drive"],
        lower_deg=lower_limit_deg[0],
        upper_deg=lower_limit_deg[1],
        axis=axis_token(spec["lower_drive_axis"]),
    )
    record_joint_check(lower_joint_name, ground_key, lower_driver_key, points["lower_drive"], "lower_linkage_drive")
    create_pin_joint(
        stage,
        f"{leg_root}/joints/{joint_key(prefix, spec['lower_passive_joint'])}",
        lower_driver,
        coupler,
        points["lower_passive"],
    )
    record_joint_check(
        joint_key(prefix, spec["lower_passive_joint"]),
        lower_driver_key,
        coupler_key,
        points["lower_passive"],
        "lower_passive_pin",
    )
    create_pin_joint(
        stage,
        f"{leg_root}/joints/{joint_key(prefix, spec['lower_coupler_joint'])}",
        coupler,
        lower_diagonal,
        points["lower_coupler"],
    )
    record_joint_check(
        joint_key(prefix, spec["lower_coupler_joint"]),
        coupler_key,
        lower_diagonal_key,
        points["lower_coupler"],
        "lower_coupler_pin",
    )
    lower_closure_joint_a_name = joint_key(prefix, spec["lower_closure_joints"][0])
    lower_closure_joint_b_name = joint_key(prefix, spec["lower_closure_joints"][1])
    if use_passive_closure:
        create_pin_joint(
            stage,
            f"{leg_root}/joints/{lower_closure_joint_a_name}",
            lower_driver,
            lower_closure_body,
            points["lower_closure_driver"],
        )
        record_joint_check(
            lower_closure_joint_a_name,
            lower_driver_key,
            lower_closure_key,
            points["lower_closure_driver"],
            "lower_loop_closure_driver_pin",
        )
        create_pin_joint(
            stage,
            f"{leg_root}/joints/{lower_closure_joint_b_name}",
            lower_diagonal,
            lower_closure_body,
            points["lower_closure_diagonal"],
            enable_projection=False,
        )
        record_joint_check(
            lower_closure_joint_b_name,
            lower_diagonal_key,
            lower_closure_key,
            points["lower_closure_diagonal"],
            "lower_loop_closure_diagonal_pin",
        )
    else:
        create_pin_joint(
            stage,
            f"{leg_root}/joints/{lower_closure_joint_a_name}",
            lower_driver,
            lower_diagonal,
            points["lower_closure"],
            enable_projection=False,
        )
        record_joint_check(
            lower_closure_joint_a_name,
            lower_driver_key,
            lower_diagonal_key,
            points["lower_closure"],
            "lower_loop_closure_pin",
        )
    upper_drive_joint = create_pin_joint(
        stage,
        f"{leg_root}/joints/{upper_joint_name}",
        ground,
        upper_driver,
        points["upper_drive"],
        lower_deg=upper_limit_deg[0],
        upper_deg=upper_limit_deg[1],
        axis=axis_token(spec["upper_drive_axis"]),
    )
    record_joint_check(upper_joint_name, ground_key, upper_driver_key, points["upper_drive"], "upper_pitch_drive")
    upper_closure_joint_a_name = joint_key(prefix, spec["upper_closure_joints"][0])
    upper_closure_joint_b_name = joint_key(prefix, spec["upper_closure_joints"][1])
    if use_passive_closure:
        create_pin_joint(
            stage,
            f"{leg_root}/joints/{upper_closure_joint_a_name}",
            coupler,
            upper_closure_body,
            points["upper_closure_coupler"],
        )
        record_joint_check(
            upper_closure_joint_a_name,
            coupler_key,
            upper_closure_key,
            points["upper_closure_coupler"],
            "upper_loop_closure_coupler_pin",
        )
        create_pin_joint(
            stage,
            f"{leg_root}/joints/{upper_closure_joint_b_name}",
            upper_driver,
            upper_closure_body,
            points["upper_closure_driver"],
            enable_projection=False,
        )
        record_joint_check(
            upper_closure_joint_b_name,
            upper_driver_key,
            upper_closure_key,
            points["upper_closure_driver"],
            "upper_loop_closure_driver_pin",
        )
    else:
        create_pin_joint(
            stage,
            f"{leg_root}/joints/{upper_closure_joint_a_name}",
            coupler,
            upper_driver,
            points["upper_closure"],
            enable_projection=False,
        )
        record_joint_check(
            upper_closure_joint_a_name,
            coupler_key,
            upper_driver_key,
            points["upper_closure"],
            "upper_loop_closure_pin",
        )

    lower_drive = apply_angular_drive(
        lower_drive_joint,
        stiffness=LINKAGE_SERVO_DRIVE_STIFFNESS,
        damping=LINKAGE_SERVO_DRIVE_DAMPING,
        max_force=LINKAGE_SERVO_STALL_TORQUE_N_M,
        target_deg=lower_center_deg,
    )
    upper_drive = apply_angular_drive(
        upper_drive_joint,
        stiffness=LINKAGE_SERVO_DRIVE_STIFFNESS,
        damping=LINKAGE_SERVO_DRIVE_DAMPING,
        max_force=LINKAGE_SERVO_STALL_TORQUE_N_M,
        target_deg=upper_center_deg,
    )
    drives.extend(
        [
            make_drive_spec(
                lower_joint_name,
                lower_drive,
                lower_center_deg,
                amplitude_source="primary",
                frequency_source="primary",
                phase_deg=spec["phase_deg"],
                role="lower_linkage_drive",
                axis=axis_token(spec["lower_drive_axis"]),
                target_limit_deg=lower_limit_deg,
                action_name=f"{prefix}_lower_linkage",
                command_center_deg=spec["lower_drive_center_deg"],
                command_limit_deg=spec["lower_drive_limit_deg"],
                target_sign=lower_sign,
                cad_axis=spec["lower_drive_axis"],
                actuator_model=SERVO_ACTUATOR_MODEL["roles"]["lower_linkage_drive"],
            ),
            make_drive_spec(
                upper_joint_name,
                upper_drive,
                upper_center_deg,
                amplitude_source="secondary",
                frequency_source="secondary",
                phase_deg=spec["phase_deg"] + 90.0,
                role="upper_pitch_drive",
                axis=axis_token(spec["upper_drive_axis"]),
                target_limit_deg=upper_limit_deg,
                action_name=f"{prefix}_upper_pitch",
                command_center_deg=spec["upper_drive_center_deg"],
                command_limit_deg=upper_command_limit_deg,
                target_sign=upper_sign,
                cad_axis=spec["upper_drive_axis"],
                actuator_model=SERVO_ACTUATOR_MODEL["roles"]["upper_pitch_drive"],
            ),
        ]
    )

    foot_center = points["lower_closure"]
    foot_source = "linkage_lower_closure"
    if actual_cad_foot is not None:
        foot_center = np.asarray(actual_cad_foot["sphere_center_without_offset_m"], dtype=np.float64) + world_offset
        foot_source = str(actual_cad_foot_collision.get("source", "actual_cad_visual_bottom"))
    foot_radius_m = float(config.foot_proxy_radius_m)
    if actual_cad_foot is not None and "radius_m" in actual_cad_foot:
        foot_radius_m = float(actual_cad_foot["radius_m"])
    foot_body_key = (
        lower_closure_key
        if use_passive_closure
        else ACTUAL_CAD_VISUAL_BODY_ALIASES.get(lower_closure_key, lower_driver_key)
    )
    foot_parent_body = {
        lower_driver_key: lower_driver,
        lower_diagonal_key: lower_diagonal,
        lower_closure_key: lower_closure_body,
    }.get(foot_body_key, lower_driver)
    if foot_body_key not in {lower_driver_key, lower_diagonal_key, lower_closure_key}:
        foot_body_key = lower_driver_key

    foot_proxy = create_body_collision_sphere(
        stage,
        foot_parent_body,
        f"{prefix}_foot_proxy",
        foot_center,
        foot_radius_m,
        proxy_visible=proxy_visible,
    )
    bodies.update(
        {
            ground_key: ground,
            lower_driver_key: lower_driver,
            coupler_key: coupler,
            lower_diagonal_key: lower_diagonal,
            upper_driver_key: upper_driver,
        }
    )
    if use_passive_closure:
        bodies[lower_closure_key] = lower_closure_body
        bodies[upper_closure_key] = upper_closure_body

    lower_loop_name = (
        f"{prefix}_lower_loop_closure_"
        f"{spec['lower_closure_joints'][0].replace(' ', '_')}_{spec['lower_closure_joints'][1].replace(' ', '_')}"
    )
    upper_loop_name = (
        f"{prefix}_upper_loop_closure_"
        f"{spec['upper_closure_joints'][0].replace(' ', '_')}_{spec['upper_closure_joints'][1].replace(' ', '_')}"
    )
    return {
        "points": {f"{prefix}_{name}": point.tolist() for name, point in points.items()},
        "bodies": bodies,
        "drives": drives,
        "passive_stabilizers": passive_stabilizers,
        "joint_checks": joint_checks,
        "loop_checks": (
            [
                {
                    "name": lower_loop_name,
                    "body_a": lower_driver_key,
                    "body_b": lower_closure_key,
                    "pivot": points["lower_closure_driver"].tolist(),
                },
                {
                    "name": f"{lower_loop_name}_diagonal",
                    "body_a": lower_diagonal_key,
                    "body_b": lower_closure_key,
                    "pivot": points["lower_closure_diagonal"].tolist(),
                },
                {
                    "name": upper_loop_name,
                    "body_a": coupler_key,
                    "body_b": upper_closure_key,
                    "pivot": points["upper_closure_coupler"].tolist(),
                },
                {
                    "name": f"{upper_loop_name}_driver",
                    "body_a": upper_driver_key,
                    "body_b": upper_closure_key,
                    "pivot": points["upper_closure_driver"].tolist(),
                },
            ]
            if use_passive_closure
            else [
                {
                    "name": lower_loop_name,
                    "body_a": lower_driver_key,
                    "body_b": lower_diagonal_key,
                    "pivot": points["lower_closure"].tolist(),
                },
                {
                    "name": upper_loop_name,
                    "body_a": coupler_key,
                    "body_b": upper_driver_key,
                    "pivot": points["upper_closure"].tolist(),
                },
            ]
        ),
        "leg": {
            "id": spec["id"],
            "hip_link": spec["hip_link"],
            "shoulder_joint": spec["shoulder_joint"],
            "shoulder_axis": spec["shoulder_axis"],
            "lower_drive_joint": spec["lower_drive_joint"],
            "upper_drive_joint": spec["upper_drive_joint"],
            "foot_proxy": foot_proxy,
            "foot_proxy_body": foot_body_key,
            "actual_cad_visual_foot_body": foot_body_key,
            "physics_closure_model": (
                "finite_two_pivot_closure_rigid_bodies"
                if use_passive_closure
                else "legacy_collapsed_direct_loop_closure"
            ),
            "foot_proxy_source": foot_source,
            "actual_cad_visual_foot_center_m": (
                (np.asarray(actual_cad_foot["sphere_center_without_offset_m"], dtype=np.float64) + world_offset).tolist()
                if actual_cad_foot is not None
                else None
            ),
            "actual_cad_visual_foot_center_local_m": (
                list(foot_proxy["local_center_m"])
                if actual_cad_foot is not None
                else None
            ),
            "actual_cad_visual_foot_radius_m": (
                float(foot_radius_m)
                if actual_cad_foot is not None
                else None
            ),
            "visual_foot_bottom_m": (
                (np.asarray(actual_cad_foot["visual_bottom_without_offset_m"], dtype=np.float64) + world_offset).tolist()
                if actual_cad_foot is not None
                else None
            ),
            "lower_closure_joints": list(spec["lower_closure_joints"]),
            "upper_closure_joints": list(spec["upper_closure_joints"]),
            "notes": spec.get("notes", []),
        },
    }


def attach_drive_target_attrs(drive_specs: list[dict]) -> None:
    for spec in drive_specs:
        spec["target_attr"] = spec["drive"].GetTargetPositionAttr()
        spec["current_target_deg"] = float(spec["center_deg"])


def target_from_normalized_action(spec: dict, normalized_action: float, action_scale_deg: float) -> float:
    normalized = max(-1.0, min(1.0, float(normalized_action)))
    command_target = float(spec.get("command_center_deg", spec["center_deg"])) + (float(action_scale_deg) * normalized)
    command_lower_deg, command_upper_deg = spec.get("command_limit_deg", spec["target_limit_deg"])
    command_target = max(float(command_lower_deg), min(float(command_upper_deg), command_target))
    target = float(spec.get("target_sign", 1.0)) * command_target
    target_lower_deg, target_upper_deg = spec["target_limit_deg"]
    return max(float(target_lower_deg), min(float(target_upper_deg), target))


def set_drive_targets_from_actions(
    drive_specs: list[dict],
    normalized_actions,
    action_scale_deg: float,
    max_target_delta_deg: float | None = None,
) -> list[float]:
    action_values = list(normalized_actions)
    if len(action_values) != len(drive_specs):
        raise ValueError(f"Expected {len(drive_specs)} normalized actions, received {len(action_values)}.")
    max_delta = None
    if max_target_delta_deg is not None and float(max_target_delta_deg) > 0.0:
        max_delta = float(max_target_delta_deg)
    targets = []
    for spec, action in zip(drive_specs, action_values):
        desired_target = target_from_normalized_action(spec, float(action), action_scale_deg)
        target = desired_target
        if max_delta is not None:
            previous_target = float(spec.get("current_target_deg", spec["center_deg"]))
            delta = max(-max_delta, min(max_delta, desired_target - previous_target))
            target = previous_target + delta
        spec["target_attr"].Set(float(target))
        spec["current_target_deg"] = float(target)
        targets.append(float(target))
    return targets


def build_domino_four_12_floating_linkage(stage, config: DominoCadLinkageBuildConfig | None = None) -> dict:
    config = config or DominoCadLinkageBuildConfig()
    root = config.root_prim_path
    UsdGeom.Xform.Define(stage, root)
    foot_contact_mode = normalized_foot_contact_mode(config)
    closure_model = normalized_closure_model(config)
    proxy_visible = not (config.include_actual_cad_visuals and config.hide_proxy_visuals_when_actual_cad)
    mesh_dir = None
    actual_cad_foot_geometry = None
    actual_cad_foot_collision = None
    calibrated_foot_alignment = None
    resolved_floating_height_m = float(config.floating_height_m)
    actual_cad_visual_lift_m = 0.0
    needs_actual_cad_geometry = bool(config.include_actual_cad_visuals) or foot_contact_mode in {
        "actual_cad_visual_bottom",
        "actual_cad_grounded_support",
    }
    if needs_actual_cad_geometry:
        mesh_dir = Path(config.actual_cad_mesh_dir).expanduser() if config.actual_cad_mesh_dir else default_actual_cad_mesh_dir()
        mesh_dir = mesh_dir.resolve()
        actual_cad_foot_geometry = actual_cad_foot_collision_points(mesh_dir, float(config.foot_proxy_radius_m))
        visual_clearance_height = (
            -float(actual_cad_foot_geometry["min_visual_bottom_z_without_offset_m"])
            + float(config.actual_cad_ground_clearance_m)
        )
        if foot_contact_mode == "actual_cad_visual_bottom":
            if bool(config.use_calibrated_neutral_pose) and closure_model == "passive":
                calibrated_foot_alignment = calibrated_neutral_foot_alignment(actual_cad_foot_geometry)
                visual_clearance_height = (
                    float(CAPTURE_RESOLVED_FLOATING_HEIGHT_M)
                    + float(config.actual_cad_ground_clearance_m)
                    - float(calibrated_foot_alignment["min_captured_bottom_z_m"])
                )
            actual_cad_foot_collision = {
                "source": "actual_cad_visual_bottom",
                "feet": actual_cad_foot_geometry["feet"],
                "min_visual_bottom_z_without_offset_m": actual_cad_foot_geometry[
                    "min_visual_bottom_z_without_offset_m"
                ],
                "ground_clearance_m": float(config.actual_cad_ground_clearance_m),
            }
            resolved_floating_height_m = max(resolved_floating_height_m, visual_clearance_height)
        elif foot_contact_mode == "actual_cad_grounded_support":
            actual_cad_visual_lift_m = (
                max(0.0, float(visual_clearance_height) - float(resolved_floating_height_m))
                if config.align_actual_cad_visual_bottom_to_ground
                else 0.0
            )
            actual_cad_foot_collision = actual_cad_grounded_support_feet(
                actual_cad_foot_geometry,
                float(config.foot_proxy_radius_m),
                float(config.actual_cad_ground_clearance_m),
                support_center_z_without_offset_m=(
                    float(config.foot_proxy_radius_m)
                    + float(config.actual_cad_ground_clearance_m)
                    - float(resolved_floating_height_m)
                ),
                rendered_visual_lift_m=actual_cad_visual_lift_m,
                align_to_rendered_visual_bottom=False,
            )
        elif config.align_actual_cad_visual_bottom_to_ground:
            resolved_floating_height_m = float(config.floating_height_m)
    world_offset = np.array(config.world_translation_m, dtype=np.float64) + np.array(
        [0.0, 0.0, resolved_floating_height_m], dtype=np.float64
    )
    visual_world_offset = world_offset
    if (
        actual_cad_foot_geometry is not None
        and config.align_actual_cad_visual_bottom_to_ground
        and (actual_cad_foot_collision is None or foot_contact_mode == "actual_cad_grounded_support")
    ):
        actual_cad_visual_lift_m = max(0.0, float(visual_clearance_height) - float(resolved_floating_height_m))

    ground_box = None
    if config.include_ground:
        ground_box = create_static_ground_box(
            stage,
            config.ground_prim_path,
            size_m=config.ground_size_m,
            thickness_m=config.ground_thickness_m,
        )

    hip_points = [
        np.array(spec["points"]["hip_origin"], dtype=np.float64) + world_offset
        for spec in DOMINO_FOUR_COMBINED_LEG_SPECS
    ]
    body_reference = create_body_from_points(
        stage,
        root,
        "body_reference",
        hip_points,
        width=0.030,
        mass=1.2,
        kinematic=bool(config.fixed_base),
        enable_gravity=config.enable_gravity,
        proxy_visible=proxy_visible,
        enable_collision=bool(config.enable_body_collisions),
    )
    shared_base = {"key": "body_reference", "body": body_reference}

    points = {}
    bodies = {"body_reference": body_reference}
    drives = []
    passive_stabilizers = []
    loop_checks = []
    joint_checks = []
    legs = []
    for spec in DOMINO_FOUR_COMBINED_LEG_SPECS:
        leg = build_domino_combined_leg_instance(
            stage,
            root,
            spec,
            shared_base,
            world_offset,
            config,
            actual_cad_foot_collision=actual_cad_foot_collision,
        )
        points.update(leg["points"])
        bodies.update(leg["bodies"])
        drives.extend(leg["drives"])
        passive_stabilizers.extend(leg.get("passive_stabilizers", []))
        joint_checks.extend(leg["joint_checks"])
        loop_checks.extend(leg["loop_checks"])
        if actual_cad_foot_geometry is not None:
            lower_closure_key = f"{spec['id']}_lower_closure"
            visual_foot = actual_cad_foot_geometry["feet"].get(lower_closure_key)
            if visual_foot is not None:
                leg["leg"]["actual_cad_visual_foot_bottom_m"] = (
                    np.asarray(visual_foot["visual_bottom_without_offset_m"], dtype=np.float64) + world_offset
                ).tolist()
        legs.append(leg["leg"])

    neutral_pose = {
        "applied": False,
        "reason": "requires passive finite closure with actual-CAD visual-bottom contacts",
    }
    if (
        bool(config.use_calibrated_neutral_pose)
        and closure_model == "passive"
        and foot_contact_mode == "actual_cad_visual_bottom"
        and actual_cad_foot_geometry is not None
    ):
        neutral_pose = apply_calibrated_neutral_body_poses(
            stage,
            bodies,
            config.world_translation_m,
            resolved_floating_height_m,
        )

    actual_cad_visuals = None
    if config.include_actual_cad_visuals:
        if mesh_dir is None:
            mesh_dir = Path(config.actual_cad_mesh_dir).expanduser() if config.actual_cad_mesh_dir else default_actual_cad_mesh_dir()
            mesh_dir = mesh_dir.resolve()
        actual_cad_visuals = attach_actual_cad_visuals(stage, bodies, visual_world_offset, mesh_dir)
        set_actual_cad_visual_local_lift(
            stage,
            actual_cad_visuals.get("corrector_paths", {}),
            actual_cad_visual_lift_m,
        )
    robot_collision_group = (
        create_robot_self_collision_filter(stage, root)
        if bool(config.enable_body_collisions)
        else None
    )
    visual_geometry_counts = domino_linkage_visual_geometry_counts(stage, root)
    target_height_m = float(body_reference["center"][2] - float(config.world_translation_m[2]))
    actual_cad_visual_alignment = None
    if actual_cad_foot_geometry is not None:
        if calibrated_foot_alignment is not None:
            min_visual_bottom_z_with_offset = (
                float(calibrated_foot_alignment["min_captured_bottom_z_m"])
                + float(resolved_floating_height_m)
                - float(CAPTURE_RESOLVED_FLOATING_HEIGHT_M)
                + float(actual_cad_visual_lift_m)
            )
            alignment_frame = "calibrated_neutral_pose"
        else:
            min_visual_bottom_z_with_offset = (
                float(actual_cad_foot_geometry["min_visual_bottom_z_without_offset_m"])
                + float(visual_world_offset[2])
                + float(actual_cad_visual_lift_m)
            )
            alignment_frame = "unrotated_export_pose"
        visual_bottom_aligned = (
            abs(min_visual_bottom_z_with_offset - float(config.actual_cad_ground_clearance_m)) <= 1.0e-4
        )
        actual_cad_visual_alignment = {
            "visual_bottom_aligned_to_ground": bool(visual_bottom_aligned),
            "alignment_frame": alignment_frame,
            "visual_z_lift_m": round(float(actual_cad_visual_lift_m), 6),
            "ground_clearance_m": float(config.actual_cad_ground_clearance_m),
            "min_visual_bottom_z_without_offset_m": round(
                float(actual_cad_foot_geometry["min_visual_bottom_z_without_offset_m"]), 6
            ),
            "min_visual_bottom_z_with_offset_m": round(min_visual_bottom_z_with_offset, 6),
            "visual_lift_mode": (
                "dynamic_global_z_via_per_body_local_xform"
                if float(actual_cad_visual_lift_m) > 0.0
                else "none"
            ),
        }

    action_names = [drive["action_name"] for drive in drives]
    validate_action_layout(action_names)
    if len(drives) != EXPECTED_ACTION_COUNT:
        raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} Domino actuator drives, found {len(drives)}.")
    attach_drive_target_attrs(drives)
    neutral_target_mismatches = []
    for drive in drives:
        neutral_target = target_from_normalized_action(drive, 0.0, 1.0)
        if abs(neutral_target - float(drive["center_deg"])) > 1.0e-6:
            neutral_target_mismatches.append(
                f"{drive['action_name']}: action-zero={neutral_target:.6f}, center={float(drive['center_deg']):.6f}"
            )
    if neutral_target_mismatches:
        raise RuntimeError(
            "Domino normalized action zero does not preserve the authored neutral servo centers: "
            + "; ".join(neutral_target_mismatches)
        )

    linkage = {
        "geometry": "domino-four-12-floating-body",
        "fixed_base": bool(config.fixed_base),
        "closure_model": closure_model,
        "visual_fidelity": ACTUAL_CAD_STL_VISUAL_FIDELITY if actual_cad_visuals else PROXY_VISUAL_FIDELITY,
        "actual_cad_visual": bool(actual_cad_visuals),
        "cad_source": {
            "linkage_pivots": LINKAGE_PIVOT_SOURCE,
            "visual_mesh_usd": ACTUAL_CAD_VISUAL_USD,
            "visual_mesh_stl": ACTUAL_CAD_STL_SOURCE,
            "visual_mesh_status": "attached_to_proxy_rigid_bodies" if actual_cad_visuals else "not_attached_to_proxy_rigid_bodies",
            "physics_status": "proxy_rigid_bodies_with_cad_derived_pin_constraints",
        },
        "actual_cad_visuals": actual_cad_visuals,
        "actual_cad_visual_lift_m": float(actual_cad_visual_lift_m),
        "actual_cad_visual_corrector_paths": (
            actual_cad_visuals.get("corrector_paths", {}) if isinstance(actual_cad_visuals, dict) else {}
        ),
        "visual_geometry_counts": visual_geometry_counts,
        "actual_cad_visual_alignment": actual_cad_visual_alignment,
        "body_ground_collisions": {
            "enabled": bool(config.enable_body_collisions),
            "collision_body_count": int(
                sum(bool(body.get("ground_collision_path")) for body in bodies.values())
            ),
            "robot_collision_group": robot_collision_group,
            "self_collision_filtered": bool(robot_collision_group),
        },
        "neutral_pose": neutral_pose,
        "resolved_floating_height_m": resolved_floating_height_m,
        "foot_contact_mode": foot_contact_mode,
        "target_height_m": target_height_m,
        "actual_cad_foot_collision": (
            {
                "enabled": True,
                "contact_model_revision": "cad_fitted_distal_sphere_v2",
                "source": str(actual_cad_foot_collision.get("source", "actual_cad_visual_bottom")),
                "ground_clearance_m": float(config.actual_cad_ground_clearance_m),
                "support_center_z_without_offset_m": (
                    round(float(actual_cad_foot_collision["support_center_z_without_offset_m"]), 6)
                    if "support_center_z_without_offset_m" in actual_cad_foot_collision
                    else None
                ),
                "min_visual_bottom_z_without_offset_m": round(
                    float(actual_cad_foot_collision["min_visual_bottom_z_without_offset_m"]), 6
                ),
                "feet": {
                    body_key: {
                        "visual_bottom_m": [
                            round(float(value), 6)
                            for value in (
                                np.asarray(details["visual_bottom_without_offset_m"], dtype=np.float64) + world_offset
                            )
                        ],
                        "sphere_center_m": [
                            round(float(value), 6)
                            for value in (
                                np.asarray(details["sphere_center_without_offset_m"], dtype=np.float64) + world_offset
                            )
                        ],
                        "radius_m": round(float(details.get("radius_m", config.foot_proxy_radius_m)), 6),
                        "bottom_sample_count": int(details["bottom_sample_count"]),
                        "fit_source": str(details.get("fit_source", "")),
                        "fit_shell_residual_p95_m": round(
                            float(details.get("fit_shell_residual_p95_m", 0.0)),
                            8,
                        ),
                        "fit_shell_residual_max_m": round(
                            float(details.get("fit_shell_residual_max_m", 0.0)),
                            8,
                        ),
                    }
                    for body_key, details in (actual_cad_foot_collision or {}).get("feet", {}).items()
                },
            }
            if actual_cad_foot_collision is not None
            else {"enabled": False, "source": "linkage_lower_closure"}
        ),
        "actuator_model": {
            **SERVO_ACTUATOR_MODEL,
            "drive_count": len(drives),
            "validated_initial_policy_action_scale_deg": VALIDATED_INITIAL_POLICY_ACTION_SCALE_DEG,
        },
        "contact_material": {
            "path": DOMINO_CONTACT_MATERIAL_PATH,
            "static_friction": DOMINO_CONTACT_STATIC_FRICTION,
            "dynamic_friction": DOMINO_CONTACT_DYNAMIC_FRICTION,
            "restitution": DOMINO_CONTACT_RESTITUTION,
            "tpu_foot_tips": {
                "path": DOMINO_TPU_FOOT_MATERIAL_PATH,
                "model": "tunable high-grip TPU approximation",
                "sphere_count": len(legs),
                "static_friction": DOMINO_TPU_FOOT_STATIC_FRICTION,
                "dynamic_friction": DOMINO_TPU_FOOT_DYNAMIC_FRICTION,
                "restitution": DOMINO_TPU_FOOT_RESTITUTION,
                "friction_combine_mode": DOMINO_TPU_FOOT_FRICTION_COMBINE_MODE,
                "restitution_combine_mode": DOMINO_TPU_FOOT_RESTITUTION_COMBINE_MODE,
            },
        },
        "passive_stabilizers": passive_stabilizers,
        "root_prim_path": root,
        "ground_box": ground_box,
        "action_contract": {
            "expected_action_count": EXPECTED_ACTION_COUNT,
            "action_names": ACTION_JOINT_NAMES,
            "action_group_counts": action_group_counts(),
            "per_leg_action_layout": per_leg_action_layout(),
        },
        "drive": {
            "joint": drives[0]["joint"],
            "target_center_deg": drives[0]["center_deg"],
        },
        "drive_joint_name": drives[0]["joint"],
        "drive_center_deg": drives[0]["center_deg"],
        "drives": drives,
        "passive_stabilizers": passive_stabilizers,
        "joint_checks": joint_checks,
        "points": points,
        "bodies": bodies,
        "loop_checks": loop_checks,
        "legs": legs,
    }
    if actual_cad_visuals:
        linkage["visual_geometry_counts"] = validate_domino_actual_cad_visuals(
            stage,
            linkage,
            require_hidden_proxy=bool(config.hide_proxy_visuals_when_actual_cad),
        )
    return linkage
