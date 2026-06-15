"""Run a minimal actuated four-bar linkage in Isaac/PhysX.

This is a linkage physics smoke test. It authors rigid bodies and revolute
pin joints directly into USD, drives one crank joint, and checks that the
closed-loop constraint remains finite over a short headless simulation.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import traceback

import numpy as np

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Run a minimal actuated pin-linkage test in Isaac/PhysX.")
parser.add_argument("--steps", type=int, default=240, help="Number of physics steps to run.")
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
parser.add_argument("--save-usd", default="", help="Optional path to save the generated USD stage.")
parser.add_argument(
    "--geometry",
    choices=("generic-four-bar", "domino-lower-triangle", "domino-upper-loop", "domino-combined-leg"),
    default="generic-four-bar",
    help="Linkage geometry to author into the Isaac stage.",
)
parser.add_argument(
    "--drive-center-deg",
    type=float,
    default=None,
    help="Driven joint target center. Defaults are geometry-specific.",
)
parser.add_argument("--drive-amplitude-deg", type=float, default=12.0, help="Driven crank target amplitude.")
parser.add_argument("--drive-frequency-hz", type=float, default=0.6, help="Driven crank target frequency.")
parser.add_argument(
    "--secondary-drive-amplitude-deg",
    type=float,
    default=None,
    help="Secondary driven joint amplitude for multi-drive geometries.",
)
parser.add_argument(
    "--secondary-drive-frequency-hz",
    type=float,
    default=None,
    help="Secondary driven joint frequency for multi-drive geometries.",
)
parser.add_argument(
    "--graceful-close",
    action="store_true",
    help="Call SimulationApp.close() before exit. Disabled by default because it can hang on some Windows setups.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleRigidPrim  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics  # noqa: E402


def vec3(values: tuple[float, float, float]) -> Gf.Vec3f:
    return Gf.Vec3f(float(values[0]), float(values[1]), float(values[2]))


def circle_intersection(
    center_a: np.ndarray, radius_a: float, center_b: np.ndarray, radius_b: float, prefer_negative_z: bool
) -> np.ndarray:
    delta = center_b - center_a
    distance = float(np.linalg.norm(delta))
    if distance <= 0.0:
        raise ValueError("Circle centers must be separated.")
    reach_min = abs(radius_a - radius_b)
    reach_max = radius_a + radius_b
    if distance < reach_min or distance > reach_max:
        raise ValueError("Invalid linkage dimensions; circles do not intersect.")

    along = (radius_a * radius_a - radius_b * radius_b + distance * distance) / (2.0 * distance)
    height_sq = max(radius_a * radius_a - along * along, 0.0)
    height = math.sqrt(height_sq)
    unit = delta / distance
    perp = np.array([-unit[1], unit[0]], dtype=np.float64)
    base = center_a + along * unit
    candidates = [base + height * perp, base - height * perp]
    if prefer_negative_z:
        return min(candidates, key=lambda point: point[1])
    return max(candidates, key=lambda point: point[1])


def bar_angle_deg(start: np.ndarray, end: np.ndarray) -> float:
    delta = end - start
    # Visual-only rotation: local X approximately follows the X/Z bar direction.
    return -math.degrees(math.atan2(float(delta[1]), float(delta[0])))


def local_endpoint(point: np.ndarray, center: np.ndarray) -> Gf.Vec3f:
    point = np.asarray(point, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64)
    if point.shape[0] == 2:
        return Gf.Vec3f(float(point[0] - center[0]), 0.0, float(point[1] - center[1]))
    return Gf.Vec3f(float(point[0] - center[0]), float(point[1] - center[1]), float(point[2] - center[2]))


def apply_rigid_body(prim, mass: float, kinematic: bool = False):
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_api.CreateRigidBodyEnabledAttr(True)
    rigid_api.CreateKinematicEnabledAttr(kinematic)

    physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    physx_api.GetDisableGravityAttr().Set(True)
    physx_api.CreateLinearDampingAttr().Set(0.02)
    physx_api.CreateAngularDampingAttr().Set(0.02)

    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(float(mass))
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(0.0002, 0.0002, 0.0002))


def create_bar(stage, root: str, name: str, start: np.ndarray, end: np.ndarray, width: float, mass: float, kinematic=False):
    center = 0.5 * (start + end)
    length = float(np.linalg.norm(end - start))
    body_path = f"{root}/{name}"
    body = UsdGeom.Xform.Define(stage, body_path)
    UsdGeom.XformCommonAPI(body).SetTranslate(Gf.Vec3d(float(center[0]), 0.0, float(center[1])))
    apply_rigid_body(body.GetPrim(), mass=mass, kinematic=kinematic)

    visual = UsdGeom.Cube.Define(stage, f"{body_path}/visual")
    visual.CreateSizeAttr(1.0)
    visual_xform = UsdGeom.XformCommonAPI(visual)
    visual_xform.SetRotate(Gf.Vec3f(0.0, float(bar_angle_deg(start, end)), 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    visual_xform.SetScale(Gf.Vec3f(length, width, width))
    return {
        "path": Sdf.Path(body_path),
        "center": center,
        "start": start,
        "end": end,
        "length": length,
    }


def create_body_from_points(
    stage,
    root: str,
    name: str,
    points: list[np.ndarray],
    width: float,
    mass: float,
    kinematic=False,
):
    point_array = np.vstack([np.asarray(point, dtype=np.float64) for point in points])
    center = point_array.mean(axis=0)
    extents = point_array.max(axis=0) - point_array.min(axis=0)
    visual_scale = np.maximum(extents, np.array([width, width, width], dtype=np.float64))

    body_path = f"{root}/{name}"
    body = UsdGeom.Xform.Define(stage, body_path)
    UsdGeom.XformCommonAPI(body).SetTranslate(Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
    apply_rigid_body(body.GetPrim(), mass=mass, kinematic=kinematic)

    visual = UsdGeom.Cube.Define(stage, f"{body_path}/visual")
    visual.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(visual).SetScale(
        Gf.Vec3f(float(visual_scale[0]), float(visual_scale[1]), float(visual_scale[2]))
    )
    return {
        "path": Sdf.Path(body_path),
        "center": center,
        "points": point_array,
    }


def create_pin_joint(
    stage,
    path: str,
    body0: dict,
    body1: dict,
    pivot: np.ndarray,
    lower_deg: float | None = None,
    upper_deg: float | None = None,
):
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0["path"]])
    joint.CreateBody1Rel().SetTargets([body1["path"]])
    joint.CreateLocalPos0Attr().Set(local_endpoint(pivot, body0["center"]))
    joint.CreateLocalPos1Attr().Set(local_endpoint(pivot, body1["center"]))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateAxisAttr("Y")
    if lower_deg is not None:
        joint.CreateLowerLimitAttr(float(lower_deg))
    if upper_deg is not None:
        joint.CreateUpperLimitAttr(float(upper_deg))
    return joint


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
    amplitude_source: str = "primary",
    frequency_source: str = "primary",
    phase_deg: float = 0.0,
):
    return {
        "joint": joint,
        "drive": drive,
        "center_deg": float(center_deg),
        "amplitude_source": amplitude_source,
        "frequency_source": frequency_source,
        "phase_rad": math.radians(float(phase_deg)),
        "phase_deg": float(phase_deg),
    }


def world_endpoint(view: SingleRigidPrim, local_point: Gf.Vec3f) -> np.ndarray:
    position, orientation = view.get_world_pose()
    position = to_numpy(position)
    orientation = to_numpy(orientation)
    # Quaternion is scalar-first for Isaac Core views.
    w, x, y, z = [float(v) for v in orientation]
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    local = np.array([float(local_point[0]), float(local_point[1]), float(local_point[2])], dtype=np.float64)
    return position.astype(np.float64) + rotation @ local


def build_generic_four_bar(stage):
    root = "/World/PinLinkage"
    UsdGeom.Xform.Define(stage, root)

    # Four-bar pivots in the X/Z plane. The dimensions are generic and stable,
    # chosen to prove the constraint pattern before copying Domino CAD pivots.
    ground_len = 0.110
    crank_len = 0.045
    coupler_len = 0.115
    rocker_len = 0.095
    initial_crank_angle = math.radians(-55.0)

    pivot_o = np.array([0.0, 0.0], dtype=np.float64)
    pivot_d = np.array([ground_len, 0.0], dtype=np.float64)
    pivot_b = pivot_o + crank_len * np.array([math.cos(initial_crank_angle), math.sin(initial_crank_angle)])
    pivot_c = circle_intersection(pivot_b, coupler_len, pivot_d, rocker_len, prefer_negative_z=True)

    ground = create_bar(stage, root, "ground", pivot_o, pivot_d, width=0.012, mass=1.0, kinematic=True)
    crank = create_bar(stage, root, "crank", pivot_o, pivot_b, width=0.010, mass=0.05)
    coupler = create_bar(stage, root, "coupler", pivot_b, pivot_c, width=0.008, mass=0.04)
    rocker = create_bar(stage, root, "rocker", pivot_d, pivot_c, width=0.010, mass=0.05)

    drive_joint = create_pin_joint(stage, f"{root}/joints/drive_crank_pin", ground, crank, pivot_o, -75.0, 15.0)
    create_pin_joint(stage, f"{root}/joints/crank_coupler_pin", crank, coupler, pivot_b)
    create_pin_joint(stage, f"{root}/joints/ground_rocker_pin", ground, rocker, pivot_d, -120.0, 40.0)
    create_pin_joint(stage, f"{root}/joints/loop_closure_pin", coupler, rocker, pivot_c)
    drive = apply_angular_drive(drive_joint, stiffness=8.0, damping=0.8, max_force=2.0, target_deg=-55.0)

    return {
        "geometry": "generic-four-bar",
        "drive": drive,
        "drive_joint_name": "drive_crank_pin",
        "drive_center_deg": -55.0,
        "drives": [make_drive_spec("drive_crank_pin", drive, -55.0)],
        "points": {
            "O": pivot_o.tolist(),
            "B": pivot_b.tolist(),
            "C": pivot_c.tolist(),
            "D": pivot_d.tolist(),
        },
        "bodies": {
            "ground": ground,
            "crank": crank,
            "coupler": coupler,
            "rocker": rocker,
        },
        "loop_checks": [
            {
                "name": "loop_closure_pin",
                "body_a": "coupler",
                "body_b": "rocker",
                "pivot": pivot_c.tolist(),
            }
        ],
    }


def build_domino_lower_triangle(stage):
    root = "/World/DominoLowerTriangle"
    UsdGeom.Xform.Define(stage, root)

    # CAD-derived pivots from simulation/isaac/reports/domino-linkage-pivots.md.
    # This loop uses one driven input pin and three passive pins:
    # R59 -> R43 -> R33 -> R26/R25.
    points = {
        "drive_revolute_59": np.array([0.323000, -0.028000, -0.010500], dtype=np.float64),
        "passive_revolute_43": np.array([0.323000, -0.036000, -0.010500], dtype=np.float64),
        "passive_revolute_33": np.array([0.294708, -0.035600, 0.017777], dtype=np.float64),
        "closure_revolute_25_26": np.array([0.182024, -0.048100, -0.095615], dtype=np.float64),
        "upper_reference_revolute_58": np.array([0.347000, -0.028000, 0.010500], dtype=np.float64),
    }

    ground = create_body_from_points(
        stage,
        root,
        "ground_hip_reference",
        [points["drive_revolute_59"], points["upper_reference_revolute_58"]],
        width=0.014,
        mass=1.0,
        kinematic=True,
    )
    lower_driver = create_body_from_points(
        stage,
        root,
        "lower_driver_dom_p_5_1",
        [points["drive_revolute_59"], points["passive_revolute_43"], points["closure_revolute_25_26"]],
        width=0.010,
        mass=0.08,
    )
    coupler = create_body_from_points(
        stage,
        root,
        "coupler_dom_p_1",
        [points["passive_revolute_43"], points["passive_revolute_33"]],
        width=0.008,
        mass=0.04,
    )
    diagonal = create_body_from_points(
        stage,
        root,
        "diagonal_dom_p_2_1",
        [points["passive_revolute_33"], points["closure_revolute_25_26"]],
        width=0.008,
        mass=0.04,
    )

    drive_joint = create_pin_joint(
        stage,
        f"{root}/joints/drive_revolute_59",
        ground,
        lower_driver,
        points["drive_revolute_59"],
        lower_deg=-120.0,
        upper_deg=0.0,
    )
    create_pin_joint(
        stage,
        f"{root}/joints/passive_revolute_43",
        lower_driver,
        coupler,
        points["passive_revolute_43"],
    )
    create_pin_joint(
        stage,
        f"{root}/joints/passive_revolute_33",
        coupler,
        diagonal,
        points["passive_revolute_33"],
    )
    create_pin_joint(
        stage,
        f"{root}/joints/loop_closure_revolute_25_26",
        lower_driver,
        diagonal,
        points["closure_revolute_25_26"],
    )
    drive = apply_angular_drive(drive_joint, stiffness=3.0, damping=0.45, max_force=1.0, target_deg=-15.0)

    return {
        "geometry": "domino-lower-triangle",
        "drive": drive,
        "drive_joint_name": "drive_revolute_59",
        "drive_center_deg": -15.0,
        "drives": [make_drive_spec("drive_revolute_59", drive, -15.0)],
        "points": {name: point.tolist() for name, point in points.items()},
        "bodies": {
            "ground": ground,
            "lower_driver": lower_driver,
            "coupler": coupler,
            "diagonal": diagonal,
        },
        "loop_checks": [
            {
                "name": "revolute_25_26_closure",
                "body_a": "lower_driver",
                "body_b": "diagonal",
                "pivot": points["closure_revolute_25_26"].tolist(),
            }
        ],
    }


def build_domino_upper_loop(stage):
    root = "/World/DominoUpperLoop"
    UsdGeom.Xform.Define(stage, root)

    # CAD-derived pivots from simulation/isaac/reports/domino-linkage-pivots.md.
    # This isolates the second DOM_P__4__1 loop:
    # R58 driven, R43 held by the lower input at rest, R32 passive, R51 closure.
    points = {
        "drive_revolute_58": np.array([0.347000, -0.028000, 0.010500], dtype=np.float64),
        "lower_input_revolute_59": np.array([0.323000, -0.028000, -0.010500], dtype=np.float64),
        "passive_revolute_43": np.array([0.323000, -0.036000, -0.010500], dtype=np.float64),
        "passive_revolute_32": np.array([0.336647, -0.035600, 0.049137], dtype=np.float64),
        "closure_revolute_51": np.array([0.336647, -0.035600, 0.049137], dtype=np.float64),
    }

    ground = create_body_from_points(
        stage,
        root,
        "ground_hip_reference",
        [points["drive_revolute_58"], points["lower_input_revolute_59"]],
        width=0.014,
        mass=1.0,
        kinematic=True,
    )
    lower_reference = create_body_from_points(
        stage,
        root,
        "held_lower_input_dom_p_5_1",
        [points["lower_input_revolute_59"], points["passive_revolute_43"]],
        width=0.010,
        mass=0.08,
        kinematic=True,
    )
    upper_driver = create_body_from_points(
        stage,
        root,
        "upper_driver_dom_p_6_1",
        [points["drive_revolute_58"], points["closure_revolute_51"]],
        width=0.010,
        mass=0.06,
    )
    coupler = create_body_from_points(
        stage,
        root,
        "coupler_dom_p_1",
        [points["passive_revolute_43"], points["passive_revolute_32"]],
        width=0.008,
        mass=0.04,
    )

    drive_joint = create_pin_joint(
        stage,
        f"{root}/joints/drive_revolute_58",
        ground,
        upper_driver,
        points["drive_revolute_58"],
        lower_deg=-30.0,
        upper_deg=60.0,
    )
    create_pin_joint(
        stage,
        f"{root}/joints/passive_revolute_43",
        lower_reference,
        coupler,
        points["passive_revolute_43"],
    )
    create_pin_joint(
        stage,
        f"{root}/joints/loop_closure_revolute_32_51",
        coupler,
        upper_driver,
        points["passive_revolute_32"],
    )
    drive = apply_angular_drive(drive_joint, stiffness=2.0, damping=0.35, max_force=0.8, target_deg=0.0)

    return {
        "geometry": "domino-upper-loop",
        "drive": drive,
        "drive_joint_name": "drive_revolute_58",
        "drive_center_deg": 0.0,
        "drives": [make_drive_spec("drive_revolute_58", drive, 0.0)],
        "points": {name: point.tolist() for name, point in points.items()},
        "bodies": {
            "ground": ground,
            "lower_reference": lower_reference,
            "upper_driver": upper_driver,
            "coupler": coupler,
        },
        "loop_checks": [
            {
                "name": "revolute_32_51_closure",
                "body_a": "coupler",
                "body_b": "upper_driver",
                "pivot": points["closure_revolute_51"].tolist(),
            }
        ],
    }


def build_domino_combined_leg(stage):
    root = "/World/DominoCombinedLeg"
    UsdGeom.Xform.Define(stage, root)

    # CAD-derived pivots from the DOM_P__4__1 leg cluster. This combines the
    # lower triangle and upper loop so both driven pitch inputs share DOM_P_1.
    points = {
        "upper_drive_revolute_58": np.array([0.347000, -0.028000, 0.010500], dtype=np.float64),
        "lower_drive_revolute_59": np.array([0.323000, -0.028000, -0.010500], dtype=np.float64),
        "passive_revolute_43": np.array([0.323000, -0.036000, -0.010500], dtype=np.float64),
        "passive_revolute_33": np.array([0.294708, -0.035600, 0.017777], dtype=np.float64),
        "upper_passive_revolute_32": np.array([0.336647, -0.035600, 0.049137], dtype=np.float64),
        "upper_closure_revolute_51": np.array([0.336647, -0.035600, 0.049137], dtype=np.float64),
        "lower_closure_revolute_25_26": np.array([0.182024, -0.048100, -0.095615], dtype=np.float64),
    }

    ground = create_body_from_points(
        stage,
        root,
        "ground_hip_reference",
        [points["upper_drive_revolute_58"], points["lower_drive_revolute_59"]],
        width=0.014,
        mass=1.0,
        kinematic=True,
    )
    lower_driver = create_body_from_points(
        stage,
        root,
        "lower_driver_dom_p_5_1",
        [points["lower_drive_revolute_59"], points["passive_revolute_43"], points["lower_closure_revolute_25_26"]],
        width=0.010,
        mass=0.08,
    )
    coupler = create_body_from_points(
        stage,
        root,
        "shared_coupler_dom_p_1",
        [points["passive_revolute_43"], points["passive_revolute_33"], points["upper_passive_revolute_32"]],
        width=0.008,
        mass=0.05,
    )
    lower_diagonal = create_body_from_points(
        stage,
        root,
        "lower_diagonal_dom_p_2_1",
        [points["passive_revolute_33"], points["lower_closure_revolute_25_26"]],
        width=0.008,
        mass=0.04,
    )
    upper_driver = create_body_from_points(
        stage,
        root,
        "upper_driver_dom_p_6_1",
        [points["upper_drive_revolute_58"], points["upper_closure_revolute_51"]],
        width=0.010,
        mass=0.06,
    )

    lower_drive_joint = create_pin_joint(
        stage,
        f"{root}/joints/lower_drive_revolute_59",
        ground,
        lower_driver,
        points["lower_drive_revolute_59"],
        lower_deg=-120.0,
        upper_deg=0.0,
    )
    create_pin_joint(
        stage,
        f"{root}/joints/passive_revolute_43",
        lower_driver,
        coupler,
        points["passive_revolute_43"],
    )
    create_pin_joint(
        stage,
        f"{root}/joints/passive_revolute_33",
        coupler,
        lower_diagonal,
        points["passive_revolute_33"],
    )
    create_pin_joint(
        stage,
        f"{root}/joints/lower_loop_closure_revolute_25_26",
        lower_driver,
        lower_diagonal,
        points["lower_closure_revolute_25_26"],
    )

    upper_drive_joint = create_pin_joint(
        stage,
        f"{root}/joints/upper_drive_revolute_58",
        ground,
        upper_driver,
        points["upper_drive_revolute_58"],
        lower_deg=-30.0,
        upper_deg=60.0,
    )
    create_pin_joint(
        stage,
        f"{root}/joints/upper_loop_closure_revolute_32_51",
        coupler,
        upper_driver,
        points["upper_passive_revolute_32"],
    )

    lower_drive = apply_angular_drive(
        lower_drive_joint, stiffness=1.4, damping=0.35, max_force=0.7, target_deg=-15.0
    )
    upper_drive = apply_angular_drive(upper_drive_joint, stiffness=1.2, damping=0.30, max_force=0.6, target_deg=0.0)

    return {
        "geometry": "domino-combined-leg",
        "drive": lower_drive,
        "drive_joint_name": "lower_drive_revolute_59",
        "drive_center_deg": -15.0,
        "drives": [
            make_drive_spec("lower_drive_revolute_59", lower_drive, -15.0, amplitude_source="primary"),
            make_drive_spec(
                "upper_drive_revolute_58",
                upper_drive,
                0.0,
                amplitude_source="secondary",
                frequency_source="secondary",
                phase_deg=90.0,
            ),
        ],
        "points": {name: point.tolist() for name, point in points.items()},
        "bodies": {
            "ground": ground,
            "lower_driver": lower_driver,
            "coupler": coupler,
            "lower_diagonal": lower_diagonal,
            "upper_driver": upper_driver,
        },
        "loop_checks": [
            {
                "name": "lower_revolute_25_26_closure",
                "body_a": "lower_driver",
                "body_b": "lower_diagonal",
                "pivot": points["lower_closure_revolute_25_26"].tolist(),
            },
            {
                "name": "upper_revolute_32_51_closure",
                "body_a": "coupler",
                "body_b": "upper_driver",
                "pivot": points["upper_closure_revolute_51"].tolist(),
            },
        ],
    }


def build_linkage(stage):
    if args_cli.geometry == "domino-combined-leg":
        return build_domino_combined_leg(stage)
    if args_cli.geometry == "domino-upper-loop":
        return build_domino_upper_loop(stage)
    if args_cli.geometry == "domino-lower-triangle":
        return build_domino_lower_triangle(stage)
    return build_generic_four_bar(stage)


def to_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def tensor_list(value) -> list[float]:
    return [round(float(v), 6) for v in to_numpy(value).flatten()]


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args_cli.device)
    sim_cfg.physx.solver_type = 1
    sim_cfg.physx.min_position_iteration_count = 8
    sim_cfg.physx.max_position_iteration_count = 16
    sim_cfg.physx.min_velocity_iteration_count = 2
    sim_cfg.physx.max_velocity_iteration_count = 8
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([0.28, -0.34, 0.22], [0.055, 0.0, -0.03])

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    linkage = build_linkage(stage)
    if args_cli.save_usd:
        save_path = Path(args_cli.save_usd).expanduser().resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(save_path))

    views = {
        name: SingleRigidPrim(str(body["path"]), name=f"{name}_view", reset_xform_properties=False)
        for name, body in linkage["bodies"].items()
    }

    sim.reset()
    for view in views.values():
        view.initialize()

    drive_specs = linkage.get("drives", [make_drive_spec(linkage["drive_joint_name"], linkage["drive"], linkage["drive_center_deg"])])
    for spec in drive_specs:
        spec["target_attr"] = spec["drive"].GetTargetPositionAttr()
    sim_dt = sim.get_physics_dt()
    max_linear_speed = 0.0
    max_loop_errors = {check["name"]: 0.0 for check in linkage["loop_checks"]}
    min_finite = True
    primary_center = float(args_cli.drive_center_deg if args_cli.drive_center_deg is not None else linkage["drive_center_deg"])
    secondary_amplitude = float(
        args_cli.secondary_drive_amplitude_deg
        if args_cli.secondary_drive_amplitude_deg is not None
        else args_cli.drive_amplitude_deg
    )
    secondary_frequency = float(
        args_cli.secondary_drive_frequency_hz
        if args_cli.secondary_drive_frequency_hz is not None
        else args_cli.drive_frequency_hz
    )

    for step in range(args_cli.steps):
        time_s = step * sim_dt
        for spec in drive_specs:
            center = primary_center if spec["amplitude_source"] == "primary" else spec["center_deg"]
            amplitude = args_cli.drive_amplitude_deg if spec["amplitude_source"] == "primary" else secondary_amplitude
            frequency = args_cli.drive_frequency_hz if spec["frequency_source"] == "primary" else secondary_frequency
            target = center + amplitude * math.sin((2.0 * math.pi * frequency * time_s) + spec["phase_rad"])
            spec["target_attr"].Set(float(target))

        sim.step()

        positions = []
        velocities = []
        for view in views.values():
            pos, quat = view.get_world_pose()
            lin_vel = view.get_linear_velocity()
            positions.extend(to_numpy(pos).flatten().tolist())
            positions.extend(to_numpy(quat).flatten().tolist())
            velocities.extend(to_numpy(lin_vel).flatten().tolist())
        if not np.isfinite(np.array(positions + velocities, dtype=np.float64)).all():
            min_finite = False
            break

        for check in linkage["loop_checks"]:
            pivot = np.array(check["pivot"], dtype=np.float64)
            body_a = linkage["bodies"][check["body_a"]]
            body_b = linkage["bodies"][check["body_b"]]
            world_a = world_endpoint(views[check["body_a"]], local_endpoint(pivot, body_a["center"]))
            world_b = world_endpoint(views[check["body_b"]], local_endpoint(pivot, body_b["center"]))
            loop_error = float(np.linalg.norm(world_a - world_b))
            max_loop_errors[check["name"]] = max(max_loop_errors[check["name"]], loop_error)

        max_linear_speed = max(
            max_linear_speed, max(float(np.linalg.norm(to_numpy(view.get_linear_velocity()))) for view in views.values())
        )

    final_poses = {}
    for name, view in views.items():
        pos, quat = view.get_world_pose()
        final_poses[name] = {"position_m": tensor_list(pos), "orientation_wxyz": tensor_list(quat)}

    report = {
        "status": "passed" if min_finite else "failed",
        "geometry": linkage["geometry"],
        "steps": step + 1,
        "physics_dt": sim_dt,
        "linkage_points_m": linkage["points"],
        "drive": {
            "joint": linkage["drive_joint_name"],
            "target_center_deg": primary_center,
            "target_amplitude_deg": args_cli.drive_amplitude_deg,
            "frequency_hz": args_cli.drive_frequency_hz,
        },
        "drives": [
            {
                "joint": spec["joint"],
                "target_center_deg": primary_center if spec["amplitude_source"] == "primary" else spec["center_deg"],
                "target_amplitude_deg": args_cli.drive_amplitude_deg
                if spec["amplitude_source"] == "primary"
                else secondary_amplitude,
                "frequency_hz": args_cli.drive_frequency_hz
                if spec["frequency_source"] == "primary"
                else secondary_frequency,
                "phase_deg": spec["phase_deg"],
            }
            for spec in drive_specs
        ],
        "max_loop_closure_error_m": round(max(max_loop_errors.values()), 8),
        "loop_closure_errors_m": {name: round(value, 8) for name, value in max_loop_errors.items()},
        "max_body_linear_speed_m_s": round(max_linear_speed, 6),
        "final_poses": final_poses,
    }

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)
    if report["status"] != "passed":
        raise RuntimeError("Pin-linkage test produced non-finite state.")


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except Exception:
        exit_code = 1
        traceback.print_exc()
    finally:
        if args_cli.graceful_close:
            simulation_app.close(wait_for_replicator=False, skip_cleanup=True)
        os._exit(exit_code)
