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
parser.add_argument("--no-print-report", action="store_true", help="Write the JSON report without printing it to stdout.")
parser.add_argument(
    "--geometry",
    choices=(
        "generic-four-bar",
        "domino-lower-triangle",
        "domino-upper-loop",
        "domino-combined-leg",
        "domino-four-combined-legs",
        "domino-four-12-actuators",
        "domino-four-12-fixed-body",
    ),
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
    "--fit-start-step",
    type=int,
    default=0,
    help="First physics step to include in the linear calibration fit.",
)
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
    "--shoulder-drive-amplitude-deg",
    type=float,
    default=None,
    help="Shoulder hip ab/ad target amplitude for twelve-actuator Domino geometries.",
)
parser.add_argument(
    "--shoulder-drive-frequency-hz",
    type=float,
    default=None,
    help="Shoulder hip ab/ad target frequency for twelve-actuator Domino geometries.",
)
parser.add_argument(
    "--drive-schedule",
    choices=("phased-sine", "independent"),
    default="phased-sine",
    help="Drive all inputs with their phase offsets, or sweep one drive at a time for calibration.",
)
parser.add_argument(
    "--independent-segment-steps",
    type=int,
    default=160,
    help="Physics steps per active drive when --drive-schedule independent is used.",
)
parser.add_argument(
    "--independent-settle-steps",
    type=int,
    default=20,
    help="Initial steps inside each independent segment to exclude from the linear calibration fit.",
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
    axis: str = "Y",
):
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0["path"]])
    joint.CreateBody1Rel().SetTargets([body1["path"]])
    joint.CreateLocalPos0Attr().Set(local_endpoint(pivot, body0["center"]))
    joint.CreateLocalPos1Attr().Set(local_endpoint(pivot, body1["center"]))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateAxisAttr(axis)
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
    role: str = "drive",
    axis: str = "Y",
):
    return {
        "joint": joint,
        "drive": drive,
        "center_deg": float(center_deg),
        "amplitude_source": amplitude_source,
        "frequency_source": frequency_source,
        "phase_rad": math.radians(float(phase_deg)),
        "phase_deg": float(phase_deg),
        "role": role,
        "axis": axis,
    }


def drive_center_deg(spec: dict, primary_center_override: float | None) -> float:
    if spec["amplitude_source"] == "primary" and primary_center_override is not None:
        return float(primary_center_override)
    return float(spec["center_deg"])


def drive_amplitude_deg(
    spec: dict,
    primary_amplitude: float,
    secondary_amplitude: float,
    shoulder_amplitude: float,
) -> float:
    source = spec["amplitude_source"]
    if source == "primary":
        return float(primary_amplitude)
    if source == "shoulder":
        return float(shoulder_amplitude)
    return float(secondary_amplitude)


def drive_frequency_hz(spec: dict, primary_frequency: float, secondary_frequency: float, shoulder_frequency: float) -> float:
    source = spec["frequency_source"]
    if source == "primary":
        return float(primary_frequency)
    if source == "shoulder":
        return float(shoulder_frequency)
    return float(secondary_frequency)


def drive_target_deg(
    spec: dict,
    time_s: float,
    primary_center: float | None,
    primary_amplitude: float,
    primary_frequency: float,
    secondary_amplitude: float,
    secondary_frequency: float,
    shoulder_amplitude: float,
    shoulder_frequency: float,
) -> float:
    center = drive_center_deg(spec, primary_center)
    amplitude = drive_amplitude_deg(spec, primary_amplitude, secondary_amplitude, shoulder_amplitude)
    frequency = drive_frequency_hz(spec, primary_frequency, secondary_frequency, shoulder_frequency)
    return center + amplitude * math.sin((2.0 * math.pi * frequency * time_s) + spec["phase_rad"])


def independent_drive_target_deg(
    spec: dict,
    drive_index: int,
    active_drive_index: int,
    segment_time_s: float,
    primary_center: float | None,
    primary_amplitude: float,
    primary_frequency: float,
    secondary_amplitude: float,
    secondary_frequency: float,
    shoulder_amplitude: float,
    shoulder_frequency: float,
) -> float:
    center = drive_center_deg(spec, primary_center)
    if drive_index != active_drive_index:
        return center
    amplitude = drive_amplitude_deg(spec, primary_amplitude, secondary_amplitude, shoulder_amplitude)
    frequency = drive_frequency_hz(spec, primary_frequency, secondary_frequency, shoulder_frequency)
    return center + amplitude * math.sin(2.0 * math.pi * frequency * segment_time_s)


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


def quat_wxyz_to_pitch_y_deg(orientation) -> float:
    orientation = to_numpy(orientation)
    w, x, y, z = [float(v) for v in orientation]
    sin_pitch = 2.0 * ((w * y) - (z * x))
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    return math.degrees(math.asin(sin_pitch))


def quat_wxyz_to_roll_x_deg(orientation) -> float:
    orientation = to_numpy(orientation)
    w, x, y, z = [float(v) for v in orientation]
    sin_roll = 2.0 * ((w * x) + (y * z))
    cos_roll = 1.0 - 2.0 * ((x * x) + (y * y))
    return math.degrees(math.atan2(sin_roll, cos_roll))


def empty_scalar_stats():
    return {"min": float("inf"), "max": float("-inf"), "final": 0.0}


def update_scalar_stats(stats: dict, value: float):
    stats["min"] = min(stats["min"], float(value))
    stats["max"] = max(stats["max"], float(value))
    stats["final"] = float(value)


def empty_vector_stats():
    return {
        "min": np.array([float("inf"), float("inf"), float("inf")], dtype=np.float64),
        "max": np.array([float("-inf"), float("-inf"), float("-inf")], dtype=np.float64),
        "final": np.zeros(3, dtype=np.float64),
    }


def update_vector_stats(stats: dict, value: np.ndarray):
    value = np.asarray(value, dtype=np.float64)
    stats["min"] = np.minimum(stats["min"], value)
    stats["max"] = np.maximum(stats["max"], value)
    stats["final"] = value


def rounded_scalar_stats(stats: dict) -> dict[str, float]:
    return {key: round(float(value), 6) for key, value in stats.items()}


def rounded_vector_stats(stats: dict) -> dict[str, list[float]]:
    return {
        "min_m": [round(float(value), 6) for value in stats["min"]],
        "max_m": [round(float(value), 6) for value in stats["max"]],
        "final_m": [round(float(value), 6) for value in stats["final"]],
    }


def fit_linear_calibration(samples: list[dict], input_names: list[str], output_names: list[str]) -> dict:
    """Fit a local linear map from commanded drive targets to measured linkage outputs."""
    if not output_names:
        return {
            "status": "skipped",
            "reason": "No calibration outputs are configured for this geometry.",
            "sample_count": len(samples),
        }

    if len(samples) <= len(input_names):
        return {
            "status": "skipped",
            "reason": "Not enough samples for a linear fit.",
            "sample_count": len(samples),
        }

    x = np.array([[1.0] + [sample["inputs"][name] for name in input_names] for sample in samples], dtype=np.float64)
    rank = int(np.linalg.matrix_rank(x))
    results = {}
    for output_name in output_names:
        y = np.array([sample["outputs"][output_name] for sample in samples], dtype=np.float64)
        coeffs, *_ = np.linalg.lstsq(x, y, rcond=None)
        predicted = x @ coeffs
        residual = y - predicted
        ss_res = float(np.sum(residual * residual))
        centered = y - float(np.mean(y))
        ss_tot = float(np.sum(centered * centered))
        r_squared = 1.0 if ss_tot <= 1e-12 else 1.0 - (ss_res / ss_tot)
        rmse = math.sqrt(ss_res / max(len(samples), 1))
        results[output_name] = {
            "intercept": round(float(coeffs[0]), 6),
            "coefficients": {
                input_name: round(float(coeffs[index + 1]), 6) for index, input_name in enumerate(input_names)
            },
            "rmse": round(float(rmse), 6),
            "r_squared": round(float(r_squared), 6),
            "min_actual": round(float(np.min(y)), 6),
            "max_actual": round(float(np.max(y)), 6),
        }

    return {
        "status": "fit",
        "model": "output_deg = intercept + sum(coeff_deg_per_deg * drive_target_deg)",
        "sample_count": len(samples),
        "matrix_rank": rank,
        "input_count_with_intercept": len(input_names) + 1,
        "rank_deficient": rank < len(input_names) + 1,
        "inputs": input_names,
        "outputs": results,
    }


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
        "characterization": {
            "pitch_bodies": ["ground", "crank", "coupler", "rocker"],
            "relative_pitch_pairs": [
                {"name": "crank_to_ground", "body_a": "crank", "body_b": "ground"},
                {"name": "coupler_to_ground", "body_a": "coupler", "body_b": "ground"},
                {"name": "rocker_to_ground", "body_a": "rocker", "body_b": "ground"},
            ],
            "drive_angle_pairs": {"drive_crank_pin": {"body_a": "crank", "body_b": "ground"}},
            "pivot_tracks": [{"name": "loop_closure_pin", "body": "coupler", "pivot": pivot_c.tolist()}],
        },
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
        "characterization": {
            "pitch_bodies": ["ground", "lower_driver", "coupler", "diagonal"],
            "relative_pitch_pairs": [
                {"name": "lower_driver_to_ground", "body_a": "lower_driver", "body_b": "ground"},
                {"name": "coupler_to_ground", "body_a": "coupler", "body_b": "ground"},
                {"name": "diagonal_to_ground", "body_a": "diagonal", "body_b": "ground"},
                {"name": "diagonal_to_coupler", "body_a": "diagonal", "body_b": "coupler"},
            ],
            "drive_angle_pairs": {"drive_revolute_59": {"body_a": "lower_driver", "body_b": "ground"}},
            "pivot_tracks": [
                {
                    "name": "lower_closure_revolute_25_26",
                    "body": "lower_driver",
                    "pivot": points["closure_revolute_25_26"].tolist(),
                }
            ],
        },
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
        "characterization": {
            "pitch_bodies": ["ground", "lower_reference", "upper_driver", "coupler"],
            "relative_pitch_pairs": [
                {"name": "upper_driver_to_ground", "body_a": "upper_driver", "body_b": "ground"},
                {"name": "coupler_to_lower_reference", "body_a": "coupler", "body_b": "lower_reference"},
                {"name": "upper_driver_to_coupler", "body_a": "upper_driver", "body_b": "coupler"},
            ],
            "drive_angle_pairs": {"drive_revolute_58": {"body_a": "upper_driver", "body_b": "ground"}},
            "pivot_tracks": [
                {"name": "upper_closure_revolute_32_51", "body": "upper_driver", "pivot": points["closure_revolute_51"].tolist()}
            ],
        },
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
        "characterization": {
            "pitch_bodies": ["ground", "lower_driver", "coupler", "lower_diagonal", "upper_driver"],
            "relative_pitch_pairs": [
                {"name": "lower_driver_to_ground", "body_a": "lower_driver", "body_b": "ground"},
                {"name": "upper_driver_to_ground", "body_a": "upper_driver", "body_b": "ground"},
                {"name": "coupler_to_ground", "body_a": "coupler", "body_b": "ground"},
                {"name": "lower_diagonal_to_coupler", "body_a": "lower_diagonal", "body_b": "coupler"},
                {"name": "upper_driver_to_coupler", "body_a": "upper_driver", "body_b": "coupler"},
            ],
            "drive_angle_pairs": {
                "lower_drive_revolute_59": {"body_a": "lower_driver", "body_b": "ground"},
                "upper_drive_revolute_58": {"body_a": "upper_driver", "body_b": "ground"},
            },
            "pivot_tracks": [
                {
                    "name": "lower_closure_revolute_25_26",
                    "body": "lower_driver",
                    "pivot": points["lower_closure_revolute_25_26"].tolist(),
                },
                {
                    "name": "upper_closure_revolute_32_51",
                    "body": "upper_driver",
                    "pivot": points["upper_closure_revolute_51"].tolist(),
                },
            ],
        },
    }


DOMINO_FOUR_COMBINED_LEG_SPECS = [
    {
        "id": "dom_p_4_1",
        "hip_link": "DOM_P__4__1",
        "shoulder_joint": "Revolute 1",
        "shoulder_axis": "-X",
        "shoulder_limit_deg": (-30.0, 30.0),
        "shoulder_center_deg": 0.0,
        "lower_drive_joint": "Revolute 59",
        "upper_drive_joint": "Revolute 58",
        "lower_passive_joint": "Revolute 43",
        "lower_coupler_joint": "Revolute 33",
        "lower_closure_joints": ("Revolute 25", "Revolute 26"),
        "upper_closure_joints": ("Revolute 32", "Revolute 51"),
        "lower_drive_limit_deg": (-120.0, 0.0),
        "lower_drive_center_deg": -15.0,
        "upper_drive_center_deg": 0.0,
        "phase_deg": 0.0,
        "points": {
            "hip_origin": (0.266500, 0.000000, 0.010500),
            "upper_drive": (0.347000, -0.028000, 0.010500),
            "lower_drive": (0.323000, -0.028000, -0.010500),
            "lower_passive": (0.323000, -0.036000, -0.010500),
            "lower_coupler": (0.294708, -0.035600, 0.017777),
            "upper_closure": (0.336647, -0.035600, 0.049137),
            "lower_closure": (0.182024, -0.048100, -0.095615),
        },
    },
    {
        "id": "dom_p_12_1",
        "hip_link": "DOM_P__12__1",
        "shoulder_joint": "Revolute 2",
        "shoulder_axis": "X",
        "shoulder_limit_deg": (-30.0, 30.0),
        "shoulder_center_deg": 0.0,
        "lower_drive_joint": "Revolute 46",
        "upper_drive_joint": "Revolute 55",
        "lower_passive_joint": "Revolute 44",
        "lower_coupler_joint": "Revolute 36",
        "lower_closure_joints": ("Revolute 23", "Revolute 24"),
        "upper_closure_joints": ("Revolute 29", "Revolute 50"),
        "lower_drive_limit_deg": (-120.0, 0.0),
        "lower_drive_center_deg": -15.0,
        "upper_drive_center_deg": 0.0,
        "phase_deg": 90.0,
        "points": {
            "hip_origin": (0.266500, 0.124750, 0.010500),
            "upper_drive": (0.347000, 0.152750, 0.010500),
            "lower_drive": (0.323000, 0.152750, -0.010500),
            "lower_passive": (0.323000, 0.160750, -0.010500),
            "lower_coupler": (0.294708, 0.160350, 0.017777),
            "upper_closure": (0.336647, 0.160350, 0.049137),
            "lower_closure": (0.181670, 0.172850, -0.095261),
        },
    },
    {
        "id": "dom_p_25_1",
        "hip_link": "DOM_P__25__1",
        "shoulder_joint": "Revolute 3",
        "shoulder_axis": "X",
        "shoulder_limit_deg": (-30.0, 30.0),
        "shoulder_center_deg": 0.0,
        "lower_drive_joint": "Revolute 47",
        "upper_drive_joint": "Revolute 56",
        "lower_passive_joint": "Revolute 45",
        "lower_coupler_joint": "Revolute 35",
        "lower_closure_joints": ("Revolute 21", "Revolute 22"),
        "upper_closure_joints": ("Revolute 34", "Revolute 54"),
        "lower_drive_limit_deg": (-120.0, 0.0),
        "lower_drive_center_deg": -15.0,
        "upper_drive_center_deg": 0.0,
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
            "upper_closure": (0.001647, 0.160350, 0.049137),
            "lower_closure": (-0.153330, 0.172850, -0.095261),
        },
    },
    {
        "id": "dom_p_21_1",
        "hip_link": "DOM_P__21__1",
        "shoulder_joint": "Revolute 4",
        "shoulder_axis": "-X",
        "shoulder_limit_deg": (-30.0, 30.0),
        "shoulder_center_deg": 0.0,
        "lower_drive_joint": "Revolute 48",
        "upper_drive_joint": "Revolute 57",
        "lower_passive_joint": "Revolute 42",
        "lower_coupler_joint": "Revolute 37",
        "lower_closure_joints": ("Revolute 27", "Revolute 28"),
        "upper_closure_joints": ("Revolute 31", "Revolute 53"),
        "lower_drive_limit_deg": (-30.0, 90.0),
        "lower_drive_center_deg": -15.0,
        "upper_drive_center_deg": 0.0,
        "phase_deg": 270.0,
        "points": {
            "hip_origin": (-0.068500, 0.000000, 0.010500),
            "upper_drive": (0.012000, -0.028000, 0.010500),
            "lower_drive": (-0.012000, -0.028000, -0.010500),
            "lower_passive": (-0.012000, -0.036000, -0.010500),
            "lower_coupler": (-0.040292, -0.035600, 0.017777),
            "upper_closure": (0.001647, -0.035600, 0.049137),
            "lower_closure": (-0.152976, -0.048100, -0.095615),
        },
    },
]


def joint_key(leg_id: str, joint_name: str) -> str:
    return f"{leg_id}_{joint_name.lower().replace(' ', '_')}"


def build_domino_combined_leg_instance(
    stage,
    root: str,
    spec: dict,
    include_shoulder: bool = False,
    shared_base: dict | None = None,
) -> dict:
    leg_root = f"{root}/{spec['id']}"
    UsdGeom.Xform.Define(stage, leg_root)
    points = {name: np.array(value, dtype=np.float64) for name, value in spec["points"].items()}
    prefix = spec["id"]

    base_anchor_key = f"{prefix}_base_anchor"
    ground_key = f"{prefix}_ground"
    lower_driver_key = f"{prefix}_lower_driver"
    coupler_key = f"{prefix}_coupler"
    lower_diagonal_key = f"{prefix}_lower_diagonal"
    upper_driver_key = f"{prefix}_upper_driver"

    bodies = {}
    drives = []
    drive_angle_pairs = {}
    ground_points = [points["upper_drive"], points["lower_drive"]]
    if include_shoulder:
        ground_points = [points["hip_origin"], points["upper_drive"], points["lower_drive"]]
        if shared_base is not None:
            base_anchor_key = shared_base["key"]
            base_anchor = shared_base["body"]
        else:
            base_anchor = create_body_from_points(
                stage,
                leg_root,
                "base_anchor",
                [points["hip_origin"]],
                width=0.018,
                mass=1.0,
                kinematic=True,
            )
            bodies[base_anchor_key] = base_anchor

    ground = create_body_from_points(
        stage,
        leg_root,
        "hip_carriage" if include_shoulder else "ground_hip_reference",
        ground_points,
        width=0.014,
        mass=0.12 if include_shoulder else 1.0,
        kinematic=not include_shoulder,
    )
    lower_driver = create_body_from_points(
        stage,
        leg_root,
        "lower_driver",
        [points["lower_drive"], points["lower_passive"], points["lower_closure"]],
        width=0.010,
        mass=0.08,
    )
    coupler = create_body_from_points(
        stage,
        leg_root,
        "shared_coupler",
        [points["lower_passive"], points["lower_coupler"], points["upper_closure"]],
        width=0.008,
        mass=0.05,
    )
    lower_diagonal = create_body_from_points(
        stage,
        leg_root,
        "lower_diagonal",
        [points["lower_coupler"], points["lower_closure"]],
        width=0.008,
        mass=0.04,
    )
    upper_driver = create_body_from_points(
        stage,
        leg_root,
        "upper_driver",
        [points["upper_drive"], points["upper_closure"]],
        width=0.010,
        mass=0.06,
    )

    lower_joint_name = joint_key(prefix, spec["lower_drive_joint"])
    upper_joint_name = joint_key(prefix, spec["upper_drive_joint"])
    lower_limit_deg = spec["lower_drive_limit_deg"]

    if include_shoulder:
        shoulder_joint_name = joint_key(prefix, spec["shoulder_joint"])
        shoulder_limit_deg = spec["shoulder_limit_deg"]
        shoulder_joint = create_pin_joint(
            stage,
            f"{leg_root}/joints/{shoulder_joint_name}",
            base_anchor,
            ground,
            points["hip_origin"],
            lower_deg=shoulder_limit_deg[0],
            upper_deg=shoulder_limit_deg[1],
            axis="X",
        )
        shoulder_drive = apply_angular_drive(
            shoulder_joint,
            stiffness=1.4,
            damping=0.35,
            max_force=0.75,
            target_deg=spec["shoulder_center_deg"],
        )
        drives.append(
            make_drive_spec(
                shoulder_joint_name,
                shoulder_drive,
                spec["shoulder_center_deg"],
                amplitude_source="shoulder",
                frequency_source="shoulder",
                phase_deg=spec["phase_deg"],
                role="shoulder_ab_ad",
                axis="X",
            )
        )
        drive_angle_pairs[shoulder_joint_name] = {
            "body_a": ground_key,
            "body_b": base_anchor_key,
            "axis": "roll_x",
        }

    lower_drive_joint = create_pin_joint(
        stage,
        f"{leg_root}/joints/{lower_joint_name}",
        ground,
        lower_driver,
        points["lower_drive"],
        lower_deg=lower_limit_deg[0],
        upper_deg=lower_limit_deg[1],
    )
    create_pin_joint(
        stage,
        f"{leg_root}/joints/{joint_key(prefix, spec['lower_passive_joint'])}",
        lower_driver,
        coupler,
        points["lower_passive"],
    )
    create_pin_joint(
        stage,
        f"{leg_root}/joints/{joint_key(prefix, spec['lower_coupler_joint'])}",
        coupler,
        lower_diagonal,
        points["lower_coupler"],
    )
    create_pin_joint(
        stage,
        f"{leg_root}/joints/{prefix}_lower_loop_closure",
        lower_driver,
        lower_diagonal,
        points["lower_closure"],
    )

    upper_drive_joint = create_pin_joint(
        stage,
        f"{leg_root}/joints/{upper_joint_name}",
        ground,
        upper_driver,
        points["upper_drive"],
        lower_deg=-30.0,
        upper_deg=60.0,
    )
    create_pin_joint(
        stage,
        f"{leg_root}/joints/{prefix}_upper_loop_closure",
        coupler,
        upper_driver,
        points["upper_closure"],
    )

    lower_drive = apply_angular_drive(
        lower_drive_joint,
        stiffness=1.2,
        damping=0.35,
        max_force=0.65,
        target_deg=spec["lower_drive_center_deg"],
    )
    upper_drive = apply_angular_drive(
        upper_drive_joint,
        stiffness=1.0,
        damping=0.30,
        max_force=0.55,
        target_deg=spec["upper_drive_center_deg"],
    )
    drives.extend(
        [
            make_drive_spec(
                lower_joint_name,
                lower_drive,
                spec["lower_drive_center_deg"],
                amplitude_source="primary",
                phase_deg=spec["phase_deg"],
                role="lower_linkage_drive",
                axis="Y",
            ),
            make_drive_spec(
                upper_joint_name,
                upper_drive,
                spec["upper_drive_center_deg"],
                amplitude_source="secondary",
                frequency_source="secondary",
                phase_deg=spec["phase_deg"] + 90.0,
                role="upper_pitch_drive",
                axis="Y",
            ),
        ]
    )
    drive_angle_pairs.update(
        {
            lower_joint_name: {"body_a": lower_driver_key, "body_b": ground_key, "axis": "pitch_y"},
            upper_joint_name: {"body_a": upper_driver_key, "body_b": ground_key, "axis": "pitch_y"},
        }
    )

    lower_loop_name = f"{prefix}_lower_loop_closure_{spec['lower_closure_joints'][0].replace(' ', '_')}_{spec['lower_closure_joints'][1].replace(' ', '_')}"
    upper_loop_name = f"{prefix}_upper_loop_closure_{spec['upper_closure_joints'][0].replace(' ', '_')}_{spec['upper_closure_joints'][1].replace(' ', '_')}"
    bodies.update(
        {
            ground_key: ground,
            lower_driver_key: lower_driver,
            coupler_key: coupler,
            lower_diagonal_key: lower_diagonal,
            upper_driver_key: upper_driver,
        }
    )

    return {
        "points": {f"{prefix}_{name}": point.tolist() for name, point in points.items()},
        "bodies": bodies,
        "drives": drives,
        "loop_checks": [
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
        ],
        "characterization": {
            "pitch_bodies": [ground_key, lower_driver_key, coupler_key, lower_diagonal_key, upper_driver_key],
            "roll_bodies": [base_anchor_key, ground_key] if include_shoulder else [],
            "relative_pitch_pairs": [
                {"name": f"{prefix}_lower_driver_to_ground", "body_a": lower_driver_key, "body_b": ground_key},
                {"name": f"{prefix}_upper_driver_to_ground", "body_a": upper_driver_key, "body_b": ground_key},
                {"name": f"{prefix}_coupler_to_ground", "body_a": coupler_key, "body_b": ground_key},
                {
                    "name": f"{prefix}_lower_diagonal_to_coupler",
                    "body_a": lower_diagonal_key,
                    "body_b": coupler_key,
                },
                {"name": f"{prefix}_upper_driver_to_coupler", "body_a": upper_driver_key, "body_b": coupler_key},
            ],
            "drive_angle_pairs": drive_angle_pairs,
            "pivot_tracks": [
                {"name": f"{prefix}_lower_closure", "body": lower_driver_key, "pivot": points["lower_closure"].tolist()},
                {"name": f"{prefix}_upper_closure", "body": upper_driver_key, "pivot": points["upper_closure"].tolist()},
            ],
        },
        "leg": {
            "id": spec["id"],
            "hip_link": spec["hip_link"],
            "shoulder_joint": spec["shoulder_joint"] if include_shoulder else None,
            "shoulder_axis": spec["shoulder_axis"] if include_shoulder else None,
            "lower_drive_joint": spec["lower_drive_joint"],
            "upper_drive_joint": spec["upper_drive_joint"],
            "lower_closure_joints": list(spec["lower_closure_joints"]),
            "upper_closure_joints": list(spec["upper_closure_joints"]),
            "notes": spec.get("notes", []),
        },
    }


def build_domino_four_combined_legs(stage, include_shoulders: bool = False, shared_body: bool = False):
    if shared_body:
        root = "/World/DominoFour12FixedBody"
    elif include_shoulders:
        root = "/World/DominoFour12Actuators"
    else:
        root = "/World/DominoFourCombinedLegs"
    UsdGeom.Xform.Define(stage, root)

    points = {}
    bodies = {}
    drives = []
    loop_checks = []
    pitch_bodies = []
    roll_bodies = []
    relative_pitch_pairs = []
    drive_angle_pairs = {}
    pivot_tracks = []
    legs = []
    shared_base = None
    if shared_body:
        hip_points = [
            np.array(spec["points"]["hip_origin"], dtype=np.float64)
            for spec in DOMINO_FOUR_COMBINED_LEG_SPECS
        ]
        body_reference = create_body_from_points(
            stage,
            root,
            "body_reference",
            hip_points,
            width=0.030,
            mass=1.2,
            kinematic=True,
        )
        shared_base = {"key": "body_reference", "body": body_reference}
        bodies[shared_base["key"]] = body_reference

    for spec in DOMINO_FOUR_COMBINED_LEG_SPECS:
        leg = build_domino_combined_leg_instance(
            stage,
            root,
            spec,
            include_shoulder=include_shoulders,
            shared_base=shared_base,
        )
        points.update(leg["points"])
        bodies.update(leg["bodies"])
        drives.extend(leg["drives"])
        loop_checks.extend(leg["loop_checks"])
        pitch_bodies.extend(leg["characterization"]["pitch_bodies"])
        roll_bodies.extend(leg["characterization"]["roll_bodies"])
        relative_pitch_pairs.extend(leg["characterization"]["relative_pitch_pairs"])
        drive_angle_pairs.update(leg["characterization"]["drive_angle_pairs"])
        pivot_tracks.extend(leg["characterization"]["pivot_tracks"])
        legs.append(leg["leg"])

    return {
        "geometry": (
            "domino-four-12-fixed-body"
            if shared_body
            else "domino-four-12-actuators"
            if include_shoulders
            else "domino-four-combined-legs"
        ),
        "drive": drives[0]["drive"],
        "drive_joint_name": drives[0]["joint"],
        "drive_center_deg": drives[0]["center_deg"],
        "drives": drives,
        "points": points,
        "bodies": bodies,
        "loop_checks": loop_checks,
        "legs": legs,
        "characterization": {
            "pitch_bodies": pitch_bodies,
            "roll_bodies": roll_bodies,
            "relative_pitch_pairs": relative_pitch_pairs,
            "drive_angle_pairs": drive_angle_pairs,
            "pivot_tracks": pivot_tracks,
        },
    }


def build_linkage(stage):
    if args_cli.geometry == "domino-four-12-fixed-body":
        return build_domino_four_combined_legs(stage, include_shoulders=True, shared_body=True)
    if args_cli.geometry == "domino-four-12-actuators":
        return build_domino_four_combined_legs(stage, include_shoulders=True)
    if args_cli.geometry == "domino-four-combined-legs":
        return build_domino_four_combined_legs(stage)
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
    primary_center = float(args_cli.drive_center_deg) if args_cli.drive_center_deg is not None else None
    primary_amplitude = float(args_cli.drive_amplitude_deg)
    primary_frequency = float(args_cli.drive_frequency_hz)
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
    shoulder_amplitude = float(
        args_cli.shoulder_drive_amplitude_deg
        if args_cli.shoulder_drive_amplitude_deg is not None
        else args_cli.drive_amplitude_deg
    )
    shoulder_frequency = float(
        args_cli.shoulder_drive_frequency_hz
        if args_cli.shoulder_drive_frequency_hz is not None
        else args_cli.drive_frequency_hz
    )
    independent_segment_steps = max(1, int(args_cli.independent_segment_steps))
    independent_settle_steps = max(0, min(int(args_cli.independent_settle_steps), independent_segment_steps - 1))
    characterization = linkage.get("characterization", {})
    body_pitch_stats = {
        name: empty_scalar_stats() for name in characterization.get("pitch_bodies", [])
    }
    body_roll_stats = {
        name: empty_scalar_stats() for name in characterization.get("roll_bodies", [])
    }
    relative_pitch_stats = {
        pair["name"]: empty_scalar_stats() for pair in characterization.get("relative_pitch_pairs", [])
    }
    drive_target_stats = {spec["joint"]: empty_scalar_stats() for spec in drive_specs}
    drive_tracking_error_stats = {spec["joint"]: empty_scalar_stats() for spec in drive_specs}
    pivot_track_stats = {
        track["name"]: empty_vector_stats() for track in characterization.get("pivot_tracks", [])
    }
    calibration_samples = []
    calibration_output_names = set()

    for step in range(args_cli.steps):
        time_s = step * sim_dt
        segment_step = 0
        active_drive_index = None
        if args_cli.drive_schedule == "independent":
            segment_step = step % independent_segment_steps
            active_drive_index = (step // independent_segment_steps) % len(drive_specs)

        for drive_index, spec in enumerate(drive_specs):
            if args_cli.drive_schedule == "independent":
                target = independent_drive_target_deg(
                    spec,
                    drive_index,
                    active_drive_index,
                    segment_step * sim_dt,
                    primary_center,
                    primary_amplitude,
                    primary_frequency,
                    secondary_amplitude,
                    secondary_frequency,
                    shoulder_amplitude,
                    shoulder_frequency,
                )
            else:
                target = drive_target_deg(
                    spec,
                    time_s,
                    primary_center,
                    primary_amplitude,
                    primary_frequency,
                    secondary_amplitude,
                    secondary_frequency,
                    shoulder_amplitude,
                    shoulder_frequency,
                )
            spec["target_attr"].Set(float(target))
            spec["current_target_deg"] = float(target)
            update_scalar_stats(drive_target_stats[spec["joint"]], float(target))

        sim.step()

        positions = []
        velocities = []
        pose_cache = {}
        for name, view in views.items():
            pos, quat = view.get_world_pose()
            lin_vel = view.get_linear_velocity()
            pose_cache[name] = {"position": pos, "orientation": quat}
            positions.extend(to_numpy(pos).flatten().tolist())
            positions.extend(to_numpy(quat).flatten().tolist())
            velocities.extend(to_numpy(lin_vel).flatten().tolist())
        if not np.isfinite(np.array(positions + velocities, dtype=np.float64)).all():
            min_finite = False
            break

        pitch_values = {}
        roll_values = {}
        relative_pitch_values = {}
        drive_angle_values = {}
        for body_name in characterization.get("pitch_bodies", []):
            pitch = quat_wxyz_to_pitch_y_deg(pose_cache[body_name]["orientation"])
            pitch_values[body_name] = pitch
            update_scalar_stats(body_pitch_stats[body_name], pitch)
        for body_name in characterization.get("roll_bodies", []):
            roll = quat_wxyz_to_roll_x_deg(pose_cache[body_name]["orientation"])
            roll_values[body_name] = roll
            update_scalar_stats(body_roll_stats[body_name], roll)

        for pair in characterization.get("relative_pitch_pairs", []):
            body_a = pair["body_a"]
            body_b = pair["body_b"]
            if body_a not in pitch_values:
                pitch_values[body_a] = quat_wxyz_to_pitch_y_deg(pose_cache[body_a]["orientation"])
            if body_b not in pitch_values:
                pitch_values[body_b] = quat_wxyz_to_pitch_y_deg(pose_cache[body_b]["orientation"])
            relative_pitch = pitch_values[body_a] - pitch_values[body_b]
            relative_pitch_values[pair["name"]] = relative_pitch
            update_scalar_stats(relative_pitch_stats[pair["name"]], relative_pitch)

        for spec in drive_specs:
            pair = characterization.get("drive_angle_pairs", {}).get(spec["joint"])
            if not pair:
                continue
            body_a = pair["body_a"]
            body_b = pair["body_b"]
            axis = pair.get("axis", "pitch_y")
            if axis == "roll_x":
                if body_a not in roll_values:
                    roll_values[body_a] = quat_wxyz_to_roll_x_deg(pose_cache[body_a]["orientation"])
                if body_b not in roll_values:
                    roll_values[body_b] = quat_wxyz_to_roll_x_deg(pose_cache[body_b]["orientation"])
                actual_deg = roll_values[body_a] - roll_values[body_b]
            else:
                if body_a not in pitch_values:
                    pitch_values[body_a] = quat_wxyz_to_pitch_y_deg(pose_cache[body_a]["orientation"])
                if body_b not in pitch_values:
                    pitch_values[body_b] = quat_wxyz_to_pitch_y_deg(pose_cache[body_b]["orientation"])
                actual_deg = pitch_values[body_a] - pitch_values[body_b]
            drive_angle_values[spec["joint"]] = actual_deg
            error_deg = actual_deg - spec["current_target_deg"]
            update_scalar_stats(drive_tracking_error_stats[spec["joint"]], error_deg)

        include_calibration_sample = step >= args_cli.fit_start_step
        if args_cli.drive_schedule == "independent":
            include_calibration_sample = include_calibration_sample and segment_step >= independent_settle_steps

        if include_calibration_sample:
            sample_outputs = {}
            for body_name in characterization.get("pitch_bodies", []):
                output_name = f"body_pitch_y_deg.{body_name}"
                sample_outputs[output_name] = pitch_values[body_name]
                calibration_output_names.add(output_name)
            for body_name in characterization.get("roll_bodies", []):
                output_name = f"body_roll_x_deg.{body_name}"
                sample_outputs[output_name] = roll_values[body_name]
                calibration_output_names.add(output_name)
            for pair in characterization.get("relative_pitch_pairs", []):
                output_name = f"relative_pitch_y_deg.{pair['name']}"
                sample_outputs[output_name] = relative_pitch_values[pair["name"]]
                calibration_output_names.add(output_name)
            for spec in drive_specs:
                if spec["joint"] not in drive_angle_values:
                    continue
                output_name = f"drive_angle_deg.{spec['joint']}"
                sample_outputs[output_name] = drive_angle_values[spec["joint"]]
                calibration_output_names.add(output_name)
            calibration_samples.append(
                {
                    "inputs": {spec["joint"]: spec["current_target_deg"] for spec in drive_specs},
                    "outputs": sample_outputs,
                }
            )

        for check in linkage["loop_checks"]:
            pivot = np.array(check["pivot"], dtype=np.float64)
            body_a = linkage["bodies"][check["body_a"]]
            body_b = linkage["bodies"][check["body_b"]]
            world_a = world_endpoint(views[check["body_a"]], local_endpoint(pivot, body_a["center"]))
            world_b = world_endpoint(views[check["body_b"]], local_endpoint(pivot, body_b["center"]))
            loop_error = float(np.linalg.norm(world_a - world_b))
            max_loop_errors[check["name"]] = max(max_loop_errors[check["name"]], loop_error)

        for track in characterization.get("pivot_tracks", []):
            pivot = np.array(track["pivot"], dtype=np.float64)
            body = linkage["bodies"][track["body"]]
            world = world_endpoint(views[track["body"]], local_endpoint(pivot, body["center"]))
            update_vector_stats(pivot_track_stats[track["name"]], world)

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
            "target_center_deg": drive_center_deg(drive_specs[0], primary_center),
            "target_amplitude_deg": primary_amplitude,
            "frequency_hz": primary_frequency,
        },
        "drive_schedule": {
            "mode": args_cli.drive_schedule,
            "independent_segment_steps": independent_segment_steps,
            "independent_settle_steps": independent_settle_steps,
            "segments_per_full_cycle": len(drive_specs),
        },
        "drives": [
            {
                "joint": spec["joint"],
                "target_center_deg": drive_center_deg(spec, primary_center),
                "target_amplitude_deg": drive_amplitude_deg(
                    spec,
                    primary_amplitude,
                    secondary_amplitude,
                    shoulder_amplitude,
                ),
                "frequency_hz": drive_frequency_hz(
                    spec,
                    primary_frequency,
                    secondary_frequency,
                    shoulder_frequency,
                ),
                "phase_deg": spec["phase_deg"],
                "role": spec["role"],
                "axis": spec["axis"],
            }
            for spec in drive_specs
        ],
        "max_loop_closure_error_m": round(max(max_loop_errors.values()), 8),
        "loop_closure_errors_m": {name: round(value, 8) for name, value in max_loop_errors.items()},
        "max_body_linear_speed_m_s": round(max_linear_speed, 6),
        "characterization": {
            "drive_target_deg": {name: rounded_scalar_stats(stats) for name, stats in drive_target_stats.items()},
            "drive_tracking_error_deg": {
                name: rounded_scalar_stats(stats) for name, stats in drive_tracking_error_stats.items()
            },
            "body_pitch_y_deg": {name: rounded_scalar_stats(stats) for name, stats in body_pitch_stats.items()},
            "body_roll_x_deg": {name: rounded_scalar_stats(stats) for name, stats in body_roll_stats.items()},
            "relative_pitch_y_deg": {
                name: rounded_scalar_stats(stats) for name, stats in relative_pitch_stats.items()
            },
            "tracked_pivots_world": {name: rounded_vector_stats(stats) for name, stats in pivot_track_stats.items()},
            "linear_calibration_fit": fit_linear_calibration(
                calibration_samples,
                [spec["joint"] for spec in drive_specs],
                sorted(calibration_output_names),
            ),
        },
        "final_poses": final_poses,
    }
    if "legs" in linkage:
        report["legs"] = linkage["legs"]

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if not args_cli.no_print_report:
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
