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
parser.add_argument("--drive-amplitude-deg", type=float, default=12.0, help="Driven crank target amplitude.")
parser.add_argument("--drive-frequency-hz", type=float, default=0.6, help="Driven crank target frequency.")
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
    return Gf.Vec3f(float(point[0] - center[0]), 0.0, float(point[1] - center[1]))


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


def build_linkage(stage):
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
        "drive": drive,
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
    }


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

    drive_attr = linkage["drive"].GetTargetPositionAttr()
    sim_dt = sim.get_physics_dt()
    max_linear_speed = 0.0
    max_loop_error = 0.0
    min_finite = True
    initial_target = -55.0

    for step in range(args_cli.steps):
        time_s = step * sim_dt
        target = initial_target + args_cli.drive_amplitude_deg * math.sin(2.0 * math.pi * args_cli.drive_frequency_hz * time_s)
        drive_attr.Set(float(target))

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

        coupler_c = world_endpoint(views["coupler"], local_endpoint(np.array(linkage["bodies"]["coupler"]["end"]), linkage["bodies"]["coupler"]["center"]))
        rocker_c = world_endpoint(views["rocker"], local_endpoint(np.array(linkage["bodies"]["rocker"]["end"]), linkage["bodies"]["rocker"]["center"]))
        loop_error = float(np.linalg.norm(coupler_c - rocker_c))
        max_loop_error = max(max_loop_error, loop_error)
        max_linear_speed = max(
            max_linear_speed, max(float(np.linalg.norm(to_numpy(view.get_linear_velocity()))) for view in views.values())
        )

    final_poses = {}
    for name, view in views.items():
        pos, quat = view.get_world_pose()
        final_poses[name] = {"position_m": tensor_list(pos), "orientation_wxyz": tensor_list(quat)}

    report = {
        "status": "passed" if min_finite else "failed",
        "steps": step + 1,
        "physics_dt": sim_dt,
        "linkage_points_xz_m": linkage["points"],
        "drive": {
            "joint": "drive_crank_pin",
            "target_center_deg": initial_target,
            "target_amplitude_deg": args_cli.drive_amplitude_deg,
            "frequency_hz": args_cli.drive_frequency_hz,
        },
        "max_loop_closure_error_m": round(max_loop_error, 8),
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
