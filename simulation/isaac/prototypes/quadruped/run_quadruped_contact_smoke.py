"""Run a floating-base gravity/contact smoke test for the Domino quadruped."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import traceback

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Run a headless Domino quadruped floating-base contact smoke test.")
parser.add_argument("--usd-path", required=True, help="Path to the imported floating-base quadruped USD.")
parser.add_argument("--steps", type=int, default=1000, help="Number of physics steps to run.")
parser.add_argument("--drive-amplitude-scale", type=float, default=0.03, help="Small fraction of joint range to move.")
parser.add_argument("--max-tilt-deg", type=float, default=75.0, help="Fail if the base tilts beyond this angle.")
parser.add_argument("--min-root-height-m", type=float, default=0.06, help="Fail if the base drops below this height.")
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
parser.add_argument(
    "--graceful-close",
    action="store_true",
    help="Call SimulationApp.close() before exit. Disabled by default because it can hang on some Windows setups.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

usd_path = Path(args_cli.usd_path).expanduser().resolve()
if not usd_path.exists():
    raise FileNotFoundError(f"USD path does not exist: {usd_path}")
os.environ["DOMINO_QUADRUPED_USD"] = str(usd_path)
os.environ.setdefault("WARP_CACHE_PATH", str((Path.cwd() / "simulation" / "isaac" / "out" / "warp_cache").resolve()))

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402
from domino_quadruped_cfg import (  # noqa: E402
    ACTION_JOINT_NAMES,
    DOMINO_QUADRUPED_CFG,
    action_group_counts,
    validate_action_layout,
)


def create_static_ground_box() -> None:
    """Create a static collision box whose top face sits at world Z=0."""
    stage = omni.usd.get_context().get_stage()
    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    xform = UsdGeom.XformCommonAPI(ground)
    xform.SetTranslate(Gf.Vec3d(0.0, 0.0, -0.025))
    xform.SetScale(Gf.Vec3f(2.0, 2.0, 0.05))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())


def design_scene() -> Articulation:
    create_static_ground_box()

    light_cfg = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    robot_cfg = DOMINO_QUADRUPED_CFG.replace(prim_path="/World/DominoQuadruped")
    return Articulation(cfg=robot_cfg)


def reset_robot(robot: Articulation):
    root_state = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(root_state[:, 7:])

    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    robot.reset()


def tensor_list(value: torch.Tensor) -> list[float]:
    return [round(float(v), 6) for v in value.detach().cpu().flatten()]


def root_tilt_deg(root_quat_w: torch.Tensor) -> torch.Tensor:
    quat = root_quat_w.reshape(-1, 4)
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    body_up_dot_world_up = torch.clamp(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)
    return torch.rad2deg(torch.acos(body_up_dot_world_up))


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args_cli.device)
    sim_cfg.physx.solver_type = 1
    sim_cfg.physx.min_position_iteration_count = 8
    sim_cfg.physx.max_position_iteration_count = 16
    sim_cfg.physx.min_velocity_iteration_count = 2
    sim_cfg.physx.max_velocity_iteration_count = 8
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([0.85, -0.85, 0.55], [0.05, 0.02, 0.02])

    robot = design_scene()
    sim.reset()
    reset_robot(robot)
    robot.update(sim.get_physics_dt())

    joint_ids, joint_names = robot.find_joints(ACTION_JOINT_NAMES, preserve_order=True)
    validate_action_layout(joint_names)

    joint_ids_tensor = torch.tensor(joint_ids, device=sim.device, dtype=torch.long)
    limits = robot.data.soft_joint_pos_limits[0, joint_ids_tensor, :]
    lower = limits[:, 0]
    upper = limits[:, 1]
    center = robot.data.default_joint_pos[:, joint_ids_tensor].clone()
    amplitude = 0.5 * (upper - lower) * float(args_cli.drive_amplitude_scale)
    phase_offsets = torch.linspace(0.0, 2.0 * math.pi, len(joint_ids), device=sim.device)

    sim_dt = sim.get_physics_dt()
    max_joint_speed = 0.0
    max_limit_violation = 0.0
    max_tracking_error = 0.0
    max_root_speed = 0.0
    max_tilt = 0.0
    min_root_z = float("inf")
    max_root_z = float("-inf")

    for step in range(args_cli.steps):
        phase = 2.0 * math.pi * step / max(args_cli.steps, 1)
        target = robot.data.default_joint_pos.clone()
        command = center + amplitude * torch.sin(torch.tensor(phase, device=sim.device) + phase_offsets)
        target[:, joint_ids_tensor] = command

        robot.set_joint_position_target(target)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim_dt)

        actual = robot.data.joint_pos[:, joint_ids_tensor]
        velocity = robot.data.joint_vel[:, joint_ids_tensor]
        root_pos = robot.data.root_pos_w
        root_quat = robot.data.root_quat_w
        root_lin_vel = robot.data.root_lin_vel_w

        state_values = [actual, velocity, root_pos, root_quat, root_lin_vel]
        if not all(torch.isfinite(value).all() for value in state_values):
            raise RuntimeError(f"Non-finite articulation state at step {step}.")

        lower_violation = torch.clamp(lower - actual, min=0.0)
        upper_violation = torch.clamp(actual - upper, min=0.0)
        tilt = root_tilt_deg(root_quat)
        root_z = root_pos[:, 2]

        max_limit_violation = max(
            max_limit_violation,
            float(torch.max(torch.maximum(lower_violation, upper_violation)).detach().cpu()),
        )
        max_tracking_error = max(max_tracking_error, float(torch.max(torch.abs(actual - command)).detach().cpu()))
        max_joint_speed = max(max_joint_speed, float(torch.max(torch.abs(velocity)).detach().cpu()))
        max_root_speed = max(max_root_speed, float(torch.max(torch.norm(root_lin_vel, dim=1)).detach().cpu()))
        max_tilt = max(max_tilt, float(torch.max(tilt).detach().cpu()))
        min_root_z = min(min_root_z, float(torch.min(root_z).detach().cpu()))
        max_root_z = max(max_root_z, float(torch.max(root_z).detach().cpu()))

        if min_root_z < args_cli.min_root_height_m:
            raise RuntimeError(f"Root height fell below {args_cli.min_root_height_m:.3f} m at step {step}.")
        if max_tilt > args_cli.max_tilt_deg:
            raise RuntimeError(f"Root tilt exceeded {args_cli.max_tilt_deg:.1f} deg at step {step}.")

    report = {
        "status": "passed",
        "usd_path": str(usd_path),
        "steps": args_cli.steps,
        "physics_dt": sim_dt,
        "action_count": len(joint_names),
        "action_group_counts": action_group_counts(),
        "action_joint_names": joint_names,
        "joint_ids": joint_ids,
        "soft_joint_limits_rad": {
            name: tensor_list(limits[index]) for index, name in enumerate(joint_names)
        },
        "final_joint_pos_rad": tensor_list(robot.data.joint_pos[:, joint_ids_tensor]),
        "root_position_m": tensor_list(robot.data.root_pos_w),
        "root_quat_wxyz": tensor_list(robot.data.root_quat_w),
        "min_root_height_m": round(min_root_z, 6),
        "max_root_height_m": round(max_root_z, 6),
        "max_root_speed_m_s": round(max_root_speed, 6),
        "max_root_tilt_deg": round(max_tilt, 6),
        "max_tracking_error_rad": round(max_tracking_error, 6),
        "max_joint_speed_rad_s": round(max_joint_speed, 6),
        "max_joint_limit_violation_rad": round(max_limit_violation, 8),
    }

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2), flush=True)


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
