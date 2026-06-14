"""Spawn and sweep the simplified Domino one-leg prototype in Isaac Lab.

This is a runtime smoke test, not a training task. It verifies that the imported
USD can be used as an Isaac Lab articulation and that the three driven joints can
accept targets for several physics steps without producing non-finite state.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import traceback

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Run a headless Domino one-leg articulation sweep.")
parser.add_argument("--usd-path", required=True, help="Path to the imported one-leg USD.")
parser.add_argument("--steps", type=int, default=600, help="Number of physics steps to run.")
parser.add_argument("--amplitude-scale", type=float, default=0.25, help="Fraction of joint range to sweep from center.")
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
os.environ["DOMINO_ONE_LEG_USD"] = str(usd_path)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from domino_one_leg_cfg import DOMINO_ONE_LEG_CFG  # noqa: E402


EXPECTED_JOINTS = ["hip_ab_ad", "upper_pitch", "lower_linkage"]


def design_scene() -> Articulation:
    """Create a minimal scene and return the Domino leg articulation."""
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)

    light_cfg = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    robot_cfg = DOMINO_ONE_LEG_CFG.replace(prim_path="/World/DominoOneLeg")
    return Articulation(cfg=robot_cfg)


def reset_robot(robot: Articulation):
    """Reset root and joint state to the configured defaults."""
    root_state = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(root_state[:, 7:])

    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    robot.reset()


def tensor_list(value: torch.Tensor) -> list[float]:
    """Convert a tensor to a rounded Python list for reports."""
    return [round(float(v), 6) for v in value.detach().cpu().flatten()]


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([0.75, -0.85, 0.55], [0.0, 0.0, 0.05])

    robot = design_scene()
    sim.reset()
    reset_robot(robot)
    robot.update(sim.get_physics_dt())

    joint_ids, joint_names = robot.find_joints(EXPECTED_JOINTS, preserve_order=True)
    if joint_names != EXPECTED_JOINTS:
        raise RuntimeError(f"Expected joints {EXPECTED_JOINTS}, found {joint_names}. All joints: {robot.joint_names}")

    joint_ids_tensor = torch.tensor(joint_ids, device=sim.device, dtype=torch.long)
    limits = robot.data.soft_joint_pos_limits[0, joint_ids_tensor, :]
    lower = limits[:, 0]
    upper = limits[:, 1]
    center = 0.5 * (lower + upper)
    amplitude = 0.5 * (upper - lower) * args_cli.amplitude_scale
    phase_offsets = torch.tensor([0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0], device=sim.device)

    sim_dt = sim.get_physics_dt()
    max_tracking_error = 0.0
    max_joint_speed = 0.0

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

        if not torch.isfinite(actual).all() or not torch.isfinite(velocity).all() or not torch.isfinite(root_pos).all():
            raise RuntimeError(f"Non-finite articulation state at step {step}.")

        max_tracking_error = max(max_tracking_error, float(torch.max(torch.abs(actual - command)).detach().cpu()))
        max_joint_speed = max(max_joint_speed, float(torch.max(torch.abs(velocity)).detach().cpu()))

    report = {
        "status": "passed",
        "usd_path": str(usd_path),
        "steps": args_cli.steps,
        "physics_dt": sim_dt,
        "joint_names": joint_names,
        "joint_ids": joint_ids,
        "soft_joint_limits_rad": {
            name: tensor_list(limits[index]) for index, name in enumerate(joint_names)
        },
        "final_joint_pos_rad": tensor_list(robot.data.joint_pos[:, joint_ids_tensor]),
        "max_tracking_error_rad": round(max_tracking_error, 6),
        "max_joint_speed_rad_s": round(max_joint_speed, 6),
        "root_position_m": tensor_list(robot.data.root_pos_w),
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
