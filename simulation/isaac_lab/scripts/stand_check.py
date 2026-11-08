"""Sanity check: hold the neutral stand (zero action) and compare the simulated
feet with the firmware leg-frame geometry. Prints a short report and exits
non-zero if the stand is not stable.

    run-train.ps1 is for training; run this with the same Python:
    python scripts/stand_check.py --headless [--height 250]
"""

from __future__ import annotations

import argparse
import math
import os
import sys

# Load h5py (and its HDF5 DLLs) before Isaac Sim starts: in a visible window
# Kit loads its own HDF5 first and h5py then fails with "DLL load failed".
import h5py  # noqa: F401,E402  isort: skip
from isaaclab.app import AppLauncher

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from domino_lab import compat as _compat  # noqa: E402  (no Isaac imports)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

parser = argparse.ArgumentParser()
parser.add_argument("--ride_mm", type=float, default=280.0, help="ride height mm")
parser.add_argument("--seconds", type=float, default=4.0)
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--no_dr", action="store_true", help="disable randomisation for a clean check")
_compat.add_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(_compat.launcher_args(args)).app

import torch  # noqa: E402
import gymnasium as gym  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

import domino_lab  # noqa: E402,F401
from domino_lab import geometry as g  # noqa: E402
from domino_lab.compat import T, quat_wxyz  # noqa: E402,F401
from domino_lab import mdp  # noqa: E402


def main():
    cfg = load_cfg_from_registry("Domino-Walk-Play-v0", "env_cfg_entry_point")
    cfg.scene.num_envs = args.num_envs
    cfg.scene.terrain.terrain_type = "plane"
    cfg.scene.terrain.terrain_generator = None
    cfg.curriculum.terrain_levels = None
    h = args.ride_mm / 1000.0
    cfg.commands.body_height.range_m = (h, h)
    cfg.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
    cfg.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
    cfg.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
    if args.no_dr:
        for name in ("add_base_mass", "leg_mass", "base_com", "servo_gains", "joint_friction", "servo_effort", "push_robot"):
            if hasattr(cfg.events, name):
                setattr(cfg.events, name, None)
        cfg.actions.feet.delay_steps = (0, 0)
        cfg.actions.feet.calibration_error_deg = 0.0
        cfg.actions.feet.foot_x_offset_mm = 0.0
    env = gym.make("Domino-Walk-Play-v0", cfg=cfg).unwrapped
    env.reset()
    robot = env.scene["robot"]
    foot_ids, _ = robot.find_bodies(mdp.FOOT_BODIES, preserve_order=True)
    joint_ids, _ = robot.find_joints(mdp.JOINT_ORDER, preserve_order=True)
    act = torch.zeros(env.num_envs, 12, device=env.device)
    steps = int(args.seconds / env.step_dt)
    for i in range(steps):
        env.step(act)
        if i % 25 == 0 or i == steps - 1:
            z = T(robot.data.root_pos_w)[:, 2].mean().item()
            tilt = torch.rad2deg(torch.acos(-T(robot.data.projected_gravity_b)[:, 2].clamp(-1, 1))).max().item()
            print(f"t={i * env.step_dt:5.2f}s base_z={z:.3f} m  max_tilt={tilt:5.1f} deg  body_h={mdp.measured_body_height(env).mean().item():.3f}")
    # Feet in the body frame vs. firmware command
    from isaaclab.utils.math import quat_apply_inverse

    rel = T(robot.data.body_pos_w)[:, foot_ids] - T(robot.data.root_pos_w)[:, None]
    q = T(robot.data.root_quat_w)[:, None].expand(-1, 4, -1)
    feet_b = quat_apply_inverse(q.reshape(-1, 4), rel.reshape(-1, 3)).reshape(-1, 4, 3)[0] * 1000
    print("foot positions in body frame (mm) [env 0]:")
    for leg, name in enumerate(g.LEG_NAMES):
        hx, hy, _ = g.hip_origin(leg)
        expected = (hx * 1000 + g.NEUTRAL_X_MM, hy * 1000 + g.LEG_SIDE[leg] * g.NEUTRAL_Y_MM, -args.ride_mm)
        print(f"  {name}: sim ({feet_b[leg,0]:7.1f},{feet_b[leg,1]:7.1f},{feet_b[leg,2]:7.1f})  firmware ({expected[0]:7.1f},{expected[1]:7.1f},{expected[2]:7.1f})")
    q_now = T(robot.data.joint_pos)[0, joint_ids]
    q_tgt = T(robot.data.joint_pos_target)[0, joint_ids]
    print("joint err deg:", [round(math.degrees(v), 1) for v in (q_now - q_tgt).tolist()])
    print("applied torque Nm:", [round(v, 2) for v in T(robot.data.applied_torque)[0, joint_ids].tolist()])
    tilt = torch.rad2deg(torch.acos(-T(robot.data.projected_gravity_b)[:, 2].clamp(-1, 1))).max().item()
    ok = tilt < 10 and abs(mdp.measured_body_height(env).mean().item() - h) < 0.02
    print("STAND_CHECK", "PASS" if ok else "FAIL")
    env.close()
    return ok


if __name__ == "__main__":
    ok = main()
    app.close()
    sys.exit(0 if ok else 1)
