"""Loop-pin tightness inside the real training environment.

Runs Domino-Walk-v0 (domain randomisation on, flat or rough) with exploration-
like random foot actions, and reports loop-pin gaps for upright and for
tipped robots, plus throughput, for a given solver iteration count.

    python scripts/pin_stress.py --num_envs 1024 --iters 8 --action_std 0.6
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import h5py  # noqa: F401  # isort: skip
from isaaclab.app import AppLauncher

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from domino_lab import compat as _compat  # noqa: E402  (no Isaac imports)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

ap = argparse.ArgumentParser()
ap.add_argument("--num_envs", type=int, default=1024)
ap.add_argument("--iters", type=int, default=8)
ap.add_argument("--action_std", type=float, default=0.6)
ap.add_argument("--seconds", type=float, default=10.0)
ap.add_argument("--rough", action="store_true")
ap.add_argument("--pin_armature", type=float, default=None)
ap.add_argument("--small_mass_scale", type=float, default=1.0, help="scale arm/pushrod/crank/rod mass+inertia (taken from the hip)")
ap.add_argument("--dt", type=float, default=None)
_compat.add_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(_compat.launcher_args(args)).app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from domino_lab.compat import T, quat_wxyz  # noqa: E402,F401

from isaaclab.utils.math import quat_apply  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

import domino_lab  # noqa: E402,F401

LOOPS = json.load(open(os.path.join(ROOT, "assets", "domino", "loops.json")))["loops"]


def main():
    cfg = load_cfg_from_registry("Domino-Walk-v0", "env_cfg_entry_point")
    cfg.scene.num_envs = args.num_envs
    cfg.scene.robot.spawn.articulation_props.solver_position_iteration_count = args.iters
    if not args.rough:
        cfg.scene.terrain.terrain_type = "plane"
        cfg.scene.terrain.terrain_generator = None
        cfg.curriculum.terrain_levels = None
    if args.pin_armature is not None:
        cfg.scene.robot.actuators["pins"].armature = args.pin_armature
    if args.dt:
        cfg.decimation = int(round(0.02 / args.dt))
        cfg.sim.dt = args.dt
        cfg.sim.render_interval = cfg.decimation
        cfg.scene.contact_forces.update_period = args.dt
    env = gym.make("Domino-Walk-v0", cfg=cfg).unwrapped
    if args.small_mass_scale != 1.0:
        r = env.scene["robot"]
        view = r.root_physx_view
        m = view.get_masses().clone()
        I = view.get_inertias().clone()
        idx = torch.arange(env.num_envs)
        for leg in ("FL", "FR", "BL", "BR"):
            hip = r.body_names.index(f"{leg}_hip")
            added = 0.0
            for part in ("arm", "pushrod", "crank", "rod"):
                b = r.body_names.index(f"{leg}_{part}")
                added += (m[0, b] * (args.small_mass_scale - 1)).item()
                m[:, b] *= args.small_mass_scale
                I[:, b] *= args.small_mass_scale
            m[:, hip] -= added
        view.set_masses(m, idx)
        view.set_inertias(I, idx)
    env.reset()
    robot = env.scene["robot"]
    bid = {n: i for i, n in enumerate(robot.body_names)}
    dev = env.device
    loops = [(bid[l["body0"]], bid[l["body1"]], torch.tensor(l["local0"], device=dev), torch.tensor(l["local1"], device=dev)) for l in LOOPS]
    up_gaps, down_gaps, per_pin = [], [], []
    steps = int(args.seconds / env.step_dt)
    smooth = torch.zeros(env.num_envs, 12, device=dev)
    t0 = time.time()
    for k in range(steps):
        smooth = 0.8 * smooth + 0.2 * torch.randn(env.num_envs, 12, device=dev) * args.action_std * 2.2
        env.step(smooth)
        if k % 5 == 0:
            p, q = T(robot.data.body_pos_w), T(robot.data.body_quat_w)
            g = torch.stack([torch.linalg.norm(p[:, b0] + quat_apply(q[:, b0], l0.expand(p.shape[0], 3))
                                               - p[:, b1] - quat_apply(q[:, b1], l1.expand(p.shape[0], 3)), dim=-1)
                             for b0, b1, l0, l1 in loops], dim=-1).max(dim=-1).values * 1000
            per = torch.stack([torch.linalg.norm(p[:, b0] + quat_apply(q[:, b0], l0.expand(p.shape[0], 3))
                                                 - p[:, b1] - quat_apply(q[:, b1], l1.expand(p.shape[0], 3)), dim=-1)
                               for b0, b1, l0, l1 in loops], dim=-1) * 1000
            per_pin.append(per.cpu())
            upright = T(robot.data.projected_gravity_b)[:, 2] < -0.5
            up_gaps.append(g[upright].cpu())
            down_gaps.append(g[~upright].cpu())
    dt = time.time() - t0
    U = torch.cat(up_gaps)
    D = torch.cat(down_gaps) if sum(len(d) for d in down_gaps) else torch.zeros(1)
    qs = lambda x: [round(v, 3) for v in torch.quantile(x.float(), torch.tensor([0.5, 0.99, 0.999])).tolist()] + [round(x.max().item(), 2)]  # noqa: E731
    print(f"STRESS iters={args.iters} pin_armature={args.pin_armature} small_mass_x{args.small_mass_scale} dt={args.dt or 0.005} envs={args.num_envs}: {steps * env.num_envs / dt:,.0f} policy steps/s", flush=True)
    print(f"  upright robots ({len(U)} samples): pin gap median/p99/p99.9/max mm = {qs(U)}", flush=True)
    print(f"  tipped robots  ({len(D)} samples): pin gap median/p99/p99.9/max mm = {qs(D)}", flush=True)
    PP = torch.cat(per_pin)
    for i, l in enumerate(LOOPS):
        print(f"    {l['name']:10s} median {PP[:, i].median():.3f}  p99 {torch.quantile(PP[:, i], 0.99):.3f} mm", flush=True)
    fa = env.action_manager.get_term("feet").solve_failures
    print(f"  foot solves that failed (held previous): {fa.sum().item() / (steps * env.num_envs * 4) * 100:.3f}%", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    app.close()
