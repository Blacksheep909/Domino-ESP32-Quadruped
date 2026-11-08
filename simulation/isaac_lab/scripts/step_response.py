"""How much does passive-pin armature slow the leg? Body pinned in the air;
the arm servo (which swings the shank through push rod and bell crank) and the
thigh servo each get a 20 degree step; report foot 10-90 % rise time and
overshoot for several pin armatures.

    python scripts/step_response.py --armatures 0 0.002 0.005
"""

from __future__ import annotations

import argparse
import os
import sys

import h5py  # noqa: F401  # isort: skip
from isaaclab.app import AppLauncher

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from domino_lab import compat as _compat  # noqa: E402  (no Isaac imports)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
ap = argparse.ArgumentParser()
ap.add_argument("--armatures", type=float, nargs="+", default=[0.0, 0.002, 0.005])
_compat.add_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(_compat.launcher_args(args)).app

import math  # noqa: E402

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402

from domino_lab import config as dcfg  # noqa: E402
from domino_lab.compat import T, quat_wxyz  # noqa: E402,F401
from domino_lab.assets import domino_cfg  # noqa: E402


def main():
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device="cuda:0"))
    robots = []
    for i, arm in enumerate(args.armatures):
        cfg = domino_cfg(dcfg.load()["robot"]).replace(prim_path=f"/World/R{i}")
        cfg.spawn.articulation_props.fix_root_link = True
        cfg.spawn.articulation_props.solver_position_iteration_count = 16
        cfg.init_state.pos = (i * 1.0, 0.0, 0.6)
        cfg.actuators["pins"].armature = arm
        robots.append(Articulation(cfg))
    sim.reset()
    for test, joint, step in (("arm servo +20 deg (shank swing)", "BR_arm_joint", 20.0), ("thigh servo +20 deg", "BR_thigh_joint", 20.0)):
        for r in robots:
            r.write_joint_state_to_sim(torch.zeros_like(T(r.data.joint_pos)), torch.zeros_like(T(r.data.joint_vel)))
            r.set_joint_position_target(torch.zeros_like(T(r.data.joint_pos)))
        for _ in range(100):
            for r in robots:
                r.write_data_to_sim()
            sim.step()
            for r in robots:
                r.update(0.005)
        traces = [[] for _ in robots]
        for k in range(120):
            for r in robots:
                tgt = torch.zeros_like(T(r.data.joint_pos))
                tgt[:, r.joint_names.index(joint)] = math.radians(step)
                r.set_joint_position_target(tgt)
                r.write_data_to_sim()
            sim.step()
            for i, r in enumerate(robots):
                r.update(0.005)
                foot = T(r.data.body_pos_w)[0, r.body_names.index("BR_foot")]
                traces[i].append(foot.cpu())
        print(f"STEP {test}:", flush=True)
        for arm, tr in zip(args.armatures, traces):
            tr = torch.stack(tr)
            d = torch.linalg.norm(tr - tr[0], dim=-1)
            final = d[-20:].mean()
            t10 = (d >= 0.1 * final).nonzero()[0].item() * 5
            t90 = (d >= 0.9 * final).nonzero()[0].item() * 5
            over = (d.max() / final - 1) * 100
            print(f"   pin armature {arm:6.4f}: foot moves {final * 1000:5.1f} mm, rise 10-90% {t90 - t10:3d} ms, overshoot {over:4.1f}%", flush=True)


if __name__ == "__main__":
    main()
    app.close()
