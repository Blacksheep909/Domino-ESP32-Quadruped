"""Physics check of the pinned linkage (no RL env).

Body pinned in the air; the hip, thigh (lower drive) and arm (upper drive)
servos of every leg follow a sweep; all other parts are passive and held only
by their pins. Reports, from PhysX body poses:
  * the gap at every loop pin (P3, P4),
  * the foot position error against the firmware solver for the same servo
    angles,
and saves a recording for tools/render.py.

    python scripts/linkage_physics_test.py --out runs/visual/linkage_sweep.npz [--num_envs 1] [--benchmark]
"""

from __future__ import annotations

import argparse
import json
import math
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
ap.add_argument("--out", default=None)
ap.add_argument("--num_envs", type=int, default=1)
ap.add_argument("--benchmark", action="store_true", help="free robots on the ground, random targets, measure steps/s")
_compat.add_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(_compat.launcher_args(args)).app

import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import quat_apply  # noqa: E402

from domino_lab import linkage as L  # noqa: E402
from domino_lab.compat import T, quat_wxyz  # noqa: E402,F401

ASSET = os.path.join(ROOT, "assets", "domino")
LOOPS = json.load(open(os.path.join(ASSET, "loops.json")))["loops"]


def robot_cfg(fixed: bool):
    return ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(ASSET, "usd", "domino.usd"),
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=1,
                fix_root_link=fixed,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.5 if fixed else 0.30)),
        actuators={
            "hip": ImplicitActuatorCfg(joint_names_expr=[".*_hip_joint"], stiffness=30.0, damping=0.8,
                                       effort_limit_sim=3.92, velocity_limit_sim=8.0, armature=0.01),
            "servos": ImplicitActuatorCfg(joint_names_expr=[".*_(thigh|arm)_joint"], stiffness=30.0, damping=0.8,
                                          effort_limit_sim=2.94, velocity_limit_sim=8.0, armature=0.01),
            "passive": ImplicitActuatorCfg(joint_names_expr=[".*_(crank|pushrod|rod|knee)_joint"], stiffness=0.0,
                                           damping=0.002, effort_limit_sim=100.0, armature=0.0002),
        },
    )


@configclass
class SceneCfg(InteractiveSceneCfg):
    pass


def main():
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device=args.device or "cuda:0"))
    scfg = SceneCfg(num_envs=args.num_envs, env_spacing=1.0)
    scfg.robot = robot_cfg(fixed=not args.benchmark)
    scene = InteractiveScene(scfg)
    ground = sim_utils.CuboidCfg(size=(40.0, 40.0, 0.1), collision_props=sim_utils.CollisionPropertiesCfg(),
                                 physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.8))
    ground.func("/World/ground", ground, translation=(0.0, 0.0, -0.05))
    sim.reset()
    robot: Articulation = scene["robot"]
    names = robot.body_names
    bid = {n: i for i, n in enumerate(names)}
    dev = robot.device
    loop_idx = [(lp["name"], bid[lp["body0"]], bid[lp["body1"]], torch.tensor(lp["local0"], device=dev), torch.tensor(lp["local1"], device=dev)) for lp in LOOPS]
    jn = robot.joint_names
    print("joints", len(jn), "bodies", len(names), flush=True)
    geo = {leg: L.CadGeometry(leg, device=dev) for leg in L.LEG_ORDER}
    hip_j = {leg: jn.index(f"{leg}_hip_joint") for leg in L.LEG_ORDER}
    thigh_j = {leg: jn.index(f"{leg}_thigh_joint") for leg in L.LEG_ORDER}
    arm_j = {leg: jn.index(f"{leg}_arm_joint") for leg in L.LEG_ORDER}

    def pin_gaps_mm():
        p, q = T(robot.data.body_pos_w), T(robot.data.body_quat_w)
        out = {}
        for name, b0, b1, l0, l1 in loop_idx:
            w0 = p[:, b0] + quat_apply(q[:, b0], l0.expand(p.shape[0], 3))
            w1 = p[:, b1] + quat_apply(q[:, b1], l1.expand(p.shape[0], 3))
            out[name] = torch.linalg.norm(w0 - w1, dim=-1) * 1000
        return out

    if args.benchmark:
        n = args.num_envs
        targets = T(robot.data.default_joint_pos).clone()
        steps = 0
        t0 = time.time()
        gap_hist = []
        for k in range(2000):
            if k % 25 == 0:
                for leg in L.LEG_ORDER:
                    up = torch.empty(n, device=dev).uniform_(-30, 25)
                    lo = torch.empty(n, device=dev).uniform_(-15, 40)
                    targets[:, thigh_j[leg]] = (lo - geo[leg].neutralLowerOffsetDeg) * math.pi / 180
                    targets[:, arm_j[leg]] = up * math.pi / 180
                    targets[:, hip_j[leg]] = torch.empty(n, device=dev).uniform_(-0.15, 0.15)
            robot.set_joint_position_target(targets)
            scene.write_data_to_sim()
            sim.step()
            scene.update(0.005)
            steps += 1
            if k % 20 == 0:
                g = torch.stack(list(pin_gaps_mm().values()), dim=-1).max(dim=-1).values  # per robot
                gap_hist.append(g.cpu())
        dt = time.time() - t0
        G = torch.stack(gap_hist)  # (samples, robots)
        q = torch.quantile(G.flatten().float(), torch.tensor([0.5, 0.99, 0.999]))
        bad = (G > 0.5).any(dim=0).float().mean().item() * 100
        tilt = torch.rad2deg(torch.acos(torch.clamp(-T(robot.data.projected_gravity_b)[:, 2], -1, 1)))
        print(f"BENCH {n} robots: {steps * n / dt:,.0f} physics steps/s = {steps * n / dt / 4:,.0f} policy steps/s (decimation 4)", flush=True)
        print(f"PINGAP median {q[0]:.3f} mm, p99 {q[1]:.3f} mm, p99.9 {q[2]:.3f} mm, max {G.max():.2f} mm; robots ever over 0.5 mm: {bad:.1f}%; "
              f"robots tipped over now: {(tilt > 60).float().mean().item() * 100:.1f}%", flush=True)
        big = G.max(dim=0).values > 0.5
        if big.any():
            print(f"PINGAP robots over 0.5 mm were tipped: {((tilt > 60) & big.to(tilt.device)).sum().item()} of {big.sum().item()}", flush=True)
        return

    # Sweep: per leg, upper and lower servos through their travel in turn.
    seq = []
    for up, lo in [(0, 0), (25, 0), (-30, 0), (0, 0), (0, 40), (0, -15), (0, 0), (20, 30), (-25, -10), (-20, 35), (0, 0)]:
        seq.append((up, lo))
    T_HOLD = 1.0
    P, Q, info, gaps_log, foot_err_log = [], [], [], [], []
    k = 0
    for i in range(len(seq) - 1):
        for s in range(int(T_HOLD / 0.005)):
            a = min(1.0, s / (0.6 / 0.005))
            up = seq[i][0] + (seq[i + 1][0] - seq[i][0]) * a
            lo = seq[i][1] + (seq[i + 1][1] - seq[i][1]) * a
            tgt = T(robot.data.default_joint_pos).clone()
            for leg in L.LEG_ORDER:
                tgt[:, thigh_j[leg]] = (lo - geo[leg].neutralLowerOffsetDeg) * math.pi / 180
                tgt[:, arm_j[leg]] = up * math.pi / 180
            robot.set_joint_position_target(tgt)
            scene.write_data_to_sim()
            sim.step()
            scene.update(0.005)
            k += 1
            if k % 4 == 0:
                g = pin_gaps_mm()
                gaps_log.append(max(v.max().item() for v in g.values()))
                # foot error vs firmware for the servo angles the physics actually reached
                errs = []
                for leg in L.LEG_ORDER:
                    q_th = T(robot.data.joint_pos)[:, thigh_j[leg]]
                    q_ar = T(robot.data.joint_pos)[:, arm_j[leg]]
                    lo_deg = q_th * 180 / math.pi + geo[leg].neutralLowerOffsetDeg
                    up_deg = q_ar * 180 / math.pi
                    foot_fw, _, _ = geo[leg].foot_for_deltas(up_deg, lo_deg)
                    # physics foot in the leg's planar frame: body frame minus hip origin, (x, z), mm
                    base_p, base_q = T(robot.data.root_pos_w), T(robot.data.root_quat_w)
                    rel = T(robot.data.body_pos_w)[:, bid[f"{leg}_foot"]] - base_p
                    hip_rel = T(robot.data.body_pos_w)[:, bid[f"{leg}_hip"]] - base_p
                    d = (rel - hip_rel) * 1000
                    errs.append(torch.linalg.norm(d[:, [0, 2]] - foot_fw, dim=-1).max().item())
                foot_err_log.append(max(errs))
                P.append((T(robot.data.body_pos_w) - scene.env_origins[:, None]).cpu().numpy())
                Q.append(quat_wxyz(robot.data.body_quat_w).cpu().numpy())  # recordings stay (w, x, y, z)
                info.append([f"servo sweep: upper {up:+5.1f} deg, lower {lo:+5.1f} deg\nworst loop-pin gap {gaps_log[-1]:.2f} mm, foot vs firmware {foot_err_log[-1]:.2f} mm"])
    print(f"SWEEP worst loop-pin gap {max(gaps_log):.3f} mm (mean {np.mean(gaps_log):.3f}); "
          f"worst foot error vs firmware solver {max(foot_err_log):.2f} mm (mean {np.mean(foot_err_log):.2f})", flush=True)
    if args.out:
        np.savez_compressed(args.out, link_names=np.array(names), pos=np.asarray(P, np.float32), quat=np.asarray(Q, np.float32),
                            info=np.array(info, dtype=object), fps=1.0 / 0.02)
        print("RECORDED", args.out, flush=True)


if __name__ == "__main__":
    main()
    app.close()
