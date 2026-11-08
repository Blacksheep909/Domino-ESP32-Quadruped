"""Pin check for the linkage model, from the URDF alone (no physics).

Poses every leg with the firmware's own solver (domino_lab/linkage.py) for
servo angles across the whole travel, computes each loop pin's position from
BOTH bodies it connects, and reports the gap. Also renders a close-up of a leg
with every pin marked (red = from body 0, yellow = from body 1).

    python tools/check_pins.py --render out.png [--leg BR] [--upper 0] [--lower 0]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import render as R  # noqa: E402
from domino_lab import linkage as L  # noqa: E402

LOOPS = json.load(open(os.path.join(ROOT, "assets", "domino", "loops.json")))["loops"]


def joint_angles(leg: str, upper_deg: float, lower_deg: float, hip_rad: float = 0.0) -> dict:
    """URDF joint angles that put leg `leg` on the firmware linkage solution."""
    g = L.CadGeometry(leg, dtype=torch.float64)
    u = torch.tensor([upper_deg], dtype=torch.float64)
    lo = torch.tensor([lower_deg], dtype=torch.float64)
    _, ok, pose = g.foot_for_deltas(u, lo)
    # Each part's rotation from Studio's authored pose (math sense in x-z);
    # URDF +y rotations are the negative of x-z math rotations.
    a = {
        "thigh": pose["thigh_angle"].item(),
        "arm": pose["arm_angle"].item(),
        "crank": pose["crank_angle"].item(),
        "shank": pose["shank_angle"].item(),
    }
    # rod: parallel to the thigh; push rod: from P2 to P3
    a["rod"] = a["thigh"]
    p2, p3 = pose["P2"][0].numpy(), pose["P3"][0].numpy()
    a0 = math.atan2(g.upperClosureCoupler[1] - g.upperClosureDriver[1], g.upperClosureCoupler[0] - g.upperClosureDriver[0])
    a["pushrod"] = math.atan2(p3[1] - p2[1], p3[0] - p2[0]) - float(a0)
    q = {
        f"{leg}_hip_joint": hip_rad,
        f"{leg}_thigh_joint": -a["thigh"],
        f"{leg}_arm_joint": -a["arm"],
        f"{leg}_crank_joint": -a["crank"],
        f"{leg}_pushrod_joint": -(a["pushrod"] - a["arm"]),
        f"{leg}_rod_joint": -(a["rod"] - a["crank"]),
        f"{leg}_knee_joint": -(a["shank"] - a["thigh"]),
    }
    return q, bool(ok)


def pin_gaps(T: dict, leg: str):
    out = []
    for lp in LOOPS:
        if not lp["name"].startswith(leg):
            continue
        p0 = T[lp["body0"]] @ np.r_[lp["local0"], 1.0]
        p1 = T[lp["body1"]] @ np.r_[lp["local1"], 1.0]
        out.append((lp["name"], p0[:3], p1[:3], float(np.linalg.norm(p0[:3] - p1[:3]) * 1000)))
    return out


def sweep():
    _, joints = R.load_urdf()
    worst = 0.0
    n = 0
    for leg in L.LEG_ORDER:
        for up in np.linspace(-45, 45, 19):
            for lo in np.linspace(-45, 45, 19):
                q, ok = joint_angles(leg, up, lo)
                if not ok:
                    continue
                T = R.forward_kinematics(joints, q)
                for _, _, _, gap in pin_gaps(T, leg):
                    worst = max(worst, gap)
                    n += 1
    return worst, n


def render(out, leg, upper, lower):
    import pyvista as pv

    _, joints = R.load_urdf()
    q = {}
    for lg in L.LEG_ORDER:
        qq, _ = joint_angles(lg, upper if lg == leg else 0.0, lower if lg == leg else 0.0)
        q.update(qq)
    base = np.eye(4)
    base[2, 3] = 0.30
    T = R.forward_kinematics(joints, q, base)
    sc = R.Scene(size=(900, 700))
    sc.set_poses(T)
    for name, p0, p1, gap in pin_gaps(T, leg):
        sc.p.add_mesh(pv.Sphere(radius=0.0035, center=p0), color="red")
        sc.p.add_mesh(pv.Sphere(radius=0.0022, center=p1), color="yellow")
    hip = T[f"{leg}_hip"][:3, 3]
    side = 1 if leg in ("FL", "BL") else -1
    sc.camera(hip + np.array([-0.045, 0, -0.11]), (0.0, side * 0.42, 0.02))
    gaps = ", ".join(f"{n.split('_')[1]} {g:.2f} mm" for n, _, _, g in pin_gaps(T, leg))
    sc.text([f"{leg} linkage at upper {upper:+.0f} deg, lower {lower:+.0f} deg (servo deltas)", f"loop pin gaps: {gaps}"])
    sc.shot(out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", default=None)
    ap.add_argument("--leg", default="BR")
    ap.add_argument("--upper", type=float, default=0.0)
    ap.add_argument("--lower", type=float, default=0.0)
    a = ap.parse_args()
    worst, n = sweep()
    print(f"pin sweep: {n} pin checks over +-45 deg on both servos, all legs; worst gap {worst:.3f} mm")
    if a.render:
        render(a.render, a.leg, a.upper, a.lower)
        print("wrote", a.render)
