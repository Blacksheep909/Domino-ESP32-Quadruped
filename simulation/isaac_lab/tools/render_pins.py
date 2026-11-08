"""Close-up renders of the pinned linkage from a recording, every joint pin
marked: red = loop pin seen from one body, yellow = from the other body, cyan
= tree pins (servo axes, crank, knee, push rod and rod top pins).

    python tools/render_pins.py runs/visual/linkage_sweep.npz --leg BR --frames 0 40 80 --out sheet.png
    python tools/render_pins.py runs/visual/linkage_sweep.npz --leg BR --video out.mp4
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import render as R  # noqa: E402

LOOPS = json.load(open(os.path.join(ROOT, "assets", "domino", "loops.json")))["loops"]
TREE_PINS = ["thigh", "arm", "crank", "pushrod", "rod", "shank"]  # link frames sit on their pivot pins


def frame(sc, rec, names, t, leg, env, pins):
    import pyvista as pv

    T = {n: R.quat_to_T(rec["pos"][t, env, i], rec["quat"][t, env, i]) for i, n in enumerate(names)}
    sc.set_poses(T)
    for a in pins:
        sc.p.remove_actor(a)
    pins.clear()
    for lp in LOOPS:
        if not lp["name"].startswith(leg):
            continue
        p0 = (T[lp["body0"]] @ np.r_[lp["local0"], 1])[:3]
        p1 = (T[lp["body1"]] @ np.r_[lp["local1"], 1])[:3]
        pins.append(sc.p.add_mesh(pv.Sphere(radius=0.0034, center=p0), color="red"))
        pins.append(sc.p.add_mesh(pv.Sphere(radius=0.0024, center=p1), color="yellow"))
    for part in TREE_PINS:
        pins.append(sc.p.add_mesh(pv.Sphere(radius=0.0026, center=T[f"{leg}_{part}"][:3, 3]), color="cyan"))
    hip = T[f"{leg}_hip"][:3, 3]
    side = 1 if leg in ("FL", "BL") else -1
    sc.camera(hip + np.array([-0.04, 0.0, -0.115]), (0.0, side * 0.62, 0.03))
    if "info" in rec.files:
        sc.text(str(rec["info"][t][env]).split("\n") + [f"{leg} leg, outside view"])
    return sc.shot()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--leg", default="BR")
    ap.add_argument("--env", type=int, default=0)
    ap.add_argument("--frames", type=int, nargs="*", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--video", default=None)
    a = ap.parse_args()
    rec = np.load(a.recording, allow_pickle=True)
    names = list(rec["link_names"])
    sc = R.Scene(size=(800, 640))
    # hide the other three legs' clutter by keeping the camera close; draw all
    pins = []
    if a.out:
        frames = a.frames or list(np.linspace(0, rec["pos"].shape[0] - 1, 8).astype(int))
        imgs = [frame(sc, rec, names, t, a.leg, a.env, pins) for t in frames]
        rows = [np.hstack(imgs[i:i + 4]) for i in range(0, len(imgs), 4)]
        w = max(r.shape[1] for r in rows)
        rows = [np.pad(r, ((0, 0), (0, w - r.shape[1]), (0, 0)), constant_values=255) for r in rows]
        import imageio.v3 as iio

        iio.imwrite(a.out, np.vstack(rows))
        print("wrote", a.out)
    if a.video:
        import imageio.v2 as io2

        w = io2.get_writer(a.video, fps=float(rec["fps"]), codec="libx264", quality=8, macro_block_size=8)
        for t in range(rec["pos"].shape[0]):
            w.append_data(frame(sc, rec, names, t, a.leg, a.env, pins))
        w.close()
        print("wrote", a.video)


if __name__ == "__main__":
    main()
