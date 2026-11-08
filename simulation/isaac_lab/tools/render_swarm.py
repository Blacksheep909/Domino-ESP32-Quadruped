"""Render every robot of a multi-env recording at once (the "training swarm"
shot) with the VTK renderer: sky gradient, dark grid floor, slow camera move.

    python tools/render_swarm.py runs/visual/swarm.npz --out swarm.mp4 \
        --t0 1 --t1 6 --width 1080 --height_px 1920 --orbit 20

The recording must come from scripts/record.py (it stores env_origins).
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from render import COLORS, link_meshes, load_urdf, quat_to_T  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--out", required=True)
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--width", type=int, default=1080)
    ap.add_argument("--height_px", type=int, default=1920)
    ap.add_argument("--azimuth", type=float, default=-35.0, help="camera azimuth at t0, deg (0 = looking along +x)")
    ap.add_argument("--orbit", type=float, default=15.0, help="degrees of orbit over the clip")
    ap.add_argument("--elevation", type=float, default=32.0, help="deg above the floor")
    ap.add_argument("--distance", type=float, default=9.0, help="m from the swarm centre at t0")
    ap.add_argument("--push", type=float, default=0.85, help="distance multiplier reached at t1 (dolly in)")
    ap.add_argument("--follow", action="store_true", help="camera target follows the swarm's mean position")
    args = ap.parse_args()

    import imageio.v2 as imageio
    import pyvista as pv

    rec = np.load(args.recording, allow_pickle=True)
    names = list(rec["link_names"])
    pos, quat = rec["pos"], rec["quat"]
    origins = rec["env_origins"]
    fps = float(rec["fps"])
    n_env = pos.shape[1]
    pos = pos + origins[None, :, None, :]
    pos[..., 2] -= origins[None, :, None, 2]

    links, _ = load_urdf()
    p = pv.Plotter(off_screen=True, window_size=(args.width, args.height_px))
    p.set_background("#d9e2ea", top="#5f86b3")  # sky: pale horizon to blue zenith
    meshes = {}
    for name, info in links.items():
        part = next((k for k in COLORS if name.endswith(k) or name == k), "base")
        meshes[name] = link_meshes(pv, info, COLORS[part], sphere_res=12)
    actors = []  # (env, link index, actor)
    for e in range(n_env):
        for name, parts in meshes.items():
            if name not in names:
                continue
            for mesh, color in parts:
                a = p.add_mesh(mesh, color=color, specular=0.35, specular_power=14, smooth_shading=False)
                actors.append((e, names.index(name), a))
    # Floor: dark grey with a 1 m grid, large enough to reach the horizon.
    c = origins[:, :2].mean(0)
    floor = pv.Plane(center=(c[0], c[1], -0.001), i_size=1000, j_size=1000, i_resolution=1, j_resolution=1)
    p.add_mesh(floor, color="#8e9297", ambient=0.25, diffuse=0.8, specular=0.05)
    lines = []
    for k in range(-40, 41):
        lines.append(pv.Line((c[0] + k, c[1] - 40, 0.0), (c[0] + k, c[1] + 40, 0.0)))
        lines.append(pv.Line((c[0] - 40, c[1] + k, 0.0), (c[0] + 40, c[1] + k, 0.0)))
    p.add_mesh(pv.merge(lines), color="#a3a7ab", line_width=1.2)
    p.enable_lightkit()
    try:
        p.enable_anti_aliasing("ssaa")
    except Exception:
        pass
    try:
        p.enable_ssao(radius=0.4, bias=0.02, kernel_size=64)
    except Exception:
        pass

    t0 = int(round(args.t0 * fps))
    t1 = pos.shape[0] if args.t1 is None else min(pos.shape[0], int(round(args.t1 * fps)))
    step = max(1, int(round(fps / args.fps)))
    base_i = names.index("base")
    writer = imageio.get_writer(args.out, fps=fps / step, codec="libx264", quality=8, macro_block_size=8)
    target0 = np.array([c[0], c[1], 0.15])
    frames = list(range(t0, t1, step))
    smooth = None
    for i, t in enumerate(frames):
        for e, li, a in actors:
            a.user_matrix = quat_to_T(pos[t, e, li], quat[t, e, li])
        u = i / max(1, len(frames) - 1)
        ease = 0.5 - 0.5 * math.cos(math.pi * u)
        target = target0.copy()
        if args.follow:
            m = np.median(pos[t, :, base_i, :2], axis=0)
            target[:2] = m
        smooth = target if smooth is None else 0.9 * smooth + 0.1 * target
        az = math.radians(args.azimuth + args.orbit * ease)
        el = math.radians(args.elevation)
        d = args.distance * (1.0 + (args.push - 1.0) * ease)
        eye = smooth + d * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        p.camera_position = [tuple(eye), tuple(smooth), (0, 0, 1)]
        p.camera.view_angle = 32.0
        p.camera.clipping_range = (0.05, 1200.0)
        p.render()
        writer.append_data(p.screenshot(return_img=True))
    writer.close()
    print("wrote", args.out, len(frames), "frames")


if __name__ == "__main__":
    main()
