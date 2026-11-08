"""Render Domino from the sim URDF without Isaac Sim's RTX renderer.

Uses VTK (pyvista) on plain OpenGL, so it works even where Isaac Sim's
viewport crashes (e.g. unsupported NVIDIA drivers), and on machines without
Isaac at all. Two sources of poses:

* joint angles  -> forward kinematics from domino.urdf (checks the model itself)
* a recording   -> per-step world pose of every link written by
                   scripts/record.py from the real Isaac physics

    python tools/render.py pose --height 280 --out stand.png
    python tools/render.py video runs/<run>/recordings/drive.npz --out drive.mp4

Needs: pip install pyvista imageio imageio-ffmpeg numpy
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

URDF = os.path.join(ROOT, "assets", "domino", "domino.urdf")
COLORS = {"base": "#2b2d30", "hip": "#3a3d42", "thigh": "#a7afb8", "arm": "#c3c9cf", "pushrod": "#8f989f",
          "crank": "#c3c9cf", "rod": "#8f989f", "shank": "#1c1d1f", "foot": "#111111"}


# --------------------------------------------------------------------------
# URDF forward kinematics
# --------------------------------------------------------------------------
def _rot_axis(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def load_urdf(path=URDF):
    tree = ET.parse(path).getroot()
    links = {}
    for link in tree.findall("link"):
        # A link can have several visuals (one mesh per material: prints,
        # carbon, servos, ...); keep them all with their URDF colours.
        visuals = []
        for vis in link.findall("visual"):
            m = vis.find("geometry/mesh")
            s = vis.find("geometry/sphere")
            c = vis.find("material/color")
            rgba = tuple(float(v) for v in c.get("rgba").split()) if c is not None else None
            if m is not None:
                visuals.append({"mesh": os.path.join(os.path.dirname(path), m.get("filename")), "sphere": None, "rgba": rgba})
            elif s is not None:
                visuals.append({"mesh": None, "sphere": float(s.get("radius")), "rgba": rgba})
        first = visuals[0] if visuals else {"mesh": None, "sphere": None}
        links[link.get("name")] = {"mesh": first["mesh"], "sphere": first["sphere"], "visuals": visuals}
    joints = []
    for j in tree.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        joints.append({
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": np.array([float(v) for v in o.get("xyz").split()]) if o is not None else np.zeros(3),
            "axis": np.array([float(v) for v in a.get("xyz").split()]) if a is not None and a.get("xyz") else np.array([1.0, 0, 0]),
        })
    return links, joints


def link_meshes(pv, info, fallback_color, sphere_res=16):
    """[(pyvista mesh, colour)] for every visual of a link."""
    out = []
    for v in info.get("visuals") or [{"mesh": info.get("mesh"), "sphere": info.get("sphere"), "rgba": None}]:
        if v["mesh"]:
            mesh = pv.read(v["mesh"])
        elif v["sphere"]:
            mesh = pv.Sphere(radius=v["sphere"], theta_resolution=sphere_res, phi_resolution=sphere_res)
        else:
            continue
        out.append((mesh, v["rgba"][:3] if v.get("rgba") else fallback_color))
    return out


def forward_kinematics(joints, q: dict, base_T=np.eye(4)):
    """World 4x4 transform of every link (URDF origins have rpy = 0)."""
    T = {"base": base_T}
    pending = list(joints)
    while pending:
        rest = []
        for j in pending:
            if j["parent"] not in T:
                rest.append(j)
                continue
            local = np.eye(4)
            local[:3, 3] = j["xyz"]
            if j["type"] == "revolute":
                local[:3, :3] = _rot_axis(j["axis"], q.get(j["name"], 0.0))
            T[j["child"]] = T[j["parent"]] @ local
        pending = rest
    return T


def quat_to_T(pos, quat_wxyz):
    w, x, y, z = quat_wxyz
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


# --------------------------------------------------------------------------
# Scene
# --------------------------------------------------------------------------
class Scene:
    def __init__(self, size=(960, 640), ground_z=0.0, show_axes=False, terrain=None):
        import pyvista as pv

        self.pv = pv
        self.links, self.joints = load_urdf()
        self.p = pv.Plotter(off_screen=True, window_size=size)
        self.p.set_background("#e9ece6", top="#c9d3cf")
        self.actors = {}
        for name, info in self.links.items():
            part = next((k for k in COLORS if name.endswith(k) or name == k), "base")
            actors = []
            for mesh, color in link_meshes(pv, info, COLORS[part]):
                actors.append(self.p.add_mesh(mesh, color=color, smooth_shading=False,
                                              specular=0.3, specular_power=12))
            if actors:
                self.actors[name] = actors
        if terrain is not None:
            self.p.add_mesh(terrain, color="#b9bfb5", smooth_shading=True)
        else:
            plane = pv.Plane(center=(0, 0, ground_z), i_size=40, j_size=40, i_resolution=200, j_resolution=200)
            self.p.add_mesh(plane, color="#c4c9c0", show_edges=True, edge_color="#aab0a6", line_width=0.5)
        self.p.enable_lightkit()
        if show_axes:
            self.p.add_axes()
        self.texts = []

    def set_poses(self, T: dict):
        for name, actors in self.actors.items():
            if name in T:
                for actor in actors:
                    actor.user_matrix = T[name]

    def camera(self, target, eye_offset=(0.75, -0.95, 0.45), up=(0, 0, 1)):
        target = np.asarray(target, float)
        self.p.camera_position = [tuple(target + np.asarray(eye_offset)), tuple(target), up]

    def text(self, lines):
        for t in self.texts:
            self.p.remove_actor(t)
        self.texts = [self.p.add_text("\n".join(lines), position="upper_left", font_size=10, color="#25382b")]

    def shot(self, path=None):
        self.p.render()  # pose-only changes do not mark the scene dirty
        img = self.p.screenshot(return_img=True)
        if path:
            import imageio.v3 as iio

            iio.imwrite(path, img)
        return img


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def render_pose(args):
    """Render the URDF at servo deltas (deg) via the firmware linkage solver;
    0, 0 is Studio's authored stand pose."""
    from check_pins import joint_angles
    from domino_lab import linkage as L

    q = {}
    for leg in L.LEG_ORDER:
        qq, _ = joint_angles(leg, args.upper, args.lower)
        q.update(qq)
    base = np.eye(4)
    base[2, 3] = 0.30
    T = forward_kinematics(load_urdf()[1], q, base)
    views = {"iso": (0.75, -0.95, 0.45), "side": (0.0, -1.25, 0.05), "front": (1.25, 0.0, 0.05), "top": (0.0, -0.01, 1.4)}
    scene = Scene(size=(900, 620))
    scene.set_poses(T)
    imgs = []
    for name, off in views.items():
        scene.camera(T["base"][:3, 3] - np.array([0, 0, 0.12]), off)
        scene.text([f"Domino pinned model - servo deltas upper {args.upper:+.0f}, lower {args.lower:+.0f} deg - {name} view"])
        imgs.append(scene.shot())
    grid = np.vstack([np.hstack(imgs[:2]), np.hstack(imgs[2:])])
    import imageio.v3 as iio

    iio.imwrite(args.out, grid)
    print("wrote", args.out)


def render_video(args):
    import imageio.v2 as imageio

    rec = np.load(args.recording, allow_pickle=True)
    names = list(rec["link_names"])
    pos, quat = rec["pos"], rec["quat"]  # (T, links, 3), (T, links, 4) wxyz
    fps = float(rec["fps"])
    env = int(args.env)
    terrain = None
    if "terrain_vertices" in rec.files:
        terrain = self_terrain(rec)
    scene = Scene(size=(args.width, args.height_px), terrain=terrain)
    info = rec["info"] if "info" in rec.files else None
    step = max(1, int(round(fps / args.fps)))
    writer = imageio.get_writer(args.out, fps=fps / step, codec="libx264", quality=8, macro_block_size=8)
    cam_target = None
    base_i = names.index("base")
    trail = []
    t0 = int(round(args.t0 * fps))
    t1 = pos.shape[0] if args.t1 is None else min(pos.shape[0], int(round(args.t1 * fps)))
    for t in range(t0, t1, step):
        T = {n: quat_to_T(pos[t, env, i], quat[t, env, i]) for i, n in enumerate(names)}
        scene.set_poses(T)
        target = pos[t, env, base_i].copy()
        target[2] = max(0.12, target[2] - 0.12)
        cam_target = target if cam_target is None else 0.85 * cam_target + 0.15 * target
        scene.camera(cam_target, tuple(args.eye))
        if info is not None and not args.clean:
            scene.text([str(info[t][env]) if info.ndim == 2 else str(info[t])])
        writer.append_data(scene.shot())
    writer.close()
    print("wrote", args.out)


def self_terrain(rec):
    import pyvista as pv

    v = rec["terrain_vertices"]
    f = rec["terrain_faces"]
    faces = np.hstack([np.full((len(f), 1), 3), f]).astype(np.int64).ravel()
    return pv.PolyData(v, faces)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("pose")
    a.add_argument("--upper", type=float, default=0.0)
    a.add_argument("--lower", type=float, default=0.0)
    a.add_argument("--out", default="pose.png")
    b = sub.add_parser("video")
    b.add_argument("recording")
    b.add_argument("--out", default="recording.mp4")
    b.add_argument("--env", type=int, default=0)
    b.add_argument("--fps", type=float, default=25.0)
    b.add_argument("--width", type=int, default=960)
    b.add_argument("--height_px", type=int, default=600)
    b.add_argument("--eye", type=float, nargs=3, default=(0.75, -0.95, 0.45))
    b.add_argument("--t0", type=float, default=0.0, help="start time, s")
    b.add_argument("--t1", type=float, default=None, help="end time, s")
    b.add_argument("--clean", action="store_true", help="no debug text overlay (showcase renders)")
    args = ap.parse_args()
    render_pose(args) if args.cmd == "pose" else render_video(args)


if __name__ == "__main__":
    main()
