"""Build Domino's simulation model with the real pinned leg linkage.

Inputs
  assets/cad_export/parts/   v9 CAD bodies exported from Fusion (body_v6 /
                             leg_v9, assembly frame, mm) + manifest.json
  assets/domino/studio_linkage.json
                             pin centres measured by Studio's Virtual Lab
                             (same numbers as the firmware's kRightCad/kLeftCad)
Outputs
  assets/domino/domino.urdf  kinematic tree (servo axes actuated, linkage pins
                             passive)
  assets/domino/loops.json   the two loop-closing pins per leg, added to the
                             USD by tools/make_usd.py
  assets/domino/meshes/      one merged visual mesh per moving part

Frames. The Fusion assembly frame IS the robot frame (+x forward, +y left,
+z up; it is also Studio's frame). The body origin is the centre of the four
hip abduction axes. Every link frame sits on its pivot pin with no rotation,
and joint zero is Studio's authored stand pose, so the URDF at zero looks
exactly like Studio's sim at rest.

Parts per leg (Studio's names in brackets), each one rigid body:
  hip block   three servos + hub                            turns on the hip axis
  thigh       L2, L2 clamps, servo clamp, bearing [lower_driver]  lower-drive servo
  arm         servo3_arm (+ horn) [upper_driver]                  upper-drive servo
  push rod    push_rod [upper_closure]                  P2 -> P3 (loop pin at P3)
  bell crank  pivot [coupler]                            on the lower-drive axis
  rod         rod, rod clamp, lower L2 clamp [lower_diagonal]  P5 -> P4 (loop pin at P4)
  shank       L3, generative bracket, foot [lower_closure]      on the knee P6

The v9 assembly is saved in a different pose (thigh vertical, shank level), so
each part is moved by the rigid planar transform that takes its two pins in
the v9 pose onto the same two pins in Studio's pose. The v9 pins come from
the Fusion joints (servo axes, push rod and bell crank pins); the knee is
159.86 mm below the lower drive (thigh = parallel rod length).
"""

from __future__ import annotations

import json
import math
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

PARTS = os.path.join(ROOT, "assets", "cad_export", "parts")
OUT = os.path.join(ROOT, "assets", "domino")
MESH_OUT = os.path.join(OUT, "meshes")
STUDIO = json.load(open(os.path.join(OUT, "studio_linkage.json")))
ORIGIN = np.array(STUDIO["assemblyOrigin"]) + np.array([0.0, 0.0, 0.0105])  # body origin on the hip axes
FRONT_SHIFT_MM = 335.0
LEGS = ("FL", "FR", "BL", "BR")
SIDE = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
SOURCE = {1: "legB", -1: "legA"}  # Fusion leg_v9_mirror:1 = back-left, leg_v9:1 = back-right

# v9 assembly-pose pins (x, z mm). Same on both sides (mirror in y only).
THIGH = 159.86
V9 = {
    "U": (12.0, 10.5),
    "L": (-12.0, -10.5),
    "P2": (-22.6, 30.5),
    "P3": (-46.6, 9.5),
    "P5": (-52.0, -10.5),
    "P6": (-12.0, -10.5 - THIGH),
    "P4": (-52.0, -10.5 - THIGH),
}

PART_GROUPS = {
    "hip": ["servo:", "servo(Mirror)", "servo_hub_v2"],
    "thigh": ["L2-servo_clamp", "L2:", "L2(Mirror)", "spring_clamp", "MR63ZZ", "L2_clamp_v2:1"],
    "arm": ["servo3_arm"],
    "pushrod": ["push_rod"],
    "crank": ["pivot"],
    "rod": ["rod:", "rod(Mirror)", "rod_clamp", "L2_clamp_v2:2"],
    "shank": ["L3", "foot", "Generative_Bracket"],
    "base": ["clamp_template", "servo_arm"],  # hip servo horn is bolted to the body clamp
}
# Real materials (owner, 2026-11-08): body prints PLA+, leg prints PETG, both
# ~20 % infill with 2-3 walls (~55 % of solid); rods, rails and the twist plate
# carbon fibre; DS servos at spec weight; CNHL 1500 mAh 4S packs; TPE feet.
# Fusion's own materials were inconsistent (servos as solid aluminium, horns
# as steel), so masses are recomputed here: mass = volume x density x fill,
# or a fixed mass for bought parts. First matching prefix wins.
PRINT_FILL = 0.55
MATERIALS = [
    # (part-name prefix, density g/cm3, fill, fixed mass kg or None)
    ("servo_arm", 2.70, 1.0, None),          # aluminium horns
    ("servo:3", None, None, 0.070),          # DS 40 kg hip servo (spec ~70 g)
    ("servo(Mirror) (2)", None, None, 0.070),
    ("servo:", None, None, 0.065),           # DS 30 kg leg servos (spec ~65 g)
    ("servo(Mirror)", None, None, 0.065),
    ("MR63ZZ", 7.85, 1.0, None),             # steel bearing
    ("LIPO", None, None, 0.183),             # CNHL 1500 mAh 4S
    ("L0", 1.60, 1.0, None),                 # carbon body rails
    ("twist_plate_v1", 1.24, PRINT_FILL, None),  # PLA+ (owner: body parts PLA+; Fusion had carbon)
    ("L2:", 1.60, 1.0, None), ("L2(Mirror)", 1.60, 1.0, None),
    ("L3", 1.60, 1.0, None), ("rod:", 1.60, 1.0, None), ("rod(Mirror)", 1.60, 1.0, None),
    ("foot", 1.20, 1.0, None),               # TPE
    # body prints (PLA+)
    ("end_plate", 1.24, PRINT_FILL, None), ("body_v1_clamp", 1.24, PRINT_FILL, None),
    ("bat_4s_holder", 1.24, PRINT_FILL, None), ("twist_plate_lower", 1.24, PRINT_FILL, None),
    ("clamp_template", 1.24, PRINT_FILL, None),
    # servo mounts (owner: PLA): the hub that holds the three servos and the
    # clamps the servos bolt to
    ("servo_hub", 1.24, PRINT_FILL, None), ("L2-servo_clamp", 1.24, PRINT_FILL, None),
    # leg prints (PETG)
    ("", 1.27, PRINT_FILL, None),
]
# Two servo horns were lost to a name clash in the export (all named
# servo_arm:1); add aluminium horns of the surviving one's size.
HORN_KG = 1.817 * 2.70 / 1000.0
EXTRA_MASS = {"thigh": HORN_KG, "arm": HORN_KG}
ELECTRONICS_KG = 0.19  # ESP32, PCA9685, PCB, SZBK07 buck, relay, receiver, IMU, wiring


def part_mass(part_name, volume_cm3):
    for prefix, rho, fill, fixed in MATERIALS:
        if part_name.startswith(prefix):
            return fixed if fixed is not None else volume_cm3 * rho * fill / 1000.0
    raise KeyError(part_name)

# part: (v9 pin A, v9 pin B, studio pin A, studio pin B, frame pin)
POSE = {
    "thigh": ("L", "P6", "lower_drive", "lower_closure_diagonal", "lower_drive"),
    "shank": ("P6", "P4", "lower_closure_diagonal", "lower_closure_driver", "lower_closure_diagonal"),
    "rod": ("P5", "P4", "lower_coupler", "lower_closure_driver", "lower_coupler"),
    "crank": ("L", "P3", "lower_passive", "upper_closure_coupler", "lower_passive"),
    "pushrod": ("P2", "P3", "upper_closure_driver", "upper_closure_coupler", "upper_closure_driver"),
    "arm": ("U", "P2", "upper_drive", "upper_closure_driver", "upper_drive"),
    "hip": (None, None, None, None, "hip_origin"),
}
COLORS = {"base": "0.12 0.12 0.13 1", "hip": "0.18 0.18 0.2 1", "thigh": "0.62 0.65 0.69 1", "arm": "0.62 0.65 0.69 1",
          "pushrod": "0.62 0.65 0.69 1", "crank": "0.62 0.65 0.69 1", "rod": "0.62 0.65 0.69 1", "shank": "0.08 0.08 0.09 1"}
EFFORT = {"hip": 3.92, "thigh": 2.94, "arm": 2.94}  # DS servo 40 kg-cm hips, 30 kg-cm legs


# --------------------------------------------------------------------------
def read_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3, 3).astype(np.float64)


def decimate(tris, cell=0.0015):
    v = tris.reshape(-1, 3)
    keys = np.round(v / cell).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(uniq), 3))
    np.add.at(sums, inv, v)
    rep = sums / np.bincount(inv, minlength=len(uniq))[:, None]
    f = inv.reshape(-1, 3)
    ok = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = f[ok]
    _, idx = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
    return rep[f[np.sort(idx)]]


def write_stl(path, tris):
    tris = decimate(tris).astype(np.float32)
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    rec["n"], rec["v"] = nrm, tris.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"Domino sim mesh".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def mass_props(tris, mass):
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    vol6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = vol6.sum() / 6.0
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    box = lambda: (lo + hi) / 2, mass / 12.0 * np.diag([(hi - lo)[1] ** 2 + (hi - lo)[2] ** 2, (hi - lo)[0] ** 2 + (hi - lo)[2] ** 2, (hi - lo)[0] ** 2 + (hi - lo)[1] ** 2])  # noqa: E731
    if abs(vol) < 1e-12:
        return box[0](), box[1]
    if vol < 0:
        vol6, vol = -vol6, -vol
    com = ((a + b + c) * vol6[:, None]).sum(0) / (24.0 * vol)
    canon = np.array([[2, 1, 1], [1, 2, 1], [1, 1, 2]]) / 120.0
    A = np.stack([a, b, c], axis=2)  # (n, 3, 3) columns = vertices
    cov = np.einsum("n,nij,jk,nlk->il", vol6, A, canon, A) * (mass / vol)
    cov -= mass * np.outer(com, com)
    inertia = np.trace(cov) * np.eye(3) - cov
    if np.linalg.eigvalsh(inertia).min() <= 0:
        return box[0](), box[1]
    return com, inertia


def planar_transform(tris_mm, a, b, a2, b2):
    """Rigid x-z transform taking pins a,b (mm) onto a2,b2; y untouched."""
    ang = math.atan2(b2[1] - a2[1], b2[0] - a2[0]) - math.atan2(b[1] - a[1], b[0] - a[0])
    c, s = math.cos(ang), math.sin(ang)
    x = tris_mm[..., 0] - a[0]
    z = tris_mm[..., 2] - a[1]
    out = tris_mm.copy()
    out[..., 0] = a2[0] + c * x - s * z
    out[..., 2] = a2[1] + s * x + c * z
    return out


def group_of(part_name):
    for g, keys in PART_GROUPS.items():
        if any(part_name.startswith(k) for k in keys):
            return g
    return None


def load_parts():
    """{(label, group): (tris_mm, mass_kg, [(tris_mm, mass_kg) per part])}"""
    manifest = json.load(open(os.path.join(PARTS, "manifest.json")))["parts"]
    out = {}
    seen = set()
    for e in manifest:
        label = e["label"]
        g = "base" if label == "body" else group_of(e["part"])
        if g is None:
            print("skip", e["part"])
            continue
        tris = read_stl(os.path.join(PARTS, e["file"]))
        if e["file"] in seen:
            # Same occurrence name exported twice, so the file holds only the last
            # copy. Batteries: rebuild the other pack by moving the surviving mesh
            # to this entry's Fusion centre of mass. Horns: added as EXTRA_MASS.
            if not e["part"].startswith("LIPO"):
                continue
            first = next(x for x in manifest if x["file"] == e["file"])
            a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
            v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
            mesh_com = ((a + b + c) * v6[:, None]).sum(0) / (4.0 * v6.sum())
            if np.linalg.norm(mesh_com - np.array(e["com_mm"])) < np.linalg.norm(mesh_com - np.array(first["com_mm"])):
                tris = tris + (np.array(first["com_mm"]) - mesh_com)
            else:
                tris = tris + (np.array(e["com_mm"]) - mesh_com)
        seen.add(e["file"])
        a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
        vol_cm3 = abs(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0) / 1000.0
        mass = part_mass(e["part"], vol_cm3)
        cur = out.setdefault((label, g), {"tris": [], "mass": 0.0, "parts": [], "names": []})
        cur["tris"].append(tris)
        cur["mass"] += mass
        cur["parts"].append((tris, mass))
        cur["names"].append(e["part"])
    return {k: (np.concatenate(v["tris"]), v["mass"], v["parts"], v["names"]) for k, v in out.items()}


# Display materials (renderers only; physics uses MATERIALS above). Matched to
# the real robot: white printed parts, black carbon tubes, black DS servos,
# aluminium horns, steel bearings, dark LiPo packs, black TPE feet.
VISUAL_COLORS = {
    "print": "0.90 0.90 0.88 1",
    "carbon": "0.06 0.06 0.065 1",
    "servo": "0.10 0.10 0.11 1",
    "horn": "0.72 0.73 0.76 1",
    "steel": "0.55 0.56 0.58 1",
    "battery": "0.14 0.17 0.24 1",
    "tpe": "0.04 0.04 0.04 1",
}


def visual_class(part_name):
    n = part_name
    if n.startswith(("servo_arm", "servo3_arm")):
        return "horn"
    if n.startswith("servo_hub"):
        return "print"
    if n.startswith("servo"):
        return "servo"
    if n.startswith("MR63ZZ"):
        return "steel"
    if n.startswith("LIPO"):
        return "battery"
    if n.startswith("foot"):
        return "tpe"
    if n.startswith(("L0", "L3", "rod:", "rod(", "push_rod")) or (n.startswith("L2") and not n.startswith(("L2-", "L2_"))):
        return "carbon"
    return "print"


def write_visuals(link, groups):
    """Per-material visual meshes for one link -> URDF <visual> elements."""
    xml = ""
    for cls in sorted(groups):
        fn = f"visual/{link}__{cls}.stl"
        write_stl(os.path.join(MESH_OUT, fn), np.concatenate(groups[cls]))
        xml += (f'    <visual>\n      <geometry><mesh filename="meshes/{fn}"/></geometry>\n'
                f'      <material name="{cls}"><color rgba="{VISUAL_COLORS[cls]}"/></material>\n    </visual>\n')
    return xml


def combined_props(parts_local):
    """Mass, COM and inertia of rigidly joined parts, each uniform."""
    total = sum(m for _, m in parts_local)
    props = [(m, *mass_props(t, m)) for t, m in parts_local if m > 0]
    com = sum(m * c for m, c, _ in props) / total
    inertia = np.zeros((3, 3))
    for m, c, I in props:
        d = c - com
        inertia += I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return total, com, inertia


def studio_points(label):
    leg = next(l for l in STUDIO["legs"] if l["label"] == label)
    return {k: np.array(v) * 1000.0 for k, v in leg["points"].items()}  # mm, assembly frame


def to_body(p_mm):
    return np.asarray(p_mm) / 1000.0 - ORIGIN


def fmt(v):
    return " ".join(f"{x:.6g}" for x in np.asarray(v).ravel())


def inertial(mass, com, inertia):
    return (f'    <inertial>\n      <origin xyz="{fmt(com)}" rpy="0 0 0"/>\n      <mass value="{mass:.6g}"/>\n'
            f'      <inertia ixx="{inertia[0,0]:.6g}" ixy="{inertia[0,1]:.6g}" ixz="{inertia[0,2]:.6g}" '
            f'iyy="{inertia[1,1]:.6g}" iyz="{inertia[1,2]:.6g}" izz="{inertia[2,2]:.6g}"/>\n    </inertial>\n')


def box_between(p, q, width, depth, shrink=0.0):
    """Collision box from p to q (body frame, x-z plane). Returns xyz, rpy, size."""
    d = q - p
    length = math.hypot(d[0], d[2])
    pitch = math.atan2(-d[2], d[0])  # rotation about +y taking +x onto d
    return (p + q) / 2, (0.0, pitch, 0.0), (max(length - shrink, 0.005), depth, width)


def build(extra_body_mass=ELECTRONICS_KG):
    os.makedirs(os.path.join(MESH_OUT, "visual"), exist_ok=True)
    for root_, _, files in os.walk(MESH_OUT):
        for f in files:
            os.remove(os.path.join(root_, f))
    parts = load_parts()
    links, joints, loops = [], [], []

    # ---- base ----------------------------------------------------------------
    base_parts = [(to_body(t), m) for t, m in parts[("body", "base")][2]]
    base_groups = {}
    for (t, _), name in zip(parts[("body", "base")][2], parts[("body", "base")][3]):
        base_groups.setdefault(visual_class(name), []).append(to_body(t))
    for side in (1, -1):
        for (t, m), name in zip(parts[(SOURCE[side], "base")][2], parts[(SOURCE[side], "base")][3]):
            shifted = t + np.array([FRONT_SHIFT_MM, 0, 0])
            base_parts += [(to_body(t), m), (to_body(shifted), m)]
            base_groups.setdefault(visual_class(name), []).extend([to_body(t), to_body(shifted)])
    base_tris = np.concatenate([t for t, _ in base_parts])
    write_stl(os.path.join(MESH_OUT, "base.stl"), base_tris)
    lo, hi = base_tris.reshape(-1, 3).min(0), base_tris.reshape(-1, 3).max(0)
    size = hi - lo
    # Electronics as a box in the cage at the body centre.
    eb = np.array([0.06, 0.07, 0.03])
    box_I = extra_body_mass / 12.0 * np.diag([eb[1] ** 2 + eb[2] ** 2, eb[0] ** 2 + eb[2] ** 2, eb[0] ** 2 + eb[1] ** 2])
    frame_mass, frame_com, frame_I = combined_props(base_parts)
    total_base = frame_mass + extra_body_mass
    ecom = np.array([0.0, 0.0, 0.02])
    com = (frame_mass * frame_com + extra_body_mass * ecom) / total_base
    base_inertia = np.zeros((3, 3))
    for m, c, I in ((frame_mass, frame_com, frame_I), (extra_body_mass, ecom, box_I)):
        d = c - com
        base_inertia += I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    links.append('  <link name="base">\n' + write_visuals("base", base_groups)
                 + f'    <collision>\n      <origin xyz="{fmt((lo + hi) / 2)}" rpy="0 0 0"/>\n      <geometry><box size="{fmt(size)}"/></geometry>\n    </collision>\n'
                 + inertial(total_base, com, base_inertia) + "  </link>\n")

    summary = {}
    for leg in LEGS:
        side = SIDE[leg]
        src = SOURCE[side]
        front = leg[0] == "F"
        shift = np.array([FRONT_SHIFT_MM if front else 0.0, 0, 0])
        sp = studio_points(leg)
        frame = {}
        masses = {}
        for part, (va, vb, sa, sb, fpin) in POSE.items():
            _, _, plist, pnames = parts[(src, part)]
            origin = to_body(sp[fpin])
            frame[part] = origin

            def place(tris):
                tris = tris + shift
                if va is not None:
                    a = np.array(V9[va]) + [shift[0], 0]
                    b = np.array(V9[vb]) + [shift[0], 0]
                    tris = planar_transform(tris, a, b, sp[sa][[0, 2]], sp[sb][[0, 2]])
                return to_body(tris) - origin

            placed = [(place(t), m) for t, m in plist]
            local = np.concatenate([t for t, _ in placed])
            vgroups = {}
            for (t, _), name in zip(placed, pnames):
                vgroups.setdefault(visual_class(name), []).append(t)
            if EXTRA_MASS.get(part):
                # missing horn: a small aluminium disc on the drive axis
                placed.append((np.array([[[0.0, 0.0, 0.0], [0.004, 0.0, 0.0], [0.0, 0.004, 0.0]]]), EXTRA_MASS[part]))
            fn = f"{leg}_{part}.stl"
            write_stl(os.path.join(MESH_OUT, fn), local)
            mass, c, inertia = combined_props(placed)
            masses[part] = mass
            coll = ""
            P = lambda name: to_body(sp[name]) - origin  # noqa: E731
            if part == "hip":
                lo_, hi_ = local.reshape(-1, 3).min(0), local.reshape(-1, 3).max(0)
                coll = f'    <collision>\n      <origin xyz="{fmt((lo_ + hi_) / 2)}" rpy="0 0 0"/>\n      <geometry><box size="{fmt((hi_ - lo_) * 0.85)}"/></geometry>\n    </collision>\n'
            elif part == "thigh":
                xyz, rpy, sz = box_between(P("lower_drive"), P("lower_closure_diagonal"), 0.016, 0.014, 0.03)
                coll = f'    <collision>\n      <origin xyz="{fmt(xyz)}" rpy="{fmt(rpy)}"/>\n      <geometry><box size="{fmt(sz)}"/></geometry>\n    </collision>\n'
            elif part == "shank":
                xyz, rpy, sz = box_between(P("lower_closure_diagonal"), P("foot_tip"), 0.014, 0.014, 0.04)
                coll = f'    <collision>\n      <origin xyz="{fmt(xyz)}" rpy="{fmt(rpy)}"/>\n      <geometry><box size="{fmt(sz)}"/></geometry>\n    </collision>\n'
            links.append(f'  <link name="{leg}_{part}">\n' + write_visuals(f"{leg}_{part}", vgroups)
                         + coll + inertial(mass, c, inertia) + "  </link>\n")
        # foot sphere on the shank's foot tip
        foot_o = to_body(sp["foot_tip"])
        links.append(f'  <link name="{leg}_foot">\n    <visual>\n      <geometry><sphere radius="0.012"/></geometry>\n'
                     '      <material name="foot"><color rgba="0.05 0.05 0.05 1"/></material>\n    </visual>\n'
                     '    <collision>\n      <geometry><sphere radius="0.012"/></geometry>\n    </collision>\n'
                     + inertial(0.002, np.zeros(3), np.eye(3) * 1e-7) + "  </link>\n")

        def joint(name, parent, child, xyz, axis, lim, effort, kind="revolute"):
            if kind == "fixed":
                return (f'  <joint name="{name}" type="fixed">\n    <parent link="{parent}"/>\n    <child link="{child}"/>\n'
                        f'    <origin xyz="{fmt(xyz)}" rpy="0 0 0"/>\n  </joint>\n')
            return (f'  <joint name="{name}" type="revolute">\n    <parent link="{parent}"/>\n    <child link="{child}"/>\n'
                    f'    <origin xyz="{fmt(xyz)}" rpy="0 0 0"/>\n    <axis xyz="{axis}"/>\n'
                    f'    <limit lower="{lim[0]:.5f}" upper="{lim[1]:.5f}" effort="{effort}" velocity="8.0"/>\n'
                    '    <dynamics damping="0.0" friction="0.0"/>\n  </joint>\n')

        r = math.radians
        off = r(next(l for l in STUDIO["legs"] if l["label"] == leg)["neutralLinkageOffsetDeg"]["lower"])
        H = f"{leg}_hip"
        joints += [
            joint(f"{leg}_hip_joint", "base", H, frame["hip"], "1 0 0", (-r(32), r(32)), EFFORT["hip"]),
            # Servo axes. Firmware deltas are limited to +-45 deg; the margin
            # lets the simulated servo overshoot slightly without hitting a stop.
            joint(f"{leg}_thigh_joint", H, f"{leg}_thigh", frame["thigh"] - frame["hip"], "0 1 0", (-r(50) - off, r(50) - off), EFFORT["thigh"]),
            joint(f"{leg}_arm_joint", H, f"{leg}_arm", frame["arm"] - frame["hip"], "0 1 0", (-r(50), r(50)), EFFORT["arm"]),
            # Passive pins.
            joint(f"{leg}_crank_joint", H, f"{leg}_crank", frame["crank"] - frame["hip"], "0 1 0", (-r(120), r(120)), 0.0),
            joint(f"{leg}_pushrod_joint", f"{leg}_arm", f"{leg}_pushrod", frame["pushrod"] - frame["arm"], "0 1 0", (-r(120), r(120)), 0.0),
            joint(f"{leg}_rod_joint", f"{leg}_crank", f"{leg}_rod", frame["rod"] - frame["crank"], "0 1 0", (-r(120), r(120)), 0.0),
            joint(f"{leg}_knee_joint", f"{leg}_thigh", f"{leg}_shank", frame["shank"] - frame["thigh"], "0 1 0", (-r(120), r(120)), 0.0),
            joint(f"{leg}_foot_fixed", f"{leg}_shank", f"{leg}_foot", foot_o - frame["shank"], "", None, None, kind="fixed"),
        ]
        P3 = to_body(sp["upper_closure_coupler"])
        P4 = to_body(sp["lower_closure_driver"])
        loops += [
            {"name": f"{leg}_P3_pin", "body0": f"{leg}_pushrod", "body1": f"{leg}_crank",
             "local0": (P3 - frame["pushrod"]).tolist(), "local1": (P3 - frame["crank"]).tolist()},
            {"name": f"{leg}_P4_pin", "body0": f"{leg}_rod", "body1": f"{leg}_shank",
             "local0": (P4 - frame["rod"]).tolist(), "local1": (P4 - frame["shank"]).tolist()},
        ]
        summary[leg] = {k: round(v, 4) for k, v in masses.items()}

    urdf = ('<?xml version="1.0"?>\n<!-- Generated by simulation/isaac_lab/tools/build_urdf.py. The two loop-closing pins per leg '
            'are in loops.json (URDF cannot express closed loops). Do not edit. -->\n<robot name="domino">\n'
            + "".join(links) + "".join(joints) + "</robot>\n")
    with open(os.path.join(OUT, "domino.urdf"), "w", newline="\n") as f:
        f.write(urdf)
    with open(os.path.join(OUT, "loops.json"), "w", newline="\n") as f:
        json.dump({"loops": loops}, f, indent=2)
    total = total_base + sum(sum(v.values()) for v in summary.values()) + 4 * 0.002
    info = {"source": "Fusion body_v6 / leg_v9 meshes posed on Studio's measured pins; masses from real materials (MATERIALS)",
            "base_mass_kg": round(total_base, 4), "electronics_kg": extra_body_mass, "leg_part_masses_kg": summary, "total_mass_kg": round(total, 4),
            "loops": len(loops)}
    with open(os.path.join(OUT, "model_info.json"), "w", newline="\n") as f:
        json.dump(info, f, indent=2)
    return info


if __name__ == "__main__":
    print(json.dumps(build(float(sys.argv[1]) if len(sys.argv) > 1 else ELECTRONICS_KG), indent=2))
