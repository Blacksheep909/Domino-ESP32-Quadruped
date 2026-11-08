"""Fusion 360 script: export Domino's CAD bodies for the Isaac Lab model.

Open `Robot Dog V2 / 01 Master Assembly / body_v6` in Fusion and run this
(Utilities > Scripts, or through the Fusion MCP). It only reads the design: it
writes one STL per body in assembly coordinates (mm, Z up) plus manifest.json
with each body's Fusion mass and centre of mass, grouped into the rigid pieces
the simulator uses. Then run tools/build_urdf.py.

Only one front leg per side is exported (leg_v9:1 = front-left,
leg_v9_mirror:1 = front-right); the rear legs are identical translations.
"""

import json
import os
import struct

import adsk.core
import adsk.fusion

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "cad_export", "parts")

GROUPS = {
    "hip": ["servo:", "servo(Mirror)", "servo_hub_v2", "servo_arm", "servo3_arm", "push_rod", "pivot"],
    "thigh": ["L2-servo_clamp", "L2:", "L2(Mirror)", "spring_clamp", "MR63ZZ", "rod:", "rod(Mirror)", "rod_clamp", "L2_clamp_v2"],
    "shank": ["L3", "foot", "Generative_Bracket"],
    "body": ["clamp_template"],
}


def group_of(name):
    for group, keys in GROUPS.items():
        if any(name.startswith(key) for key in keys):
            return group
    return None


def write_body(body, filename):
    calc = body.meshManager.createMeshCalculator()
    calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.LowQualityTriangleMesh)
    mesh = calc.calculate()
    coords = mesh.nodeCoordinatesAsDouble
    idx = mesh.nodeIndices
    count = len(idx) // 3
    with open(filename, "wb") as f:
        f.write(b"world mm z-up".ljust(80, b" "))
        f.write(struct.pack("<I", count))
        for t in range(count):
            f.write(struct.pack("<3f", 0, 0, 0))
            for k in range(3):
                i = idx[3 * t + k]
                f.write(struct.pack("<3f", coords[3 * i] * 10, coords[3 * i + 1] * 10, coords[3 * i + 2] * 10))
            f.write(b"\x00\x00")


def run(_context: str):
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = design.rootComponent
    os.makedirs(OUT, exist_ok=True)
    for old in os.listdir(OUT):
        if old.endswith(".stl"):
            os.remove(os.path.join(OUT, old))
    manifest = []

    def export_occ(occ, label, group):
        for i in range(occ.bRepBodies.count):
            body = occ.bRepBodies.item(i)
            safe = occ.name.replace(":", "_").replace(" ", "_").replace("(", "").replace(")", "")
            filename = os.path.join(OUT, f"{label}__{group}__{safe}_{i}.stl")
            write_body(body, filename)
            props = body.physicalProperties
            com = props.centerOfMass
            manifest.append({
                "file": os.path.basename(filename), "label": label, "group": group, "part": occ.name,
                "mass": props.mass, "com_mm": [com.x * 10, com.y * 10, com.z * 10],
            })

    def walk(occ, label, group):
        export_occ(occ, label, group)
        for sub in occ.childOccurrences:
            walk(sub, label, group)

    for top in root.occurrences:
        if top.name.startswith("body_frame_v6"):
            for child in top.childOccurrences:
                walk(child, "body", "body")
        elif top.name in ("leg_v9:1", "leg_v9_mirror:1"):
            label = "legA" if top.name == "leg_v9:1" else "legB"
            for child in top.childOccurrences:
                group = group_of(child.name)
                if group is None:
                    print("UNGROUPED", child.name)
                    continue
                walk(child, label, group)
    with open(os.path.join(OUT, "manifest.json"), "w") as f:
        json.dump({"units": "mm", "frame": "Fusion assembly (Z up)",
                   "document": "Robot Dog V2/01 Master Assembly/body_v6", "parts": manifest}, f, indent=1)
    print("exported", len(manifest), "bodies, mass", round(sum(e["mass"] for e in manifest), 3), "kg")
