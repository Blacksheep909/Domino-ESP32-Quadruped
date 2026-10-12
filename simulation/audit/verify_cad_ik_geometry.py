"""Compare firmware IK dimensions with the manufactured holes in CAD STLs.

Run with a Python environment containing NumPy. The fitted centres come from
mesh vertices, not from the simulator's hand-entered linkage coordinates.
"""

import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

from run_pose_audit import CAD, ROOT, mesh_vertices, pin_circle


POINT_NAMES = (
    "lowerDrive", "upperDrive", "lowerPassive", "lowerClosureDriver",
    "upperClosureDriver", "upperClosureCoupler", "lowerCoupler",
    "lowerClosureDiagonal", "foot", "neutralFoot",
)
PIN_MESHES = {
    "right": {
        "lowerDrive": "DOM_P__18__1", "upperDrive": "DOM_P__19__1",
        "lowerClosureDriver": "DOM_P__18__1",
        "upperClosureDriver": "DOM_P__19__1",
        "upperClosureCoupler": "DOM_P__24__1",
        "lowerCoupler": "DOM_P__24__1",
        "lowerClosureDiagonal": "DOM_P__23__1",
        "foot": "DOM_P__22__1", "upperClosure": "DOM_P__20__1",
    },
    "left": {
        "lowerDrive": "DOM_P__16__1", "upperDrive": "DOM_P__17__1",
        "lowerClosureDriver": "DOM_P__16__1",
        "upperClosureDriver": "DOM_P__17__1",
        "upperClosureCoupler": "DOM_P__15__1",
        "lowerCoupler": "DOM_P__15__1",
        "lowerClosureDiagonal": "DOM_P__14__1",
        "foot": "DOM_P__26__1", "upperClosure": "DOM_P__27__1",
    },
}
LINKS = (
    ("upper crank", "upperDrive", "upperClosureDriver"),
    ("lower long arm", "lowerDrive", "lowerClosureDriver"),
    ("upper coupler arm", "lowerPassive", "upperClosureCoupler"),
    ("upper closure", "upperClosureDriver", "upperClosureCoupler"),
    ("coupler separation", "upperClosureCoupler", "lowerCoupler"),
    ("lower coupler arm", "lowerPassive", "lowerCoupler"),
    ("lower diagonal", "lowerCoupler", "lowerClosureDiagonal"),
    ("lower closure", "lowerClosureDriver", "lowerClosureDiagonal"),
    ("closure to foot", "lowerClosureDiagonal", "foot"),
)
HIP_X_MM, HIP_Z_MM = -68.5, 10.5  # rear joint origins in the CAD URDF


def firmware_points(side):
    source = (ROOT / "src/leg_controller.cpp").read_text(encoding="utf-8")
    name = "kRightCad" if side == "right" else "kLeftCad"
    block = re.search(rf"constexpr CadGeometry {name}\{{(.*?)\}};", source, re.S)
    if not block:
        raise ValueError(f"Missing {name} in firmware")
    pairs = re.findall(
        r"\{\s*([-+]?\d+(?:\.\d+)?)f\s*,\s*([-+]?\d+(?:\.\d+)?)f\s*\}",
        block.group(1),
    )
    if len(pairs) != len(POINT_NAMES):
        raise ValueError(f"Expected {len(POINT_NAMES)} point pairs in {name}, found {len(pairs)}")
    return {label: np.array([float(x), float(z)])
            for label, (x, z) in zip(POINT_NAMES, pairs)}


def vertices(side, part):
    return mesh_vertices(CAD / "meshes" / f"{PIN_MESHES[side][part]}.stl")


def fit_bore(side, part, local_xz):
    seed = local_xz + [HIP_X_MM, HIP_Z_MM]
    found = pin_circle(vertices(side, part), seed)
    if not found or found[0]["fitRmsMm"] > 0.01:
        raise AssertionError(f"No cylindrical bore at {side} {part}: {seed}")
    return np.array(found[0]["centreMm"]) - [HIP_X_MM, HIP_Z_MM]


def fit_foot_ball(side, local_xz):
    foot_y = -38.1 if side == "right" else 162.85
    hip_y = 0.0 if side == "right" else 124.75
    seed = np.array([local_xz[0] + HIP_X_MM, foot_y,
                     local_xz[1] + HIP_Z_MM])
    samples = vertices(side, "foot")
    samples = samples[np.linalg.norm(samples - seed, axis=1) < 13.0]
    if len(samples) < 100:
        raise AssertionError(f"Foot ball not found on {side} mesh")
    matrix = np.column_stack([2 * samples, np.ones(len(samples))])
    fit = np.linalg.lstsq(matrix, (samples * samples).sum(axis=1), rcond=None)[0]
    centre = fit[:3]
    radius = math.sqrt(fit[3] + centre @ centre)
    rms = np.std(np.linalg.norm(samples - centre, axis=1))
    if abs(radius - 12.0) > 0.01 or rms > 0.01:
        raise AssertionError(f"Foot ball fit failed on {side}: {radius=} {rms=}")
    if abs(abs(centre[1] - hip_y) - 38.1) > 0.01:
        raise AssertionError(f"Foot lateral offset differs from firmware on {side}")
    return centre[[0, 2]] - [HIP_X_MM, HIP_Z_MM]


def hip_geometry():
    path = CAD / "urdf/Domino_URDF_Parts_Combined_Final.urdf"
    tree = ET.parse(path).getroot()
    hips = np.array([
        [float(v) * 1000 for v in joint.find("origin").attrib["xyz"].split()]
        for joint in tree.findall("joint")
        if joint.find("parent").attrib["link"] == "base_link"
    ])
    if hips.shape != (4, 3):
        raise AssertionError(f"Expected four CAD hip joints, found {hips.shape}")
    source = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
    firmware = {}
    for name in ("BODY_HALF_LENGTH_X", "BODY_HALF_WIDTH_Y", "HIP_HEIGHT_Z"):
        value = re.search(rf"constexpr float {name} = ([-+]?\d+(?:\.\d+)?)f", source)
        if not value:
            raise ValueError(f"Missing {name} in firmware")
        firmware[name] = float(value.group(1))
    cad = {
        "BODY_HALF_LENGTH_X": (hips[:, 0].max() - hips[:, 0].min()) / 2,
        "BODY_HALF_WIDTH_Y": (hips[:, 1].max() - hips[:, 1].min()) / 2,
        "HIP_HEIGHT_Z": float(hips[0, 2]),
    }
    print("Body geometry, CAD / firmware (mm):")
    for name in cad:
        print(f"  {name}: {cad[name]:.6f} / {firmware[name]:.6f}")
        assert abs(cad[name] - firmware[name]) < 0.001
    assert np.ptp(hips[:, 2]) < 0.001
    return hips


def neutral_frame_geometry():
    source = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
    right, left = firmware_points("right"), firmware_points("left")
    expected = {
        "CAD_NEUTRAL_FOOT_X_FROM_HIP": (right["neutralFoot"][0] + left["neutralFoot"][0]) / 2,
        "CAD_NEUTRAL_FOOT_Y_FROM_HIP": 38.1,
        "CAD_NEUTRAL_FOOT_Z_FROM_HIP": (right["neutralFoot"][1] + left["neutralFoot"][1]) / 2,
    }
    print("\nNeutral foot frame, leg solver / body rotation (mm):")
    for name, solver_value in expected.items():
        match = re.search(rf"constexpr float {name} = ([-+]?\d+(?:\.\d+)?)f", source)
        if not match:
            raise ValueError(f"Missing {name} in body rotation")
        body_value = float(match.group(1))
        print(f"  {name}: {solver_value:.6f} / {body_value:.6f}")
        assert abs(solver_value - body_value) < 0.000001


def main():
    hip_geometry()
    neutral_frame_geometry()
    max_error = 0.0
    for side in ("right", "left"):
        code = firmware_points(side)
        measured = {}
        print(f"\n{side.capitalize()} linkage, CAD fit / firmware local to hip (mm):")
        for point in POINT_NAMES:
            if point in ("lowerPassive", "neutralFoot"):
                continue
            measured[point] = (fit_foot_ball(side, code[point]) if point == "foot"
                               else fit_bore(side, point, code[point]))
            error = float(np.linalg.norm(measured[point] - code[point]))
            max_error = max(max_error, error)
            print(f"  {point:23} CAD {measured[point][0]:9.4f},"
                  f" {measured[point][1]:9.4f}  code {code[point][0]:9.4f},"
                  f" {code[point][1]:9.4f}  error {error:.4f}")
        # The passive pin is on the same lower shaft in X/Z as the lower drive.
        measured["lowerPassive"] = measured["lowerDrive"]
        for point in ("upperClosureDriver", "upperClosureCoupler"):
            mating = fit_bore(side, "upperClosure", code[point])
            error = float(np.linalg.norm(mating - measured[point]))
            max_error = max(max_error, error)
            print(f"  mating upper {point:10} pin-centre gap {error:.4f} mm")
        for point in ("lowerClosureDriver", "lowerClosureDiagonal"):
            mating = fit_bore(side, "foot", code[point])
            error = float(np.linalg.norm(mating - measured[point]))
            max_error = max(max_error, error)
            print(f"  mating lower {point:10} pin-centre gap {error:.4f} mm")
        print("  Rigid distances, CAD / firmware (mm):")
        for label, a, b in LINKS:
            cad_length = float(np.linalg.norm(measured[a] - measured[b]))
            code_length = float(np.linalg.norm(code[a] - code[b]))
            print(f"    {label:21} {cad_length:8.3f} / {code_length:8.3f}")
            max_error = max(max_error, abs(cad_length - code_length))
    print(f"\nMaximum CAD/firmware point, mating or length error: {max_error:.4f} mm")
    assert max_error < 0.1, "Firmware IK geometry differs from CAD by at least 0.1 mm"


if __name__ == "__main__":
    main()
