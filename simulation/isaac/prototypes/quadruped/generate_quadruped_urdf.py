"""Generate the clean 12-DoF Domino quadruped URDF.

The raw CAD URDF contains duplicate link names and closed loops, so it is not a
good training articulation. This generator keeps the CAD-derived actuator names,
limits, shoulder axis signs, and hip locations, then emits a simple tree model
that Isaac Lab can import and sweep.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom


OUTPUT_PATH = Path(__file__).with_name("domino_quadruped_clean.urdf")

BODY_CENTER = (0.099000, 0.062375, 0.010500)
UPPER_LINK_LENGTH_M = 0.160
LOWER_LINK_LENGTH_M = 0.153

LEG_SPECS = [
    {
        "id": "dom_p_4_1",
        "hip_origin": (0.266500, 0.000000, 0.010500),
        "shoulder_axis": (-1.0, 0.0, 0.0),
        "upper_drive": (0.347000, -0.028000, 0.010500),
        "lower_limit": (-2.094395, 0.0),
        "lower_default": -0.75,
        "phase_deg": 0.0,
    },
    {
        "id": "dom_p_12_1",
        "hip_origin": (0.266500, 0.124750, 0.010500),
        "shoulder_axis": (1.0, 0.0, 0.0),
        "upper_drive": (0.347000, 0.152750, 0.010500),
        "lower_limit": (-2.094395, 0.0),
        "lower_default": -0.75,
        "phase_deg": 90.0,
    },
    {
        "id": "dom_p_25_1",
        "hip_origin": (-0.068500, 0.124750, 0.010500),
        "shoulder_axis": (1.0, 0.0, 0.0),
        "upper_drive": (0.012000, 0.152750, 0.010500),
        "lower_limit": (-2.094395, 0.0),
        "lower_default": -0.75,
        "phase_deg": 180.0,
    },
    {
        "id": "dom_p_21_1",
        "hip_origin": (-0.068500, 0.000000, 0.010500),
        "shoulder_axis": (-1.0, 0.0, 0.0),
        "upper_drive": (0.012000, -0.028000, 0.010500),
        "lower_limit": (-0.523599, 1.570796),
        "lower_default": 0.25,
        "phase_deg": 270.0,
    },
]


def add_material(robot: ET.Element, name: str, rgba: str) -> None:
    material = ET.SubElement(robot, "material", {"name": name})
    ET.SubElement(material, "color", {"rgba": rgba})


def xyz(values: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.6f}" for value in values)


def origin(parent: ET.Element, values: tuple[float, float, float], rpy: str = "0 0 0") -> None:
    ET.SubElement(parent, "origin", {"xyz": xyz(values), "rpy": rpy})


def inertial(
    link: ET.Element,
    mass: float,
    inertia: tuple[float, float, float],
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    element = ET.SubElement(link, "inertial")
    origin(element, origin_xyz)
    ET.SubElement(element, "mass", {"value": f"{mass:.6f}"})
    ET.SubElement(
        element,
        "inertia",
        {
            "ixx": f"{inertia[0]:.8f}",
            "ixy": "0",
            "ixz": "0",
            "iyy": f"{inertia[1]:.8f}",
            "iyz": "0",
            "izz": f"{inertia[2]:.8f}",
        },
    )


def box_geometry(parent: ET.Element, size: tuple[float, float, float]) -> None:
    geometry = ET.SubElement(parent, "geometry")
    ET.SubElement(geometry, "box", {"size": xyz(size)})


def cylinder_geometry(parent: ET.Element, radius: float, length: float) -> None:
    geometry = ET.SubElement(parent, "geometry")
    ET.SubElement(geometry, "cylinder", {"radius": f"{radius:.6f}", "length": f"{length:.6f}"})


def sphere_geometry(parent: ET.Element, radius: float) -> None:
    geometry = ET.SubElement(parent, "geometry")
    ET.SubElement(geometry, "sphere", {"radius": f"{radius:.6f}"})


def visual_box(link: ET.Element, size: tuple[float, float, float], material: str, offset=(0.0, 0.0, 0.0)) -> None:
    visual = ET.SubElement(link, "visual")
    origin(visual, offset)
    box_geometry(visual, size)
    ET.SubElement(visual, "material", {"name": material})


def collision_box(link: ET.Element, size: tuple[float, float, float], offset=(0.0, 0.0, 0.0)) -> None:
    collision = ET.SubElement(link, "collision")
    origin(collision, offset)
    box_geometry(collision, size)


def visual_cylinder(link: ET.Element, radius: float, length: float, material: str, offset=(0.0, 0.0, 0.0)) -> None:
    visual = ET.SubElement(link, "visual")
    origin(visual, offset)
    cylinder_geometry(visual, radius, length)
    ET.SubElement(visual, "material", {"name": material})


def collision_cylinder(link: ET.Element, radius: float, length: float, offset=(0.0, 0.0, 0.0)) -> None:
    collision = ET.SubElement(link, "collision")
    origin(collision, offset)
    cylinder_geometry(collision, radius, length)


def visual_sphere(link: ET.Element, radius: float, material: str) -> None:
    visual = ET.SubElement(link, "visual")
    origin(visual, (0.0, 0.0, 0.0))
    sphere_geometry(visual, radius)
    ET.SubElement(visual, "material", {"name": material})


def collision_sphere(link: ET.Element, radius: float) -> None:
    collision = ET.SubElement(link, "collision")
    origin(collision, (0.0, 0.0, 0.0))
    sphere_geometry(collision, radius)


def add_joint(
    robot: ET.Element,
    name: str,
    joint_type: str,
    parent: str,
    child: str,
    origin_xyz: tuple[float, float, float],
    axis: tuple[float, float, float] | None = None,
    limit: tuple[float, float] | None = None,
) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": joint_type})
    origin(joint, origin_xyz)
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": xyz(axis)})
    if limit is not None:
        ET.SubElement(
            joint,
            "limit",
            {
                "lower": f"{limit[0]:.6f}",
                "upper": f"{limit[1]:.6f}",
                "effort": "8.0",
                "velocity": "6.0",
            },
        )
        ET.SubElement(joint, "dynamics", {"damping": "0.05", "friction": "0.0"})


def add_base_link(robot: ET.Element) -> None:
    link = ET.SubElement(robot, "link", {"name": "base_link"})
    inertial(link, mass=1.20, inertia=(0.0100, 0.0200, 0.0250))
    visual_box(link, (0.430, 0.205, 0.050), "domino_carbon")
    collision_box(link, (0.430, 0.205, 0.050))


def add_leg(robot: ET.Element, spec: dict) -> None:
    leg_id = spec["id"]
    hip_origin = tuple(spec["hip_origin"][index] - BODY_CENTER[index] for index in range(3))
    upper_drive_offset = tuple(spec["upper_drive"][index] - spec["hip_origin"][index] for index in range(3))

    hip_link = f"{leg_id}_hip_carriage"
    upper_link = f"{leg_id}_upper_link"
    lower_link = f"{leg_id}_lower_link"
    foot_link = f"{leg_id}_foot"

    hip = ET.SubElement(robot, "link", {"name": hip_link})
    inertial(hip, mass=0.10, inertia=(0.0004, 0.0004, 0.0004), origin_xyz=(0.0, 0.0, -0.010))
    visual_box(hip, (0.060, 0.055, 0.030), "domino_joint", offset=(0.0, 0.0, -0.010))
    collision_box(hip, (0.060, 0.055, 0.030), offset=(0.0, 0.0, -0.010))

    upper = ET.SubElement(robot, "link", {"name": upper_link})
    inertial(upper, mass=0.12, inertia=(0.0005, 0.0005, 0.00008), origin_xyz=(0.0, 0.0, -0.080))
    visual_cylinder(upper, 0.012, UPPER_LINK_LENGTH_M, "domino_printed", offset=(0.0, 0.0, -0.080))
    visual_cylinder(upper, 0.004, 0.145, "domino_carbon", offset=(0.018, 0.0, -0.080))
    visual_cylinder(upper, 0.004, 0.145, "domino_carbon", offset=(-0.018, 0.0, -0.080))
    collision_cylinder(upper, 0.014, UPPER_LINK_LENGTH_M, offset=(0.0, 0.0, -0.080))

    lower = ET.SubElement(robot, "link", {"name": lower_link})
    inertial(lower, mass=0.10, inertia=(0.0004, 0.0004, 0.00006), origin_xyz=(0.0, 0.0, -0.0765))
    visual_cylinder(lower, 0.010, LOWER_LINK_LENGTH_M, "domino_printed", offset=(0.0, 0.0, -0.0765))
    visual_cylinder(lower, 0.0035, 0.135, "domino_carbon", offset=(0.016, 0.0, -0.0765))
    visual_cylinder(lower, 0.0035, 0.135, "domino_carbon", offset=(-0.016, 0.0, -0.0765))
    collision_cylinder(lower, 0.012, LOWER_LINK_LENGTH_M, offset=(0.0, 0.0, -0.0765))

    foot = ET.SubElement(robot, "link", {"name": foot_link})
    inertial(foot, mass=0.03, inertia=(0.00004, 0.00004, 0.00004))
    visual_sphere(foot, 0.024, "domino_foot")
    collision_sphere(foot, 0.024)

    add_joint(
        robot,
        name=f"{leg_id}_shoulder_ab_ad",
        joint_type="revolute",
        parent="base_link",
        child=hip_link,
        origin_xyz=hip_origin,
        axis=spec["shoulder_axis"],
        limit=(-0.523599, 0.523599),
    )
    add_joint(
        robot,
        name=f"{leg_id}_upper_pitch",
        joint_type="revolute",
        parent=hip_link,
        child=upper_link,
        origin_xyz=upper_drive_offset,
        axis=(0.0, 1.0, 0.0),
        limit=(-0.523599, 1.047198),
    )
    add_joint(
        robot,
        name=f"{leg_id}_lower_linkage",
        joint_type="revolute",
        parent=upper_link,
        child=lower_link,
        origin_xyz=(0.0, 0.0, -UPPER_LINK_LENGTH_M),
        axis=(0.0, 1.0, 0.0),
        limit=spec["lower_limit"],
    )
    add_joint(
        robot,
        name=f"{leg_id}_foot_fixed",
        joint_type="fixed",
        parent=lower_link,
        child=foot_link,
        origin_xyz=(0.0, 0.0, -LOWER_LINK_LENGTH_M),
    )


def build_robot() -> ET.Element:
    robot = ET.Element("robot", {"name": "domino_quadruped_clean"})
    add_material(robot, "domino_carbon", "0.08 0.09 0.10 1.0")
    add_material(robot, "domino_printed", "0.65 0.68 0.70 1.0")
    add_material(robot, "domino_joint", "0.15 0.28 0.42 1.0")
    add_material(robot, "domino_foot", "0.02 0.02 0.02 1.0")
    add_base_link(robot)
    for spec in LEG_SPECS:
        add_leg(robot, spec)
    return robot


def main() -> None:
    robot = build_robot()
    rough_xml = ET.tostring(robot, encoding="utf-8")
    pretty_xml = minidom.parseString(rough_xml).toprettyxml(indent="  ")
    OUTPUT_PATH.write_text(pretty_xml, encoding="utf-8")
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
