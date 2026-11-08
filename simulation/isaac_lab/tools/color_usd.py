"""Give the robot USD its real-world colours.

The URDF importer turns every STL into an OmniPBR "DefaultMaterial" in white and
drops the URDF <material> colours. build_urdf.py writes one visual mesh per
material class (``<link>__<class>.stl``); this sets each class's OmniPBR colour,
roughness and metalness in the base layer, so every robot instance renders like
the real dog: white printed parts, black carbon tubes, black DS servos,
aluminium horns, steel bearings, dark LiPo packs and black TPE feet.

    python tools/color_usd.py            (any Python with pxr / usd-core)

make_usd.py runs this automatically after a conversion.
"""

from __future__ import annotations

import os
import re

from pxr import Gf, Sdf, Usd, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(os.path.dirname(HERE), "assets", "domino", "usd", "configuration", "domino_base.usd")

# class -> (sRGB colour, roughness, metallic)
LOOKS = {
    "print": ((0.90, 0.90, 0.88), 0.55, 0.0),    # white PLA+ / PETG
    "carbon": ((0.05, 0.05, 0.055), 0.35, 0.0),  # carbon-fibre tube, slight sheen
    "servo": ((0.09, 0.09, 0.10), 0.45, 0.0),    # black DS servo case
    "horn": ((0.75, 0.76, 0.78), 0.30, 1.0),     # aluminium horn
    "steel": ((0.60, 0.61, 0.63), 0.25, 1.0),    # bearings
    "battery": ((0.13, 0.16, 0.24), 0.50, 0.0),  # CNHL pack wrap
    "tpe": ((0.035, 0.035, 0.035), 0.85, 0.0),   # matte TPE foot
}
CLASS_RE = re.compile(r"__([a-z]+)/")


def _set(shader: UsdShade.Shader, name: str, type_name, value) -> None:
    inp = shader.GetInput(name) or shader.CreateInput(name, type_name)
    inp.Set(value)


def main(path: str = BASE) -> int:
    stage = Usd.Stage.Open(path)
    count = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Shader":
            continue
        m = CLASS_RE.search(str(prim.GetPath()) + "/")
        if not m or m.group(1) not in LOOKS:
            continue
        rgb, rough, metal = LOOKS[m.group(1)]
        sh = UsdShade.Shader(prim)
        _set(sh, "diffuse_color_constant", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(*rgb))
        _set(sh, "reflection_roughness_constant", Sdf.ValueTypeNames.Float, float(rough))
        _set(sh, "metallic_constant", Sdf.ValueTypeNames.Float, float(metal))
        count += 1
    stage.GetRootLayer().Save()
    print(f"coloured {count} materials in {os.path.basename(path)}")
    return count


if __name__ == "__main__":
    main()
