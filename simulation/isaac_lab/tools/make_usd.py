"""Convert domino.urdf to USD and add the loop-closing pins from loops.json.

URDF cannot describe closed loops, so the kinematic tree comes from the URDF
importer and the two loop pins per leg (push rod -> bell crank at P3, parallel
rod -> shank at P4) are added here as spherical joints excluded from the
articulation tree. PhysX then solves them as constraints alongside the
articulation, which is how Isaac closes linkages.

    python tools/make_usd.py           (with the Isaac Lab Python)
"""

from __future__ import annotations

import json
import os
import sys

import h5py  # noqa: F401  # isort: skip
from isaaclab.app import AppLauncher

app = AppLauncher(headless=True).app

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ASSET = os.path.join(ROOT, "assets", "domino")
USD_DIR = os.path.join(ASSET, "usd")


def main():
    cfg = UrdfConverterCfg(
        asset_path=os.path.join(ASSET, "domino.urdf"),
        usd_dir=USD_DIR,
        usd_file_name="domino.usd",
        fix_base=False,
        merge_fixed_joints=False,
        self_collision=False,
        force_usd_conversion=True,
        make_instanceable=False,
        joint_drive=UrdfConverterCfg.JointDriveCfg(gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)),
    )
    conv = UrdfConverter(cfg)
    usd_path = conv.usd_path
    stage = Usd.Stage.Open(usd_path)
    # find link prims by name
    links = {}
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            links[prim.GetName()] = prim.GetPath()
    root = stage.GetDefaultPrim().GetPath()
    loops = json.load(open(os.path.join(ASSET, "loops.json")))["loops"]
    jroot = root.AppendChild("loop_pins")
    stage.DefinePrim(jroot, "Scope")
    for lp in loops:
        j = UsdPhysics.SphericalJoint.Define(stage, jroot.AppendChild(lp["name"]))
        j.CreateBody0Rel().SetTargets([links[lp["body0"]]])
        j.CreateBody1Rel().SetTargets([links[lp["body1"]]])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*lp["local0"]))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*lp["local1"]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr().Set(True)
    # TPE foot balls: soft (compliant) contact and TPE friction defaults. The
    # training randomises friction / restitution per robot around these.
    robot_cfg = json.load(open(os.path.join(ROOT, "config", "training.json")))["robot"]
    mat_path = root.AppendChild("materials").AppendChild("tpe_foot")
    stage.DefinePrim(mat_path.GetParentPath(), "Scope")
    mat = UsdShade.Material.Define(stage, mat_path)
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr().Set(0.85)
    pm.CreateDynamicFrictionAttr().Set(0.7)
    pm.CreateRestitutionAttr().Set(0.1)
    px = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
    px.CreateCompliantContactStiffnessAttr().Set(float(robot_cfg["foot_contact_stiffness_n_per_m"]))
    px.CreateCompliantContactDampingAttr().Set(float(robot_cfg["foot_contact_damping_n_s_per_m"]))
    px.CreateFrictionCombineModeAttr().Set("multiply")
    px.CreateRestitutionCombineModeAttr().Set("multiply")
    # The importer instances collision prims; materials cannot be bound inside an
    # instance, so make each foot's collision prim local first.
    for name, path in links.items():
        if name.endswith("_foot"):
            coll = stage.GetPrimAtPath(path.AppendChild("collisions"))
            if coll and coll.IsInstance():
                coll.SetInstanceable(False)
    bound = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI) and "_foot" in str(prim.GetPath()):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
            bound += 1
    print("TPE foot material bound to", bound, "colliders", flush=True)
    # The importer may write the stage as layers; save the root layer edits.
    stage.GetRootLayer().Save()
    # Real-world colours on the per-material visual meshes (base layer).
    sys.path.insert(0, HERE)
    import color_usd

    color_usd.main()
    print("USD", usd_path, "links", len(links), "loop pins", len(loops), flush=True)
    print("LINKS", sorted(links)[:40], flush=True)


if __name__ == "__main__":
    main()
    app.close()
