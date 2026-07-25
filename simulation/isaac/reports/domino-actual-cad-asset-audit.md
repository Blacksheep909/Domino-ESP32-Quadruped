# Domino Actual CAD Asset Audit

This audit separates the real Domino CAD visual export from the simplified Isaac physics prototypes.

## Source Of Truth

Use `simulation/usd/Domino_Quadruped.usd` as the current visual source of truth for Domino. It opens as a mesh-based CAD visual export:

| Check | Result |
| --- | --- |
| Default prim | `/Domino_USD_Parts_Combined_Final` |
| Stage units | `metersPerUnit = 0.001` |
| Up axis | `Z` |
| Mesh prims | `30` |
| Cube prims | `0` |
| Sphere prims | `0` |
| Rigid bodies | `0` |
| Joints | `0` |

This confirms it is actual CAD visual geometry, not the proxy linkage scene. It is not yet a policy-ready Isaac robot because it has no articulation, rigid-body setup, contact model, actuator config, or closed-loop linkage constraints.

The policy bring-up path now uses exported Domino mesh parts as visuals on top of the controllable CAD-derived linkage proxy. The current moving-scene audit attaches `29` Domino STL meshes with `135508` total triangles to the proxy rigid bodies, keeps `12` actuator drives, authors `28` revolute joints, and hides the proxy robot cubes/spheres so the rendered robot is not the blocky debug scaffold. Those proxy collision/debug shapes are also marked as guide-purpose transparent geometry when actual CAD visuals are enabled.

The runtime visual audit now fails if the scene only exposes proxy geometry, a partial CAD attachment, or a different STL package with the wrong Domino triangle total:

| Check | Result |
| --- | --- |
| Runtime visual fidelity | `actual_cad_stl_visuals_on_proxy_physics` |
| Expected Domino STL parts | `29` |
| Runtime Domino STL mesh prims | `29` |
| Visible Domino STL mesh prims | `29` |
| Visible Domino STL triangles | `135508` |
| Visible proxy cubes | `0` |
| Visible proxy spheres | `0` |
| Proxy visual render guard | `guide-purpose + transparent` |
| Runtime visual bounds | about `0.611 m x 0.238 m x 0.391 m` |

This still means the physics model is a proxy linkage with CAD-derived pivots and hidden proxy collision bodies. It does **not** mean the raw CAD USD has become a clean closed-chain articulation. For visual checks and portfolio screenshots, use a single robot scene and the actual-CAD audit path; do not use multi-env training screenshots, proxy-only debug renders, or renderer-crash screenshots as evidence of the Domino geometry. If the viewport shows several dark blocky robots, assume it is a training/debug view until this audit confirms the visible STL mesh count and hidden proxy counts.

## Proxy Scene Contrast

The current floating linkage training scene generated under `simulation/isaac/out/cad_floating/` is a physics proxy:

| Check | Result |
| --- | --- |
| Mesh prims | `0` |
| Cube prims | `22` |
| Sphere prims | `4` |
| Rigid bodies | `21` |
| PhysX revolute joints | `28` |

That proxy is still useful because it tests the twelve actuator channels and closed-loop pin constraints without relying on the broken raw URDF topology. It should not be used for visual evidence of the finished Domino CAD.

## Isaac Visual Path

Use `simulation/isaac/prototypes/actual_cad/domino_actual_cad_usd.py` to audit the real CAD export and create a meter-based visual wrapper:

```powershell
$env:ISAAC_SIM_ROOT = "<isaac-sim-root>" # ISAAC_PATH from Isaac's python wrapper also works
<python> simulation/isaac/prototypes/actual_cad/domino_actual_cad_usd.py `
  --cad-usd simulation/usd/Domino_Quadruped.usd `
  --write-wrapper `
  --output-usd <output-folder>/domino_actual_cad_visual.usda `
  --json-report <output-folder>/domino_actual_cad_audit.json
```

The wrapper is visual-only. The runtime `DirectRLEnv` path now attaches the exported Domino STL mesh groups to the simplified linkage bodies. The remaining work is not visual identity; it is improving the closed-chain physics/contact model and training policy quality while keeping the proxy colliders and loop constraints stable.

Use `simulation/isaac/prototypes/pin_linkage/audit_domino_cad_linkage_visuals.py` to audit the runtime CAD-linkage visual attachment without launching the renderer:

```powershell
$env:ISAAC_SIM_ROOT = "<isaac-sim-root>" # ISAAC_PATH from Isaac's python wrapper also works
<python> simulation/isaac/prototypes/pin_linkage/audit_domino_cad_linkage_visuals.py `
  --output-usd <output-folder>/domino_actual_cad_linkage_visual_audit.usda `
  --json-report <output-folder>/domino_actual_cad_linkage_visual_audit.json
```

That audit builds the current linkage scene in memory, checks the real CAD USD source, verifies the expected Domino STL parts and triangle total are visible, and verifies the proxy cubes/spheres are hidden and marked as non-render debug geometry.
