# Domino Actual CAD USD Checks

This folder is for checking the real Domino CAD export separately from the simplified Isaac physics prototypes.

Use `simulation/usd/Domino_Quadruped.usd` as the current visual source of truth. It is a mesh-based USD export from the Domino CAD package. It is not an articulated Isaac Lab robot yet: it has mesh visuals, but no rigid bodies, joints, actuators, or policy-ready contact model.

Do not use the standalone pin-linkage proxy screenshots as portfolio evidence of the final robot geometry. The older pin-linkage scene uses CAD-derived pivot coordinates, but its visible bodies are cubes and foot spheres. The current CAD-linkage audit path attaches the Domino STL link package and rejects the scene if the expected `29` link meshes, `135508` visible triangles, or hidden-proxy checks do not pass.

## Audit

Run the audit with a Python that can import `pxr`:

```powershell
$env:ISAAC_SIM_ROOT = "<isaac-sim-root>"
<python> simulation/isaac/prototypes/actual_cad/domino_actual_cad_usd.py `
  --cad-usd simulation/usd/Domino_Quadruped.usd `
  --write-wrapper `
  --output-usd <output-folder>/domino_actual_cad_visual.usda `
  --json-report <output-folder>/domino_actual_cad_audit.json
```

The wrapper is visual-only. It references the real CAD USD and applies a 0.001 scale into a meter-based stage for Isaac viewing. It is intended for visual verification before the mesh is attached to the training articulation.

## Current Asset Status

| Asset | Status |
| --- | --- |
| `simulation/usd/Domino_Quadruped.usd` | Real mesh CAD visual export; opens as 30 mesh prims, 0 rigid bodies, and 0 joints. |
| `cad/step/combined/Domino_URDF_Parts_Combined_Final.step` | Combined CAD reference export for geometry inspection and re-export. |
| `simulation/urdf/generated/Domino_URDF_Parts_Combined_Final_description/` | Generated URDF/STL export with named links and pivots; useful for topology and linkage mapping. |
| `simulation/isaac/out/cad_floating/domino_four_12_floating_body.usd` | Older generated proxy physics scene; cube/sphere visuals and PhysX revolute constraints. |
| `simulation/isaac/prototypes/pin_linkage/audit_domino_cad_linkage_visuals.py` | Current no-render audit for the runtime scene that attaches Domino STL mesh parts to the CAD-derived linkage proxy, verifies the known Domino triangle total, and verifies proxy visuals are hidden. |

## Next Work

The current simulator path attaches Domino STL mesh parts to the already tested twelve-actuator linkage bodies. Physics is still simplified at first, but the visible model is no longer the cube/sphere proxy when the actual-CAD path is enabled. The next work is to improve visual inspection renders, then decide whether the final training asset stays on this closed-linkage proxy or moves to a cleaner 12-DoF articulation validated against it.
