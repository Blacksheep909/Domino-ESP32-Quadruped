# CAD Exports

This folder contains neutral STEP exports for the current Domino prototype. They are included for inspection, measurement, adaptation, and CAD-to-simulation work.

> Work in progress: these are reference exports, not a complete manufacturing package. Validate clearances, servo fit, print tolerances, fastener sizes, and safe joint travel before building from them.

## STEP Files

| Path | Purpose |
| --- | --- |
| `step/assembly/Domino_URDF_Parts.step` | Full assembly-style STEP export for viewing the complete robot. |
| `step/combined/Domino_URDF_Parts_Combined.step` | Combined model export from the CAD-to-URDF trail. |
| `step/combined/Domino_URDF_Parts_Combined_Final.step` | Main combined reference export for the current documentation state. |
| `step/parts/Body.step` | Central body and electronics cage reference. |
| `step/parts/FL.step` | Front-left leg module. |
| `step/parts/FR.step` | Front-right leg module. |
| `step/parts/BL.step` | Back-left leg module. |
| `step/parts/BR.step` | Back-right leg module. |

Start with `step/combined/Domino_URDF_Parts_Combined_Final.step` to inspect the full robot. Use the individual leg files when checking mirrored geometry, servo orientation, linkage clearance, and horn travel.

More design context is in [../docs/cad-design.md](../docs/cad-design.md).
