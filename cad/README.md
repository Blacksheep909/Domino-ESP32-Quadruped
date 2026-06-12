# CAD Exports

This folder contains neutral STEP exports for Domino's current mechanical design. These files are included so the robot can be inspected, measured, remixed, or converted into other CAD and simulation workflows without needing the original Fusion project.

> Work in progress: these exports represent the prototype CAD state used for the current Domino documentation. Check clearances, servo alignment, print orientation, and fastener fit before treating any part as production-ready.

## STEP Files

| Path | Purpose |
| --- | --- |
| `step/assembly/Domino_URDF_Parts.step` | Full assembly-style STEP export. Useful for viewing the complete robot and checking the overall layout. |
| `step/combined/Domino_URDF_Parts_Combined.step` | Combined model export from the URDF/simulation export trail. |
| `step/combined/Domino_URDF_Parts_Combined_Final.step` | Final combined STEP export used as the main CAD reference in this repo. |
| `step/parts/Body.step` | Central body/electronics cage reference. |
| `step/parts/FL.step` | Front-left leg module. |
| `step/parts/FR.step` | Front-right leg module. |
| `step/parts/BL.step` | Back-left leg module. |
| `step/parts/BR.step` | Back-right leg module. |

## Suggested Use

- Open `step/combined/Domino_URDF_Parts_Combined_Final.step` first if you want to understand the whole robot.
- Use the individual leg files when checking mirrored geometry, servo orientation, linkage clearance, and horn travel.
- Use `Body.step` when adapting the electronics cage, PCB mount, receiver position, or wire routing.
- Re-check firmware geometry and servo limits if you materially change link lengths, servo horn positions, or body mounting points.

More design context is in [../docs/cad-design.md](../docs/cad-design.md).
