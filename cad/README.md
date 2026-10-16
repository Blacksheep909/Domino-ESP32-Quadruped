# CAD Exports

This folder contains neutral STEP exports for the Domino prototype. They are included for inspection, measurement, adaptation, and CAD-to-simulation work.

> Work in progress: these are reference exports, not a complete manufacturing package. Validate clearances, servo fit, print tolerances, fastener sizes, and safe joint travel before building from them.

## Download The Latest Robot (v6)

These are the newest exports, taken from the `body_v6` master assembly. They include the generative knee bracket, the round-hub L2 clamps, and the MR63ZZ pivot bearing. To download one, open the file on GitHub and use the **Download raw file** button.

| File | Contents |
| --- | --- |
| [`step/latest/Domino_v6_full_robot.step`](step/latest/Domino_v6_full_robot.step) | **Complete robot**: body frame plus all four legs in their mounted positions (127 solids). |
| [`step/latest/Domino_v6_leg_right_FR_BR.step`](step/latest/Domino_v6_leg_right_FR_BR.step) | One right-hand leg module (front-right and back-right use the same parts). |
| [`step/latest/Domino_v6_leg_left_FL_BL.step`](step/latest/Domino_v6_leg_left_FL_BL.step) | One left-hand leg module (mirror of the right leg; front-left and back-left). |
| [`step/latest/Domino_v6_body_frame.step`](step/latest/Domino_v6_body_frame.step) | Central body frame, end plates, twist plates and battery holders. |

Units are millimetres. The pivot's bearing pocket is 6.15 mm for a 6.00 mm MR63ZZ, which gives a slide-in fit on a well-tuned FDM printer.

## Analysis

| Folder | Contents |
| --- | --- |
| [`fea_generative_bracket/`](fea_generative_bracket/) | Linear static stress (von Mises) study of the generative knee bracket under the Quadruped Studio pin-reaction load cases, with scripts, inputs and rendered results. |

## Older STEP Files

These earlier exports come from the CAD-to-URDF trail and match the current simulation and URDF assets.

| Path | Purpose |
| --- | --- |
| `step/assembly/Domino_URDF_Parts.step` | Full assembly-style STEP export for viewing the complete robot. |
| `step/combined/Domino_URDF_Parts_Combined.step` | Combined model export from the CAD-to-URDF trail. |
| `step/combined/Domino_URDF_Parts_Combined_Final.step` | Combined reference export used by the URDF and simulation work. |
| `step/parts/Body.step` | Central body and electronics cage reference. |
| `step/parts/FL.step` | Front-left leg module. |
| `step/parts/FR.step` | Front-right leg module. |
| `step/parts/BL.step` | Back-left leg module. |
| `step/parts/BR.step` | Back-right leg module. |

More design context is in [../docs/cad-design.md](../docs/cad-design.md).
