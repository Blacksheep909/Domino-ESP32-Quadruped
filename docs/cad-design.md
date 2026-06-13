# CAD And Mechanical Design

Domino is built around a carbon-and-printed composite structure, modular 3-DoF legs, and a serviceable central electronics cage. The CAD files are included because the mechanical design is a major part of the project: firmware geometry, servo calibration, electronics packaging, and simulation work all depend on the physical layout.

> Work in progress: these CAD exports are prototype reference files. They are suitable for inspection, measurement, and adaptation, but they are not yet a complete manufacturing package. Servo fit, fastener sizes, rod lengths, print tolerances, and safe joint travel still need builder-side validation.

## Included STEP Exports

The STEP files live under [`cad/step`](../cad/step).

| File | Role |
| --- | --- |
| [`cad/step/assembly/Domino_URDF_Parts.step`](../cad/step/assembly/Domino_URDF_Parts.step) | Full assembly export for viewing the complete robot. |
| [`cad/step/combined/Domino_URDF_Parts_Combined.step`](../cad/step/combined/Domino_URDF_Parts_Combined.step) | Combined geometry export from the CAD-to-URDF trail. |
| [`cad/step/combined/Domino_URDF_Parts_Combined_Final.step`](../cad/step/combined/Domino_URDF_Parts_Combined_Final.step) | Main combined reference export for the current documentation state. |
| [`cad/step/parts/Body.step`](../cad/step/parts/Body.step) | Central body and electronics cage. |
| [`cad/step/parts/FL.step`](../cad/step/parts/FL.step) | Front-left leg module. |
| [`cad/step/parts/FR.step`](../cad/step/parts/FR.step) | Front-right leg module. |
| [`cad/step/parts/BL.step`](../cad/step/parts/BL.step) | Back-left leg module. |
| [`cad/step/parts/BR.step`](../cad/step/parts/BR.step) | Back-right leg module. |

Start with the combined final file to inspect the whole robot. Use the individual body and leg files when checking mirrored geometry, servo orientation, linkage clearance, and packaging changes.

## Design Intent

The design leaves the structure, electronics, wiring, and linkages visible and serviceable. That makes the robot easier to debug and makes the engineering decisions readable in photos, CAD views, and code review.

The body acts as an electronics spine. Printed plates and braces locate the electronics bay and leg modules, while carbon members carry longer span loads without making the body bulky.

![Domino electronics cage](images/domino-electronics-cage.jpg)

Each leg module provides:

- Hip abduction/adduction.
- Upper-leg pitch.
- Lower linkage / knee motion.

The lower mechanism is closer to a closed-chain/four-bar linkage than a simple serial arm. That affects the whole project: the CAD must provide clearance, the firmware must use the effective leg geometry, and simulation must either approximate or explicitly constrain the closed loop.

![Domino side linkage view](images/domino-linkage-side.jpg)

## Composite Cage Structure

The central chassis can be treated as a composite cage:

- Printed parts define mounting surfaces, pivots, and electronics placement.
- Carbon rods/tubes provide stiffness across longer spans.
- The PCB, receiver, regulator, and servo harnessing remain accessible.
- Leg motion remains the dominant packaging constraint rather than body width.
- Body, leg, electronics, and carbon-member changes can be iterated separately.

This modularity is why the repo includes separate body and corner-leg STEP files instead of only a single monolithic assembly.

## Firmware Relationship

CAD changes can affect firmware behavior. Changes to link length, servo horn clocking, pivot location, or body width may require updates to:

- IK reachability in [`src/ik.cpp`](../src/ik.cpp).
- Servo direction and trim values in [`src/leg_controller.cpp`](../src/leg_controller.cpp).
- Per-servo safety limits.
- Stand, stow, ride-height, and tilt poses in [`src/main.cpp`](../src/main.cpp).

After a mechanical change, repeat the bring-up and calibration process. Do not assume the existing trim values or safety limits still protect the mechanism.

## Simulation And URDF Notes

The current STEP exports came from the same CAD export trail used for the URDF and Isaac Sim experiments. USD/USDZ exports are included under [`simulation/usd`](../simulation/usd).

The visual CAD import is useful, but it is not automatically a physically correct simulation. URDF-style robots are usually tree-structured, while Domino's lower leg linkage behaves more like a closed chain.

A useful simulation path is:

1. Validate one simplified leg first.
2. Match joint axes and limits to the CAD.
3. Add rods and linkage members as visual geometry where needed.
4. Add closed-chain constraints only if the simulator workflow supports them cleanly.
5. Scale to the full quadruped after one leg is controllable.

More detail is in [simulation-notes.md](simulation-notes.md).
