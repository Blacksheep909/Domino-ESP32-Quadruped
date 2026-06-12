# CAD And Mechanical Design

Domino is a prototype ESP32 quadruped built around a carbon-and-printed composite structure, modular 3-DoF legs, and a serviceable central electronics cage. The CAD is included because the mechanical design is a major part of the project: the firmware, wiring layout, servo calibration, and simulation experiments all depend on the geometry being understandable.

> Work in progress: these CAD exports are documentation and prototype reference files. They are useful for inspection and adaptation, but they still need builder-side checks for servo fit, fastener fit, clearances, print tolerances, and safe joint travel.

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

The combined final file is the easiest place to start. The individual leg files are more useful once you are checking clearances, servo horn orientation, mirrored geometry, or adapting the robot to different servos.

## Design Intent

Domino's mechanical direction is to keep the robot visually and physically honest: the structure, electronics, wiring, and linkage work are visible rather than hidden inside a decorative shell. That makes it easier to debug and easier to explain as a portfolio project.

The body is built as an electronics spine. Printed side plates, cross braces, and carbon members create a protected middle bay for the PCB, ESP32, CRSF receiver, regulator, battery wiring, and servo harnesses. This is deliberately more serviceable than a sealed body, because twelve servos and a radio receiver create enough wiring that access becomes part of the mechanical design.

![Domino electronics cage](images/domino-electronics-cage.jpg)

The leg modules are designed as repeated corner units. Each leg provides:

- Hip abduction/adduction.
- Upper leg pitch.
- Lower linkage / knee motion.

The lower mechanism behaves closer to a closed-chain or four-bar linkage than a simple serial robot arm. That helps the physical packaging, but it also means the CAD, firmware, and simulation model cannot be treated as three independent things. The firmware needs the effective leg geometry, the CAD needs enough clearance for the linkage to move without binding, and simulation needs either a constrained closed-loop model or a simplified open-chain approximation.

![Domino side linkage view](images/domino-linkage-side.jpg)

## Composite Cage Structure

The central chassis can be thought of as a composite cage: printed structural nodes and plates locate the electronics and pivots, while carbon rods/tubes carry longer span loads and keep the body stiff without making it bulky. The practical advantages are:

- The electronics remain visible and reachable.
- Servo leads can be routed through the middle rather than around the outside.
- The receiver can be mounted inside the protected bay after the CRSF conversion.
- The body stays narrow enough that leg motion, not chassis width, defines the robot's movement envelope.
- Parts can be iterated independently: legs, body plates, electronics mounts, and carbon members do not all need to change at once.

That modularity is the main reason the repo includes separate body and corner-leg STEP files instead of only one monolithic assembly.

## Firmware Relationship

The CAD is not just cosmetic. Changes to leg length, servo horn clocking, pivot location, or body width can affect:

- IK reachability in [`src/ik.cpp`](../src/ik.cpp).
- Servo channel direction and trim values in [`src/leg_controller.cpp`](../src/leg_controller.cpp).
- Per-servo safety limits used to keep the robot away from hard mechanical stops.
- Stand, stow, ride-height, and tilt poses in [`src/main.cpp`](../src/main.cpp).

If you modify the CAD, re-run the bring-up sequence from [calibration-guide.md](calibration-guide.md). Do not assume the existing trim values or safety limits still protect the mechanism after a geometry change.

## Simulation And URDF Notes

The current STEP exports came from the same CAD export trail used for the URDF and Isaac Sim experiments. The useful lesson from that work is that a model can import visually while still not being a physically correct robot model.

URDF-style robots are normally tree-structured. Domino's lower leg linkage is closer to a closed chain, so the exported model needs extra thought before it becomes a controllable simulation:

- Use STEP or mesh exports for visual geometry.
- Build a simplified joint hierarchy for control.
- Match the simplified joint limits to real CAD clearances.
- Treat decorative or duplicate linkage pieces as visual geometry unless the simulator supports the needed constraints.
- Validate one leg before scaling to the full quadruped.

More detail is in [simulation-notes.md](simulation-notes.md).

## Practical CAD Checklist

Before printing or machining from these files:

1. Open the combined final STEP and inspect the full assembly.
2. Check each leg module for servo horn clearance through the expected motion range.
3. Confirm rod, pivot, bearing, screw, and insert sizes against the hardware you actually have.
4. Print or prototype one leg first.
5. Center the servos mechanically before relying on software trim.
6. Keep the robot lifted for first stand/stow tests.
7. Update firmware safety limits if the mechanical stops or useful servo ranges change.

Domino is a good base to learn from, but it is not a finished kit. Treat the CAD as a prototype design package and validate each mechanical assumption before letting all twelve servos move at once.
