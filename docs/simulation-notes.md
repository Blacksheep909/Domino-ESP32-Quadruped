# Simulation Notes

This project includes exploratory work toward bringing the CAD model into Isaac Sim.

## CAD exports

CAD exports include assembly geometry, USD assets, and generated URDF mesh
packages. Export paths are selected during CAD export and should be referenced
relative to the project rather than through a contributor's home directory.

The exported joint hierarchy is a starting point; it does not automatically
reproduce the robot's closed-chain linkage constraints.

## Main Issue

The real leg design behaves like a closed-chain/four-bar linkage. That is mechanically useful, but it is awkward for many robotics simulation workflows.

URDF-style robot descriptions generally want a tree of links and joints. A closed-loop linkage violates that simple tree structure unless the loop is approximated, constrained separately, or rewritten into an equivalent open-chain model.

In practice, this means a direct CAD export can look correct but still fail to behave like the physical robot:

- The visual meshes import.
- The parts exist as separate bodies.
- The joint hierarchy may not match the real linkage.
- Closed-chain constraints are not automatically recovered from the CAD.
- Servo actuation needs to be mapped to the simulated joint model manually.

## Useful Next Steps

Best next simulation path:

1. Start with one leg, not the whole robot.
2. Build a simplified open-chain model that matches the firmware IK.
3. Add visual rods/linkage parts as non-actuated geometry where needed.
4. Validate joint axes and limits against CAD dimensions.
5. Only then scale to all four legs and the body.

That approach is less literal than importing every CAD mate, but it is more likely to produce a controllable Isaac Sim robot.
