# Simulation

Everything that runs Domino without the physical robot.

| Folder | What it is |
| --- | --- |
| [`standalone/`](standalone/) | **Domino Quadruped Studio**, the desktop app: 3D simulation, LIVE robot tools, calibration, gaits and the Policy Lab. See the [Studio guide](../docs/quadruped-studio.md). |
| [`sil/`](sil/) | The production firmware compiled for the desktop (software-in-the-loop). Studio drives this, so the simulated robot runs the same code as the ESP32. |
| [`isaac_lab/`](isaac_lab/) | NVIDIA Isaac Lab training for the learned walking policy, and the export to the ESP32. |
| [`urdf/generated/`](urdf/generated/) | URDF package and STL meshes exported from the CAD. Studio renders these meshes. The URDF itself is a raw export and is not a usable articulation; the training URDF is [`isaac_lab/assets/domino/domino.urdf`](isaac_lab/assets/domino/domino.urdf). |
| [`audit/`](audit/) | Offline scripts that check the firmware kinematics against the CAD geometry. |

For the CAD itself (STEP files), see [`cad/`](../cad/).
