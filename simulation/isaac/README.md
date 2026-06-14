# Domino Isaac Sim / Isaac Lab Bring-Up

This folder is the working plan for turning Domino from CAD and firmware into a stable Isaac Sim / Isaac Lab robot. The aim is a trainable quadruped model, not just a pretty visual import.

> Current status: the Isaac runtime is not included in this repository, and the raw CAD-generated URDF is not ready to use as a training articulation. Treat the files here as a structured bring-up path.

## Included Files

| Path | Purpose |
| --- | --- |
| `../usd/` | Existing USD/USDZ visual exports from the CAD-to-Isaac exploration. |
| `../urdf/generated/Domino_URDF_Parts_Combined_Final_description/` | Generated URDF package copied from the CAD export for reference and analysis. |
| `analyze-domino-urdf.ps1` | Repeatable URDF topology validator. |
| `reports/domino-urdf-topology.md` | Current topology report generated from the URDF export. |

## Current Finding

The generated URDF is useful as a record of the CAD export, but it should not be imported blindly into Isaac as the final training model.

The current topology report shows:

- 37 link elements.
- 29 unique link names.
- 36 joint elements.
- 8 duplicate link-name groups.
- 8 child links with multiple incoming joints.
- 11 limited revolute joints and 25 continuous joints.

That means the file contains loop-style structure and repeated link names. Isaac/PhysX articulations need a tree structure for the articulation itself; loops can be closed separately using rigid-body joints between articulation links, but the core articulation still has to be authored cleanly. See the official PhysX articulation notes on tree structure and closing loops: [PhysX Articulations](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Articulations.html#articulation-tree-structure).

## Recommended Strategy

Do not try to make the exported CAD mates become the RL policy model directly. Build the simulator in layers:

1. **Visual model**
   - Use the USD exports for visual inspection, scale checks, orientation checks, and render/reference work.
   - Keep mesh collisions disabled or heavily simplified during early physics tests.

2. **Training articulation**
   - Create a clean 12-DoF tree articulation: hip ab/ad, upper-leg pitch, and lower/knee actuator coordinate for each leg.
   - Keep the action space tied to the twelve real servo commands rather than every passive CAD pin.
   - Give every training joint a readable name before training; do not keep names like `Revolute 46` in the final model.

3. **Passive linkage fidelity**
   - First pass: treat the lower four-bar linkage as visual geometry attached to the controlled lower/knee body.
   - Second pass: add passive revolute pins and loop-closing rigid-body joints after a one-leg model is stable.
   - If the loop constraints fight the driven joints, keep the training model simplified and use the detailed linkage model only for validation.

4. **Isaac Lab asset config**
   - Spawn the final robot from USD using `ArticulationCfg` and `UsdFileCfg`.
   - Configure actuator groups for the twelve driven joints only.
   - Disable self-collision until basic falling, standing, and joint-limit tests are stable.
   - Use conservative solver settings and low drive gains during first motion tests.

5. **Policy path**
   - Validate one leg.
   - Validate all four legs standing on a raised fixture.
   - Train a stand/height-control task.
   - Add velocity commands and gait rewards only after reset behavior, joint limits, and contacts are reliable.

## Commands

Run the URDF topology report:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/analyze-domino-urdf.ps1
```

Run it against another exported URDF:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/analyze-domino-urdf.ps1 -UrdfPath path\to\robot.urdf
```

## Isaac Runtime Notes

NVIDIA's current Isaac Lab import workflow recommends converting robot assets into USD and then writing an asset configuration for spawning and training. The Isaac Lab docs also call out useful URDF import settings such as fixed base selection, fixed-joint merging, joint drive configuration, and setting joint target type to `none` during import when you want to configure drives later. See:

- [Isaac Lab: Importing a New Asset](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)
- [Isaac Lab: Writing an Asset Configuration](https://isaac-sim.github.io/IsaacLab/main/source/how-to/write_articulation_cfg.html)
- [Isaac Sim: Import URDF](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/import_urdf.html)

## Immediate Next Milestone

The next useful asset to create is not the full robot. It is a clean one-leg USD with:

- Three driven joints with readable names.
- Simplified collision bodies.
- Correct joint axes and limits.
- Passive linkage geometry either visual-only or constrained after the driven model works.
- A small Isaac Lab script that can reset the leg and sweep each joint through safe limits.

Once the one-leg model behaves, duplicate it into a four-leg robot and wire the Isaac Lab action space to the same twelve-servo abstraction used by the firmware.
