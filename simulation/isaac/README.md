# Domino Isaac Sim / Isaac Lab Bring-Up

This folder is the working plan for turning Domino from CAD and firmware into a stable Isaac Sim / Isaac Lab robot. The aim is a trainable quadruped model, not just a pretty visual import.

> Current status: the Isaac runtime is not included in this repository, and the raw CAD-generated URDF is not ready to use as a training articulation. Treat the files here as a structured bring-up path.

## Included Files

| Path | Purpose |
| --- | --- |
| `../usd/` | Existing USD/USDZ visual exports from the CAD-to-Isaac exploration. |
| `../urdf/generated/Domino_URDF_Parts_Combined_Final_description/` | Generated URDF package copied from the CAD export for reference and analysis. |
| `analyze-domino-urdf.ps1` | Repeatable URDF topology validator. |
| `analyze-domino-linkage-pivots.ps1` | Extracts named linkage pivot coordinates from the generated URDF for CAD-derived pin-joint tests. |
| `run-domino-urdf-import.ps1` | Parameterized Isaac Lab URDF import smoke-test helper. |
| `prototypes/one_leg/` | Clean three-joint one-leg prototype for the first stable Isaac articulation. |
| `prototypes/pin_linkage/` | Minimal actuated pin-joint constraint prototype with generic and CAD-derived Domino linkage modes. |
| `reports/domino-urdf-topology.md` | Current topology report generated from the URDF export. |
| `reports/domino-linkage-pivots.md` | CAD-derived pivot report for the first lower-linkage loops. |
| `reports/domino-linkage-pivots.json` | Machine-readable pivot extraction output. |
| `reports/domino-urdf-import-smoke-test.md` | Result of the first Isaac Lab raw import smoke test. |
| `reports/domino-one-leg-import-smoke-test.md` | Result of the first clean one-leg import smoke test. |
| `reports/domino-one-leg-runtime-sweep.md` | Result of the first clean one-leg Isaac Lab articulation sweep. |
| `reports/domino-pin-linkage-runtime.md` | Results of the generic and CAD-derived actuated passive pin-linkage runtime tests. |
| `reports/domino-combined-linkage-characterization.md` | Motion characterization for the combined two-drive CAD-derived one-leg linkage. |
| `reports/domino-four-leg-linkage-runtime.md` | First all-leg CAD-derived pitch-linkage Isaac/PhysX smoke test. |

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

Run a raw Isaac Lab import smoke test:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-domino-urdf-import.ps1 `
  -IsaacLabRoot <path-to-IsaacLab> `
  -IsaacPython <path-to-isaac-python> `
  -OutputUsd <output-folder>/domino_raw_import.usd `
  -AcceptEula
```

The helper deliberately takes Isaac paths as arguments or environment variables. Do not hard-code personal workstation paths into this repository.

Run the clean one-leg prototype through the same importer:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/run-domino-urdf-import.ps1 `
  -UrdfPath simulation/isaac/prototypes/one_leg/domino_one_leg_clean.urdf `
  -IsaacLabRoot <path-to-IsaacLab> `
  -IsaacPython <path-to-isaac-python> `
  -OutputUsd <output-folder>/domino_one_leg_clean.usd `
  -FixBase `
  -AcceptEula
```

Extract CAD linkage pivots from the generated URDF:

```powershell
powershell -ExecutionPolicy Bypass -File simulation/isaac/analyze-domino-linkage-pivots.ps1
```

Run the CAD-derived one-joint lower-linkage test:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-lower-triangle `
  --steps 600 `
  --drive-amplitude-deg 8 `
  --drive-frequency-hz 0.4 `
  --report-path <output-folder>/domino_lower_triangle_report.json `
  --save-usd <output-folder>/domino_lower_triangle.usd
```

Run the CAD-derived one-joint upper-linkage test:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-upper-loop `
  --steps 600 `
  --drive-amplitude-deg 5 `
  --drive-frequency-hz 0.4 `
  --report-path <output-folder>/domino_upper_loop_report.json `
  --save-usd <output-folder>/domino_upper_loop.usd
```

Run the combined two-drive one-leg linkage test:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-combined-leg `
  --steps 1000 `
  --fit-start-step 60 `
  --drive-amplitude-deg 2 `
  --secondary-drive-amplitude-deg 2 `
  --drive-frequency-hz 0.2 `
  --secondary-drive-frequency-hz 0.2 `
  --report-path <output-folder>/domino_combined_leg_report.json `
  --save-usd <output-folder>/domino_combined_leg.usd
```

Run the all-leg CAD-derived pitch-linkage test:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-combined-legs `
  --steps 600 `
  --fit-start-step 60 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.15 `
  --secondary-drive-frequency-hz 0.15 `
  --report-path <output-folder>/domino_four_combined_legs_report.json `
  --save-usd <output-folder>/domino_four_combined_legs.usd
```

## Isaac Runtime Notes

NVIDIA's current Isaac Lab import workflow recommends converting robot assets into USD and then writing an asset configuration for spawning and training. The Isaac Lab docs also call out useful URDF import settings such as fixed base selection, fixed-joint merging, joint drive configuration, and setting joint target type to `none` during import when you want to configure drives later. See:

- [Isaac Lab: Importing a New Asset](https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html)
- [Isaac Lab: Writing an Asset Configuration](https://isaac-sim.github.io/IsaacLab/main/source/how-to/write_articulation_cfg.html)
- [Isaac Sim: Import URDF](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/import_urdf.html)

## Immediate Next Milestone

The next useful asset is a clean four-leg Isaac Lab robot that combines the validated pieces:

- Hip ab/ad articulation from the generated CAD hip links.
- The CAD-derived pitch-linkage constraints proven by `domino-four-combined-legs`.
- Twelve driven policy actions with readable names.
- Simplified collision bodies.
- Correct joint axes, limits, reset defaults, and hard-stop checks.
- A small Isaac Lab script that can reset the robot and sweep each driven joint through safe limits.

That should be validated fixed-base first, then with gravity and simple contacts, before any policy training.

Current runtime status: the simplified one-leg prototype imports, spawns as an Isaac Lab articulation, finds the three expected driven joints, and completes a headless joint sweep.

Current linkage status: a generic one-actuator four-bar linkage, the CAD-derived lower triangle, the CAD-derived upper loop, a combined two-drive one-leg linkage, and an all-leg four-module pitch-linkage scene all run headlessly with passive pin joints and loop-closing revolute constraints. The four-leg scene validates eight CAD-derived pitch drives and eight loop closures together, but it is still fixed-base, no-contact, no-gravity, and not a policy-training robot yet. The next unresolved gate is independent per-drive calibration, merging the pitch linkage pattern with hip ab/ad articulation, and then reintroducing gravity, contacts, hard stops, and the Isaac Lab training environment.
