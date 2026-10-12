# DominoQuadruped project layout

This folder is the consolidated Domino robot-dog project. Its source layout is
kept compatible with the existing firmware, URDF, USD, Domino Virtual Lab,
and Isaac Sim scripts.

## Deliberate path choices

- `simulation/isaac` remains in the same relative location used by the scripts.
- `simulation/urdf` and `simulation/usd` remain in their existing relative
  locations because the Isaac tooling and documentation refer to them there.
- The six external USD export snapshots are copied unchanged under
  `assets/legacy_isaac_exports/`. Each snapshot keeps its original
  `Materials/` and `parts/` children so its internal relative asset references
  continue to resolve.
- The old source tree's copied working files and Git metadata were removed
  after hash verification. Its uncopied generated and cache directories
  remain in their original locations.
- The six original external export paths are Windows junctions to the copies
  under `assets/legacy_isaac_exports/`, so existing absolute Isaac references
  continue to resolve without keeping duplicate files.
- Isaac Sim and Isaac Lab installations were not moved. The existing helper
  scripts continue to use `C:\isaac-sim` and `C:\isaac-projects\IsaacLab` by
  default, or the `ISAAC_SIM_ROOT` / `ISAACLAB_ROOT` environment overrides.

## Preserved outside the canonical project

Generated runtime/build data was excluded from the consolidation:

- `simulation/isaac/out/`
- `simulation/standalone/runtime/`
- `simulation/standalone/node_modules/`
- `simulation/standalone/dist/`
- PlatformIO `.pio/`

These are local caches or run outputs, are ignored by the project, and remain
in the old source location because they were not part of the verified copy.
They can be regenerated in the canonical project when needed.
