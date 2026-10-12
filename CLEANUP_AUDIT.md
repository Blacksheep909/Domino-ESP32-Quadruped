# Duplicate cleanup receipt

Date: 2026-08-10

The user-authorized duplicate cleanup is complete. Only data with verified
copies in the canonical project was removed. Generated, cached, installed, or
otherwise uncopied data was preserved.

## Canonical project

The consolidated project is:

`C:\Users\charl\Documents\DominoQuadruped`

It contains the project source, CAD, URDF, USD, firmware, hardware, Isaac
scripts, Git history, and the copied legacy USD export snapshots.

## Removed duplicate data

- 244 old source working-tree files were removed from
  `C:\Users\charl\Documents\Codex\2026-06-12\im-not-sure-where-but-there\outputs\V10_CRSF_TY90_portfolio`.
  Each file was SHA-256 verified against the canonical copy first. The
  verified total was 56,599,577 bytes.
- The old source `.git` directory was removed after the canonical repository
  passed `git fsck --full`.
- Six old USD export directories were removed after verifying all 150 files
  and 34,443,946 bytes against the canonical copies. Their old paths were
  immediately recreated as junctions, so they do not hold a second data copy.

## Isaac-compatible legacy paths

The old paths below now point to the canonical directories:

| Existing path | Canonical target |
| --- | --- |
| `C:\Users\charl\Documents\Domino_URDF_Parts_Combined_Final.step` | `C:\Users\charl\Documents\DominoQuadruped\assets\legacy_isaac_exports\Domino_URDF_Parts_Combined_Final.step` |
| `C:\Users\charl\Documents\Domino_URDF_Parts_Combined_Final.step_01` | `C:\Users\charl\Documents\DominoQuadruped\assets\legacy_isaac_exports\Domino_URDF_Parts_Combined_Final.step_01` |
| `C:\Users\charl\Documents\Domino_URDF_Parts_Combined_Final.step_02` | `C:\Users\charl\Documents\DominoQuadruped\assets\legacy_isaac_exports\Domino_URDF_Parts_Combined_Final.step_02` |
| `C:\Users\charl\Documents\Domino_URDF_Parts_Combined_Final.step_03` | `C:\Users\charl\Documents\DominoQuadruped\assets\legacy_isaac_exports\Domino_URDF_Parts_Combined_Final.step_03` |
| `C:\Users\charl\Documents\Domino_URDF_Parts_Combined_Final.step_04` | `C:\Users\charl\Documents\DominoQuadruped\assets\legacy_isaac_exports\Domino_URDF_Parts_Combined_Final.step_04` |
| `C:\Users\charl\Documents\Domino_URDF_Parts_Combined_Final.step_05` | `C:\Users\charl\Documents\DominoQuadruped\assets\legacy_isaac_exports\Domino_URDF_Parts_Combined_Final.step_05` |

All six old paths were verified as Windows junctions, and a sample file was
read successfully through an old path.

## Preserved data

These were not copied and remain in their original locations:

- `simulation\isaac\out\` - Isaac run outputs;
- `simulation\standalone\runtime\` - generated runtime data;
- `simulation\standalone\node_modules\` - installed JavaScript dependencies;
- `simulation\standalone\dist\` - build output; and
- `.pio\` - PlatformIO build data.

Other ignored generated artifacts, including nested PlatformIO output,
Python `__pycache__` files, SIL binaries, and SIL viewer runtime state, also
remain outside the canonical project. The remaining files in the old source
tree are all ignored by the canonical project's ignore rules and were not
part of the verified copy.

The Isaac Sim installation at `C:\isaac-sim` and Isaac Lab installation at
`C:\isaac-projects\IsaacLab` were not moved or changed. Separate/reference
projects, including `ESP32_CRSF_Reader` and `SpotMicroESP32-Nitro-Fork`, were
also left untouched.

## Verification

- Canonical Git repository: `git fsck --full` passed.
- Canonical repository still contains all 241 tracked source files.
- All 244 removed working-tree files were hash-verified before deletion.
- All 150 legacy USD files were hash-verified before deletion.
- All six legacy paths are junctions to the canonical asset copies.
- Canonical USD and URDF paths exist.
- The protected Isaac, runtime, dependency, build-cache, and PlatformIO paths
  remain present.
- No stale old absolute paths were found in project source or scripts; old
  paths appear here only as compatibility documentation.
