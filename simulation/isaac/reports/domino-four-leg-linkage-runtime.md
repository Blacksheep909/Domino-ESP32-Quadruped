# Domino Four-Leg CAD Linkage Runtime Test

This report records the first Isaac/PhysX smoke test where all four Domino CAD-derived pitch linkages run in one scene.

## Result

Status: **passed for a fixed-base, no-contact, all-leg linkage smoke test**.

This is not the finished training robot. The test validates the CAD-derived pitch linkage constraints for all four legs at once:

- Four hip-link locations from the generated URDF.
- Eight driven pitch inputs: one lower linkage drive and one upper pitch drive per leg.
- Eight loop-closing pin constraints: one lower loop and one upper loop per leg.
- Passive revolute pins between the driven links, couplers, and diagonal links.

The hip ab/ad joints are not actuated in this test. The body and hip references are fixed/kinematic, gravity is disabled, mesh collisions are not enabled, and the motion uses conservative 1 degree sweeps.

## Runtime Command Shape

Stability smoke test:

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

Independent calibration sweep:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-combined-legs `
  --drive-schedule independent `
  --steps 3200 `
  --independent-segment-steps 400 `
  --independent-settle-steps 80 `
  --fit-start-step 0 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.15 `
  --secondary-drive-frequency-hz 0.15 `
  --report-path <output-folder>/domino_four_independent_calibration_report.json `
  --save-usd <output-folder>/domino_four_independent_calibration.usd
```

## CAD Leg Mapping

| Leg module | CAD hip link | Lower drive | Upper drive | Lower closure | Upper closure |
| --- | --- | --- | --- | --- | --- |
| `dom_p_4_1` | `DOM_P__4__1` | `Revolute 59` | `Revolute 58` | `Revolute 25` / `Revolute 26` | `Revolute 32` / `Revolute 51` |
| `dom_p_12_1` | `DOM_P__12__1` | `Revolute 46` | `Revolute 55` | `Revolute 23` / `Revolute 24` | `Revolute 29` / `Revolute 50` |
| `dom_p_25_1` | `DOM_P__25__1` | `Revolute 47` | `Revolute 56` | `Revolute 21` / `Revolute 22` | `Revolute 34` / `Revolute 54` |
| `dom_p_21_1` | `DOM_P__21__1` | `Revolute 48` | `Revolute 57` | `Revolute 27` / `Revolute 28` | `Revolute 31` / `Revolute 53` |

The generated CAD URDF marks `Revolute 47` as `continuous`, even though its location mirrors the lower drive pivots on the other legs. This smoke test drives it with the same conservative lower-input range so the fourth leg can be validated, but that joint should be corrected or explicitly documented before a final training asset is built.

## Stability Summary

| Metric | Result |
| --- | ---: |
| Physics steps | `600` |
| Physics dt | `0.005 s` |
| Driven pitch inputs | `8` |
| Loop closure checks | `8` |
| Max loop-closure error | `0.00001217 m` |
| Max body linear speed | `0.039359 m/s` |

| Closure | Max error |
| --- | ---: |
| `dom_p_4_1` lower loop | `0.00001124 m` |
| `dom_p_4_1` upper loop | `0.00000223 m` |
| `dom_p_12_1` lower loop | `0.00001210 m` |
| `dom_p_12_1` upper loop | `0.00000233 m` |
| `dom_p_25_1` lower loop | `0.00001217 m` |
| `dom_p_25_1` upper loop | `0.00000233 m` |
| `dom_p_21_1` lower loop | `0.00001213 m` |
| `dom_p_21_1` upper loop | `0.00000210 m` |

## Drive Sweep

| Drive | Center | Amplitude | Frequency | Phase |
| --- | ---: | ---: | ---: | ---: |
| `dom_p_4_1_revolute_59` | `-15 deg` | `1 deg` | `0.15 Hz` | `0 deg` |
| `dom_p_4_1_revolute_58` | `0 deg` | `1 deg` | `0.15 Hz` | `90 deg` |
| `dom_p_12_1_revolute_46` | `-15 deg` | `1 deg` | `0.15 Hz` | `90 deg` |
| `dom_p_12_1_revolute_55` | `0 deg` | `1 deg` | `0.15 Hz` | `180 deg` |
| `dom_p_25_1_revolute_47` | `-15 deg` | `1 deg` | `0.15 Hz` | `180 deg` |
| `dom_p_25_1_revolute_56` | `0 deg` | `1 deg` | `0.15 Hz` | `270 deg` |
| `dom_p_21_1_revolute_48` | `-15 deg` | `1 deg` | `0.15 Hz` | `270 deg` |
| `dom_p_21_1_revolute_57` | `0 deg` | `1 deg` | `0.15 Hz` | `360 deg` |

## Independent Calibration Sweep

The original phased-sine smoke test is rank-deficient because the eight inputs share frequencies and phase offsets. The independent sweep moves one drive at a time while the other seven hold their centre positions, then excludes the first 80 settling steps of each segment from the fit.

| Metric | Result |
| --- | ---: |
| Physics steps | `3200` |
| Segment length | `400 steps` |
| Segment settle exclusion | `80 steps` |
| Calibration samples | `2560` |
| Matrix rank | `9` |
| Input columns including intercept | `9` |
| Rank deficient | `false` |
| Max loop-closure error | `0.00001147 m` |
| Max body linear speed | `0.035345 m/s` |

The fitted model is:

```text
output_deg = intercept + sum(coeff_deg_per_deg * drive_target_deg)
```

These are still body-pitch proxy outputs, not final policy coordinates. The useful result is that the all-leg calibration is now full-rank and each leg has a repeatable local drive-to-output fit over this conservative range.

| Output proxy | Own drive coefficient | RMSE | R^2 |
| --- | ---: | ---: | ---: |
| `dom_p_4_1` lower driver to ground | `0.086007` | `0.001125` | `0.998446` |
| `dom_p_4_1` upper driver to ground | `0.100251` | `0.001244` | `0.998420` |
| `dom_p_12_1` lower driver to ground | `0.086100` | `0.001157` | `0.998356` |
| `dom_p_12_1` upper driver to ground | `0.100579` | `0.001269` | `0.998364` |
| `dom_p_25_1` lower driver to ground | `0.085985` | `0.001153` | `0.998364` |
| `dom_p_25_1` upper driver to ground | `0.100439` | `0.001276` | `0.998339` |
| `dom_p_21_1` lower driver to ground | `0.085998` | `0.001157` | `0.998351` |
| `dom_p_21_1` upper driver to ground | `0.100296` | `0.001277` | `0.998331` |

## Next Work

1. Decide how to handle `Revolute 47`: fix the CAD/URDF metadata if it is meant to be driven, or model it as passive if the physical design says otherwise.
2. Replace body-pitch proxy outputs with cleaner policy coordinates for each linkage.
3. Merge the stable all-leg pitch linkage pattern with a clean base and hip ab/ad articulation.
4. Reintroduce gravity, simple collisions, mass/inertia tuning, hard stops, and reset-safe joint defaults.
5. Build the Isaac Lab environment around the twelve-servo action space before policy training.
