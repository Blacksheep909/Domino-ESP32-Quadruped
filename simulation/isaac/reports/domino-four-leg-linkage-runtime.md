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

## Calibration Note

The JSON report includes a linear calibration fit, but the all-leg run is a stability smoke test, not a full 8-input calibration. The fit used 540 samples and had matrix rank `3` with `9` input columns including the intercept, because the eight drives are sinusoidal with shared frequencies and phase offsets. Treat that fit as diagnostic output only.

## Next Work

1. Add an independent one-input sweep mode for the four-leg geometry so each drive can be calibrated without rank deficiency.
2. Decide how to handle `Revolute 47`: fix the CAD/URDF metadata if it is meant to be driven, or model it as passive if the physical design says otherwise.
3. Merge the stable all-leg pitch linkage pattern with a clean base and hip ab/ad articulation.
4. Reintroduce gravity, simple collisions, mass/inertia tuning, hard stops, and reset-safe joint defaults.
5. Build the Isaac Lab environment around the twelve-servo action space before policy training.
