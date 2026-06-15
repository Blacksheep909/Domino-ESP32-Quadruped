# Domino Combined Linkage Characterization

This report records the first motion-characterization pass for the simplified CAD-derived DOM_P__4__1 leg mechanism.

## Result

Status: **passed for finite-state PhysX runtime, not yet calibrated for policy use**.

The combined linkage mode uses:

- Lower driven input: `Revolute 59`
- Upper driven input: `Revolute 58`
- Shared coupler: `DOM_P_1`
- Lower closure: `Revolute 25` / `Revolute 26`
- Upper closure: direct `Revolute 32` / `Revolute 51` pin closure

## Runtime Command Shape

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-combined-leg `
  --steps 600 `
  --drive-amplitude-deg 2 `
  --secondary-drive-amplitude-deg 2 `
  --drive-frequency-hz 0.2 `
  --secondary-drive-frequency-hz 0.2 `
  --report-path <output-folder>/domino_combined_leg_characterization.json
```

## Closure Stability

| Metric | Result |
| --- | ---: |
| Physics steps | `600` |
| Physics dt | `0.005 s` |
| Max lower closure error | `0.00001394 m` |
| Max upper closure error | `0.00000294 m` |
| Max body linear speed | `0.037481 m/s` |

The combined leg did not produce non-finite poses or velocities. Both loop closures stayed bounded under simultaneous two-drive motion.

## Motion Characterization

| Quantity | Min | Max | Final |
| --- | ---: | ---: | ---: |
| Lower drive target, deg | `-16.165381` | `-13.000000` | `-16.165381` |
| Upper drive target, deg | `-2.000000` | `2.000000` | `-1.625388` |
| Lower driver body pitch, deg | `-1.808369` | `-0.491281` | `-1.808369` |
| Upper driver body pitch, deg | `-1.628322` | `-0.182148` | `-1.628322` |
| Coupler body pitch, deg | `-1.155894` | `-0.208075` | `-1.155894` |
| Lower diagonal relative to coupler, deg | `-0.618743` | `-0.278530` | `-0.618743` |
| Upper driver relative to coupler, deg | `-0.472943` | `0.025928` | `-0.472428` |

Drive tracking error against simple world/body-pitch measurements:

| Drive | Min error | Max error | Final error |
| --- | ---: | ---: | ---: |
| `lower_drive_revolute_59` | `11.662777 deg` | `14.508719 deg` | `14.357012 deg` |
| `upper_drive_revolute_58` | `-2.987380 deg` | `0.447099 deg` | `-0.002933 deg` |

## Interpretation

The direct closure representation fixed the previous upper-loop drift. The remaining issue is not loop explosion; it is calibration.

The lower drive target range is much larger than the measured lower driver body-pitch range. That means the simplified PhysX drive target is not yet a reliable substitute for the `lower_linkage` coordinate used by `prototypes/one_leg`. Before this becomes a training asset, the next step is to calibrate rest offsets, drive gains, and the effective output angle mapping from the CAD linkage to the clean one-leg abstraction.

## Next Work

1. Add explicit rest-angle offsets for the CAD drive inputs.
2. Sweep one input at a time and fit the output angle mapping.
3. Compare the fitted mapping to `upper_pitch` and `lower_linkage` in `prototypes/one_leg`.
4. Only after the mapping is stable, merge the constrained linkage into the one-leg USD/asset path.
