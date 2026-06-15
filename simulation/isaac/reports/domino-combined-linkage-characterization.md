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
  --steps 1000 `
  --fit-start-step 60 `
  --drive-amplitude-deg 2 `
  --secondary-drive-amplitude-deg 2 `
  --drive-frequency-hz 0.2 `
  --secondary-drive-frequency-hz 0.2 `
  --report-path <output-folder>/domino_combined_leg_characterization.json `
  --save-usd <output-folder>/domino_combined_leg_characterization.usd
```

## Closure Stability

| Metric | Result |
| --- | ---: |
| Physics steps | `1000` |
| Physics dt | `0.005 s` |
| Max lower closure error | `0.00001490 m` |
| Max upper closure error | `0.00000294 m` |
| Max body linear speed | `0.037481 m/s` |

The combined leg did not produce non-finite poses or velocities. Both loop closures stayed bounded under simultaneous two-drive motion.

## Motion Characterization

| Quantity | Min | Max | Final |
| --- | ---: | ---: | ---: |
| Lower drive target, deg | `-17.000000` | `-13.000000` | `-15.012566` |
| Upper drive target, deg | `-2.000000` | `2.000000` | `1.999961` |
| Lower driver body pitch, deg | `-1.820513` | `-0.491281` | `-1.381006` |
| Upper driver body pitch, deg | `-1.628365` | `-0.182148` | `-1.106722` |
| Coupler body pitch, deg | `-1.158683` | `-0.208075` | `-0.826778` |
| Lower diagonal relative to coupler, deg | `-0.635458` | `-0.278530` | `-0.531013` |
| Upper driver relative to coupler, deg | `-0.472943` | `0.025928` | `-0.279944` |

Drive tracking error against simple world/body-pitch measurements:

| Drive | Min error | Max error | Final error |
| --- | ---: | ---: | ---: |
| `lower_drive_revolute_59` | `11.662777 deg` | `15.243038 deg` | `13.631561 deg` |
| `upper_drive_revolute_58` | `-3.115988 deg` | `0.447099 deg` | `-3.106682 deg` |

These tracking-error numbers use simple body-pitch measurements as a proxy for the effective linkage angle. They are useful for first-pass characterization, but they are not the final PhysX joint-state mapping that should feed a policy.

## Local Linear Calibration Fit

The latest characterization run fits a local linear model after the first 60 startup steps:

```text
output_deg = intercept + lower_coeff * lower_drive_target_deg + upper_coeff * upper_drive_target_deg
```

Fit inputs:

- Sample count: `940`
- Matrix rank: `3`
- Lower input: `lower_drive_revolute_59`
- Upper input: `upper_drive_revolute_58`

| Output proxy | Intercept | Lower coeff | Upper coeff | RMSE | R^2 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Lower driver to ground, deg | `0.081800` | `0.108579` | `0.083813` | `0.000784` | `0.999983` |
| Upper driver to ground, deg | `0.080162` | `0.094291` | `0.114446` | `0.000931` | `0.999979` |
| Coupler to ground, deg | `0.056903` | `0.068106` | `0.069478` | `0.000582` | `0.999982` |
| Lower diagonal to coupler, deg | `0.022524` | `0.038466` | `0.012053` | `0.000217` | `0.999986` |
| Upper driver to coupler, deg | `0.023260` | `0.026185` | `0.044969` | `0.000362` | `0.999974` |

This is a small-range calibration around the conservative `-15 deg` lower-drive centre and `0 deg` upper-drive centre. It proves the simulated pin-joint linkage has a stable, repeatable relationship between commanded inputs and measured output proxies over this range. It does not yet prove that the same coefficients are valid across the full servo range or under ground contact.

## Interpretation

The direct closure representation fixed the previous upper-loop drift. The remaining issue is not loop explosion; it is turning this local fit into a useful training-coordinate mapping.

The lower drive target range is much larger than the measured lower driver body-pitch range. That means the simplified PhysX drive target is not a direct substitute for the `lower_linkage` coordinate used by `prototypes/one_leg`. Before this becomes a training asset, the fitted mapping needs to be compared against the clean one-leg abstraction and then converted into actuator targets, joint limits, and reset-safe default poses.

## Next Work

1. Run one-input sweeps for the lower and upper drives separately so the cross-coupling terms can be checked independently.
2. Compare the fitted output proxies to `upper_pitch` and `lower_linkage` in `prototypes/one_leg`.
3. Replace the proxy body-pitch measurement with a cleaner output coordinate before feeding it to an RL environment.
4. Only after the mapping is stable, merge the constrained linkage into the one-leg USD/asset path.
