# Domino Policy To CAD Linkage Contract

This report records the first automated contract check between the clean Isaac Lab policy articulation and the CAD-derived shared fixed-body linkage prototype.

Status: **passed for policy action contract versus CAD fixed-body linkage report**.

## What Was Checked

The checker compares three sources:

- The shared Domino policy action contract in `prototypes/quadruped/domino_action_contract.py`.
- The clean training URDF in `prototypes/quadruped/domino_quadruped_clean.urdf`.
- A fresh `domino-four-12-fixed-body` independent CAD-linkage JSON report from the pin-linkage prototype.

The check verifies:

- The CAD report exposes the same twelve action names in the same order as the policy env.
- The twelve actions are grouped as four shoulder actuators plus two linkage-drive actuators per leg.
- The policy contract target limits match the CAD report limits.
- The clean URDF joint limits match the CAD report limits.
- The default policy joint positions sit inside the CAD-derived limits.
- The CAD fixed-body linkage calibration is full rank across all twelve actuator channels plus intercept.

## Result

| Metric | Result |
| --- | ---: |
| CAD geometry | `domino-four-12-fixed-body` |
| CAD linkage steps | `1200` |
| Action count | `12` |
| Shoulder actions | `4` |
| Linkage-drive actions | `8` |
| Linkage-drive actions per leg | `2` |
| Lower-linkage drive actions | `4` |
| Upper-pitch drive actions | `4` |
| Max policy-to-CAD limit delta | `0.0 deg` |
| Max URDF-to-CAD limit delta | `0.000026 deg` |
| Calibration status | `fit` |
| Calibration matrix rank | `13` |
| Calibration inputs including intercept | `13` |
| Rank deficient | `false` |
| Status | `passed` |

## Commands

Generate the CAD-side fixed-body linkage evidence:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-fixed-body `
  --drive-schedule independent `
  --steps 1200 `
  --independent-segment-steps 100 `
  --independent-settle-steps 20 `
  --fit-start-step 0 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --shoulder-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.5 `
  --secondary-drive-frequency-hz 0.5 `
  --shoulder-drive-frequency-hz 0.5 `
  --report-path <output-folder>/domino_four_12_fixed_body_independent_report.json `
  --no-print-report
```

Run the contract check:

```powershell
<python> simulation/isaac/prototypes/quadruped/check_cad_linkage_contract.py `
  --linkage-report <output-folder>/domino_four_12_fixed_body_independent_report.json `
  --urdf-path simulation/isaac/prototypes/quadruped/domino_quadruped_clean.urdf `
  --report-path <output-folder>/domino_policy_cad_contract_report.json
```

## Remaining Gap

This is a contract bridge, not the final CAD-fidelity solution. It proves the policy model's twelve action names, per-leg actuator layout, roles, limits, defaults, and training URDF limits are aligned with the CAD-derived shared fixed-body linkage gate. The passive CAD linkage constraints are still not merged into the floating policy-training articulation.
