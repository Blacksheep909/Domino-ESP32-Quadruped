# Domino Floating CAD Linkage Contact Smoke

This report records the first gravity/contact smoke for the CAD-derived twelve-actuator pin-linkage scene.

Status: **passed for a floating shared-body CAD-linkage smoke with gravity and simple foot contact proxies**.

## What Changed

The earlier CAD-linkage gates were intentionally fixed-base and gravity-free. They proved the passive pin joints and loop-closing revolute constraints, but they did not prove that the actual linkage assembly could be placed under gravity.

The new `domino-four-12-floating-body` mode keeps the CAD-derived linkage structure and twelve actuator drives, then adds:

- A dynamic shared `body_reference` instead of a kinematic body anchor.
- Gravity on the dynamic linkage bodies.
- A simple static ground box.
- Four spherical foot contact proxies attached at the CAD lower-closure points.
- Optional failure gates for maximum loop-closure error and final floating-body height.

## Runtime Command

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-floating-body `
  --steps 300 `
  --drive-amplitude-deg 0 `
  --secondary-drive-amplitude-deg 0 `
  --shoulder-drive-amplitude-deg 0 `
  --drive-frequency-hz 0.5 `
  --secondary-drive-frequency-hz 0.5 `
  --shoulder-drive-frequency-hz 0.5 `
  --max-loop-closure-error-m 0.005 `
  --min-floating-root-height-m 0.02 `
  --report-path <output-folder>/domino_four_12_floating_body_report.json `
  --save-usd <output-folder>/domino_four_12_floating_body.usd `
  --no-print-report
```

## Result

| Metric | Result |
| --- | ---: |
| Geometry mode | `domino-four-12-floating-body` |
| Physics steps | `300` |
| Dynamic gravity | `true` |
| Driven actuator channels | `12` |
| Ground box size | `10.0 m` |
| Floating CAD Z offset | `0.12 m` |
| Foot contact proxies | `4` |
| Max loop-closure error | `0.00007413 m` |
| Max body linear speed | `0.097383 m/s` |
| Body reference min height | `0.128749 m` |
| Body reference final height | `0.129020 m` |
| Failure reasons | none |
| Status | `passed` |

The twelve action channels remain the same contract used by the clean policy model: four shoulder ab/ad drives, four lower-linkage drives, and four upper-pitch drives.

The next reset/action gate is tracked in [`domino-floating-cad-policy-reset.md`](domino-floating-cad-policy-reset.md).

## Remaining Gap

This is still not the final policy-training robot. The floating CAD linkage is authored as a direct PhysX rigid-body/joint scene, not yet as a cloned Isaac Lab `DirectRLEnv` training asset. The next gate is to wrap or convert this floating CAD-linkage scene so policy code can clone it across environments, read body/contact observations, compute rewards and terminations, and command the same twelve action targets through an Isaac Lab training loop.
