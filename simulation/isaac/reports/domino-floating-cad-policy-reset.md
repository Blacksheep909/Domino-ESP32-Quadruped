# Domino Floating CAD Policy-Style Reset Smoke

This report records the first policy-style action and reset smoke for the floating CAD-derived twelve-actuator linkage scene.

Status: **passed for 12-channel policy-style target updates plus runtime rigid-body resets**.

## What Was Checked

The previous floating CAD-linkage smoke proved that the actual CAD-derived pin-linkage assembly can sit under gravity with simple foot contact proxies. This gate moves one step closer to policy training by checking two policy-environment requirements:

- The scene accepts a bounded twelve-channel action vector using the same action order as the clean policy model.
- The scene can reset every CAD-linkage rigid body back to its initial pose during runtime.

This is not RSL-RL training yet. It is a direct PhysX smoke test for the reset/action mechanics that a future Isaac Lab environment needs.

## Runtime Command

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-floating-body `
  --drive-schedule policy-step `
  --steps 360 `
  --policy-action-scale-deg 0.25 `
  --policy-hold-steps 20 `
  --reset-interval-steps 120 `
  --max-post-reset-position-error-m 0.001 `
  --max-loop-closure-error-m 0.005 `
  --min-floating-root-height-m 0.02 `
  --report-path <output-folder>/domino_four_12_floating_policy_step_reset_report.json `
  --no-print-report
```

## Result

| Metric | Result |
| --- | ---: |
| Geometry mode | `domino-four-12-floating-body` |
| Drive schedule | `policy-step` |
| Physics steps | `360` |
| Dynamic gravity | `true` |
| Driven actuator channels | `12` |
| Policy action scale | `0.25 deg` |
| Policy hold interval | `20 steps` |
| Reset interval | `120 steps` |
| Runtime reset count | `2` |
| Reset steps | `120`, `240` |
| Max post-reset position error | `0.0 m` |
| Max post-reset orientation error | `0.0 quaternion norm` |
| Max loop-closure error | `0.00007423 m` |
| Max body linear speed | `0.099219 m/s` |
| Body reference min height | `0.128755 m` |
| Body reference final height | `0.129004 m` |
| Failure reasons | none |
| Status | `passed` |

## Remaining Gap

This proves the floating CAD linkage can accept policy-shaped commands and reset its rigid bodies in a single-scene smoke test. The remaining work is to wrap this mechanism as a real Isaac Lab training environment with cloned environments, observation tensors, rewards, terminations, logging, and RSL-RL checkpoint generation.
