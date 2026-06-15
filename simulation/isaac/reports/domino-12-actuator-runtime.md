# Domino Twelve-Actuator Runtime Test

This report records the first fixed-base Isaac/PhysX test where the Domino CAD-derived linkage scene exposes the same twelve actuator channels as the physical robot:

- Four shoulder hip ab/ad drives.
- Four lower two-bar/four-bar linkage drives.
- Four upper pitch drives.

This is still a bring-up scene, not the final policy-training robot. The bodies are simplified, gravity and contacts are disabled, and the base anchors are kinematic so the linkage can be validated before ground interaction is introduced.

Two fixed-base variants are tracked here:

- `domino-four-12-actuators`: each leg has its own kinematic hip reference. This isolates the twelve actuator layout.
- `domino-four-12-fixed-body`: all four shoulder joints attach to one shared kinematic body reference. This is closer to the final robot body structure and is the next gate toward a resettable/floating Isaac Lab robot.

## Runtime Command Shape

Phased-sine stability smoke test:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-actuators `
  --steps 600 `
  --fit-start-step 60 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --shoulder-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.15 `
  --secondary-drive-frequency-hz 0.15 `
  --shoulder-drive-frequency-hz 0.15 `
  --report-path <output-folder>/domino_four_12_actuators_report.json `
  --no-print-report
```

Independent twelve-drive calibration sweep:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-actuators `
  --drive-schedule independent `
  --steps 2400 `
  --independent-segment-steps 200 `
  --independent-settle-steps 40 `
  --fit-start-step 0 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --shoulder-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.25 `
  --secondary-drive-frequency-hz 0.25 `
  --shoulder-drive-frequency-hz 0.25 `
  --report-path <output-folder>/domino_four_12_actuators_independent_report.json `
  --no-print-report
```

Shared fixed-body smoke test:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --geometry domino-four-12-fixed-body `
  --steps 600 `
  --fit-start-step 60 `
  --drive-amplitude-deg 1 `
  --secondary-drive-amplitude-deg 1 `
  --shoulder-drive-amplitude-deg 1 `
  --drive-frequency-hz 0.15 `
  --secondary-drive-frequency-hz 0.15 `
  --shoulder-drive-frequency-hz 0.15 `
  --report-path <output-folder>/domino_four_12_fixed_body_report.json `
  --no-print-report
```

Shared fixed-body independent sweep:

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

## Actuator Map

| Leg module | Shoulder hip ab/ad | Lower linkage drive | Upper pitch drive |
| --- | --- | --- | --- |
| `dom_p_4_1` | `Revolute 1` | `Revolute 59` | `Revolute 58` |
| `dom_p_12_1` | `Revolute 2` | `Revolute 46` | `Revolute 55` |
| `dom_p_25_1` | `Revolute 3` | `Revolute 47` | `Revolute 56` |
| `dom_p_21_1` | `Revolute 4` | `Revolute 48` | `Revolute 57` |

The generated CAD URDF marks `Revolute 47` as continuous, but its pivot mirrors the other lower-linkage drive pivots. This test continues to treat it as a driven lower-linkage actuator until the CAD/export metadata is corrected or the physical design says otherwise.

## Action Contract

The current shared-body test exposes twelve actuator targets. The intended Isaac Lab action vector is:

| Index | Action name | CAD-derived joint | Role | Target limit |
| ---: | --- | --- | --- | --- |
| 0 | `dom_p_4_1_shoulder_ab_ad` | `dom_p_4_1_revolute_1` | Shoulder hip ab/ad | `-30 deg` to `30 deg` |
| 1 | `dom_p_4_1_lower_linkage` | `dom_p_4_1_revolute_59` | Lower two-bar/four-bar drive | `-120 deg` to `0 deg` |
| 2 | `dom_p_4_1_upper_pitch` | `dom_p_4_1_revolute_58` | Upper pitch drive | `-30 deg` to `60 deg` |
| 3 | `dom_p_12_1_shoulder_ab_ad` | `dom_p_12_1_revolute_2` | Shoulder hip ab/ad | `-30 deg` to `30 deg` |
| 4 | `dom_p_12_1_lower_linkage` | `dom_p_12_1_revolute_46` | Lower two-bar/four-bar drive | `-120 deg` to `0 deg` |
| 5 | `dom_p_12_1_upper_pitch` | `dom_p_12_1_revolute_55` | Upper pitch drive | `-30 deg` to `60 deg` |
| 6 | `dom_p_25_1_shoulder_ab_ad` | `dom_p_25_1_revolute_3` | Shoulder hip ab/ad | `-30 deg` to `30 deg` |
| 7 | `dom_p_25_1_lower_linkage` | `dom_p_25_1_revolute_47` | Lower two-bar/four-bar drive | `-120 deg` to `0 deg` |
| 8 | `dom_p_25_1_upper_pitch` | `dom_p_25_1_revolute_56` | Upper pitch drive | `-30 deg` to `60 deg` |
| 9 | `dom_p_21_1_shoulder_ab_ad` | `dom_p_21_1_revolute_4` | Shoulder hip ab/ad | `-30 deg` to `30 deg` |
| 10 | `dom_p_21_1_lower_linkage` | `dom_p_21_1_revolute_48` | Lower two-bar/four-bar drive | `-30 deg` to `90 deg` |
| 11 | `dom_p_21_1_upper_pitch` | `dom_p_21_1_revolute_57` | Upper pitch drive | `-30 deg` to `60 deg` |

The runtime report writes this same list to `action_space`, and each drive entry includes its `action_index`, `action_name`, role, axis, center, and modeled target limits. By default, the prototype now fails if a generated target exceeds those modeled limits. `--disable-drive-limit-checks` is available for deliberate stress tests, but it should stay off for training bring-up.

## Stability Smoke Test

| Metric | Result |
| --- | ---: |
| Physics steps | `600` |
| Driven actuator channels | `12` |
| Shoulder drives | `4` |
| Lower linkage drives | `4` |
| Upper pitch drives | `4` |
| Max loop-closure error | `0.00001473 m` |
| Max body linear speed | `0.032290 m/s` |
| Status | `passed` |

The phased-sine run exercises all twelve drive targets together. Its linear calibration output is diagnostic only because the shared sine schedule is rank deficient.

## Independent Calibration Sweep

Per-leg anchor variant:

| Metric | Result |
| --- | ---: |
| Physics steps | `2400` |
| Segment length | `200 steps` |
| Segment settle exclusion | `40 steps` |
| Calibration samples | `1920` |
| Matrix rank | `13` |
| Input columns including intercept | `13` |
| Rank deficient | `false` |
| Max loop-closure error | `0.00001427 m` |
| Max body linear speed | `0.029792 m/s` |
| Status | `passed` |

The full-rank result is the useful milestone: the fixed-base scene now exposes the complete twelve-servo command layout and each actuator can be swept independently without the closed pitch linkages becoming unstable.

## Shared Fixed Body Sweep

| Metric | Result |
| --- | ---: |
| Geometry mode | `domino-four-12-fixed-body` |
| Physics steps | `1200` |
| Segment length | `100 steps` |
| Segment settle exclusion | `20 steps` |
| Calibration samples | `960` |
| Matrix rank | `13` |
| Input columns including intercept | `13` |
| Rank deficient | `false` |
| Max loop-closure error | `0.00001425 m` |
| Max body linear speed | `0.029793 m/s` |
| Modeled target limit violations | `0 deg` |
| Status | `passed` |

This is the stronger current gate. The four shoulder joints now attach to one shared kinematic body reference rather than four separate kinematic anchors, while the twelve actuator inputs and eight closed pitch-linkage loops remain stable over the tested range.

## Remaining Gap

This does not yet prove policy training. The next gate is to turn the shared fixed-body 12-actuator linkage scene into a clean Isaac Lab robot with:

1. A single floating or resettable base.
2. Hip ab/ad articulation integrated with the body as an Isaac Lab articulation or equivalent resettable asset.
3. Gravity, contacts, simple collision bodies, and hard-stop checks.
4. A twelve-action Isaac Lab environment wired to the same actuator order shown above.
