# Offline movement audit

`run_pose_audit.py` compiles `pose_probe.cpp` against production firmware and SIL
peripherals. It never opens a robot connection or sends physical commands.

Requirements: Python with NumPy, Node with the Studio dependencies, and the
MinGW compiler used by `simulation/sil/build.ps1`. Defaults locate this machine's
bundled runtimes; `--node`, `--compiler` and `--output` override them.

```powershell
python simulation/audit/run_pose_audit.py --session 'C:/path/to/engineering-session.json'
```

The audit covers 2,089 poses under each of three profiles: defaults, offsets
inferred from the September 25 recording, and those offsets with owner-confirmed
FL/BL hip inversion. Only the hip inversion is confirmed for the current profile;
the other offsets and limits remain assumptions until the actual profile is
exported. This is not a calibration installer.

The Python forward model closes triangles with an independent law-of-cosines
construction. It does not call the firmware forward solver, the JavaScript
renderer, or Rapier. A rigid least-squares fit reconstructs chassis motion from
four planted feet without receiving the requested attitude as a constraint.
Reported foot errors are predictions, not measured foot positions. Quantized
predictions additionally assume a 27 MHz PCA9685 clock and the firmware's
270-degree / 500–2500 microsecond servo model.

The script verifies byte identity of build/render CAD, fits 28 STL pin-bore
centres, sweeps the normalized roll/pitch/yaw envelope at five ride heights,
checks four body translations, and optionally replays the recording. Initial
stand-transition rows are excluded from settled-pose comparisons. The first
4.3-second exclusion and candidate offsets are specific to this capture.

Outputs in `tmp/full-pose-audit`:

- `summary.json`: assumptions, CAD checks, counts, limits and worst cases.
- `poses.json`: every tested pose, command and pulse prediction, and fit errors.
- `pose_probe.exe`: the freshly compiled production-source probe.

The run fails if the build/render CAD diverges, pin fits fail, or the default
envelope / isolated pitch exceeds 1 mm of predicted endpoint error. It reports
candidate-profile clipping explicitly rather than treating it as a successful
physical pose. Real joint neutral, servo gain, channel mapping, contact shape
and loaded compliance still require hardware measurements.
