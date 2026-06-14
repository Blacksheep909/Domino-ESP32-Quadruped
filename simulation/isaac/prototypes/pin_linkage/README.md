# Domino Pin-Linkage Prototype

This prototype is the next physics gate after the simplified one-leg articulation.

The goal is to prove that a small closed linkage can run in Isaac/PhysX with:

- One actuated revolute input pin.
- Passive revolute pins for the coupler and rocker.
- A loop-closing revolute pin.
- No contacts or gravity in the first pass.

This is intentionally smaller than the Domino leg. It is a controlled four-bar test used to find stable solver settings before adding the real CAD geometry and leg-specific proportions.

## Runtime Test

Run with the Isaac Lab Python environment:

```powershell
<isaac-python> simulation/isaac/prototypes/pin_linkage/run_pin_linkage.py `
  --headless `
  --steps 240 `
  --report-path <output-folder>/pin_linkage_report.json
```

The script authors the linkage directly into the current Isaac stage, applies a sinusoidal target to the driven crank joint, steps physics, and reports body state plus loop-closure drift.

Runtime status: the generic linkage has passed a 600-step headless Isaac/PhysX run. See [`../../reports/domino-pin-linkage-runtime.md`](../../reports/domino-pin-linkage-runtime.md).

## What Passing Means

Passing means the pin-joint concept can run without non-finite state or obvious constraint explosion. It does **not** mean the Domino lower leg is finished. The next step after this is to replace the generic four-bar dimensions with Domino linkage pivots from CAD, then compare the effective lower-linkage motion against the simplified one-leg model.
