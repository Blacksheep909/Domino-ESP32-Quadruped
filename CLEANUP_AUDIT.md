# Repository data and generated outputs

Project files and documentation use paths relative to the repository root.
Machine-specific workspace locations and local maintenance records are not
part of the public project documentation.

## Source and assets

- `src/`, `include/`, and `lib/`: firmware source and dependencies.
- `cad/`: CAD sources, exports, and analysis documentation.
- `hardware/`: electronics and fabrication references.
- `simulation/urdf/` and `simulation/usd/`: robot description and simulation assets.
- `simulation/isaac/`: Isaac Sim and Isaac Lab integration scripts.
- `simulation/standalone/`: the standalone Studio application.

See the [CAD guide](cad/README.md) and
[simulation guide](simulation/README.md) for asset details and setup instructions.

## Generated data

Build products, installed dependencies, caches, and local run results are
generated separately from the tracked source. Examples include:

- `simulation/isaac/out/`
- `simulation/standalone/runtime/`
- `simulation/standalone/node_modules/`
- `simulation/standalone/dist/`
- PlatformIO `.pio/` directories
- Python `__pycache__/` directories

Use the relevant build or simulation instructions to regenerate these outputs.
Configure external runtime locations through the documented environment variables
or command-line options rather than depending on another contributor's filesystem.
