# Portfolio Media Roadmap

This page tracks media that would make the project easier to evaluate visually. It is intentionally a roadmap, not a claim that the repository already contains every build photo or demo clip needed for a complete public build guide.

## Already Added

- Hero photo of the Domino prototype.
- Electronics cage photo showing the serviceable central body layout.
- Side photo showing the leg linkage, rods, pivots, and servo placement.
- CAD side view.
- Domino PCB V1.1B layout and render screenshots.
- Simulation workspace overview.
- LIVE digital-twin workspace overview.
- LIVE connection manager and fail-closed safety state.
- LIVE synchronized telemetry recording and data graph.
- LIVE guided servo-calibration workflow.
- LIVE gait-profile transfer workflow.
- LIVE command-chain and controller diagnostics.
- Short locally captured Virtual Lab tour covering Simulation, LIVE comparison,
  channel mapping, Data, Gaits, and Diagnostics.
- Current physical servo channel-map editor screenshot.

## Highest-Value Missing Media

- Short clip of stand-to-stow movement.
- Short clip of body tilt responding to transmitter input.
- Annotated wiring photo for receiver, ESP32, PCA9685, and servo power.
- CAD view with one leg isolated and labeled.
- Isaac Sim / USD import screenshot showing the current simulation state.
- A short, real-hardware LIVE telemetry session after the electrical bring-up is
  safe enough to publish.

## Future Portfolio Improvements

- Side-by-side comparison of the earlier SpotMicro build and Domino.
- Before/after packaging comparison showing the move toward the central electronics cage.
- Photo of the RC transmitter and ExpressLRS receiver setup.
- Photo of the robot safely supported during calibration.

## Short GIF capture list

Keep each clip focused, silent, and roughly 4-8 seconds so GitHub can render it
without turning the repository into a video archive:

1. Switch from Simulation to LIVE and show that the two workspaces retain their
   own state.
2. Toggle the calibration preview between floating and floor modes.
3. Start a synthetic telemetry recording, let the graph fill, then stop it.
4. Edit a gait in Simulation, export it, and open the same profile in LIVE.
5. Connect the synthetic verification adapter and demonstrate the armed-state
   prerequisites without enabling any physical servo output.

Do not stage recordings that expose Wi-Fi credentials, Bluetooth PINs, serial
device identifiers, or a real robot link key.
