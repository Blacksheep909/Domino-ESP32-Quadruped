# Face & voice firmware steps

Where the firmware is up to for the body_v7 hardware (2.42" OLED eyes, VL53L5CX
depth sensor, MG90S antennas, MAX98357A amplifier and speaker). Update the
status column whenever a step moves; a step is not "done" until it has run on
the real robot.

Base firmware when this started: **0.8.30** (Studio 0.5.7). None of these steps
has been released, so the reported firmware version is unchanged.

| Step | What | Status |
| --- | --- | --- |
| 1 | **Droid voice player.** `src/droid_voice.*` plays the emotes in `src/droid_voice_presets.h` over I2S, behind `-DDOMINO_AUDIO_ENABLED=1`. Plays `startup` at boot. | Written and compiled. Not run on hardware (amplifier not wired yet). |
| 2 | **Sound triggers.** `src/droid_reactions.*` plays `pushed` and `fallen` from the IMU (in every mode, including while the policy walks). LIVE command `kind:"voice"` with `action:"play"` (`payload.emote`) or `action:"volume"` (`payload.percent`); capability `droidVoice`. | Firmware side written, compiled, detector checked on PC. Thresholds are untuned guesses; not run on hardware. **Still to do:** Studio buttons, and `low_battery` / `alert` from the power and safety events. |
| 3 | **OLED eyes.** Two SSD1309 128x64 panels over I2C (one needs its address moved to 0x3D), eye expressions tied to the emotes. | **Written** (branch `feat/emote-sound`, 2026-11-16), behind `-DDOMINO_FACE_ENABLED=1`; not run on hardware. `src/face_renderer.*` draws BURN-E style eyes (rounded squares, dithered glow, lids for brows, ^ happy eyes), a battery icon with percent, and 5x7 telemetry text (`src/face_font.h`). `src/face_engine.*` animates them: blinks, idle glances, eyes follow the sticks, slim sleepy lines when stowed and opening on standing, emote eye tracks (`src/face_emotes.h`, same timing as the personality emotes), wide eyes on a shove, squeezed eyes after a fall; low battery flashes on one eye (6 s every 20 s, constantly below half the threshold); battery shown for 5 s after 15 s idle, at most every N minutes; modes eyes / telemetry / off. `src/face.*`: settings in NVS record `face`, smoothed 4S battery percent, SSD1309 driver. **Wiring: both screens on the IMU's own bus (Wire1, GPIO 32 SDA / 33 SCL, 400 kHz), addresses 0x3C (left eye) and 0x3D (right eye)**; never the 100 kHz servo bus (a frame is 1 KB). The IMU task sends one changed 128-byte page per pass. Flip a screen with `DOMINO_FACE_LEFT_FLIP` / `RIGHT_FLIP`. Studio: LIVE / SCREENS (settings + live preview), and the eyes play on the 3D face in LIVE / EMOTES. Tests: `simulation/sil/test-face.ps1`, `face-render-parity.test.mjs` (Studio and firmware renderers byte-identical). |
| 4 | **Depth sensor.** VL53L5CX 8x8 ranging at 15 Hz, tilted 20 degrees down; publish the grid to LIVE, then step and obstacle detection. | Not started. |
| 5 | **Antennas.** Two MG90S on the free PCA9685 channels (5, 6, 12 and 13 are free); rest pose 45 degrees back, travel 85 degrees back to 40 degrees forward. | Not started. |
| 6 | **Emotes over the policy.** An emote is voice + eyes + antennas + body motion layered on top of the learned walk, so it has to run while the policy is driving the legs. | **Part 1 written** (branch `feat/emote-sound`, 2026-11-15): `src/personality.*` plays five emotes (curious, happy, bow, look, shy) as voice + antenna timeline + a ride-height nudge, from the Boxer's momentary **SH** switch (mixed to CH8 by default) or a Studio preview. Idle sounds are rare by default (every 1.5-3 min after 20 s standing still). Settings (master on/off, sound on/off, volume, idle sounds, switch channel, random or fixed emote) live in their own NVS record `personality`, separate from calibration; master off = silent and still from boot. Studio: LIVE / EMOTES tab. Antennas behind `-DDOMINO_ANTENNAS_ENABLED=1` (PCA channels 12/13 by default). Compiled and unit-tested on PC (`simulation/sil/test-personality.ps1`); not run on hardware. **Still to do:** eyes per emote (step 3), body pitch/roll emotes (needs a policy trained with body-pose commands), choose-your-emote control. |

## Changing the sounds

`sounds/droid_presets.json` is the single source. After editing it run:

```bash
node sounds/build.mjs
```

That rewrites the numbered WAVs in `sounds/wav/`, the listening page
`sounds/preview.html`, and the firmware table `src/droid_voice_presets.h`.
The firmware and the page run the same synthesis maths, so what you hear on the
PC is what the robot plays.

## Wiring for step 1

MAX98357A: BCLK to GPIO 26, LRC to GPIO 25, DIN to GPIO 27, VIN to 5 V, GND to
ground, speaker on the screw terminal. The GPIOs are defaults; check them
against PCB V1.1B before soldering and override with the `DOMINO_AUDIO_*_PIN`
flags in `platformio.ini` if needed. The amplifier has no volume input: volume
is `droidVoiceSetVolume()` in software, and the GAIN pin sets a fixed ceiling.
