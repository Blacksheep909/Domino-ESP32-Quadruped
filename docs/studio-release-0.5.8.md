## Domino Quadruped Studio 0.5.8 (firmware 0.8.31)

Personality for Domino: emotes and droid sounds, a SCREENS tab for the coming OLED eyes, ears that follow tilt mode, and the newest learned walk. The sim now looks like the real robot. Flash firmware **0.8.31** from **FIRMWARE** (hold BOOT while uploading).

The face, ear and speaker hardware (body_v7) is not fitted yet, so their drivers stay off in this firmware (`DOMINO_FACE_ENABLED`, `DOMINO_ANTENNAS_ENABLED`, `DOMINO_AUDIO_ENABLED`). Everything can be set up and previewed in Studio now.

### Walking policy
- **walk_final11** is installed (iteration 12495), trained with a stronger turn and a final calm stage, so it stands quieter.
- In policy mode the sticks now ramp at the rates the policy was trained with, so it isn't jolted by instant stick changes.

### Emotes and sounds (new)
- **LIVE > EMOTES:** five emotes (curious, happy, bow, look, shy), each with a droid sound, an ear move and a small body bob. Preview them on the 3D body_v7 model with sound.
- **Boxer SH switch** plays a random emote, never the same one twice in a row. Rare idle sounds after 20 s standing still.
- Settings are saved on the robot, separate from calibration: master on/off, sound, volume, idle sounds, switch channel, and random or fixed emote. With master off the robot stays silent and still from boot.
- No emotes while Domino is lying down.

### Screens (new)
- **LIVE > SCREENS** sets up the two OLED eyes: BURN-E style rounded square eyes (four styles), glow, brightness, blinking, eyes that follow the sticks, and eye animations with each emote.
- The eyes close to slim lines when lying down and open when it stands.
- Low battery flashes on one eye. When idle, the battery shows for 5 s every few minutes.
- A telemetry mode shows raw data instead of eyes.
- The live preview runs the exact firmware renderer (tested byte for byte) on the 3D model's screens.

### Ears follow tilt mode
- In tilt mode the ears perk up and follow the body: they swing with pitch, split with roll and twist, overshoot a little and settle like floppy ears. They fold back when lying down.
- The eyes glance in the direction of the tilt.

### Simulator
- **Real-robot colours:** white printed frame and plates, black legs, servos, carbon tubes and feet.
- The simulated robot wears the body_v7 face (FACE V7 button): ears and eyes react to the simulated firmware.
- The body_v7 model is complete: parts that sat in the wrong place are back where they belong, and the hip mounts are added.

Windows and Apple-silicon Mac builds are attached.
