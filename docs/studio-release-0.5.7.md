## Domino Quadruped Studio 0.5.7 (firmware 0.8.30)

A much better walk, sidestepping on the right stick, steadier radio control, and a flat new look. Flash firmware **0.8.30** from **FIRMWARE** (hold BOOT while uploading). Then open **Gaits**, press **BALANCED** and apply it to the robot: the robot keeps the last gait you applied, so it needs the new values once.

### Walking
- **The feet no longer drag and skid.** Measured with the robot carrying its full weight in the simulator, the old trot's swinging feet were still touching the floor for almost half the swing, and every foot landed skidding forward. Each step now lifts off quickly, swings high and lands moving with the ground.
- **New presets.** Balanced now walks about **3x faster in the simulator (110 mm/s vs 34)** and stays straight (about 4 mm sideways per metre, was 61). Higher foot lift, a wider stance, a slightly lower body. Stable and Fast are retuned too.
- **Turning on the spot is about 3x faster** (Balanced 35 deg/s, was 10) and stays on its spot.
- The firmware's built-in gait, used until you apply one from Studio, is now Balanced. The old fallback barely moved.

### Sidestepping (new)
- **Right stick left/right sidesteps** when you are not pushing forward. As you push forward it hands over to steering.
- **"Right-stick steering while walking"** on the Motion smoothing panel (LIVE > Calibration) sets that hand-over: **100%** (default) steers when moving and sidesteps when standing; **0%** always sidesteps, Spot-style, so forward + right walks diagonally and you turn with the left stick.
- The stick does not need to be pushed perfectly sideways: a push within about 15 deg of sideways is a clean sidestep.
- **Heading hold:** while sidestepping, the gyro keeps the robot facing the same way, so right-then-left comes back along the same line instead of drifting back in a V. It needs the IMU (see below) and turns itself off without fresh IMU data.

### Tilt
- Entering tilt no longer waits for the sticks to be dead centre. A held stick fades in over 0.7 s so the body glides to it; with centred sticks control is immediate.

### Radio and IMU
- **The MPU6050 can have its own I2C bus.** Wire SDA to **GPIO 32**, SCL to **GPIO 33**, VCC to the ESP32's 3.3 V and GND to its GND. The firmware finds it there at boot, reads it in a background task, and falls back to the shared servo bus if it is not there. Diagnostics shows **OWN BUS** or **SHARED BUS** on the IMU I2C row.
- **Faster recovery after a stall.** If the control loop stalls (the shared I2C bus can hang for about 1 s), the old radio data is dropped and the newest stick position applies at once, instead of taking about a second and tripping failsafe.

### Studio
- **Flat look:** no more bevelled, raised buttons, tabs and cards.
- The Gaits preview's steer slider is now **STEER / SIDE** and follows the same right-stick rules as the robot.

Windows and Apple-silicon Mac builds are attached.
