## Domino Quadruped Studio 0.5.0 (firmware 0.8.26)

Turning on the spot, a rebuilt careful walk, a Gaits page that shows turning, and a hardened I2C bus so the MPU6050 can be used again. Flash firmware 0.8.26 after installing.

### Turning and steering (firmware and simulator)
- **Left stick left/right turns on the spot.** Each planted foot now sweeps along the circle around the body centre (with sideways foot motion), instead of the old "tank" scheme that only lengthened one side's stride and drifted forward.
- **Right stick left/right steers while walking.** Its turn is scaled by forward speed, so it bends the path and does nothing when standing still.
- Right stick up/down still drives forward and back. LIVE manual control's roll axis still rolls the body.

### Careful walk rebuilt
Careful mode lifted one foot with no weight shift. That left the centre of mass on the line between the two diagonal feet still down, so the robot tipped toward the lifted corner and that foot dragged. Now:
- the body shifts 30 mm away from each leg before it lifts;
- there is a short four-feet-down pause between steps to make that shift;
- legs lift in the lateral order back-left, front-left, back-right, front-right.

The simulator confirms at least three feet stay down throughout.

### Gaits page
- **SPIN** (left stick) and new **STEER** (right stick) preview sliders, with **TURN RATE** and **TURN RADIUS** readouts, so turn gain and turn limit are visible.
- The robot stays centred while the floor moves and turns under it, and a green ribbon shows the next 6 s of path.
- Profile library: **DUPLICATE**, **RENAME** and **DELETE SELECTED** (asks first; affects this PC only).
- The preview camera now aims at the robot instead of leaving it clipped at the bottom.

### I2C bus and MPU6050 (firmware 0.8.26)
The IMU was disabled because plugging in the MPU6050 made the shared I2C bus hang and stalled the servo driver. The most likely cause is servo noise interrupting a long IMU read, which leaves the IMU holding SDA low. Firmware 0.8.26:
- reads the IMU at a fixed 100 Hz instead of on every loop pass;
- identifies the MPU6050 (0x68 or 0x69) and verifies its configuration at boot;
- detects a stuck bus at boot, before every servo tick and after any IMU error, and frees it (clocks SCL until SDA is released, sends STOP, restarts I2C);
- stops reading the IMU until reboot after 25 errors in 10 s, so the servos always come first;
- halves servo-bus traffic by not re-reading the PCA9685 prescale before every servo write.

The IMU is enabled again; with no MPU fitted it is probed once at boot and then ignored. **Diagnostics** now reports bus recoveries (with a chart option), failed recoveries, and an IMU shut-off. Balance mode remains disabled; the IMU only adds measurements.

### Upgrading
Install over any previous version (or use **RESTART & UPDATE**), then flash firmware **0.8.26** from Studio (hold BOOT while uploading). Calibration, gaits and settings are kept.
