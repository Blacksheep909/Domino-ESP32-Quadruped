## HOTFIX: Domino Quadruped Studio 0.5.4 (firmware 0.8.28)

**Hotfix for anyone running the MPU6050.** After repeated I²C errors, firmware 0.8.26–0.8.27 switched the IMU off until the next reboot. The IMU now recovers by itself. Install, then flash firmware **0.8.28** from **FIRMWARE** (hold BOOT while uploading). Calibration, gaits and settings are kept.

### IMU and I²C bus (firmware 0.8.28)
- **Self-healing IMU.** Repeated read errors now *pause* the MPU6050 instead of disabling it. The firmware frees the bus (clocks SCL until SDA is released, sends STOP, restarts I²C), then re-identifies and re-configures the IMU and resumes. If that fails it retries with a growing back-off (3 s on the shared bus, doubling up to 30 s). The servos still always come first.
- **Optional dedicated IMU bus.** Moving the MPU6050's SDA/SCL wires to two free pins and building with `-DDOMINO_IMU_SDA_PIN=32 -DDOMINO_IMU_SCL_PIN=33` puts the IMU on the ESP32's second I²C controller at 400 kHz, read by a background task on the other CPU core. An IMU fault can then never stall the servo bus or the control loop. The default build keeps today's shared wiring.
- **Optional IMU power-cycle.** Powering the MPU6050's VCC from a GPIO and building with `-DDOMINO_IMU_POWER_PIN=<gpio>` lets the firmware power-cycle an IMU that stays wedged.

### Diagnostics
- **IMU I²C ERRORS** now also shows **RE-INITS** (IMU recovered) and **IMU PAUSED** while a recovery is pending. The event log notes each recovery.
- How to read a test run:
  - Errors with **BUS RESETS** that succeed and **RE-INITS** that bring the IMU back: short glitches, most likely servo noise on the shared wiring. The dedicated bus is the real fix.
  - **"I²C bus could not be freed"**: a device is still holding a line low. That points at a wedged IMU, wiring or power; the power-cycle pin or rewiring fixes it.
  - Errors that line up with servo motion or load (walking, standing up): electrical noise from the servo current.

### Studio
- **"Studio updated" notice.** On the first launch after an update, Studio shows which version you are now on, with a link to these notes.

Windows and Apple-silicon Mac builds are both attached.
