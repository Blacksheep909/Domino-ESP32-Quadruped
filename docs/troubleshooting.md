# Troubleshooting

## PlatformIO Does Not Build

Check:

- PlatformIO extension is installed.
- You opened the repository root, not only the `src` folder.
- `platformio.ini` is visible in the VS Code Explorer.
- Board environment is `esp32dev`.

Try:

```powershell
pio run
```

## Upload Fails

Check:

- Correct USB cable.
- Correct COM port.
- ESP32 is not held in reset.
- No other serial monitor is using the port.

Try unplugging/replugging the ESP32 and uploading again.

## No Serial Output

Check:

- Monitor speed is 115200.
- ESP32 is actually running the uploaded firmware.
- USB cable supports data, not just charging.

Command:

```powershell
pio device monitor
```

## CRSF Link Not Alive

Check:

- Receiver TX goes to ESP32 RX2 / GPIO 16.
- Receiver and ESP32 share ground.
- Receiver is powered.
- Radio is bound to the receiver.
- Receiver is configured for CRSF output.
- Firmware baud is 420000.

Use the standalone [ESP32_CRSF_Reader](https://github.com/Blacksheep909/ESP32_CRSF_Reader) project to debug the receiver without servo code.

## Servos Do Not Move

Check:

- External servo power is on.
- Servo power ground is common with ESP32 ground.
- PCA9685 is powered.
- PCA9685 I2C wiring is correct.
- Servo plug orientation is correct.
- The robot is receiving a valid stand command.

## Servos Twitch Or Brown Out

Likely causes:

- Regulator cannot supply enough current.
- Wires are too thin or too long.
- Grounds are poor.
- Battery voltage is sagging.
- Too many servos are fighting mechanical stops.

Test with one servo or one leg before powering all twelve.

## One Leg Moves Backwards

Check:

- Servo channel assignment.
- Servo horn orientation.
- `hipDir`, `upperDir`, and `lowerDir` in the leg configuration.
- Left/right mechanical mirroring.

Do not fix a wiring/channel mistake by adding random trim values.

## Tilt Or Balance Mode Behaves Strangely

Check:

- Robot is already stable in stand mode.
- IMU is detected.
- IMU mounting orientation matches the assumptions in `src/main.cpp`.
- RC switch channels match your transmitter setup.
- Sticks are centered when testing neutral pose.

Balance mode is experimental. Treat it as a tuning area, not a finished stabilizer.

