# Troubleshooting Notes

> Work in progress: this page covers common bring-up checks. It is not a complete fault tree for every hardware configuration.

## PlatformIO Does Not Build

Check:

- PlatformIO extension is installed.
- The repository root is open, not only the `src` folder.
- `platformio.ini` is visible in the VS Code Explorer.
- The active environment is `esp32dev`.

Command:

```powershell
pio run
```

## Upload Fails

Check:

- USB cable supports data.
- Correct COM port is selected.
- ESP32 is not held in reset.
- No other serial monitor is using the port.

Try unplugging and reconnecting the ESP32 before uploading again.

## No Serial Output

Check:

- Monitor speed is 460800 for current application logs. ESP32 ROM boot text
  uses its fixed startup baud and may look garbled before Domino starts.
- ESP32 is running the uploaded firmware.
- USB cable supports data.

Command:

```powershell
pio device monitor
```

## CRSF Link Not Alive

Check:

- Receiver TX is wired to ESP32 RX2 / GPIO 16.
- Receiver and ESP32 share ground.
- Receiver is powered.
- Radio is bound to the receiver.
- Receiver is configured for CRSF output.
- Firmware baud rate is 420000.

Use [ESP32_CRSF_Reader](https://github.com/Blacksheep909/ESP32_CRSF_Reader) to debug the receiver without servo code.

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
- Wiring is too thin or too long.
- Grounding is poor.
- Battery voltage is sagging.
- Servos are fighting mechanical stops.

Test with one servo or one leg before powering all twelve.

## One Leg Moves Backwards

Check:

- Servo channel assignment.
- Servo horn orientation.
- `hipDir`, `upperDir`, and `lowerDir` in the leg configuration.
- Left/right mechanical mirroring.

Do not compensate for wiring or channel mistakes with random trim values.

## Tilt Or Balance Mode Behaves Incorrectly

Check:

- Robot is stable in stand mode first.
- IMU is detected.
- IMU mounting orientation matches the assumptions in `src/main.cpp`.
- RC switch channels match the transmitter setup.
- Sticks are centered when testing neutral pose.

Balance mode is experimental and should be treated as a tuning area, not a finished stabilizer.
