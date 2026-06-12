# Build Guide

This guide is intentionally written as a bring-up path, not just a list of parts. The goal is to avoid the common failure mode where the whole robot is assembled before any one subsystem has been proven.

## 1. Prepare The Firmware Environment

Install:

- VS Code.
- PlatformIO IDE extension.
- Git, if cloning from GitHub.

Clone or download the repo, then build:

```powershell
pio run
```

Expected result: PlatformIO builds the `esp32dev` environment without errors.

## 2. Confirm ESP32 Upload

Connect the ESP32 by USB and upload:

```powershell
pio run -t upload
```

Open the serial monitor:

```powershell
pio device monitor
```

Expected result: boot/debug logs appear at 115200 baud.

## 3. Confirm CRSF / ExpressLRS Input

Before connecting all servos, verify the receiver path.

Recommended:

- Use the standalone CRSF reader repo first: [ESP32_CRSF_Reader](https://github.com/Blacksheep909/ESP32_CRSF_Reader)
- Confirm channels move in serial output.
- Confirm switch positions match the expected channel indexes.

Robot firmware expects:

- `Serial2` CRSF input.
- RX pin 16.
- TX pin 17.
- CRSF baud 420000.

## 4. Wire Servo Power Carefully

The ESP32 should not power the servos. Use a suitable external regulator / BEC and common ground.

Check:

- ESP32 ground connected to servo power ground.
- PCA9685 ground connected to the same ground.
- Receiver ground connected to the same ground.
- Servo power voltage matches the servos.
- No servo rail short before plugging in the battery.

## 5. Test One Servo

Before attaching linkages:

1. Power one servo.
2. Confirm it can move through the expected range.
3. Confirm 135 degrees is close to mechanical center.
4. Re-clock the horn if it is far away from neutral.

Do not compensate for a badly mounted horn entirely in software if it can be fixed mechanically.

## 6. Build One Leg

Assemble one leg and test it on the bench.

Check:

- Linkage moves freely.
- Rod ends do not bind.
- Servo horns do not hit printed parts.
- The leg can reach neutral without forcing the servo.
- The leg can stow without a hard mechanical stop.

## 7. Assemble All Four Legs On A Stand

Mount the body and all legs, but keep the robot lifted so the legs are not supporting weight.

First tests:

- Boot with servo power off.
- Confirm CRSF link alive.
- Turn servo power on.
- Test stow.
- Test stand.
- Test ride-height presets.
- Test tilt only after stand is stable.

## 8. First Floor Test

Only place the robot on the floor after:

- All legs move symmetrically on the stand.
- Stand/stow transitions are predictable.
- Link-loss behavior is understood.
- Servo trims are close.

Start with short tests and keep power easy to disconnect.

