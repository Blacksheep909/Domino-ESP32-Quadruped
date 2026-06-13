# Build And Bring-Up Notes

> Work in progress: this is not a complete assembly manual. It is a cautious bring-up sequence for the firmware and major hardware subsystems. The repo still needs a measured BOM, fastener list, wiring diagram, print settings, rod lengths, and annotated assembly photos before it should be treated as a full public build guide.

## 1. Firmware Environment

Install:

- VS Code.
- PlatformIO IDE extension.
- Git, if cloning the repository.

Build from the repository root:

```powershell
pio run
```

Expected result: PlatformIO builds the `esp32dev` environment without errors.

## 2. ESP32 Upload

Connect the ESP32 by USB and upload:

```powershell
pio run -t upload
```

Open the serial monitor:

```powershell
pio device monitor
```

Expected result: boot/debug logs appear at 115200 baud.

## 3. CRSF / ExpressLRS Receiver Check

Verify the radio link before connecting servos.

Recommended:

- Use the standalone CRSF reader repo first: [ESP32_CRSF_Reader](https://github.com/Blacksheep909/ESP32_CRSF_Reader)
- Confirm decoded channels move in the serial output.
- Confirm switch positions match the expected channel indexes.

Robot firmware expects:

- `Serial2` CRSF input.
- ESP32 RX pin 16.
- ESP32 TX pin 17.
- CRSF baud rate of 420000.

## 4. Servo Power Check

The ESP32 must not power the servos. Use an external regulator or BEC sized for the selected servos.

Check:

- ESP32 ground is connected to servo power ground.
- PCA9685 ground is connected to the same ground.
- Receiver ground is connected to the same ground.
- Servo voltage matches the selected servos.
- The servo rail has no short before battery power is connected.

## 5. One-Servo Test

Before attaching linkages:

1. Power one servo.
2. Command the servo to the expected midpoint.
3. Confirm the horn can be mounted close to mechanical neutral.
4. Confirm the servo does not bind at the intended limits.

Do not use large software trims to hide a badly mounted horn.

## 6. One-Leg Test

The repo does not yet contain a complete mechanical assembly manual, so the first meaningful hardware test should be one leg on the bench.

Check:

- Linkage moves freely.
- Rod ends and pivots do not bind.
- Servo horns clear printed parts.
- The leg can reach neutral without forcing a servo.
- The leg can stow without hitting a hard stop.

## 7. Full Robot On A Stand

Only connect all twelve servos once the receiver, one servo, and one leg have been tested.

First full-system tests:

- Boot with servo power off.
- Confirm CRSF link status.
- Turn servo power on.
- Test stow.
- Test stand.
- Test ride-height presets.
- Test tilt only after stand mode is stable.

## 8. Floor Testing

The robot should only be placed on the floor after:

- All legs move symmetrically on a stand.
- Stand/stow transitions are predictable.
- Link-loss behavior is understood.
- Servo trims are close.
- The power system does not brown out under load.

Start with short tests and keep power easy to disconnect.
