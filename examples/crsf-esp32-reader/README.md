# ESP32 CRSF Reader Example

Standalone PlatformIO example for reading CRSF / ExpressLRS RC channel frames on an ESP32.

This is extracted from the Domino quadruped firmware so the receiver work can be understood without the robot state machine, inverse kinematics, or servo control code around it.

## Hardware

- ESP32 DevKit.
- CRSF / ExpressLRS receiver.
- Receiver TX wired to ESP32 RX pin 16.
- Common ground between receiver and ESP32.

The example initializes `Serial2` at 420000 baud and prints decoded channel values to the USB serial monitor.

## Build

```powershell
pio run
```

## Monitor

```powershell
pio device monitor
```

Serial monitor speed is 115200 baud.
