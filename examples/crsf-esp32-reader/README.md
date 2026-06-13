# ESP32 CRSF Reader Example

This PlatformIO example reads CRSF / ExpressLRS RC channel frames on an ESP32 without running the full Domino robot firmware.

Use it when bringing up a receiver or checking channel mappings before connecting servos.

## Hardware

- ESP32 DevKit.
- CRSF / ExpressLRS receiver.
- Receiver TX wired to ESP32 RX pin 16.
- Common ground between receiver and ESP32.

The example initializes `Serial2` at 420000 baud and prints decoded channel values to the USB serial monitor at 115200 baud.

## Build

```powershell
pio run
```

## Monitor

```powershell
pio device monitor
```

The reusable standalone version is published here:

- [Blacksheep909/ESP32_CRSF_Reader](https://github.com/Blacksheep909/ESP32_CRSF_Reader)
