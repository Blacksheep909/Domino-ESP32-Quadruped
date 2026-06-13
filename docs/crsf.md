# CRSF / ExpressLRS Notes

Moving Domino from simpler iBUS-style receiver handling to CRSF/ExpressLRS was one of the major firmware goals.

ExpressLRS and CRSF are common in modern RC systems and give the robot a fast serial control link. The tradeoff is that the ESP32 firmware must parse the receiver stream correctly before the rest of the robot can trust stick and switch state.

## What The Custom Code Does

The CRSF layer lives in `src/crsf.cpp` and `src/crsf.h`.

It handles:

- ESP32 `Serial2` input.
- 420000 baud CRSF serial.
- Frame synchronization on CRSF device addresses.
- Length validation.
- DVB-S2 CRC8 validation.
- `0x16` packed RC channel frames.
- Sixteen 11-bit channel values.
- Conversion into 1000-2000 us style channel values.
- First- and second-order filtering.
- Link timeout detection for failsafe behavior.

The rest of the robot reads normalized channel values and link-health state instead of handling the serial protocol directly.

## Why This Matters

On a quadruped, RC input is not just steering. It changes robot state:

- Stand / stow.
- Body tilt.
- Ride height.
- Balance mode request.
- Future gait selection.

A bad switch read can move twelve servos at once. The CRSF layer exists to keep the receiver interface isolated, validated, and easier to test.

## Parser Summary

The CRSF RC channel frame uses 22 bytes of payload for 16 channels. Each channel is packed as an 11-bit value.

The parser:

1. Waits for a valid CRSF address byte.
2. Reads the frame length.
3. Buffers the payload and CRC byte.
4. Computes CRC8 using the DVB-S2 polynomial `0xD5`.
5. Accepts the frame only if CRC matches.
6. Unpacks the 16 packed 11-bit channel values.
7. Maps each channel to a 1000-2000 us style range.
8. Filters values before exposing them to the control loop.

## Reusable Reader

The robot repo includes a small local example at:

```text
examples/crsf-esp32-reader
```

The reusable standalone extraction is published separately:

- [Blacksheep909/ESP32_CRSF_Reader](https://github.com/Blacksheep909/ESP32_CRSF_Reader)

Use the standalone reader first when bringing up a receiver. It is easier to debug the radio link without the robot state machine and servo code running at the same time.
