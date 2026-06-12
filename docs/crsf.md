# CRSF / ExpressLRS Notes

Moving the robot from simpler iBUS-style receiver handling to CRSF/ExpressLRS was one of the major firmware goals for Domino.

The reason is practical: ExpressLRS and CRSF are common in modern hobby RC hardware, have low latency, and expose a richer serial protocol than older receiver approaches. The cost is that the ESP32 firmware has to parse the receiver stream properly before any robot control code can trust it.

## What The Custom Code Does

The CRSF layer lives in `src/crsf.cpp` and `src/crsf.h`.

It handles:

- ESP32 `Serial2` input.
- 420000 baud CRSF serial.
- Frame synchronization on CRSF device addresses.
- Length validation.
- DVB-S2 CRC8 validation.
- `0x16` RC channel packed frame decoding.
- 16 channels, each packed as 11-bit values.
- Conversion from CRSF channel range into 1000-2000 us style values.
- First- and second-order filtering.
- Link timeout detection for failsafe behavior.

The rest of the robot reads normalized channel values rather than touching the serial protocol directly.

## Why This Matters

On a quadruped, RC input is not just steering. It controls the robot's mode state:

- Stand / stow.
- Body tilt.
- Ride height.
- Balance mode request.
- Future gait selection.

That means receiver parsing has to be boringly reliable. A bad switch read can move twelve servos at once. The custom CRSF layer exists so the state machine receives clean, debounced, filtered values and can fall back safely when the link is lost.

## Parser Summary

The CRSF RC channel frame uses 22 bytes of payload for 16 channels. Each channel is packed as an 11-bit value.

The parser:

1. Waits for a valid CRSF address byte.
2. Reads the frame length.
3. Buffers the frame payload and CRC byte.
4. Computes CRC8 using the DVB-S2 polynomial `0xD5`.
5. Accepts the frame only if CRC matches.
6. Unpacks the 16 packed 11-bit channel values.
7. Maps each channel to a 1000-2000 us style range.
8. Filters the values before exposing them to the robot control loop.

## Standalone Example

This repo includes a standalone extraction at:

```text
examples/crsf-esp32-reader
```

That example is intentionally separated from the robot code. It is a good starting point if the CRSF reader becomes its own small GitHub project later.

## Separate Repository?

It is probably worth uploading the CRSF code separately once it has a little more polish:

- A minimal README.
- A clean single-file or small-library API.
- A wiring diagram for ESP32 RX/TX and receiver wiring.
- A serial monitor demo showing live channel values.
- A note about tested receiver/radio hardware.

For now, keeping the standalone example inside this repo is useful because it shows the CRSF work in context: this was not an isolated toy parser, it was built to solve a real robot control problem.
