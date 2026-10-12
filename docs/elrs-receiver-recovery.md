# SpeedyBee ELRS receiver recovery (2026-09-29)

The Domino robot uses a SpeedyBee Nano 2.4 GHz RX (ESP8285) on the ESP32 CRSF UART. The Boxer transmitter reported ExpressLRS 3.4.1. The receiver was restored with the official SpeedyBee target for ExpressLRS 3.6.4 and the `guttercat` binding phrase. The receiver image was written through Domino's serial bridge to the **receiver**, not the ESP32.

## Observations

- Before recovery, the receiver repeatedly showed a double blink and did not connect to the Boxer or enter WiFi automatically.
- An ordinary receiver firmware write with a phrase added did not restore the link.
- A receiver-only `--erase-all` followed by that same image restored the normal slow-blink to WiFi sequence when the Boxer was off. With the Boxer on, the user confirmed the link worked.
- The receiver's saved configuration was not backed up before erasure. The exact previous setting that entered bind mode cannot be identified afterward.

## Why the ordinary write was insufficient

ExpressLRS stores the receiver binding UID and bind-storage mode in saved configuration. Version 3.6.4 copies a UID from the flashed image only when its `flash-discriminator` changes or when saved configuration is reset. The first custom image inserted the `guttercat` UID but retained the official image's discriminator (`1990673502`). Consequently, that write could have kept the receiver's previous UID. The full erase cleared that state and let the flashed UID take effect. [ExpressLRS 3.6.4 source](https://github.com/ExpressLRS/ExpressLRS/blob/3.6.4/src/lib/CONFIG/config.cpp)

The local helper [`../tmp/configure_elrs_receiver.py`](../tmp/configure_elrs_receiver.py) now creates a separate image for any **future** ordinary update and assigns a fresh discriminator. The successfully flashed image remains unchanged. The future image passed `esptool.py image_info` with a valid ESP8266 checksum; it has **not** been flashed to the working receiver.

## Preventing another bind-mode lockout

1. On the connected Boxer, open ExpressLRS Lua → Other Devices → SB Nano 2G4 RX → Bind Storage and set **Persistent**. Volatile deliberately enters bind mode after a power cycle. [ExpressLRS Lua guide](https://www.expresslrs.org/quick-start/transmitters/lua-howto/)
2. Let the receiver remain powered for more than two seconds after each startup. Three short power cycles within two seconds are an ExpressLRS bind-mode gesture; a noisy receiver supply can mimic that sequence. [ExpressLRS binding guide](https://www.expresslrs.org/quick-start/binding/)
3. If double blinking returns, capture the receiver's Bind Storage setting and measure voltage at the receiver's power pads while it starts, before erasing anything. One clean power cycle normally exits a manually entered bind mode. [ExpressLRS binding guide](https://www.expresslrs.org/quick-start/binding/)
4. For a future phrase-changing receiver firmware write that preserves settings, use a new flash discriminator (the helper now does this) or the official ExpressLRS Configurator. Preserve the current working receiver image as the recovery baseline.

The double blink means **binding mode**, rather than a firmware bootloader fault. Slow blink means waiting for a transmitter; fast blink means WiFi. [ExpressLRS LED status](https://www.expresslrs.org/quick-start/led-status/)
