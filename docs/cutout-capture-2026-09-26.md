# Captured control cutout, 26 September 2026

Sources: `domino-live-engineering-session-2026-09-26T08-32-22-864Z.json` and `domino-live-session-2026-09-26T08-32-26-020Z.csv` in the owner's Downloads directory, plus the owner's observation that the physical dropout happened near the end. These files are evidence, not a record of every robot packet or a direct measurement of the ELRS UART.

The session contains 305 samples. The first eight are from 06:25:51–06:25:52 UTC; then there is a 2 h 5 min gap before the relevant 08:31:44–08:32:19 UTC run. The session duration and historical packet totals therefore must not be interpreted as a continuous 2-hour test.

## The reproduced interruption

| Signal | Immediately before | First sample after | Meaning |
| --- | ---: | ---: | --- |
| Host capture time | 08:32:14.034 UTC | 08:32:16.746 UTC | 2,712 ms without a recorded sample |
| Engineering packet age | 69 ms | 78 ms | No peak was sampled inside the outage; the graph connects two fresh points across the missing interval |
| Engineering packet rate | 8.71 Hz | 4.09 Hz | The 5-second host rolling rate dropped; it recovered to ~9.45 Hz by the end |
| Robot-reported ESP32 loop rate | 43.35 Hz | 9.81 Hz | The robot's own loop-rate estimate fell sharply, then recovered to 57.60 Hz and later 65.87 Hz |
| Studio dropped / rejected packet totals | 2,681 / 2,681 | 2,689 / 2,689 | Both rose by eight; these are Studio telemetry bookkeeping, not ELRS RF-loss counters |
| Reported pack voltage | 15.775 V | 15.775 V | No sustained pack dip is visible; sampled pack voltage cannot exclude a brief ESP32 rail transient |

The preceding regular packet-age sawtooth of roughly 0–110 ms is expected from a ~10 Hz engineering telemetry stream. The flat line during the cutout is chart interpolation over missing data. It does not mean packets remained fresh.

Studio increments `rejectedPackets` when a received `live-telemetry` packet fails its connection/sequence checks. It increments `droppedPackets` when the next accepted telemetry sequence skips values. The matching +8 increments could refer to the same eight packets, so they are not independent loss counts. The export does not include the rejected packets or their reasons. The relay sequence increments for telemetry received from the robot while a live session is active; therefore the +8 is compatible with some robot packets reaching the app during the gap, but does not timestamp or prove continuous robot execution.

The robot-reported loop-rate collapse is evidence against a graph-only rendering pause. It favors a device-side stall or severe slowdown during at least part of the interruption. It does not, without boot ID/uptime, distinguish a stall from a reset, nor identify which call delayed the loop. Pure ELRS RF loss would normally leave the main loop and USB telemetry running; ELRS UART/parser trouble, I2C trouble, serial output blocking, and power/ground transients remain distinct possibilities.

The current source calls CRSF parsing, an MPU6050 I2C read, LIVE USB work, and PCA9685 servo writes in one control loop. The PCA driver’s `writeMicroseconds` reads its prescale register and then writes PWM for each of twelve servos, using the same `Wire` bus as the MPU. The installed Arduino-ESP32 `Wire` implementation has a 50 ms I2C transaction timeout. Repeated timeouts across many operations could create a long pause; the capture does not show I2C errors, so this is a testable mechanism, not a confirmed cause. The older firmware’s telemetry write path may also block on USB backpressure. Local firmware 0.8.6 source has a nonblocking best-effort periodic USB path, a larger CRSF RX ring, reset/loop-gap counters, per-stage timing, IMU I2C error count, and a best-effort RTC marker for the stage active at reset. None was present in this older export.

## Next decisive capture

Use Studio 0.2.15 and verify the robot reports firmware 0.8.6 before comparing `bootId`, `uptimeMs`, `resetReason`, `priorResetStage`, `maximumLoopGapMs`, the stage maxima, `imuI2cErrors`, `telemetryTxSkipped`, CRSF counters, and CRSF frame age across a new cutout. The diagnostic screen shows these directly; CSV and JSON exports preserve them. No firmware version or boot fields are present in the older export, so the installed versions during that recording are not established here. If boot identity stays fixed and `maximumLoopGapMs` rises, compare stage maxima to find the blocking path; if boot identity changes, compare reset reason and the retained prior stage, while investigating fast 3.3 V/ground transients if indicated. A continuing loop with stale CRSF frames would shift attention to the receiver/UART. The prior-stage marker can be lost on a power reset and is never proof of causation by itself.
