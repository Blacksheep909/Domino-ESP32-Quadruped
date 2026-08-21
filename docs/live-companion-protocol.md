# Domino LIVE companion protocol

The LIVE companion is the fail-closed bridge between the browser's localhost
relay and the physical robot. The browser never talks directly to a servo
controller. The companion binds every operation to one negotiated engineering
session and waits for a physical acknowledgement before reporting success.

```text
Browser LIVE UI <-> ws://127.0.0.1:8770/control
                         |
                 live-companion-adapter.mjs
                         |
              domino-robot-link-v1 / JSONL
                         |
                 physical ESP32 endpoint
```

## Running the PC companion

Start the Virtual Lab first. Then open a second PowerShell window in
`simulation/standalone` and choose one physical link:

```powershell
# Match DOMINO_LIVE_LINK_KEY in the robot's local secrets header.
$env:DOMINO_ROBOT_LINK_KEY = "your-separate-long-random-key"

# Wi-Fi TCP endpoint on the robot (IP is printed on USB serial after joining)
.\start-live-companion.ps1 -Transport wifi -RobotHost 192.168.1.123

# Wired USB serial
.\start-live-companion.ps1 -Transport usb -Device COM5

# Paired Bluetooth SPP serial port
.\start-live-companion.ps1 -Transport bluetooth -Device COM7
```

USB and Bluetooth default to 115200 baud, 8 data bits, no parity and one stop
bit. The launcher configures that serial mode before opening the device. Wi-Fi
defaults to TCP port 8766. Keep the companion terminal open; Ctrl+C commands
neutral, closes the robot link and removes the adapter from LIVE.

Wireless firmware is opt-in. Copy `src/live_robot_secrets.example.h` to the
gitignored `src/live_robot_secrets.h`, set the station SSID/password, a separate
16+ character LIVE link key, and a non-default Bluetooth PIN, then rebuild and
flash. Wi-Fi joins the configured WPA2 network and never creates an access
point. The serial monitor reports its assigned IP without printing credentials.
Bluetooth must be paired in Windows before its SPP COM port can be passed to
the launcher. USB works without a link key and remains the recovery path.
The TCP protocol is not TLS: use it only on a trusted WPA2/WPA3 LAN, do not
forward port 8766 from a router, and use a different random link key from the
Wi-Fi password. The key is checked in constant time before any wireless command
can claim ownership or reach the safety state machine.

The first authenticated transport to send a command owns the robot command
stream. Other transports remain telemetry-only until that owner disconnects or
becomes idle while safely disarmed. An owner disconnect while armed enters the
watchdog state immediately; a disconnect in calibration turns every servo off.
An E-stop is still accepted from any authenticated transport.

The adapter appears in **LIVE -> CONNECT** only after the physical endpoint has
sent a fresh hello or telemetry message. A stale or disconnected robot is
advertised as an error and cannot negotiate a session.

## Physical-link framing

The physical link uses UTF-8 newline-delimited JSON. Every JSON object occupies
one line and includes:

```json
{"protocol":"domino-robot-link-v1","type":"robot-hello"}
```

The PC reader ignores non-JSON serial log lines, allowing normal ESP32 logs to
share the USB serial port. Individual lines should remain below 64 KiB.

### Robot identity

The robot sends this after boot and whenever a link is established:

```json
{
  "protocol": "domino-robot-link-v1",
  "type": "robot-hello",
  "robotId": "domino-1",
  "robotName": "Domino",
  "firmwareVersion": "0.3.0",
  "robotState": "disarmed",
  "capabilities": {
    "telemetry": true,
    "calibration": true,
    "gaitProfiles": false,
    "persistentProfiles": false,
    "manualControl": false
  }
}
```

The companion advertises only capabilities explicitly reported by the physical
endpoint. Unsupported firmware features therefore stay disabled in LIVE rather
than being inferred from the PC adapter.

Valid states are `unknown`, `disarmed`, `arming`, `armed`, `disarming`,
`estopped`, `fault`, and `watchdog`.

### Telemetry

The robot publishes telemetry at 20 Hz. `expected` is the pose accepted by the
robot controller; `measured` comes only from physical feedback. The current
hardware has IMU attitude but no servo encoders, so firmware sends a partial
measured body pose and deliberately omits measured servo angles. LIVE can graph
roll/pitch error but keeps the measured skeleton unavailable instead of copying
commands and calling them feedback.

```json
{
  "protocol": "domino-robot-link-v1",
  "type": "robot-telemetry",
  "robotState": "disarmed",
  "robotTimeMs": 48250,
  "expected": {
    "timestampMs": 1723600000000,
    "servoAngleDeg": [135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135],
    "body": {"rollDeg":0,"pitchDeg":0,"yawDeg":0,"heightMm":280}
  },
  "measured": {
    "timestampMs": 48250,
    "body": {"rollDeg":0,"pitchDeg":0}
  },
  "power": {"voltageV":15.8,"currentA":1.1,"powerW":17.38},
  "controller": {
    "source":"boxer-elrs",
    "frameTimestampMs":1723600000000,
    "packetRateHz":150,
    "frameLossCount":0,
    "failsafe":false,
    "failsafeCount":0,
    "linkQualityPercent":95,
    "rssi1Dbm":-62,
    "rssi2Dbm":-64,
    "snrDb":8,
    "rfMode":"250hz",
    "txPowerMw":100,
    "activeAntenna":1,
    "receiverVoltageV":5.1,
    "channelsUs":[1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
  },
  "diagnostics": {"robotState":"disarmed"}
}
```

An ESP32 may use `millis()` for `robotTimeMs` and each nested timestamp. The
companion translates those monotonic values into the host clock domain while
preserving expected/measured alignment. Endpoints with a synchronized clock may
send Unix millisecond nested timestamps directly.

## Companion commands

PC-to-robot messages use `type: "companion-command"` and one of these kinds:

| Kind | Purpose | Required robot-side enforcement |
|---|---|---|
| `safety` | Request state, arm, disarm, E-stop, reset E-stop | Physical state machine, drive-link prerequisite and latched E-stop |
| `safety-heartbeat` | Armed-session watchdog | Disable outputs if not refreshed within 400 ms |
| `manual-authority` | Grant/release a maximum 30 s browser lease | Unique token, armed state and one authority owner |
| `manual-control` | Bounded stand/careful/trot axes | Deadman, token, monotonic sequence and 250 ms neutral timeout |
| `calibration` | Enter/exit bench mode, limited jog, save profile | Disarmed state, supported chassis, 5 deg/s and +/-10 deg jog bounds |
| `gait` | Read, validate, persist or roll back a gait profile | Disarmed state, full bounds validation and atomic storage |

Most commands include the validated browser envelope under `payload`. The
physical endpoint must validate it again; the companion is a second boundary,
not a replacement for robot-side checks.

After completing an operation, the robot returns:

```json
{
  "protocol": "domino-robot-link-v1",
  "type": "robot-ack",
  "kind": "safety",
  "action": "arm",
  "requestId": "browser-request-id",
  "accepted": true,
  "robotState": "armed"
}
```

Rejected acknowledgements include a concise `reason`. Calibration and gait
acknowledgements may include the profile actually stored by the robot.

For each `safety-heartbeat`, return the same sequence:

```json
{
  "protocol": "domino-robot-link-v1",
  "type": "robot-safety-heartbeat-ack",
  "sequence": 42,
  "robotState": "armed",
  "watchdogRemainingMs": 400
}
```

## Fail-closed invariants

- Connecting is read-only and never arms or moves the robot.
- No telemetry or command crosses a mismatched adapter/session boundary.
- Arming requires disarmed state, fresh telemetry and a fresh failsafe-clear
  Boxer/ELRS controller frame with acceptable RF quality.
- Browser manual control requires a robot acknowledgement before the PC assumes
  authority. Loss of the token, lease, telemetry, controller, session, or
  deadman commands neutral within 250 ms.
- The armed watchdog commands neutral and enters `watchdog` after 400 ms without
  a browser safety heartbeat.
- Calibration and gait writes are rejected while armed.
- PC request timeouts produce explicit rejection; they never produce an
  optimistic success state.

## Current ESP32 endpoint

Firmware `0.3.0` implements the matching USB serial endpoint. Build and flash
the normal PlatformIO `esp32dev` environment, connect the ESP32 over USB, then
run the companion with `-Transport usb -Device COMx`.

The endpoint boots with every PCA9685 channel fully off. It currently supports
telemetry, CRSF link statistics, arm/disarm, a latched E-stop, the 400 ms armed
watchdog, bounded calibration bench jogs, and persistent calibration profiles.
Arming additionally requires the Boxer SA switch low; after arming, the normal
Boxer controls remain primary.

Calibration schema v2 saves contain exactly 12 stable logical channels and 12
unique physical PCA9685 output channels. Each record also carries bounded
offsets, direction and logical limits. A bench jog names both the logical joint
used for safety bounds and the physical channel to energize; the firmware keeps
only one such output active. Firmware turns all outputs off before a map change,
writes a candidate NVS
blob, reads it back, validates its checksum and full contents, and only then
replaces the active profile. At boot, a missing or corrupt active blob falls
back to the compiled safe defaults. Runtime commands are converted back to a
neutral-relative logical joint angle, clamped to the saved joint limits, then
mapped through the saved direction and offset before the existing hard servo
envelope is applied.

Gait persistence and browser manual driving remain disabled and are advertised
as unsupported until their independent robot-side enforcement is implemented.
Wi-Fi TCP and Bluetooth SPP are implemented but compile disabled until the
local secrets header explicitly enables them.
