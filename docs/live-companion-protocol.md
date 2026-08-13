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
# Wi-Fi TCP endpoint on the robot
.\start-live-companion.ps1 -Transport wifi -RobotHost 192.168.4.1

# Wired USB serial
.\start-live-companion.ps1 -Transport usb -Device COM5

# Paired Bluetooth SPP serial port
.\start-live-companion.ps1 -Transport bluetooth -Device COM7
```

USB and Bluetooth default to 115200 baud, 8 data bits, no parity and one stop
bit. The launcher configures that serial mode before opening the device. Wi-Fi
defaults to TCP port 8766. Keep the companion terminal open; Ctrl+C commands
neutral, closes the robot link and removes the adapter from LIVE.

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
  "firmwareVersion": "0.2.0",
  "robotState": "disarmed"
}
```

Valid states are `unknown`, `disarmed`, `arming`, `armed`, `disarming`,
`estopped`, `fault`, and `watchdog`.

### Telemetry

The robot should publish telemetry at 20-50 Hz. `expected` is the pose accepted
by the robot controller; `measured` comes from physical feedback/IMU data. Do
not copy expected into measured when sensors are unavailable—omit measured so
the browser reports it unavailable.

```json
{
  "protocol": "domino-robot-link-v1",
  "type": "robot-telemetry",
  "robotState": "disarmed",
  "expected": {
    "timestampMs": 1723600000000,
    "servoAngleDeg": [135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135],
    "body": {"rollDeg":0,"pitchDeg":0,"yawDeg":0,"heightMm":280}
  },
  "measured": {
    "timestampMs": 1723600000001,
    "servoAngleDeg": [135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135],
    "body": {"rollDeg":0,"pitchDeg":0,"yawDeg":0,"heightMm":279}
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

Timestamps are Unix milliseconds. If the robot does not have a synchronized
clock, the endpoint should maintain a host-time offset established during its
link handshake rather than sending `millis()` as an epoch timestamp.

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

The repository now contains the PC companion and its tested protocol boundary.
The production ESP32 firmware still needs the matching physical endpoint before
LIVE can control real hardware. Until that endpoint independently enforces all
rules above, use the adapter only with a mock or bench-isolated robot.
