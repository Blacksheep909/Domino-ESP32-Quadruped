# Virtual Lab Roadmap Status

This status separates implemented software from features that still require
physical hardware, wiring, or measured validation. Passing a browser test or
compiling firmware is not presented as proof that a real robot was connected.

## Implemented and verified in software

| Roadmap requirement | Current implementation | Evidence |
| --- | --- | --- |
| Separate Simulation and LIVE workspaces | Independent top-level tabs and state ownership; hidden Simulation controls cannot own inputs while LIVE is active. | Application-state tests and current usage tour. |
| Simulation heartbeat | Browser, relay, and firmware-in-the-loop acknowledgement age/RTT are tracked independently. | Heartbeat-state tests. |
| Physical connection and safety | Read-only discovery plus USB, Wi-Fi TCP, and Bluetooth SPP companion transports; paired-adapter automatic reconnect with visible bounded backoff; robot-reported state, hold-to-arm, latched E-stop, and 400 ms armed watchdog. | Companion, connection, and safety tests; ESP32 build. |
| Expected versus measured digital twin | Commanded joint/body state and independently timestamped physical feedback are aligned; stale or malformed streams disappear. | LIVE telemetry tests. |
| Data, graphs, logging, and export | Synchronized recording; selectable body, voltage, current, power, command-alignment, four foot-Z, and all 12 joint-angle graphs; IndexedDB-backed session archive restored across reloads; bounded retention; baseline/candidate optimization comparison; expanded CSV export; and diagnostic JSON bundle. | Session analysis/persistence, telemetry, and diagnostics tests. |
| Guided calibration | Five steps, floating/floor preview, safe bench acknowledgement, one-servo limited jog, offsets, directions, limits, JSON backup, and persistent ESP32 profile. | Calibration tests, SIL routing check, ESP32 build. |
| Remappable servo channels | All 12 logical joints can select unique PCA9685 outputs 0-15; changed-only confirmation and separate robot-write confirmation. | Channel-map test, current screenshot, firmware routing assertion. |
| CRSF/ELRS and gamepad input | RadioMaster/EdgeTX/OpenTX transmitters retain direct CRSF channels and detected identity. Xbox, PlayStation, and generic gamepads use normalized or per-device persistent axis/button mappings, inversion, deadzone, and response curves, with a live raw-to-processed-to-command trace. All 16 robot-side CRSF channels, RF statistics, and failsafe evidence remain available in LIVE. | HID, gamepad-profile, controller-state, and endpoint tests. |
| Simple and Expert modes | Presentation detail is independent of workspace; Expert exposes deeper diagnostics and manual body controls. | Application-state and view-state tests. |
| Separate but compatible gait labs | Simulation experiments and LIVE drafts are separate while sharing versioned JSON profiles; robot apply is disarmed-only with two-slot NVS rollback. | Gait, companion, and SIL tests; both ESP32 builds. |
| Guarded browser manual control | Maximum 30-second authority, constant-time token check, deadman, monotonic sequence, bounded axes, 250 ms neutral timeout, and CRSF-link prerequisite. | Manual-control browser tests, physical guard SIL tests, ESP32 build. |
| Repository media and local run instructions | Source is locally runnable; README contains current screenshots and a 6.8-second workflow GIF. | Production Vite build and committed media. |
| Public build documentation framework | Start-to-finish build/commissioning manual, evidence labels, functional BOM, wiring architecture, default channel map, layered gates, and a reusable as-built measurement record. | Cross-linked repository documentation and source-derived interface audit. |

The complete standalone/state suite currently contains 135 passing tests. The
same C++ controller passes the native SIL scenario, and firmware 0.6.0 builds
with both the default configuration and the optional power-monitor path.

## Implemented but awaiting physical validation

- Wi-Fi and Bluetooth are compile-disabled until a local secrets header supplies
  real credentials, a non-default PIN, and a 16+ character link key. USB remains
  the recovery path.
- Calibration persistence, gait persistence, manual control, and watchdog logic
  compile for the ESP32 and are protocol/SIL tested, but still require supported-
  chassis testing with physical power isolation and an E-stop within reach.
- Optional INA226 voltage/current/power telemetry is implemented but disabled.
  The existing PCB documentation does not prove that a monitor or suitable
  external shunt is installed. The shunt, wiring, connectors, fuse, and scaling
  must be measured before enabling it.
- The current IMU provides measured body attitude. There are no documented joint
  encoders, so LIVE deliberately does not fabricate measured servo angles.

## Physical deliverables still missing

- A real-hardware LIVE telemetry capture and repository GIF after safe electrical
  bring-up.
- A measured wiring diagram covering receiver, ESP32, PCA9685, servo rail,
  optional power monitor, fuse, regulators, and grounds.
- Measured shunt value/current range and bench comparison against independent
  voltage/current instruments.
- Joint encoders or another feedback source if per-joint physical pose comparison
  is required rather than expected servo output plus measured IMU attitude.
- A completed and physically validated as-built record containing the measured
  BOM, print settings, wiring diagram, power results, calibration, gate evidence,
  and final media. The repository now supplies the manual and record template,
  but a template is not physical validation.

The remaining robot-side gates are sequenced in the
[LIVE hardware bring-up procedure](live-hardware-bring-up.md).

## Verification commands

```powershell
cd simulation/standalone
node --test app-state.test.mjs boxer-hid.test.mjs cad-endpoint.test.mjs control-state.test.mjs firmware-service.test.mjs gait-lab.test.mjs gamepad-profile.test.mjs heartbeat-state.test.mjs linkage.test.mjs live-calibration-state.test.mjs live-companion-adapter.test.mjs live-companion-core.test.mjs live-connection-state.test.mjs live-controller-state.test.mjs live-diagnostics-state.test.mjs live-gait-state.test.mjs live-manual-control-state.test.mjs live-safety-state.test.mjs live-session-state.test.mjs live-telemetry-state.test.mjs live-view-state.test.mjs physics.test.mjs
node node_modules/vite/bin/vite.js build

cd ../sil
.\build.ps1
.\bin\domino_sil.exe --duration-ms 11500

cd ../..
pio run
```
