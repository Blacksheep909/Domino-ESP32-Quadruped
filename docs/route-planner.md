# Route planner and sensors

The LIVE **Sensors** view has a top-down route planner, a camera panel, and an autonomy HUD.
Planning works offline. **Sending a route to a vehicle is always a separate, explicit action.**

## Planning

| Control | What it does |
| --- | --- |
| Click the map | Place a waypoint; edit its name (`DOCK`, `GATE`...), speed, arrival radius and dwell time |
| Drag a marker | Move it; the GPS conversion stays in sync |
| Marker keys | `Delete` removes; arrows nudge 0.5 m (`Shift` 5 m) |
| **UNDO / REDO** | `Ctrl+Z`, `Ctrl+Shift+Z`, `Ctrl+Y`; covers place, drag, reorder, remove, nudge, fit, referencing, import and clear |
| **REVERSE ROUTE** | Reverses waypoint order for a return pass; undoable |
| **LOOPS** | 1x, 2x, 3x or 5x; includes the closing leg in distance and ETA |
| **FIT ROUTE** | Picks the smallest 40, 80 or 160 m range that fits the route |
| **FOCUS MAP** | Expands the map and hides the 3D preview; `Esc` returns. Presentation only |
| **CENTER DOG / FOLLOW DOG** | Re-centre or track the dog; view only |
| Map keys | `+`/`-` zoom, `F` fit, `Home` reset centre, `C` centre on dog; `Shift`+drag or middle-drag pans |

Plans made before a GPS fix use a local north/east metre frame and keep it through import and export.
Once a GPS or home reference exists, new points also carry coordinates. **REFERENCE LOCAL PLAN** binds an
offline draft to the home position; **CLEAR ORIGIN** forgets the geographic reference without deleting the draft.
The draft, route name, range, obstacle policy and geofence autosave locally, never with connection, arm or execution state.

## Import and export

- **JSON** plan, and **EXPORT / IMPORT LIBRARY** for the named-route collection (replaces only the library).
- **GeoJSON**: an ordered line plus waypoint features with label, speed, radius, dwell, loops and origin. Local drafts convert through the active origin and are refused if none exists. Import accepts ordered Points or a LineString, up to 100 waypoints, and still goes through review and geofence gates.
- **CREATE PLAN FROM TRACK** turns the recent GPS trail into a draft, keeping the first and last fixes.
- **EXPORT LOG** saves the bounded autonomy activity timeline, also included in the diagnostic bundle.

## Maps

- **LAYER**: **LOCAL GRID** is the offline default. **OPEN MAP** overlays OpenStreetMap tiles when an origin is set; tile health shows `LOADING`, `READY`, `PARTIAL` or `UNAVAILABLE`, with **RETRY MAP**. A tile failure never blocks planning.
- **OVERLAYS** hide route, track, LiDAR, fence or vehicle layers; display only.
- **USE DEVICE** fills the origin from the computer's location; **SEARCH PLACE / ADDRESS** looks up to five places via OpenStreetMap Nominatim. Either only fills the field until **SET ORIGIN**.
- **CURSOR POSITION** shows local east/north metres, and latitude/longitude once an origin is set.
- **STREET VIEW** opens a Google Maps URL at the selected point, aimed along the route. Nothing is read back; **COPY VIEW LINK** shares it.

## Running a route

| Action | Behaviour |
| --- | --- |
| **PREVIEW ROUTE** | Animates the local marker only; pause, resume, 1 s step, 0.5x-4x. Never sends a command |
| **DRIVE** (per waypoint) | Runs to that point through Domino's guarded native controller |
| **GUIDE** (per waypoint) | Explicit ArduPilot handoff for that point |
| **START DOMINO ROUTE** | App-owned execution over the time-limited manual lease |
| **PAUSE / RESUME** | Pause holds neutral and keeps the index and lease; resume rechecks every gate |
| **RETURN HOME** | Separate one-shot mode toward the home origin; the saved draft is unchanged |
| **OPEN MANUAL OVERRIDE** (`M`) | Neutralizes and releases the route, then opens the manual-control handoff |

**START DOMINO ROUTE** unlocks only when the adapter advertises `nativeNavigation` and `manualControl`,
the robot is armed, telemetry is fresh, GPS heading and position are valid, LiDAR is fresh if the obstacle
guard is on, and the route is inside the home-radius fence. A stale sensor, fence breach, disarm, link loss,
hidden tab or lease expiry sends neutral and blocks the route. The current ESP32 firmware does **not**
advertise `nativeNavigation` yet; its physical GPS and LiDAR still need integration and validation.

## HUD and obstacle guard

- The HUD shows pass summary, position quality, heading, next-point and total distance, a conservative ETA, and front obstacle distance.
- The native controller turns target, heading, front LiDAR range and the fence into a bounded forward/turn intent with a left/right clearance bias, and reports `SENSOR WAIT`, `OBSTACLE STOP` or `GEOFENCE STOP`.
- With the obstacle guard on and fresh LiDAR, four clearance sectors change colour at the slow and stop thresholds.
- The route checklist flags an empty plan, a still-local plan, waypoints outside the fence, or readiness for GPS upload.

## Camera

- Enter an HTTP or MJPEG stream URL and press **CONNECT** (or Enter). A failure shows `STREAM ERROR` and the button becomes **RETRY**.
- **SNAPSHOT** and **FULLSCREEN** stay disabled until a frame arrives. The header shows frame size and age.
- Optional robot camera telemetry supplies yaw, pitch, field of view and frame rate; nothing is faked when the robot sends none.
- **CLEAR** disconnects and forgets the URL. The diagnostic bundle includes camera state but omits the URL.
