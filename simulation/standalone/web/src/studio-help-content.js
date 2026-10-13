const menuLabels = {
  simulation: "Open Simulation", tuning: "Open TUNE", controller: "Open MAP", inspection: "Open Inspect",
  connection: "Open Pair robot", compare: "Open Compare", data: "Open Data", sensors: "Open GPS / LiDAR",
  calibration: "Open Calibration", smoothing: "Open Motion response", gaits: "Open Gaits",
  diagnostics: "Open Diagnostics", sessions: "Open Sessions", manual: "Open Manual control",
  firmware: "Open Upload FW", project: "Open Project",
};
const section = (title, steps, target = null, openLabel = null) => ({ title, steps, target, openLabel });
const guide = (id, group, title, summary, target, sections) =>
  ({ id, group, title, summary, target, openLabel: menuLabels[target], sections });
const calibrationStep = (step) => ({ view: "calibration", step });

export const STUDIO_HELP_GUIDES = [
  guide("start", "Start here", "Getting started", "Try the controls in Simulation, then use LIVE to connect to your robot.", "simulation", [
    section("Try a short simulation", [
      "Open **Simulation** and wait for the model to load. Check that **Firmware** and **Physics** are running in the links panel.",
      "Click **Standing**, then choose **Careful** and gently move the forward input. Use **Stand** to stop stepping.",
      "Use **Reset robot** to start again. **Reset view** only recentres the camera.",
    ]),
    section("Connect your robot", [
      "Open **LIVE**, then **Pair robot**. Search for your robot, select it and click **Connect**.",
      "Wait for the connected notification. **Pair robot** changes to **Manage link** when connected. The robot’s radio readings are in **Diagnostics**.",
      "Leave the robot disarmed while you read its calibration. Use **Compare** to check its state before any physical movement.",
    ], "connection", "Open Pair robot"),
  ]),
  guide("simulation", "Simulation", "Simulation controls", "Practice standing, tilting and walking with the virtual robot.", "simulation", [
    section("Stand, tilt and walk", [
      "Click **Sitting / Standing** or press **Space** to change posture. Click **Level / Tilt** or press **T** to use body tilt.",
      "Choose **Stand**, **Careful** or **Trot**. **G** cycles through them. On the radio, CH7 / SC is up for stand, middle for careful and down for trot.",
      "Set the standing height with **Ride height** (CH3). Centre the motion sticks when changing modes, then wait for the robot to settle before moving them again.",
      "Press **R** to reset the robot. Use **Reset view** if you only want to recenter the camera.",
    ]),
    section("Check what is driving the model", [
      "The **Command** panel names the input source. Watch CH1–CH8 while moving one stick or switch at a time.",
      "The links panel lists firmware, controller and physics separately. **Model health** shows problems such as failed IK or a clipped servo target.",
      "For a firmware check, turn **Override** off in TUNE. The simulation helps support the body, so staying upright here does not prove the robot will balance on the floor.",
    ], "tuning", "Open TUNE"),
  ]),
  guide("tuning", "Simulation", "Tuning a gait", "Change one setting at a time, preview the result and save the profiles you want to keep.", "tuning", [
    section("Make a small change", [
      "Open **TUNE** beside the walking controls. Choose **Stable**, **Balanced** or **Fast** as a starting point.",
      "Enable **Override** to use the Studio gait preview. **Cadence** changes step speed, **Stride** changes step length and **Clearance** changes foot lift.",
      "Use **Ground time** to change how long a foot stays down. **Gait height** and **Stance width** change the walking stance.",
      "Try a short straight run before changing turn gain, response, swing shape or diagonal phase. Watch the feet through mode changes as well as during steady walking.",
    ]),
    section("Save and use a profile", [
      "Enter a **Profile name** and click **Save**. Wait for the saved notification. Select a saved profile and click **Load** to use it again.",
      "Open **LIVE → Gaits** to review the same profile before sending it to your robot. Saving in TUNE only saves on this PC.",
      "Turn **Override** off when you finish checking the preview. Closing TUNE leaves that setting as it was.",
    ], "gaits", "Open Gaits"),
  ]),
  guide("controller", "Simulation", "Controller mapping", "Assign a gamepad's sticks and buttons to the simulator.", "controller", [
    section("Set up your gamepad", [
      "Connect the controller, then open **MAP** in the Command panel. Check that the correct device name appears.",
      "Move one stick and watch **Raw → Processed → Robot command**. Choose the matching Roll, Forward and Turn axes, then set the Stand, Tilt and Reset buttons.",
      "Use **Invert** if an axis moves the wrong way. Increase **Deadzone** if the stick drifts at centre; a higher **Response curve** makes small movements gentler.",
      "Click **Save mapping** and check the result. Close the menu, then verify each input in the channel strip.",
    ]),
    section("Using a radio", [
      "MAP changes gamepad controls. Radio channel assignments come from the transmitter's model settings.",
      "Check the Command source name before testing. In LIVE, receiver health comes from the robot and appears under **Diagnostics**.",
    ]),
  ]),
  guide("inspection", "Simulation", "Inspecting joints", "Check joint travel and foot motion from a useful camera angle.", "inspection", [
    section("Inspect a leg", [
      "Enable **Inspect**, then choose FR, FL, BR, BL or All. The joint labels and motion panel follow that selection.",
      "Double-click a part to select its joint. The callout shows the model angle and configured travel; expand its details to see the estimated servo output and distance to each limit.",
      "Use **Focus joint** to bring it into view. The axis gizmo gives front, side and top views; its home button recentres the camera.",
      "Adjust **Opacity** if the labels are hard to see. You can move the motion panel by its header grip.",
    ]),
    section("Check geometry without the floor", [
      "Enable **Float** to suspend the model. Turn it off to put the model back on the simulated floor.",
      "A CAD proximity warning needs a visual check. The small hip pivot nubs in this model are absent from this robot, and wiring or flex may differ from the model.",
    ]),
  ]),
  guide("connection", "LIVE", "Connect your robot", "Pair Studio with your robot and check the result before using its controls.", "connection", [
    section("Connect", [
      "Click **Pair robot** in the header. For Domino, choose the Domino protocol and the USB, network or Bluetooth connection you are using.",
      "Set the companion endpoint if your transport needs one, then click **Search**. Choose your robot from the results and check its identity.",
      "Click **Connect**. The button and notification show progress, followed by a connected message or a reason it failed.",
      "**Manage link** means Studio is paired. Check the transmitter’s readings in **Diagnostics** too; pairing alone does not confirm a healthy radio link.",
    ]),
    section("Disconnect or try again", [
      "Disarm the robot, open the connection manager and click **Disconnect**. Wait for the disconnected notification.",
      "If the link drops, Studio shows that it is reconnecting. **Cancel retry** stops those attempts.",
      "For a failed connection, check the cable or endpoint and read the error in this menu. Use **Restart link** or **Clear fault** as offered, then try again.",
      "If the PC stays connected but telemetry is stale, check the robot and companion. The displayed pose may be the last packet received.",
    ]),
    section("ArduPilot vehicles", [
      "Choose the ArduPilot protocol and enter the MAVLink host and port. The available menus depend on the features reported by that vehicle.",
    ]),
  ]),
  guide("compare", "LIVE", "Compare", "Check the robot's reported state, commands and available measurements in one view.", "compare", [
    section("Read the digital twin", [
      "Connect your robot and check its state in **Compare** and its radio link in **Diagnostics**. The model follows reported servo commands; it does not measure the physical joint angles.",
      "Read **Body pose**, **Joint error** and **Power**. A **No stream** or stale label means that measurement is unavailable.",
      "Use **Inspect** and the leg selector to look more closely. **Float** changes the preview only.",
      "Open **Data** for graphs or **Diagnostics** to investigate a fault.",
    ]),
    section("Before physical movement", [
      "Check that the robot is supported and the motion inputs are centred, then use **Arm** when you are ready. Wait for the robot to report that it is armed.",
      "Use **Disarm** to end motion authority. **E-stop** latches a stop; fix the cause before acknowledging the fault.",
      "Open **Manual control** if you want to drive from Studio. It has its own control request and a hold-to-drive button.",
    ], "manual", "Open Manual control"),
  ]),
  guide("data", "LIVE", "Data and recording", "Graph telemetry, record a run and export it for comparison.", "data", [
    section("Build a graph", [
      "Connect your robot, then choose a pose, power or joint signal. Missing measurements stay blank.",
      "Click **+ Graph** for another panel. Choose its signal and display settings, and drag the header grip to reorder panels.",
      "Drag within a graph to inspect a time range. Check the alignment readout when comparing commanded and measured values.",
    ]),
    section("Keep a run", [
      "Click **Start recording** before the run and **Stop recording** when finished. Studio saves the session on this PC; check the save notification.",
      "Use **Export CSV** for a spreadsheet or **Export JSON** to keep the structured recording. Open **Sessions** to find saved runs.",
      "Export or save anything you need before using **Clear data**, which clears the current buffer.",
    ], "sessions", "Open Sessions"),
  ]),
  guide("sensors", "LIVE", "GPS, LiDAR and routes", "Check sensor readings, view a camera feed and plan a route.", "sensors", [
    section("Check the sensors", [
      "Look at **Sensor bus** and the packet ages. GPS fix, satellite count and accuracy should update from the robot.",
      "Use the **3D IMU inspector** to check roll, pitch and yaw. To capture a level reference, keep the robot still and follow the support, power and axis checks shown there.",
      "**Capture level** saves a reference in Studio; **Reset reference** removes it. Saving on the robot is available only when its firmware supports sensor calibration.",
      "For LiDAR, review the Stop and Slow distances before **Apply to robot**, then check the result.",
    ]),
    section("Open a camera feed", [
      "Exit **Map focus** to see the **Camera / Front** panel. If it is hidden, widen the Studio window.",
      "Enter the camera's HTTP or MJPEG stream URL and click **Connect**. Wait for the first image and the connected notification.",
      "Use **Cancel connect** while waiting, **Retry** after an error, or **Disconnect** to stop the feed. **Clear** removes the saved URL.",
      "**Snapshot** downloads the current image, if the camera server allows it. **Fullscreen** enlarges the feed. Yaw, pitch and FOV display readings from the robot.",
    ], { view: "sensors", focus: "live-camera-url" }, "Open camera panel"),
    section("Plan a route", [
      "Choose a map reference or enter latitude and longitude. You can plan locally without a robot; geographic waypoints need a map reference.",
      "Add waypoints and select one to edit it. Arrow keys move it 0.5 m; **Shift + arrows** moves it 5 m. **Delete** removes it, and **Ctrl + Z** undoes an edit.",
      "Set the geofence and obstacle options, then preview the route. Give it a name under **Saved routes** or export a copy.",
    ]),
    section("Send a route", [
      "Connect the intended vehicle and check its position, mode and navigation support.",
      "Review the route and its checks before sending or starting it. Wait for the vehicle's acknowledgement.",
      "Use **Stop / Hold** to halt the mission. **Autonomy activity** records transfers and execution events for troubleshooting.",
    ]),
  ]),
  guide("calibration", "LIVE", "Servo calibration", "Set the servo centres and travel limits. Send to robot saves them on the robot immediately; it does not need a firmware upload.", "calibration", [
    section("Use an existing JSON profile", [
      "Connect the robot and leave it disarmed. Click **Read robot** to see the calibration it is currently using.",
      "Open **05 Review**, click **Import JSON** and choose your profile. Your imported values appear as a draft alongside the **On robot** values.",
      "Check all 12 joints: channel, direction, trim and travel. Trim corrects the physical servo centre, so changing it does not move the CAD's neutral feet.",
      "Complete **01 Safety** and request bench mode. Return to **05 Review**, click **Send to robot** and confirm the profile.",
      "Wait for **Saved and verified on robot**. If the save fails or the values differ, Studio keeps your draft and explains what to check.",
      "The save turns outputs off. For a physical check, request bench mode again and use a small joint jog. Power-cycle and read the robot again to check that the profile survives a restart.",
    ], calibrationStep("review"), "Open Review"),
    section("1. Prepare the bench", [
      "Support the chassis with the feet clear. Remove load from the selected leg and keep the servo power switch within reach.",
      "Complete the safety checks and click **Request bench mode**. Wait for the robot to confirm it before trying a physical jog.",
    ], calibrationStep("safety"), "Open Safety"),
    section("2. Choose a joint", [
      "Select an actuator on the leg map. **CH** is its physical servo output.",
      "Use **Edit channel map** if the wiring differs. Each of the 12 actuators needs a unique output; apply the map to your draft, then review it before sending.",
    ], calibrationStep("select"), "Open Select joint"),
    section("3. Set the neutral", [
      "Adjust **Offset** to match the physical servo centre. Change **Direction** if the shaft turns the wrong way.",
      "The model slider, **Play sweep** and **Neutral** let you inspect the geometry. **Enable physical jog** is a separate control, available in bench mode, with a ±10° range at 5°/s.",
      "Test jog is temporary. Only channel, direction, offset and travel limits are saved.",
    ], calibrationStep("neutral"), "Open Set neutral"),
    section("4. Set the travel limits", [
      "Enter **Min** and **Max** around the calibrated neutral. Check the linkage and displayed electrical range at both ends.",
      "Use the model slider to inspect the chosen travel. Check clearance on the real robot too; the CAD's hip pivot nubs are absent from this build.",
      "The existing electrical servo caps still apply. A configured mechanical range cannot extend an electrical output beyond those caps.",
    ], calibrationStep("limits"), "Open Set limits"),
    section("5. Save and back up", [
      "**Save browser copy** saves on this PC. **Send to robot** applies and stores the profile on the connected robot.",
      "**Export browser draft** backs up your edits. **Export robot profile** reads and exports the robot's active calibration.",
      "If a save cannot be verified, click **Read robot** and compare the values before retrying. **Use robot values** replaces your draft with that read-back.",
      "**Restore defaults** changes your draft. The robot changes only after you send it.",
    ], calibrationStep("review"), "Open Review"),
  ]),
  guide("smoothing", "LIVE", "Motion smoothing", "Set how quickly height and tilt commands change, then preview the response before saving.", "smoothing", [
    section("Try a slower response", [
      "Open **Calibration → 05 Review → Motion response / Ramp**. Choose Roll, Pitch, Yaw or Height and set a **Test move**.",
      "Lower a rate to make the command approach its target more slowly. The graph shows the time it will take.",
      "Increase **Centre deadband** to ignore small stick movements near zero. Use the centre-response graph to see its effect.",
      "The smoothing switch controls these optional ramps. Mode changes still use a gradual transition when this switch is off.",
    ]),
    section("Save the settings", [
      "Use **Read robot** in Motion response to check the active settings. **Default draft** resets your local values.",
      "Leave bench mode and keep the robot disarmed, then click **Save to robot**. Wait for the verified-save result.",
      "This preview shows the shaped command, not the physical servo's measured response.",
    ]),
  ]),
  guide("gaits", "LIVE", "Gait profiles", "Review a gait from your local library and send it to the robot when ready.", "gaits", [
    section("Prepare a profile", [
      "Load a saved profile from TUNE or import a gait JSON. Check its stance, foot paths and timing in the preview.",
      "Make small changes and read any validation messages. A preview or library save only changes Studio.",
      "Click **Request robot profile** to read the active gait. Compare the settings and export a backup before replacing it.",
    ]),
    section("Apply it", [
      "Connect your robot, leave it disarmed and complete the apply review. The robot must support gait profiles and saving them.",
      "Send the profile and wait for the saved result and matching read-back. If it is rejected, fix the reported setting before trying again.",
      "Return to **Compare** for a physical run. Start with careful walking; centre the motion sticks through each mode change.",
    ], "compare", "Open Compare"),
  ]),
  guide("diagnostics", "LIVE", "Diagnostics", "Find where an input stops becoming a valid servo command.", "diagnostics", [
    section("Trace a fault", [
      "Connect the robot and check that telemetry is fresh before reading the counters.",
      "Follow the command chain from input to servo output. Open the first warning or failed stage and read its reason.",
      "Check IK failures, clipped targets and servo-write errors. The radio section shows received channels, frame age and failsafe state.",
      "Use the timestamped events to find what changed just before a problem. Record a run in **Data** if it happens intermittently.",
      "Fix the reported cause before clearing a robot fault and testing again.",
    ]),
  ]),
  guide("sessions", "LIVE", "Saved sessions", "Reopen recordings, back them up and compare two runs.", "sessions", [
    section("Find a recording", [
      "Start and stop a recording in **Data**. Check its save result, then open **Sessions**.",
      "Select a saved run to see its samples and summary. Use **Export JSON** to keep a copy or **Import JSON** to reopen one from another PC.",
      "Export anything you need before deleting a session from the local list.",
    ]),
    section("Compare two runs", [
      "Choose a baseline and a candidate under **Compare saved runs**. Use similar controller, gait and test conditions.",
      "Compare pose tracking, alignment and power where measurements are available. A commanded angle is not a measured joint angle.",
    ]),
  ]),
  guide("manual", "Tools", "Manual control", "Use Studio's controls to drive the robot while holding the deadman button.", "manual", [
    section("Take control", [
      "Open **Manual control** in LIVE. Check the robot and radio state, then arm through the safety controls when ready.",
      "Request browser control and wait for it to be granted. This permission expires, so check it before each run.",
      "Choose **Stand**, **Careful** or **Trot**, hold **Hold to drive** and move the controls gently. Centre the inputs during a mode change.",
      "Release the deadman to return the inputs to neutral. Closing the menu or leaving LIVE releases browser control; use **Disarm** when the run is finished.",
    ]),
  ]),
  guide("firmware", "Tools", "Upload firmware", "Build the current firmware, then upload the validated package to the ESP32.", "firmware", [
    section("Build first", [
      "Open **Upload FW** in LIVE. Check the board and package, and select any source file you want to review.",
      "Click **Build & validate**. Wait for a successful build and check the deployment log.",
      "To change servo calibration, use **Calibration → Send to robot**. Uploading firmware does not apply an imported calibration draft.",
    ]),
    section("Upload to the ESP32", [
      "Export a calibration backup. Support the robot, isolate servo power and leave it disarmed. Studio releases its robot connection when you start uploading.",
      "Connect the ESP32 over USB and select its upload port. **Refresh ports** updates the list.",
      "Hold **BOOT**, tick the preparation checkbox and click **Upload to ESP32**. Release BOOT when connecting begins, as shown in the uploader.",
      "Wait for upload success. If the robot was paired, Studio tries to reconnect after it restarts. Read its calibration and gait profile again before restoring servo power.",
    ]),
  ]),
  guide("project", "Tools", "Project files", "Keep your Studio settings in a portable project file.", "project", [
    section("Save or open a project", [
      "Click **Project** in the header and enter a name. **Export .qstudio** saves tuning, gait profiles, gamepad mappings, calibration drafts and routes.",
      "Use **Import project** to restore a compatible file. Review the imported calibration and gait before sending them to your robot.",
      "Recordings are kept in **Sessions**. Connections, arming and bench-mode permissions belong to the current robot session and are not restored from a project file.",
    ]),
  ]),
];

export const STUDIO_KEYBINDS = [
  ["Space", "Stand / sit", "Simulation"], ["T", "Level / tilt", "Simulation"],
  ["G", "Cycle stand / careful / trot", "Simulation"], ["R", "Reset robot", "Simulation"],
  ["0", "Open Simulation", "Workspace"], ["1", "Compare", "LIVE tabs"],
  ["2", "Data", "LIVE tabs"], ["3", "GPS / LiDAR", "LIVE tabs"],
  ["4", "Calibration", "LIVE tabs"], ["5", "Gaits", "LIVE tabs"],
  ["6", "Diagnostics", "LIVE tabs"], ["7", "Sessions", "LIVE tabs"],
  ["M", "Open manual control", "LIVE"], ["Delete / Backspace", "Remove selected waypoint", "Route editor"],
  ["Arrow keys", "Move selected waypoint 0.5 m", "Route editor"], ["Shift + arrows", "Move selected waypoint 5 m", "Route editor"],
  ["Ctrl + Z", "Undo route edit", "Route editor"], ["Ctrl + Shift + Z / Ctrl + Y", "Redo route edit", "Route editor"],
  ["H", "Collapse / expand autonomy HUD", "Workspace"], ["F", "Toggle fullscreen", "Workspace"],
  ["?", "Open this guide", "Workspace"], ["Escape", "Close help or exit map focus", "Workspace"],
];

export function findStudioGuides(query = "") {
  const terms = String(query).trim().toLowerCase().split(/\s+/).filter(Boolean);
  return STUDIO_HELP_GUIDES.filter((entry) => {
    const text = [entry.group, entry.title, entry.summary,
      ...entry.sections.flatMap(({ title, steps }) => [title, ...steps])].join(" ").replaceAll("**", "").toLowerCase();
    return terms.every((term) => text.includes(term));
  });
}
