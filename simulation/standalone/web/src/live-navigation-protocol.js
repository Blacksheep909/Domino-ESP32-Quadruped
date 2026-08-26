export const LIVE_NAVIGATION_ACTIONS = Object.freeze([
  "request-navigation-state",
  "set-mode",
  "arm",
  "disarm",
  "start-mission",
  "pause-mission",
  "resume-mission",
  "clear-mission",
  "upload-mission",
  "set-home",
  "goto",
  "set-geofence",
  "set-obstacle-behavior",
  "set-parameter",
]);

export const ARDUPILOT_ROVER_MODES = Object.freeze([
  { id: "manual", label: "Manual", code: 0, requiresPosition: false, autonomous: false },
  { id: "acro", label: "Acro", code: 1, requiresPosition: false, autonomous: false },
  { id: "steering", label: "Steering", code: 3, requiresPosition: false, autonomous: false },
  { id: "hold", label: "Hold", code: 4, requiresPosition: false, autonomous: false },
  { id: "loiter", label: "Loiter", code: 5, requiresPosition: true, autonomous: true },
  { id: "follow", label: "Follow", code: 6, requiresPosition: true, autonomous: true },
  { id: "simple", label: "Simple", code: 7, requiresPosition: false, autonomous: false },
  { id: "dock", label: "Dock", code: 8, requiresPosition: true, autonomous: true },
  { id: "circle", label: "Circle", code: 9, requiresPosition: true, autonomous: true },
  { id: "auto", label: "Auto mission", code: 10, requiresPosition: true, autonomous: true },
  { id: "rtl", label: "Return to launch", code: 11, requiresPosition: true, autonomous: true },
  { id: "smart-rtl", label: "Smart RTL", code: 12, requiresPosition: true, autonomous: true },
  { id: "guided", label: "Guided", code: 15, requiresPosition: true, autonomous: true },
]);

export const LIVE_NAVIGATION_CAPABILITIES = Object.freeze([
  "navigation",
  "gps",
  "lidar",
  "ardupilot",
  "autonomy",
]);

const boundedString = (value, maximum = 96) =>
  typeof value === "string" && value.length > 0 && value.length <= maximum;

const finite = (value) => Number.isFinite(Number(value));

const finiteInRange = (value, minimum, maximum) =>
  finite(value) && Number(value) >= minimum && Number(value) <= maximum;

const modeById = (mode) => ARDUPILOT_ROVER_MODES.find((candidate) => candidate.id === mode) || null;

function validEnvelope(message) {
  return Boolean(
    message &&
    message.type === "live-navigation-command" &&
    LIVE_NAVIGATION_ACTIONS.includes(message.action) &&
    boundedString(message.requestId, 96) &&
    finite(message.timestampMs) &&
    (!message.adapterId || boundedString(message.adapterId, 96)) &&
    (!message.sessionId || boundedString(message.sessionId, 96)),
  );
}

function validWaypoint(waypoint) {
  return Boolean(
    waypoint &&
    finiteInRange(waypoint.lat, -90, 90) &&
    finiteInRange(waypoint.lon, -180, 180) &&
    finiteInRange(waypoint.radiusM ?? 1.5, 0.1, 100) &&
    finiteInRange(waypoint.speedMps ?? 0, 0, 30) &&
    finiteInRange(waypoint.holdS ?? 0, 0, 360) &&
    (!waypoint.label || boundedString(waypoint.label, 64)),
  );
}

function validMission(mission) {
  return Boolean(
    Array.isArray(mission) &&
    mission.length <= 100 &&
    mission.every(validWaypoint),
  );
}

function validCoordinate(coordinate) {
  return Boolean(
    coordinate &&
    finiteInRange(coordinate.lat, -90, 90) &&
    finiteInRange(coordinate.lon, -180, 180) &&
    finiteInRange(coordinate.altM ?? 0, -1_000, 10_000),
  );
}

function validPolygon(polygon) {
  return Array.isArray(polygon) && polygon.length <= 100 && polygon.length >= 3 && polygon.every(validCoordinate);
}

export function navigationMode(mode) {
  return modeById(String(mode || "").toLowerCase()) || null;
}

export function validLiveNavigationCommand(message) {
  if (!validEnvelope(message)) return false;
  const payload = message.payload && typeof message.payload === "object" ? message.payload : message;
  switch (message.action) {
    case "request-navigation-state":
    case "arm":
    case "disarm":
    case "start-mission":
    case "pause-mission":
    case "resume-mission":
    case "clear-mission":
      return true;
    case "set-mode": {
      const mode = navigationMode(payload.mode);
      return Boolean(mode && (payload.modeCode === undefined || Number(payload.modeCode) === mode.code));
    }
    case "upload-mission":
      return validMission(payload.mission);
    case "set-home":
      return payload.useCurrent === true || validCoordinate(payload.home);
    case "goto":
      return Boolean(
        validCoordinate(payload.target) &&
        finiteInRange(payload.acceptRadiusM ?? 1.5, 0.1, 100) &&
        finiteInRange(payload.speedMps ?? 0, 0, 30),
      );
    case "set-geofence":
      return Boolean(
        typeof payload.enabled === "boolean" &&
        finiteInRange(payload.maxRadiusM ?? 0, 0, 100_000) &&
        (payload.polygon === undefined || payload.polygon === null || validPolygon(payload.polygon)),
      );
    case "set-obstacle-behavior":
      return Boolean(
        typeof payload.enabled === "boolean" &&
        finiteInRange(payload.stopDistanceM, 0.05, 20) &&
        finiteInRange(payload.slowDistanceM, 0.1, 30) &&
        Number(payload.slowDistanceM) >= Number(payload.stopDistanceM) &&
        finiteInRange(payload.maxSpeedMps ?? 0, 0, 30),
      );
    case "set-parameter":
      return Boolean(
        boundedString(payload.name, 32) &&
        (finite(payload.value) || boundedString(String(payload.value || ""), 64)),
      );
    default:
      return false;
  }
}

export function createLiveNavigationCommand(action, payload = {}, requestId, now = Date.now()) {
  const command = {
    type: "live-navigation-command",
    action,
    requestId: String(requestId || globalThis.crypto?.randomUUID?.() || `${now}-${Math.random()}`),
    timestampMs: now,
    payload: { ...payload },
  };
  return validLiveNavigationCommand(command) ? command : null;
}

export function validLiveNavigationAcknowledgement(message) {
  return Boolean(
    message &&
    message.type === "live-navigation-ack" &&
    LIVE_NAVIGATION_ACTIONS.includes(message.action) &&
    boundedString(message.requestId, 96) &&
    typeof message.accepted === "boolean" &&
    (!message.adapterId || boundedString(message.adapterId, 96)) &&
    (!message.sessionId || boundedString(message.sessionId, 96)) &&
    (!message.reason || boundedString(message.reason, 256)) &&
    (!message.navigation || typeof message.navigation === "object") &&
    (!message.state || typeof message.state === "object"),
  );
}
