import { ARDUPILOT_ROVER_MODES } from "./live-navigation-protocol.js";

const finite = (value) => {
  if (value === null || value === undefined || value === "") return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
};

const clamp = (value, minimum, maximum) => Math.max(minimum, Math.min(maximum, Number(value)));

const boundedText = (value, fallback = "") =>
  typeof value === "string" ? value.slice(0, 160) : fallback;

const coordinate = (value) => {
  if (!value || typeof value !== "object") return null;
  const lat = finite(value.lat);
  const lon = finite(value.lon);
  if (lat === null || lon === null || lat < -90 || lat > 90 || lon < -180 || lon > 180) return null;
  return {
    lat,
    lon,
    altM: finite(value.altM ?? value.altitudeM ?? value.alt),
    accuracyM: finite(value.accuracyM ?? value.hAccM ?? value.ephM),
  };
};

const localOffset = (value) => {
  if (!value || typeof value !== "object") return null;
  const northM = finite(value.northM ?? value.north);
  const eastM = finite(value.eastM ?? value.east);
  if (northM === null || eastM === null || Math.abs(northM) > 10_000 || Math.abs(eastM) > 10_000) return null;
  return { northM, eastM };
};

function timestampFor(value, receivedAt) {
  const timestampMs = finite(value?.timestampMs ?? value?.timeMs ?? value?.time_usec / 1_000);
  return timestampMs !== null && timestampMs > 0 ? timestampMs : receivedAt;
}

export function sanitizeLiveGps(gps, receivedAt = Date.now()) {
  if (!gps || typeof gps !== "object") return null;
  const position = coordinate(gps.position || gps);
  const home = coordinate(gps.home);
  const fixType = finite(gps.fixType ?? gps.fix_type);
  const satellites = finite(gps.satellites ?? gps.satellitesVisible ?? gps.satellites_visible);
  const hdop = finite(gps.hdop ?? gps.eph);
  const vdop = finite(gps.vdop ?? gps.epv);
  const groundSpeedMps = finite(gps.groundSpeedMps ?? gps.speedMps ?? gps.velMps);
  const courseDeg = finite(gps.courseDeg ?? gps.headingDeg ?? gps.cogDeg);
  const altitudeM = finite(gps.altitudeM ?? gps.alt);
  const verticalSpeedMps = finite(gps.verticalSpeedMps ?? gps.climbMps);
  if (!position && fixType === null && satellites === null && groundSpeedMps === null) return null;
  return {
    receivedAt,
    timestampMs: timestampFor(gps, receivedAt),
    fixType: fixType === null ? null : clamp(Math.round(fixType), 0, 6),
    satellites: satellites === null ? null : clamp(Math.round(satellites), 0, 255),
    hdop: hdop === null ? null : Math.max(0, hdop),
    vdop: vdop === null ? null : Math.max(0, vdop),
    position,
    home,
    groundSpeedMps: groundSpeedMps === null ? null : Math.max(0, groundSpeedMps),
    courseDeg: courseDeg === null ? null : ((courseDeg % 360) + 360) % 360,
    altitudeM,
    verticalSpeedMps,
    source: boundedText(gps.source, "GNSS"),
    constellation: boundedText(gps.constellation, ""),
  };
}

function normalizeRanges(lidar) {
  const source = Array.isArray(lidar.rangesM)
    ? lidar.rangesM
    : Array.isArray(lidar.distancesCm)
      ? lidar.distancesCm.map((value) => Number(value) / 100)
      : Array.isArray(lidar.distances)
        ? lidar.distances.map((value) => Number(value) / (Number(lidar.unitScale) || 100))
        : [];
  return source
    .slice(0, 360)
    .map((value) => finite(value))
    .map((value) => value === null || value <= 0 ? null : clamp(value, 0.02, 200));
}

export function sanitizeLiveLidar(lidar, receivedAt = Date.now()) {
  if (!lidar || typeof lidar !== "object") return null;
  const rangesM = normalizeRanges(lidar);
  const online = lidar.online !== false && (rangesM.length > 0 || lidar.online === true);
  if (!online && lidar.online !== false) return null;
  const minRangeM = finite(lidar.minRangeM ?? lidar.minDistanceM) ?? 0.05;
  const maxRangeM = finite(lidar.maxRangeM ?? lidar.maxDistanceM) ?? 30;
  const incrementDeg = finite(lidar.angleIncrementDeg ?? lidar.incrementDeg) ?? (rangesM.length ? 360 / rangesM.length : 5);
  const offsetDeg = finite(lidar.angleOffsetDeg ?? lidar.offsetDeg) ?? 0;
  const validRanges = rangesM.filter((value) => value !== null);
  return {
    receivedAt,
    timestampMs: timestampFor(lidar, receivedAt),
    online,
    sensorId: boundedText(lidar.sensorId ?? lidar.name, "LiDAR"),
    frame: boundedText(lidar.frame, "base_link"),
    scanRateHz: finite(lidar.scanRateHz ?? lidar.rateHz),
    minRangeM: Math.max(0.01, minRangeM),
    maxRangeM: Math.max(minRangeM, maxRangeM),
    incrementDeg: Math.max(0.1, incrementDeg),
    offsetDeg,
    rangesM,
    validCount: validRanges.length,
    obstacleCount: finite(lidar.obstacleCount) ?? validRanges.filter((value) => value < maxRangeM).length,
  };
}

function sanitizeMission(mission) {
  if (!mission || typeof mission !== "object") return null;
  const count = finite(mission.count ?? mission.total);
  const current = finite(mission.current ?? mission.seq);
  return {
    count: count === null ? null : clamp(Math.round(count), 0, 10_000),
    current: current === null ? null : clamp(Math.round(current), 0, 10_000),
    reached: finite(mission.reached),
    state: boundedText(mission.state, "unknown").toLowerCase(),
    paused: mission.paused === true,
    uploaded: mission.uploaded === true,
    name: boundedText(mission.name, ""),
  };
}

function normalizeMissionWaypoint(waypoint, index = 0) {
  if (!waypoint || typeof waypoint !== "object") return null;
  const lat = finite(waypoint.lat);
  const lon = finite(waypoint.lon);
  const local = localOffset(waypoint.local || waypoint.offsetM);
  const hasCoordinate = lat !== null && lon !== null && lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;
  if (!hasCoordinate && !local) return null;
  return {
    lat: hasCoordinate ? lat : null,
    lon: hasCoordinate ? lon : null,
    local,
    altM: clamp(finite(waypoint.altM) ?? 0, -1_000, 10_000),
    radiusM: clamp(finite(waypoint.radiusM) ?? 1.5, 0.1, 100),
    speedMps: clamp(finite(waypoint.speedMps) ?? 0, 0, 30),
    holdS: clamp(finite(waypoint.holdS) ?? 0, 0, 360),
    label: boundedText(waypoint.label, `WP ${index + 1}`),
  };
}

export function sanitizeLiveAutopilot(autopilot, receivedAt = Date.now()) {
  if (!autopilot || typeof autopilot !== "object") return null;
  const mode = boundedText(autopilot.mode, "unknown").toLowerCase();
  const modeCode = finite(autopilot.modeCode ?? autopilot.customMode);
  const mission = sanitizeMission(autopilot.mission);
  const home = coordinate(autopilot.home);
  const systemId = finite(autopilot.systemId ?? autopilot.sysid);
  const componentId = finite(autopilot.componentId ?? autopilot.compid);
  if (mode === "unknown" && modeCode === null && systemId === null && autopilot.heartbeat !== true) return null;
  return {
    receivedAt,
    timestampMs: timestampFor(autopilot, receivedAt),
    stack: boundedText(autopilot.stack, "ArduPilot"),
    vehicleType: boundedText(autopilot.vehicleType, "rover").toLowerCase(),
    systemId: systemId === null ? null : clamp(Math.round(systemId), 1, 255),
    componentId: componentId === null ? null : clamp(Math.round(componentId), 1, 255),
    armed: autopilot.armed === true,
    mode,
    modeCode: modeCode === null ? null : Math.round(modeCode),
    heartbeat: autopilot.heartbeat !== false,
    prearmReady: autopilot.prearmReady === true ? true : autopilot.prearmReady === false ? false : null,
    ekfHealthy: autopilot.ekfHealthy === true ? true : autopilot.ekfHealthy === false ? false : null,
    failsafe: autopilot.failsafe === true,
    failsafeReason: boundedText(autopilot.failsafeReason, ""),
    statusText: boundedText(autopilot.statusText, ""),
    mission,
    home,
  };
}

export function sanitizeLiveNavigation(navigation, receivedAt = Date.now()) {
  if (!navigation || typeof navigation !== "object") return null;
  const gps = sanitizeLiveGps(navigation.gps, receivedAt);
  const lidar = sanitizeLiveLidar(navigation.lidar, receivedAt);
  const autopilot = sanitizeLiveAutopilot(navigation.autopilot, receivedAt);
  const home = coordinate(navigation.home) || gps?.home || autopilot?.home || null;
  const geofence = navigation.geofence && typeof navigation.geofence === "object"
    ? {
        enabled: navigation.geofence.enabled === true,
        breached: navigation.geofence.breached === true,
        maxRadiusM: finite(navigation.geofence.maxRadiusM),
        polygon: Array.isArray(navigation.geofence.polygon)
          ? navigation.geofence.polygon.map(coordinate).filter(Boolean).slice(0, 100)
          : [],
      }
    : null;
  const obstacleBehavior = navigation.obstacleBehavior && typeof navigation.obstacleBehavior === "object"
    ? {
        enabled: navigation.obstacleBehavior.enabled !== false,
        stopDistanceM: finite(navigation.obstacleBehavior.stopDistanceM) ?? 0.45,
        slowDistanceM: finite(navigation.obstacleBehavior.slowDistanceM) ?? 1.2,
        maxSpeedMps: finite(navigation.obstacleBehavior.maxSpeedMps) ?? 0.5,
      }
    : null;
  if (!gps && !lidar && !autopilot && !home && !geofence && !obstacleBehavior) return null;
  return {
    receivedAt,
    gps,
    lidar,
    autopilot,
    home,
    geofence,
    obstacleBehavior,
  };
}

function mergeObject(previous, next) {
  if (!previous) return next;
  if (!next) return previous;
  return { ...previous, ...next };
}

export function mergeLiveNavigation(previous, next) {
  if (!next) return previous || null;
  return {
    ...previous,
    ...next,
    receivedAt: next.receivedAt ?? previous?.receivedAt ?? 0,
    gps: mergeObject(previous?.gps, next.gps),
    lidar: mergeObject(previous?.lidar, next.lidar),
    autopilot: mergeObject(previous?.autopilot, next.autopilot),
    home: next.home || previous?.home || null,
    geofence: mergeObject(previous?.geofence, next.geofence),
    obstacleBehavior: mergeObject(previous?.obstacleBehavior, next.obstacleBehavior),
  };
}

export function createLiveNavigationState() {
  return {
    missionDraft: [],
    missionName: "Domino patrol",
    plannerOrigin: null,
    plannerRangeM: 40,
    gpsTrack: [],
    obstacleBehavior: {
      enabled: true,
      stopDistanceM: 0.45,
      slowDistanceM: 1.2,
      maxSpeedMps: 0.5,
    },
    geofence: { enabled: false, maxRadiusM: 50, polygon: [] },
    selectedMode: "hold",
    pendingRequestId: "",
    pendingAction: "",
    lastCommandStatus: "Vehicle actions are locked until a navigation adapter reports in.",
    lastCommandAt: 0,
    lastAck: null,
  };
}

export function navigationFixLabel(fixType) {
  const labels = ["NO FIX", "NO FIX", "2D FIX", "3D FIX", "DGPS", "RTK FLOAT", "RTK FIXED"];
  const index = Number.isFinite(Number(fixType)) ? Math.round(Number(fixType)) : 0;
  return labels[Math.max(0, Math.min(labels.length - 1, index))];
}

function fresh(stream, now, maximumAgeMs = 2_000) {
  return Boolean(stream && Number.isFinite(stream.receivedAt) && now >= stream.receivedAt && now - stream.receivedAt <= maximumAgeMs);
}

function sectorMinimum(ranges, incrementDeg, offsetDeg, centerDeg, widthDeg, maxRangeM) {
  if (!Array.isArray(ranges) || !ranges.length) return null;
  const values = [];
  ranges.forEach((range, index) => {
    if (!Number.isFinite(range)) return;
    const angle = ((offsetDeg + index * incrementDeg + 540) % 360) - 180;
    const delta = Math.abs(((angle - centerDeg + 540) % 360) - 180);
    if (delta <= widthDeg / 2) values.push(Math.min(range, maxRangeM));
  });
  return values.length ? Math.min(...values) : null;
}

export function liveNavigationSnapshot(navigation, uiState = createLiveNavigationState(), now = Date.now()) {
  const gps = navigation?.gps || null;
  const lidar = navigation?.lidar || null;
  const autopilot = navigation?.autopilot || null;
  const gpsFresh = fresh(gps, now, 2_000);
  const lidarFresh = fresh(lidar, now, 2_000) && lidar.online !== false;
  const autopilotFresh = fresh(autopilot, now, 2_000);
  const hasFix = gpsFresh && gps.fixType !== null && gps.fixType >= 3 && Boolean(gps.position);
  const minRangeM = lidar?.maxRangeM ?? 30;
  const frontM = sectorMinimum(lidar?.rangesM, lidar?.incrementDeg, lidar?.offsetDeg, 0, 70, minRangeM);
  const rightM = sectorMinimum(lidar?.rangesM, lidar?.incrementDeg, lidar?.offsetDeg, 90, 70, minRangeM);
  const rearM = sectorMinimum(lidar?.rangesM, lidar?.incrementDeg, lidar?.offsetDeg, 180, 70, minRangeM);
  const leftM = sectorMinimum(lidar?.rangesM, lidar?.incrementDeg, lidar?.offsetDeg, -90, 70, minRangeM);
  const obstacle = uiState?.obstacleBehavior || navigation?.obstacleBehavior || {};
  const stopDistanceM = Number(obstacle.stopDistanceM) || 0.45;
  const slowDistanceM = Number(obstacle.slowDistanceM) || 1.2;
  const frontState = frontM !== null && frontM <= stopDistanceM
    ? "stop"
    : frontM !== null && frontM <= slowDistanceM
      ? "slow"
      : lidarFresh ? "clear" : "unknown";
  const positionReady = hasFix && autopilot?.ekfHealthy !== false;
  const autonomyReady = Boolean(
    autopilotFresh &&
    autopilot?.heartbeat !== false &&
    positionReady &&
    autopilot?.failsafe !== true &&
    autopilot?.prearmReady !== false,
  );
  const mode = ARDUPILOT_ROVER_MODES.find((candidate) => candidate.id === autopilot?.mode) || null;
  return {
    gps,
    lidar,
    autopilot,
    gpsFresh,
    lidarFresh,
    autopilotFresh,
    hasFix,
    positionReady,
    autonomyReady,
    mode,
    frontM,
    rightM,
    rearM,
    leftM,
    frontState,
    obstacleCount: lidar?.obstacleCount ?? 0,
    home: navigation?.home || gps?.home || autopilot?.home || null,
    geofence: navigation?.geofence || uiState?.geofence || null,
    mission: autopilot?.mission || { count: uiState?.missionDraft?.length || 0, current: null, state: "draft" },
    lastNavigationAgeMs: navigation?.receivedAt > 0 ? Math.max(0, now - navigation.receivedAt) : null,
  };
}

export function addNavigationWaypoint(state, waypoint) {
  if (!state || !waypoint || state.missionDraft.length >= 100) return false;
  const normalized = normalizeMissionWaypoint(waypoint, state.missionDraft.length);
  if (!normalized) return false;
  state.missionDraft.push(normalized);
  return true;
}

export function removeNavigationWaypoint(state, index) {
  if (!state || !Number.isInteger(Number(index)) || !state.missionDraft[Number(index)]) return false;
  state.missionDraft.splice(Number(index), 1);
  return true;
}

export function moveNavigationWaypoint(state, index, direction) {
  if (!state) return false;
  const from = Number(index);
  const to = from + (direction === "up" ? -1 : 1);
  if (!Number.isInteger(from) || !Number.isInteger(to) || from < 0 || to < 0 || from >= state.missionDraft.length || to >= state.missionDraft.length) return false;
  [state.missionDraft[from], state.missionDraft[to]] = [state.missionDraft[to], state.missionDraft[from]];
  return true;
}

export function navigationMissionJson(state) {
  return JSON.stringify({
    schemaVersion: 1,
    name: state?.missionName || "Domino patrol",
    mission: state?.missionDraft || [],
    plannerOrigin: state?.plannerOrigin || null,
    geofence: state?.geofence || null,
    obstacleBehavior: state?.obstacleBehavior || null,
  }, null, 2);
}

export function parseNavigationMissionJson(text) {
  const parsed = JSON.parse(text);
  if (!parsed || typeof parsed !== "object" || !Array.isArray(parsed.mission) || parsed.mission.length > 100) {
    throw new Error("Mission package must contain up to 100 waypoints.");
  }
  const mission = parsed.mission.map((waypoint, index) => normalizeMissionWaypoint(waypoint, index));
  if (mission.some((waypoint) => !waypoint)) throw new Error("Mission contains an invalid coordinate or waypoint limit.");
  return {
    name: boundedText(parsed.name, "Domino patrol"),
    mission,
    plannerOrigin: coordinate(parsed.plannerOrigin),
    geofence: parsed.geofence && typeof parsed.geofence === "object" ? parsed.geofence : null,
    obstacleBehavior: parsed.obstacleBehavior && typeof parsed.obstacleBehavior === "object" ? parsed.obstacleBehavior : null,
  };
}

export function missionWaypointHasCoordinate(waypoint) {
  const lat = waypoint?.lat;
  const lon = waypoint?.lon;
  return Boolean(
    waypoint &&
    lat !== null && lat !== undefined &&
    lon !== null && lon !== undefined &&
    Number.isFinite(Number(lat)) &&
    Number.isFinite(Number(lon)),
  );
}

export function coordinateToLocalOffset(position, origin) {
  const point = coordinate(position);
  const reference = coordinate(origin);
  if (!point || !reference) return null;
  const earthRadiusM = 6_371_000;
  const toRad = (value) => value * Math.PI / 180;
  return {
    northM: toRad(point.lat - reference.lat) * earthRadiusM,
    eastM: toRad(point.lon - reference.lon) * earthRadiusM * Math.cos(toRad(reference.lat)),
  };
}

export function localOffsetToCoordinate(offset, origin) {
  const local = localOffset(offset);
  const reference = coordinate(origin);
  if (!local || !reference) return null;
  const earthRadiusM = 6_371_000;
  const toDeg = (value) => value * 180 / Math.PI;
  const latitudeRadians = reference.lat * Math.PI / 180;
  const cosLatitude = Math.max(0.01, Math.cos(latitudeRadians));
  return {
    lat: reference.lat + toDeg(local.northM / earthRadiusM),
    lon: reference.lon + toDeg(local.eastM / (earthRadiusM * cosLatitude)),
    altM: reference.altM,
  };
}

function missionWaypointLocalPosition(waypoint, origin) {
  if (waypoint?.local) return localOffset(waypoint.local);
  return missionWaypointHasCoordinate(waypoint) && origin
    ? coordinateToLocalOffset(waypoint, origin)
    : null;
}

function coordinateDistanceM(first, second) {
  const a = coordinate(first);
  const b = coordinate(second);
  if (!a || !b) return null;
  const earthRadiusM = 6_371_000;
  const toRad = (value) => value * Math.PI / 180;
  const latitude = toRad(b.lat - a.lat);
  const longitude = toRad(b.lon - a.lon);
  const latitudeA = toRad(a.lat);
  const latitudeB = toRad(b.lat);
  const haversine = Math.sin(latitude / 2) ** 2
    + Math.cos(latitudeA) * Math.cos(latitudeB) * Math.sin(longitude / 2) ** 2;
  return 2 * earthRadiusM * Math.asin(Math.min(1, Math.sqrt(haversine)));
}

/**
 * Return display-only route metrics without claiming vehicle execution.
 * Local drafts remain measurable offline; GPS-backed routes use the same
 * north/east frame when an origin is available and fall back to haversine
 * distance when it is not.
 */
export function navigationMissionMetrics(mission = [], origin = null) {
  const waypoints = Array.isArray(mission) ? mission : [];
  const positions = waypoints.map((waypoint) => missionWaypointLocalPosition(waypoint, origin));
  const unresolvedCount = positions.filter((position) => !position).length;
  const localRouteKnown = waypoints.length > 0 && positions.every(Boolean);
  const coordinateRouteKnown = !localRouteKnown && waypoints.length > 0 && waypoints.every(missionWaypointHasCoordinate);
  let totalDistanceM = 0;
  if (localRouteKnown) {
    for (let index = 1; index < positions.length; index += 1) {
      totalDistanceM += Math.hypot(
        positions[index].northM - positions[index - 1].northM,
        positions[index].eastM - positions[index - 1].eastM,
      );
    }
  } else if (coordinateRouteKnown) {
    for (let index = 1; index < waypoints.length; index += 1) {
      totalDistanceM += coordinateDistanceM(waypoints[index - 1], waypoints[index]) || 0;
    }
  }
  const distanceKnown = localRouteKnown || coordinateRouteKnown;
  const estimatedSeconds = distanceKnown
    ? waypoints.reduce((seconds, waypoint, index) => {
        const speedMps = Number(waypoint?.speedMps) > 0 ? Number(waypoint.speedMps) : 0.5;
        const segmentDistanceM = index > 0 && localRouteKnown
          ? Math.hypot(positions[index].northM - positions[index - 1].northM, positions[index].eastM - positions[index - 1].eastM)
          : index > 0 && coordinateRouteKnown
            ? coordinateDistanceM(waypoints[index - 1], waypoint) || 0
            : 0;
        return seconds + segmentDistanceM / speedMps + (Number(waypoint?.holdS) || 0);
      }, 0)
    : null;
  return {
    waypointCount: waypoints.length,
    totalDistanceM: distanceKnown ? totalDistanceM : null,
    estimatedSeconds: distanceKnown ? estimatedSeconds : null,
    unresolvedCount,
    coordinateReady: waypoints.length > 0 && waypoints.every(missionWaypointHasCoordinate),
  };
}

export function navigationMissionGeofenceStatus(mission = [], origin = null, radiusM = 0, enabled = false) {
  const waypoints = Array.isArray(mission) ? mission : [];
  const radius = Number(radiusM);
  if (!enabled || !Number.isFinite(radius) || radius <= 0) {
    return { enabled: false, checked: false, outsideCount: 0, unresolvedCount: 0 };
  }
  const positions = waypoints.map((waypoint) => missionWaypointLocalPosition(waypoint, origin));
  const unresolvedCount = positions.filter((position) => !position).length;
  const outsideCount = positions.filter((position) => position && Math.hypot(position.northM, position.eastM) > radius).length;
  return {
    enabled: true,
    checked: Boolean(origin) && unresolvedCount === 0,
    outsideCount,
    unresolvedCount,
  };
}
