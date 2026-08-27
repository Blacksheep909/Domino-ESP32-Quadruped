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

function sanitizeLiveCamera(camera, receivedAt) {
  if (!camera || typeof camera !== "object") return null;
  const yawDeg = finite(camera.yawDeg ?? camera.yaw);
  const pitchDeg = finite(camera.pitchDeg ?? camera.pitch);
  const fovDeg = finite(camera.fovDeg ?? camera.fieldOfViewDeg ?? camera.fov);
  const fps = finite(camera.fps ?? camera.frameRate);
  const connected = typeof camera.connected === "boolean" ? camera.connected : null;
  const error = typeof camera.error === "boolean" ? camera.error : null;
  if (yawDeg === null && pitchDeg === null && fovDeg === null && fps === null && connected === null && error === null) return null;
  return {
    receivedAt,
    timestampMs: timestampFor(camera, receivedAt),
    yawDeg: yawDeg === null ? null : clamp(yawDeg, -180, 180),
    pitchDeg: pitchDeg === null ? null : clamp(pitchDeg, -90, 90),
    fovDeg: fovDeg === null ? null : clamp(fovDeg, 1, 179),
    fps: fps === null ? null : clamp(fps, 0, 240),
    connected,
    error,
    source: boundedText(camera.source, "camera"),
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
  const camera = sanitizeLiveCamera(navigation.camera, receivedAt);
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
  if (!gps && !lidar && !autopilot && !camera && !home && !geofence && !obstacleBehavior) return null;
  return {
    receivedAt,
    gps,
    lidar,
    autopilot,
    camera,
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

function mergeSparseObject(previous, next) {
  if (!previous) return next;
  if (!next) return previous;
  return Object.fromEntries(Object.keys({ ...previous, ...next }).map((key) => [
    key,
    next[key] === null ? previous[key] ?? null : next[key],
  ]));
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
    camera: mergeSparseObject(previous?.camera, next.camera),
    home: next.home || previous?.home || null,
    geofence: mergeObject(previous?.geofence, next.geofence),
    obstacleBehavior: mergeObject(previous?.obstacleBehavior, next.obstacleBehavior),
  };
}

export function createLiveNavigationState() {
  return {
    missionDraft: [],
    missionName: "Domino patrol",
    loopCount: 1,
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
    camera: navigation?.camera || null,
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

export function duplicateNavigationWaypoint(state, index) {
  if (!state || !Array.isArray(state.missionDraft) || state.missionDraft.length >= 100) return false;
  const sourceIndex = Number(index);
  if (!Number.isInteger(sourceIndex) || sourceIndex < 0 || sourceIndex >= state.missionDraft.length) return false;
  const source = state.missionDraft[sourceIndex];
  const copy = {
    ...source,
    ...(source.local ? { local: { ...source.local } } : {}),
    label: `${source.label || `WP ${sourceIndex + 1}`} copy`.slice(0, 64),
  };
  state.missionDraft.splice(sourceIndex + 1, 0, copy);
  return true;
}

export function removeNavigationWaypoint(state, index) {
  if (!state || !Number.isInteger(Number(index)) || !state.missionDraft[Number(index)]) return false;
  state.missionDraft.splice(Number(index), 1);
  return true;
}

/**
 * Turn a recorded GPS trail into a bounded mission draft. The first and last
 * fixes are always retained; intermediate fixes are evenly resampled so a
 * long walk cannot exceed the mission waypoint limit.
 */
export function gpsTrackToMission(track = [], origin = null, maxWaypoints = 100) {
  const candidates = (Array.isArray(track) ? track : [])
    .map((point) => coordinate(point?.position || point))
    .filter(Boolean);
  if (!candidates.length) return [];
  const limit = Math.max(1, Math.min(100, Math.floor(Number(maxWaypoints) || 100)));
  const sampled = candidates.length <= limit
    ? candidates
    : limit === 1
      ? [candidates[0]]
      : Array.from({ length: limit }, (_, index) => candidates[Math.round(index * (candidates.length - 1) / (limit - 1))]);
  return sampled.map((point, index) => ({
    lat: point.lat,
    lon: point.lon,
    local: origin ? coordinateToLocalOffset(point, origin) : null,
    altM: point.altM ?? 0,
    radiusM: 1.5,
    speedMps: 0.5,
    holdS: 0,
    label: `TRACK ${String(index + 1).padStart(2, "0")}`,
  }));
}

export function moveNavigationWaypoint(state, index, direction) {
  if (!state) return false;
  const from = Number(index);
  const to = from + (direction === "up" ? -1 : 1);
  if (!Number.isInteger(from) || !Number.isInteger(to) || from < 0 || to < 0 || from >= state.missionDraft.length || to >= state.missionDraft.length) return false;
  [state.missionDraft[from], state.missionDraft[to]] = [state.missionDraft[to], state.missionDraft[from]];
  return true;
}

export function reverseNavigationWaypoints(state) {
  if (!state || !Array.isArray(state.missionDraft) || state.missionDraft.length < 2) return false;
  state.missionDraft.reverse();
  return true;
}

export function navigationMissionJson(state) {
  return JSON.stringify({
    schemaVersion: 1,
    name: state?.missionName || "Domino patrol",
    mission: state?.missionDraft || [],
    loopCount: Math.max(1, Math.min(5, Math.round(Number(state?.loopCount) || 1))),
    plannerOrigin: state?.plannerOrigin || null,
    plannerRangeM: state?.plannerRangeM || 40,
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
  const plannerRangeM = [40, 80, 160].includes(Number(parsed.plannerRangeM))
    ? Number(parsed.plannerRangeM)
    : 40;
  return {
    name: boundedText(parsed.name, "Domino patrol"),
    mission,
    loopCount: Math.max(1, Math.min(5, Math.round(Number(parsed.loopCount) || 1))),
    plannerOrigin: coordinate(parsed.plannerOrigin),
    plannerRangeM,
    geofence: parsed.geofence && typeof parsed.geofence === "object" ? parsed.geofence : null,
    obstacleBehavior: parsed.obstacleBehavior && typeof parsed.obstacleBehavior === "object" ? parsed.obstacleBehavior : null,
  };
}

function geoJsonCoordinate(value) {
  if (!Array.isArray(value) || value.length < 2) return null;
  const lon = finite(value[0]);
  const lat = finite(value[1]);
  if (lat === null || lon === null || lat < -90 || lat > 90 || lon < -180 || lon > 180) return null;
  return {
    lat,
    lon,
    altM: finite(value[2]),
  };
}

function geoJsonFeature(geometry, properties = {}) {
  if (!geometry || typeof geometry !== "object") return null;
  if (geometry.type === "Point") {
    const coordinateValue = geoJsonCoordinate(geometry.coordinates);
    return coordinateValue ? { coordinate: coordinateValue, properties } : null;
  }
  if (geometry.type === "LineString") {
    const coordinates = Array.isArray(geometry.coordinates)
      ? geometry.coordinates.map(geoJsonCoordinate).filter(Boolean)
      : [];
    return coordinates.length ? { line: coordinates, properties } : null;
  }
  if (geometry.type === "MultiLineString") {
    const lines = Array.isArray(geometry.coordinates)
      ? geometry.coordinates
        .map((line) => Array.isArray(line) ? line.map(geoJsonCoordinate).filter(Boolean) : [])
        .filter((line) => line.length)
      : [];
    return lines.length ? { line: lines.flat(), properties } : null;
  }
  return null;
}

function geoJsonRouteFeatures(parsed) {
  const candidates = [];
  const visit = (value) => {
    if (!value || typeof value !== "object") return;
    if (value.type === "FeatureCollection") {
      (Array.isArray(value.features) ? value.features : []).slice(0, 200).forEach(visit);
      return;
    }
    if (value.type === "Feature") {
      const feature = geoJsonFeature(value.geometry, value.properties || {});
      if (feature) candidates.push(feature);
      return;
    }
    const feature = geoJsonFeature(value, parsed?.properties || {});
    if (feature) candidates.push(feature);
  };
  visit(parsed);
  return candidates;
}

/**
 * Export a route in standard GeoJSON while retaining Domino-only waypoint
 * settings in feature properties. Local-only points require a valid origin so
 * they can be converted into geographic coordinates before export.
 */
export function navigationMissionGeoJson(state, origin = null) {
  const reference = coordinate(origin) || coordinate(state?.plannerOrigin);
  const features = [];
  const coordinates = [];
  (Array.isArray(state?.missionDraft) ? state.missionDraft : []).forEach((waypoint, index) => {
    const point = missionWaypointHasCoordinate(waypoint)
      ? coordinate(waypoint)
      : reference && waypoint?.local
        ? localOffsetToCoordinate(waypoint.local, reference)
        : null;
    if (!point) throw new Error(`Waypoint ${index + 1} needs a geographic coordinate or map origin.`);
    const coordinateArray = [point.lon, point.lat];
    if (Number.isFinite(point.altM)) coordinateArray.push(point.altM);
    coordinates.push(coordinateArray);
    features.push({
      type: "Feature",
      properties: {
        dominoWaypoint: true,
        sequence: index,
        label: boundedText(waypoint.label, `WP ${index + 1}`),
        radiusM: waypoint.radiusM,
        speedMps: waypoint.speedMps,
        holdS: waypoint.holdS,
        local: waypoint.local || null,
      },
      geometry: { type: "Point", coordinates: coordinateArray },
    });
  });
  if (coordinates.length >= 2) {
    features.unshift({
      type: "Feature",
      properties: { dominoRoute: true, name: boundedText(state?.missionName, "Domino patrol") },
      geometry: { type: "LineString", coordinates },
    });
  }
  return JSON.stringify({
    type: "FeatureCollection",
    properties: {
      dominoSchema: "route-v1",
      name: boundedText(state?.missionName, "Domino patrol"),
      loopCount: Math.max(1, Math.min(5, Math.round(Number(state?.loopCount) || 1))),
      plannerOrigin: reference,
      plannerRangeM: state?.plannerRangeM || 40,
    },
    features,
  }, null, 2) + "\n";
}

/**
 * Accept a standard GeoJSON Point/LineString/MultiLineString route. Point
 * features take precedence so waypoint labels and Domino settings survive a
 * round trip; a bare line remains a useful ordered route import.
 */
export function parseNavigationGeoJson(text) {
  const parsed = JSON.parse(text);
  if (!parsed || typeof parsed !== "object") throw new Error("GeoJSON route must be an object.");
  const features = geoJsonRouteFeatures(parsed);
  const pointFeatures = features
    .map((feature, index) => ({ ...feature, index }))
    .filter((feature) => feature.coordinate)
    .sort((first, second) => {
      const a = finite(first.properties?.sequence ?? first.properties?.order);
      const b = finite(second.properties?.sequence ?? second.properties?.order);
      if (a === null || b === null) return first.index - second.index;
      return a - b || first.index - second.index;
    });
  const lineFeature = features.find((feature) => feature.line?.length);
  const rootProperties = parsed.properties && typeof parsed.properties === "object" ? parsed.properties : {};
  const routeProperties = Object.keys(rootProperties).length
    ? rootProperties
    : lineFeature?.properties || pointFeatures[0]?.properties || {};
  const source = pointFeatures.length
    ? pointFeatures.map((feature) => ({ coordinate: feature.coordinate, properties: feature.properties }))
    : (lineFeature?.line || []).map((coordinateValue) => ({ coordinate: coordinateValue, properties: lineFeature.properties }));
  if (!source.length || source.length > 100) throw new Error("GeoJSON route must contain between 1 and 100 valid points.");
  const mission = source.map(({ coordinate: point, properties }, index) => normalizeMissionWaypoint({
    lat: point.lat,
    lon: point.lon,
    altM: point.altM ?? properties?.altM ?? 0,
    radiusM: properties?.radiusM,
    speedMps: properties?.speedMps,
    holdS: properties?.holdS,
    label: properties?.label || properties?.name,
  }, index));
  if (mission.some((waypoint) => !waypoint)) throw new Error("GeoJSON route contains an invalid waypoint.");
  return {
    name: boundedText(routeProperties.name, "Imported GeoJSON route"),
    mission,
    loopCount: Math.max(1, Math.min(5, Math.round(Number(routeProperties.loopCount) || 1))),
    plannerOrigin: coordinate(routeProperties.plannerOrigin),
    plannerRangeM: [40, 80, 160].includes(Number(routeProperties.plannerRangeM)) ? Number(routeProperties.plannerRangeM) : 40,
    geofence: null,
    obstacleBehavior: null,
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

export function moveNavigationWaypointToLocal(state, index, offset, origin = null) {
  if (!state || !Number.isInteger(Number(index)) || !state.missionDraft[Number(index)]) return false;
  const local = localOffset(offset);
  if (!local) return false;
  const waypointIndex = Number(index);
  const waypoint = state.missionDraft[waypointIndex];
  const coordinateValue = origin ? localOffsetToCoordinate(local, origin) : null;
  state.missionDraft[waypointIndex] = {
    ...waypoint,
    ...(coordinateValue
      ? { lat: coordinateValue.lat, lon: coordinateValue.lon }
      : { lat: null, lon: null }),
    local,
  };
  return true;
}

export function nudgeNavigationWaypoint(state, index, direction, stepM = 0.5, origin = null) {
  const waypointIndex = Number(index);
  const waypoint = state?.missionDraft?.[waypointIndex];
  const current = localOffset(waypoint?.local) || (origin ? coordinateToLocalOffset(waypoint, origin) : null);
  const step = finite(stepM);
  const delta = {
    ArrowUp: { northM: 1, eastM: 0 },
    ArrowDown: { northM: -1, eastM: 0 },
    ArrowLeft: { northM: 0, eastM: -1 },
    ArrowRight: { northM: 0, eastM: 1 },
  }[direction];
  if (!current || !delta || step === null || step <= 0 || step > 100) return false;
  return moveNavigationWaypointToLocal(state, waypointIndex, {
    northM: current.northM + delta.northM * step,
    eastM: current.eastM + delta.eastM * step,
  }, origin);
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
export function navigationMissionMetrics(mission = [], origin = null, loopCount = 1) {
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
  const loops = Math.max(1, Math.min(5, Math.round(Number(loopCount) || 1)));
  const firstPosition = positions[0];
  const lastPosition = positions.at(-1);
  const closingDistanceM = distanceKnown && positions.length > 1
    ? localRouteKnown
      ? Math.hypot(lastPosition.northM - firstPosition.northM, lastPosition.eastM - firstPosition.eastM)
      : coordinateDistanceM(waypoints.at(-1), waypoints[0]) || 0
    : 0;
  const firstSpeedMps = Number(waypoints[0]?.speedMps) > 0 ? Number(waypoints[0].speedMps) : 0.5;
  const oneLoopSeconds = distanceKnown
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
  const estimatedSeconds = oneLoopSeconds === null
    ? null
    : oneLoopSeconds * loops + (closingDistanceM / firstSpeedMps) * Math.max(0, loops - 1);
  return {
    waypointCount: waypoints.length,
    totalDistanceM: distanceKnown ? totalDistanceM * loops + closingDistanceM * Math.max(0, loops - 1) : null,
    estimatedSeconds: distanceKnown ? estimatedSeconds : null,
    loopCount: loops,
    unresolvedCount,
    coordinateReady: waypoints.length > 0 && waypoints.every(missionWaypointHasCoordinate),
  };
}

export function navigationMissionPreview(mission = [], origin = null, elapsedSeconds = 0, loopCount = 1) {
  const waypoints = Array.isArray(mission) ? mission : [];
  const positions = waypoints.map((waypoint) => missionWaypointLocalPosition(waypoint, origin));
  const loops = Math.max(1, Math.min(5, Math.round(Number(loopCount) || 1)));
  if (!waypoints.length || positions.some((position) => !position)) {
    return {
      ready: false,
      position: null,
      headingDeg: null,
      currentIndex: -1,
      progress: 0,
      elapsedSeconds: 0,
      totalSeconds: null,
      loopCount: loops,
      complete: false,
    };
  }

  const timeline = [];
  let totalSeconds = 0;
  let activeLoopIndex = 0;
  const headingBetween = (from, to) => {
    const northDelta = Number(to?.northM) - Number(from?.northM);
    const eastDelta = Number(to?.eastM) - Number(from?.eastM);
    if (!Number.isFinite(northDelta) || !Number.isFinite(eastDelta) || Math.hypot(northDelta, eastDelta) < 0.0001) return null;
    return ((Math.atan2(eastDelta, northDelta) * 180 / Math.PI) + 360) % 360;
  };
  const append = (type, index, start, end, duration) => {
    const boundedDuration = Math.max(0, Number(duration) || 0);
    if (boundedDuration <= 0) return;
    timeline.push({ type, index, loopIndex: activeLoopIndex, start, end, from: start, to: end, headingDeg: headingBetween(start, end), duration: boundedDuration, beginsAt: totalSeconds });
    totalSeconds += boundedDuration;
  };
  for (let loopIndex = 0; loopIndex < loops; loopIndex += 1) {
    activeLoopIndex = loopIndex;
    if (loopIndex > 0) {
      const previous = positions.at(-1);
      const current = positions[0];
      const distanceM = Math.hypot(current.northM - previous.northM, current.eastM - previous.eastM);
      const speedMps = Number(waypoints[0]?.speedMps) > 0 ? Number(waypoints[0].speedMps) : 0.5;
      append("return", 0, previous, current, distanceM / speedMps);
    }
    append("hold", 0, positions[0], positions[0], waypoints[0]?.holdS);
    for (let index = 1; index < positions.length; index += 1) {
      const previous = positions[index - 1];
      const current = positions[index];
      const distanceM = Math.hypot(current.northM - previous.northM, current.eastM - previous.eastM);
      const speedMps = Number(waypoints[index]?.speedMps) > 0 ? Number(waypoints[index].speedMps) : 0.5;
      append("segment", index, previous, current, distanceM / speedMps);
      append("hold", index, current, current, waypoints[index]?.holdS);
    }
  }

  // Holds have no geometric direction of their own. Use the next segment,
  // then the previous one, so the marker remains intuitive while dwelling.
  timeline.forEach((entry, index) => {
    if (entry.headingDeg !== null) return;
    for (let next = index + 1; next < timeline.length; next += 1) {
      if (timeline[next].headingDeg !== null) {
        entry.headingDeg = timeline[next].headingDeg;
        return;
      }
    }
    for (let previous = index - 1; previous >= 0; previous -= 1) {
      if (timeline[previous].headingDeg !== null) {
        entry.headingDeg = timeline[previous].headingDeg;
        return;
      }
    }
  });

  const requestedSeconds = Math.max(0, Number(elapsedSeconds) || 0);
  if (!timeline.length) {
    return {
      ready: true,
      position: positions[0],
      headingDeg: null,
      currentIndex: 0,
      progress: 1,
      elapsedSeconds: requestedSeconds,
      totalSeconds: 0,
      loopCount: loops,
      loopIndex: 0,
      complete: requestedSeconds > 0,
    };
  }
  if (requestedSeconds >= totalSeconds) {
    return {
      ready: true,
      position: positions.at(-1),
      headingDeg: timeline.at(-1)?.headingDeg ?? null,
      currentIndex: positions.length - 1,
      progress: 1,
      elapsedSeconds: totalSeconds,
      totalSeconds,
      loopCount: loops,
      loopIndex: loops - 1,
      complete: true,
    };
  }
  const active = timeline.find((entry) => requestedSeconds < entry.beginsAt + entry.duration) || timeline.at(-1);
  const progress = Math.max(0, Math.min(1, (requestedSeconds - active.beginsAt) / active.duration));
  return {
    ready: true,
    position: {
      northM: active.from.northM + (active.to.northM - active.from.northM) * progress,
      eastM: active.from.eastM + (active.to.eastM - active.from.eastM) * progress,
    },
    headingDeg: active.headingDeg,
    currentIndex: active.index,
    progress,
    elapsedSeconds: requestedSeconds,
    totalSeconds,
    loopCount: loops,
    loopIndex: active.loopIndex,
    complete: false,
  };
}

export function navigationMissionRecommendedRange(mission = [], origin = null, ranges = [40, 80, 160]) {
  const positions = (Array.isArray(mission) ? mission : []).map((waypoint) => missionWaypointLocalPosition(waypoint, origin));
  if (!positions.length || positions.some((position) => !position)) return null;
  const choices = ranges
    .map((range) => Number(range))
    .filter((range) => Number.isFinite(range) && range > 0)
    .sort((first, second) => first - second);
  if (!choices.length) return null;
  const maximumExtentM = Math.max(...positions.map((position) => Math.max(Math.abs(position.northM), Math.abs(position.eastM))));
  const requiredRangeM = Math.max(20, maximumExtentM * 2.4);
  return choices.find((range) => range >= requiredRangeM) || choices.at(-1);
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
