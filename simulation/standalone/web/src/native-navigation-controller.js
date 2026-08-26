const clamp = (value, minimum, maximum) => Math.max(minimum, Math.min(maximum, Number(value) || 0));

const finite = (value) => {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
};

const localPoint = (value) => {
  const point = value?.local || value;
  const northM = finite(point?.northM);
  const eastM = finite(point?.eastM);
  return northM === null || eastM === null ? null : { northM, eastM };
};

function headingErrorDeg(targetBearingDeg, headingDeg) {
  const error = ((targetBearingDeg - headingDeg + 540) % 360) - 180;
  return error === -180 ? 180 : error;
}

function zeroCommand(state, reason, currentIndex = -1) {
  return {
    state,
    reason,
    currentIndex,
    target: null,
    distanceM: null,
    bearingDeg: null,
    headingErrorDeg: null,
    forward: 0,
    turn: 0,
    speedScale: 0,
  };
}

/**
 * Compute a bounded, app-owned navigation intent for a local route.
 *
 * This is deliberately a command planner, not a claim that a physical robot
 * has executed the command. A native adapter can consume the same normalized
 * intent later; the browser uses it now for honest preview and safety UI.
 */
export function nativeNavigationCommand({
  waypoints = [],
  position = { northM: 0, eastM: 0 },
  headingDeg,
  currentIndex = 0,
  loopCount = 1,
  completedLoops = 0,
  obstacle = {},
  geofence = {},
} = {}) {
  const route = Array.isArray(waypoints) ? waypoints.map(localPoint) : [];
  if (!route.length || route.some((point) => !point)) return zeroCommand("blocked", "Route has no usable local waypoints.");

  const current = localPoint(position);
  if (!current) return zeroCommand("blocked", "A local vehicle position is required.");

  const index = Math.max(0, Math.min(route.length - 1, Math.round(Number(currentIndex) || 0)));
  const target = route[index];
  const distanceM = Math.hypot(target.northM - current.northM, target.eastM - current.eastM);
  const radiusM = finite(geofence.maxRadiusM);
  if (geofence.enabled === true && radiusM !== null && radiusM > 0) {
    const targetDistanceFromOriginM = Math.hypot(target.northM, target.eastM);
    if (targetDistanceFromOriginM > radiusM) {
      return {
        ...zeroCommand("geofence-stop", "Target is outside the active home-radius geofence.", index),
        target,
        distanceM,
      };
    }
    const currentDistanceFromOriginM = Math.hypot(current.northM, current.eastM);
    if (currentDistanceFromOriginM > radiusM) {
      return {
        ...zeroCommand("geofence-stop", "Vehicle is outside the active home-radius geofence.", index),
        target,
        distanceM,
      };
    }
  }

  const arrivalRadiusM = Math.max(0.1, finite(waypoints[index]?.radiusM) ?? 1.5);
  if (distanceM <= arrivalRadiusM) {
    const holdS = Math.max(0, finite(waypoints[index]?.holdS) ?? 0);
    const finalLoop = Math.max(1, Math.min(5, Math.round(Number(loopCount) || 1))) <= Math.max(0, Math.round(Number(completedLoops) || 0)) + 1;
    const complete = index === route.length - 1 && holdS <= 0 && finalLoop;
    return {
      ...zeroCommand(complete ? "complete" : "arrived", complete ? "Final waypoint reached." : "Waypoint arrival radius reached.", index),
      target,
      distanceM,
      holdS,
    };
  }

  const heading = finite(headingDeg);
  if (heading === null) {
    return {
      ...zeroCommand("sensor-wait", "Waiting for a fresh vehicle heading.", index),
      target,
      distanceM,
    };
  }
  const bearingDeg = (Math.atan2(target.eastM - current.eastM, target.northM - current.northM) * 180) / Math.PI;
  const normalizedHeading = ((heading % 360) + 360) % 360;
  const errorDeg = headingErrorDeg(bearingDeg, normalizedHeading);
  const alignment = Math.max(0, Math.cos((errorDeg * Math.PI) / 180));
  const requestedSpeedMps = Math.max(0.05, finite(waypoints[index]?.speedMps) ?? 0.5);
  const speedScale = clamp(requestedSpeedMps / 1.5, 0.08, 1);
  let obstacleScale = 1;
  const obstacleEnabled = obstacle.enabled !== false;
  const frontM = finite(obstacle.frontM);
  const stopDistanceM = Math.max(0.05, finite(obstacle.stopDistanceM) ?? 0.45);
  const slowDistanceM = Math.max(stopDistanceM, finite(obstacle.slowDistanceM) ?? 1.2);
  if (obstacleEnabled && frontM !== null) {
    if (frontM <= stopDistanceM) {
      return {
        ...zeroCommand("obstacle-stop", "Front obstacle is inside the stop distance.", index),
        target,
        distanceM,
        bearingDeg,
        headingErrorDeg: errorDeg,
      };
    }
    if (frontM < slowDistanceM) obstacleScale = clamp((frontM - stopDistanceM) / Math.max(0.05, slowDistanceM - stopDistanceM), 0.15, 1);
  }

  if (obstacleEnabled && frontM === null) {
    return {
      ...zeroCommand("sensor-wait", "Waiting for a fresh front obstacle range.", index),
      target,
      distanceM,
      bearingDeg,
      headingErrorDeg: errorDeg,
    };
  }

  const turn = clamp(errorDeg / 55, -1, 1) * 0.8;
  const leftM = finite(obstacle.leftM);
  const rightM = finite(obstacle.rightM);
  const leftPressure = leftM === null ? 0 : clamp((slowDistanceM - leftM) / slowDistanceM, 0, 1);
  const rightPressure = rightM === null ? 0 : clamp((slowDistanceM - rightM) / slowDistanceM, 0, 1);
  const avoidanceTurn = clamp(leftPressure - rightPressure, -0.5, 0.5) * 0.6;
  const forward = clamp((distanceM / 2) * speedScale * alignment * obstacleScale, 0, 0.8);
  return {
    state: "navigating",
    reason: "Native route command is ready for a supported adapter.",
    currentIndex: index,
    target,
    distanceM,
    bearingDeg,
    headingErrorDeg: errorDeg,
    forward,
    turn: clamp(turn + avoidanceTurn, -1, 1),
    speedScale: obstacleScale,
  };
}

export { headingErrorDeg };
