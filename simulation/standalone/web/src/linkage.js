const EPSILON = 1e-7;

export function point2(values) {
  return { x: values[0], z: values[2] };
}

export function distance2(a, b) {
  return Math.hypot(b.x - a.x, b.z - a.z);
}

function rotate2(point, pivot, angle) {
  const x = point.x - pivot.x;
  const z = point.z - pivot.z;
  const cosine = Math.cos(angle);
  const sine = Math.sin(angle);
  return {
    x: pivot.x + cosine * x - sine * z,
    z: pivot.z + sine * x + cosine * z,
  };
}

function orientation(a, b, point) {
  return (b.x - a.x) * (point.z - a.z) - (b.z - a.z) * (point.x - a.x);
}

function circleIntersection(
  centerA,
  radiusA,
  centerB,
  radiusB,
  preferred,
  assemblyOrientation = 0,
) {
  const dx = centerB.x - centerA.x;
  const dz = centerB.z - centerA.z;
  const centerDistance = Math.hypot(dx, dz);
  const minimumDistance = Math.abs(radiusA - radiusB);
  const maximumDistance = radiusA + radiusB;

  if (
    centerDistance < EPSILON ||
    centerDistance < minimumDistance - EPSILON ||
    centerDistance > maximumDistance + EPSILON
  ) {
    return null;
  }

  const distance = Math.min(maximumDistance, Math.max(minimumDistance, centerDistance));
  const along = (radiusA ** 2 - radiusB ** 2 + distance ** 2) / (2 * distance);
  const height = Math.sqrt(Math.max(0, radiusA ** 2 - along ** 2));
  const nx = dx / centerDistance;
  const nz = dz / centerDistance;
  const base = { x: centerA.x + along * nx, z: centerA.z + along * nz };
  const candidates = [
    { x: base.x - height * nz, z: base.z + height * nx },
    { x: base.x + height * nz, z: base.z - height * nx },
  ];

  // A closed linkage has two mathematically valid assemblies. Selecting the
  // point nearest the authored pose eventually changes branches near low ride
  // height and flips the foot link through the chassis. Preserve the signed
  // assembly orientation instead.
  if (Math.abs(assemblyOrientation) > EPSILON && height > EPSILON) {
    const expectedSign = Math.sign(assemblyOrientation);
    const matching = candidates.find(
      (candidate) => Math.sign(orientation(centerA, centerB, candidate)) === expectedSign,
    );
    if (matching) return matching;
  }

  return distance2(candidates[0], preferred) <= distance2(candidates[1], preferred)
    ? candidates[0]
    : candidates[1];
}

function solveAtScale(points, upperDelta, lowerDelta, scale) {
  if (scale < EPSILON) {
    return {
      lowerDrive: point2(points.lower_drive),
      upperDrive: point2(points.upper_drive),
      lowerPassive: point2(points.lower_passive),
      // The exported closure STL is anchored from the diagonal-side hole.
      // Keep these solver names in mesh order; the URDF labels the same two
      // physical holes in the opposite parent/child order.
      lowerClosureDriver: point2(points.lower_closure_diagonal),
      upperClosureDriver: point2(points.upper_closure_driver),
      upperClosureCoupler: point2(points.upper_closure_coupler),
      lowerCoupler: point2(points.lower_coupler),
      lowerClosureDiagonal: point2(points.lower_closure_driver),
    };
  }

  const lowerDrive = point2(points.lower_drive);
  const upperDrive = point2(points.upper_drive);
  const lowerPassive = rotate2(point2(points.lower_passive), lowerDrive, lowerDelta * scale);
  const lowerClosureDriver = rotate2(
    point2(points.lower_closure_diagonal),
    lowerDrive,
    lowerDelta * scale,
  );
  const upperClosureDriver = rotate2(
    point2(points.upper_closure_driver),
    upperDrive,
    upperDelta * scale,
  );

  const upperCouplerRadius = distance2(
    point2(points.lower_passive),
    point2(points.upper_closure_coupler),
  );
  const upperClosureLength = distance2(
    point2(points.upper_closure_coupler),
    point2(points.upper_closure_driver),
  );
  const upperClosureCoupler = circleIntersection(
    lowerPassive,
    upperCouplerRadius,
    upperClosureDriver,
    upperClosureLength,
    point2(points.upper_closure_coupler),
    orientation(
      point2(points.lower_passive),
      point2(points.upper_closure_driver),
      point2(points.upper_closure_coupler),
    ),
  );
  if (!upperClosureCoupler) return null;

  const originalCouplerAngle = Math.atan2(
    points.upper_closure_coupler[2] - points.lower_passive[2],
    points.upper_closure_coupler[0] - points.lower_passive[0],
  );
  const currentCouplerAngle = Math.atan2(
    upperClosureCoupler.z - lowerPassive.z,
    upperClosureCoupler.x - lowerPassive.x,
  );
  const lowerCoupler = rotate2(
    point2(points.lower_coupler),
    point2(points.lower_passive),
    currentCouplerAngle - originalCouplerAngle,
  );
  lowerCoupler.x += lowerPassive.x - points.lower_passive[0];
  lowerCoupler.z += lowerPassive.z - points.lower_passive[2];

  const diagonalLength = distance2(
    point2(points.lower_coupler),
    point2(points.lower_closure_driver),
  );
  const lowerClosureLength = distance2(
    point2(points.lower_closure_diagonal),
    point2(points.lower_closure_driver),
  );
  const lowerClosureDiagonal = circleIntersection(
    lowerCoupler,
    diagonalLength,
    lowerClosureDriver,
    lowerClosureLength,
    point2(points.lower_closure_driver),
    orientation(
      point2(points.lower_coupler),
      point2(points.lower_closure_diagonal),
      point2(points.lower_closure_driver),
    ),
  );
  if (!lowerClosureDiagonal) return null;

  return {
    lowerDrive,
    upperDrive,
    lowerPassive,
    lowerClosureDriver,
    upperClosureDriver,
    upperClosureCoupler,
    lowerCoupler,
    lowerClosureDiagonal,
  };
}

export function solveLinkagePose(points, upperDelta, lowerDelta) {
  const target = solveAtScale(points, upperDelta, lowerDelta, 1);
  if (target) return { ...target, commandScale: 1, limited: false };

  let reachableScale = 0;
  let unreachableScale = 1;
  let solution = solveAtScale(points, upperDelta, lowerDelta, 0);

  for (let iteration = 0; iteration < 24; iteration += 1) {
    const candidateScale = (reachableScale + unreachableScale) / 2;
    const candidate = solveAtScale(points, upperDelta, lowerDelta, candidateScale);
    if (candidate) {
      reachableScale = candidateScale;
      solution = candidate;
    } else {
      unreachableScale = candidateScale;
    }
  }

  return {
    ...solution,
    commandScale: reachableScale,
    limited: true,
  };
}
