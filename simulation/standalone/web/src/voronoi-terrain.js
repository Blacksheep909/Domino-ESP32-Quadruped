const SEED_LAYOUT = [
  [-0.78, -0.70], [-0.28, -0.80], [0.30, -0.70], [0.78, -0.76],
  [-0.86, -0.04], [-0.38, -0.14], [0.13, -0.07], [0.68, -0.13],
  [-0.72, 0.66], [-0.18, 0.50], [0.38, 0.70], [0.84, 0.48],
];

const CELL_HEIGHTS = [0.030, 0.050, 0.022, 0.044, 0.058, 0.034, 0.070, 0.038, 0.026, 0.062, 0.046, 0.032];

function clipPolygon(polygon, seed, otherSeed) {
  const dx = otherSeed[0] - seed[0];
  const dz = otherSeed[1] - seed[1];
  const limit = (otherSeed[0] ** 2 + otherSeed[1] ** 2 - seed[0] ** 2 - seed[1] ** 2) / 2;
  const inside = (point) => dx * point[0] + dz * point[1] <= limit + 1e-6;
  const intersection = (start, end) => {
    const startValue = dx * start[0] + dz * start[1] - limit;
    const endValue = dx * end[0] + dz * end[1] - limit;
    const ratio = startValue / (startValue - endValue);
    return [
      start[0] + (end[0] - start[0]) * ratio,
      start[1] + (end[1] - start[1]) * ratio,
    ];
  };
  const clipped = [];
  polygon.forEach((point, index) => {
    const next = polygon[(index + 1) % polygon.length];
    const pointInside = inside(point);
    const nextInside = inside(next);
    if (pointInside && nextInside) clipped.push([...next]);
    else if (pointInside && !nextInside) clipped.push(intersection(point, next));
    else if (!pointInside && nextInside) {
      clipped.push(intersection(point, next));
      clipped.push([...next]);
    }
  });
  return clipped;
}

export function createVoronoiTerrain(spec) {
  const halfWidth = spec.size[0] / 2;
  const halfDepth = spec.size[2] / 2;
  const baseY = spec.position[1] + spec.size[1] / 2;
  const seeds = SEED_LAYOUT.map(([x, z]) => [x * halfWidth, z * halfDepth]);
  const bounds = [
    [-halfWidth, -halfDepth], [halfWidth, -halfDepth],
    [halfWidth, halfDepth], [-halfWidth, halfDepth],
  ];
  const vertices = [];
  const indices = [];
  const cellRanges = [];

  seeds.forEach((seed, cellIndex) => {
    let polygon = bounds.map((point) => [...point]);
    seeds.forEach((otherSeed, otherIndex) => {
      if (otherIndex !== cellIndex && polygon.length) {
        polygon = clipPolygon(polygon, seed, otherSeed);
      }
    });
    if (polygon.length < 3) return;

    const centreHeight = CELL_HEIGHTS[cellIndex % CELL_HEIGHTS.length];
    const centreIndex = vertices.length / 3;
    vertices.push(seed[0], baseY + centreHeight, seed[1]);
    const rimStart = vertices.length / 3;
    polygon.forEach(([x, z]) => vertices.push(x, baseY, z));
    const indexStart = indices.length;
    polygon.forEach((_, pointIndex) => {
      indices.push(
        centreIndex,
        rimStart + ((pointIndex + 1) % polygon.length),
        rimStart + pointIndex,
      );
    });
    cellRanges.push({ cellIndex, indexStart, indexCount: indices.length - indexStart });
  });

  return {
    vertices: new Float32Array(vertices),
    indices: new Uint32Array(indices),
    cellRanges,
    maxHeight: Math.max(...CELL_HEIGHTS),
  };
}
