// Shared by rendering and Rapier so every visible obstacle has the same
// collision shape. Dimensions are deliberately matched to Domino's nominal
// 34 mm gait clearance and 313 mm total leg reach.
export const terrainSpecs = [
  {
    id: "ramp",
    size: [1.2, 0.04, 0.9],
    position: [1.55, 0.065, -1.15],
    slope: 0.075,
    friction: 0.88,
    color: 0x728187,
    roughness: 0.86,
  },
  {
    id: "ramp-platform",
    size: [0.75, 0.13, 0.9],
    position: [2.52, 0.065, -1.15],
    friction: 0.92,
    color: 0x667177,
    roughness: 0.82,
  },
  ...[0.025, 0.05, 0.075, 0.10].map((height, index) => ({
    id: `step-${index + 1}`,
    size: [0.30, height, 0.85],
    position: [1.30 + index * 0.30, height / 2, 0.12],
    friction: 0.98,
    color: 0x69757b,
    roughness: 0.88,
  })),
  {
    id: "voronoi",
    kind: "voronoi",
    size: [1.6, 0.026, 1.2],
    position: [-1.65, 0.013, -1.85],
    friction: 1.10,
    color: 0x807b70,
    roughness: 1.0,
  },
  {
    id: "high-grip-pad",
    size: [1.1, 0.018, 1.0],
    position: [-0.20, 0.009, -1.85],
    friction: 1.25,
    color: 0x3f4a4c,
    roughness: 0.92,
  },
  {
    id: "low-grip-pad",
    size: [1.1, 0.014, 1.0],
    position: [1.10, 0.007, -1.85],
    friction: 0.35,
    color: 0x6e8388,
    roughness: 0.30,
  },
  {
    id: "stepping-block-wide",
    size: [0.52, 0.06, 0.52],
    position: [-1.95, 0.03, 0.45],
    friction: 0.94,
    color: 0x765843,
    roughness: 0.94,
  },
  {
    id: "stepping-block-low",
    size: [0.42, 0.09, 0.42],
    position: [-1.95, 0.045, 1.02],
    friction: 0.94,
    color: 0x806249,
    roughness: 0.94,
  },
];

export const logSpecs = [
  {
    id: "log-low",
    radius: 0.035,
    length: 1.20,
    position: [1.65, 0.035, 1.48],
    axis: "x",
    yaw: 0,
  },
  {
    id: "log-medium",
    radius: 0.045,
    length: 1.05,
    position: [1.65, 0.045, 1.75],
    axis: "x",
    yaw: 0,
  },
];

export const environmentBallSpecs = [
  {
    id: "ball-a",
    radius: 0.10,
    position: [0.55, 0.104, 1.12],
    mass: 0.18,
    color: 0xd47b42,
  },
  {
    id: "ball-b",
    radius: 0.075,
    position: [-0.45, 0.079, 1.12],
    mass: 0.12,
    color: 0x4d8f9e,
  },
];
