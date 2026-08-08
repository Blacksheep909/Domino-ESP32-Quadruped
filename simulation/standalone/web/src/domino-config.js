export const assemblyOrigin = [0.099, 0.062375, 0.0];

const translatedFront = [0.335, 0, 0];

// P1-P6 below are cylindrical centres measured from both mating STL meshes.
// The active q2/q3 axes retain their coincident CAD/URDF centres.

// Centered, high-ride STAND output captured from the production firmware SIL.
// The source CAD is authored in this pose, so all rendered motion is measured
// against this fixed reference instead of cancelling each frame's servo delta.
export const standServoReference = [
  129.87, 171.59, 113.53, 105.44,
  78.30, 0.0, 0.0, 151.60,
  98.55, 115.42, 78.30, 136.35,
  0.0, 0.0, 119.88, 128.38,
];

export const legs = [
  {
    id: "dom_p_4_1",
    label: "FR",
    source: "dom_p_21_1",
    sourceTranslation: translatedFront,
    shoulderSign: -1,
    neutralLinkageOffsetDeg: { upper: 0, lower: 0 },
    channels: { shoulder: 3, upper: 4, lower: 15 },
    directions: { shoulder: 1, upper: 1, lower: 1 },
    meshes: {
      ground: "DOM_P__21__1",
      lower_driver: "DOM_P__18__1",
      coupler: "DOM_P__24__1",
      lower_diagonal: "DOM_P__23__1",
      lower_closure: "DOM_P__22__1",
      upper_driver: "DOM_P__19__1",
      upper_closure: "DOM_P__20__1",
    },
    points: {
      hip_origin: [0.2665, 0, 0.0105],
      foot_tip: [0.269045, -0.0381, -0.265493],
      upper_drive: [0.347, -0.028, 0.0105],
      lower_drive: [0.323, -0.028, -0.0105],
      lower_passive: [0.323, -0.036, -0.0105],
      lower_coupler: [0.302989, -0.0356, 0.024135],
      upper_closure_coupler: [0.322987, -0.0356, 0.0295],
      upper_closure_driver: [0.347, -0.0356, 0.0505],
      // P6/P4 are measured from the cylindrical holes in DOM_P__18/__23
      // and DOM_P__22. Do not replace them with the alternate URDF solution.
      lower_closure_diagonal: [0.210315, -0.0481, -0.123892],
      lower_closure_driver: [0.190304, -0.0481, -0.089257],
    },
  },
  {
    id: "dom_p_12_1",
    label: "FL",
    source: "dom_p_25_1",
    sourceTranslation: translatedFront,
    shoulderSign: 1,
    // The left CAD was exported 4.18 degrees away from the right-side
    // assembly pose. This neutral-only CAD calibration aligns the physical
    // STL foot and pin geometry without altering firmware servo commands.
    neutralLinkageOffsetDeg: { upper: 0, lower: 4.18 },
    channels: { shoulder: 0, upper: 1, lower: 2 },
    directions: { shoulder: 1, upper: -1, lower: -1 },
    meshes: {
      ground: "DOM_P__25__1",
      lower_driver: "DOM_P__16__1",
      coupler: "DOM_P__15__1",
      lower_diagonal: "DOM_P__14__1",
      lower_closure: "DOM_P__26__1",
      upper_driver: "DOM_P__17__1",
      upper_closure: "DOM_P__27__1",
    },
    points: {
      hip_origin: [0.2665, 0.12475, 0.0105],
      foot_tip: [0.261073, 0.16285, -0.25696],
      upper_drive: [0.347, 0.15275, 0.0105],
      lower_drive: [0.323, 0.15275, -0.0105],
      lower_passive: [0.323, 0.16075, -0.0105],
      lower_coupler: [0.302989, 0.16035, 0.024135],
      upper_closure_coupler: [0.322987, 0.16035, 0.0295],
      upper_closure_driver: [0.347, 0.16035, 0.0505],
      // Mirrored P6/P4 hole centres measured from DOM_P__16/__14/__26.
      lower_closure_diagonal: [0.202343, 0.17285, -0.115367],
      lower_closure_driver: [0.182332, 0.17285, -0.080732],
    },
  },
  {
    id: "dom_p_25_1",
    label: "BL",
    source: "dom_p_25_1",
    sourceTranslation: [0, 0, 0],
    shoulderSign: 1,
    neutralLinkageOffsetDeg: { upper: 0, lower: 4.18 },
    channels: { shoulder: 14, upper: 7, lower: 8 },
    directions: { shoulder: 1, upper: -1, lower: -1 },
    meshes: {
      ground: "DOM_P__25__1",
      lower_driver: "DOM_P__16__1",
      coupler: "DOM_P__15__1",
      lower_diagonal: "DOM_P__14__1",
      lower_closure: "DOM_P__26__1",
      upper_driver: "DOM_P__17__1",
      upper_closure: "DOM_P__27__1",
    },
    points: {
      hip_origin: [-0.0685, 0.12475, 0.0105],
      foot_tip: [-0.073927, 0.16285, -0.25696],
      upper_drive: [0.012, 0.15275, 0.0105],
      lower_drive: [-0.012, 0.15275, -0.0105],
      lower_passive: [-0.012, 0.16075, -0.0105],
      lower_coupler: [-0.032011, 0.16035, 0.024135],
      upper_closure_coupler: [-0.012013, 0.16035, 0.0295],
      upper_closure_driver: [0.012, 0.16035, 0.0505],
      lower_closure_diagonal: [-0.132657, 0.17285, -0.115367],
      lower_closure_driver: [-0.152668, 0.17285, -0.080732],
    },
  },
  {
    id: "dom_p_21_1",
    label: "BR",
    source: "dom_p_21_1",
    sourceTranslation: [0, 0, 0],
    shoulderSign: -1,
    neutralLinkageOffsetDeg: { upper: 0, lower: 0 },
    channels: { shoulder: 9, upper: 10, lower: 11 },
    directions: { shoulder: 1, upper: 1, lower: 1 },
    meshes: {
      ground: "DOM_P__21__1",
      lower_driver: "DOM_P__18__1",
      coupler: "DOM_P__24__1",
      lower_diagonal: "DOM_P__23__1",
      lower_closure: "DOM_P__22__1",
      upper_driver: "DOM_P__19__1",
      upper_closure: "DOM_P__20__1",
    },
    points: {
      hip_origin: [-0.0685, 0, 0.0105],
      foot_tip: [-0.065955, -0.0381, -0.265493],
      upper_drive: [0.012, -0.028, 0.0105],
      lower_drive: [-0.012, -0.028, -0.0105],
      lower_passive: [-0.012, -0.036, -0.0105],
      lower_coupler: [-0.032011, -0.0356, 0.024135],
      upper_closure_coupler: [-0.012013, -0.0356, 0.0295],
      upper_closure_driver: [0.012, -0.0356, 0.0505],
      lower_closure_diagonal: [-0.124685, -0.0481, -0.123892],
      lower_closure_driver: [-0.144696, -0.0481, -0.089257],
    },
  },
];

export const expectedMeshCount = 29;
