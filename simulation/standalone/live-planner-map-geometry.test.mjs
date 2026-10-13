import test from "node:test";
import assert from "node:assert/strict";

import {
  plannerCenterAfterPan,
  plannerCenterAfterZoom,
  plannerLocalAtFraction,
  plannerViewport,
} from "./web/src/live-planner-map-geometry.js";

test("wide planner uses the full map width without stretching metres", () => {
  const viewport = plannerViewport(1200, 600);
  assert.deepEqual(viewport, { aspect: 2, widthUnits: 200, minX: -50, maxX: 150 });
  const center = { eastM: 0, northM: 0 };
  assert.deepEqual(plannerLocalAtFraction(1, 0, center, 40, viewport.aspect), { eastM: 40, northM: 20 });
  assert.deepEqual(plannerLocalAtFraction(0.5, 0.5, center, 40, viewport.aspect), center);
});

test("pan and anchored zoom keep map locations under the cursor", () => {
  const center = { eastM: 0, northM: 0 };
  assert.deepEqual(plannerCenterAfterPan(center, 300, -150, 600, 40), { eastM: -20, northM: -10 });
  const pointer = plannerLocalAtFraction(0.75, 0.25, center, 80, 2);
  const zoomedCenter = plannerCenterAfterZoom(center, 80, 40, 0.75, 0.25, 2);
  assert.deepEqual(plannerLocalAtFraction(0.75, 0.25, zoomedCenter, 40, 2), pointer);
});
