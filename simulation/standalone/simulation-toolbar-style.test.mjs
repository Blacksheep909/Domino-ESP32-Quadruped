import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const styles = await readFile(new URL("./web/src/styles.css", import.meta.url), "utf8");

test("desktop simulation toolbar uses a compact, visible light-theme rhythm", () => {
  assert.match(styles, /@media \(min-width: 1021px\)[\s\S]*#simulation-workspace \{ top: 138px; \}/);
  assert.match(styles, /#simulation-workspace \.gait-lab-button[\s\S]*border: 1px solid #b9cbbf/);
  assert.match(styles, /#height-meter-fill \{ background: #4e9b68; \}/);
  assert.match(styles, /\.joint-opacity-control input[\s\S]*accent-color: #4e9b68/);
  assert.match(styles, /#simulation-workspace \.walk-control[\s\S]*grid-template-columns: 38px 162px 58px/);
  assert.match(styles, /#simulation-workspace \.segmented-control button[\s\S]*flex: 0 0 54px/);
});
