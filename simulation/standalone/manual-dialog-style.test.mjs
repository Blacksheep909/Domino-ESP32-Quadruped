import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const html = await readFile(new URL("./web/index.html", import.meta.url), "utf8");
const styles = await readFile(new URL("./web/src/styles.css", import.meta.url), "utf8");

test("manual control close stays inside the toolbox header", () => {
  // The close control is an accessible icon button (labelled, with an SVG glyph).
  assert.match(html, /id="live-manual-close"[^>]*aria-label="Close manual control"[^>]*><svg[\s\S]*?<\/svg><\/button>/);
  assert.match(styles, /\.live-manual-shell > header \{[\s\S]*position: relative;[\s\S]*inset: auto;[\s\S]*background: transparent;/);
  assert.match(styles, /\.live-manual-shell #live-manual-close \{[\s\S]*width: 32px;[\s\S]*place-items: center;/);
});
