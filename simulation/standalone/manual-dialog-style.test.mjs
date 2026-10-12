import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const html = await readFile(new URL("./web/index.html", import.meta.url), "utf8");
const styles = await readFile(new URL("./web/src/styles.css", import.meta.url), "utf8");

test("manual control close stays inside the toolbox header", () => {
  assert.match(html, /id="live-manual-close"[^>]*>×<\/button>/);
  assert.match(styles, /\.live-manual-shell > header \{[\s\S]*position: relative;[\s\S]*inset: auto;[\s\S]*background: transparent;/);
  assert.match(styles, /\.live-manual-shell #live-manual-close \{[\s\S]*width: 32px;[\s\S]*place-items: center;/);
});
