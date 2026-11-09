import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

// Studio's controls and cards are flat: no bevels, bottom lips, button drop
// shadows, press-down movement or thick accent edges. flat.css enforces that
// and must load after every other stylesheet.
test("flat.css loads last and flattens controls and cards", () => {
  const main = readFileSync(new URL("./web/src/main.js", import.meta.url), "utf8");
  const cssImports = [...main.matchAll(/^import "\.\/([\w-]+\.css)";/gm)].map((match) => match[1]);
  assert.equal(cssImports.at(-1), "flat.css");
  const flat = readFileSync(new URL("./web/src/flat.css", import.meta.url), "utf8");
  assert.match(flat, /:is\(button, \[role="tab"\], select, input, textarea, kbd, \.app-update-chip\)[^{]*\{\s*box-shadow: none !important;/);
  assert.match(flat, /:active \{\s*transform: none !important;\s*translate: none !important;/);
  assert.match(flat, /:focus-visible \{\s*outline: 2px solid var\(--ui-accent\);/);
  assert.match(flat, /border-left-width: 1px !important;/);
});
