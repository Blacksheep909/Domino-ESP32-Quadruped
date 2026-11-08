import assert from "node:assert/strict";
import test from "node:test";

import { compareVersions, updatedNoticeView } from "./web/src/app-updated-notice.js";

test("compareVersions orders semantic versions numerically", () => {
  assert.equal(compareVersions("0.5.10", "0.5.9"), 1);
  assert.equal(compareVersions("0.5.3", "0.5.4"), -1);
  assert.equal(compareVersions("1.0.0", "1.0.0"), 0);
  assert.equal(compareVersions("junk", "1.0.0"), 0);
});

test("updated notice shows once after an upgrade and never on a fresh install", () => {
  const shown = updatedNoticeView("0.5.4", "0.5.3");
  assert.equal(shown.visible, true);
  assert.match(shown.text, /0\.5\.4/);
  assert.match(shown.detail, /0\.5\.3/);
  assert.match(shown.notesUrl, /releases\/tag\/v0\.5\.4$/);

  assert.equal(updatedNoticeView("0.5.4", null).visible, false);      // fresh install
  assert.equal(updatedNoticeView("0.5.4", "0.5.4").visible, false);   // same version again
  assert.equal(updatedNoticeView("0.5.3", "0.5.4").visible, false);   // downgrade
  assert.equal(updatedNoticeView("", "0.5.3").visible, false);        // browser / no updater
});
