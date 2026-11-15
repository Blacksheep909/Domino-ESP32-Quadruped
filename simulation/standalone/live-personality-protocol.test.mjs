import assert from "node:assert/strict";
import test from "node:test";

import { LiveCompanionCore } from "./live-companion-core.mjs";
import {
  createLivePersonalityCommand,
  DEFAULT_PERSONALITY_SETTINGS,
  describePersonalitySettings,
  PERSONALITY_EMOTES,
  sanitizePersonalitySettings,
  validLivePersonalityAcknowledgement,
  validLivePersonalityCommand,
  validPersonalitySettings,
} from "./web/src/live-personality-protocol.js";

test("emote names match the firmware table", () => {
  assert.deepEqual(PERSONALITY_EMOTES.map((emote) => emote.id), ["curious", "happy", "bow", "look", "shy"]);
});

test("defaults are valid: on, sound on, rare idle sounds, Boxer SH on CH8, random emote", () => {
  assert.equal(validPersonalitySettings(DEFAULT_PERSONALITY_SETTINGS), true);
  assert.equal(DEFAULT_PERSONALITY_SETTINGS.emoteChannel, 8);
  assert.equal(DEFAULT_PERSONALITY_SETTINGS.fixedEmote, "random");
  assert.match(describePersonalitySettings(DEFAULT_PERSONALITY_SETTINGS), /^On: CH8 switch plays a random emote/);
});

test("turning everything off is a valid saved state and reads as silent", () => {
  const off = sanitizePersonalitySettings({ enabled: false });
  assert.equal(validPersonalitySettings(off), true);
  assert.match(describePersonalitySettings(off), /^Off: silent and still/);
});

test("sanitising clamps out-of-range values and unknown names", () => {
  const s = sanitizePersonalitySettings({ volumePercent: 250, emoteChannel: 40, idleChatter: "loud", fixedEmote: "dance" });
  assert.deepEqual(
    [s.volumePercent, s.emoteChannel, s.idleChatter, s.fixedEmote],
    [100, 16, "rare", "random"],
  );
});

test("commands are validated per action", () => {
  assert.ok(createLivePersonalityCommand("request-settings", "r1"));
  assert.ok(createLivePersonalityCommand("apply-settings", "r2", { settings: { enabled: false } }));
  assert.ok(createLivePersonalityCommand("preview-emote", "r3", { emote: "bow" }));
  assert.equal(createLivePersonalityCommand("preview-emote", "r4", { emote: "dance" }), null);
  assert.equal(validLivePersonalityCommand({ type: "live-personality-command", action: "apply-settings", requestId: "x", settings: { enabled: "yes" } }), false);
  assert.equal(validLivePersonalityCommand({ type: "live-personality-command", action: "reboot", requestId: "x" }), false);
});

test("acknowledgements must carry valid settings when they carry any", () => {
  const base = { type: "live-personality-ack", action: "request-settings", requestId: "r", accepted: true };
  assert.equal(validLivePersonalityAcknowledgement({ ...base, settings: DEFAULT_PERSONALITY_SETTINGS }), true);
  assert.equal(validLivePersonalityAcknowledgement({ ...base, settings: { enabled: 1 } }), false);
  assert.equal(validLivePersonalityAcknowledgement(base), true);
});

test("companion forwards emote commands only to firmware that reports the capability", () => {
  const core = new LiveCompanionCore({ adapterId: "a" });
  core.sessionId = "s";
  const message = { ...createLivePersonalityCommand("request-settings", "r1"), adapterId: "a", sessionId: "s" };
  let out = core.handleRelay(message, 1000);
  assert.equal(out.robot.length, 0);
  assert.equal(out.relay[0]?.type, "live-personality-ack");
  assert.equal(out.relay[0]?.accepted, false);
  core.capabilities.personality = true;
  out = core.handleRelay({ ...message, requestId: "r2" }, 1000);
  assert.equal(out.relay.length, 0);
  assert.equal(out.robot[0]?.kind, "personality");
  assert.equal(out.robot[0]?.action, "request-settings");
});
