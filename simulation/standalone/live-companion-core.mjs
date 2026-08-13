import { randomUUID } from "node:crypto";

import { validCalibrationCommand } from "./web/src/live-calibration-protocol.js";
import { validLiveConnectionCommand } from "./web/src/live-connection-protocol.js";
import { validLiveGaitCommand } from "./web/src/live-gait-protocol.js";
import { sanitizeLiveControllerTelemetry } from "./web/src/live-controller-state.js";
import { acceptLiveTelemetryPacket, createLiveTelemetryState } from "./web/src/live-telemetry-state.js";
import {
  LIVE_MANUAL_MAX_LEASE_MS,
  LIVE_MANUAL_TIMEOUT_MS,
  validLiveManualAuthorityCommand,
  validLiveManualControlFrame,
} from "./web/src/live-manual-control-protocol.js";
import {
  LIVE_SAFETY_WATCHDOG_MS,
  validLiveSafetyCommand,
  validLiveSafetyHeartbeat,
} from "./web/src/live-safety-protocol.js";

export const DOMINO_ROBOT_LINK_PROTOCOL = "domino-robot-link-v1";
export const COMPANION_ROBOT_FRESH_MS = 1_000;
export const COMPANION_ANNOUNCE_INTERVAL_MS = 1_000;

const boundedString = (value, max = 128) => typeof value === "string" && value.length > 0 && value.length <= max;
const finite = (value) => Number.isFinite(Number(value));
const robotStates = new Set(["unknown", "disarmed", "arming", "armed", "disarming", "estopped", "fault", "watchdog"]);

function controllerReady(controller, now) {
  const sanitized = sanitizeLiveControllerTelemetry(controller);
  const frameAgeMs = sanitized ? now - sanitized.frameTimestampMs : -1;
  return Boolean(
    sanitized?.source === "boxer-elrs" &&
    sanitized.failsafe === false &&
    frameAgeMs >= 0 &&
    frameAgeMs <= 500 &&
    sanitized.linkQualityPercent >= 50 &&
    sanitized.rssi1Dbm >= -105,
  );
}

function validRobotHello(message) {
  return Boolean(
    message?.type === "robot-hello" &&
    boundedString(message.robotId) &&
    boundedString(message.robotName) &&
    boundedString(message.firmwareVersion) &&
    message.capabilities && typeof message.capabilities === "object" &&
    robotStates.has(message.robotState)
  );
}

function validRobotTelemetry(message, receivedAt) {
  if (message?.type !== "robot-telemetry" || !robotStates.has(message.robotState)) return false;
  const validationState = createLiveTelemetryState();
  return acceptLiveTelemetryPacket(validationState, {
    type: "live-telemetry",
    sequence: 0,
    expected: message.expected,
    measured: message.measured,
    power: message.power,
  }, receivedAt);
}

function normalizeRobotTelemetryClock(message, receivedAt) {
  const robotTimeMs = Number(message?.robotTimeMs);
  if (!Number.isFinite(robotTimeMs) || robotTimeMs < 0) return message;
  const toHostTime = (value) => {
    const timestamp = Number(value);
    if (!Number.isFinite(timestamp) || timestamp >= 1_000_000_000_000) return value;
    return receivedAt - Math.max(0, robotTimeMs - timestamp);
  };
  const normalizePose = (pose) => pose && typeof pose === "object"
    ? { ...pose, timestampMs: toHostTime(pose.timestampMs) }
    : pose;
  return {
    ...message,
    expected: normalizePose(message.expected),
    measured: normalizePose(message.measured),
    controller: message.controller && typeof message.controller === "object"
      ? { ...message.controller, frameTimestampMs: toHostTime(message.controller.frameTimestampMs) }
      : message.controller,
  };
}

function validRobotAcknowledgement(message) {
  return Boolean(
    message?.type === "robot-ack" &&
    boundedString(message.kind) &&
    boundedString(message.action) &&
    boundedString(message.requestId) &&
    typeof message.accepted === "boolean" &&
    robotStates.has(message.robotState) &&
    (!message.reason || boundedString(message.reason, 256))
  );
}

function validRobotSafetyHeartbeatAcknowledgement(message) {
  return Boolean(
    message?.type === "robot-safety-heartbeat-ack" &&
    Number.isSafeInteger(Number(message.sequence)) &&
    Number(message.sequence) >= 0 &&
    robotStates.has(message.robotState) &&
    finite(message.watchdogRemainingMs) &&
    Number(message.watchdogRemainingMs) >= 0 &&
    Number(message.watchdogRemainingMs) <= LIVE_SAFETY_WATCHDOG_MS
  );
}

function neutralManualCommand(now, reason) {
  return {
    type: "companion-command",
    protocol: DOMINO_ROBOT_LINK_PROTOCOL,
    kind: "manual-control",
    action: "neutral",
    timestampMs: now,
    reason,
    deadman: false,
    neutral: true,
    mode: "stand",
    axes: { forward: 0, turn: 0, roll: 0, height: 0 },
    timeoutMs: LIVE_MANUAL_TIMEOUT_MS,
  };
}

export class LiveCompanionCore {
  constructor(options = {}) {
    this.adapterId = options.adapterId || "domino-companion";
    this.name = options.name || "Domino Companion";
    this.transport = options.transport || "wifi";
    this.endpoint = options.endpoint || "unconfigured";
    this.firmwareVersion = options.firmwareVersion || "unknown";
    this.robot = { id: options.robotId || "unknown", name: options.robotName || "Domino" };
    this.capabilities = {
      telemetry: false,
      calibration: false,
      gaitProfiles: false,
      persistentProfiles: false,
      manualControl: false,
    };
    this.robotConnected = false;
    this.lastRobotMessageAt = 0;
    this.lastTelemetryAt = 0;
    this.robotState = "unknown";
    this.controller = null;
    this.sessionId = "";
    this.telemetrySequence = 0;
    this.pending = new Map();
    this.lastSafetyHeartbeatAt = 0;
    this.lastSafetyHeartbeatSequence = -1;
    this.manualAuthority = null;
    this.lastManualFrameAt = 0;
    this.lastManualSequence = -1;
    this.benchMode = false;
  }

  announcement(now = Date.now()) {
    return {
      type: "live-adapter-announce",
      adapterId: this.adapterId,
      name: this.name,
      transport: this.transport,
      state: this.robotFresh(now) ? (this.sessionId ? "connected" : "available") : "error",
      timestampMs: now,
      endpoint: this.endpoint,
      signalPercent: null,
      robot: {
        id: this.robot.id,
        name: this.robot.name,
        firmwareVersion: this.firmwareVersion,
      },
      capabilities: { ...this.capabilities },
    };
  }

  robotFresh(now = Date.now()) {
    return this.robotConnected && now >= this.lastRobotMessageAt && now - this.lastRobotMessageAt <= COMPANION_ROBOT_FRESH_MS;
  }

  telemetryFresh(now = Date.now()) {
    return this.robotFresh(now) && this.lastTelemetryAt > 0 && now - this.lastTelemetryAt <= COMPANION_ROBOT_FRESH_MS;
  }

  driveLinkReady(now = Date.now()) {
    return this.telemetryFresh(now) && controllerReady(this.controller, now);
  }

  disconnectRobot(now = Date.now(), reason = "robot-link-closed") {
    const robot = [];
    if (this.manualAuthority) robot.push(neutralManualCommand(now, reason));
    this.robotConnected = false;
    this.lastRobotMessageAt = 0;
    this.lastTelemetryAt = 0;
    this.controller = null;
    this.robotState = "unknown";
    this.sessionId = "";
    this.pending.clear();
    this.lastSafetyHeartbeatAt = 0;
    this.lastSafetyHeartbeatSequence = -1;
    this.manualAuthority = null;
    this.benchMode = false;
    return { relay: [], robot };
  }

  handleRelay(message, now = Date.now()) {
    const relay = [];
    const robot = [];
    if (!message || typeof message !== "object") return { relay, robot };

    if (message.type === "live-connection-command" && validLiveConnectionCommand(message)) {
      if (message.action === "discover") return { relay, robot };
      if (message.adapterId !== this.adapterId) return { relay, robot };
      if (message.action === "connect") {
        const accepted = this.robotFresh(now) && robotStates.has(this.robotState) && this.robotState !== "unknown";
        if (accepted) {
          this.sessionId = randomUUID();
          this.lastSafetyHeartbeatAt = 0;
          this.lastSafetyHeartbeatSequence = -1;
          this.manualAuthority = null;
          this.lastManualSequence = -1;
        }
        relay.push({
          type: "live-connection-ack",
          action: "connect",
          requestId: message.requestId,
          accepted,
          adapterId: this.adapterId,
          ...(accepted ? { sessionId: this.sessionId, robotState: this.robotState } : {}),
          ...(!accepted ? { reason: "The physical robot link is stale or has not reported a safe state." } : {}),
        });
      } else if (message.action === "disconnect" && message.sessionId === this.sessionId) {
        if (this.manualAuthority) robot.push(neutralManualCommand(now, "engineering-session-disconnected"));
        const sessionId = this.sessionId;
        this.sessionId = "";
        this.lastSafetyHeartbeatAt = 0;
        this.lastSafetyHeartbeatSequence = -1;
        this.manualAuthority = null;
        this.benchMode = false;
        relay.push({ type: "live-connection-ack", action: "disconnect", requestId: message.requestId, accepted: true, adapterId: this.adapterId, sessionId });
      }
      return { relay, robot };
    }

    if (!this.sessionId || message.adapterId !== this.adapterId || message.sessionId !== this.sessionId) {
      return { relay, robot };
    }

    if (message.type === "live-safety-command" && validLiveSafetyCommand(message)) {
      let reason = "";
      if (message.action === "arm" && this.robotState !== "disarmed") reason = "Robot must report disarmed before arming.";
      if (message.action === "arm" && !this.telemetryFresh(now)) reason = "Fresh physical telemetry is required before arming.";
      if (message.action === "arm" && !this.driveLinkReady(now)) reason = "A fresh failsafe-clear Boxer/ELRS link is required before arming.";
      if (message.action === "reset-estop" && this.robotState !== "estopped") reason = "Robot is not in the E-stop state.";
      if (reason) {
        relay.push(this.safetyAck(message, false, reason));
      } else {
        this.remember(message, "safety", now);
        robot.push(this.robotCommand("safety", message, now));
      }
      return { relay, robot };
    }

    if (message.type === "live-safety-heartbeat" && validLiveSafetyHeartbeat(message)) {
      const heartbeatAgeMs = now - Number(message.timestampMs);
      if (
        this.robotState !== "armed" ||
        Number(message.sequence) <= this.lastSafetyHeartbeatSequence ||
        heartbeatAgeMs < 0 || heartbeatAgeMs > LIVE_SAFETY_WATCHDOG_MS
      ) return { relay, robot };
      this.lastSafetyHeartbeatAt = now;
      this.lastSafetyHeartbeatSequence = Number(message.sequence);
      robot.push({
        type: "companion-command", protocol: DOMINO_ROBOT_LINK_PROTOCOL, kind: "safety-heartbeat",
        sequence: message.sequence, timestampMs: now, watchdogMs: LIVE_SAFETY_WATCHDOG_MS,
      });
      return { relay, robot };
    }

    if (message.type === "live-manual-authority-command" && validLiveManualAuthorityCommand(message)) {
      if (message.action === "request-authority") {
        const accepted = this.robotState === "armed" && this.telemetryFresh(now) && this.driveLinkReady(now) && !this.manualAuthority;
        if (!accepted) {
          relay.push(this.manualAuthorityAck(message, false, "Armed state, fresh telemetry, and a healthy Boxer/ELRS link are required."));
        } else {
          const token = randomUUID();
          const leaseMs = Math.min(Number(message.requestedLeaseMs), LIVE_MANUAL_MAX_LEASE_MS);
          this.pending.set(message.requestId, { kind: "manual-authority", action: message.action, message, token, leaseMs, sentAt: now });
          robot.push({ ...this.robotCommand("manual-authority", message, now), authorityToken: token, leaseMs });
        }
      } else if (this.manualAuthority?.token === message.authorityToken) {
        robot.push(neutralManualCommand(now, "manual-authority-release"));
        this.remember(message, "manual-authority", now);
        robot.push(this.robotCommand("manual-authority", message, now));
      }
      return { relay, robot };
    }

    if (message.type === "live-manual-control-frame" && validLiveManualControlFrame(message)) {
      const authority = this.manualAuthority;
      const frameAgeMs = now - Number(message.timestampMs);
      const valid = authority && authority.token === message.authorityToken && now < authority.expiresAt &&
        this.robotState === "armed" && this.telemetryFresh(now) && this.driveLinkReady(now) &&
        Number(message.sequence) > this.lastManualSequence && frameAgeMs >= 0 && frameAgeMs <= LIVE_MANUAL_TIMEOUT_MS;
      if (valid) {
        this.lastManualSequence = Number(message.sequence);
        this.lastManualFrameAt = now;
        robot.push({
          type: "companion-command", protocol: DOMINO_ROBOT_LINK_PROTOCOL, kind: "manual-control",
          timestampMs: now, sequence: this.lastManualSequence, authorityToken: authority.token,
          deadman: message.deadman, neutral: message.neutral, mode: message.mode, axes: { ...message.axes },
          timeoutMs: LIVE_MANUAL_TIMEOUT_MS,
        });
      }
      return { relay, robot };
    }

    if (message.type === "live-calibration-command" && validCalibrationCommand(message)) {
      const allowed = this.robotState === "disarmed" && (message.action === "enter" || this.benchMode);
      if (!allowed) {
        relay.push(this.calibrationAck(message, false, "Calibration requires disarmed robot state and acknowledged bench mode."));
      } else {
        this.remember(message, "calibration", now);
        robot.push(this.robotCommand("calibration", message, now));
      }
      return { relay, robot };
    }

    if (message.type === "live-gait-command" && validLiveGaitCommand(message)) {
      if (message.action !== "request-profile" && this.robotState !== "disarmed") {
        relay.push(this.gaitAck(message, false, "Gait profile changes require the robot to be disarmed."));
      } else {
        this.remember(message, "gait", now);
        robot.push(this.robotCommand("gait", message, now));
      }
    }
    return { relay, robot };
  }

  handleRobot(message, now = Date.now()) {
    const relay = [];
    const robot = [];
    if (!message || message.protocol !== DOMINO_ROBOT_LINK_PROTOCOL) return { relay, robot };

    if (validRobotHello(message)) {
      this.robotConnected = true;
      this.lastRobotMessageAt = now;
      this.robot.id = message.robotId;
      this.robot.name = message.robotName;
      this.firmwareVersion = message.firmwareVersion;
      this.robotState = message.robotState;
      Object.keys(this.capabilities).forEach((capability) => {
        this.capabilities[capability] = message.capabilities?.[capability] === true;
      });
      return { relay, robot };
    }

    const normalizedTelemetry = message.type === "robot-telemetry"
      ? normalizeRobotTelemetryClock(message, now)
      : message;
    if (validRobotTelemetry(normalizedTelemetry, now)) {
      this.robotConnected = true;
      this.lastRobotMessageAt = now;
      this.robotState = normalizedTelemetry.robotState;
      this.controller = sanitizeLiveControllerTelemetry(normalizedTelemetry.controller);
      if (normalizedTelemetry.capabilities && typeof normalizedTelemetry.capabilities === "object") {
        Object.keys(this.capabilities).forEach((capability) => {
          this.capabilities[capability] = normalizedTelemetry.capabilities[capability] === true;
        });
      }
      this.lastTelemetryAt = now;
      if (this.sessionId) {
        relay.push({
          type: "live-telemetry", adapterId: this.adapterId, sessionId: this.sessionId,
          sequence: this.telemetrySequence++, expected: normalizedTelemetry.expected, measured: normalizedTelemetry.measured,
          power: normalizedTelemetry.power, controller: this.controller, diagnostics: normalizedTelemetry.diagnostics,
          gaitProfile: normalizedTelemetry.gaitProfile, capabilities: normalizedTelemetry.capabilities,
        });
      }
      return { relay, robot };
    }

    if (validRobotAcknowledgement(message)) {
      const pending = this.pending.get(message.requestId);
      if (!pending || pending.kind !== message.kind || pending.action !== message.action) return { relay, robot };
      this.robotConnected = true;
      this.lastRobotMessageAt = now;
      this.pending.delete(message.requestId);
      this.robotState = message.robotState;
      let accepted = message.accepted === true;
      let reason = message.reason || "";
      if (
        accepted && pending.kind === "manual-authority" && pending.action === "request-authority" &&
        (this.robotState !== "armed" || !this.telemetryFresh(now) || !this.driveLinkReady(now))
      ) {
        accepted = false;
        reason = "Manual authority acknowledgement arrived after a safety prerequisite expired.";
      }
      if (accepted && ["calibration", "gait"].includes(pending.kind) && this.robotState !== "disarmed") {
        accepted = false;
        reason = `${pending.kind === "calibration" ? "Calibration" : "Gait persistence"} acknowledgement is invalid while the robot is not disarmed.`;
      }
      if (message.kind === "safety") {
        if (accepted && message.action === "arm") {
          this.lastSafetyHeartbeatAt = now;
          this.lastSafetyHeartbeatSequence = -1;
        }
        if (accepted && message.action !== "arm") {
          this.manualAuthority = null;
          this.lastSafetyHeartbeatAt = 0;
          this.lastSafetyHeartbeatSequence = -1;
        }
        relay.push(this.safetyAck(pending.message, accepted, reason));
      } else if (message.kind === "manual-authority") {
        if (accepted && message.action === "request-authority") {
          this.manualAuthority = { token: pending.token, expiresAt: now + pending.leaseMs };
          this.lastManualFrameAt = now;
          this.lastManualSequence = -1;
        }
        if (message.action === "release-authority") this.manualAuthority = null;
        relay.push(this.manualAuthorityAck(pending.message, accepted, reason, pending));
      } else if (message.kind === "calibration") {
        if (accepted && message.action === "enter") this.benchMode = true;
        if (message.action === "exit") this.benchMode = false;
        relay.push(this.calibrationAck(pending.message, accepted, reason, message.profile, message));
      } else if (message.kind === "gait") {
        relay.push(this.gaitAck(pending.message, accepted, reason, message.profile));
      }
      return { relay, robot };
    }

    if (validRobotSafetyHeartbeatAcknowledgement(message) && this.sessionId) {
      this.robotConnected = true;
      this.lastRobotMessageAt = now;
      this.robotState = message.robotState;
      relay.push({
        type: "live-safety-heartbeat-ack", adapterId: this.adapterId, sessionId: this.sessionId,
        sequence: Number(message.sequence), robotState: this.robotState,
        watchdogRemainingMs: Math.max(0, Math.min(LIVE_SAFETY_WATCHDOG_MS, Number(message.watchdogRemainingMs) || 0)),
      });
    }
    return { relay, robot };
  }

  tick(now = Date.now()) {
    const relay = [];
    const robot = [];
    if (this.manualAuthority && (
      now >= this.manualAuthority.expiresAt || this.robotState !== "armed" || !this.telemetryFresh(now) ||
      !this.driveLinkReady(now) || now - this.lastManualFrameAt > LIVE_MANUAL_TIMEOUT_MS
    )) {
      robot.push(neutralManualCommand(now, "manual-control-timeout"));
      this.manualAuthority = null;
    }
    if (this.robotState === "armed" && this.lastSafetyHeartbeatAt > 0 && now - this.lastSafetyHeartbeatAt > LIVE_SAFETY_WATCHDOG_MS) {
      robot.push(neutralManualCommand(now, "safety-watchdog-timeout"));
      robot.push({ type: "companion-command", protocol: DOMINO_ROBOT_LINK_PROTOCOL, kind: "safety", action: "watchdog", timestampMs: now });
      this.robotState = "watchdog";
      this.manualAuthority = null;
    }
    for (const [requestId, pending] of this.pending) {
      if (now - pending.sentAt <= 2_000) continue;
      this.pending.delete(requestId);
      if (pending.kind === "safety") relay.push(this.safetyAck(pending.message, false, "Physical robot acknowledgement timed out."));
      if (pending.kind === "manual-authority") relay.push(this.manualAuthorityAck(pending.message, false, "Physical robot acknowledgement timed out."));
      if (pending.kind === "calibration") relay.push(this.calibrationAck(pending.message, false, "Physical robot acknowledgement timed out."));
      if (pending.kind === "gait") relay.push(this.gaitAck(pending.message, false, "Physical robot acknowledgement timed out."));
    }
    return { relay, robot };
  }

  remember(message, kind, now) {
    this.pending.set(message.requestId, { kind, action: message.action, message, sentAt: now });
  }

  robotCommand(kind, message, now) {
    return { type: "companion-command", protocol: DOMINO_ROBOT_LINK_PROTOCOL, kind, action: message.action, requestId: message.requestId, timestampMs: now, payload: message };
  }

  safetyAck(message, accepted, reason = "") {
    return { type: "live-safety-ack", action: message.action, requestId: message.requestId, accepted, adapterId: this.adapterId, sessionId: this.sessionId, robotState: this.robotState, ...(reason ? { reason } : {}) };
  }

  manualAuthorityAck(message, accepted, reason = "", pending = null) {
    return {
      type: "live-manual-authority-ack", action: message.action, requestId: message.requestId, accepted,
      adapterId: this.adapterId, sessionId: this.sessionId,
      ...(message.action === "request-authority" && accepted ? { authorityToken: pending.token, leaseMs: pending.leaseMs, robotState: this.robotState } : {}),
      ...(message.action === "release-authority" ? { authorityToken: message.authorityToken } : {}),
      ...(reason ? { reason } : {}),
    };
  }

  calibrationAck(message, accepted, reason = "", profile = undefined, physical = undefined) {
    return {
      type: "live-calibration-ack", action: message.action, requestId: message.requestId, accepted,
      adapterId: this.adapterId, sessionId: this.sessionId,
      ...(reason ? { reason } : {}), ...(profile ? { profile } : {}),
      ...(typeof physical?.benchMode === "boolean" ? { benchMode: physical.benchMode } : {}),
      ...(typeof physical?.supportsSafeJog === "boolean" ? { supportsSafeJog: physical.supportsSafeJog } : {}),
      ...(Number(physical?.maxSpeedDegPerSec) > 0 && Number(physical?.maxSpeedDegPerSec) <= 5
        ? { maxSpeedDegPerSec: Number(physical.maxSpeedDegPerSec) } : {}),
      ...(typeof physical?.persisted === "boolean" ? { persisted: physical.persisted } : {}),
    };
  }

  gaitAck(message, accepted, reason = "", profile = undefined) {
    return { type: "live-gait-ack", action: message.action, requestId: message.requestId, accepted, adapterId: this.adapterId, sessionId: this.sessionId, ...(reason ? { reason } : {}), ...(profile ? { profile } : {}) };
  }
}
