import dgram from "node:dgram";
import path from "node:path";
import process from "node:process";
import { fileURLToPath } from "node:url";
import { WebSocket } from "ws";

import {
  DOMINO_ROBOT_LINK_PROTOCOL,
  LiveCompanionCore,
} from "./live-companion-core.mjs";

export const MAVLINK_MESSAGE_IDS = Object.freeze({
  HEARTBEAT: 0,
  SYS_STATUS: 1,
  SET_MODE: 11,
  PARAM_VALUE: 22,
  PARAM_SET: 23,
  GPS_RAW_INT: 24,
  GLOBAL_POSITION_INT: 33,
  MISSION_SET_CURRENT: 41,
  MISSION_CURRENT: 42,
  MISSION_REQUEST_LIST: 43,
  MISSION_COUNT: 44,
  MISSION_CLEAR_ALL: 45,
  MISSION_ITEM_REACHED: 46,
  MISSION_ACK: 47,
  MISSION_REQUEST: 40,
  MISSION_REQUEST_INT: 51,
  NAV_CONTROLLER_OUTPUT: 62,
  MISSION_ITEM_INT: 73,
  VFR_HUD: 74,
  COMMAND_LONG: 76,
  COMMAND_ACK: 77,
  SET_POSITION_TARGET_GLOBAL_INT: 86,
  EKF_STATUS_REPORT: 193,
  HOME_POSITION: 242,
  STATUSTEXT: 253,
  OBSTACLE_DISTANCE: 330,
});

const MAVLINK_CRC_EXTRAS = Object.freeze({
  0: 50,
  1: 124,
  11: 89,
  22: 220,
  23: 168,
  24: 24,
  33: 104,
  40: 230,
  41: 28,
  42: 28,
  43: 132,
  44: 221,
  45: 232,
  46: 11,
  47: 153,
  51: 196,
  62: 183,
  73: 38,
  74: 20,
  76: 152,
  77: 143,
  86: 5,
  193: 71,
  242: 104,
  253: 83,
  330: 23,
});

const MAVLINK_MODE_NAMES = Object.freeze({
  0: "manual",
  1: "acro",
  3: "steering",
  4: "hold",
  5: "loiter",
  6: "follow",
  7: "simple",
  8: "dock",
  9: "circle",
  10: "auto",
  11: "rtl",
  12: "smart-rtl",
  15: "guided",
});

const MAV_COMMANDS = Object.freeze({
  COMPONENT_ARM_DISARM: 400,
  DO_SET_HOME: 179,
  MISSION_START: 300,
});

const MAV_RESULT_ACCEPTED = 0;
const MAV_FRAME_GLOBAL_INT = 5;
const MAV_FRAME_GLOBAL = 0;
const MAV_TYPE_GCS = 6;
const MAV_AUTOPILOT_INVALID = 8;
const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;
const MAV_MODE_FLAG_SAFETY_ARMED = 128;
const MAV_PARAM_TYPE_REAL32 = 9;
const MAV_MISSION_TYPE_MISSION = 0;
const MAV_MISSION_ACCEPTED = 0;
const MAV_MISSION_TYPE_MISSION_ITEM = 16;
const MAV_SYS_STATUS_PREARM_CHECK = 1 << 28;

function crcAccumulate(byte, crc) {
  let value = (byte ^ (crc & 0xff)) & 0xff;
  value ^= (value << 4) & 0xff;
  return (
    (crc >> 8) ^
    (value << 8) ^
    (value << 3) ^
    (value >> 4)
  ) & 0xffff;
}

export function mavlinkCrc(data, extra) {
  let crc = 0xffff;
  for (const byte of data) crc = crcAccumulate(byte, crc);
  if (Number.isFinite(extra)) crc = crcAccumulate(Number(extra), crc);
  return crc;
}

export function encodeMavlinkFrame(messageId, payload = Buffer.alloc(0), options = {}) {
  const body = Buffer.from(payload);
  const version = options.version === 1 ? 1 : 2;
  const sequence = Number(options.sequence || 0) & 0xff;
  const systemId = Number(options.systemId || 255) & 0xff;
  const componentId = Number(options.componentId || 190) & 0xff;
  const extra = MAVLINK_CRC_EXTRAS[Number(messageId)];
  if (!Number.isFinite(extra)) throw new Error(`No MAVLink CRC extra is registered for message ${messageId}.`);
  if (version === 1 && Number(messageId) > 255) throw new Error("MAVLink 1 cannot encode a 24-bit message id.");
  if (body.length > 255) throw new Error("MAVLink payload exceeds the 255-byte limit.");
  if (version === 1) {
    const header = Buffer.from([body.length, sequence, systemId, componentId, Number(messageId) & 0xff]);
    const crc = mavlinkCrc(Buffer.concat([header, body]), extra);
    return Buffer.concat([Buffer.from([0xfe]), header, body, Buffer.from([crc & 0xff, crc >> 8])]);
  }
  const id = Number(messageId);
  const header = Buffer.from([
    body.length,
    0,
    0,
    sequence,
    systemId,
    componentId,
    id & 0xff,
    (id >> 8) & 0xff,
    (id >> 16) & 0xff,
  ]);
  const crc = mavlinkCrc(Buffer.concat([header, body]), extra);
  return Buffer.concat([Buffer.from([0xfd]), header, body, Buffer.from([crc & 0xff, crc >> 8])]);
}

function decodeCandidate(frame) {
  const magic = frame[0];
  const version = magic === 0xfd ? 2 : magic === 0xfe ? 1 : 0;
  if (!version) return null;
  const payloadLength = frame[1];
  const headerLength = version === 2 ? 9 : 5;
  const payloadStart = 1 + headerLength;
  const payload = frame.subarray(payloadStart, payloadStart + payloadLength);
  const messageId = version === 2
    ? frame[7] | (frame[8] << 8) | (frame[9] << 16)
    : frame[5];
  const crcStart = payloadStart + payloadLength;
  const receivedCrc = frame[crcStart] | (frame[crcStart + 1] << 8);
  const extra = MAVLINK_CRC_EXTRAS[messageId];
  if (!Number.isFinite(extra)) return null;
  const computedCrc = mavlinkCrc(frame.subarray(1, crcStart), extra);
  if (computedCrc !== receivedCrc) return null;
  return {
    version,
    sequence: version === 2 ? frame[4] : frame[2],
    systemId: version === 2 ? frame[5] : frame[3],
    componentId: version === 2 ? frame[6] : frame[4],
    messageId,
    payload,
  };
}

export class MavlinkParser {
  constructor() {
    this.buffer = Buffer.alloc(0);
  }

  push(chunk) {
    this.buffer = Buffer.concat([this.buffer, Buffer.from(chunk)]);
    const messages = [];
    while (this.buffer.length) {
      const start = this.buffer.findIndex((byte) => byte === 0xfd || byte === 0xfe);
      if (start < 0) {
        this.buffer = Buffer.alloc(0);
        break;
      }
      if (start > 0) this.buffer = this.buffer.subarray(start);
      if (this.buffer.length < 2) break;
      const version = this.buffer[0] === 0xfd ? 2 : 1;
      const payloadLength = this.buffer[1];
      const signatureLength = version === 2 && (this.buffer[2] & 0x01) ? 13 : 0;
      const frameLength = (version === 2 ? 12 : 8) + payloadLength + signatureLength;
      if (this.buffer.length < frameLength) break;
      const frame = this.buffer.subarray(0, frameLength - signatureLength);
      this.buffer = this.buffer.subarray(frameLength);
      const decoded = decodeCandidate(frame);
      if (decoded) messages.push(decoded);
    }
    return messages;
  }
}

function optionsFromArgs(args) {
  const options = {};
  for (let index = 0; index < args.length; index += 1) {
    if (!args[index].startsWith("--")) continue;
    const key = args[index].slice(2);
    const value = args[index + 1];
    if (!value || value.startsWith("--")) options[key] = true;
    else { options[key] = value; index += 1; }
  }
  return options;
}

function readUInt8(payload, offset, fallback = 0) {
  return offset < payload.length ? payload.readUInt8(offset) : fallback;
}

function readUInt16(payload, offset, fallback = 0) {
  return offset + 2 <= payload.length ? payload.readUInt16LE(offset) : fallback;
}

function readInt16(payload, offset, fallback = 0) {
  return offset + 2 <= payload.length ? payload.readInt16LE(offset) : fallback;
}

function readUInt32(payload, offset, fallback = 0) {
  return offset + 4 <= payload.length ? payload.readUInt32LE(offset) : fallback;
}

function readInt32(payload, offset, fallback = 0) {
  return offset + 4 <= payload.length ? payload.readInt32LE(offset) : fallback;
}

function readFloat(payload, offset, fallback = 0) {
  return offset + 4 <= payload.length ? payload.readFloatLE(offset) : fallback;
}

function readUInt64(payload, offset, fallback = 0) {
  if (offset + 8 > payload.length) return fallback;
  const value = payload.readBigUInt64LE(offset);
  return Number(value <= BigInt(Number.MAX_SAFE_INTEGER) ? value : BigInt(fallback));
}

function readText(payload, offset, length) {
  return payload.subarray(offset, Math.min(payload.length, offset + length)).toString("utf8").replace(/\0+.*$/s, "").trim();
}

function writeString(buffer, offset, length, value) {
  Buffer.from(String(value || "").slice(0, length), "utf8").copy(buffer, offset, 0, length);
}

function roverModeName(code) {
  return MAVLINK_MODE_NAMES[Number(code)] || `mode-${Number(code)}`;
}

function modeCode(mode) {
  const entry = Object.entries(MAVLINK_MODE_NAMES).find(([, name]) => name === String(mode || "").toLowerCase());
  return entry ? Number(entry[0]) : null;
}

function nowMs() {
  return Date.now();
}

export class ArduPilotMavlinkBridge {
  constructor(options = {}) {
    this.options = options;
    this.adapterId = options.adapterId || "ardupilot-mavlink-1";
    this.name = options.name || "ArduPilot MAVLink Companion";
    this.transport = options.transport || "wifi";
    this.endpoint = options.endpoint || `${options.host || "127.0.0.1"}:${options.port || 14550}`;
    this.host = options.host || "127.0.0.1";
    this.port = Number(options.port || 14550);
    this.localPort = Number(options.localPort || 0);
    this.relayUrl = options.relayUrl || "ws://127.0.0.1:8770/control";
    this.logger = options.logger || console;
    this.core = options.core || new LiveCompanionCore({
      adapterId: this.adapterId,
      name: this.name,
      transport: this.transport,
      endpoint: this.endpoint,
      robotId: "ardupilot-unbound",
      robotName: "ArduPilot Rover",
    });
    this.socket = dgram.createSocket("udp4");
    this.parser = new MavlinkParser();
    this.relay = null;
    this.sequence = 0;
    this.systemId = 1;
    this.componentId = 1;
    this.lastHeartbeatAt = 0;
    this.lastTelemetryAt = 0;
    this.lastPublishAt = 0;
    this.currentMode = "unknown";
    this.currentModeCode = null;
    this.currentArmed = false;
    this.currentAutopilot = null;
    this.gps = null;
    this.lidar = null;
    this.home = null;
    this.power = null;
    this.ekfHealthy = null;
    this.prearmReady = null;
    this.failsafe = false;
    this.statusText = "";
    this.mission = { count: 0, current: null, reached: null, state: "unknown", paused: false, uploaded: false };
    this.obstacleBehavior = { enabled: true, stopDistanceM: 0.45, slowDistanceM: 1.2, maxSpeedMps: 0.5 };
    this.geofence = { enabled: false, breached: false, maxRadiusM: null, polygon: [] };
    this.pendingMavlink = new Map();
    this.missionUpload = null;
    this.running = false;
    this.socket.on("message", (packet) => this.handleUdpPacket(packet));
    this.socket.on("error", (error) => this.logger.error?.(`ArduPilot MAVLink UDP: ${error.message}`));
  }

  announcement(now = nowMs()) {
    const base = this.core.announcement(now);
    return {
      ...base,
      capabilities: {
        ...base.capabilities,
        telemetry: true,
        navigation: true,
        gps: true,
        lidar: true,
        ardupilot: true,
        autonomy: true,
      },
    };
  }

  start() {
    if (this.running) return;
    this.running = true;
    this.socket.bind(this.localPort, "0.0.0.0", () => {
      this.logger.log?.(`ArduPilot MAVLink companion listening on UDP ${this.socket.address().port}, target ${this.endpoint}`);
    });
    this.connectRelay();
    this.heartbeatTimer = setInterval(() => this.sendGroundStationHeartbeat(), 1_000);
    this.announceTimer = setInterval(() => this.writeRelay(this.announcement()), 1_000);
    this.tickTimer = setInterval(() => this.dispatch(this.core.tick()), 25);
  }

  stop() {
    this.running = false;
    clearInterval(this.heartbeatTimer);
    clearInterval(this.announceTimer);
    clearInterval(this.tickTimer);
    this.relay?.close();
    this.relay = null;
    try { this.socket.close(); } catch { /* Already closed. */ }
  }

  connectRelay() {
    if (!this.running) return;
    this.relay = new WebSocket(this.relayUrl);
    this.relay.on("open", () => {
      this.writeRelay(this.announcement());
      this.logger.log?.(`ArduPilot companion connected to ${this.relayUrl}`);
    });
    this.relay.on("message", (payload) => {
      try {
        const message = JSON.parse(payload.toString());
        this.dispatch(this.core.handleRelay(message, nowMs()));
      } catch {
        // Ignore malformed local relay packets.
      }
    });
    this.relay.on("close", () => {
      this.relay = null;
      if (this.running) setTimeout(() => this.connectRelay(), 800);
    });
    this.relay.on("error", (error) => this.logger.error?.(`ArduPilot relay: ${error.message}`));
  }

  writeRelay(message) {
    if (this.relay?.readyState === WebSocket.OPEN) this.relay.send(JSON.stringify(message));
  }

  dispatch(result) {
    result?.relay?.forEach((message) => this.writeRelay(message));
    result?.robot?.forEach((message) => this.handleCompanionCommand(message));
  }

  sendMavlink(messageId, payload) {
    const frame = encodeMavlinkFrame(messageId, payload, {
      sequence: this.sequence++,
      systemId: 255,
      componentId: 190,
    });
    this.socket.send(frame, this.port, this.host);
  }

  sendGroundStationHeartbeat() {
    const payload = Buffer.alloc(9);
    payload.writeUInt32LE(0, 0);
    payload.writeUInt8(MAV_TYPE_GCS, 4);
    payload.writeUInt8(MAV_AUTOPILOT_INVALID, 5);
    payload.writeUInt8(0, 6);
    payload.writeUInt8(4, 7);
    payload.writeUInt8(3, 8);
    this.sendMavlink(MAVLINK_MESSAGE_IDS.HEARTBEAT, payload);
  }

  handleUdpPacket(packet) {
    for (const message of this.parser.push(packet)) this.handleMavlinkMessage(message);
  }

  handleMavlinkMessage(message) {
    const now = nowMs();
    this.systemId = message.systemId || this.systemId;
    this.componentId = message.componentId || this.componentId;
    switch (message.messageId) {
      case MAVLINK_MESSAGE_IDS.HEARTBEAT:
        this.handleHeartbeat(message.payload, now);
        break;
      case MAVLINK_MESSAGE_IDS.SYS_STATUS:
        this.handleSysStatus(message.payload);
        break;
      case MAVLINK_MESSAGE_IDS.GPS_RAW_INT:
        this.handleGpsRaw(message.payload, now);
        break;
      case MAVLINK_MESSAGE_IDS.GLOBAL_POSITION_INT:
        this.handleGlobalPosition(message.payload, now);
        break;
      case MAVLINK_MESSAGE_IDS.OBSTACLE_DISTANCE:
        this.handleObstacleDistance(message.payload, now);
        break;
      case MAVLINK_MESSAGE_IDS.MISSION_CURRENT:
        this.mission.current = readUInt16(message.payload, 0);
        this.mission.count = readUInt16(message.payload, 2, this.mission.count);
        this.mission.state = readUInt8(message.payload, 4, 0) === 0 ? "active" : "paused";
        this.publishTelemetry(now);
        this.ackPendingFromState();
        break;
      case MAVLINK_MESSAGE_IDS.MISSION_ITEM_REACHED:
        this.mission.reached = readUInt16(message.payload, 0);
        this.publishTelemetry(now);
        break;
      case MAVLINK_MESSAGE_IDS.MISSION_REQUEST:
      case MAVLINK_MESSAGE_IDS.MISSION_REQUEST_INT:
        this.handleMissionRequest(message.payload);
        break;
      case MAVLINK_MESSAGE_IDS.MISSION_ACK:
        this.handleMissionAck(message.payload, now);
        break;
      case MAVLINK_MESSAGE_IDS.COMMAND_ACK:
        this.handleCommandAck(message.payload, now);
        break;
      case MAVLINK_MESSAGE_IDS.PARAM_VALUE:
        this.handleParamValue(message.payload, now);
        break;
      case MAVLINK_MESSAGE_IDS.HOME_POSITION:
        this.home = {
          lat: readInt32(message.payload, 0) / 1e7,
          lon: readInt32(message.payload, 4) / 1e7,
          altM: readInt32(message.payload, 8) / 1_000,
        };
        this.publishTelemetry(now);
        break;
      case MAVLINK_MESSAGE_IDS.EKF_STATUS_REPORT:
        this.ekfHealthy = (readUInt16(message.payload, 0) & 0x03ff) !== 0;
        this.publishTelemetry(now);
        break;
      case MAVLINK_MESSAGE_IDS.STATUSTEXT:
        this.statusText = readText(message.payload, 1, 50);
        this.publishTelemetry(now);
        break;
      default:
        break;
    }
  }

  handleHeartbeat(payload, now) {
    const customMode = readUInt32(payload, 0);
    const baseMode = readUInt8(payload, 6);
    const systemStatus = readUInt8(payload, 7);
    const armed = (baseMode & MAV_MODE_FLAG_SAFETY_ARMED) !== 0;
    this.lastHeartbeatAt = now;
    this.currentMode = roverModeName(customMode);
    this.currentModeCode = customMode;
    this.currentArmed = armed;
    this.failsafe = systemStatus >= 5;
    this.mission.paused = roverModeName(customMode) === "hold";
    if (!this.core.robotConnected) {
      this.dispatch(this.core.handleRobot({
        protocol: DOMINO_ROBOT_LINK_PROTOCOL,
        type: "robot-hello",
        robotId: `ardupilot-${this.systemId}`,
        robotName: "ArduPilot Rover",
        firmwareVersion: "ArduPilot MAVLink",
        robotState: armed ? "armed" : "disarmed",
        capabilities: {
          telemetry: true,
          navigation: true,
          gps: true,
          lidar: true,
          ardupilot: true,
          autonomy: true,
          manualControl: false,
        },
      }, now));
    }
    this.publishTelemetry(now, {
      mode: this.currentMode,
      modeCode: this.currentModeCode,
      armed,
      heartbeat: true,
      prearmReady: armed ? true : this.prearmReady,
      ekfHealthy: this.ekfHealthy,
      failsafe: this.failsafe,
      statusText: this.statusText,
      mission: this.mission,
      home: this.home,
    });
    this.ackPendingFromState({ mode: roverModeName(customMode), modeCode: customMode, armed });
  }

  handleSysStatus(payload) {
    const health = readUInt32(payload, 8);
    const voltageV = readUInt16(payload, 14, 0) / 1_000;
    const batteryRemaining = payload.length > 18 ? payload.readInt8(18) : -1;
    this.prearmReady = (health & MAV_SYS_STATUS_PREARM_CHECK) !== 0;
    if (voltageV > 0) this.power = {
      timestampMs: nowMs(),
      voltageV,
      ...(batteryRemaining >= 0 ? { remainingPercent: batteryRemaining } : {}),
    };
    this.publishTelemetry(nowMs());
  }

  handleGpsRaw(payload, now) {
    const velocityCms = readUInt16(payload, 24, 65535);
    const courseCentiDeg = readUInt16(payload, 26, 65535);
    this.gps = {
      timestampMs: now,
      source: "MAVLink GPS_RAW_INT",
      fixType: readUInt8(payload, 28),
      satellites: readUInt8(payload, 29),
      hdop: readUInt16(payload, 20, 0) / 100,
      vdop: readUInt16(payload, 22, 0) / 100,
      position: { lat: readInt32(payload, 8) / 1e7, lon: readInt32(payload, 12) / 1e7, altM: readInt32(payload, 16) / 1_000 },
      groundSpeedMps: velocityCms === 65535 ? null : velocityCms / 100,
      courseDeg: courseCentiDeg === 65535 ? null : courseCentiDeg / 100,
      altitudeM: readInt32(payload, 16) / 1_000,
      home: this.home,
    };
    this.publishTelemetry(now);
  }

  handleGlobalPosition(payload, now) {
    const vx = readInt16(payload, 20) / 100;
    const vy = readInt16(payload, 22) / 100;
    this.gps = {
      ...(this.gps || {}),
      timestampMs: now,
      source: "MAVLink GLOBAL_POSITION_INT",
      position: { lat: readInt32(payload, 4) / 1e7, lon: readInt32(payload, 8) / 1e7, altM: readInt32(payload, 12) / 1_000 },
      groundSpeedMps: Math.hypot(vx, vy),
      courseDeg: readUInt16(payload, 26) === 65535 ? null : readUInt16(payload, 26) / 100,
      altitudeM: readInt32(payload, 12) / 1_000,
      home: this.home,
    };
    this.publishTelemetry(now);
  }

  handleObstacleDistance(payload, now) {
    const rangesM = [];
    for (let index = 0; index < 72 && 8 + index * 2 + 2 <= payload.length; index += 1) {
      const distanceCm = readUInt16(payload, 8 + index * 2, 65535);
      rangesM.push(distanceCm === 65535 ? null : distanceCm / 100);
    }
    const increment = readFloat(payload, 158, 0) || readUInt8(payload, 153, 5);
    this.lidar = {
      timestampMs: now,
      online: true,
      sensorId: "MAVLink OBSTACLE_DISTANCE",
      frame: readUInt8(payload, 166, 12) === 12 ? "base_link" : `mav-frame-${readUInt8(payload, 166)}`,
      scanRateHz: this.lidar?.timestampMs ? 1_000 / Math.max(1, now - this.lidar.timestampMs) : null,
      minRangeM: readUInt16(payload, 152, 5) / 100,
      maxRangeM: readUInt16(payload, 154, 3_000) / 100,
      incrementDeg: increment || 5,
      offsetDeg: readFloat(payload, 162, 0),
      rangesM,
      obstacleCount: rangesM.filter((value) => Number.isFinite(value)).length,
    };
    this.publishTelemetry(now);
  }

  navigationPacket(autopilotOverride = null) {
    const autopilot = {
      timestampMs: this.lastHeartbeatAt || nowMs(),
      stack: "ArduPilot",
      vehicleType: "rover",
      systemId: this.systemId,
      componentId: this.componentId,
      armed: this.core.robotState === "armed" || autopilotOverride?.armed === true,
      mode: autopilotOverride?.mode || this.currentMode || "unknown",
      modeCode: autopilotOverride?.modeCode ?? this.currentModeCode ?? null,
      heartbeat: this.lastHeartbeatAt > 0,
      prearmReady: this.prearmReady,
      ekfHealthy: this.ekfHealthy,
      failsafe: this.failsafe,
      statusText: this.statusText,
      mission: this.mission,
      home: this.home,
    };
    return {
      gps: this.gps,
      lidar: this.lidar,
      autopilot,
      home: this.home,
      geofence: this.geofence,
      obstacleBehavior: this.obstacleBehavior,
    };
  }

  publishTelemetry(now = nowMs(), autopilotOverride = null) {
    if (autopilotOverride) {
      this.currentMode = autopilotOverride.mode;
      this.currentModeCode = autopilotOverride.modeCode;
      this.currentArmed = autopilotOverride.armed;
      this.currentAutopilot = autopilotOverride;
    }
    if (this.lastPublishAt && now - this.lastPublishAt < 75) return;
    this.lastPublishAt = now;
    const navigation = this.navigationPacket(this.currentAutopilot);
    const result = this.core.handleRobot({
      protocol: DOMINO_ROBOT_LINK_PROTOCOL,
      type: "robot-telemetry",
      robotState: this.currentArmed ? "armed" : "disarmed",
      robotTimeMs: now,
      navigation,
      ...(this.power ? { power: this.power } : {}),
      diagnostics: {
        robotState: this.currentArmed ? "armed" : "disarmed",
        autopilotHeartbeatAgeMs: this.lastHeartbeatAt ? Math.max(0, now - this.lastHeartbeatAt) : null,
        gpsFixType: this.gps?.fixType ?? null,
        gpsSatellites: this.gps?.satellites ?? null,
        ekfHealthy: this.ekfHealthy,
        prearmReady: this.prearmReady,
        obstacleGuardEnabled: this.obstacleBehavior.enabled,
        obstacleFrontM: this.lidar?.rangesM?.[0] ?? null,
      },
      capabilities: {
        telemetry: true,
        navigation: true,
        gps: true,
        lidar: true,
        ardupilot: true,
        autonomy: true,
        manualControl: false,
      },
    }, now);
    this.dispatch(result);
  }

  ackPendingFromState(state = {}) {
    const mode = state.mode || this.currentMode;
    const modeCodeValue = state.modeCode ?? this.currentModeCode;
    const armed = state.armed ?? this.currentArmed;
    for (const [requestId, pending] of this.pendingMavlink) {
      if (pending.kind === "mode" && (pending.mode === mode || pending.modeCode === modeCodeValue)) {
        this.pendingMavlink.delete(requestId);
        this.completeNavigation(pending.action, requestId, true, "ArduPilot reported the requested mode.", { mode, modeCode: modeCodeValue });
      } else if (pending.kind === "arm" && armed === pending.armed) {
        this.pendingMavlink.delete(requestId);
        this.completeNavigation(pending.action, requestId, true, "ArduPilot reported the requested arm state.", { mode, modeCode: modeCodeValue });
      }
    }
  }

  completeNavigation(action, requestId, accepted, reason = "", extra = {}) {
    const result = this.core.handleRobot({
      protocol: DOMINO_ROBOT_LINK_PROTOCOL,
      type: "robot-ack",
      kind: "navigation",
      action,
      requestId,
      accepted,
      robotState: this.currentArmed ? "armed" : "disarmed",
      reason: accepted ? undefined : reason,
      navigation: this.navigationPacket({ mode: this.currentMode, modeCode: this.currentModeCode, armed: this.currentArmed }),
      ...extra,
    }, nowMs());
    this.dispatch(result);
  }

  handleCommandAck(payload, now) {
    const command = readUInt16(payload, 0);
    const resultCode = readUInt8(payload, 2, 255);
    for (const [requestId, pending] of this.pendingMavlink) {
      if (pending.command !== command) continue;
      this.pendingMavlink.delete(requestId);
      this.completeNavigation(pending.action, requestId, resultCode === MAV_RESULT_ACCEPTED, `ArduPilot command ${command} returned result ${resultCode}.`);
      break;
    }
    this.publishTelemetry(now);
  }

  handleParamValue(payload, now) {
    const name = readText(payload, 8, 16);
    const value = readFloat(payload, 0);
    for (const [requestId, pending] of this.pendingMavlink) {
      if (pending.kind !== "parameter" || pending.name !== name) continue;
      this.pendingMavlink.delete(requestId);
      this.completeNavigation(pending.action, requestId, true, "ArduPilot confirmed the parameter value.", { state: { parameter: { name, value } } });
      break;
    }
    this.publishTelemetry(now);
  }

  handleMissionRequest(payload) {
    if (!this.missionUpload) return;
    const sequence = readUInt16(payload, 0);
    const waypoint = this.missionUpload.mission[sequence];
    if (!waypoint) return;
    this.sendMavlink(MAVLINK_MESSAGE_IDS.MISSION_ITEM_INT, this.missionItemPayload(waypoint, sequence));
  }

  handleMissionAck(payload, now) {
    const resultCode = readUInt8(payload, 0, 255);
    for (const [requestId, pending] of this.pendingMavlink) {
      if (pending.kind !== "mission") continue;
      this.pendingMavlink.delete(requestId);
      this.missionUpload = null;
      this.mission.uploaded = resultCode === MAV_MISSION_ACCEPTED;
      this.mission.count = pending.count;
      this.mission.state = resultCode === MAV_MISSION_ACCEPTED ? "uploaded" : "rejected";
      this.completeNavigation(pending.action, requestId, resultCode === MAV_MISSION_ACCEPTED, `ArduPilot mission result ${resultCode}.`, { mission: this.mission });
      break;
    }
    this.publishTelemetry(now);
  }

  handleCompanionCommand(message) {
    if (!message || message.protocol !== DOMINO_ROBOT_LINK_PROTOCOL || message.kind !== "navigation") return;
    const payload = message.payload?.payload || message.payload || {};
    const action = message.action;
    if (action === "request-navigation-state") {
      this.completeNavigation(action, message.requestId, true, "Current MAVLink navigation state returned.");
      return;
    }
    if (action === "set-mode") {
      const selectedMode = String(payload.mode || "").toLowerCase();
      const selectedCode = Number(payload.modeCode ?? modeCode(selectedMode));
      if (!Number.isFinite(selectedCode)) {
        this.completeNavigation(action, message.requestId, false, `Unknown ArduPilot Rover mode ${selectedMode}.`);
        return;
      }
      const baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (this.currentArmed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);
      const modePayload = Buffer.alloc(6);
      modePayload.writeUInt32LE(selectedCode >>> 0, 0);
      modePayload.writeUInt8(this.systemId, 4);
      modePayload.writeUInt8(baseMode, 5);
      this.sendMavlink(MAVLINK_MESSAGE_IDS.SET_MODE, modePayload);
      this.pendingMavlink.set(message.requestId, { kind: "mode", action, mode: selectedMode, modeCode: selectedCode });
      return;
    }
    if (action === "arm" || action === "disarm") {
      this.sendCommandLong(MAV_COMMANDS.COMPONENT_ARM_DISARM, [action === "arm" ? 1 : 0, 0, 0, 0, 0, 0, 0]);
      this.pendingMavlink.set(message.requestId, { kind: "arm", action, armed: action === "arm", command: MAV_COMMANDS.COMPONENT_ARM_DISARM });
      return;
    }
    if (action === "start-mission") {
      this.sendCommandLong(MAV_COMMANDS.MISSION_START, [0, -1, 0, 0, 0, 0, 0]);
      this.pendingMavlink.set(message.requestId, { kind: "command", action, command: MAV_COMMANDS.MISSION_START });
      return;
    }
    if (action === "pause-mission") {
      this.sendSetMode("hold", message.requestId, action);
      return;
    }
    if (action === "resume-mission") {
      this.sendSetMode("auto", message.requestId, action);
      return;
    }
    if (action === "clear-mission") {
      const clearPayload = Buffer.from([this.systemId, this.componentId, MAV_MISSION_TYPE_MISSION]);
      this.sendMavlink(MAVLINK_MESSAGE_IDS.MISSION_CLEAR_ALL, clearPayload);
      this.pendingMavlink.set(message.requestId, { kind: "mission", action, count: 0 });
      return;
    }
    if (action === "upload-mission") {
      const mission = Array.isArray(payload.mission) ? payload.mission : [];
      const countPayload = Buffer.alloc(5);
      countPayload.writeUInt8(this.systemId, 0);
      countPayload.writeUInt8(this.componentId, 1);
      countPayload.writeUInt16LE(mission.length, 2);
      countPayload.writeUInt8(MAV_MISSION_TYPE_MISSION, 4);
      this.missionUpload = { mission };
      this.pendingMavlink.set(message.requestId, { kind: "mission", action, count: mission.length });
      this.sendMavlink(MAVLINK_MESSAGE_IDS.MISSION_COUNT, countPayload);
      return;
    }
    if (action === "set-home") {
      const home = payload.useCurrent === true ? null : payload.home;
      this.sendCommandLong(MAV_COMMANDS.DO_SET_HOME, [home ? 0 : 1, 0, 0, 0, home?.lat || 0, home?.lon || 0, home?.altM || 0]);
      this.pendingMavlink.set(message.requestId, { kind: "command", action, command: MAV_COMMANDS.DO_SET_HOME });
      return;
    }
    if (action === "goto") {
      const target = payload.target;
      const gotoPayload = Buffer.alloc(53);
      gotoPayload.writeUInt32LE(0, 0);
      gotoPayload.writeInt32LE(Math.round(Number(target.lat) * 1e7), 4);
      gotoPayload.writeInt32LE(Math.round(Number(target.lon) * 1e7), 8);
      gotoPayload.writeFloatLE(Number(target.altM) || 0, 12);
      gotoPayload.writeUInt16LE(0x0df8, 48);
      gotoPayload.writeUInt8(this.systemId, 50);
      gotoPayload.writeUInt8(this.componentId, 51);
      gotoPayload.writeUInt8(MAV_FRAME_GLOBAL_INT, 52);
      this.sendMavlink(MAVLINK_MESSAGE_IDS.SET_POSITION_TARGET_GLOBAL_INT, gotoPayload);
      this.completeNavigation(action, message.requestId, true, "Guided target sent to ArduPilot.");
      return;
    }
    if (action === "set-obstacle-behavior") {
      this.obstacleBehavior = { ...this.obstacleBehavior, ...payload };
      this.completeNavigation(action, message.requestId, true, "Obstacle guard policy is active in the companion and UI.", { state: { obstacleBehavior: this.obstacleBehavior } });
      return;
    }
    if (action === "set-geofence") {
      this.geofence = {
        ...this.geofence,
        enabled: payload.enabled === true,
        maxRadiusM: Number(payload.maxRadiusM) || null,
        polygon: Array.isArray(payload.polygon) ? payload.polygon : [],
      };
      this.sendParameter("FENCE_ENABLE", this.geofence.enabled ? 1 : 0);
      if (this.geofence.maxRadiusM) this.sendParameter("FENCE_RADIUS", this.geofence.maxRadiusM);
      this.completeNavigation(action, message.requestId, true, "ArduPilot geofence parameters requested; verify the vehicle reports the active fence.", { state: { geofence: this.geofence } });
      return;
    }
    if (action === "set-parameter") {
      this.sendParameter(String(payload.name), Number(payload.value));
      this.pendingMavlink.set(message.requestId, { kind: "parameter", action, name: String(payload.name) });
    }
  }

  sendSetMode(mode, requestId, action) {
    const code = modeCode(mode);
    if (code === null) {
      this.completeNavigation(action, requestId, false, `Unknown ArduPilot mode ${mode}.`);
      return;
    }
    const payload = Buffer.alloc(6);
    payload.writeUInt32LE(code, 0);
    payload.writeUInt8(this.systemId, 4);
    payload.writeUInt8(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (this.currentArmed ? MAV_MODE_FLAG_SAFETY_ARMED : 0), 5);
    this.sendMavlink(MAVLINK_MESSAGE_IDS.SET_MODE, payload);
    this.pendingMavlink.set(requestId, { kind: "mode", action, mode, modeCode: code });
  }

  sendCommandLong(command, params = []) {
    const payload = Buffer.alloc(33);
    for (let index = 0; index < 7; index += 1) payload.writeFloatLE(Number(params[index] || 0), index * 4);
    payload.writeUInt16LE(command, 28);
    payload.writeUInt8(this.systemId, 30);
    payload.writeUInt8(this.componentId, 31);
    payload.writeUInt8(0, 32);
    this.sendMavlink(MAVLINK_MESSAGE_IDS.COMMAND_LONG, payload);
  }

  sendParameter(name, value) {
    const payload = Buffer.alloc(23);
    payload.writeFloatLE(Number(value) || 0, 0);
    payload.writeUInt8(this.systemId, 4);
    payload.writeUInt8(this.componentId, 5);
    writeString(payload, 6, 16, name);
    payload.writeUInt8(MAV_PARAM_TYPE_REAL32, 22);
    this.sendMavlink(MAVLINK_MESSAGE_IDS.PARAM_SET, payload);
  }

  missionItemPayload(waypoint, sequence) {
    const payload = Buffer.alloc(38);
    payload.writeFloatLE(0, 0);
    payload.writeFloatLE(0, 4);
    payload.writeFloatLE(0, 8);
    payload.writeFloatLE(0, 12);
    payload.writeInt32LE(Math.round(Number(waypoint.lat) * 1e7), 16);
    payload.writeInt32LE(Math.round(Number(waypoint.lon) * 1e7), 20);
    payload.writeFloatLE(Number(waypoint.altM) || 0, 24);
    payload.writeUInt16LE(sequence, 28);
    payload.writeUInt16LE(MAV_MISSION_TYPE_MISSION_ITEM, 30);
    payload.writeUInt8(this.systemId, 32);
    payload.writeUInt8(this.componentId, 33);
    payload.writeUInt8(MAV_FRAME_GLOBAL, 34);
    payload.writeUInt8(sequence === 0 ? 1 : 0, 35);
    payload.writeUInt8(1, 36);
    payload.writeUInt8(MAV_MISSION_TYPE_MISSION, 37);
    return payload;
  }
}

async function main() {
  const options = optionsFromArgs(process.argv.slice(2));
  if (options.help) {
    console.log(`ArduPilot MAVLink companion adapter

UDP: node ardupilot-companion-adapter.mjs --mavlink-host 127.0.0.1 --mavlink-port 14550

Optional: --relay ws://127.0.0.1:8770/control --adapter-id ardupilot-mavlink-1`);
    return;
  }
  const host = options["mavlink-host"] || options.host || "127.0.0.1";
  const port = Number(options["mavlink-port"] || options.port || 14550);
  const bridge = new ArduPilotMavlinkBridge({
    host,
    port,
    relayUrl: options.relay || "ws://127.0.0.1:8770/control",
    adapterId: options["adapter-id"] || "ardupilot-mavlink-1",
    name: options.name || "ArduPilot MAVLink Companion",
    endpoint: `${host}:${port}`,
  });
  bridge.start();
  const shutdown = () => { bridge.stop(); process.exit(0); };
  process.once("SIGINT", shutdown);
  process.once("SIGTERM", shutdown);
}

if (process.argv[1] && path.resolve(fileURLToPath(import.meta.url)) === path.resolve(process.argv[1])) {
  main().catch((error) => {
    console.error(error instanceof Error ? error.message : error);
    process.exitCode = 1;
  });
}
