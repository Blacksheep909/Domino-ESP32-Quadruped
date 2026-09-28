import assert from "node:assert/strict";
import test from "node:test";

import {
  ArduPilotMavlinkBridge,
  MAVLINK_MESSAGE_IDS,
  MavlinkParser,
  encodeMavlinkFrame,
} from "./ardupilot-companion-adapter.mjs";

function heartbeatPayload(customMode = 10, armed = false, systemStatus = 4) {
  const payload = Buffer.alloc(9);
  payload.writeUInt32LE(customMode, 0);
  payload.writeUInt8(10, 4);
  payload.writeUInt8(3, 5);
  payload.writeUInt8(armed ? 128 : 0, 6);
  payload.writeUInt8(systemStatus, 7);
  payload.writeUInt8(3, 8);
  return payload;
}

test("MAVLink v2 frames round-trip through the streaming parser", () => {
  const frame = encodeMavlinkFrame(MAVLINK_MESSAGE_IDS.HEARTBEAT, heartbeatPayload(), {
    sequence: 7,
    systemId: 1,
    componentId: 1,
  });
  const parser = new MavlinkParser();
  assert.deepEqual(parser.push(frame.subarray(0, 4)), []);
  const [message] = parser.push(frame.subarray(4));
  assert.equal(message.messageId, MAVLINK_MESSAGE_IDS.HEARTBEAT);
  assert.equal(message.sequence, 7);
  assert.equal(message.systemId, 1);
  assert.deepEqual(message.payload, heartbeatPayload());
});

test("the parser drops frames with an invalid CRC", () => {
  const frame = encodeMavlinkFrame(MAVLINK_MESSAGE_IDS.HEARTBEAT, heartbeatPayload());
  frame[frame.length - 1] ^= 0xff;
  assert.deepEqual(new MavlinkParser().push(frame), []);
});

test("the bridge turns ArduPilot heartbeats and GPS packets into navigation state", () => {
  const bridge = new ArduPilotMavlinkBridge({ logger: {} });
  bridge.handleMavlinkMessage({
    messageId: MAVLINK_MESSAGE_IDS.HEARTBEAT,
    payload: heartbeatPayload(10, true),
    systemId: 1,
    componentId: 1,
  });
  const gps = Buffer.alloc(30);
  gps.writeBigUInt64LE(1_000_000n, 0);
  gps.writeInt32LE(Math.round(-36.85 * 1e7), 8);
  gps.writeInt32LE(Math.round(174.76 * 1e7), 12);
  gps.writeInt32LE(22_000, 16);
  gps.writeUInt16LE(90, 20);
  gps.writeUInt16LE(120, 22);
  gps.writeUInt16LE(150, 24);
  gps.writeUInt16LE(9_000, 26);
  gps.writeUInt8(3, 28);
  gps.writeUInt8(12, 29);
  bridge.handleMavlinkMessage({
    messageId: MAVLINK_MESSAGE_IDS.GPS_RAW_INT,
    payload: gps,
    systemId: 1,
    componentId: 1,
  });
  assert.equal(bridge.currentMode, "auto");
  assert.equal(bridge.currentArmed, true);
  assert.equal(bridge.gps.fixType, 3);
  assert.equal(bridge.gps.satellites, 12);
  assert.equal(bridge.core.robotConnected, true);
  bridge.stop();
});

test("MAVLink UDP listens on the GCS port and replies to the vehicle sender", () => {
  const bridge = new ArduPilotMavlinkBridge({ logger: {}, host: "127.0.0.1", port: 14550 });
  assert.equal(bridge.localPort, 14550);
  const sent = [];
  bridge.socket.send = (_frame, port, host) => sent.push({ port, host });
  const heartbeat = encodeMavlinkFrame(MAVLINK_MESSAGE_IDS.HEARTBEAT, heartbeatPayload(), { systemId: 1 });
  bridge.handleUdpPacket(heartbeat, { address: "127.0.0.1", port: 49152 });
  bridge.sendGroundStationHeartbeat();
  assert.deepEqual(sent, [{ port: 49152, host: "127.0.0.1" }]);
  bridge.stop();
});

test("an echoed ground-station heartbeat cannot impersonate a vehicle", () => {
  const bridge = new ArduPilotMavlinkBridge({ logger: {} });
  const groundStation = Buffer.alloc(9);
  groundStation.writeUInt8(6, 4);
  const echo = encodeMavlinkFrame(MAVLINK_MESSAGE_IDS.HEARTBEAT, groundStation, { systemId: 255 });
  bridge.handleUdpPacket(echo, { address: "127.0.0.1", port: 14550 });
  assert.equal(bridge.remoteEndpoint, null);
  assert.equal(bridge.core.robotConnected, false);
  bridge.stop();
});

test("GPS sentinels and LiDAR clear sectors stay distinct from measured obstacles", () => {
  const bridge = new ArduPilotMavlinkBridge({ logger: {} });
  const gps = Buffer.alloc(30);
  gps.writeUInt16LE(65535, 20);
  gps.writeUInt16LE(65535, 22);
  gps.writeUInt8(1, 28);
  gps.writeUInt8(255, 29);
  bridge.handleGpsRaw(gps, 10_000);
  assert.equal(bridge.gps.position, null);
  assert.equal(bridge.gps.satellites, null);
  assert.equal(bridge.gps.hdop, null);
  const lidar = Buffer.alloc(167, 255);
  lidar.writeUInt16LE(5, 152);
  lidar.writeUInt16LE(300, 154);
  lidar.writeUInt8(10, 157);
  lidar.writeFloatLE(0, 158);
  lidar.writeFloatLE(0, 162);
  lidar.writeUInt8(12, 166);
  lidar.writeUInt16LE(0, 8);
  lidar.writeUInt16LE(301, 10);
  bridge.handleObstacleDistance(lidar, 11_000);
  assert.equal(bridge.lidar.incrementDeg, 10);
  assert.equal(bridge.lidar.rangesM[0], 0);
  assert.equal(bridge.lidar.rangesM[1], 3);
  assert.equal(bridge.lidar.rangesM[2], null);
  assert.equal(bridge.lidar.obstacleCount, 1);
  bridge.stop();
});
