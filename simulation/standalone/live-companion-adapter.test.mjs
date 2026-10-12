import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { createServer as createHttpServer } from "node:http";
import net from "node:net";
import process from "node:process";
import test from "node:test";
import { WebSocketServer } from "ws";

import { DOMINO_ROBOT_LINK_PROTOCOL } from "./live-companion-core.mjs";

const integrationLinkKey = "integration-link-key-123";

const listen = (server) => new Promise((resolve, reject) => {
  server.once("error", reject);
  server.listen(0, "127.0.0.1", () => resolve(server.address().port));
});
const closeServer = (server) => new Promise((resolve) => server.close(resolve));

function robotTelemetry(timestampMs) {
  const pose = {
    timestampMs,
    servoAngleDeg: Array(16).fill(135),
    body: { rollDeg: 0, pitchDeg: 0, yawDeg: 0, heightMm: 280 },
  };
  return {
    type: "robot-telemetry",
    protocol: DOMINO_ROBOT_LINK_PROTOCOL,
    robotState: "disarmed",
    expected: pose,
    measured: pose,
    power: { voltageV: 15.8, currentA: 1.1, powerW: 17.38 },
    controller: {
      source: "boxer-elrs", frameTimestampMs: timestampMs, packetRateHz: 150,
      frameLossCount: 0, failsafe: false, failsafeCount: 0,
      linkQualityPercent: 95, rssi1Dbm: -62, rssi2Dbm: -64,
      snrDb: 8, rfMode: "250hz", txPowerMw: 100,
      activeAntenna: 1, receiverVoltageV: 5.1, channelsUs: Array(16).fill(1500),
    },
    diagnostics: { robotState: "disarmed" },
  };
}

test("the runnable adapter bridges relay commands and physical acknowledgements", { timeout: 10_000 }, async () => {
  let robotSocket;
  let robotBuffer = "";
  const robotServer = net.createServer((socket) => {
    robotSocket = socket;
    const now = Date.now();
    socket.write(`${JSON.stringify({
      type: "robot-hello", protocol: DOMINO_ROBOT_LINK_PROTOCOL,
      robotId: "domino-e2e", robotName: "Domino integration", firmwareVersion: "test", robotState: "disarmed",
      capabilities: { telemetry: true, calibration: true, gaitProfiles: true, persistentProfiles: true, manualControl: true },
    })}\n`);
    socket.write(`${JSON.stringify(robotTelemetry(now))}\n`);
    socket.setEncoding("utf8");
    socket.on("data", (chunk) => {
      robotBuffer += chunk;
      while (robotBuffer.includes("\n")) {
        const newline = robotBuffer.indexOf("\n");
        const line = robotBuffer.slice(0, newline).trim();
        robotBuffer = robotBuffer.slice(newline + 1);
        if (!line.startsWith("{")) continue;
        const message = JSON.parse(line);
        if (message.type === "companion-auth") assert.equal(message.linkKey, integrationLinkKey);
        if (message.kind === "safety" && message.action === "request-state") {
          assert.equal(message.linkKey, integrationLinkKey);
          socket.write(`${JSON.stringify({
            type: "robot-ack", protocol: DOMINO_ROBOT_LINK_PROTOCOL,
            kind: "safety", action: "request-state", requestId: message.requestId,
            accepted: true, robotState: "disarmed",
          })}\n`);
        }
      }
    });
  });
  const robotPort = await listen(robotServer);

  const httpServer = createHttpServer();
  const relayPort = await listen(httpServer);
  const relayServer = new WebSocketServer({ server: httpServer, path: "/control" });
  let child;
  let stderr = "";
  try {
    const result = await new Promise((resolve, reject) => {
      const timeout = setTimeout(() => reject(new Error(`Adapter integration timed out. ${stderr}`)), 8_000);
      relayServer.once("connection", (socket) => {
        let connectRequested = false;
        let stateRequested = false;
        socket.on("message", (payload) => {
          const message = JSON.parse(payload.toString());
          if (message.type === "live-adapter-announce" && message.state === "available" && !connectRequested) {
            connectRequested = true;
            socket.send(JSON.stringify({
              type: "live-connection-command", action: "connect", requestId: "connect-e2e",
              timestampMs: Date.now(), adapterId: "adapter-e2e", transport: "wifi",
              safety: { readOnlyHandshake: true, commandsBlockedUntilStateKnown: true },
            }));
          }
          if (message.type === "live-connection-ack" && message.accepted && !stateRequested) {
            stateRequested = true;
            socket.send(JSON.stringify({
              type: "live-safety-command", action: "request-state", requestId: "state-e2e",
              timestampMs: Date.now(), adapterId: "adapter-e2e", sessionId: message.sessionId,
            }));
          }
          if (message.type === "live-safety-ack" && message.requestId === "state-e2e") {
            clearTimeout(timeout);
            resolve(message);
          }
        });
      });

      child = spawn(process.execPath, [
        "live-companion-adapter.mjs",
        "--robot-host", "127.0.0.1",
        "--robot-port", String(robotPort),
        "--relay", `ws://127.0.0.1:${relayPort}/control`,
        "--adapter-id", "adapter-e2e",
      ], {
        cwd: import.meta.dirname, windowsHide: true, stdio: ["ignore", "pipe", "pipe"],
        env: { ...process.env, DOMINO_ROBOT_LINK_KEY: integrationLinkKey },
      });
      child.stderr.setEncoding("utf8");
      child.stderr.on("data", (chunk) => { stderr += chunk; });
      child.once("exit", (code) => {
        if (code && code !== 0) reject(new Error(`Adapter exited ${code}. ${stderr}`));
      });
    });
    assert.equal(result.accepted, true);
    assert.equal(result.robotState, "disarmed");
    assert.ok(robotSocket, "the companion did not establish its robot TCP link");
  } finally {
    child?.kill("SIGTERM");
    robotSocket?.destroy();
    relayServer.close();
    await closeServer(httpServer);
    await closeServer(robotServer);
  }
});
