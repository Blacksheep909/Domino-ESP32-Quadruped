import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { mkdtempSync, rmSync } from "node:fs";
import net from "node:net";
import { tmpdir } from "node:os";
import path from "node:path";
import process from "node:process";
import test from "node:test";
import { fileURLToPath } from "node:url";

import { WebSocket } from "ws";

const standaloneRoot = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(standaloneRoot, "..", "..");

function freePort() {
  return new Promise((resolve, reject) => {
    const probe = net.createServer();
    probe.once("error", reject);
    probe.listen(0, "127.0.0.1", () => {
      const { port } = probe.address();
      probe.close((error) => error ? reject(error) : resolve(port));
    });
  });
}

function waitForOutput(child, pattern, timeoutMs = 5_000) {
  return new Promise((resolve, reject) => {
    let output = "";
    const timer = setTimeout(() => {
      cleanup();
      reject(new Error(`Timed out waiting for server output ${pattern}: ${output}`));
    }, timeoutMs);
    const onData = (chunk) => {
      output += String(chunk);
      if (!pattern.test(output)) return;
      cleanup();
      resolve(output);
    };
    const onExit = (code, signal) => {
      cleanup();
      reject(new Error(`Server exited before becoming ready (${code ?? signal}): ${output}`));
    };
    const cleanup = () => {
      clearTimeout(timer);
      child.stdout?.off("data", onData);
      child.off("exit", onExit);
    };
    child.stdout.on("data", onData);
    child.once("exit", onExit);
  });
}

function openSocket(url) {
  return new Promise((resolve, reject) => {
    const socket = new WebSocket(url);
    const onError = (error) => {
      socket.off("open", onOpen);
      reject(error);
    };
    const onOpen = () => {
      socket.off("error", onError);
      resolve(socket);
    };
    socket.once("error", onError);
    socket.once("open", onOpen);
  });
}

function waitForMessage(socket, predicate, timeoutMs = 3_000) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      cleanup();
      reject(new Error("Timed out waiting for WebSocket message."));
    }, timeoutMs);
    const onMessage = (payload) => {
      let message;
      try { message = JSON.parse(payload.toString()); } catch { return; }
      if (!predicate(message)) return;
      cleanup();
      resolve(message);
    };
    const cleanup = () => {
      clearTimeout(timer);
      socket.off("message", onMessage);
    };
    socket.on("message", onMessage);
  });
}

function noMessage(socket, predicate, timeoutMs = 350) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      cleanup();
      resolve(true);
    }, timeoutMs);
    const onMessage = (payload) => {
      let message;
      try { message = JSON.parse(payload.toString()); } catch { return; }
      if (!predicate(message)) return;
      cleanup();
      reject(new Error("Unexpected WebSocket message."));
    };
    const cleanup = () => {
      clearTimeout(timer);
      socket.off("message", onMessage);
    };
    socket.on("message", onMessage);
  });
}

function closeSocket(socket) {
  if (!socket || socket.readyState === WebSocket.CLOSED) return Promise.resolve();
  return new Promise((resolve) => {
    const timer = setTimeout(resolve, 500);
    socket.once("close", () => {
      clearTimeout(timer);
      resolve();
    });
    socket.close();
  });
}

async function requestJson(url, options) {
  const response = await fetch(url, options);
  return { status: response.status, body: await response.json() };
}

test("running service enforces API errors, session-bound telemetry, and adapter expiry", { timeout: 18_000 }, async () => {
  const port = await freePort();
  const tempRoot = mkdtempSync(path.join(tmpdir(), "domino-server-"));
  const runtimeRoot = path.join(tempRoot, "runtime");
  const child = spawn(process.execPath, [path.join(standaloneRoot, "server.mjs")], {
    cwd: standaloneRoot,
    windowsHide: true,
    stdio: ["ignore", "pipe", "pipe"],
    env: {
      ...process.env,
      DOMINO_STANDALONE_PORT: String(port),
      DOMINO_PROJECT_ROOT: repoRoot,
      DOMINO_DIST_ROOT: path.join(standaloneRoot, "dist"),
      DOMINO_RUNTIME_ROOT: runtimeRoot,
      DOMINO_CAD_ROOT: path.join(tempRoot, "cad"),
      DOMINO_COMPANION_ENTRY: path.join(standaloneRoot, "live-companion-adapter.mjs"),
      DOMINO_DISABLE_RAW_HID: "1",
      DOMINO_EMBEDDED_DESKTOP: "0",
    },
  });
  let browserSocket;
  let adapterSocket;
  try {
    child.stderr.on("data", () => {});
    await waitForOutput(child, /Domino Virtual Lab:/);

    const baseUrl = `http://127.0.0.1:${port}`;
    const unknownApi = await requestJson(`${baseUrl}/api/not-a-real-endpoint`);
    assert.equal(unknownApi.status, 404);
    assert.equal(unknownApi.body.error, "API endpoint not found.");
    const bareApi = await requestJson(`${baseUrl}/api`);
    assert.equal(bareApi.status, 404);
    assert.equal(bareApi.body.error, "API endpoint not found.");

    const malformedJson = await requestJson(`${baseUrl}/api/companion/discover`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: "{not-json",
    });
    assert.equal(malformedJson.status, 400);
    assert.equal(malformedJson.body.error, "Request body must be valid JSON.");

    const oversized = await requestJson(`${baseUrl}/api/companion/discover`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ transport: "usb", padding: "x".repeat(40_000) }),
    });
    assert.equal(oversized.status, 413);
    assert.equal(oversized.body.error, "Request body is too large.");

    browserSocket = await openSocket(`ws://127.0.0.1:${port}/control`);
    adapterSocket = await openSocket(`ws://127.0.0.1:${port}/control`);
    const adapterId = "integration-adapter";
    const sessionId = "integration-session-old";
    const announcement = {
      type: "live-adapter-announce",
      adapterId,
      name: "Integration adapter",
      transport: "wifi",
      state: "available",
      timestampMs: Date.now(),
      robot: { id: "domino-integration", name: "Domino", firmwareVersion: "test" },
      capabilities: { telemetry: true, calibration: true, gaitProfiles: true, manualControl: true },
    };
    const announced = waitForMessage(browserSocket, (message) =>
      message.type === "live-adapter-announce" && message.adapterId === adapterId);
    adapterSocket.send(JSON.stringify(announcement));
    await announced;

    const requestId = "integration-connect";
    const connectCommand = {
      type: "live-connection-command",
      action: "connect",
      requestId,
      adapterId,
      transport: "wifi",
      timestampMs: Date.now(),
      safety: { readOnlyHandshake: true, commandsBlockedUntilStateKnown: true },
    };
    const adapterConnect = waitForMessage(adapterSocket, (message) =>
      message.type === "live-connection-command" && message.requestId === requestId);
    browserSocket.send(JSON.stringify(connectCommand));
    await adapterConnect;

    const connected = waitForMessage(browserSocket, (message) =>
      message.type === "live-connection-ack" && message.requestId === requestId);
    adapterSocket.send(JSON.stringify({
      type: "live-connection-ack",
      action: "connect",
      requestId,
      accepted: true,
      adapterId,
      sessionId,
      robotState: "disarmed",
    }));
    assert.equal((await connected).accepted, true);

    const pose = {
      timestampMs: Date.now(),
      servoAngleDeg: Array(16).fill(135),
      body: { rollDeg: 0, pitchDeg: 0, yawDeg: 0, heightMm: 280 },
    };
    const telemetry = waitForMessage(browserSocket, (message) =>
      message.type === "live-telemetry" && message.sequence === 0);
    adapterSocket.send(JSON.stringify({
      type: "live-telemetry",
      adapterId,
      sessionId,
      sequence: 0,
      expected: pose,
    }));
    await telemetry;

    const malformedTelemetry = noMessage(
      browserSocket,
      (message) => message.type === "live-telemetry" && message.sequence === 1,
    );
    adapterSocket.send(JSON.stringify({
      type: "live-telemetry",
      adapterId,
      sessionId,
      sequence: 1,
      expected: { body: {} },
    }));
    await malformedTelemetry;

    const removed = waitForMessage(browserSocket, (message) =>
      message.type === "live-adapter-removed" && message.adapterId === adapterId, 7_000);
    await removed;

    const reannounced = waitForMessage(browserSocket, (message) =>
      message.type === "live-adapter-announce" && message.adapterId === adapterId);
    adapterSocket.send(JSON.stringify({ ...announcement, timestampMs: Date.now() }));
    await reannounced;

    const staleRequestId = "stale-safety-command";
    const staleCommand = noMessage(adapterSocket, (message) =>
      message.type === "live-safety-command" && message.requestId === staleRequestId);
    browserSocket.send(JSON.stringify({
      type: "live-safety-command",
      action: "request-state",
      requestId: staleRequestId,
      adapterId,
      sessionId,
      timestampMs: Date.now(),
    }));
    await staleCommand;
  } finally {
    await Promise.all([closeSocket(browserSocket), closeSocket(adapterSocket)]);
    if (child.exitCode === null) child.kill();
    await new Promise((resolve) => child.once("exit", resolve));
    rmSync(tempRoot, { recursive: true, force: true });
  }
});
