import {
  appendFileSync,
  createReadStream,
  existsSync,
  mkdirSync,
  readFileSync,
  statSync,
  writeFileSync,
} from "node:fs";
import { createServer } from "node:http";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocketServer } from "ws";

import { CrsfTransmitterHidInput } from "./boxer-hid.mjs";
import { FirmwareService } from "./firmware-service.mjs";
import {
  clientControlIsFresh,
  hasActiveClient,
  radioInputIsFresh,
  releaseClientControl,
} from "./control-state.mjs";
import {
  validCalibrationAcknowledgement,
  validCalibrationCommand,
} from "./web/src/live-calibration-protocol.js";
import {
  validLiveGaitAcknowledgement,
  validLiveGaitCommand,
} from "./web/src/live-gait-protocol.js";
import {
  validLiveAdapterAnnouncement,
  validLiveConnectionAcknowledgement,
  validLiveConnectionCommand,
  validSessionEnvelope,
} from "./web/src/live-connection-protocol.js";
import {
  validLiveSafetyAcknowledgement,
  validLiveSafetyCommand,
  validLiveSafetyHeartbeat,
  validLiveSafetyHeartbeatAcknowledgement,
} from "./web/src/live-safety-protocol.js";
import {
  validLiveManualAuthorityAcknowledgement,
  validLiveManualAuthorityCommand,
  validLiveManualControlFrame,
} from "./web/src/live-manual-control-protocol.js";

const here = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(here, "../..");
const distRoot = path.join(here, "dist");
const runtimeRoot = path.join(here, "runtime");
const cadRoot = path.join(
  repoRoot,
  "simulation",
  "urdf",
  "generated",
  "Domino_URDF_Parts_Combined_Final_description",
  "meshes",
);
const statePath = path.join(runtimeRoot, "state.json");
const stateSlotPaths = [`${statePath}.0`, `${statePath}.1`];
const controlPath = path.join(runtimeRoot, "controls.txt");
const debugStartedAt = new Date();
const debugStamp = debugStartedAt.toISOString().replaceAll(":", "-").replaceAll(".", "-");
const debugLogPath = path.join(runtimeRoot, `debug-session-${debugStamp}.jsonl`);
const port = Number(process.env.DOMINO_STANDALONE_PORT || 8770);
const maxDebugBytes = 64 * 1024 * 1024;
const firmwareService = new FirmwareService({ projectRoot: repoRoot, runtimeRoot });

mkdirSync(runtimeRoot, { recursive: true });
writeFileSync(
  debugLogPath,
  `${JSON.stringify({ type: "session", started_at: debugStartedAt.toISOString(), format: 1 })}\n`,
  "utf8",
);
let debugBytes = statSync(debugLogPath).size;

const channels = Array(16).fill(1500);
channels[4] = 1000;
// CRSF CH3 is the transmitter left-stick vertical and owns ride-height selection.
channels[2] = 2000;
channels[6] = 1000;
channels[7] = 1000;

function writeControls(nextChannels = channels) {
  const safeChannels = nextChannels.map((value) =>
    Math.max(1000, Math.min(2000, Math.round(Number(value) || 1500))),
  );
  writeFileSync(controlPath, `${safeChannels.join(" ")}\n`, "utf8");
}

writeControls();

let radioInput = {
  connected: false,
  name: "CRSF TRANSMITTER NOT FOUND",
  channels: null,
  axes: null,
  battery: null,
  buttonBits: 0,
  layout: null,
  updatedAt: 0,
};

const mimeTypes = {
  ".css": "text/css; charset=utf-8",
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".jsonl": "application/x-ndjson; charset=utf-8",
  ".map": "application/json; charset=utf-8",
  ".stl": "model/stl",
  ".svg": "image/svg+xml",
};

function sendFile(response, filePath) {
  if (!existsSync(filePath) || !statSync(filePath).isFile()) {
    response.writeHead(404);
    response.end("Not found");
    return;
  }
  response.writeHead(200, {
    "Cache-Control": filePath.endsWith(".stl") ? "public, max-age=3600" : "no-store",
    "Content-Type": mimeTypes[path.extname(filePath).toLowerCase()] || "application/octet-stream",
  });
  createReadStream(filePath).pipe(response);
}

function readLatestFirmwareState() {
  let latest = null;
  for (const slotPath of stateSlotPaths) {
    try {
      const candidate = JSON.parse(readFileSync(slotPath, "utf8"));
      if (!latest || Number(candidate.elapsed_ms) > Number(latest.elapsed_ms)) {
        latest = candidate;
      }
    } catch {
      // One slot may be observed while the firmware is rewriting it.
    }
  }
  return latest;
}

function sendFirmwareState(response) {
  const state = readLatestFirmwareState();
  if (!state) {
    response.writeHead(503, { "Content-Type": "application/json; charset=utf-8" });
    response.end(JSON.stringify({ error: "Firmware state unavailable" }));
    return;
  }
  response.writeHead(200, {
    "Cache-Control": "no-store",
    "Content-Type": "application/json; charset=utf-8",
  });
  response.end(JSON.stringify(state));
}

function safeJoinedPath(root, requestPath) {
  const candidate = path.resolve(root, `.${requestPath}`);
  const relative = path.relative(path.resolve(root), candidate);
  return relative === "" || (!relative.startsWith("..") && !path.isAbsolute(relative))
    ? candidate
    : null;
}

function sendJson(response, status, payload) {
  response.writeHead(status, {
    "Cache-Control": "no-store",
    "Content-Type": "application/json; charset=utf-8",
  });
  response.end(JSON.stringify(payload));
}

function readJsonBody(request, limit = 32 * 1024) {
  return new Promise((resolve, reject) => {
    let body = "";
    request.setEncoding("utf8");
    request.on("data", (chunk) => {
      body += chunk;
      if (body.length > limit) reject(new Error("Request body is too large."));
    });
    request.on("end", () => {
      try {
        resolve(body ? JSON.parse(body) : {});
      } catch {
        reject(new Error("Request body must be valid JSON."));
      }
    });
    request.on("error", reject);
  });
}

async function handleFirmwareApi(request, response, url) {
  try {
    if (request.method === "GET" && url.pathname === "/api/firmware/status") {
      sendJson(response, 200, firmwareService.status());
      return;
    }
    if (request.method === "GET" && url.pathname === "/api/firmware/ports") {
      sendJson(response, 200, { ports: await firmwareService.ports() });
      return;
    }
    if (request.method === "GET" && url.pathname === "/api/firmware/file") {
      const file = firmwareService.file(url.searchParams.get("path") || "");
      sendJson(response, file ? 200 : 404, file || { error: "Firmware file not found." });
      return;
    }
    if (request.method === "POST" && url.pathname === "/api/firmware/build") {
      sendJson(response, 202, { job: firmwareService.startBuild() });
      return;
    }
    if (request.method === "POST" && url.pathname === "/api/firmware/upload") {
      sendJson(response, 202, { job: firmwareService.startUpload(await readJsonBody(request)) });
      return;
    }
    if (request.method === "POST" && url.pathname === "/api/firmware/cancel") {
      sendJson(response, 200, { cancelled: firmwareService.cancel() });
      return;
    }
    sendJson(response, 404, { error: "Firmware endpoint not found." });
  } catch (error) {
    sendJson(response, 409, { error: error.message || "Firmware operation failed." });
  }
}

const server = createServer(async (request, response) => {
  const url = new URL(request.url || "/", `http://${request.headers.host || "127.0.0.1"}`);
  if (url.pathname.startsWith("/api/firmware/")) {
    await handleFirmwareApi(request, response, url);
    return;
  }
  if (url.pathname === "/runtime/state.json") {
    sendFirmwareState(response);
    return;
  }
  if (url.pathname === "/runtime/debug/latest.jsonl") {
    sendFile(response, debugLogPath);
    return;
  }
  if (url.pathname.startsWith("/cad/")) {
    const cadPath = safeJoinedPath(cadRoot, url.pathname.slice(4));
    if (!cadPath) {
      response.writeHead(403);
      response.end("Forbidden");
      return;
    }
    sendFile(response, cadPath);
    return;
  }

  const requestedPath = url.pathname === "/" ? "/index.html" : url.pathname;
  const staticPath = safeJoinedPath(distRoot, requestedPath);
  if (staticPath && existsSync(staticPath) && statSync(staticPath).isFile()) {
    sendFile(response, staticPath);
    return;
  }
  sendFile(response, path.join(distRoot, "index.html"));
});

const sockets = new WebSocketServer({ server, path: "/control" });
let activeControlSocket = null;
let browserInput = { source: "none", name: "", axes: [], physics: null };
let controlsSource = "safe";
const liveAdapters = new Map();

function broadcast(message) {
  const payload = JSON.stringify(message);
  for (const socket of sockets.clients) {
    if (socket.readyState === socket.OPEN) socket.send(payload);
  }
}

function sendSocket(socket, message) {
  if (socket?.readyState === socket?.OPEN) socket.send(JSON.stringify(message));
}

function liveAdapterList() {
  return [...liveAdapters.values()].map(({ announcement }) => announcement);
}

function liveAdapterForSession(message) {
  if (!validSessionEnvelope(message)) return null;
  const record = liveAdapters.get(message.adapterId);
  if (!record || record.socket.liveSessionId !== message.sessionId) return null;
  return record;
}

function rememberAdapterRequest(socket, message, kind) {
  socket.livePendingRequests ??= new Map();
  socket.livePendingRequests.set(message.requestId, {
    action: message.action,
    kind,
    sessionId: message.sessionId || "",
    sentAt: Date.now(),
  });
}

function consumeAdapterRequest(socket, message, kind) {
  const pending = socket?.livePendingRequests?.get(message.requestId);
  if (
    !pending ||
    pending.kind !== kind ||
    pending.action !== message.action ||
    (pending.sessionId && pending.sessionId !== message.sessionId)
  ) return false;
  socket.livePendingRequests.delete(message.requestId);
  return true;
}

function removeLiveAdapter(socket, reason = "offline") {
  if (!socket?.liveAdapterId) return;
  const record = liveAdapters.get(socket.liveAdapterId);
  if (record?.socket !== socket) return;
  liveAdapters.delete(socket.liveAdapterId);
  broadcast({
    type: "live-adapter-removed",
    adapterId: socket.liveAdapterId,
    reason,
    timestampMs: Date.now(),
  });
}

setInterval(() => {
  const now = Date.now();
  for (const { socket, receivedAt } of liveAdapters.values()) {
    if (now - receivedAt > 5_000) removeLiveAdapter(socket, "heartbeat-timeout");
    if (socket.livePendingRequests) {
      for (const [requestId, request] of socket.livePendingRequests) {
        if (now - request.sentAt > 5_000) socket.livePendingRequests.delete(requestId);
      }
    }
  }
}, 1_000);

function demoClientIsActive() {
  return hasActiveClient(sockets.clients, (socket) => socket.controlMode === "demo");
}

function manualClientIsActive() {
  return hasActiveClient(sockets.clients, (socket) => socket.manualOverride === true);
}

function writeSafeControls() {
  channels.fill(1500);
  channels[2] = 2000;
  channels[4] = 1000;
  channels[6] = 1000;
  channels[7] = 1000;
  writeControls(channels);
  controlsSource = "safe";
}

const radio = new CrsfTransmitterHidInput((state) => {
  const connectionChanged = state.connected !== radioInput.connected;
  radioInput = state;
  if (connectionChanged) broadcast({ type: "input", input: radioInput });
});
radio.start();

setInterval(() => {
  if (activeControlSocket && !clientControlIsFresh(activeControlSocket)) {
    releaseClientControl(activeControlSocket);
    activeControlSocket = null;
  }
  if (demoClientIsActive() || manualClientIsActive()) return;
  if (!radioInputIsFresh(radioInput)) {
    if (controlsSource !== "safe" && !activeControlSocket) writeSafeControls();
    return;
  }

  for (let index = 0; index < 8; index += 1) {
    channels[index] = radioInput.channels[index];
  }
  writeControls(channels);
  controlsSource = "radio";
  broadcast({ type: "input", input: radioInput });
}, 25);

setInterval(() => {
  if (debugBytes >= maxDebugBytes) return;

  let firmware = null;
  firmware = readLatestFirmwareState();

  const record = {
    type: "sample",
    t_ms: Date.now() - debugStartedAt.getTime(),
    source: demoClientIsActive()
      ? "demo"
      : manualClientIsActive()
        ? "browser_override"
        : radioInput.connected
          ? "crsf_transmitter_hid"
          : "browser",
    hid: {
      connected: radioInput.connected,
      axes: radioInput.axes,
      channels_us: radioInput.channels,
      buttons: radioInput.buttonBits,
      battery_percent: radioInput.battery,
      updated_at: radioInput.updatedAt,
    },
    browser: browserInput,
    controls_us: [...channels],
    firmware,
  };
  const line = `${JSON.stringify(record)}\n`;
  appendFileSync(debugLogPath, line, "utf8");
  debugBytes += Buffer.byteLength(line);
}, 100);

sockets.on("connection", (socket) => {
  releaseClientControl(socket);
  socket.send(JSON.stringify({ type: "ready", channels, input: radioInput }));
  socket.send(JSON.stringify({ type: "live-adapter-list", adapters: liveAdapterList() }));
  socket.on("close", () => {
    releaseClientControl(socket);
    if (activeControlSocket === socket) activeControlSocket = null;
    removeLiveAdapter(socket);
  });
  socket.on("message", (payload) => {
    try {
      const message = JSON.parse(payload.toString());
      if (message.type === "heartbeat") {
        const sequence = Number(message.sequence);
        const clientSentAt = Number(message.clientSentAt);
        if (Number.isSafeInteger(sequence) && Number.isFinite(clientSentAt) && clientSentAt > 0) {
          socket.send(JSON.stringify({
            type: "heartbeat-ack",
            sequence,
            clientSentAt,
            serverAt: Date.now(),
          }));
        }
        return;
      }
      if (message.type === "live-adapter-announce") {
        if (Buffer.byteLength(payload) > 8 * 1024 || !validLiveAdapterAnnouncement(message)) return;
        const existing = liveAdapters.get(message.adapterId);
        if (existing && existing.socket !== socket && Date.now() - existing.receivedAt <= 5_000) return;
        if (socket.liveAdapterId && socket.liveAdapterId !== message.adapterId) {
          removeLiveAdapter(socket, "identity-changed");
        }
        socket.liveAdapterId = message.adapterId;
        const announcement = { ...message, serverReceivedAt: Date.now() };
        liveAdapters.set(message.adapterId, { socket, announcement, receivedAt: Date.now() });
        broadcast(announcement);
        return;
      }
      if (message.type === "live-connection-command") {
        if (Buffer.byteLength(payload) > 8 * 1024 || !validLiveConnectionCommand(message)) return;
        if (message.action === "discover") {
          for (const { socket: adapterSocket } of liveAdapters.values()) sendSocket(adapterSocket, message);
          sendSocket(socket, {
            type: "live-connection-ack",
            action: "discover",
            requestId: message.requestId,
            accepted: true,
          });
          return;
        }
        const record = liveAdapters.get(message.adapterId);
        if (!record || (message.action === "disconnect" && record.socket.liveSessionId !== message.sessionId)) {
          sendSocket(socket, {
            type: "live-connection-ack",
            action: message.action,
            requestId: message.requestId,
            accepted: false,
            adapterId: message.adapterId,
            sessionId: message.sessionId,
            reason: "The selected adapter or session is no longer available.",
          });
          return;
        }
        rememberAdapterRequest(record.socket, message, "connection");
        sendSocket(record.socket, message);
        return;
      }
      if (message.type === "live-connection-ack") {
        if (
          Buffer.byteLength(payload) > 8 * 1024 ||
          !validLiveConnectionAcknowledgement(message) ||
          !socket.liveAdapterId ||
          (message.adapterId && message.adapterId !== socket.liveAdapterId) ||
          !consumeAdapterRequest(socket, message, "connection")
        ) return;
        if (message.action === "connect" && message.accepted) socket.liveSessionId = message.sessionId;
        if (
          message.action === "disconnect" &&
          message.accepted &&
          message.sessionId === socket.liveSessionId
        ) socket.liveSessionId = "";
        if (message.action === "disconnect" && message.accepted) socket.liveManualAuthority = null;
        broadcast(message);
        return;
      }
      if (message.type === "live-telemetry") {
        // Only a registered adapter may publish telemetry, and every packet is
        // bound to the engineering session negotiated by the browser.
        const record = liveAdapterForSession(message);
        if (record?.socket === socket && Buffer.byteLength(payload) <= 64 * 1024) broadcast(message);
        return;
      }
      if (message.type === "live-calibration-command") {
        // Calibration commands are relayed only as the canonical, bounded
        // browser-to-adapter contract. The robot adapter must refuse motion
        // until it has entered bench mode and must enforce the supplied speed
        // and jog limits independently of this browser.
        const record = liveAdapterForSession(message);
        if (
          Buffer.byteLength(payload) <= 32 * 1024 &&
          validCalibrationCommand(message) &&
          record
        ) {
          rememberAdapterRequest(record.socket, message, "calibration");
          sendSocket(record.socket, message);
        }
        return;
      }
      if (message.type === "live-calibration-ack") {
        // A future wired/Wi-Fi/Bluetooth adapter publishes this response after
        // robot-side bench-mode and persistence checks have actually passed.
        const record = liveAdapterForSession(message);
        if (
          Buffer.byteLength(payload) <= 8 * 1024 &&
          validCalibrationAcknowledgement(message) &&
          record?.socket === socket &&
          consumeAdapterRequest(socket, message, "calibration")
        ) broadcast(message);
        return;
      }
      if (message.type === "live-gait-command") {
        // The browser only publishes versioned profiles after local preview.
        // A robot adapter must independently require DISARMED state, validate
        // every parameter, store atomically, and acknowledge the applied hash.
        const record = liveAdapterForSession(message);
        if (Buffer.byteLength(payload) <= 16 * 1024 && validLiveGaitCommand(message) && record) {
          rememberAdapterRequest(record.socket, message, "gait");
          sendSocket(record.socket, message);
        }
        return;
      }
      if (message.type === "live-gait-ack") {
        const record = liveAdapterForSession(message);
        if (
          Buffer.byteLength(payload) <= 16 * 1024 &&
          validLiveGaitAcknowledgement(message) &&
          record?.socket === socket &&
          consumeAdapterRequest(socket, message, "gait")
        ) {
          broadcast(message);
        }
        return;
      }
      if (message.type === "live-safety-command") {
        const record = liveAdapterForSession(message);
        if (Buffer.byteLength(payload) <= 8 * 1024 && validLiveSafetyCommand(message) && record) {
          rememberAdapterRequest(record.socket, message, "safety");
          sendSocket(record.socket, message);
        }
        return;
      }
      if (message.type === "live-safety-ack") {
        const record = liveAdapterForSession(message);
        if (
          Buffer.byteLength(payload) <= 8 * 1024 &&
          validLiveSafetyAcknowledgement(message) &&
          record?.socket === socket &&
          consumeAdapterRequest(socket, message, "safety")
        ) broadcast(message);
        return;
      }
      if (message.type === "live-safety-heartbeat") {
        const record = liveAdapterForSession(message);
        if (Buffer.byteLength(payload) <= 4 * 1024 && validLiveSafetyHeartbeat(message) && record) {
          sendSocket(record.socket, message);
        }
        return;
      }
      if (message.type === "live-safety-heartbeat-ack") {
        const record = liveAdapterForSession(message);
        if (
          Buffer.byteLength(payload) <= 4 * 1024 &&
          validLiveSafetyHeartbeatAcknowledgement(message) &&
          record?.socket === socket
        ) broadcast(message);
        return;
      }
      if (message.type === "live-manual-authority-command") {
        const record = liveAdapterForSession(message);
        if (
          Buffer.byteLength(payload) <= 8 * 1024 &&
          validLiveManualAuthorityCommand(message) &&
          record &&
          (message.action !== "release-authority" || record.socket.liveManualAuthority?.token === message.authorityToken)
        ) {
          rememberAdapterRequest(record.socket, message, "manual-authority");
          sendSocket(record.socket, message);
        }
        return;
      }
      if (message.type === "live-manual-authority-ack") {
        const record = liveAdapterForSession(message);
        if (
          Buffer.byteLength(payload) <= 8 * 1024 &&
          validLiveManualAuthorityAcknowledgement(message) &&
          record?.socket === socket &&
          consumeAdapterRequest(socket, message, "manual-authority")
        ) {
          if (message.action === "request-authority" && message.accepted) {
            socket.liveManualAuthority = {
              token: message.authorityToken,
              sessionId: message.sessionId,
              expiresAt: Date.now() + message.leaseMs,
            };
          }
          if (message.action === "release-authority" && message.accepted) socket.liveManualAuthority = null;
          broadcast(message);
        }
        return;
      }
      if (message.type === "live-manual-control-frame") {
        const record = liveAdapterForSession(message);
        const authority = record?.socket.liveManualAuthority;
        if (
          Buffer.byteLength(payload) <= 8 * 1024 &&
          validLiveManualControlFrame(message) &&
          record &&
          authority?.token === message.authorityToken &&
          authority.sessionId === message.sessionId &&
          Date.now() < authority.expiresAt
        ) sendSocket(record.socket, message);
        return;
      }
      if (message.type !== "control" || !Array.isArray(message.channels) || message.channels.length !== 16) {
        return;
      }
      // Multiple open sandbox tabs used to race on controls.txt. Only the
      // visible tab may own the bridge; hidden tabs continue receiving state
      // but cannot overwrite the active tab's RC command.
      if (message.active !== true) {
        if (activeControlSocket === socket) activeControlSocket = null;
        releaseClientControl(socket);
        return;
      }
      if (activeControlSocket && activeControlSocket !== socket) {
        if (message.claimControl !== true) return;
        activeControlSocket = socket;
      }
      activeControlSocket = socket;
      socket.controlActive = true;
      socket.lastControlAt = Date.now();
      socket.controlMode = message.mode;
      socket.manualOverride = message.manualOverride === true;
      if (message.clientInput && typeof message.clientInput === "object") {
        browserInput = {
          source: String(message.clientInput.source || "unknown").slice(0, 32),
          name: String(message.clientInput.name || "").slice(0, 160),
          axes: Array.isArray(message.clientInput.axes)
            ? message.clientInput.axes.slice(0, 16).map((value) =>
              Number.isFinite(Number(value)) ? Number(value) : 0)
            : [],
          physics: message.physics && typeof message.physics === "object"
            ? {
                engine: String(message.physics.engine || "").slice(0, 32),
                body_height_m: Number(message.physics.bodyHeight) || 0,
                contact_count: Number(message.physics.contactCount) || 0,
                foot_contacts: Array.isArray(message.physics.footContacts)
                  ? message.physics.footContacts.slice(0, 4).map(Boolean)
                  : [],
                reset_count: Number(message.physics.resetCount) || 0,
              }
            : null,
        };
      }
      if (radioInputIsFresh(radioInput) && message.mode !== "demo" && !socket.manualOverride) return;
      for (let index = 0; index < 16; index += 1) {
        channels[index] = message.channels[index];
      }
      writeControls(channels);
      controlsSource = "browser";
    } catch {
      // Ignore malformed local control packets.
    }
  });
});

server.listen(port, "127.0.0.1", () => {
  console.log(`Domino Virtual Lab: http://127.0.0.1:${port}`);
  console.log(`CAD source: ${cadRoot}`);
  console.log(`Control file: ${controlPath}`);
  console.log("Debug log: /runtime/debug/latest.jsonl");
});

function shutdown() {
  radio.stop();
  server.close();
}

process.once("SIGINT", shutdown);
process.once("SIGTERM", shutdown);
