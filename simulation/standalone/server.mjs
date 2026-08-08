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

import { BoxerHidInput } from "./boxer-hid.mjs";

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

mkdirSync(runtimeRoot, { recursive: true });
writeFileSync(
  debugLogPath,
  `${JSON.stringify({ type: "session", started_at: debugStartedAt.toISOString(), format: 1 })}\n`,
  "utf8",
);
let debugBytes = statSync(debugLogPath).size;

const channels = Array(16).fill(1500);
channels[4] = 1000;
// CRSF CH3 is the Boxer left-stick vertical and owns ride-height selection.
channels[2] = 1000;
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
  name: "BOXER NOT FOUND",
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
  return candidate.startsWith(path.resolve(root)) ? candidate : null;
}

const server = createServer((request, response) => {
  const url = new URL(request.url || "/", `http://${request.headers.host || "127.0.0.1"}`);
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

function broadcast(message) {
  const payload = JSON.stringify(message);
  for (const socket of sockets.clients) {
    if (socket.readyState === socket.OPEN) socket.send(payload);
  }
}

function demoClientIsActive() {
  return [...sockets.clients].some(
    (socket) => socket.readyState === socket.OPEN && socket.controlMode === "demo",
  );
}

function manualClientIsActive() {
  return [...sockets.clients].some(
    (socket) => socket.readyState === socket.OPEN && socket.manualOverride === true,
  );
}

const boxer = new BoxerHidInput((state) => {
  const connectionChanged = state.connected !== radioInput.connected;
  radioInput = state;
  if (connectionChanged) broadcast({ type: "input", input: radioInput });
});
boxer.start();

setInterval(() => {
  if (demoClientIsActive() || manualClientIsActive()) return;
  const inputIsFresh =
    radioInput.connected &&
    radioInput.channels &&
    Date.now() - radioInput.updatedAt < 250;
  if (!inputIsFresh) return;

  for (let index = 0; index < 8; index += 1) {
    channels[index] = radioInput.channels[index];
  }
  writeControls(channels);
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
          ? "boxer_hid"
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
  socket.send(JSON.stringify({ type: "ready", channels, input: radioInput }));
  socket.on("close", () => {
    if (activeControlSocket === socket) activeControlSocket = null;
  });
  socket.on("message", (payload) => {
    try {
      const message = JSON.parse(payload.toString());
      if (message.type !== "control" || !Array.isArray(message.channels) || message.channels.length !== 16) {
        return;
      }
      // Multiple open sandbox tabs used to race on controls.txt. Only the
      // visible tab may own the bridge; hidden tabs continue receiving state
      // but cannot overwrite the active tab's RC command.
      if (message.active !== true) {
        if (activeControlSocket === socket) activeControlSocket = null;
        return;
      }
      if (activeControlSocket && activeControlSocket !== socket) {
        if (message.claimControl !== true) return;
        activeControlSocket = socket;
      }
      activeControlSocket = socket;
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
      if (radioInput.connected && message.mode !== "demo" && !socket.manualOverride) return;
      for (let index = 0; index < 16; index += 1) {
        channels[index] = message.channels[index];
      }
      writeControls(channels);
    } catch {
      // Ignore malformed local control packets.
    }
  });
});

server.listen(port, "127.0.0.1", () => {
  console.log(`Domino standalone simulator: http://127.0.0.1:${port}`);
  console.log(`CAD source: ${cadRoot}`);
  console.log(`Control file: ${controlPath}`);
  console.log("Debug log: /runtime/debug/latest.jsonl");
});

function shutdown() {
  boxer.stop();
  server.close();
}

process.once("SIGINT", shutdown);
process.once("SIGTERM", shutdown);
