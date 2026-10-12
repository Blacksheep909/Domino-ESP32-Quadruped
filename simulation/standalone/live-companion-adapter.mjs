import { createReadStream, createWriteStream, openSync } from "node:fs";
import { spawn, spawnSync } from "node:child_process";
import net from "node:net";
import path from "node:path";
import process from "node:process";
import { fileURLToPath } from "node:url";
import { WebSocket } from "ws";

import {
  COMPANION_ANNOUNCE_INTERVAL_MS,
  LiveCompanionCore,
} from "./live-companion-core.mjs";

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

function lineReader(stream, onMessage) {
  let buffer = "";
  stream.setEncoding("utf8");
  stream.on("data", (chunk) => {
    buffer += chunk;
    while (buffer.includes("\n")) {
      const newline = buffer.indexOf("\n");
      const line = buffer.slice(0, newline).trim();
      buffer = buffer.slice(newline + 1);
      if (!line.startsWith("{")) continue;
      try { onMessage(JSON.parse(line)); } catch { /* Ignore malformed physical-link lines. */ }
    }
    if (buffer.length > 64 * 1024) buffer = "";
  });
}

function configureSerialDevice(device, baud) {
  const numericBaud = Number(baud || 460800);
  if (!Number.isSafeInteger(numericBaud) || numericBaud < 9_600 || numericBaud > 921_600) {
    throw new Error(`Invalid serial baud rate: ${baud}`);
  }
  const command = process.platform === "win32"
    ? { file: "mode.com", args: [`${device.toUpperCase()}:`, `BAUD=${numericBaud}`, "PARITY=N", "DATA=8", "STOP=1"] }
    : { file: "stty", args: [process.platform === "darwin" ? "-f" : "-F", device, String(numericBaud), "cs8", "-cstopb", "-parenb", "raw", "-echo"] };
  const result = spawnSync(command.file, command.args, { stdio: "ignore", windowsHide: true });
  if (result.error || result.status !== 0) {
    throw new Error(`Could not configure ${device} for ${numericBaud} baud, 8-N-1.`);
  }
}

function openWindowsSerialLink(device, baud, onMessage, onClose) {
  const helper = path.join(path.dirname(fileURLToPath(import.meta.url)), "windows-serial-bridge.ps1");
  const child = spawn(
    "powershell.exe",
    ["-NoLogo", "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", helper,
      "-Device", device.toUpperCase(), "-Baud", String(baud)],
    { windowsHide: true, stdio: ["pipe", "pipe", "pipe"] },
  );
  let closed = false;
  const closeOnce = () => {
    if (closed) return;
    closed = true;
    onClose();
  };
  lineReader(child.stdout, onMessage);
  child.stderr.setEncoding("utf8");
  child.stderr.on("data", (chunk) => {
    const message = String(chunk).trim();
    if (message) console.error(`Robot USB link: ${message}`);
  });
  child.on("error", (error) => console.error(`Robot USB bridge: ${error.message}`));
  child.on("close", closeOnce);
  return {
    write: (message) => child.stdin.writable && child.stdin.write(`${JSON.stringify(message)}\n`),
    close: () => {
      child.stdin.end();
      child.kill();
    },
  };
}

function openRobotLink(options, onMessage, onClose) {
  if (options["robot-host"]) {
    const socket = net.createConnection({ host: options["robot-host"], port: Number(options["robot-port"] || 8766) });
    socket.on("connect", () => {
      if (options.robotLinkKey) socket.write(`${JSON.stringify({
        type: "companion-auth", protocol: "domino-robot-link-v1", linkKey: options.robotLinkKey,
      })}\n`);
    });
    lineReader(socket, onMessage);
    socket.on("close", onClose);
    socket.on("error", (error) => console.error(`Robot TCP link: ${error.message}`));
    return { write: (message) => socket.writable && socket.write(`${JSON.stringify(message)}\n`), close: () => socket.destroy() };
  }
  if (options.device) {
    configureSerialDevice(options.device, options.baud);
    if (process.platform === "win32") {
      return openWindowsSerialLink(options.device, Number(options.baud || 460800), onMessage, onClose);
    }
    const device = process.platform === "win32" && /^COM\d+$/i.test(options.device)
      ? `\\\\.\\${options.device.toUpperCase()}`
      : options.device;
    const fd = openSync(device, "r+");
    const input = createReadStream(null, { fd, autoClose: false });
    const output = createWriteStream(null, { fd, autoClose: false });
    if (options.robotLinkKey) output.write(`${JSON.stringify({
      type: "companion-auth", protocol: "domino-robot-link-v1", linkKey: options.robotLinkKey,
    })}\n`);
    lineReader(input, onMessage);
    input.on("close", onClose);
    input.on("error", (error) => console.error(`Robot USB link: ${error.message}`));
    return { write: (message) => output.write(`${JSON.stringify(message)}\n`), close: () => { input.destroy(); output.destroy(); } };
  }
  throw new Error("Specify --robot-host HOST [--robot-port 8766] for Wi-Fi, or --device COM5 for USB.");
}

const options = optionsFromArgs(process.argv.slice(2));
if (options.help) {
  console.log(`Domino LIVE companion adapter

Wi-Fi:     node live-companion-adapter.mjs --robot-host 192.168.4.1 [--robot-port 8766]
USB:       node live-companion-adapter.mjs --device COM5 [--baud 460800]
Bluetooth: node live-companion-adapter.mjs --transport bluetooth --device COM7 [--baud 460800]

Wireless links require the DOMINO_ROBOT_LINK_KEY environment variable (16+ characters).

Optional: --relay ws://127.0.0.1:8770/control --adapter-id domino-physical-1 --robot-id domino-1`);
  process.exit(0);
}
const relayUrl = options.relay || "ws://127.0.0.1:8770/control";
const transport = options.transport || (options.device ? "usb" : "wifi");
if (!["wifi", "bluetooth", "usb"].includes(transport)) throw new Error(`Unsupported transport: ${transport}`);
const robotLinkKey = process.env.DOMINO_ROBOT_LINK_KEY || options["robot-key"] || "";
if (transport !== "usb" && robotLinkKey.length < 16) {
  throw new Error("Wireless robot links require DOMINO_ROBOT_LINK_KEY with at least 16 characters.");
}
options.robotLinkKey = robotLinkKey;
const endpoint = options.device || `${options["robot-host"] || "unconfigured"}:${options["robot-port"] || 8766}`;
const core = new LiveCompanionCore({
  adapterId: options["adapter-id"] || "domino-physical-1",
  name: options.name || "Domino Physical Companion",
  transport,
  endpoint,
  robotId: options["robot-id"] || "domino-1",
});

let relay;
let robotLink;
let shuttingDown = false;

function writeRelay(message) {
  if (relay?.readyState === WebSocket.OPEN) relay.send(JSON.stringify(message));
}
function writeRobot(message) {
  if (message?.kind === "calibration") {
    console.log(`Calibration TX: ${message.action || "unknown"} / ${message.requestId || "no-request-id"}`);
  }
  robotLink?.write(robotLinkKey ? { ...message, linkKey: robotLinkKey } : message);
}
function dispatch(result) {
  result.relay.forEach(writeRelay);
  result.robot.forEach(writeRobot);
}

function connectRelay() {
  if (shuttingDown) return;
  relay = new WebSocket(relayUrl);
  relay.on("open", () => {
    console.log(`Companion connected to ${relayUrl}`);
    writeRelay(core.announcement());
  });
  relay.on("message", (payload) => {
    try {
      const message = JSON.parse(payload.toString());
      if (
        message?.type === "live-connection-command" &&
        message.action === "restart" &&
        message.adapterId === core.adapterId
      ) {
        restartRobotLink("browser-link-restart");
        writeRelay({
          type: "live-connection-ack",
          action: "restart",
          requestId: message.requestId,
          accepted: true,
          adapterId: core.adapterId,
        });
        return;
      }
      dispatch(core.handleRelay(message));
    } catch { /* Ignore malformed relay packets. */ }
  });
  relay.on("close", () => { if (!shuttingDown) setTimeout(connectRelay, 800); });
  relay.on("error", (error) => console.error(`Relay link: ${error.message}`));
}

function connectRobot() {
  if (shuttingDown || robotLink) return;
  try {
    let openedLink;
    openedLink = openRobotLink(
      options,
      (message) => {
        if (message?.type === "robot-ack" && message.kind === "calibration") {
          console.log(
            `Calibration RX: ${message.action || "unknown"} / ${message.requestId || "no-request-id"} / ` +
            `${message.accepted === true ? "accepted" : "rejected"}` +
            `${message.reason ? ` / ${message.reason}` : ""}`,
          );
        }
        dispatch(core.handleRobot(message));
      },
      () => {
        if (robotLink !== openedLink) return;
        robotLink = null;
        dispatch(core.disconnectRobot(Date.now(), "physical-link-closed"));
        if (!shuttingDown) setTimeout(connectRobot, 800);
      },
    );
    robotLink = openedLink;
    console.log(`Companion opening ${transport.toUpperCase()} robot link at ${endpoint}`);
  } catch (error) {
    console.error(`Robot link: ${error instanceof Error ? error.message : error}`);
    if (!shuttingDown) setTimeout(connectRobot, 800);
  }
}

function restartRobotLink(reason = "link-restart") {
  const previousLink = robotLink;
  robotLink = null;
  dispatch(core.disconnectRobot(Date.now(), reason));
  try { previousLink?.close(); } catch { /* Reopening below is the recovery path. */ }
  if (!shuttingDown) setTimeout(connectRobot, 150);
  console.log(`Companion restarting ${transport.toUpperCase()} robot link at ${endpoint}`);
}

connectRobot();
connectRelay();

setInterval(() => writeRelay(core.announcement()), COMPANION_ANNOUNCE_INTERVAL_MS);
setInterval(() => dispatch(core.tick()), 25);

function shutdown() {
  shuttingDown = true;
  dispatch(core.disconnectRobot(Date.now(), "companion-shutdown"));
  robotLink?.close();
  relay?.close();
  process.exit(0);
}
process.once("SIGINT", shutdown);
process.once("SIGTERM", shutdown);
