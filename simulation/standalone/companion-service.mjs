import { spawn } from "node:child_process";
import path from "node:path";
import process from "node:process";

const PORT_QUERY = [
  "-NoProfile",
  "-NonInteractive",
  "-Command",
  "Get-CimInstance Win32_SerialPort | Select-Object DeviceID,Description,PNPDeviceID | ConvertTo-Json -Compress",
];

function normalizedPort(port) {
  return {
    port: String(port?.port || port?.DeviceID || "").trim().toUpperCase(),
    description: String(port?.description || port?.Description || "Serial device").trim(),
    hwid: String(port?.hwid || port?.PNPDeviceID || "").trim(),
  };
}

export function selectCompanionPort(ports, transport = "auto", explicitPort = "") {
  const normalized = ports.map(normalizedPort).filter((port) => /^COM\d+$/i.test(port.port));
  if (explicitPort) {
    const requested = explicitPort.trim().toUpperCase();
    return normalized.find((port) => port.port === requested) || null;
  }
  const bluetooth = transport === "bluetooth";
  const ranked = normalized
    .map((port) => {
      const identity = `${port.description} ${port.hwid}`;
      let score = 0;
      if (/esp32|cp210|ch340|usb.*serial|uart|silicon labs/i.test(identity)) score += 100;
      if (/bluetooth|standard serial over bluetooth/i.test(identity)) score += bluetooth ? 120 : -80;
      if (/radiomaster|edge\s*tx|open\s*tx/i.test(identity)) score -= 120;
      return { port, score };
    })
    .filter(({ port }) => transport !== "usb" || !/bluetooth/i.test(`${port.description} ${port.hwid}`))
    .sort((left, right) => right.score - left.score || left.port.port.localeCompare(right.port.port));
  return ranked[0]?.port || null;
}

function windowsSerialPorts() {
  if (process.platform !== "win32") return Promise.resolve([]);
  return new Promise((resolve) => {
    const child = spawn("powershell.exe", PORT_QUERY, {
      windowsHide: true,
      stdio: ["ignore", "pipe", "ignore"],
    });
    let output = "";
    child.stdout.on("data", (chunk) => { output += chunk; });
    child.once("error", () => resolve([]));
    child.once("close", () => {
      try {
        const parsed = JSON.parse(output || "[]");
        resolve((Array.isArray(parsed) ? parsed : [parsed]).map(normalizedPort));
      } catch {
        resolve([]);
      }
    });
  });
}

export class CompanionService {
  constructor({
    firmwareService,
    relayUrl,
    adapterEntry,
    ardupilotEntry = path.join(path.dirname(adapterEntry), "ardupilot-companion-adapter.mjs"),
    protocol = process.env.DOMINO_COMPANION_PROTOCOL || "domino",
    logger = console,
  }) {
    this.firmwareService = firmwareService;
    this.relayUrl = relayUrl;
    this.adapterEntry = adapterEntry;
    this.ardupilotEntry = ardupilotEntry;
    this.protocol = protocol === "ardupilot" ? "ardupilot" : "domino";
    this.logger = logger;
    this.child = null;
    this.endpoint = "";
    this.transport = "";
    this.logs = [];
  }

  status() {
    return {
      running: Boolean(this.child && this.child.exitCode === null),
      protocol: this.protocol,
      endpoint: this.endpoint,
      transport: this.transport,
      logs: this.logs.slice(-30),
    };
  }

  record(stream, chunk) {
    const text = String(chunk).trim();
    if (!text) return;
    this.logs.push({ at: new Date().toISOString(), stream, text });
    if (this.logs.length > 200) this.logs.shift();
    this.logger[stream === "stderr" ? "warn" : "log"](`[companion] ${text}`);
  }

  async ports() {
    const platformioPorts = await this.firmwareService.ports();
    if (platformioPorts.length) return platformioPorts.map(normalizedPort);
    return windowsSerialPorts();
  }

  async discover(transport = "auto", options = {}) {
    const requestedTransport = ["auto", "usb", "bluetooth", "wifi"].includes(transport)
      ? transport
      : "auto";
    const protocol = options.protocol ?? this.protocol;
    if (!["domino", "ardupilot"].includes(protocol)) throw new Error("Unknown companion protocol.");
    if (protocol === "ardupilot") {
      if (requestedTransport === "usb" || requestedTransport === "bluetooth") {
        return {
          ...this.status(),
          started: false,
          reason: "The ArduPilot companion uses MAVLink UDP; choose AUTO or WI-FI.",
        };
      }
      const host = String(
        options.host || process.env.DOMINO_ARDUPILOT_HOST || process.env.DOMINO_ROBOT_WIFI_HOST || "127.0.0.1",
      ).trim();
      const port = Number(options.port ?? process.env.DOMINO_ARDUPILOT_PORT ?? 14550);
      if (!/^[a-z0-9.-]{1,253}$/i.test(host) || host.includes("..") || host.startsWith("-") || !Number.isInteger(port) || port < 1 || port > 65535) {
        throw new Error("Enter a valid MAVLink host and UDP port (1–65535).");
      }
      if (protocol !== this.protocol) {
        this.stop();
        this.protocol = protocol;
      }
      return this.start("wifi", `${host}:${port}`);
    }
    if (protocol !== this.protocol) {
      this.stop();
      this.protocol = protocol;
    }
    if (requestedTransport === "wifi") {
      const host = String(process.env.DOMINO_ROBOT_WIFI_HOST || "").trim();
      if (!host) return { ...this.status(), started: false, reason: "Set a Wi-Fi robot host before discovery." };
      return this.start("wifi", `${host}:${process.env.DOMINO_ROBOT_WIFI_PORT || 8766}`);
    }

    const effectiveTransport = requestedTransport === "bluetooth" ? "bluetooth" : "usb";
    const explicitPort = effectiveTransport === "bluetooth"
      ? process.env.DOMINO_ROBOT_BLUETOOTH_PORT
      : process.env.DOMINO_ROBOT_USB_PORT;
    const ports = await this.ports();
    const selected = selectCompanionPort(ports, effectiveTransport, explicitPort || "");
    if (!selected) return { ...this.status(), started: false, ports, reason: "No compatible serial robot link was found." };
    return { ...(await this.start(effectiveTransport, selected.port)), ports };
  }

  start(transport, endpoint) {
    if (this.child?.exitCode === null && this.endpoint === endpoint && this.transport === transport) {
      return { ...this.status(), started: false, reused: true };
    }
    this.stop();
    const ardupilot = this.protocol === "ardupilot";
    const entry = ardupilot ? this.ardupilotEntry : this.adapterEntry;
    const args = [
      entry,
      "--transport", transport,
      "--relay", this.relayUrl,
      "--adapter-id", "domino-physical-1",
    ];
    if (ardupilot) {
      const separator = endpoint.lastIndexOf(":");
      const host = separator > 0 ? endpoint.slice(0, separator) : endpoint;
      const port = separator > 0 ? endpoint.slice(separator + 1) : "14550";
      args.push("--mavlink-host", host, "--mavlink-port", port);
    } else {
      args.push("--robot-id", "domino-1");
    }
    if (!ardupilot && transport === "wifi") {
      const [host, port = "8766"] = endpoint.split(":");
      args.push("--robot-host", host, "--robot-port", port);
    } else if (!ardupilot) {
      args.push("--device", endpoint, "--baud", "460800");
    }
    const childEnvironment = { ...process.env };
    if (process.versions.electron) childEnvironment.ELECTRON_RUN_AS_NODE = "1";
    else delete childEnvironment.ELECTRON_RUN_AS_NODE;
    const child = spawn(process.execPath, args, {
      cwd: path.dirname(entry),
      windowsHide: true,
      stdio: ["ignore", "pipe", "pipe"],
      env: childEnvironment,
    });
    this.child = child;
    this.endpoint = endpoint;
    this.transport = transport;
    child.stdout.on("data", (chunk) => this.record("stdout", chunk));
    child.stderr.on("data", (chunk) => this.record("stderr", chunk));
    child.once("error", (error) => this.record("stderr", error.message));
    child.once("exit", (code) => {
      this.record(code === 0 ? "stdout" : "stderr", `Companion stopped with code ${code}.`);
      if (this.child === child) this.child = null;
    });
    return { ...this.status(), started: true };
  }

  stop() {
    const child = this.child;
    this.child = null;
    if (child?.exitCode === null) child.kill("SIGTERM");
  }
}
