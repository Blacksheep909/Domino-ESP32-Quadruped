import { HID, devices } from "node-hid";

export const BOXER_VENDOR_ID = 0x1209;
export const BOXER_PRODUCT_ID = 0x4f54;

const AXIS_COUNT = 8;
const AXIS_MAX = 2048;
const REPORT_BYTES_WITHOUT_BATTERY = 19;

function decodeCandidate(report, offset, layout) {
  if (report.length < offset + REPORT_BYTES_WITHOUT_BATTERY) return null;

  const axesOffset = layout === "buttons-first" ? offset + 3 : offset;
  const buttonsOffset = layout === "buttons-first" ? offset : offset + 16;
  if (report.length < axesOffset + AXIS_COUNT * 2 || report.length < buttonsOffset + 3) return null;

  const axes = Array.from(
    { length: AXIS_COUNT },
    (_, index) => report.readUInt16LE(axesOffset + index * 2),
  );
  const validAxes = axes.filter((value) => value >= 0 && value <= AXIS_MAX).length;
  const buttonBytes = [...report.subarray(buttonsOffset, buttonsOffset + 3)];
  const batteryOffset = Math.max(axesOffset + AXIS_COUNT * 2, buttonsOffset + 3);

  return {
    axes,
    battery: report.length > batteryOffset ? report[batteryOffset] : null,
    buttonBits: buttonBytes[0] | (buttonBytes[1] << 8) | (buttonBytes[2] << 16),
    buttonBytes,
    layout,
    offset,
    score: validAxes * 100 + (layout === "buttons-first" ? 1 : 0),
    validAxes,
  };
}

export function rawAxisToMicroseconds(value) {
  return Math.max(1000, Math.min(2000, Math.round(1000 + (value / AXIS_MAX) * 1000)));
}

export function decodeBoxerReport(report) {
  if (!Buffer.isBuffer(report)) report = Buffer.from(report);

  const candidates = [];
  for (const offset of [0, 1]) {
    for (const layout of ["buttons-first", "axes-first"]) {
      const candidate = decodeCandidate(report, offset, layout);
      if (candidate) candidates.push(candidate);
    }
  }

  candidates.sort((left, right) => right.score - left.score);
  const decoded = candidates[0];
  if (!decoded || decoded.validAxes !== AXIS_COUNT) return null;

  return {
    ...decoded,
    channels: decoded.axes.map(rawAxisToMicroseconds),
  };
}

export class BoxerHidInput {
  constructor(onState, logger = console) {
    this.onState = onState;
    this.logger = logger;
    this.device = null;
    this.scanTimer = null;
    this.state = {
      connected: false,
      name: "BOXER NOT FOUND",
      channels: null,
      axes: null,
      battery: null,
      buttonBits: 0,
      layout: null,
      updatedAt: 0,
    };
  }

  publish(patch) {
    this.state = { ...this.state, ...patch };
    this.onState(this.state);
  }

  connect() {
    if (this.device) return;

    let info;
    try {
      info = devices().find(
        (device) =>
          device.vendorId === BOXER_VENDOR_ID &&
          device.productId === BOXER_PRODUCT_ID &&
          device.usagePage === 1 &&
          device.usage === 5,
      );
    } catch (error) {
      this.logger.warn(`Boxer HID scan failed: ${error.message}`);
      return;
    }

    if (!info) {
      if (this.state.connected) {
        this.publish({ connected: false, name: "BOXER NOT FOUND", channels: null, updatedAt: Date.now() });
      }
      return;
    }

    try {
      this.device = new HID(info.path);
    } catch (error) {
      this.logger.warn(`Boxer HID open failed: ${error.message}`);
      this.device = null;
      return;
    }

    this.logger.log(`Boxer HID connected: ${info.product}`);
    this.publish({ connected: true, name: info.product || "RADIOMASTER BOXER", updatedAt: Date.now() });

    this.device.on("data", (report) => {
      const decoded = decodeBoxerReport(report);
      if (!decoded) return;
      this.publish({
        connected: true,
        name: info.product || "RADIOMASTER BOXER",
        channels: decoded.channels,
        axes: decoded.axes,
        battery: decoded.battery,
        buttonBits: decoded.buttonBits,
        layout: decoded.layout,
        updatedAt: Date.now(),
      });
    });
    this.device.on("error", (error) => {
      this.logger.warn(`Boxer HID disconnected: ${error.message}`);
      this.disconnect();
    });
  }

  disconnect() {
    const device = this.device;
    this.device = null;
    if (device) {
      try {
        device.close();
      } catch {
        // The OS may already have closed a removed USB device.
      }
    }
    this.publish({ connected: false, name: "BOXER NOT FOUND", channels: null, updatedAt: Date.now() });
  }

  start() {
    this.connect();
    this.scanTimer = setInterval(() => this.connect(), 1000);
  }

  stop() {
    if (this.scanTimer) clearInterval(this.scanTimer);
    this.scanTimer = null;
    this.disconnect();
  }
}
