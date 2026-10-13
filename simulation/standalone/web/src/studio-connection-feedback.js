import { liveConnectionIsReady, visibleLiveAdapters } from "./live-connection-state.js";

export function studioConnectionFeedback(state, packetAgeMs = null, now = Date.now()) {
  const adapter = state?.adapters?.[state.selectedAdapterId];
  const name = adapter?.robot?.name || adapter?.name || "robot";
  const ready = liveConnectionIsReady(state, now);
  const base = { ready, name, error: state?.error || "", sessionId: ready ? state.sessionId : "",
    adapterCount: state ? visibleLiveAdapters(state, now).length : 0,
    status: state?.status || "", phase: state?.phase, action: ready ? "MANAGE LINK" : "PAIR ROBOT", tone: "offline" };
  if (state?.phase === "uploading") return { ...base, ready: false, code: "uploading", tone: "warning", title: "Firmware upload in progress", detail: "Robot pairing is paused until the uploader releases the port." };
  if (state?.phase === "releasing") return { ...base, ready: false, code: "releasing", tone: "warning", title: "Releasing robot port", detail: "Waiting for the companion to close." };
  if (ready) {
    const waiting = packetAgeMs === null;
    const stale = !waiting && packetAgeMs > 1000;
    return { ...base, code: waiting ? "waiting" : stale ? "stale" : "connected", tone: waiting || stale ? "warning" : "connected",
      title: `Connected to ${name}`,
      detail: waiting ? "PC paired · waiting for robot telemetry · motion needs a separate arm acknowledgement."
        : stale ? "PC paired · telemetry stale · fresh readings are required for physical commands."
          : `Telemetry fresh · robot reports ${state.robotState} · radio drive is a separate link.` };
  }
  if (state?.phase === "fault") return { ...base, code: "fault", tone: "fault", action: "REVIEW FAULT",
    title: "Robot connection fault", detail: state.error || state.status || "Open the manager to inspect the fault." };
  if (state?.phase === "reconnecting") return { ...base, code: "reconnecting", tone: "warning", action: "MANAGE RETRY",
    title: `Robot disconnected — retrying ${name}`, detail: "Commands blocked. Open the connection manager to inspect or cancel the retry." };
  if (state?.phase === "restarting") return { ...base, code: "restarting", tone: "warning", action: "VIEW LINK",
    title: "Robot link restarting", detail: "Waiting for the selected transport to restart. Commands remain blocked." };
  if (state?.phase === "connecting") return { ...base, code: "connecting", tone: "warning", action: "VIEW LINK",
    title: `Connecting to ${name}…`, detail: "Waiting for the robot handshake. Physical commands are still blocked." };
  if (state?.phase === "disconnecting") return { ...base, code: "disconnecting", tone: "warning", action: "VIEW LINK",
    title: "Disconnecting robot…", detail: "Waiting for the session to close." };
  if (state?.phase === "discovering") return { ...base, code: "discovering", tone: "warning",
    title: "Searching for robots…", detail: "Waiting for the search results." };
  return { ...base, code: state?.bridgeConnected ? "disconnected" : "bridge-offline", title: "Robot disconnected",
    detail: state?.bridgeConnected ? "PC bridge ready · pair a robot to read telemetry and unlock its tools."
      : "Studio's local bridge is offline. Waiting for it to reconnect; robot commands are blocked." };
}

export function studioConnectionNotice(previous, current) {
  if (!previous) return null;
  if (current.error && current.error !== previous.error && current.code !== "reconnecting") {
    return { tone: "error", message: current.error };
  }
  if (current.code !== previous.code) {
    const progress = {
      discovering: "Searching for robots…",
      connecting: `Connecting to ${current.name}…`,
      disconnecting: "Disconnecting robot…",
      restarting: "Restarting robot link…",
      reconnecting: `Connection lost. Reconnecting to ${current.name}…`,
      uploading: "Robot pairing paused for firmware upload…",
      releasing: "Disconnecting and releasing the robot port…",
    }[current.code];
    if (progress) return { tone: "pending", message: progress, durationMs: 0 };
  }
  if (previous.code === "discovering" && current.code !== "discovering") {
    return { tone: "info", message: current.adapterCount
      ? `Search complete. ${current.adapterCount} device${current.adapterCount === 1 ? "" : "s"} found.`
      : "Search complete. No devices found." };
  }
  if (current.ready && (!previous.ready || current.sessionId !== previous.sessionId)) {
    return { tone: "success", message: `${current.name} connected.${current.code === "waiting" ? " Waiting for telemetry." : ""}` };
  }
  if (current.code === "disconnected" && previous.code === "releasing") return { tone: "success", message: "Disconnected. Robot port available for upload." };
  if (current.code === "disconnected" && ["disconnecting", "connected", "waiting", "stale", "reconnecting", "restarting"].includes(previous.code)) {
    return { tone: "info", message: previous.code === "reconnecting" ? "Reconnect cancelled." : "Robot disconnected." };
  }
  if (current.code === "bridge-offline" && previous.code !== current.code) {
    return { tone: "warning", message: "Studio connection lost. Waiting for the local bridge." };
  }
  if (current.code === "stale" && previous.code !== "stale") return { tone: "warning", message: "Robot telemetry has stopped updating. Check the connection." };
  if (previous.code === "stale" && current.code === "connected") return { tone: "success", message: "Robot telemetry is updating again." };
  if (current.code === "fault" && previous.code !== "fault") return { tone: "error", message: current.detail };
  return null;
}
