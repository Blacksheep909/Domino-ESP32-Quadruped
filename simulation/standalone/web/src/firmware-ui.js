import { showStudioButtonResult } from "./studio-action-feedback.js";

const formatBytes = (bytes) => {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 ** 2) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / 1024 ** 2).toFixed(1)} MB`;
};

async function api(path, options) {
  const response = await fetch(path, {
    cache: "no-store",
    headers: options?.body ? { "Content-Type": "application/json" } : undefined,
    ...options,
  });
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || `Request failed (${response.status})`);
  return payload;
}

function formatLog(logs = []) {
  if (!logs.length) return "Waiting for a build or upload job.";
  return logs.map((entry) => {
    const time = new Date(entry.at).toLocaleTimeString([], { hour12: false });
    return `[${time}] ${entry.text.trimEnd()}`;
  }).join("\n");
}

export function initializeFirmwareWorkspace() {
  const dialog = document.querySelector("#firmware-deploy-dialog");
  const openButton = document.querySelector("#firmware-deploy-button");
  if (!dialog || !openButton) return;

  const elements = {
    state: document.querySelector("#firmware-package-state"),
    packageName: document.querySelector("#firmware-package-name"),
    packageSize: document.querySelector("#firmware-package-size"),
    board: document.querySelector("#firmware-board-name"),
    hash: document.querySelector("#firmware-package-hash"),
    fileList: document.querySelector("#firmware-file-list"),
    reviewPath: document.querySelector("#firmware-review-path"),
    reviewSize: document.querySelector("#firmware-review-size"),
    source: document.querySelector("#firmware-source-viewer code"),
    toolchain: document.querySelector("#firmware-toolchain-state"),
    build: document.querySelector("#firmware-build-button"),
    port: document.querySelector("#firmware-port-select"),
    refreshPorts: document.querySelector("#firmware-refresh-ports"),
    confirmation: document.querySelector("#firmware-boot-confirm"),
    upload: document.querySelector("#firmware-upload-button"),
    jobLabel: document.querySelector("#firmware-job-label"),
    jobStage: document.querySelector("#firmware-job-stage"),
    jobPercent: document.querySelector("#firmware-job-percent"),
    progress: document.querySelector("#firmware-progress-fill"),
    console: document.querySelector("#firmware-console"),
    cancel: document.querySelector("#firmware-cancel-job"),
    copy: document.querySelector("#firmware-copy-log"),
  };
  let latestStatus = null;
  let selectedFile = null;
  let pollTimer = null;
  let busy = false;
  let busyAction = "";

  const packageIsBuilt = () => Boolean(
    latestStatus?.lastBuild?.ok &&
    latestStatus.lastBuild.packageHash === latestStatus.package.hash,
  );

  function updateActionState() {
    const jobRunning = ["running", "cancelling"].includes(latestStatus?.job?.status);
    elements.build.disabled = busy || jobRunning || !latestStatus?.toolchain?.available;
    elements.upload.disabled = busy || jobRunning || !packageIsBuilt() || !elements.confirmation.checked;
    elements.cancel.disabled = !jobRunning;
    const building = busyAction === "build" || (jobRunning && latestStatus?.job?.type === "build");
    const uploading = busyAction === "upload" || (jobRunning && latestStatus?.job?.type === "upload");
    elements.build.textContent = building ? "BUILDING…" : "BUILD & VALIDATE";
    elements.upload.textContent = uploading ? "UPLOADING…" : "UPLOAD TO ESP32";
    elements.build.setAttribute("aria-busy", String(building));
    elements.upload.setAttribute("aria-busy", String(uploading));
  }

  function actionFeedback(message, state) {
    if (state === "error") elements.jobStage.textContent = message;
  }

  async function reviewFile(filePath) {
    selectedFile = filePath;
    elements.fileList.querySelectorAll("button").forEach((button) => {
      button.classList.toggle("active", button.dataset.path === filePath);
    });
    elements.reviewPath.textContent = filePath;
    elements.reviewSize.textContent = "LOADING";
    try {
      const file = await api(`/api/firmware/file?path=${encodeURIComponent(filePath)}`);
      if (selectedFile !== filePath) return;
      elements.reviewSize.textContent = formatBytes(file.size);
      elements.source.textContent = file.contents;
      elements.source.parentElement.scrollTop = 0;
      elements.source.parentElement.scrollLeft = 0;
    } catch (error) {
      elements.reviewSize.textContent = "ERROR";
      elements.source.textContent = error.message;
    }
  }

  function renderManifest(firmwarePackage) {
    elements.packageName.textContent = firmwarePackage.environment;
    elements.packageSize.textContent = `${firmwarePackage.files.length} FILES / ${formatBytes(firmwarePackage.bytes)}`;
    elements.board.textContent = `${firmwarePackage.board} / ${firmwarePackage.framework}`;
    elements.hash.textContent = `SHA256 ${firmwarePackage.hash.slice(0, 12)}`;
    const existingPaths = [...elements.fileList.querySelectorAll("button")].map((button) => button.dataset.path);
    const nextPaths = firmwarePackage.files.map((file) => file.path);
    if (existingPaths.join("|") === nextPaths.join("|")) return;
    elements.fileList.replaceChildren(...firmwarePackage.files.map((file) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "firmware-file-item";
      button.dataset.path = file.path;
      button.setAttribute("role", "option");
      const name = document.createElement("span");
      name.textContent = file.path;
      const size = document.createElement("small");
      size.textContent = formatBytes(file.size);
      button.append(name, size);
      button.addEventListener("click", () => reviewFile(file.path));
      return button;
    }));
    const firstPath = firmwarePackage.files.find((file) => file.path === "src/main.cpp")?.path
      || firmwarePackage.files[0]?.path;
    if (firstPath) reviewFile(firstPath);
  }

  function renderStatus(status) {
    latestStatus = status;
    renderManifest(status.package);
    elements.toolchain.textContent = status.toolchain.available ? "READY" : "NOT FOUND";
    elements.toolchain.style.color = status.toolchain.available ? "#7bd798" : "#ff897a";
    const job = status.job;
    if (job) {
      elements.jobLabel.textContent = job.type.toUpperCase();
      elements.jobStage.textContent = job.stage;
      elements.jobPercent.textContent = `${Math.round(job.progress)}%`;
      elements.progress.style.width = `${job.progress}%`;
      const log = formatLog(job.logs);
      if (elements.console.textContent !== log) {
        elements.console.textContent = log;
        elements.console.scrollTop = elements.console.scrollHeight;
      }
      elements.state.textContent = job.status === "success" ? "READY" : job.status.toUpperCase();
      elements.state.dataset.state = job.status;
    } else {
      elements.jobLabel.textContent = packageIsBuilt() ? "VALIDATED" : "READY";
      elements.jobStage.textContent = packageIsBuilt() ? "Current package is ready to upload" : "Build the current package before upload";
      elements.jobPercent.textContent = packageIsBuilt() ? "100%" : "0%";
      elements.progress.style.width = packageIsBuilt() ? "100%" : "0%";
      elements.state.textContent = status.toolchain.available ? "PACKAGE READY" : "TOOLCHAIN MISSING";
      elements.state.dataset.state = status.toolchain.available ? "ready" : "failed";
    }
    updateActionState();
  }

  async function refreshStatus() {
    try {
      renderStatus(await api("/api/firmware/status"));
    } catch (error) {
      elements.state.textContent = "SERVICE OFFLINE";
      elements.state.dataset.state = "failed";
      elements.jobStage.textContent = error.message;
    }
  }

  async function refreshPorts() {
    elements.refreshPorts.disabled = true;
    elements.refreshPorts.textContent = "SCANNING…";
    elements.refreshPorts.setAttribute("aria-busy", "true");
    actionFeedback("Searching for USB serial ports…", "waiting");
    const current = elements.port.value;
    try {
      const { ports } = await api("/api/firmware/ports");
      elements.port.replaceChildren(new Option("AUTO DETECT", ""), ...ports.map((device) =>
        new Option(`${device.port} / ${device.description}`, device.port)));
      if ([...elements.port.options].some((option) => option.value === current)) elements.port.value = current;
      actionFeedback(ports.length ? `Port list refreshed. ${ports.length} serial port${ports.length === 1 ? "" : "s"} found.`
        : "Search finished. No USB serial ports found; connect the ESP32 and refresh again.", ports.length ? "success" : "info");
    } catch (error) {
      elements.jobStage.textContent = error.message;
      actionFeedback(`Port search failed: ${error.message}`, "error");
    } finally {
      elements.refreshPorts.disabled = false;
      elements.refreshPorts.textContent = "REFRESH PORTS";
      elements.refreshPorts.setAttribute("aria-busy", "false");
    }
  }

  async function runAction(path, body) {
    if (busy) return;
    busy = true;
    busyAction = path.split("/").at(-1);
    actionFeedback(busyAction === "cancel" ? "Requesting job cancellation…" : `Requesting firmware ${busyAction}…`, "waiting");
    updateActionState();
    try {
      await api(path, { method: "POST", body: body ? JSON.stringify(body) : undefined });
      await refreshStatus();
    } catch (error) {
      elements.jobStage.textContent = error.message;
      elements.state.textContent = "ACTION BLOCKED";
      elements.state.dataset.state = "failed";
      actionFeedback(`Firmware ${busyAction} could not start: ${error.message}`, "error");
    } finally {
      busy = false;
      busyAction = "";
      updateActionState();
    }
  }

  openButton.addEventListener("click", async () => {
    dialog.showModal();
    await Promise.all([refreshStatus(), refreshPorts()]);
    clearInterval(pollTimer);
    pollTimer = setInterval(refreshStatus, 700);
  });
  document.querySelector("#firmware-deploy-close").addEventListener("click", () => dialog.close());
  dialog.addEventListener("close", () => {
    clearInterval(pollTimer);
    pollTimer = null;
  });
  dialog.addEventListener("click", (event) => {
    if (event.target === dialog) dialog.close();
  });
  elements.build.addEventListener("click", () => runAction("/api/firmware/build"));
  elements.refreshPorts.addEventListener("click", refreshPorts);
  elements.confirmation.addEventListener("change", updateActionState);
  elements.upload.addEventListener("click", () => runAction("/api/firmware/upload", {
    confirmation: elements.confirmation.checked ? "BOOT HELD" : "",
    port: elements.port.value,
  }));
  elements.cancel.addEventListener("click", () => runAction("/api/firmware/cancel"));
  elements.copy.addEventListener("click", async () => {
    try {
      await navigator.clipboard.writeText(elements.console.textContent);
      showStudioButtonResult(elements.copy, "COPIED ✓");
    } catch {
      actionFeedback("The log could not be copied. Select the text in the log to copy it manually.", "error");
    }
  });
}
