// POLICY LAB: Isaac Lab training dashboard, ESP32 budget and deployment of the
// learned walking policy ("simulation mode", radio SB fully down).
//
// Talks to /api/policy/* (policy-lab-service.mjs). Live robot policy status
// arrives as `domino:live-telemetry` window events from main.js.

const POLL_MS = 2500;
const ESP32_FLASH_APP_BYTES = 3 * 1024 * 1024;
const ESP32_RAM_BYTES = 320 * 1024;
const CONTROL_BUDGET_US = 20000;

const CHARTS = [
  { key: "mean_reward", label: "Mean episode reward", digits: 2 },
  { key: "mean_episode_length", label: "Episode length (s)", digits: 1, scale: 0.02 },
  { key: "Metrics/base_velocity/error_vel_xy", label: "Velocity error (m/s)", digits: 3 },
  { key: "Metrics/body_height/error_height", label: "Height error (mm)", digits: 1, scale: 1000 },
  { key: "Curriculum/terrain_levels", label: "Terrain level", digits: 2 },
  { key: "noise_std", label: "Action noise std", digits: 3 },
];

const CONFIG_GROUPS = [
  { key: "commands", label: "Commands (remote ranges)" },
  { key: "domain_randomization", label: "Domain randomization" },
  { key: "robot", label: "Robot and servos" },
  { key: "terrain", label: "Terrain" },
  { key: "rewards", label: "Reward weights" },
  { key: "gait", label: "Gait clock" },
  { key: "training", label: "PPO training" },
];

const state = {
  status: null,
  selectedRun: null,
  metrics: new Map(), // run -> { next, records }
  config: null,
  configDirty: false,
  liveTelemetry: null,
  liveAt: 0,
  pollTimer: null,
  busy: false,
};

const $ = (selector, root = document) => root.querySelector(selector);

async function api(method, route, body) {
  const response = await fetch(`/api/policy/${route}`, {
    method,
    headers: body === undefined ? {} : { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.error || `Request failed (${response.status})`);
  return data;
}

function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (value === undefined || value === null || value === false) continue;
    if (key === "class") node.className = value;
    else if (key === "text") node.textContent = value;
    else if (key.startsWith("on")) node.addEventListener(key.slice(2), value);
    else node.setAttribute(key, value === true ? "" : String(value));
  }
  for (const child of children.flat()) {
    if (child === null || child === undefined || child === false) continue;
    node.append(child instanceof Node ? child : document.createTextNode(String(child)));
  }
  return node;
}

function fmt(value, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

function fmtBytes(bytes) {
  if (!Number.isFinite(bytes)) return "--";
  return bytes >= 1024 * 1024 ? `${(bytes / 1024 / 1024).toFixed(2)} MB` : `${(bytes / 1024).toFixed(1)} KB`;
}

function fmtAgo(seconds) {
  if (!Number.isFinite(seconds)) return "";
  const delta = Math.max(0, Date.now() / 1000 - seconds);
  if (delta < 90) return `${Math.round(delta)} s ago`;
  if (delta < 5400) return `${Math.round(delta / 60)} min ago`;
  if (delta < 172800) return `${Math.round(delta / 3600)} h ago`;
  return `${Math.round(delta / 86400)} d ago`;
}

function feedback(message, kind = "info") {
  const node = $("#policy-lab-feedback");
  node.textContent = message;
  node.dataset.kind = kind;
  node.hidden = !message;
}

async function guarded(action, label) {
  if (state.busy) return;
  state.busy = true;
  feedback(`${label}...`);
  try {
    const result = await action();
    feedback(`${label}: done.`, "ok");
    await refresh();
    return result;
  } catch (error) {
    feedback(error.message, "error");
  } finally {
    state.busy = false;
    render();
  }
}

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------
async function refresh() {
  try {
    state.status = await api("GET", "status");
  } catch (error) {
    state.status = { error: error.message, runs: [], settings: {} };
  }
  const runs = state.status.runs || [];
  if (!state.selectedRun || !runs.some((run) => run.name === state.selectedRun)) {
    state.selectedRun = runs[0]?.name || null;
  }
  if (state.selectedRun) await loadMetrics(state.selectedRun);
  if (state.status.job?.status === "running" || state.status.job?.status === "stopping") {
    try {
      const log = await api("GET", "log");
      state.logLines = log.lines;
    } catch {
      // keep the previous log
    }
  }
  render();
}

async function loadMetrics(run) {
  const cached = state.metrics.get(run) || { next: 0, records: [] };
  try {
    const page = await api("GET", `runs/${encodeURIComponent(run)}/metrics?from=${cached.next}`);
    if (page.from < cached.next) cached.records = []; // file was rewritten
    cached.records.push(...page.records);
    cached.next = page.next;
    state.metrics.set(run, cached);
  } catch {
    // run may have vanished; the next status refresh drops it
  }
}

async function loadConfig() {
  try {
    state.config = (await api("GET", "config")).config;
    state.configDirty = false;
  } catch (error) {
    state.config = null;
    feedback(error.message, "error");
  }
  renderConfig();
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
function statusPill(status) {
  const map = {
    training: "positive", starting: "positive", finished: "neutral", stopped: "warning",
    interrupted: "warning", failed: "error", running: "positive",
  };
  return el("span", { class: "policy-pill", "data-state": map[status] || "neutral", text: (status || "unknown").toUpperCase() });
}

function renderRuns() {
  const list = $("#policy-run-list");
  list.replaceChildren();
  const runs = state.status?.runs || [];
  if (!runs.length) {
    list.append(el("p", { class: "policy-empty", text: "No training runs yet. Start one below." }));
    return;
  }
  for (const run of runs) {
    const done = (run.iteration ?? 0) - (run.startIteration ?? 0);
    const pct = run.maxIterations ? Math.min(100, (100 * (done + 1)) / run.maxIterations) : 0;
    list.append(el("button", {
      type: "button",
      class: "policy-run",
      "aria-pressed": run.name === state.selectedRun ? "true" : "false",
      onclick: () => {
        state.selectedRun = run.name;
        loadMetrics(run.name).then(render);
        render();
      },
    },
    el("span", { class: "policy-run-top" }, el("strong", { text: run.name.replace(/^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}_/, "") }), statusPill(run.status)),
    el("span", { class: "policy-run-meta", text: `${run.started ? new Date(run.started * 1000).toLocaleString() : ""} · ${run.numEnvs ?? "?"} robots` }),
    el("span", { class: "policy-run-progress" }, el("i", { style: `width:${pct.toFixed(1)}%` })),
    el("span", { class: "policy-run-meta", text: `${run.startIteration ? `it ${run.iteration} (+${done + 1}/${run.maxIterations ?? "?"} from ${run.startIteration})` : `it ${run.iteration ?? 0}/${run.maxIterations ?? "?"}`} · reward ${fmt(run.meanReward)}${run.export ? ` · exported ${run.export.policy_id}` : ""}` })));
  }
}

function drawChart(canvas, points, { digits = 2 } = {}) {
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth || 260;
  const height = canvas.clientHeight || 120;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const styles = getComputedStyle(canvas);
  const line = styles.getPropertyValue("--policy-chart-line").trim() || "#2f507a";
  const grid = styles.getPropertyValue("--policy-chart-grid").trim() || "rgba(0,0,0,.08)";
  const text = styles.getPropertyValue("--policy-chart-text").trim() || "#666";
  const pad = { l: 44, r: 8, t: 8, b: 18 };
  if (points.length < 2) {
    ctx.fillStyle = text;
    ctx.font = "11px system-ui, sans-serif";
    ctx.fillText("Waiting for data", pad.l, height / 2);
    return;
  }
  const xs = points.map((p) => p[0]);
  const ys = points.map((p) => p[1]);
  let lo = Math.min(...ys);
  let hi = Math.max(...ys);
  if (hi - lo < 1e-9) { hi += 1; lo -= 1; }
  const x0 = xs[0];
  const x1 = xs[xs.length - 1] || x0 + 1;
  const px = (x) => pad.l + ((x - x0) / Math.max(1e-9, x1 - x0)) * (width - pad.l - pad.r);
  const py = (y) => pad.t + (1 - (y - lo) / (hi - lo)) * (height - pad.t - pad.b);
  ctx.strokeStyle = grid;
  ctx.lineWidth = 1;
  ctx.fillStyle = text;
  ctx.font = "10px system-ui, sans-serif";
  for (let i = 0; i <= 3; i += 1) {
    const value = lo + ((hi - lo) * i) / 3;
    const y = py(value);
    ctx.beginPath();
    ctx.moveTo(pad.l, y);
    ctx.lineTo(width - pad.r, y);
    ctx.stroke();
    ctx.fillText(value.toFixed(digits), 4, y + 3);
  }
  ctx.fillText(`it ${x1}`, width - pad.r - 40, height - 4);
  // Downsample to ~2 points per pixel.
  const step = Math.max(1, Math.floor(points.length / (width * 2)));
  ctx.strokeStyle = line;
  ctx.lineWidth = 1.6;
  ctx.beginPath();
  for (let i = 0; i < points.length; i += step) {
    const [x, y] = points[i];
    if (i === 0) ctx.moveTo(px(x), py(y));
    else ctx.lineTo(px(x), py(y));
  }
  const last = points[points.length - 1];
  ctx.lineTo(px(last[0]), py(last[1]));
  ctx.stroke();
}

function renderCharts() {
  const host = $("#policy-charts");
  const records = state.metrics.get(state.selectedRun)?.records || [];
  if (!host.childElementCount) {
    for (const chart of CHARTS) {
      host.append(el("figure", { class: "policy-chart", "data-key": chart.key },
        el("figcaption", {}, el("span", { text: chart.label }), el("output", { text: "--" })),
        el("canvas", { "aria-label": chart.label })));
    }
  }
  for (const chart of CHARTS) {
    const figure = host.querySelector(`[data-key="${CSS.escape(chart.key)}"]`);
    const points = records
      .filter((record) => Number.isFinite(record[chart.key]))
      .map((record) => [record.it, record[chart.key] * (chart.scale || 1)]);
    figure.querySelector("output").textContent = points.length ? fmt(points[points.length - 1][1], chart.digits) : "--";
    drawChart(figure.querySelector("canvas"), points, chart);
  }
  // Reward terms and terminations of the latest iteration.
  const last = records[records.length - 1] || {};
  const terms = Object.entries(last).filter(([key]) => key.startsWith("Episode_Reward/"))
    .map(([key, value]) => [key.slice("Episode_Reward/".length), value]);
  const maxAbs = Math.max(1e-6, ...terms.map(([, value]) => Math.abs(value)));
  $("#policy-reward-terms").replaceChildren(...terms.map(([name, value]) => el("div", { class: "policy-term" },
    el("span", { text: name.replaceAll("_", " ") }),
    el("i", { "data-sign": value >= 0 ? "pos" : "neg", style: `width:${(48 * Math.abs(value)) / maxAbs}%` }),
    el("output", { text: fmt(value, 3) }))));
  const terminations = Object.entries(last).filter(([key]) => key.startsWith("Episode_Termination/"));
  $("#policy-terminations").replaceChildren(...terminations.map(([key, value]) => el("span", { class: "policy-chip" },
    `${key.slice("Episode_Termination/".length).replaceAll("_", " ")} ${fmt(value * 100, 0)}%`)));
  $("#policy-throughput").textContent = last.fps
    ? `${Math.round(last.fps).toLocaleString()} steps/s · ${fmt((last.collection_s || 0) + (last.learn_s || 0), 2)} s/iteration`
    : "";
}

function budgetBar(label, used, total, unit) {
  const pct = Number.isFinite(used) && total ? Math.min(100, (100 * used) / total) : 0;
  return el("div", { class: "policy-budget" },
    el("div", {}, el("span", { text: label }), el("output", { text: `${unit(used)} / ${unit(total)}` })),
    el("span", { class: "policy-budget-bar", "data-level": pct > 80 ? "high" : pct > 40 ? "mid" : "low" }, el("i", { style: `width:${Math.max(pct, 0.6).toFixed(2)}%` })));
}

function renderDeploy() {
  const run = (state.status?.runs || []).find((candidate) => candidate.name === state.selectedRun);
  const exp = run?.export;
  const host = $("#policy-export");
  host.replaceChildren();
  if (!run) {
    host.append(el("p", { class: "policy-empty", text: "Select a run." }));
  } else if (!exp) {
    host.append(el("p", { class: "policy-empty", text: run.checkpoints.length ? "Not exported yet." : "No checkpoint saved yet." }));
  } else {
    host.append(
      el("dl", { class: "policy-facts" },
        el("dt", { text: "Policy" }), el("dd", { text: `${exp.policy_id} · iteration ${exp.iteration}` }),
        el("dt", { text: "Network" }), el("dd", { text: `${exp.layer_dims.join(" → ")} (ELU)` }),
        el("dt", { text: "Parameters" }), el("dd", { text: exp.params.toLocaleString() }),
        el("dt", { text: "Multiply-adds" }), el("dd", { text: `${exp.macs.toLocaleString()} per step` })),
      budgetBar("Flash (weights, const)", exp.flash_bytes, ESP32_FLASH_APP_BYTES, fmtBytes),
      budgetBar("RAM (history + buffers)", exp.ram_bytes, ESP32_RAM_BYTES, fmtBytes),
      budgetBar("Inference time (estimate)", exp.estimated_inference_us, CONTROL_BUDGET_US, (v) => `${Math.round(v)} µs`),
    );
  }
  const checkpoint = $("#policy-checkpoint");
  const previous = checkpoint.value;
  checkpoint.replaceChildren(...(run?.checkpoints || []).slice().reverse().map((name) => el("option", { value: name, text: name })));
  if ([...checkpoint.options].some((option) => option.value === previous)) checkpoint.value = previous;
  const running = ["running", "stopping"].includes(state.status?.job?.status);
  $("#policy-export-button").disabled = !run?.checkpoints.length || state.busy;
  $("#policy-install-button").disabled = !exp || state.busy;
  $("#policy-play-button").disabled = !run?.checkpoints.length || running || state.busy;
  $("#policy-drive-button").disabled = !run?.checkpoints.length || running || state.busy;
  $("#policy-resume-button").disabled = !run?.checkpoints.length || running || state.busy;

  const installed = state.status?.installed;
  $("#policy-installed").replaceChildren(installed?.available
    ? el("span", {}, el("strong", { text: installed.id }), ` from ${installed.run} (it ${installed.iteration}) · ${installed.source === "studio" ? "installed in Studio" : "in firmware source"}`)
    : el("span", { text: "No policy installed. SB-down simulation mode stays unavailable on the robot." }));
  $("#policy-uninstall-button").hidden = !(installed?.available && installed.source === "studio");
}

function renderRobot() {
  const host = $("#policy-robot");
  const fresh = state.liveTelemetry && Date.now() - state.liveAt < 3000;
  const p = fresh ? state.liveTelemetry.policy : null;
  host.replaceChildren();
  if (!p) {
    host.append(el("p", { class: "policy-empty", text: fresh ? "Robot firmware does not report a policy yet (flash this build)." : "Connect to the robot in LIVE to see the on-board policy." }));
    return;
  }
  const installed = state.status?.installed;
  const matches = installed?.available && p.id === installed.id;
  host.append(el("dl", { class: "policy-facts" },
    el("dt", { text: "On robot" }), el("dd", { text: p.installed ? `${p.id} (it ${p.iteration})${matches ? "" : " · differs from installed build"}` : "none" }),
    el("dt", { text: "Self-test" }), el("dd", { text: p.installed ? (p.selfTestOk ? `ok (${Number(p.selfTestError).toExponential(1)})` : "FAILED") : "--" }),
    el("dt", { text: "SB switch" }), el("dd", { text: `${p.sbUs} µs · ${p.requested ? "DOWN (policy requested)" : "not down"}` }),
    el("dt", { text: "State" }), el("dd", { text: p.active ? `ACTIVE · ${p.steps} steps` : p.faultLatched ? `fault: ${p.fault} (release SB)` : "idle" }),
    el("dt", { text: "Inference" }), el("dd", { text: `${p.inferUs} µs (max ${p.inferMaxUs} µs of 20 000)` }),
    el("dt", { text: "Gravity (body)" }), el("dd", { text: `${(p.gravity || []).map((v) => fmt(v, 2)).join(", ")} · tilt ${fmt(p.tiltDeg, 1)}°` }),
  ));
}

function renderJob() {
  const job = state.status?.job;
  const running = ["running", "stopping"].includes(job?.status);
  const pill = $("#policy-lab-state");
  pill.dataset.state = state.status?.error ? "error" : running ? "positive" : state.status?.ready ? "neutral" : "warning";
  pill.textContent = state.status?.error ? "ERROR" : running ? ({ play: "PLAYBACK", drive: "DRIVING" }[job.kind] || "TRAINING") : state.status?.ready ? "READY" : "SETUP NEEDED";
  $("#policy-train-button").disabled = running || !state.status?.ready || state.busy;
  $("#policy-stop-button").disabled = !running;
  const settings = state.status?.settings || {};
  $("#policy-setup").textContent = state.status?.ready
    ? `Isaac Lab: ${settings.labRoot}`
    : `Isaac Lab project ${settings.labRoot ? "found" : "NOT found"} · Python ${settings.pythonFound ? "found" : `NOT found (${settings.python || "unset"})`}`;
  const console = $("#policy-console");
  console.textContent = (state.logLines || []).slice(-80).join("\n") || (job ? `${job.kind} ${job.status}` : "No job started from Studio in this session.");
  console.scrollTop = console.scrollHeight;
}

function render() {
  if (!$("#policy-lab-dialog")?.open) return;
  renderJob();
  renderRuns();
  renderCharts();
  renderDeploy();
  renderRobot();
}

function renderConfig() {
  const host = $("#policy-config");
  host.replaceChildren();
  if (!state.config) {
    host.append(el("p", { class: "policy-empty", text: "Training settings unavailable." }));
    return;
  }
  for (const group of CONFIG_GROUPS) {
    const values = state.config[group.key];
    if (!values) continue;
    const fields = Object.entries(values).map(([key, value]) => {
      const id = `policy-cfg-${group.key}-${key}`;
      const label = key.replaceAll("_", " ");
      if (typeof value === "boolean") {
        return el("label", { class: "policy-field policy-field-check", for: id },
          el("input", { id, type: "checkbox", checked: value, onchange: (event) => { values[key] = event.target.checked; state.configDirty = true; } }),
          el("span", { text: label }));
      }
      const list = Array.isArray(value);
      return el("label", { class: "policy-field", for: id },
        el("span", { text: label }),
        el("input", {
          id,
          type: "text",
          inputmode: "decimal",
          value: list ? value.join(", ") : String(value),
          spellcheck: "false",
          onchange: (event) => {
            const parts = event.target.value.split(",").map((part) => part.trim()).filter(Boolean);
            const parsed = parts.map(Number);
            if (!parsed.every(Number.isFinite) || (list && parsed.length !== value.length) || (!list && parsed.length !== 1)) {
              event.target.setAttribute("aria-invalid", "true");
              return;
            }
            event.target.removeAttribute("aria-invalid");
            values[key] = list ? parsed : parsed[0];
            state.configDirty = true;
          },
        }));
    });
    host.append(el("fieldset", { class: "policy-fieldset" }, el("legend", { text: group.label }), ...fields));
  }
}

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------
function open() {
  const dialog = $("#policy-lab-dialog");
  if (!dialog.open) dialog.showModal();
  feedback("");
  refresh();
  loadConfig();
  clearInterval(state.pollTimer);
  state.pollTimer = setInterval(() => {
    if (dialog.open) refresh();
  }, POLL_MS);
}

function close() {
  clearInterval(state.pollTimer);
  state.pollTimer = null;
  $("#policy-lab-dialog").close();
}

export function initializePolicyLab() {
  const dialog = $("#policy-lab-dialog");
  if (!dialog) return;
  $("#policy-lab-open")?.addEventListener("click", open);
  // policy-lab.ps1 opens Studio at #policy-lab to land straight on this page.
  if (window.location.hash === "#policy-lab") setTimeout(open, 300);
  $("#policy-lab-close").addEventListener("click", close);
  dialog.addEventListener("cancel", (event) => {
    event.preventDefault();
    close();
  });
  window.addEventListener("domino:live-telemetry", (event) => {
    state.liveTelemetry = event.detail;
    state.liveAt = Date.now();
    if (dialog.open) renderRobot();
  });
  window.addEventListener("resize", () => dialog.open && renderCharts());

  $("#policy-train-form").addEventListener("submit", (event) => {
    event.preventDefault();
    const form = new FormData(event.target);
    guarded(() => api("POST", "train", {
      numEnvs: Number(form.get("numEnvs")),
      iterations: Number(form.get("iterations")),
      runName: String(form.get("runName") || "").trim() || undefined,
      visible: form.get("visible") === "on",
    }), "Starting training");
  });
  $("#policy-stop-button").addEventListener("click", () => guarded(() => api("POST", "stop"), "Stopping"));
  $("#policy-resume-button").addEventListener("click", () => {
    const form = new FormData($("#policy-train-form"));
    guarded(() => api("POST", "train", {
      numEnvs: Number(form.get("numEnvs")),
      iterations: Number(form.get("iterations")),
      runName: String(form.get("runName") || "").trim() || undefined,
      visible: form.get("visible") === "on",
      resume: { run: state.selectedRun, checkpoint: $("#policy-checkpoint").value },
    }), "Continuing from checkpoint");
  });
  $("#policy-play-button").addEventListener("click", () => guarded(() => api("POST", "play", {
    run: state.selectedRun, checkpoint: $("#policy-checkpoint").value,
  }), "Opening Isaac Sim playback"));
  $("#policy-drive-button").addEventListener("click", () => guarded(() => api("POST", "play", {
    run: state.selectedRun, checkpoint: $("#policy-checkpoint").value, drive: true,
  }), "Opening Isaac Sim drive mode (Boxer over USB or keyboard)"));
  $("#policy-export-button").addEventListener("click", () => guarded(() => api("POST", "export", {
    run: state.selectedRun, checkpoint: $("#policy-checkpoint").value,
  }), "Exporting for the ESP32"));
  $("#policy-install-button").addEventListener("click", () => guarded(async () => {
    await api("POST", "install", { run: state.selectedRun });
    feedback("Installed. Build and upload firmware (UPLOAD FW) to put it on the robot.", "ok");
  }, "Installing into the firmware build"));
  $("#policy-uninstall-button").addEventListener("click", () => guarded(() => api("POST", "uninstall"), "Removing the installed policy"));
  $("#policy-flash-button").addEventListener("click", () => {
    close();
    document.querySelector("#firmware-deploy-button")?.click();
  });
  $("#policy-config-save").addEventListener("click", () => guarded(async () => {
    await api("PUT", "config", { config: state.config });
    state.configDirty = false;
  }, "Saving training settings"));
  $("#policy-config-reload").addEventListener("click", loadConfig);
  $("#policy-settings-form").addEventListener("submit", (event) => {
    event.preventDefault();
    const form = new FormData(event.target);
    const body = {};
    if (form.get("labRoot")) body.labRoot = String(form.get("labRoot")).trim();
    if (form.get("python")) body.python = String(form.get("python")).trim();
    guarded(() => api("POST", "settings", body), "Saving paths");
  });
}
