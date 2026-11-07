// In-app update banner for the desktop build. The desktop updater checks on
// every launch, downloads a new version in the background, and this banner
// offers a one-click "restart and update".

const RELEASES_URL = "https://github.com/Blacksheep909/Domino-ESP32-Quadruped/releases/tag/v";
const UP_TO_DATE_VISIBLE_MS = 4_000;

// Pure description of what the banner shows. `blockReason` is non-empty when
// restarting now would interrupt the robot (armed, or a jog session running).
export function appUpdateBannerView(state, blockReason = "", dismissed = false) {
  const status = state?.status || "idle";
  const version = String(state?.version || "");
  const current = String(state?.currentVersion || "");
  const hidden = { visible: false };
  if (["idle", "unsupported"].includes(status)) return hidden;
  if (status === "checking") {
    return { visible: !dismissed, tone: "info", text: "Checking for updates…", busy: true };
  }
  if (status === "up-to-date") {
    return {
      visible: !dismissed, tone: "success", autoHideMs: UP_TO_DATE_VISIBLE_MS,
      text: current ? `Studio ${current} is up to date.` : "Studio is up to date.",
    };
  }
  if (status === "downloading") {
    const percent = Math.max(0, Math.min(100, Math.round(Number(state?.percent) || 0)));
    return {
      visible: !dismissed, tone: "info", busy: true, percent,
      text: `Version ${version} is available. Downloading… ${percent}%`,
      notesUrl: version ? `${RELEASES_URL}${version}` : "",
    };
  }
  if (status === "ready") {
    return {
      // A ready update always comes back, even after "Later", on the next launch.
      visible: !dismissed, tone: "ready",
      text: `Version ${version} is ready to install.`,
      detail: blockReason || "Studio closes, updates and reopens by itself. Settings and calibration are kept.",
      action: { id: "install", label: "RESTART & UPDATE", disabled: Boolean(blockReason) },
      dismissLabel: "LATER",
      notesUrl: version ? `${RELEASES_URL}${version}` : "",
    };
  }
  if (status === "manual") {
    // macOS: the app cannot replace itself, so offer the release download.
    const url = version ? `${RELEASES_URL}${version}` : "";
    return {
      visible: !dismissed, tone: "ready",
      text: `Version ${version} is available.`,
      detail: "Download the new .dmg, then drag Domino Quadruped Studio into Applications to replace this version. Settings and calibration are kept.",
      action: { id: "download", label: "DOWNLOAD", disabled: !url },
      dismissLabel: "LATER",
      downloadUrl: url,
    };
  }
  if (status === "error") {
    return {
      visible: !dismissed, tone: "warning",
      text: "Couldn't check for updates.",
      detail: state?.message || "",
      action: { id: "check", label: "TRY AGAIN", disabled: false },
      dismissLabel: "DISMISS",
    };
  }
  return hidden;
}

// The header button for a waiting update. Unlike the banner it can't be
// dismissed, so "LATER" never hides an update until the next launch.
export function appUpdateChipView(state) {
  const status = state?.status || "idle";
  const version = String(state?.version || "");
  if (status === "ready") {
    return { visible: true, label: "UPDATE READY", title: `Version ${version} is downloaded. Click to restart and update.` };
  }
  if (status === "manual") {
    return { visible: true, label: `UPDATE ${version}`.trim(), title: `Version ${version} is available to download.` };
  }
  if (status === "downloading") {
    const percent = Math.max(0, Math.min(100, Math.round(Number(state?.percent) || 0)));
    return { visible: true, label: `UPDATING ${percent}%`, title: `Downloading version ${version}.` };
  }
  return { visible: false };
}

export function initAppUpdateBanner(updates, {
  getBlockReason = () => "",
  root = document.body,
  chip = document.getElementById("app-update-chip"),
} = {}) {
  if (!updates || typeof updates.getState !== "function") return null;
  const banner = document.createElement("div");
  banner.className = "app-update-banner";
  banner.setAttribute("role", "status");
  banner.setAttribute("aria-live", "polite");
  banner.hidden = true;
  banner.innerHTML = `
    <div class="app-update-copy"><strong></strong><span></span></div>
    <div class="app-update-progress" hidden><i></i></div>
    <div class="app-update-actions">
      <a class="app-update-notes" target="_blank" rel="noopener" hidden>WHAT'S NEW</a>
      <button type="button" class="app-update-dismiss" hidden></button>
      <button type="button" class="app-update-action" hidden></button>
    </div>`;
  root.append(banner);
  const title = banner.querySelector(".app-update-copy strong");
  const detail = banner.querySelector(".app-update-copy span");
  const progress = banner.querySelector(".app-update-progress");
  const bar = progress.querySelector("i");
  const notes = banner.querySelector(".app-update-notes");
  const dismissButton = banner.querySelector(".app-update-dismiss");
  const actionButton = banner.querySelector(".app-update-action");

  let state = { status: "idle" };
  let dismissedStatus = "";
  let hideTimer = null;
  let installing = false;

  function render() {
    if (chip) {
      const chipView = appUpdateChipView(state);
      chip.hidden = !chipView.visible;
      if (chipView.visible) {
        chip.textContent = chipView.label;
        chip.title = chipView.title;
      }
    }
    const view = appUpdateBannerView(state, getBlockReason() || "", dismissedStatus === state.status);
    banner.hidden = !view.visible;
    if (!view.visible) return;
    banner.dataset.tone = view.tone;
    title.textContent = installing ? "Restarting to update…" : view.text;
    detail.textContent = installing ? "" : view.detail || "";
    detail.hidden = !detail.textContent;
    banner.setAttribute("aria-busy", String(Boolean(view.busy || installing)));
    progress.hidden = !Number.isFinite(view.percent);
    if (!progress.hidden) bar.style.width = `${view.percent}%`;
    notes.hidden = !view.notesUrl;
    if (view.notesUrl) notes.href = view.notesUrl;
    dismissButton.hidden = !view.dismissLabel || installing;
    dismissButton.textContent = view.dismissLabel || "";
    actionButton.hidden = !view.action;
    if (view.action) {
      actionButton.textContent = installing ? "RESTARTING…" : view.action.label;
      actionButton.disabled = view.action.disabled || installing;
      actionButton.dataset.action = view.action.id;
      actionButton.title = view.action.disabled ? view.detail : "";
    }
    // render() also runs on robot-state changes; schedule the auto-hide once.
    if (view.autoHideMs && !hideTimer) {
      hideTimer = setTimeout(() => {
        hideTimer = null;
        dismissedStatus = state.status;
        render();
      }, view.autoHideMs);
    }
  }

  function accept(next) {
    if (!next || typeof next !== "object") return;
    if (next.status !== state.status) {
      dismissedStatus = "";
      clearTimeout(hideTimer);
      hideTimer = null;
    }
    state = next;
    render();
  }

  actionButton.addEventListener("click", async () => {
    if (actionButton.dataset.action === "install") {
      if (getBlockReason()) return render();
      installing = true;
      render();
      const started = await updates.install().catch(() => false);
      if (!started) {
        installing = false;
        render();
      }
    } else if (actionButton.dataset.action === "download") {
      const url = appUpdateBannerView(state).downloadUrl;
      // Studio opens external links in the default browser.
      if (url) window.open(url, "_blank", "noopener");
    } else if (actionButton.dataset.action === "check") {
      await updates.check().catch(() => false);
    }
  });
  chip?.addEventListener("click", () => {
    // Bring the banner back with its RESTART & UPDATE / DOWNLOAD button.
    dismissedStatus = "";
    render();
    actionButton.focus?.();
  });
  dismissButton.addEventListener("click", () => {
    dismissedStatus = state.status;
    render();
  });

  updates.onState?.(accept);
  const refresh = () => updates.getState().then(accept).catch(() => {});
  refresh();
  // Re-read on focus too, in case a state message was missed.
  window.addEventListener("focus", refresh);
  return { render };
}
